# `host/` — things the container cannot do for itself

These scripts run on the Jetson, outside the container: the UART DMA/IOMMU
device-tree patch, the OAK-D udev rule, the docker nvidia runtime, the power
model, and starting the container itself.

They replace the old numbered scripts at the repo root
(`01_prereqs.sh` … `04_run_container.sh`, `run_pipeline.sh`,
`setup_and_build.sh`) — that logic now lives in `provision.sh` and `run.sh`.

Nothing here needs `sudo` to *inspect* the machine. On this fleet `sudo`
prompts for a password and `dmesg` is restricted, so every check reads
`/proc/device-tree` and `/sys` instead. Only `provision.sh` actually mutates
the host, and only in the phases that genuinely require root.

| Script | What it does |
|---|---|
| `provision.sh` | One idempotent, resumable entry point: apt deps → UART patch → power → image → doctor |
| `doctor.sh` | Unprivileged health check. Non-zero exit on any FAIL |
| `run.sh` | Launch the container: `stack`, `shell`, `bag`, `sim` |
| `jetson_uart_dma_fix.sh` | The R36.5 UARTA DMA/IOMMU device-tree patch |
| `build_image.sh` / `pull_image.sh` / `push_image.sh` | Image build and registry transport |
| `uart_loopback_test.py` | Bench-test the Jetson TX/RX path with a jumper across pins 8 and 10 |
| `tests/test_extlinux_fix.sh` | Unit test for the extlinux FDT rewrite |

## First-time setup

```bash
host/provision.sh --dry-run     # read it before you run it
host/provision.sh
# ...if the UART patch was staged, it stops and asks for a reboot:
sudo reboot
host/provision.sh               # resumes; sees the patch is live; continues
host/doctor.sh
```

Progress is recorded one phase per line in `$HOME/.skybeta/state` (under
`$HOME` on purpose — recording progress must never itself need root).

```bash
host/provision.sh --list             # which phases are done
host/provision.sh --from power       # re-run this phase and everything after
host/provision.sh --skip pull        # repeatable
```

Phases, in order:

| Phase | Root? | Does |
|---|---|---|
| `prereqs` | yes | apt deps, `03e7` udev rule, registers the docker `nvidia` runtime (does **not** touch `default-runtime` — every `docker run` here passes `--runtime nvidia` explicitly), `dialout` + `docker` group membership |
| `uart_patch` | yes | `jetson_uart_dma_fix.sh apply`. Needs one reboot the first time; prints instructions and exits 0, and the next run detects the live patch and carries on |
| `power` | yes | `nvpmodel` to the MAXN model (**resolved by name** — on the Orin Nano Super that is ID 2, not 0) + `jetson_clocks`, made to survive reboot by a `skybeta-jetson-clocks.service` oneshot |
| `pull` | no | `pull_image.sh <tag>` using the tag in the repo-root `VERSION`. Needs `REGISTRY` set; skipped with a warning otherwise |
| `doctor` | no | `doctor.sh` |

## Health check

```bash
host/doctor.sh                  # every check except the camera boot
host/doctor.sh --boot-camera    # also opens the OAK-D and reads getUsbSpeed()
host/doctor.sh --no-uart        # skip the 3 s serial sniff
```

Ten checks: L4T release · UARTA `iommus`/`dmas` in the live device tree ·
`3100000.serial` bound to `serial-tegra` owning `ttyTHS1` · exactly one `FDT`
line in `extlinux.conf` · `dialout` + `docker` membership · docker `nvidia`
runtime · OAK-D enumerated · UART traffic · image present · power model.

`PASS`/`WARN`/`FAIL`, one line each; exit 1 if anything FAILed. Two results
are worth knowing in advance on the dev unit:

- **`uart-traffic` FAILs.** The TELEM2 JST-GH connector on that airframe is
  physically damaged. A negative UART result there says nothing about
  configuration — replace the cable first (`CLAUDE.md`, Hardware status).
- **`oakd-usb` at `03e7:2485`, 480 Mbps, is normal.** The MyriadX enumerates
  twice; it re-appears as `03e7:f63b` at 5000 Mbps once depthai pushes
  firmware. Use `--boot-camera` if you actually need the link speed.

## Running the stack

```bash
host/run.sh stack                          # camera at the body origin
host/run.sh stack 0.10 0 -0.02             # x y z, metres, FLU
host/run.sh stack 0.10 0 -0.02 0 0.35 0    # ...plus roll pitch yaw, radians
host/run.sh shell                          # interactive bash, or exec into a running container
host/run.sh bag                            # rosbag2 into ./bags (needs WP6's record_bag.sh)
host/run.sh sim                            # stub — WP7
```

The camera pose becomes the `base_link → oak_parent_frame` static TF and is
passed in as `CAMERA_X/Y/Z` and `CAMERA_ROLL/PITCH/YAW`. Keep `EKF2_EV_POS_*`
at zero on the flight controller: the TF already carries the lever arm, and
setting both applies it twice (`CLAUDE.md` bug #9).

Image reference: `skybeta:$(cat VERSION)`, falling back to `skybeta:latest`.
Override the whole thing with `IMAGE=`. Other knobs: `NAME=`, `DEV=`
(default `/dev/ttyTHS1`), `XRCE_BAUD=`, `BAGDIR=`.

`stack` and `shell` refuse to start when another running container already has
`/dev/ttyTHS1` passed through — the check inspects every running container's
device list, not just the names this script uses. Two MicroXRCEAgents on one
serial port corrupt each other's framing.

There is no `sudo` in `run.sh`. The old `sudo systemctl stop nvgetty` and
`sudo chmod 666 $DEV` lines were no-ops here: `nvgetty.service` does not exist
on this image, and the user is in `dialout`.

## UART device-tree patch

```bash
host/jetson_uart_dma_fix.sh apply --dry-run   # no root: shows the extlinux diff
sudo host/jetson_uart_dma_fix.sh apply        # then reboot
sudo host/jetson_uart_dma_fix.sh status
sudo host/jetson_uart_dma_fix.sh restore
```

`apply` is a genuine no-op when the patch is already live — it checks for
`/proc/device-tree/bus@0/serial@3100000/iommus`, `/boot/dtb/uart_dma_fix.dtb`,
and a matching `FDT` line, all readable without root, and exits 0 before it
ever reaches `need_root`. `--force` rewrites anyway.

Two bugs fixed here:

- **#1** — every branch of `main()` now ends in an explicit `exit 0`. It used
  to end on a bare `[[ $do_reboot -eq 1 ]] && { … }` list, which returns 1 when
  `--reboot` is absent; as the script's last command under `set -Eeuo pipefail`
  that made a *successful* apply exit 1 and abort every caller.
- **#2** — the extlinux rewrite now deletes **every** `FDT` line inside
  `LABEL primary` and inserts exactly one immediately after `LINUX`. The old
  version inserted before it reached the pre-existing line, so each run added a
  duplicate; the dev unit had accumulated four.

## Tests

```bash
bash host/tests/test_extlinux_fix.sh
```

Copies the live `/boot/extlinux/extlinux.conf` (world-readable) to a temp dir,
runs the rewrite twice, and asserts: exactly one `FDT` line, placed on the line
immediately after `LINUX`, every other line byte-identical, and the second run
a no-op. Falls back to a synthetic four-`FDT` fixture where `/boot` is absent.
No root, no `/boot` writes, no reboot.

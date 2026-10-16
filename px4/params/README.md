# PX4 parameter sets

Staged QGroundControl `.params` files for the skybeta stack. Values verified against the
**PX4 v1.16** parameter reference — the firmware this airframe actually runs (see `EKF2_RNG_FOG`
in the May dump; it does not exist in v1.14 or v1.15).

## Back up first

```
QGC -> Vehicle Setup -> Parameters -> Tools -> Save to file
```

Do this before loading anything. These are **delta files** — they contain only the parameters
being changed, so everything else on the vehicle is left alone. Your calibration,
`SYS_AUTOSTART`, and the measured `EKF2_RNG_POS_Y = -0.027432` are not touched. A backup means
you can get back regardless.

## Load them in order, one at a time, rebooting between

| File | Purpose | Load when | Needs the UART? |
|---|---|---|---|
| `03a_height_reference.params` | height: range finder reference, baro backup (PX4 "Indoor (non-flat terrain)") | **now** — fixes the runaway climb | no |
| `01_telem2_mavlink.params` | TELEM2 → MAVLink | first — proves the UART wiring | — |
| `02_telem2_uxrce_dds.params` | TELEM2 → uXRCE-DDS | after stage 1 gives a HEARTBEAT | yes |
| `03b_ekf2_vision.params` | EV fusion | after stage 2 shows data **and** the `pose_frame` bug is fixed | yes |

**`03a` is deliberately separable.** The height-reference fix has nothing to do with vision, the
UART or the container — it addresses the upward drift on its own and can go in today. Everything
else waits on a working link. Do not load `03b` early: `EKF2_EV_CTRL = 9` is inert while no
vision data arrives, but it means you would later bring up the link and enable vision fusion in
the same step, against a bridge that still declares `POSE_FRAME_NED` — which is the toilet-bowl
bug. Fix the bridge first, then load `03b`.

`QGC -> Parameters -> Tools -> Load from file`, then **reboot the FC** — every parameter in
these files is reboot-gated.

### After stage 1

```bash
mavproxy.py --master=/dev/ttyTHS1,57600 --show-errors
```

A HEARTBEAT within a second or two means the wire is good. Nothing means the fault is physical —
see the loopback test (`uart_loopback_test.py`) and the wiring table in `CLAUDE.md`. **Do not
move on to stage 2 until this works**; DDS on a dead wire looks identical to DDS misconfigured.

### After stage 2

```bash
run_agent.sh                       # in the container
ros2 topic list | grep /fmu/       # should list many topics
check_topics.sh                    # echoes /fmu/out/timesync_status
```

### After stage 3

Bench first, props off. Confirm `EKF2_EV_CTRL` is actually fusing before flying — watch
`/fmu/out/estimator_status_flags` and check the vehicle's local position tracks the camera when
you move the airframe by hand.

## Recorded dumps: `fleet/`

One file per airframe, as it is actually running (fleet guide step 8), diffed with
`tools/param_diff.py --dump fleet/<serial>.params`.

- **`fleet/SN0001-reference-2026-05-07.parm`** — MAVProxy dump of the reference airframe,
  1129 parameters, captured 2026-05-07, before any staged file was loaded. `00_baseline.params`
  is generated from it (`tools/param_diff.py --make-baseline`), and `--selftest` uses it to
  prove the baseline round-trips. It predates `UXRCE_DDS_PTCFG` (absent) and was taken with
  TELEM2 still on MAVLink. Do not overwrite it; record new dumps alongside it.
- **`fleet/SN0001-2026-10-15.params`** — QGC dump of the same airframe as it actually ran on
  2026-10-15: **firmware v1.16.1** (git `94cb2012` = tag `v1.16.1`), not the v1.16.2 the fleet
  pins. `param_diff --dump` reports 28 changed parameters; the ones that matter:
  `UXRCE_DDS_DOM_ID=0` and `UXRCE_DDS_PTCFG=0` (no `/fmu` data reaches the stack),
  `EKF2_HGT_REF=0` with `EKF2_BARO_CTRL=0` and `EKF2_GPS_CTRL=0` (no height aiding once the
  range finder drops out), `EKF2_EV_CTRL=9` with the placeholder `EKF2_EV_DELAY=50` and EV
  gates widened to 50, `NAV_RCL_ACT=6` (Disarm on RC loss), and motor functions on both
  `PWM_MAIN_FUNC1-4` and `PWM_AUX_FUNC1-4`.
- **`fleet/SN0001-fixes-2026-10-15.params`** — the delta that airframe should load (QGC "Load
  from file", then reboot): the DDS pair for drone 1, PX4's "Indoor (flat terrain)" height
  configuration ("Indoor (non-flat terrain)": range finder as the reference, conditional, baro
  kept as backup), vision fusion kept on with the measured `EKF2_EV_DELAY=70` and the EV gates
  back at 5 / 3, and a 3 m ceiling inside the VL53L1X's 4 m range. Its
  header explains each value and lists what it deliberately leaves for a human decision.

## Deliberately not included

- **`EKF2_OF_CTRL` / `EKF2_RNG_CTRL`.** Retiring optical flow and the rangefinder is a separate
  decision. Changing them in the same sitting as enabling vision makes any result
  un-attributable.
- **Vertical position and velocity EV fusion.** `EKF2_EV_CTRL = 9` is horizontal position + yaw
  only. Add bit 1 (→ 11) for vertical, bit 2 (→ 13) for velocity, once vision is trusted —
  vertical is the weakest axis for pure stereo VO with `enable_imu_fusion` off.

## Values that are placeholders, not answers

- **`EKF2_EV_DELAY = 70`** — measured on the bench Orin, not yet on a flying airframe
  (2026-10-16, `tools/latency_probe.py`: image → bridge publish p50 68.7 ms, p95 102 ms, plus ~1–2 ms
  for the serial hop). It replaced a guessed 50. Re-run the probe on each airframe and cross-check
  with an EKF2 replay log (`SDLOG_PROFILE` bit 7); a wrong delay is a prime toilet-bowl cause.
- **`UXRCE_DDS_DOM_ID`** — no longer in `02_telem2_uxrce_dds.params`: it is **per airframe**.
  Set it by hand to that drone's `ROS_DOMAIN_ID` from `/etc/skybeta/drone.env` (convention:
  `= DRONE_ID`). On a shared network, every drone left on 0 shares one DDS domain.
  `param_diff` treats it like `MAV_SYS_ID`: excluded from the fleet comparison.

# PX4 firmware — pinned for the fleet

One file, vendored on purpose. Ten drones running three different PX4 builds is
the kind of fleet problem that only shows up at the venue.

| | |
|---|---|
| File | `px4_fmu-v6c_default-v1.16.2.px4` |
| Board | Pixhawk 6C (`board_id` 56, `PX4FMUv6C`) |
| PX4 version | **v1.16.2**, `git_identity` `v1.16.2` |
| Released | 2026-04-22 |
| Size | 1 845 192 bytes (1.8 MB) |
| SHA-256 | `f23a3ed4334ff14c9bcc927ce378a55a62fb6d871cb1ef05813c080e04c5776f` |
| Source | <https://github.com/PX4/PX4-Autopilot/releases/download/v1.16.2/px4_fmu-v6c_default.px4> |

The upstream filename is `px4_fmu-v6c_default.px4`; the version is appended here
so a directory listing tells you what you are holding.

## Verify before flashing

```bash
sha256sum -c px4/firmware/px4_fmu-v6c_default-v1.16.2.px4.sha256
```

The published SHA-256 is not a claim of ours — GitHub reports the same digest for
the release asset (`GET /repos/PX4/PX4-Autopilot/releases/tags/v1.16.2`, the
asset's `digest` field), and the two were compared when this file was vendored.

You can also read the identity straight out of the bundle without flashing it. A
`.px4` file is JSON:

```bash
python3 -c "
import json; d=json.load(open('px4/firmware/px4_fmu-v6c_default-v1.16.2.px4'))
print(d['summary'], d['git_identity'], 'board_id', d['board_id'])"
# PX4FMUv6C v1.16.2 board_id 56
```

## Why v1.16.2 and not v1.14

The repo used to claim v1.14 throughout. It was wrong. The reference airframe's
own parameter dump contains `EKF2_RNG_FOG`, which does not exist in the v1.14 or
v1.15 parameter reference and first appears in v1.16. Everything downstream was
re-pinned to match: `px4_msgs` and `px4-ros2-interface-lib` are both
`release/1.16` in `docker/Dockerfile.jetson`.

v1.16.2 is the latest 1.16.x stable at the time of writing, and it is the version
`sim/` targets for SITL, so the simulator and the vehicle run the same firmware
generation. A sim on different firmware from the vehicle is a sim that lies about
parameters.

## Flashing

QGroundControl → Vehicle Setup → Firmware → *Advanced settings* → **Custom
firmware file** → select this file.

**Flashing erases every parameter.** That is the whole reason
`px4/params/00_baseline.params` exists. Flash, then load the baseline, then
calibrate. The ordered procedure is [`docs/fleet_guide.md`](../../docs/fleet_guide.md).

## The parameter metadata is in here too

The bundle carries the full v1.16.2 parameter reference as a zlib+base64
`parameter_xml` blob — 2065 definitions with their types. `tools/param_diff.py`
reads it, which is why the generated baseline's `INT32`/`REAL32` column is what
the firmware declares rather than something inferred from decimal points.

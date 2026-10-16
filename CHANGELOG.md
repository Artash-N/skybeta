# Changelog

One entry per image version (`VERSION`, tag `skybeta:<VERSION>`). The evidence behind each
change is in [`CLAUDE.md`](CLAUDE.md); this file says what changed and what you must do.

## 0.2.0 — fleet-ready (2026-10-12, branch `fleet-ready`)

Makes the stack safe to run on many drones sharing one WiFi network, with teams writing code
on the drone itself.

### You must do this when moving a drone from 0.1.0

1. **Write `/etc/skybeta/drone.env`** from `host/drone.env.example`: a unique `DRONE_ID` and
   `ROS_DOMAIN_ID` (convention: the drone number), the camera mount pose, the team.
2. **On the flight controller**: load `px4/params/02_telem2_uxrce_dds.params` again (it now
   sets **`UXRCE_DDS_PTCFG = 2`**), set **`UXRCE_DDS_DOM_ID`** to the drone's `ROS_DOMAIN_ID`
   by hand, and reboot the FC. With `PTCFG=0` the stack connects but **no `/fmu` data
   arrives** in the default local DDS scope.
3. **Rebuild or pull the image**: `skybeta:0.2.0`. The container scripts changed.
4. **Move team code** out of any container into `~/team_ws/src`.

### Changed

- **DDS isolation.** Each drone has its own DDS domain, and by default
  (`SKYBETA_DDS_SCOPE=local`) its DDS never leaves the Jetson: the ROS nodes use
  `host/dds/fastdds_localhost.xml`, the agent builds PX4's participant from
  `host/dds/agent_refs_local.xml` (`MicroXRCEAgent -r`, FC `UXRCE_DDS_PTCFG=2`).
  `SKYBETA_DDS_SCOPE=network` opens a drone up for laptop debugging. Bench: 0 DDS packets on
  WiFi with the full stack running; cuVSLAM still at 30 Hz.
- **Per-drone identity** in `/etc/skybeta/drone.env`, read by `host/run.sh` and
  `host/doctor.sh`. Parsed, never sourced.
- **Team workspace.** `$TEAM_WS` (default `~/team_ws`) is mounted at `/root/team_ws` in every
  container and survives restarts and image updates. `team_build.sh` builds it as an overlay;
  the stack's `competitor` window builds and launches it. Extra apt/pip packages:
  `host/run.sh build --init`, then `host/run.sh build` (image `skybeta-team:<VERSION>`).
- **The flight pipeline never loads team code**: agent, cuVSLAM and the bridge source the
  skybeta underlay only, so a team `px4_msgs` cannot break the state estimate.
- `host/run.sh shell` attaches to the running stack instead of refusing.
- `COMPETITOR_PKG` / `COMPETITOR_LAUNCH` now actually reach the container (they never did).
- `host/doctor.sh`: two new checks, `drone-identity` and `team-ws` (twelve in total).
- `host/doctor.sh --json`: one JSON object instead of the table (schema 1: `checks[]` of
  `{name, result, detail}` plus `pass`/`warn`/`fail`/`ok`), same checks and exit status.
  The fleet status agent reads it. Tested by `host/tests/test_doctor_json.sh`.
- `UXRCE_DDS_DOM_ID` is per-airframe: removed from the 02 file, excluded by `param_diff`.
- pip packages pinned to what 0.1.0 shipped (`depthai==3.10.0`, `MAVProxy==1.8.74`,
  `pymavlink==2.4.49`, `numpy<2`); an unpinned rebuild pulled numpy 2 and opencv 5.
- The reference airframe's parameter dump is committed
  (`px4/params/fleet/SN0001-reference-2026-05-07.parm`), so `param_diff --selftest` runs all
  six steps anywhere.
- `.gitattributes` forces LF, so scripts from a Windows checkout run on a Jetson.

### Not yet verified

- `/fmu` data through the 0.2.0 stack on a real FC with `UXRCE_DDS_PTCFG=2` (the bench
  TELEM2 connector is damaged and dropped the link first).
- Two drones on one access point.

## 0.1.0 (2026-09-03, branch `competition-base`)

PX4 v1.16.2 competition base: pinned firmware and staged parameters, fleet provisioning
(`host/provision.sh`, `doctor.sh`, `run.sh`), the competitor API (`skybeta_flight_interface`,
examples), the sim profile. See the commit message of `00567ef` and `CLAUDE.md`.

# skybeta — repo guide

Containerized Jetson stack pairing an **OAK-D-S2** (cuVSLAM visual odometry) with a
**Pixhawk 6C** (PX4 **v1.16.2**) over uXRCE-DDS on `/dev/ttyTHS1` @ 921600.

Read [`jetson_pixhawk_uxrce_ros2_setup.md`](jetson_pixhawk_uxrce_ros2_setup.md) for the
canonical hardware/PX4 procedure; [`README.md`](README.md) for the container workflow.

## Layout at a glance

- **`host/`** does what a container can't: `provision.sh` (idempotent, resumable: prereqs →
  UART DMA patch → power mode → image pull), `doctor.sh` (unprivileged health check),
  `run.sh stack|shell|bag|build|sim`, `build_image.sh` / `pull_image.sh` / `push_image.sh`,
  `jetson_uart_dma_fix.sh`, `uart_loopback_test.py`. The old numbered scripts are gone.
  Per-drone identity: `/etc/skybeta/drone.env` (template `drone.env.example`, parsed by
  `drone_env.sh`). DDS isolation: `dds/fastdds_localhost.xml` (ROS nodes) and
  `dds/agent_refs_{local,network}.xml` (the agent's PX4 participant, `UXRCE_DDS_PTCFG=2`).
- **`docker/Dockerfile.jetson`** builds `skybeta:<VERSION>` on the NGC Isaac ROS Humble base
  (content-hash pinned, aarch64). `docker/Dockerfile.sim` is the x86 sim twin.
- **`container_scripts/*.sh`** land on `$PATH` inside the image (`run_agent.sh`, `run_slam.sh`,
  `run_bridge.sh`, `run_full_stack.sh`, `run_competitor.sh`, `check_topics.sh`, `record_bag.sh`,
  `team_build.sh`, and `skybeta_env.sh`, which is sourced, not run).
- **Team code lives outside the image**, in `$TEAM_WS` (default `~/team_ws`) on the host,
  bind-mounted at `/root/team_ws` and built as an overlay on `/root/skybeta_ws`. The VIO -> FC
  pipeline (agent, cuVSLAM, bridge) never sources that overlay.
- **`src/`** — `oakd_vslam_bringup` (cuVSLAM launch + the PX4 odometry bridge),
  `skybeta_flight_interface` (the competitor-facing offboard wrapper: 10 Hz heartbeat, ENU↔NED,
  geofence, **no arm path**), `competitor_example` (Python reference mission),
  `competitor_advanced_cpp` (px4_ros2_cpp custom mode). Competitors add their own packages
  in the team workspace, not here.
- **`px4/`** — pinned v1.16.2 firmware + staged parameter files; **`tools/`** — `param_diff.py`,
  `latency_probe.py`; **`sim/`** — Isaac Sim + Pegasus profile; **`docs/`** — competitor, fleet,
  hardware and debugging guides.

Data path: `OAK-D → depthai driver (bgr8) → isaac_ros_image_proc ImageFormatConverterNode
(GPU, mono8) → cuVSLAM → /visual_slam/tracking/odometry → px4_odom_bridge (ENU→NED, FLU→FRD)
→ /fmu/in/vehicle_visual_odometry → MicroXRCEAgent → UART → PX4 EKF2`. A competitor's node
consumes the odometry and publishes to `/skybeta/setpoint/*`.

## Verified facts about this Jetson

Measured 2026-09-02 on the dev unit, not assumed:

- **Jetson Orin Nano Engineering Reference Developer Kit Super**, L4T **R36.5.0**,
  Ubuntu 22.04.5, kernel 5.15.185-tegra.
- **UART DMA/IOMMU patch is live.** `/proc/device-tree/bus@0/serial@3100000/` carries
  `iommus`, `dmas`, `dma-names`; the platform device is bound into
  `/sys/kernel/iommu_groups/1` via `smmu.0x0000000012000000`, driver `serial-tegra`,
  compatible `nvidia,tegra194-hsuart`, status `okay`.
- **UART TX is healthy.** 4096 bytes written at 921600 8N1 completed in 55.8 ms against a
  44.4 ms theoretical floor — the DMA path is not stalling.
- `dmesg` is restricted (`kernel.dmesg_restrict=1`) and **sudo requires a password**, so
  `02b_uart_fix_verify.sh` / `jetson_uart_dma_fix.sh status` can't be run unattended. Use
  the `/proc/device-tree` + `/sys/bus/platform/devices/3100000.serial/iommu_group` checks
  above instead — they need no privileges.
- User is in `dialout` **and** `docker`, so the UART and docker need no sudo. The
  `sudo chmod 666 $DEV` / `sudo systemctl stop nvgetty` lines in `04_run_container.sh` and
  `run_pipeline.sh` are already no-ops here (`nvgetty.service` does not exist on this image).
- `/etc/docker/daemon.json` registers the `nvidia` runtime but does **not** set
  `default-runtime: nvidia` (contrary to what `01_prereqs.sh` would install). Harmless —
  every `docker run` in this repo passes `--runtime nvidia` explicitly, and the build needs
  no GPU.

## Hardware status

- **OAK-D-S2 is on USB 3.0 SuperSpeed — do not be fooled by `lsusb` before it boots.**
  The MyriadX enumerates twice. Unbooted it appears as `03e7:2485` at **480 Mbps** on bus 1;
  once depthai loads firmware it re-enumerates as `03e7:f63b` at **5000 Mbps** on `2-1.1`
  (bus 2, the USB3 hub). Verified: `dai.Device().getUsbSpeed()` → `UsbSpeed.SUPER`. Any
  bandwidth or latency reasoning based on the pre-boot `lsusb` reading is wrong.
- **Camera identity, read off the device** (`getConnectedCameraFeatures()`):
  `CAM_A = IMX378` 4056×3040 COLOR; `CAM_B`/`CAM_C` = **`OV9282` 1280×800**, supported types
  `[MONO, COLOR]`. IMU is **BNO086** fw 3.9.9 (the 9-axis part, not the BMI270 fallback).
- **Pixhawk link is intermittent — the TELEM2 JST-GH connector is physically damaged and
  does not seat fully.** Confirmed by the user 2026-09-02. This is the root cause of every
  UART symptom in this repo's history; do not go looking for a software explanation.

  Observed sequence in one session: silent for hours → one clean run in which the agent
  established a session and built the **entire** bridge (51 topics, 24 datawriters,
  27 datareaders, `DataWriter::write` carrying real payloads, including
  `/fmu/in/vehicle_visual_odometry`) → silent again → 0 bytes across a continuous 60 s watch.
  That one good run is proof the wiring, 921600 baud, `UXRCE_DDS_CFG=102` and the agent are
  all correct. Everything else was the connector losing contact.

  **Same pattern again 2026-10-12** (cable still not replaced): 0 bytes at every baud rate ->
  after a reseat, uXRCE framing (`7e 01 00 ...`) at 921600 and a full session with the 0.2.0
  stack (24 datawriters, 27 datareaders, a replier) -> 0 bytes again after the stack was
  restarted, PX4 never reconnected. That session also showed the FC is on
  `UXRCE_DDS_DOM_ID=0`, `UXRCE_DDS_PTCFG=0` (agent participant multicasting on WiFi to
  `239.255.0.1:7400`).

  **Consequences for debugging:** any *negative* UART result recorded while this connector is
  in place is uninformative — it tells you nothing about configuration. Positive results are
  still valid. Replace the JST-GH 1.25 mm 6-pin cable before trusting any further UART
  measurement, and re-run anything that previously "failed".

  **Consequences for flight:** an intermittent EV datalink is a plausible contributor to the
  reported toilet-bowl and altitude-drift symptoms in its own right. If the link drops mid-
  flight while `EKF2_EV_CTRL` is fusing vision, EKF2 loses position aiding and the estimator
  degrades — which looks exactly like the drift being chased. **Fix the connector before
  attributing those symptoms to `pose_frame`, `EKF2_EV_DELAY` or anything else in this
  document.** Do not bodge it with tape or glue on a flight vehicle.

Quick re-test (no sudo needed, user is in `dialout`):

```bash
python3 -c "
import serial,time
s=serial.Serial('/dev/ttyTHS1',921600,timeout=0.2); s.reset_input_buffer()
b=bytearray(); t=time.time()
while time.time()-t<4: b+=s.read(4096)
print(len(b),'bytes'); print(bytes(b[:64]).hex(' '))"
```

## Flight controller — what the saved logs actually show

`~/Downloads/mav.parm` (1129 params) and `~/Downloads/mav.tlog` (1.38 MB) are a MAVProxy
capture from **2026-05-07 22:25:49–22:26:44** (55 s, 19918 messages: `ATTITUDE`,
`HIGHRES_IMU`, `LOCAL_POSITION_NED`, `ODOMETRY`, `TIMESYNC`, plus a param download). That
capture is the proof the UART link *has* worked. Read it before re-debugging the wiring.

Two things it establishes that contradict the repo's premises:

**The FC runs PX4 v1.16+, not v1.14.** The dump contains `EKF2_RNG_FOG`, which is absent
from the v1.14 *and* v1.15 parameter references and first appears in v1.16 (checked against
`docs.px4.io/{v1.14,v1.15,v1.16}/en/advanced_config/parameter_reference.html`). See bug #8.

**The DDS path was never enabled on the airframe.** As of that capture:

| Param | Value | What the setup doc wants |
|---|---|---|
| `UXRCE_DDS_CFG` | **0** (disabled) | `TELEM2` |
| `MAV_1_CONFIG` | **102** (= TELEM2, MAVLink) | `0` |
| `MAV_1_MODE` | 2 (Onboard) | — |
| `SER_TEL2_BAUD` | 921600 | 921600 ✓ |
| `EKF2_EV_CTRL` | **0** (vision fusion off) | bits 0+3 = 9 |
| `EKF2_EV_DELAY` | **0** | 50 ms |
| `EKF2_HGT_REF` | **2** (Range) | Baro |
| `EKF2_BARO_CTRL` | **0** (baro off) | — |
| `EKF2_EVP/EVV/EVA_NOISE` | 0.1 / 0.1 / 0.1 | ✓ (already set) |

So TELEM2 was carrying **MAVLink**, not uXRCE-DDS — i.e. the working state on disk is §3 of
the setup doc (the MAVProxy sanity test), not §7 (the DDS bridge). The estimator was also
still on the *old* sensor set the container is meant to replace: `EKF2_OF_CTRL=1` (optical
flow on), `EKF2_RNG_CTRL=2` with a measured `EKF2_RNG_POS_Y=-0.027432`, `EKF2_GPS_CTRL=7`.

Caveat: the dump may be truncated (1129 lines vs 1174 `PARAM_VALUE` messages in the tlog), so
treat a *missing* param as unknown; a *present* one is reliable. And these are May-7 values —
re-dump before trusting them.

## DepthAI grounding (checked against Luxonis docs, v3 driver)

**This container runs DepthAI ROS v3, not v2** — worth stating plainly because it is easy to
assume otherwise. Both lines are published to `packages.ros.org` and the base image can see
both:

```
ros-humble-depthai-ros-driver      Candidate: 2.12.2-1jammy   <- v2 line
ros-humble-depthai-ros-driver-v3   Candidate: 3.3.0-1jammy    <- v3 line, what we install
```

The Dockerfile installs the **`-v3`** packages, `oakd_vslam.launch.py` resolves
`get_package_share_directory('depthai_ros_driver_v3')`, and it includes `driver.launch.py`
(v2's entry point is `camera.launch.py`, deprecated in v3). The `driver:` / `pipeline_gen:`
namespaces in `config/oakd_vslam.yaml` are also v3 spelling — v2 nests the same settings under
`camera:`. The three are consistent, so the repo is coherently v3 throughout.

If a previous working setup was on v2, its YAML is **not** portable here: `camera:` →
`driver:`/`pipeline_gen:`, and `i_mx_id` → `i_device_id`.

Documented defaults that matter here:

| Parameter | Upstream default | Repo sets | Note |
|---|---|---|---|
| `left/right.i_width` × `i_height` | **640 × 400** | 1280 × 800 | **silently discarded — see #3** |
| `left/right.i_fps` | 30.0 | 30.0 | same as default |
| `left/right.i_enable_lazy_publisher` | `true` | `false` | correct: cuVSLAM subscribes late |
| `stereo.i_publish_left_rect` / `_right_rect` | **`false`** | `true` | unused by the launch — see #5 |
| `stereo.i_publish_synced_rect_pair` | *not in v3 docs* | `true` | likely a dead v2-era key |
| `stereo.i_depth_preset` | `FAST_ACCURACY` | `DEFAULT` | `DEFAULT` is a valid enum |
| `stereo.i_reverse_stereo_socket_order` | `false` | `true` | flips which camera is "first" in `CameraInfo` — verify the sign of `P[3]` on `/oak/right/camera_info` |
| `pipeline_gen.i_nn_type` | `spatial` | `none` | correct — avoids loading a blob |
| `pipeline_gen.i_pipeline_type` | — | `stereo` | valid set is `RGB, RGBD, Stereo, RGBStereo, Depth, CamArray, Tof, DepthTof, StereoToF, RGBToF, Thermal` |
| `driver.i_enable_ir` | `true` | `false` | no-op: OAK-D **S2 has no IR emitter or dot projector** |
| `imu.i_batch_report_threshold` | 5 | 1 | lower latency. **Not the IMU-rate lever** — `i_acc_freq`/`i_gyro_freq` are (see 3b): left at their documented 480/400 defaults the BNO086 delivers 26 Hz; set to 200 it delivers 199 |

**Hardware spec correction:** the OAK-D **S2 stereo pair is OV9282 _monochrome_**, 1280×800
native — *not* the OV9782 colour part the old launch-file docstring claimed. Confirmed on this
device: `CAM_B`/`CAM_C = OV9282`, IMU = **BNO086** fw 3.9.9. Note the driver publishes those
sockets as **bgr8 anyway**, so a host-side mono8 conversion is still required (bug #7); the
sensor part number was never the reason it was needed. Re-check on any new unit with:

```bash
docker exec -it skybeta python3 -c "
import depthai as dai
with dai.Device() as d:
    for f in d.getConnectedCameraFeatures(): print(f)"
ros2 topic echo /oak/left/image_raw --field encoding --once
```

Sources: [driver](https://docs.luxonis.com/software-v3/depthai/ros/driver/) ·
[parameters](https://docs.luxonis.com/software-v3/depthai/ros/parameters/) ·
[OAK-D S2 hardware](https://docs.luxonis.com/hardware/products/OAK-D%20S2) ·
[v3 release notes](https://discuss.luxonis.com/blog/6392-depthai-ros-release-30)

## cuVSLAM grounding (isaac_ros_visual_slam **3.2**, which is what ships here)

**Read the version-matched docs.** `nvidia-isaac-ros.github.io` defaults to `latest`
(release-4.6), and the parameter set changed between 3.x and 4.x. The image installs
**3.2.6**. Checking `latest` will tell you `enable_imu_fusion` does not exist and that IMU
fusion is controlled by `tracking_mode` (`Multi-camera (0), Visual Inertial (1), RGBD (2)`) —
true for 4.x, **wrong for us**. The release-3.2 source is authoritative:

```
https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/release-3.2/
    isaac_ros_visual_slam/src/visual_slam_node.cpp     # the declare_parameter<> block
    isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp
    isaac_ros_visual_slam/src/impl/pose_cache.cpp
```

Verified defaults in **3.2** vs what `oakd_vslam.launch.py` sets:

| Parameter | 3.2 default | Launch sets | Note |
|---|---|---|---|
| `enable_imu_fusion` | `false` | `False` | **valid in 3.2** (renamed `tracking_mode` in 4.x); redundant but correct |
| `gyro_*` / `accel_*` noise | 0.000244 / 0.000019393 / 0.001862 / 0.003 | same values | verbatim copies of the defaults — no-ops, and only used when IMU fusion is on |
| `rectified_images` | `true` | `False` | correct — the launch feeds `image_raw` |
| `camera_optical_frames` | **`{}` (empty)** | *not set* | verify the base_link→optical-frame extrinsics actually resolve |
| `imu_frame` | `""` | `oak_imu_frame` | inert while IMU fusion is off |
| `image_jitter_threshold_ms` | **34.0** | *not set* | 30 fps = 33.3 ms nominal — 0.7 ms of headroom |
| `sync_matching_threshold_ms` | 5.0 | *not set* | left/right stamps must agree within 5 ms |
| `enable_localization_n_mapping` | **`true`** | *not set* | full SLAM + loop closure is running, not just odometry |
| `enable_slam_visualization` | `false` | now `False` | **the 13 `visual_slam/vis/*` topics are advertised regardless** — these flags gate *publishing*, not advertisement. 11 of 13 carry zero messages when off; `vis/slam_odometry` and `vis/velocity` are ungated in 3.2 |
| `enable_observations_view` | `false` | **`True`** | ditto |
| `enable_landmarks_view` | `false` | **`True`** | ditto |
| `path_max_size` | 1024 | 2048 | ditto |
| `image_qos` | `SENSOR_DATA` (BEST_EFFORT) | *not set* | the GPU converter publishes default RELIABLE — compatible; NITROS negotiation does not complete either way (see bug #7) |

**`image_jitter_threshold_ms` is warn-only — verified in the shipped binary.** In release-3.2
`visual_slam_impl.cpp:671-678` the over-threshold branch is a bare `RCLCPP_WARN` with no
`return`; `Track()` runs on the next line. The same value is passed to the library as
`configuration.max_frame_delta_ms` (3.2 uses ms; the `max_frame_delta_s = …/1000` form is
release-main/4.x), which the installed `cuvslam.h` documents as comparing deltas *"to warn user
and prevent mistakes"*. Its third use, `epsilon_stream2_` in `message_stream_sequencer.hpp`, is
a timeout to stop waiting for IMU; the only discard there is an out-of-order stamp. **Raising
the threshold recovers nothing.** Set it to ~40 ms only to silence a ~1/min cosmetic warning
caused by the driver's stamp wobble (32.2–33.6 ms) tipping over 34.0.

### What the docs do **not** say

NVIDIA documents `odom_frame` only as "the frame corresponding to the origin of the odometry
that cuVSLAM estimates", with the odometry pose being `odom_frame -> base_frame`. **Nothing
states its initial orientation.** So the claim underpinning the toilet-bowl diagnosis — that
`odom` is aligned to the rig's startup pose rather than to north or gravity — is *inference*,
not a documented fact. What supports it:

- `enable_imu_fusion` defaults to `false` and the launch keeps it off, so there is no inertial
  input and gravity is not observable.
- `visual_slam/vis/gravity` is documented as "the currently **estimated** gravity vector", and
  in `visual_slam_impl.cpp` `PublishGravity` early-returns unless `tracking_mode_ == VIO`.
- There is no north/mag input to cuVSLAM at all.

A frame with no gravity and no heading reference cannot be NED. But **measure it before acting**:

```bash
# 1. Start the stack, leave the vehicle still, then read the startup orientation:
ros2 run tf2_ros tf2_echo odom base_link
# 2. Yaw the airframe ~90 deg by hand and compare cuVSLAM yaw against the FC's:
ros2 topic echo /visual_slam/tracking/odometry --field pose.pose.orientation
ros2 topic echo /fmu/out/vehicle_attitude --qos-reliability best_effort --qos-durability transient_local
```

If cuVSLAM's yaw is offset from the FC's by a constant that equals the heading at startup, the
frame is startup-aligned and `POSE_FRAME_NED` is the wrong declaration.

## Target PX4 parameter sets

Enum values below are quoted from the **v1.16** parameter reference (the firmware this FC
actually runs — see bug #8). `*_CONFIG` params share one serial-port enum:
`0 = Disabled, 6 = UART 6, 101 = TELEM 1, **102 = TELEM 2**, 103 = TELEM 3,
104 = TELEM/SERIAL 4, 201/202/203 = GPS 1/2/3, 300 = Radio Controller, 301 = Wifi Port,
401 = EXT2, 1000 = Ethernet`.

**The one rule that matters: `MAV_1_CONFIG` and `UXRCE_DDS_CFG` must never both point at 102.**
One serial port, one owner. Whichever you are not using goes to `0`.

### State A — MAVLink, for proving the wire (§3 of the setup doc)

| Param | Value | Meaning |
|---|---|---|
| `MAV_1_CONFIG` | `102` | TELEM 2 |
| `MAV_1_MODE` | `2` | Onboard |
| `MAV_1_FLOW_CTRL` | `0` | Force off — CTS/RTS are not wired |
| `MAV_1_FORWARD` | `0` | — |
| `SER_TEL2_BAUD` | `57600` | drop to 57600 while debugging; 921600 also fine |
| `UXRCE_DDS_CFG` | `0` | Disabled — must not hold TELEM2 |

Then `mavproxy.py --master=/dev/ttyTHS1,57600 --show-errors`.

**Note: the May 7 dump already satisfies this** (`MAV_1_CONFIG=102`, `MAV_1_MODE=2`,
`MAV_1_FLOW_CTRL=0`, `SER_TEL2_BAUD=921600`, `UXRCE_DDS_CFG=0`). So a powered, correctly wired
6C would already be streaming. Silence on the wire is **not** a parameter problem.

### State B — uXRCE-DDS, what the container needs (§4/§7)

| Param | Value | Meaning |
|---|---|---|
| `MAV_1_CONFIG` | `0` | Disabled — frees TELEM 2 |
| `UXRCE_DDS_CFG` | `102` | TELEM 2 |
| `SER_TEL2_BAUD` | `921600` | must match `run_agent.sh` / `XRCE_BAUD` |
| `UXRCE_DDS_DOM_ID` | this drone's ID | **must equal `ROS_DOMAIN_ID`** in `/etc/skybeta/drone.env`. Per airframe; set by hand, not in the 02 file |
| `UXRCE_DDS_KEY` | non-zero | unique per client; default is fine |
| `UXRCE_DDS_PTCFG` | `2` | custom participant: the agent builds it from `host/dds/agent_refs_<scope>.xml`. **With `0`, the loopback-only stack never sees `/fmu/*`** (bench-measured) |

Reboot the FC after either change — every param above is reboot-gated.

### EKF2 for vision

| Param | Set to | Currently (May) | Why |
|---|---|---|---|
| `EKF2_EV_CTRL` | `9` | **0** | bitmask: `0`=horiz pos, `1`=vert pos, `2`=3D velocity, `3`=yaw. 9 = horiz pos + yaw. Add bit 1 (→11) only once vision altitude is trusted |
| `EKF2_EV_DELAY` | measured, ~50 | **0** | ms, range 0–300. Measure the real chain latency |
| `EKF2_HGT_REF` | `0` (Baro) | **2** (Range) | values: `0`=Baro, `1`=GPS, `2`=Range, `3`=Vision. Default is 1 |
| `EKF2_BARO_CTRL` | `1` | **0** | default is Enabled. `HGT_REF=0` with `BARO_CTRL=0` leaves **no height aiding** |
| `EKF2_EV_NOISE_MD` | `0` | 0 | 0 = trust the variance in the message (params as lower bound); 1 = use params directly. **At 0 the bridge's swapped variances (bug #4) actually reach the estimator** |
| `EKF2_EVP/EVV_NOISE` | `0.1` | 0.1 ✓ | — |
| `EKF2_EVA_NOISE` | `0.05` | 0.1 | README asks for 0.05 |
| `EKF2_EV_POS_X/Y/Z` | `0` | 0 ✓ | keep zero — the static TF carries the lever arm (bug #9) |

Only once vision is trusted, retire the sensors it replaces: `EKF2_OF_CTRL = 0` (optical flow,
currently `1`) and review `EKF2_RNG_CTRL` (currently `2`). Do not change these in the same
sitting as enabling EV — change one thing at a time or you cannot attribute the result.

## Flight symptoms → most likely causes

Two reported behaviours in position hold: **toilet-bowling**, and **altitude drifting up and
continuing up**. Both have specific suspects here. Ranked by how well they explain the
symptom and how cheap they are to check.

### Toilet bowl

**#1 suspect (now addressed): `pose_frame` was declared `POSE_FRAME_NED`, but the vision frame
is not NED.** The bridge used to set `out.pose_frame = VehicleOdometry.POSE_FRAME_NED`; it now
defaults to `POSE_FRAME_FRD` (verified on the bench: `pose_frame=2`). From
`px4_msgs/msg/VehicleOdometry.msg` (identical in `release/1.14` and `release/1.16`):

```
uint8 POSE_FRAME_NED     = 1 # NED earth-fixed frame
uint8 POSE_FRAME_FRD     = 2 # FRD world-fixed frame, arbitrary heading reference
```

cuVSLAM initialises its `odom` frame at the rig's **startup pose** — its x axis points wherever
`base_link` pointed when the node came up, not north. The ENU→NED rotation in the bridge is a
correct rotation, so what PX4 receives is a valid *local* frame with an **arbitrary heading
reference** — which is the literal definition of `POSE_FRAME_FRD`, not `POSE_FRAME_NED`.

Telling EKF2 the measurements are north-referenced when they are offset by (90° − startup
heading) leaves a constant yaw error between the position measurements and the mag/GPS-derived
attitude. The position controller then pushes at a fixed angle to the actual error, and the
vehicle orbits instead of converging. That is the textbook toilet-bowl mechanism.

Fix: use `POSE_FRAME_FRD` and let EKF2 estimate the frame rotation, **or** fuse EV yaw so the
estimator adopts the vision heading. Do not do a partial version of both.

**#2 suspect: `EKF2_EV_CTRL` without the yaw bit.** The README calls for bits 0+3
(horizontal position + yaw) = 9. The May param dump has `EKF2_EV_CTRL = 0`. Fusing EV
*position* while heading still comes from the magnetometer is the same frame-mismatch as
above, arrived at from the PX4 side.

**#3 suspect: `EKF2_EV_DELAY = 0`.** The dump has 0; the README asks for 50 ms. True
end-to-end latency here is plausibly 50–150 ms: USB 2.0 camera → depthai → a **Python**
GPU mono8 conversion → cuVSLAM → DDS → 921600 serial. Fusing stale position as if it were current
turns any oscillation into a growing orbit. Measure it, don't guess — `tools/latency_probe.py`
does exactly this and prints a recommended value.

**#4 suspect (now addressed): camera mount orientation was unmodelled.** The static TF was
`base_link → oak_parent_frame` with hardcoded `--roll 0 --pitch 0 --yaw 0`, and the old
`run_pipeline.sh <x> <y> <z>` exposed **translation only** — a down-pitched VIO camera, the norm
on a drone, could not be expressed at all, and an unmodelled mount rotation couples vehicle
rotation into apparent translation, which orbits. `oakd_vslam.launch.py` now takes
`camera_roll/pitch/yaw`, passed through `host/run.sh stack`. **Measure your actual mount and
pass it** — the default is still zero.

### Upward drift

**#1 suspect: the PX4 height reference, not the vision code.** The May dump has
`EKF2_HGT_REF = 2` (Range), `EKF2_BARO_CTRL = 0` (baro fusion **off**), and
`EKF2_RNG_A_HMAX = 5.0`. Above 5 m the rangefinder stops aiding, and with baro disabled there
is **no height aiding left at all** — the altitude estimate free-runs, the controller reads it
as sinking, and commands a climb that never terminates. That matches "drifts upward and then
continues upward" precisely, including the fact that it *starts* fine.

Check `EKF2_HGT_REF`, `EKF2_BARO_CTRL` and `EKF2_RNG_CTRL` on the FC **before** touching the
vision code. This is the cheapest fix on the list.

**#2 suspect: EV vertical fusion on top of pure stereo VO.** If `EKF2_EV_CTRL` bit 1 got
enabled, vision z goes straight into altitude — and `enable_imu_fusion: False` in
`oakd_vslam.launch.py` means this is *pure* stereo VO with no inertial or gravity constraint.
Vertical is the weakest axis for pure VO, especially over a low-texture floor with rotor
downwash in frame. The bridge's z math itself is correct (`-p.z` for ENU→NED).

**#3 contributor (now addressed): no reset handling.** `out.reset_counter` was hard-coded to 0,
so when cuVSLAM lost tracking and re-initialised, its pose jumped and PX4 was never told to
expect a discontinuity — EKF2 integrated the jump as real motion. The bridge now subscribes to
`/visual_slam/status` and increments a wrapping `reset_counter` on every non-tracking →
tracking transition (`vo_state`: `0` unknown, `1` success, `2` failed).

### Verified *not* the cause

cuVSLAM's `twist` **is** body-frame, so the bridge's `VELOCITY_FRAME_BODY_FRD` +
`(x, −y, −z)` is right. Confirmed in `pose_cache.cpp`: `GetVelocity` differentiates
`dp = poses.front().inverse() * poses.back()`, i.e. `T_body(t0)_body(t1)`, whose translation is
in the body frame — not the odom frame. (This also confirms bug #4: `velocity_variance` must
**not** take the world-frame x/y swap.)

Caveat: that velocity is a finite difference across the whole pose cache, not the last two
samples, so it is smoothed and lags. And `angular_velocity` is Euler-**rate**, not the body
angular-velocity vector — they diverge at non-zero roll/pitch. EKF2 does not fuse
`angular_velocity`, so it is cosmetic today.

## Simulation grounding (Isaac Sim 5.1.0 + Pegasus v5.1.0, x86 only) — see `sim/`

Isaac Sim cannot run on a Jetson (NVIDIA floor: RTX 4080 16 GB, 32 GB RAM, Ubuntu 22.04,
driver >= 595; only DGX Spark is supported on aarch64). The sim profile is x86-only and lives in
`sim/` + `docker/Dockerfile.sim`. Pegasus **v5.1.0 is pinned to Isaac Sim 5.1.0 and is NOT
compatible with Isaac Sim 6.0**.

**x86 base image:** `nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_f247dd1051869171c3fc53bb35f6b907`
— the amd64 twin of the aarch64 tag `docker/Dockerfile.jetson` pins (same `isaac_ros_common`
layer history, build timestamps 2025-11-07). Established by enumerating all 291 NGC tags and
diffing image configs, not guessed.

Four findings from building the profile that also matter on hardware:

- **cuVSLAM takes the stereo baseline from TF, not from `CameraInfo` `P[3]`.** The static
  transforms in the launch file *are* the rig geometry. This is why bug #5's
  `i_reverse_stereo_socket_order` is not the lever for a baseline sign error — the TF is.
- **Isaac Sim 5.1 deprecated `set_rational_polynomial_properties`**, so Pegasus's `intrinsics`
  fx/fy and `diagonal_fov` config keys are silently ignored and fx/fy are derived from the prim
  lens instead. Unpatched, the sim camera is ~47 deg HFOV rather than 80. `sim/pegasus_skybeta.py`
  overrides this before the CameraInfo writer snapshots.
- **Zero distortion coefficients are load-bearing in sim:** any non-zero k4/k5/k6 makes
  `read_camera_info` report a `rational_polynomial` model, which contradicts
  `rectified_images: True` and breaks cuVSLAM's assumption.
- **Pegasus `px4_autolaunch` defaults off**, and when it does launch PX4 it uses
  `cwd=TemporaryDirectory()`, so `param save` never persists. Set SITL params every run.
  PX4 SITL is the TCP *client*: the pass condition for step 1 of the spike is PX4 blocking at
  "Waiting for simulator", not a prompt.

The RGB-capture risk flagged during planning is smaller than feared: images do not flow through
the commented-out `get_rgba()` — `ros2_backend.py:398-403` attaches a Replicator writer to the
camera's render product. The spike checklist still verifies it at step 4.

Pegasus was tested upstream against PX4 v1.14.3; we target **v1.16.2** SITL. That gap is the
main open risk, and `sim/README.md` is an ordered spike checklist with the expected output and
failure meaning for each step. Gazebo Harmonic is the documented fallback.

## End-to-end bench verification (2026-09-03, image `skybeta:0.1.0`)

Full stack on the rebuilt image — driver + GPU converter + cuVSLAM + bridge, camera static on
the bench, launched with `camera_x:=0.10 camera_z:=-0.05 camera_pitch:=-0.2618`. 44 s mcap,
header stamps. This is the state a competitor forks.

| topic | n | rate | p99 gap | max gap | note |
|---|---|---|---|---|---|
| `/oak/left/image_raw` | 1312 | 30.000 Hz | 33.33 ms | 66.67 ms | **1024×640 bgr8**, one dropped frame in 44 s |
| `/oak/left/image_mono` | 1312 | 30.000 Hz | 33.33 ms | 66.67 ms | **mono8**, GPU converter |
| `/oak/imu/data` | 8618 | **197.9 Hz** | — | — | p50 5.02 ms |
| `/visual_slam/tracking/odometry` | 1313 | 30.023 Hz | — | — | `vo_state = 1` on **1313/1313** |
| `/fmu/in/vehicle_visual_odometry` | 1301 | 29.952 Hz | — | — | `pose_frame=2` (FRD), `velocity_frame=3` (BODY_FRD), `reset_counter=[0]` |

- Static TF resolves: `base_link → oak_left_camera_optical_frame` = translation
  `[0.099, 0.038, -0.050]` with the −15° pitch composed into the optical rotation, so the new
  `camera_roll/pitch/yaw` launch args do reach the transform.
- Pose covariance identity fallback: **0 of 1313**.
- **The bridge's identity-covariance warning fired anyway, and that was a bug in the warning.**
  cuVSLAM's `PoseCache` returns the 1.0 identity for the first frame or two before it holds two
  poses, so warning on first sight cries wolf on every run. It now requires a full second (30
  consecutive messages) of identity before warning, and any real covariance resets the streak.
  Tested three ways: startup transient silent, sustained failure warns exactly once at the 30th
  message, 29-message bursts silent.

Not covered on the bench: `vo_state` under motion (camera was static), and everything
downstream of the UART — the TELEM2 connector is still broken.

## Fleet bench verification (2026-10-12, image `skybeta:0.2.0`, branch `fleet-ready`)

Context: every team gets one drone, all drones share one WiFi network, and teams write code on
the drone itself. Measured on the dev Orin (`wlP1p1s0`), not assumed.

**DDS isolation**

| Check | Result |
|---|---|
| RMW in the image | `rmw_fastrtps_cpp`, Fast DDS 2.6 |
| MicroXRCEAgent's Fast DDS | its own **2.14.7** (`/usr/local/lib/libfastrtps.so.2.14.7`), separate from ROS's |
| `fastdds_localhost.xml`, ros:humble container, 7 publishers + 2 echoes | all discovered; **0** UDP packets on eth0 (294 without the profile) |
| same, but default `maxInitialPeersRange` (4) | only 4 of 7 topics discovered — loopback has no multicast, so the range is load-bearing |
| full stack, local scope, domain 1 | 29 topics, cuVSLAM odometry 29.99–30.02 Hz, **0** DDS packets on WiFi in 30 s |
| full stack, network scope (control) | 71 packets in 30 s, all to `239.255.0.1:7650` = 7400 + 250 x domain 1 |

**The agent does not take the default profile.** An XRCE client compiled from the agent's own
examples, sending PX4 v1.16.2's exact participant XML (quoted from
`uxrce_dds_client.cpp`) to the real agent in the image, with the ROS side on
`fastdds_localhost.xml`; discovery judged by a long-lived rclpy node (see gotchas):

| FC `UXRCE_DDS_PTCFG` | ROS node discovers the agent's topic | DDS packets on WiFi |
|---|---|---|
| no profile anywhere (old behaviour) | yes | 31 |
| 0, ROS on the localhost profile | **never** (15 s) | 15 |
| 1 (PX4's localhost-only XML) | yes, 1.4 s | 0 |
| 2 + `agent_refs_local.xml` | yes, 0.2 s (1.6 s inside the real `run.sh` stack) | 0 |
| 2 + `agent_refs_network.xml`, ROS default transports | yes, 0.2 s | 20 (expected) |
| 2, agent started without its refs file | fails loudly: `reference file ... does not exist` | — |

The real PX4 then confirmed the `PTCFG=0` row: session up, its participant on WiFi, no `/fmu`
data in domain 1 — see Hardware status.

**Team workspace**: a package written on the host into `~/team_ws/src` built in the
competitor window (7 s), launched, and published (received by an rclpy subscriber); `build/`,
`install/` and `src/` owned by the host user; everything survived the container exiting. A
team `px4_msgs` is flagged by `team_build.sh`; the VIO pipeline stays on the underlay.

**Also run, all passing**: 121/121 unit tests in the image (`skybeta_flight_interface`,
`oakd_vslam_bringup`), `param_diff --selftest` all six steps against the committed reference
dump, `test_drone_env.sh` 30/30, `test_extlinux_fix.sh` 8/8 on the live `extlinux.conf`.

**Not yet verified**: `/fmu` data arriving through the 0.2.0 stack with `UXRCE_DDS_PTCFG=2` on
a real FC (the TELEM2 link dropped first); two drones on one access point.

## Known bugs

**Status: #0–#9 are all FIXED as of 2026-09-03** (branch `competition-base`, verified — see the
end-to-end bench section above). **#10–#14 are FIXED on branch `fleet-ready` as of
2026-10-12** (image `skybeta:0.2.0`, see the fleet bench section above); #13 additionally
needs `UXRCE_DDS_PTCFG=2` on every FC, which has not yet been carried through a real
`/fmu` data run. They are kept here because the *diagnosis* is the valuable part:
each one records how the symptom presented and what the evidence was, so a regression is
recognisable. Anything still open is called out explicitly in its own entry.

### 0. FIXED — camera driver could not load at all (`libdiagnostic_updater.so`)

The image built cleanly but `run_slam.sh` was dead on arrival:

```
[oak_container] Failed to load library: Could not load library dlopen error:
libdiagnostic_updater.so: cannot open shared object file: No such file or directory
```

Root cause is a silent apt trap, not a missing dependency declaration:

1. `ros-humble-depthai-ros-driver-v3` correctly declares `Depends: ros-humble-diagnostic-updater`.
2. The **NGC base image already has `ros-humble-diagnostic-updater 4.0.6` installed**, and that
   build ships **only the Python module** — `dpkg -L` lists 68 files and **zero** `.so`.
3. apt sees the dependency as already satisfied and never upgrades it.
4. The composable node `dlopen`s the C++ library at runtime and dies.

Fix applied to the Dockerfile: name `ros-humble-diagnostic-updater` explicitly in the apt list,
which pulls the candidate (4.0.7) and provides `/opt/ros/humble/lib/libdiagnostic_updater.so`.
Verified by upgrading in a throwaway container before committing to a rebuild.

**Watch for more of this.** The base image carries a set of pre-installed ROS packages that apt
will not upgrade just because something depends on them. If another node fails to `dlopen` a
library, check `dpkg -l` against `apt-cache policy` before assuming the package is absent.

**Related smell, also fixed:** the Dockerfile used to install `ros2-testing-apt-source`, which
added `packages.ros.org/ros2-testing`; the candidate for several packages then came from
**testing**, whose versions move without notice, so two builds of this "pinned" image weeks apart
could differ. It is no longer installed, and the ROS source is now a dated snapshot — see the
Gotchas section.

### 1. FIXED — `jetson_uart_dma_fix.sh apply` exited 1 on success

`main()` ends with `[[ $do_reboot -eq 1 ]] && { ...; reboot; }` (`jetson_uart_dma_fix.sh:404`).
Without `--reboot` that list returns 1, and because `main "$@"` is the script's last command
under `set -Eeuo pipefail`, the script exits 1 despite having succeeded. Same pattern in the
`restore` branch.

Cascade: `02_uart_fix.sh` and `setup_and_build.sh` both call `sudo jetson_uart_dma_fix.sh apply`
under their own `set -euo pipefail`, so **both abort immediately after a successful patch** —
`02` never reaches its `sudo reboot`, and `setup_and_build.sh` never reaches step 3 (the image
build).

**Fix:** every branch of `main()` now ends with an explicit `exit 0`. `apply --dry-run` returns 0
and is covered by `host/tests/test_extlinux_fix.sh`.

### 2. FIXED — `extlinux.conf` gained a duplicate `FDT` line on every `apply`

This host currently has the line four times:

```
      FDT /boot/dtb/uart_dma_fix.dtb   ×4
```

The Python helper (`jetson_uart_dma_fix.sh:308-319`) iterates lines in order and reaches
`LINUX /boot/Image` *before* the pre-existing `FDT`, so `not inserted` is still true and it
inserts a fresh line — then separately rewrites the old one further down. Boots fine (all four
paths are identical, U-Boot takes one), but it grows without bound. Almost certainly a
downstream symptom of #1: the bogus exit 1 invites re-running `apply`.

**Fix:** the helper now does a pre-pass that deletes every `FDT` inside `LABEL primary`, then
inserts exactly one after `LINUX`. `apply` also no-ops when the patch is already live (detected
from `/proc/device-tree/bus@0/serial@3100000/iommus`, no sudo needed).
`host/tests/test_extlinux_fix.sh` asserts one FDT line, correct placement, byte-identical
remainder, and idempotency across two passes — run against this host's real 4-line file.

**Note:** this host's `/boot/extlinux/extlinux.conf` still physically has the four lines.
Collapsing them needs root, so `provision.sh` does it; it is harmless (all four paths identical).

### 3. FIXED — duplicate YAML keys dropped the resolution; but 1280×800 does **not** fit

`config/oakd_vslam.yaml` declared `left:` and `right:` **twice each**; YAML keeps the last
occurrence, so `i_width`/`i_height`/`i_fps` never reached the driver and it ran at the
documented 640×400 default. Merged into one block per side.

**The obvious fix — asking for the sensor's native 1280×800 — fails.** Measured driver-only,
six configs, 40 s each, `camera_info` only:

| config | rate | note |
|---|---|---|
| 1280×800 ×2 @30 | **23.3 Hz** | 143 MB/s — the ceiling; 23 % of frames lost |
| 1280×800 **left only** @30 | 30.0 Hz | proves it is throughput, not sensor/ISP/host |
| 1280×800 ×2 @20 | 20.0 Hz | clean, but costs a third of the rate |
| 1280×800 ×2 @30 + `i_low_bandwidth` | 23.3 Hz | no help |
| **1024×640 ×2 @30** | **30.0 Hz** | 118 MB/s, 2.56× the pixels — **shipped** |
| 960×600 ×2 @30 | 30.0 Hz | 104 MB/s, more margin |

**This is the XLink ceiling, not USB.** 143 MB/s is ~1.1 Gbps, far below USB 3.0 SuperSpeed's
5 Gbps — so the earlier reasoning here ("USB 3.0, therefore 1280×800 has headroom") was wrong.
The MyriadX's XLink transport tops out near 120–140 MB/s regardless of the USB link rate.

Worse, at 1280×800 the two eyes drop *different* frames: L/R header sync p95 rises to 33.3 ms
(one whole frame), few stereo pairs match, and cuVSLAM odometry collapses to **16.4 Hz** with
1154 jitter warnings. The rationale is recorded in a comment block in the YAML — do not "fix"
it back to 1280×800 without re-measuring.

### 3b. MEASURED (corrected) — camera timing is clean; the earlier jitter alarm was an artefact

Two 60 s `ros2 bag record` runs of the driver alone, analysed from **header stamps** (what
cuVSLAM actually compares), on power mode `MAXN_SUPER` (`pmode 0002`, already the max):

| | run 1 | run 2 |
|---|---|---|
| rate | 30.02 Hz | 30.02 Hz |
| interval p50 / p99 / max | 33.33 / 33.34 / 33.36 ms | 33.33 / 33.34 / 33.35 ms |
| intervals > 34 ms | **0** | **0** |

The device clock is essentially perfect. The earlier "27.9 Hz, max 98 ms" figure came from
`ros2 topic hz`, which measures **arrival** time through a CLI subscriber — and arrival does
jitter (p95 ≈ 39 ms, max 50–83 ms, 14–17 % of gaps > 34 ms; header→arrival latency p50
≈ 22 ms, max 73 ms). That is host delivery jitter, not dropped frames, and cuVSLAM's check
uses header stamps (`latest_ts - last_track_ts` in `visual_slam_impl.cpp`), so it does not
see it.

Under full load (driver + `bgr_to_mono` + cuVSLAM) cuVSLAM logged six over-threshold
warnings in 3.5 min: 933 ms at tracker start (initialisation), four of 34.1–35.1 ms (driver
stamp wobble tipping over a threshold with 0.7 ms of margin), and one 66.7 ms whose header
stamp sits 0.1 s *before* the recorder subscribed — a rosbag2-startup artefact. Steady-state
frame loss: **0 in 1954**. So: not a jitter crisis. Consequences:

- `jetson_clocks`/`nvpmodel` are not a fix here — it is already MAXN_SUPER.
- **`ros2 topic hz` is the wrong tool for this stream.** Re-run under load over 1026 samples it
  still said 27.1 Hz / max 150 ms while the same stream was 30.016 Hz by header stamp and
  30.18 Hz by C++ bag receive. The Python subscriber drops ~10 % in its own queue. Measure
  with `ros2 bag record -s mcap` and analyse header stamps.
- Over-threshold jitter is **warn-only** (see cuVSLAM grounding). The launch now sets 40 ms,
  which is cosmetic: it stops the driver's 32.2–33.6 ms stamp wobble tripping a 34.0 threshold.

**State after the WP2/WP3 fixes** (60 s mcap, full stack, header stamps, 1024×640):

| metric | before | after |
|---|---|---|
| resolution / encoding | 640×400 bgr8 | **1024×640**, `image_mono` is **mono8** |
| image_raw rate, max gap | 30.02 Hz, 33.37 ms | 30.017 Hz, 33.37 ms, 0 % > 34 ms |
| bgr8→mono8 added latency | p50 10.12 / p95 16.42 ms (Python) | **p50 5.18** / p95 17.25 ms (GPU) |
| frames lost in conversion | 2 of 1714 | **0 of 1743** |
| odometry | 30.001 Hz | 30.017 Hz, `vo_state`=1 on **1744/1744** |
| cuVSLAM track time | p50 2.5 / max 20.4 ms | p95 3.88 / max 13.25 ms |
| **IMU** | **26.3 Hz** | **199.1 Hz** (p50 5.02 ms) |
| jitter warnings | ~1/min | 1 in ~80 s |

- **The IMU root cause was the frequency defaults, not batching.** Leaving `i_acc_freq` /
  `i_gyro_freq` at their documented 480/400 yields 26.3 Hz on the BNO086; asking for 200
  explicitly gives 199.5. `i_sync_method: LINEAR_INTERPOLATE_ACCEL` beats `COPY` on regularity
  (p95 5.5 vs 8.8 ms). IMU is no longer a blocker for enabling `enable_imu_fusion`.
- Covariance from cuVSLAM is real (0/1962 identity fallbacks) but tiny — median σx ≈ 78 µm,
  5–7 orders below `EKF2_EVP_NOISE = 0.1`, which `EKF2_EV_NOISE_MD = 0` applies as a lower
  bound. EKF2 clamps all six. Bug #4 is therefore **unobservable as configured**; the fix is
  still correct (the raw values are anisotropic, x ≈ 5× y) but not urgent.

### 4. FIXED — covariance conversion applied a world-frame swap to body-frame quantities

`src/oakd_vslam_bringup/oakd_vslam_bringup/px4_odom_bridge.py:130-137`.

`position_variance = [pc[7], pc[0], pc[14]]` is **correct** — ENU→NED permutes x/y. But
`orientation_variance` and `velocity_variance` reuse that same permutation, and both are
body-frame quantities whose FLU→FRD mapping is `(x, -y, -z)`: sign flips only, which square
out of a variance. The node even publishes `velocity = [v.x, -v.y, -v.z]` with no swap while
its variance *is* swapped, so roll↔pitch and vx↔vy variances are transposed relative to the
data they describe. Should be `[pc[21], pc[28], pc[35]]` and `[tc[0], tc[7], tc[14]]`.

**Fix:** now `orientation_variance = [pc[21], pc[28], pc[35]]` and
`velocity_variance = [tc[0], tc[7], tc[14]]`; `position_variance` keeps the x/y swap. Covered by
unit tests with distinct diagonal values.

**Impact was nil as configured** — cuVSLAM's real variances are 5–7 orders below
`EKF2_EVP_NOISE`, which `EKF2_EV_NOISE_MD = 0` applies as a lower bound, so EKF2 clamps all six
regardless (see 3b). It matters the moment that bound drops: the raw values are anisotropic,
x ≈ 5× y.

### 5. FIXED — the `stereo:` config block was dead; nothing it set took effect

`config/oakd_vslam.yaml` sets `stereo.i_publish_left_rect`, `i_publish_right_rect`,
`i_publish_synced_rect_pair`, `i_depth_preset` and `i_reverse_stereo_socket_order`. With the
driver actually running, the published topic list is:

```
/oak/imu/data
/oak/left/camera_info   /oak/left/image_raw   (+ compressed / compressedDepth / theora)
/oak/right/camera_info  /oak/right/image_raw  (+ compressed / compressedDepth / theora)
```

No `*_rect` topics and **no depth topic at all** — the stereo/depth node is never instantiated,
so the whole `stereo:` block was inert. Not a bandwidth waste (nothing was published), but
`i_reverse_stereo_socket_order: true` was **also** doing nothing.

**Fix:** the block is deleted from `config/oakd_vslam.yaml`. If the stereo baseline sign ever
needs flipping, the lever is the **static TF**, not this parameter — cuVSLAM takes the baseline
from TF, not from `CameraInfo` (see the simulation-grounding section).

### 6. FIXED — `remain-on-exit` only applied to the first tmux window

`container_scripts/run_full_stack.sh` runs `tmux set-option -t "$SESSION" remain-on-exit on`
*before* creating the `slam` and `bridge` windows. `remain-on-exit` is a **window** option, so
targeting a session applies it to that session's current window only. The `slam` and `bridge`
panes therefore vanished on crash instead of holding their exit code — the opposite of what the
README promised.

**Fix:** `tmux set-option -g remain-on-exit on`, set in the same invocation that starts the
server (a bare `tmux start-server` does not survive — `exit-empty` defaults on). Verified: all
four windows report `remain-on-exit=on pane_dead=1`; under the old form only the first did.

### 7. FIXED — `bgr_to_mono` replaced by the GPU converter (NITROS does *not* negotiate)

The driver publishes `/oak/left/image_raw` as **`bgr8`** even though `CAM_B`/`CAM_C` are
`OV9282` *mono* sensors (the v3 driver reports `supportedTypes=[MONO, COLOR]` and publishes
these sockets as colour), and the v3 parameter reference exposes no encoding/output-type option
for them. So a host-side conversion to the `mono8` cuVSLAM needs is unavoidable.

The two Python `bgr_to_mono` nodes are gone, replaced by two `isaac_ros_image_proc`
`ImageFormatConverterNode`s (`encoding_desired: mono8`) loaded into the same composable
container as cuVSLAM. Measured: conversion latency **p50 10.12 → 5.18 ms**, frames lost
**2/1714 → 0/1743**.

**But it is not zero-copy.** cuVSLAM 3.2 uses a *Managed*NitrosSubscriber, so type negotiation
never completes and `type_negotiation_duration_s` changes nothing. The conversion itself runs on
the GPU; the hand-off to cuVSLAM is a plain in-container `sensor_msgs/Image`. The docstring says
so rather than claiming NITROS zero-copy. The converter's `nitros_image_rgb8` input label is
harmless here — the mono8 output is bit-exact with the bgr8 channels, since all three channels
are identical on a mono sensor.

The old docstring claim that the S2 has "color stereo sensors (OV9782)" was wrong twice over:
the part is OV9282, and the bgr8 comes from the driver, not the sensor.

### 8. FIXED — `px4_msgs` was pinned to `release/1.14` but the FC runs v1.16.2

The Dockerfile used to clone `px4_msgs` at branch `release/1.14`, and the README, the bridge
docstring and the launch comments all asserted "PX4 v1.14". The evidence above says the flight
controller is **v1.16 or newer**. uXRCE-DDS matches topics by message definition, so a
`px4_msgs` generation that doesn't match the firmware is exactly how you get an agent that
connects, advertises topics, and moves no data.

**Fix:** `docker/Dockerfile.jetson` pins `PX4_MSGS_REF=release/1.16` (recorded SHA
`392e831c…`) and adds `px4-ros2-interface-lib` at `release/1.16`. The installed
`VehicleOdometry.msg` carries `uint32 MESSAGE_VERSION = 0`, which only exists from 1.16.

**Caveat — the "moves no data" claim was never demonstrated.** Humble's rmw matches on type
*name*, not a hash (hashes arrive in Iron), and `VehicleOdometry`'s wire layout is identical
between 1.14 and 1.16. Data may well have flowed. The pin is right regardless; the real test is
the bench one: `check_topics.sh` must show a topic **echoing data**, not merely appearing in
`ros2 topic list`. That test is still blocked on the TELEM2 cable.

### 9. FIXED — `EKF2_EV_POS_*` does exist; the launch comment claimed it didn't

`src/oakd_vslam_bringup/launch/oakd_vslam.launch.py:24` justifies the static
`base_link → oak_parent_frame` TF with "(v1.14 has no `EKF2_EV_POS_*`)". Those parameters are
in the v1.14 parameter reference, in v1.15, in v1.16, **and** in this FC's own param dump
(`EKF2_EV_POS_X/Y/Z`, all `0.000000`).

The static TF is a perfectly good way to do it — but the comment's reasoning was false, and it
hid a real hazard: setting `EKF2_EV_POS_*` on the FC *and* leaving the static TF in place applies
**the camera lever arm twice**.

**Fix:** the comment is corrected. The TF is the single source of truth, driven by
`host/run.sh stack <x> <y> <z> [roll pitch yaw]`; `px4/params/03b_ekf2_vision.params` pins
`EKF2_EV_POS_X/Y/Z` to `0` and says why.

### 10. FIXED — every drone's DDS met every other drone's on a shared network

`host/run.sh` runs every container `--network host` and set no domain or transport, so all
drones' ROS nodes and agents shared domain 0 on the WiFi: drone A's `/fmu/in/*` setpoints
were visible to drone B's agent, and the 30 Hz stereo topics went out as multicast. Fixed by a
per-drone `ROS_DOMAIN_ID` (`/etc/skybeta/drone.env`, matched by `UXRCE_DDS_DOM_ID` on the FC,
now a per-unit exclusion in `param_diff`) and `SKYBETA_DDS_SCOPE=local` by default. See #13
for the part of this that did not work first time.

### 11. FIXED — team code vanished when the container exited

Every container is `--rm`, and the competitor guide said to `colcon build` inside it. Code,
build output and any `apt`/`pip` installs were lost on exit. Team code now lives in
`$TEAM_WS` on the host (mounted at `/root/team_ws`, built by `team_build.sh`, chowned back to
the host user); extra packages go in `$TEAM_WS/Dockerfile` (`host/run.sh build`).

### 12. FIXED — the image's pip packages were unpinned

`pip3 install depthai MAVProxy`. A rebuild for 0.2.0 resolved numpy 2.2.6 over the base's
1.26.4, opencv-python 5.0 and vtk (via MAVProxy 1.8.75), and depthai 3.11 — none of it ever run
on the vehicle. Pinned to what 0.1.0 shipped (`pip3 list` in that image): depthai 3.10.0,
MAVProxy 1.8.74, pymavlink 2.4.49, `numpy<2`.

### 13. FIXED (needs `UXRCE_DDS_PTCFG=2` on each FC) — the agent ignored the localhost profile

The first fix for #10 mounted `fastdds_localhost.xml` as the *default* profile, on the
assumption that it reaches MicroXRCEAgent too. It does not: with `UXRCE_DDS_PTCFG=0` PX4 sends
its own participant XML and the agent builds exactly that, on every interface. Worse, the ROS
nodes, now loopback-only, never discover it — so in local scope `/fmu/*` would silently never
reach the bridge. It only looked healthy on the bench because TELEM2 was dead. Measured
(table above), then seen on the real PX4. Fixed by `UXRCE_DDS_PTCFG=2` (in
`02_telem2_uxrce_dds.params`): PX4 asks for the participant by the reference
`px4_participant`, which the agent builds from `host/dds/agent_refs_<scope>.xml` passed as
`-r`. PTCFG=2 over 1 so that the scope switch in `drone.env` moves the agent too.

### 14. FIXED — `COMPETITOR_PKG` / `COMPETITOR_LAUNCH` never reached the container

The competitor guide documented `COMPETITOR_PKG=my_team_nav host/run.sh stack`, but `run.sh`
never passed either variable into `docker run`, so the stack always launched
`competitor_example`. Now forwarded (and settable in `drone.env`).

## Gotchas when working here

- The image is **aarch64-only** — the NGC Isaac ROS base publishes no amd64 manifest.
- Apt sources in the NGC base, precisely: `/etc/apt/sources.list.d/` holds `cuda`,
  `nvidia-l4t`, `ros2`, `nodesource`, `yarn`, `mosquitto` — **and** `/etc/apt/sources.list`
  itself carries `deb https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3.0`.
  So `ros-humble-isaac-ros-visual-slam` (3.2.6) comes from NVIDIA's pinned `release-3`
  channel, while `ros-humble-depthai-ros-driver-v3` (3.3.0) comes from the ROS repo. The old
  `rm -f .../nvidia-isaac-ros*.list` never matched anything (wrong directory) and has been
  removed. `docker/Dockerfile.jetson` now swaps the ROS source for a dated
  `snapshots.ros.org/humble/<SNAPSHOT_DATE>` (build ARG, currently `2026-08-07`, key vendored
  in `docker/ros-snapshots.key.asc`) and leaves the Isaac channel alone — that is what makes
  two builds weeks apart identical.
- `Micro-XRCE-DDS-Agent` is pinned to **v2.4.3** because the **PX4 v1.16 ROS 2 guide specifies exactly that tag** (`git clone -b v2.4.3 …`). The older "v3.x is wire-incompatible with v1.14" rationale in this repo's history was wrong. Don't bump it.
- `px4_msgs` (`release/1.16`) and `px4-ros2-interface-lib` (`release/1.16`) are cloned at build
  time; their resolved SHAs are baked into the image at `/root/skybeta_ws/*.sha` so a drifted
  rebuild is detectable.
- **Test DDS discovery with a long-lived node, not `ros2 topic list --no-daemon`.** With
  loopback-only transports, discovery is unicast and slower than the CLI's brief wait; the CLI
  reported "not found" for an agent participant that a 15 s rclpy node found in 0.2 s.
- **Never put `--` inside an XML comment in `host/dds/`.** It makes the file malformed, and Fast
  DDS then silently ignores the profile and falls back to every interface. Done twice in one
  session; `run.sh` and `test_drone_env.sh` now refuse a profile that does not parse.
- **Prove isolation with a packet capture**, e.g. `tcpdump -ni wlP1p1s0 'udp portrange
  7400-7700'` from `docker run --rm --network host alpine` (no sudo needed), and always run the
  `network`-scope control so you know the capture can see a leak.
- `host/run.sh stack` needs a TTY (`docker run -it`). Headless, e.g. over SSH from a script:
  `sleep infinity | script -qfc "host/run.sh stack" stack.log`. And never `pkill -f` a pattern
  that also appears in your own SSH command line — it kills the session.
- Editing `container_scripts/` rebuilds only the last layers (~1 s); editing anything under
  `src/` re-runs colcon for all four packages (~10 min on Orin Nano). `.dockerignore` keeps
  local `build/`/`install/`/`log/` and recorded bags out of the context so they don't bust that
  layer.

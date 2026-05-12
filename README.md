# skybeta

Self-contained Jetson image for a Pixhawk-paired drone:

- **cuVSLAM** on an OAK-D-S2 (visual odometry @ ~30 Hz, replaces optical flow)
- **PX4 bridge** publishing `/fmu/in/vehicle_visual_odometry` (ENU→NED + FLU→FRD, BEST_EFFORT QoS)
- **Micro-XRCE-DDS-Agent v2.4.3** speaking to PX4 v1.14 over `/dev/ttyTHS1` @ 921600
- **MAVProxy** for the initial UART sanity test (§3 of the setup doc)

The Dockerfile builds one image (~18–22 GB) on the NGC Isaac ROS base. Distribute to other Jetsons via `docker save | docker load` or a registry — host scripts handle the few things a container can't (UART DMA patch, OAK-D udev rule, docker daemon's nvidia runtime).

## What runs where

| Section of [`jetson_pixhawk_uxrce_ros2_setup.md`](jetson_pixhawk_uxrce_ros2_setup.md) | Where |
|---|---|
| §1 Jetson UART DMA fix | host (`02_uart_fix.sh` → `jetson_uart_dma_fix.sh apply` + reboot) |
| §2 Wire Pixhawk TELEM2 | hardware |
| §3 MAVProxy sanity test | container (`mavproxy.py` baked in) |
| §4 PX4 / QGC params | Pixhawk side (set via QGroundControl) |
| §5 Micro-XRCE-DDS-Agent v2.4.3 | container (`run_agent.sh` on `$PATH`) |
| §6 ROS workspace (px4_msgs + oakd_vslam_bringup) | container (`/root/skybeta_ws/install`, colcon-built) |
| §7/§8/§10 Run agent + verify | container (`run_full_stack.sh`, `check_topics.sh`) |
| cuVSLAM + camera-offset TF | container (`run_slam.sh`) |
| ENU→NED + FLU→FRD bridge | container (`run_bridge.sh`) |

## One-shot setup (fresh Jetson)

```bash
./setup_and_build.sh        # 01 prereqs + 02 UART patch (reboots) + 03 image build
```

Re-run the script after the reboot to finish the image build. First build pulls ~8 GB NGC base + apt + colcon → 30–40 min on Orin Nano. Subsequent builds with no source change are cached in seconds.

## Run the full stack

```bash
./run_pipeline.sh                       # camera at body origin
./run_pipeline.sh 0.10 0.0 -0.02        # camera_x camera_y camera_z (FLU, meters)
```

Drops you into a tmux session inside the container with three windows: `agent`, `slam`, `bridge`. `Ctrl-b 0..2` switches; `Ctrl-b d` detaches. Exit the container or `tmux kill-session -t skybeta` tears everything down.

Verify from a host shell:

```bash
docker exec -it skybeta-pipeline bash -lc \
  'source install/setup.bash && ros2 topic hz /fmu/in/vehicle_visual_odometry'
```

Expect ~30 Hz once cuVSLAM is tracking. On the Pixhawk MAVLink console, `uxrce_dds_client status` should show `/fmu/in/vehicle_visual_odometry` subscribed with `recv > 0`.

## Step-by-step (instead of one-shot)

```bash
./01_prereqs.sh             # apt deps, docker nvidia runtime, OAK-D udev
./02_uart_fix.sh            # device-tree patch + reboot           (host, one-time)
./02b_uart_fix_verify.sh    # after reboot: confirm "Adding to iommu group"
./03_build_image.sh         # docker build -t skybeta:latest .
./run_pipeline.sh           # full stack in one tmux'd container
```

## Interactive container (for poking around / MAVProxy / individual launches)

```bash
./04_run_container.sh       # drops into bash inside skybeta:latest
# inside the container — every command is on $PATH:
run_agent.sh                # MicroXRCEAgent only
run_slam.sh                 # cuVSLAM only
run_bridge.sh               # PX4 odom bridge only
check_topics.sh             # ros2 topic list + timesync_status echo
mavproxy.py --master=/dev/ttyTHS1,57600   # §3 MAVLink sanity test
```

Re-running `./04_run_container.sh` from a second host shell attaches a new bash to the same container.

## Inside the image

Base: `nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c` — same NGC Isaac ROS Humble image that `isaac_ros_common`'s `run_dev.sh` pulls (the bare `aarch64-ros2_humble` tag does not exist on NGC, only the content-addressed variants). Pinned by hash so every Jetson in a fleet builds against the exact same base. JetPack 6.x / L4T R36.x compatible. aarch64-only.

| Layer | Step | Purpose |
|---|---|---|
| 1 | apt | `depthai-ros-driver-v3`, `isaac-ros-visual-slam`, NITROS imu/odometry types, build deps, tmux. Kept in one RUN so the ~3 GB layer caches as a unit. |
| 2 | pip | `depthai` (runtime), `MAVProxy` (§3 UART sanity test) |
| 3 | udev | `03e7` OAK-D rule baked in (also installed on host by `01_prereqs.sh`) |
| 4 | git+cmake | Micro-XRCE-DDS-Agent **v2.4.3** built from source. v3 is wire-incompatible with PX4 v1.14. |
| 5 | colcon | `/root/skybeta_ws` with `px4_msgs @ release/1.14` + `oakd_vslam_bringup` |
| 6 | COPY | `container_scripts/` onto `/usr/local/bin`, so `run_agent.sh` etc. are callable by name |
| 7 | bashrc | Auto-source `/opt/ros/humble` + the workspace overlay for every interactive shell |

Resulting image is ~13 GB unpacked (most of it depthai + Isaac ROS apt packages from layer 1).

### Runtime configuration

`run_pipeline.sh` and `04_run_container.sh` both `docker run` with:

| Flag | Why |
|---|---|
| `--privileged --runtime nvidia` | GPU access for cuVSLAM (NITROS) |
| `--network host --ipc host` | DDS discovery + shared-memory transport across other ROS 2 nodes on the Jetson |
| `-v /dev:/dev -v /dev/bus/usb:/dev/bus/usb` | OAK-D hotplug + UART devices visible inside |
| `--device /dev/ttyTHS1:/dev/ttyTHS1` | Pixhawk UART passthrough (added only if the host has it) |
| `-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` | X11 forwarding so `rviz2` can render to the host display |
| `-e XRCE_DEV=... -e CAMERA_X/Y/Z=...` | Override the UART path or camera offset without editing scripts |

### Topics published / subscribed

| Direction | Topic | QoS | Source |
|---|---|---|---|
| pub | `/visual_slam/tracking/odometry` | RELIABLE / VOLATILE | cuVSLAM |
| pub | `/visual_slam/vis/landmarks_cloud` | RELIABLE / VOLATILE | cuVSLAM |
| pub | `/visual_slam/status` | RELIABLE / VOLATILE | cuVSLAM |
| pub | `/oak/left/image_raw`, `/oak/right/image_raw`, `/oak/imu/data` | BEST_EFFORT | depthai driver |
| pub | `/fmu/in/vehicle_visual_odometry` | **BEST_EFFORT / VOLATILE** | `px4_odom_bridge.py` — PX4 v1.14 expects this exact QoS |
| sub | `/fmu/out/timesync_status` | BEST_EFFORT / TRANSIENT_LOCAL | uXRCE-DDS Agent ← PX4 |

TF tree: `map → odom → base_link → oak` (the static `base_link → oak` is the camera offset).

### Camera offset (PX4 body frame)

`./run_pipeline.sh <x> <y> <z>` passes a static TF from `base_link` to the OAK-D's left camera focal point, in meters, FLU body axes:

- **Origin**: Pixhawk's **primary IMU** location, *not* CoG / GPS antenna / frame center. PX4 defines the body frame at the IMU; all sensor offsets must be measured from there or the lever-arm during rotation breaks EKF2 fusion.
- **Target**: left stereo lens focal point of the OAK-D-S2 (cuVSLAM uses the left camera as the optical center).
- **Axes (FLU)**: +x forward (direction of the Pixhawk's mounting arrow), +y left, +z up.

Worked example — OAK-D 10 cm in front of the Pixhawk, on centerline, 2 cm below:

```bash
./run_pipeline.sh 0.10 0.0 -0.02
```

If unsure of your Pixhawk's exact IMU location, the case center is within a few mm on most boards; check the board datasheet for the IMU stack offset.

### Debugging the container

```bash
docker logs -f skybeta-pipeline                  # combined stdout (tmux noise)
docker exec -it skybeta-pipeline bash            # new interactive shell
docker exec -it skybeta-pipeline tmux attach -t skybeta   # reattach if you detached
```

| Symptom | Cause | Fix |
|---|---|---|
| `AMENT_TRACE_SETUP_FILES: unbound variable` and pane dies | `set -u` + ROS 2 setup script that touches unset vars | Already wrapped with `set +u/-u` in all `container_scripts/*.sh` |
| `ERROR: container 'skybeta' already running` | `04_run_container.sh` left a container that would clash with `run_pipeline.sh` on the UART | `docker rm -f skybeta` |
| `nvcr.io/nvidia/isaac/ros:... not found` at build | NGC base hash tag not cached and no NGC login | `docker pull nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c` (or build `isaac_ros_common` once which pulls it) |
| `X_LINK_UNBOOTED` / `Skipping X_LINK_UNBOOTED device` | OAK-D enumerated *after* container start, hot-plug not seen | Plug OAK-D in **before** `run_pipeline.sh`, then verify with `lsusb \| grep 03e7` |
| RViz from inside the container shows blank / "cannot connect" | Host hasn't permitted X | On host: `xhost +local:root` |
| `uxrce_dds_client status` shows 0 subscribers | PX4 params not set, or wrong baud | Recheck `MAV_1_CONFIG=0`, `UXRCE_DDS_CFG=TELEM2`, `SER_TEL2_BAUD=921600`, reboot Pixhawk |
| `/fmu/in/vehicle_visual_odometry` publishing but EKF2 not fusing | QoS mismatch | Bridge already pins BEST_EFFORT / VOLATILE — if you wrote a replacement, match it exactly |

### Rebuilding after a code change

```bash
./03_build_image.sh        # layer cache hits everything that didn't change
```

- Edit `container_scripts/*.sh` → only layer 6 rebuilds (~1 s)
- Edit `src/oakd_vslam_bringup/` → layer 5 (colcon) rebuilds (~7 min on Orin Nano) + layers 6+ on top
- Change `XRCE_AGENT_TAG` or any apt line → all downstream layers rebuild (~20 min)

## PX4 / QGC parameters (Pixhawk side, manual)

Set in QGroundControl and reboot the Pixhawk:

```text
MAV_1_CONFIG    = 0
UXRCE_DDS_CFG   = TELEM2
SER_TEL2_BAUD   = 921600
EKF2_EV_CTRL    = bits 0+3 (horiz pos + yaw)   # add bit 1 for vert pos / bit 2 for vel later
EKF2_HGT_REF    = Baro                         # keep until vision altitude is trusted
EKF2_EV_DELAY   = 50 (ms)
EKF2_EVP_NOISE  = 0.1
EKF2_EVV_NOISE  = 0.1
EKF2_EVA_NOISE  = 0.05
```

## Fleet distribution

```bash
# On the build Jetson:
docker save skybeta:latest -o skybeta.tar
zstd -19 skybeta.tar -o skybeta.tar.zst       # ~6–8 GB compressed

# On a fresh Jetson (after ./01_prereqs.sh + ./02_uart_fix.sh + reboot):
zstd -d skybeta.tar.zst -o skybeta.tar
docker load -i skybeta.tar
./run_pipeline.sh 0.10 0.0 -0.02
```

The image is **aarch64-only** (NGC Isaac ROS base has no amd64 manifest) — by design, since cuVSLAM and NITROS only ship precompiled for Jetson.

## Repo layout

```
skybeta/
├── Dockerfile                      # NGC base + apt + agent + workspace
├── 01_prereqs.sh                   # host: apt + docker daemon + OAK-D udev
├── 02_uart_fix.sh                  # host: UART DMA patch + reboot
├── 02b_uart_fix_verify.sh          # host: post-reboot status check
├── 03_build_image.sh               # host: docker build
├── 04_run_container.sh             # host: docker run -> interactive bash
├── run_pipeline.sh                 # host: docker run -> tmux'd full stack
├── setup_and_build.sh              # host: 01 -> 02 -> 03 one-shot
├── jetson_pixhawk_uxrce_ros2_setup.md   # the canonical setup reference
├── jetson_uart_dma_fix.sh          # UART DMA / IOMMU patcher (host-only)
├── container_scripts/              # baked into /usr/local/bin in the image
│   ├── run_agent.sh                # MicroXRCEAgent serial
│   ├── run_slam.sh                 # ros2 launch oakd_vslam.launch.py
│   ├── run_bridge.sh               # ros2 launch px4_bridge.launch.py
│   ├── run_full_stack.sh           # tmux session of all three above
│   └── check_topics.sh             # verify topics
└── src/oakd_vslam_bringup/         # ROS package: cuVSLAM launch + bridge node
```

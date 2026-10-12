# skybeta competitor guide

You have a drone that already knows where it is. Your job is to decide where it should go.

The stack below you — OAK-D-S2 → cuVSLAM → PX4 EKF2 over uXRCE-DDS — publishes a
GPS-denied pose at ~30 Hz. Everything between your setpoints and the flight controller is
owned by `skybeta_flight_interface`, which you should not need to modify.

If you would rather write the localisation than consume it, you can **replace cuVSLAM with
your own SLAM or VIO** — the camera, the bridge and the flight interface all stay. That is
§7.

There is **always a safety pilot on RC**. They arm the vehicle, they select Offboard, and
they can take it back at any instant. Nothing you write may assume otherwise.

---

## 1. The contract

Three topics. That is the whole interface.

| Topic | Type | Direction | Frame / units |
|---|---|---|---|
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | **you subscribe** | ENU position + orientation in `odom`; body-frame FLU twist; metres, m/s, rad |
| `/skybeta/setpoint/pose` | `geometry_msgs/PoseStamped` | **you publish** | ENU position in `odom`, metres; orientation = commanded yaw |
| `/skybeta/setpoint/twist` | `geometry_msgs/TwistStamped` | **you publish** | ENU world velocity, m/s; `angular.z` = yaw rate about ENU up, rad/s |
| `/skybeta/state` | `std_msgs/String` (JSON) | **you subscribe** | what PX4 is doing — see §3 |

Two optional services:

| Service | Type | What it does |
|---|---|---|
| `/skybeta/hold` | `std_srvs/Trigger` | Freeze at the last position the interface considered safe. Always available. |
| `/skybeta/request_offboard` | `std_srvs/Trigger` | Ask PX4 for Offboard. **Refused unless `allow_mode_request:=true`. SITL only.** |

Do not publish to `/fmu/in/*` yourself. Two publishers on `offboard_control_mode` fight
over the control flags and PX4 sees whichever arrived last.

### Frames — read this twice

`odom` is **ENU**: `+x East, +y North, +z Up`.

...except cuVSLAM has no compass and no gravity vector (`enable_imu_fusion` is off, so this
is pure stereo VO). "East" and "North" actually mean *whatever direction the vehicle's x and
y axes pointed when the SLAM node started*. The origin is the rig's startup pose, so at
power-on the vehicle is at `(0, 0, 0)` and `z` is height above that point. **Do not assume
magnetic north anywhere in your code.**

PX4 is **NED**: `+x North, +y East, +z Down`. The flight interface does that conversion:

```
position:   (x_ned, y_ned, z_ned) = (y_enu, x_enu, -z_enu)     # world: permute x/y
yaw:        yaw_ned = 90 deg - yaw_enu                          # its own inverse
yaw rate:   yawspeed_ned = -yawspeed_enu                        # z flips
```

You never write those. If you find yourself negating a `z`, you are working in the wrong
frame.

**Yaw is optional.** Leave the setpoint's orientation quaternion all-zero and the interface
sends `yaw = NaN`, which tells PX4 to hold whatever heading it has. Set a real (normalised)
quaternion to command one.

**Pose setpoints persist; twist setpoints expire.** A position setpoint *is* a hold, so it
stays in effect until you replace it. A velocity setpoint is a rate command that is only
safe while someone is actively refreshing it, so it expires after `setpoint_timeout_s`
(default 1.0 s) and the interface falls back to holding the last safe position. If your node
dies mid-manoeuvre the vehicle hovers; it does not keep flying.

### The one frame caveat nobody can fix for you

PX4's local NED frame is EKF2's, whose origin and heading are **not** the same as the vision
`odom` frame's. With `pose_frame=FRD` on the vision message, EKF2 estimates that rotation
itself, and your ENU setpoint is rotated by the plain REP-103 map with no correction. In
practice the residual is a constant yaw offset and a constant translation.

Two parameters exist for this once you have measured it on the bench:

- `anchor_setpoints` (default `false`) — express setpoints as a displacement from the vision
  origin, re-anchored on the PX4 local position captured at the same instant. Fixes the
  translation.
- `odom_to_ned_yaw_offset_rad` (default `0.0`) — fixes the rotation.

Measure before you set either:

```bash
ros2 topic echo /visual_slam/tracking/odometry --field pose.pose.orientation
ros2 topic echo /fmu/out/vehicle_attitude --qos-reliability best_effort \
                                          --qos-durability transient_local
```

If the two yaws differ by a constant, that constant is your offset. Fly a hover with both at
zero first — you may not need them at all.

---

## 2. The safety envelope

Every setpoint is **clamped, never rejected**. Rejecting would leave PX4 holding whatever it
had last, which for a velocity setpoint means a vehicle still flying at the command that took
it out of bounds. Clamping keeps the stream continuous and pins the commanded state to the
boundary.

| Parameter | Default | Meaning |
|---|---|---|
| `geofence_radius_m` | `10.0` | Horizontal radius about the origin captured at the **first odometry sample** |
| `max_altitude_m` | `3.0` | Ceiling above that origin |
| `min_altitude_m` | `0.3` | Floor above that origin — stops you commanding into the ground |
| `max_speed_mps` | `1.5` | Cap on the **norm** of a twist setpoint, not per-axis |

Horizontal excursions are scaled back onto the fence along the same bearing, so your
commanded *direction* survives and only the distance changes. Altitude is clipped
independently. Each clamp logs a throttled WARN and shows up in `/skybeta/state`.

**A clamped setpoint you never notice looks exactly like a vehicle that will not reach its
waypoint.** If your mission hangs at a corner, check `clamped` in `/skybeta/state` before you
check anything else. The example node logs this for you.

The 5 m square in `competitor_example` reaches √2 × 5 = 7.07 m at its far corner, inside the
10 m default. Enlarge the square and you must enlarge the fence too — or it will silently
never arrive.

Tighten the envelope from the launch file; do not loosen it without asking:

```bash
ros2 launch competitor_example competitor.launch.py \
    geofence_radius_m:=6.0 max_altitude_m:=2.0 square_side_m:=3.0
```

---

## 3. `/skybeta/state`

A JSON string at 5 Hz. Parse it with `json.loads(msg.data)`.

```json
{
  "stamp": 1234.567,
  "px4_link": true,
  "armed": true,
  "nav_state": 14,
  "nav_state_name": "OFFBOARD",
  "offboard_active": true,
  "odom_enu":   [0.12, -0.04, 1.50],
  "origin_enu": [0.00,  0.00, 0.00],
  "local_enu":  [0.11, -0.05, 1.49],
  "geofence_ok": true,
  "setpoint_mode": "position",
  "clamped": [],
  "frozen": false,
  "envelope": {"geofence_radius_m": 10.0, "min_altitude_m": 0.3,
               "max_altitude_m": 3.0, "max_speed_mps": 1.5}
}
```

- `offboard_active` — **wait for this before you command anything.** It goes true when the
  pilot flips the switch, and false the instant they take over.
- `odom_enu` is cuVSLAM's position; `local_enu` is EKF2's own estimate reported back in ENU.
  They should track each other. If they diverge, the vision link is degrading.
- `geofence_ok` describes the **vehicle**, not the setpoint — so you can see it being carried
  out of bounds by momentum or by the pilot. It covers the radius and the ceiling only:
  a vehicle on the pad, or being landed, is legitimately below `min_altitude_m` and is not
  reported as a breach.
- `clamped` lists the reasons the last setpoint was modified: `geofence_radius`,
  `max_altitude`, `min_altitude`, `max_speed`.

---

## 4. RC and arming — the procedure

**The flight interface never arms and never changes mode.** There is no arm service. This is
deliberate and not negotiable.

Before every flight:

1. Props off for the first run of any new code. Bench it.
2. Safety pilot has the transmitter, is standing clear, and has confirmed the kill switch.
3. Start the stack (§5) and confirm on the ground station:
   - `/skybeta/state` shows `px4_link: true`,
   - `/visual_slam/tracking/odometry` is publishing and `odom_enu` moves sensibly when you
     carry the vehicle,
   - your node is publishing `/skybeta/setpoint/pose` **before** the pilot switches mode.
4. **Pilot arms** — from the transmitter, in a manual/position mode.
5. Pilot takes off manually and establishes a stable hover at ~1.5 m.
6. **Pilot flips the RC mode switch to Offboard.** `offboard_active` goes true and your node
   takes over.
7. Pilot flips out of Offboard to take the vehicle back at any time. Your node must notice
   and stop advancing its mission (the example does).

PX4 will refuse to *enter* Offboard unless the `OffboardControlMode` stream is already
running above ~2 Hz, and will drop *out* of it if the stream stalls (`COM_OF_LOSS_T`, then
`COM_OBL_RC_ACT`). The flight interface streams at 10 Hz from the moment it starts, before
any setpoint of yours exists, so this is handled — but it is why you must start the interface
*before* the pilot reaches for the switch.

`/skybeta/request_offboard` exists for SITL, where there is no pilot. It is refused unless
`allow_mode_request:=true`, and even then it only asks — it never arms.

---

## 5. Running it

The stack launcher opens a tmux window named `competitor`:

```bash
host/run.sh stack            # windows: agent, slam, bridge, competitor
```

That window runs `ros2 launch competitor_example competitor.launch.py`, which starts
`flight_interface` **and** the example node. Point it at your own package instead:

```bash
COMPETITOR_PKG=my_team_nav COMPETITOR_LAUNCH=my_mission.launch.py host/run.sh stack
```

Replacing cuVSLAM with your own SLAM instead of building on top of it? That is §7, and
`run_full_stack.sh --no-slam` is the launcher for it.

Or work by hand in a shell on the running container:

```bash
host/run.sh shell
ros2 launch competitor_example competitor.launch.py competitor:=false   # interface only
ros2 run my_team_nav my_node
```

### Writing your own

Fork `src/competitor_example/competitor_example/square_mission.py`. It is heavily commented
and does nothing you will not need: wait for Offboard, capture a start pose, publish ENU
setpoints on a timer, check arrival with `math.dist`, advance a state machine, notice when
Offboard is lost.

Keep `flight_interface` in your launch file exactly as `competitor.launch.py` has it.

`colcon build --symlink-install --packages-select my_team_nav` inside the container; a Python
package with `--symlink-install` picks up edits on the next `ros2 run` with no rebuild.

### Recording a bag

Do this for every flight. It is the only way to explain a bad one afterwards.

```bash
host/run.sh bag                        # writes mcap into ./bags on the host
host/run.sh bag --duration 60          # or a fixed-length one
```

The default set is deliberately **light**: odometry, tracking status, camera calibration,
the IMU, every `/fmu/*` and `/skybeta/*` topic on the graph — and **no camera images**.
Images are behind a flag because they are not free:

```bash
host/run.sh bag --images=mono          # the mono8 pair, ~1/3 the bytes
host/run.sh bag --images=raw           # the bgr8 pair -- ~36 MB/s
host/run.sh bag --all                  # everything, images included
```

Recording `/oak/left/image_raw` alone wrote **1.1 GB in 30 s** on this rig and dragged the
camera to 19.6 Hz and cuVSLAM odometry to **11.3 Hz** while it ran. Any rate you measure
from such a bag is a rate under recording load, not the rate your stack normally achieves.
`record_bag.sh` prints a warning whenever images are included; believe it.

Record from a *second* container (`host/run.sh bag` does), not from the stack container.

---

## 6. Debugging

| Symptom | First thing to check |
|---|---|
| Nothing moves when the pilot selects Offboard | `/skybeta/state` → `px4_link`. If false, the uXRCE agent is not talking to the FC. |
| Vehicle will not enter Offboard at all | Is `flight_interface` running? `ros2 topic hz /fmu/in/offboard_control_mode` must show ~10 Hz. |
| Mission hangs at a waypoint | `clamped` in `/skybeta/state`. You are asking for something outside the envelope. |
| Position drifts / orbits | cuVSLAM tracking. `ros2 topic echo /visual_slam/status`; a `vo_state` of 2 is lost tracking. |
| `odom_enu` and `local_enu` diverge | The EV link into EKF2 is dropping. Check the TELEM2 cable seating before anything else. |
| Setpoints ignored after your node pauses | You were sending twists. They expire after 1 s by design. Send poses, or keep the twist stream up. |

Useful one-liners:

```bash
ros2 topic hz /fmu/in/offboard_control_mode          # must be ~10 Hz
ros2 topic hz /visual_slam/tracking/odometry         # must be ~30 Hz
ros2 topic echo /skybeta/state --once
ros2 topic echo /fmu/in/trajectory_setpoint          # what PX4 actually got
ros2 param get /flight_interface geofence_radius_m
ros2 service call /skybeta/hold std_srvs/srv/Trigger # panic button
```

The `/fmu/out/vehicle_status` topic is spelled `/fmu/out/vehicle_status_v1` on PX4 v1.16 —
message versioning added a `_v<N>` suffix. `flight_interface` resolves this from the
installed `px4_msgs` at startup and logs the map it chose; do not hard-code either spelling.

---

## 7. Bring your own SLAM

Nothing in this stack forces you to use cuVSLAM. The competition explicitly allows you to
replace it with your own SLAM or VIO, and the swap point is the **PX4 odometry bridge**,
which is parameterised rather than hard-wired:

```bash
ros2 launch oakd_vslam_bringup px4_bridge.launch.py --show-args
```

| Launch argument | Default | What it is for |
|---|---|---|
| `input_topic` | `/visual_slam/tracking/odometry` | **the swap point** — point it at your tracker |
| `output_topic` | `/fmu/in/vehicle_visual_odometry` | leave it alone |
| `status_topic` | `/visual_slam/status` | tracking loss → `reset_counter`; see §7.4 |
| `pose_frame` | `FRD` | see §7.5. Leave it alone |
| `publish_velocity` | `true` | `false` if your twist is not trustworthy |
| `min_position_variance` | `0.0` | m², a floor on the published position variance |

Publish a `nav_msgs/Odometry` that satisfies §7.3, point `input_topic` at it, and
everything downstream — the ENU↔NED conversion, `reset_counter`, the PX4 QoS, the
`skybeta_flight_interface` envelope, `/skybeta/state` — keeps working unchanged.

### 7.1 What you get as input

Measured on the bench with the full stack running, 44 s, from header stamps — not
estimates:

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/oak/left/image_raw`, `/oak/right/image_raw` | `sensor_msgs/Image` | **30.000 Hz** | **1024×640 `bgr8`**, p99 gap 33.33 ms |
| `/oak/left/image_mono`, `/oak/right/image_mono` | `sensor_msgs/Image` | 30.000 Hz | **`mono8`**, converted on the GPU by `isaac_ros_image_proc` |
| `/oak/left/camera_info`, `/oak/right/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | intrinsics + distortion; **not** the baseline — see §7.2 |
| `/oak/imu/data` | `sensor_msgs/Imu` | **197.9 Hz** | BNO086, interval p50 5.02 ms, no fused quaternion |

**The images are `bgr8` even though the sensors are monochrome.** `CAM_B`/`CAM_C` are
OV9282 mono parts, but the depthai v3 driver publishes those sockets as colour and exposes
no encoding parameter, so the three channels are identical and a host-side conversion is
unavoidable. Either subscribe to `image_mono` or take any one channel yourself — the
converted `mono8` is bit-exact with each `bgr8` channel.

**`image_mono` only exists while cuVSLAM does.** The two converters are composable nodes
inside cuVSLAM's container, so in `--no-slam` mode (§7.4) they are not running. If you
want mono8 there, convert from `image_raw` in your own node.

**Do not raise the resolution.** 1024×640 is not a conservative default, it is the measured
limit. The MyriadX's XLink transport saturates near **143 MB/s** regardless of the USB link
rate, and the OAK-D-S2 *is* on USB 3.0 SuperSpeed:

| Config | Rate | |
|---|---|---|
| 1280×800 ×2 @30 | **23.3 Hz** | at the ceiling; 23 % of frames lost |
| 1280×800 left only @30 | 30.0 Hz | proves it is throughput, not the sensor |
| **1024×640 ×2 @30** | **30.0 Hz** | 118 MB/s — **what ships** |
| 960×600 ×2 @30 | 30.0 Hz | 104 MB/s, more margin |

At 1280×800 the two eyes drop *different* frames: L/R header-stamp sync p95 rises to
33.3 ms — one whole frame — few stereo pairs match at all, and cuVSLAM's own odometry
collapsed to 16.4 Hz. If you need headroom, go **down** to 960×600 in
[`../src/oakd_vslam_bringup/config/oakd_vslam.yaml`](../src/oakd_vslam_bringup/config/oakd_vslam.yaml),
never up. Re-measure if you touch it.

### 7.2 Extrinsics — read the baseline from TF, not from `CameraInfo`

**This is the one that silently ruins a stereo front end.** cuVSLAM takes the stereo
baseline from the **TF tree**, not from `CameraInfo`'s `P[3]`, and so should you. The
static transforms published by the launch file *are* the rig geometry as far as the
tracker is concerned. `i_reverse_stereo_socket_order` in the YAML is **not** the lever for
a baseline sign error — the whole `stereo:` block it lived in was inert and has been
deleted. The TF is the lever.

```bash
# the stereo baseline, as the tracker sees it
ros2 run tf2_ros tf2_echo oak_left_camera_optical_frame oak_right_camera_optical_frame

# where the camera sits on the airframe (FLU body frame)
ros2 run tf2_ros tf2_echo base_link oak_left_camera_optical_frame
ros2 run tf2_ros tf2_echo base_link oak_parent_frame
```

The chain is `base_link → oak_parent_frame → oak_*_camera_optical_frame`. The first link is
a static transform driven by `host/run.sh stack <x> <y> <z> [roll pitch yaw]`; the rest
comes from depthai's URDF and the on-device calibration. **Measure your actual mount and
pass it.** The default is zero, and an unmodelled mount rotation couples vehicle rotation
into apparent translation — which orbits. Set `EKF2_EV_POS_X/Y/Z` on the flight controller
to `0` and leave them there: this TF already carries the lever arm, and setting both
applies it twice.

### 7.3 The output contract

Publish `nav_msgs/Odometry` on a topic of your choosing. The bridge is the only consumer,
and it expects exactly this:

| Field | What goes in it |
|---|---|
| `header.stamp` | when the **image** was captured, not when you finished computing. It lands in the `EKF2_EV_DELAY` budget |
| `header.frame_id` | your odometry frame (cuVSLAM uses `odom`) |
| `child_frame_id` | the body frame — `base_link` |
| `pose.pose.position` | **ENU**, metres: `+x East, +y North, +z Up` |
| `pose.pose.orientation` | **ENU**, normalised quaternion, body→odom |
| `twist.twist.linear` | **body frame, FLU**, m/s — *not* the odom frame |
| `twist.twist.angular` | **body frame, FLU**, rad/s |
| `pose.covariance` | row-major 6×6, order `[x y z roll pitch yaw]` — see §7.6 |
| `twist.covariance` | row-major 6×6, order `[vx vy vz wx wy wz]` |

Aim for **30 Hz**. The rest of the stack is built around it.

**The twist really is body-frame, and that is not a convention we invented.** cuVSLAM's
`pose_cache.cpp` computes `dp = poses.front().inverse() * poses.back()`, i.e.
`T_body(t0)_body(t1)`, whose translation is in the body frame. The bridge relies on that to
emit `VELOCITY_FRAME_BODY_FRD`. If your tracker reports velocity in the world frame,
rotate it into the body frame before publishing, or set `publish_velocity:=false` and let
EKF2 do without it.

**Do not pre-convert to NED or FRD.** The bridge does all of it:

```
position:     (x_ned, y_ned, z_ned) = (y_enu, x_enu, -z_enu)      # ENU -> NED
orientation:  the same rotation, applied to the quaternion
velocity:     (vx, -vy, -vz)                                      # FLU -> FRD, body
```

If you hand it NED already, it will rotate it a second time. Units are metres, m/s and
radians throughout; nothing in this chain takes degrees.

### 7.4 Wiring it in

Just the bridge, against your own topic:

```bash
ros2 launch oakd_vslam_bringup px4_bridge.launch.py input_topic:=/my_vio/odometry
```

The whole stack with cuVSLAM replaced by yours — agent, camera driver, camera TF, bridge
and competitor windows, no cuVSLAM:

```bash
host/run.sh shell                                          # one container, UART attached
BRIDGE_INPUT_TOPIC=/my_vio/odometry run_full_stack.sh --no-slam
```

`--no-slam` (or `SKYBETA_NO_SLAM=1`) runs the depthai driver from the repo's own params
file and publishes the `base_link → oak_parent_frame` static TF, so the TF tree and every
camera topic are exactly as in §7.1 — minus `/oak/*/image_mono` and all of
`/visual_slam/*`. Start your tracker in a new tmux window (`Ctrl-b c`) or a second
`host/run.sh shell`.

> `host/run.sh stack` forwards only the `CAMERA_*` variables into the container, so
> `SKYBETA_NO_SLAM` and `BRIDGE_INPUT_TOPIC` have to be set **inside** it — hence
> `host/run.sh shell` above. Same for `COMPETITOR_PKG`.

**Tracking loss, and why the bridge wants to know.** When a tracker loses tracking and
re-initialises, its pose jumps. If PX4 is not told, EKF2 integrates that jump as real
motion — metres of phantom translation in one 30 Hz tick. The bridge handles this by
watching a status topic and incrementing a wrapping `reset_counter` on every
non-tracking → tracking transition.

If your SLAM can signal tracking loss, publish
`isaac_ros_visual_slam_interfaces/VisualSlamStatus` (`vo_state`: `0` unknown, `1` success,
`2` failed) and point the bridge at it:

```bash
ros2 launch oakd_vslam_bringup px4_bridge.launch.py \
    input_topic:=/my_vio/odometry status_topic:=/my_vio/status
```

If it cannot, you can leave `status_topic` at its default: with cuVSLAM not running nothing
publishes `/visual_slam/status`, the subscription sits idle and `reset_counter` stays `0`.
To switch it off explicitly you must run the node directly, because `ros2 launch` on Humble
rejects an empty argument value (`status_topic:=` is a "malformed launch argument"):

```bash
ros2 run oakd_vslam_bringup px4_odom_bridge --ros-args \
    -p input_topic:=/my_vio/odometry -p status_topic:=''
```

Either way the bridge logs which it chose (`status=/my_vio/status` or `status=unavailable`)
on the line it prints at startup. **`reset_counter` stuck at 0 is a real hazard, not a
formality** — it is [`debugging.md`](debugging.md) §4.2 suspect #3.

### 7.5 `pose_frame` — leave it at `FRD`

`pose_frame` defaults to `FRD` and you almost certainly want to keep it there.

```
uint8 POSE_FRAME_NED = 1 # NED earth-fixed frame
uint8 POSE_FRAME_FRD = 2 # FRD world-fixed frame, arbitrary heading reference
```

A stereo VIO with no magnetometer and no gravity observation initialises its origin at the
rig's **startup pose**: its x axis points wherever `base_link` pointed when your node came
up, not north. That is the literal definition of `POSE_FRAME_FRD` — a valid local frame
with an arbitrary heading reference. Declaring `NED` tells EKF2 the measurements are
north-referenced when they are offset by a constant, so the position controller pushes at a
fixed angle to the actual error and the vehicle orbits instead of converging. **That is the
toilet-bowl mechanism**, and it is the first thing to check if you see one.

Use `NED` only if your frame is *genuinely* north-aligned — you fuse a magnetometer, or you
initialise the yaw from the FC's attitude. Declare FRD and let EKF2 estimate the rotation,
**or** fuse EV yaw so the estimator adopts your heading. Do not do a partial version of
both.

```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry --field pose_frame   # must be 2
```

### 7.6 Covariance

Fill `pose.covariance` with an honest 6×6, row-major, ordered `[x y z roll pitch yaw]`. The
bridge takes the diagonal and maps it: position gets the ENU→NED x/y swap
(`[pc[7], pc[0], pc[14]]`), orientation and velocity do not (FLU→FRD is sign flips only,
and signs square out of a variance).

Then temper your expectations. With `EKF2_EV_NOISE_MD = 0` — the shipped setting — PX4
treats `EKF2_EVP_NOISE` / `EKF2_EVV_NOISE` / `EKF2_EVA_NOISE` as a **lower bound** on the
variance in the message. cuVSLAM's own numbers are five to seven orders below
`EKF2_EVP_NOISE = 0.1` (median σx ≈ 78 µm), so EKF2 clamps all six components and the
message's values never reach the estimator. Yours will very likely be clamped too. Set
`EKF2_EV_NOISE_MD = 1` if you want your covariance used directly, and only if you trust it.

Do not publish an all-zero or identity covariance and hope. The bridge warns after 30
consecutive identity-covariance messages (one full second), which is long enough to ignore
a startup transient and short enough to catch a tracker that never fills the field.
`min_position_variance:=<m^2>` is a stopgap floor if you need one.

### 7.7 Checklist before you fly your own SLAM

Props off, on the bench, in this order. Every step has a number you can read.

1. **Odometry rate.** Your topic is not in `record_bag.sh`'s default set, so name it
   explicitly — everything after `--` is handed to `ros2 bag record`:
   ```bash
   record_bag.sh --duration 60 -- /my_vio/odometry
   ```
   Then measure header-stamp deltas offline — **not** `ros2 topic hz`, which under-reports
   this stack by ~10 % ([`debugging.md`](debugging.md) §0). You want **≥ 25 Hz** sustained
   with no gap over ~100 ms.
2. **The bridge is actually publishing.**
   `ros2 topic hz /fmu/in/vehicle_visual_odometry` should track your odometry rate.
3. **`pose_frame` is 2.**
   `ros2 topic echo /fmu/in/vehicle_visual_odometry --field pose_frame --once`.
4. **Scale.** Pick the vehicle up, translate it **1.0 m** along one axis, put it down.
   `ros2 topic echo /my_vio/odometry --field pose.pose.position` must report 1.0 m, on the
   axis you moved, with the right sign. A scale error here is a stereo-baseline error, so
   go back to §7.2.
5. **Heading.** Yaw the airframe ~90° by hand and compare your yaw against the FC's:
   ```bash
   ros2 topic echo /my_vio/odometry --field pose.pose.orientation
   ros2 topic echo /fmu/out/vehicle_attitude --qos-reliability best_effort \
                                             --qos-durability transient_local
   ```
   The two should change by the same amount. If they differ by a *constant* equal to the
   heading at startup, your frame is startup-aligned — which is correct, and is exactly why
   `pose_frame` is `FRD`.
6. **Tracking loss is reported.** Cover the lenses until your tracker gives up, uncover,
   and confirm `reset_counter` increments:
   `ros2 topic echo /fmu/in/vehicle_visual_odometry --field reset_counter`.
7. **Covariance is not identity.** `ros2 topic echo /my_vio/odometry --field pose.covariance --once`
   and check the bridge's log is free of the identity-covariance warning.
8. Only then §4, with the safety pilot, props on, at 1.5 m.

---

## 8. The advanced track — `competitor_advanced_cpp`

`skybeta_flight_interface` drives PX4's built-in Offboard mode. The alternative is to
register **your own flight mode** with PX4 using `px4_ros2_cpp`, which is what
`src/competitor_advanced_cpp` demonstrates: a `px4_ros2::ModeBase` subclass called "Skybeta
Goto" that flies to a parameterised NED target using `GotoSetpointType`.

Once registered it appears in QGroundControl's mode list and can be bound to an RC mode
switch position exactly like Position or Altitude, and PX4's commander runs its arming checks
against it. The pilot selects your mode *by name*.

```bash
ros2 run competitor_advanced_cpp skybeta_goto_mode --ros-args \
    -p target_north_m:=5.0 -p target_east_m:=0.0 -p target_down_m:=-1.5
```

**Take the extra risk seriously:**

- **No safety envelope.** The Python flight interface is not in this loop at all. There is no
  geofence, no altitude clamp, no speed cap — whatever `updateSetpoint()` publishes goes
  to the controller. You own the envelope yourself.
- **NED, not ENU. `+z` is DOWN.** `target_down_m: -1.5` means 1.5 m *up*. A sign error here
  puts the vehicle in the ground.
- **It reads EKF2's estimate, not cuVSLAM's odometry.** GPS-denied that is only as good as
  the vision link feeding it, and it degrades silently when the link drops.
- **Registration is a live handshake.** The node throws at construction if the agent is not
  running or `px4_msgs` does not match the firmware. It is C++, so a mistake is a segfault
  rather than a traceback.

`relative_to_activation` (default true) anchors the target to wherever the pilot handed the
vehicle over, rather than to the EKF2 origin, which may be anywhere. Leave it true.

Fly the Python track first. Only move to this one if you actually need a named mode on the
switch.

---

## 9. Package map

```
src/
├── skybeta_flight_interface/          do not modify
│   ├── flight_interface.py            the 10 Hz heartbeat + the node
│   ├── frames.py                      ENU<->NED / FLU<->FRD math
│   ├── envelope.py                    the geofence / altitude / speed clamp
│   ├── topics.py                      /fmu topic names vs px4_msgs MESSAGE_VERSION
│   └── test/                          pytest; integration_offboard.py needs a DDS peer
├── competitor_example/                COPY THIS
│   ├── square_mission.py              hover -> 5 m square -> hold
│   └── launch/competitor.launch.py    flight_interface + your node
├── competitor_advanced_cpp/           the advanced track (§7)
└── oakd_vslam_bringup/                camera + cuVSLAM + the PX4 odometry bridge
```

Run the unit tests any time you touch the interface:

```bash
cd /root/skybeta_ws/src/skybeta_flight_interface && python3 -m pytest -q test/
```

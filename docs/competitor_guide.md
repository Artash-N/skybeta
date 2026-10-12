# skybeta competitor guide

You have a drone that already knows where it is. Your job is to decide where it should go.

The stack below you — OAK-D-S2 → cuVSLAM → PX4 EKF2 over uXRCE-DDS — publishes a
GPS-denied pose at ~30 Hz. Everything between your setpoints and the flight controller is
owned by `skybeta_flight_interface`, which you should not need to modify.

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

That window builds your team workspace (below), then runs `ros2 launch competitor_example
competitor.launch.py`, which starts `flight_interface` **and** the example node. Point it at
your own package by setting `COMPETITOR_PKG` / `COMPETITOR_LAUNCH` in `/etc/skybeta/drone.env`,
or for one run:

```bash
COMPETITOR_PKG=my_team_nav COMPETITOR_LAUNCH=my_mission.launch.py host/run.sh stack
```

Or work by hand in a shell on the running container (`host/run.sh shell` attaches to the
running stack if there is one):

```bash
host/run.sh shell
ros2 launch competitor_example competitor.launch.py competitor:=false   # interface only
ros2 run my_team_nav my_node
```

### Writing your own

**Your code lives in `~/team_ws/src` on the drone** — the team workspace. `host/run.sh`
bind-mounts it at `/root/team_ws` in every container, so it survives the container exiting
(every container here is `--rm`) and survives the organisers updating the skybeta image. Code
anywhere else inside a container — including `/root/skybeta_ws/src` — is gone the moment the
container stops.

Start from the example:

```bash
# on the drone (host shell)
cp -r ~/skybeta/src/competitor_example ~/team_ws/src/my_team_nav
# ...rename the package in package.xml / setup.py / setup.cfg / resource/, then:
```

Fork `square_mission.py` inside it. It is heavily commented and does nothing you will not
need: wait for Offboard, capture a start pose, publish ENU setpoints on a timer, check arrival
with `math.dist`, advance a state machine, notice when Offboard is lost.

Keep `flight_interface` in your launch file exactly as `competitor.launch.py` has it.

Build inside the container — the stack's `competitor` window does this automatically on
start:

```bash
team_build.sh                                  # whole workspace
team_build.sh --packages-select my_team_nav    # any colcon arguments
```

It builds `/root/team_ws` as an overlay on the skybeta workspace and hands the files back to
your host user. A Python package built with `--symlink-install` (the default here) picks up
edits on the next `ros2 run` with no rebuild. New shells source the overlay automatically.

**Do not put `px4_msgs` in your workspace.** PX4's own tutorials tell you to clone it; here
the image already carries the generation that matches the flight controller (`release/1.16`
for v1.16.2), and a different one connects, lists every topic, and moves no data.
`team_build.sh` warns about any package that shadows one the image already has. The VIO → FC
pipeline (agent, cuVSLAM, bridge) never loads your overlay, so a mistake here can break your
code but not the drone's state estimate.

#### Extra system or Python packages

Never `apt install` or `pip install` in a running container — it vanishes on exit. Put them
in a team image instead, layered on the pinned skybeta base:

```bash
host/run.sh build --init     # writes ~/team_ws/Dockerfile + .dockerignore
# edit ~/team_ws/Dockerfile: RUN apt-get install ... / RUN pip3 install ...
host/run.sh build            # builds skybeta-team:<VERSION>
host/run.sh stack            # uses the team image automatically from now on
```

When the organisers roll out a new skybeta version, run `host/run.sh build` again: the team
image is tagged per base version, and the stack falls back to the plain base (with a warning)
until you rebuild.

#### Your drone's DDS domain

Each drone has its own `ROS_DOMAIN_ID`, set in `/etc/skybeta/drone.env`, and by default its DDS
traffic never leaves the Jetson. Everything started through `host/run.sh` gets both
automatically. To use rviz2 or `ros2 topic echo` from a laptop, set
`SKYBETA_DDS_SCOPE=network` in `drone.env`, restart the stack, and export the same
`ROS_DOMAIN_ID` on the laptop. Switch it back before flying: in `network` scope, the camera
streams go over the WiFi.

### Recording a bag

Do this for every flight. It is the only way to explain a bad one afterwards.

```bash
host/run.sh bag                        # writes mcap into ./bags on the host
```

Or by hand from a container shell:

```bash
ros2 bag record -s mcap -o /bags/$(date +%Y%m%d-%H%M%S) \
  /oak/left/image_raw /oak/right/image_raw /oak/imu/data \
  /visual_slam/tracking/odometry /visual_slam/status \
  /skybeta/setpoint/pose /skybeta/setpoint/twist /skybeta/state \
  /fmu/in/vehicle_visual_odometry /fmu/in/trajectory_setpoint \
  /fmu/in/offboard_control_mode /fmu/out/vehicle_local_position
```

Record from a *second* container (`host/run.sh bag` does), not from the stack container —
recording competes for CPU with cuVSLAM, and the camera topics are the expensive ones.

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

## 7. The advanced track — `competitor_advanced_cpp`

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

## 8. Package map

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

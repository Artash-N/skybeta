#!/usr/bin/env bash
# Inside the skybeta container -- start the full stack in a tmux session named
# 'skybeta'. Attaches at the end; Ctrl-b d to detach, `tmux kill-session -t
# skybeta` (or just exit the container) to tear everything down.
#
#   run_full_stack.sh              agent, slam, bridge, competitor  (default)
#   run_full_stack.sh --no-slam    agent, camera, cam_tf, bridge, competitor
#
# --no-slam (equivalently SKYBETA_NO_SLAM=1) is the "bring your own SLAM" mode.
# Everything starts except cuVSLAM: the OAK-D driver runs from the repo's own
# params file and the base_link -> oak_parent_frame static TF is published, so
# /oak/{left,right}/image_raw, /oak/{left,right}/camera_info and /oak/imu/data
# are there for your tracker, and the TF tree carries the rig geometry that a
# stereo algorithm needs for the baseline.
#
# What you do NOT get in --no-slam: /visual_slam/* (no cuVSLAM) and
# /oak/{left,right}/image_mono (the GPU bgr8->mono8 converters are composable
# nodes inside cuVSLAM's container, so they go with it). Convert host-side if
# you need mono8 -- the sensors are monochrome, so all three bgr8 channels are
# identical and any one of them is the mono image.
#
# Point the bridge at your own odometry with BRIDGE_INPUT_TOPIC. See
# docs/competitor_guide.md, "Bring your own SLAM", for the output contract.
#
# Environment:
#   SESSION                tmux session name           (skybeta)
#   SKYBETA_NO_SLAM        1 = skip cuVSLAM            (0)
#   SLAM_DELAY_S           agent -> slam gap in s      (3)
#   BRIDGE_DELAY_S         slam -> bridge gap in s     (8)
#   COMPETITOR_DELAY_S     bridge -> competitor gap    (2)
#   CAMERA_X/Y/Z           camera translation, m FLU   (0.0)
#   CAMERA_ROLL/PITCH/YAW  camera rotation, rad FLU    (0.0)
#   BRIDGE_INPUT_TOPIC     odometry the bridge reads   (launch default)
#   BRIDGE_STATUS_TOPIC    tracking-status topic       (launch default)
#   COMPETITOR_PKG         competitor package          (competitor_example)
#   COMPETITOR_LAUNCH      competitor launch file      (competitor.launch.py)
#   XRCE_DEV / XRCE_BAUD   agent transport             (/dev/ttyTHS1, 921600)
set -euo pipefail

# Print the comment header above, whatever length it grows to.
usage() { sed -n '2,/^set -euo/p' "${BASH_SOURCE[0]}" | grep '^#' | sed 's/^# \{0,1\}//'; }

NO_SLAM=0
case "${SKYBETA_NO_SLAM:-0}" in 1|y|yes|true|TRUE|on) NO_SLAM=1 ;; esac

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-slam) NO_SLAM=1; shift ;;
    --slam)    NO_SLAM=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "run_full_stack.sh: unknown argument '$1'" >&2; usage >&2; exit 2 ;;
  esac
done

SESSION="${SESSION:-skybeta}"
SLAM_DELAY_S="${SLAM_DELAY_S:-3}"           # let agent bind UART before SLAM starts
BRIDGE_DELAY_S="${BRIDGE_DELAY_S:-8}"       # let cuVSLAM init before bridge subscribes
COMPETITOR_DELAY_S="${COMPETITOR_DELAY_S:-2}"

CAMERA_X="${CAMERA_X:-0.0}"
CAMERA_Y="${CAMERA_Y:-0.0}"
CAMERA_Z="${CAMERA_Z:-0.0}"
CAMERA_ROLL="${CAMERA_ROLL:-0.0}"
CAMERA_PITCH="${CAMERA_PITCH:-0.0}"
CAMERA_YAW="${CAMERA_YAW:-0.0}"

COMPETITOR_PKG="${COMPETITOR_PKG:-competitor_example}"
COMPETITOR_LAUNCH="${COMPETITOR_LAUNCH:-competitor.launch.py}"

# --- the bridge --------------------------------------------------------------
# run_bridge.sh takes no arguments, so a retargeted bridge goes straight to the
# launch file. px4_bridge.launch.py declares input_topic, output_topic,
# status_topic, pose_frame, publish_velocity and min_position_variance
# (`ros2 launch oakd_vslam_bringup px4_bridge.launch.py --show-args`).
#
# NB: ros2launch in Humble REJECTS an empty value -- `status_topic:=` is a
# "malformed launch argument". To disable reset_counter tracking outright, run
# the node directly instead:
#   ros2 run oakd_vslam_bringup px4_odom_bridge --ros-args -p status_topic:=''
# In --no-slam mode you usually do not need to: nothing publishes
# /visual_slam/status when cuVSLAM is not running, so the subscription simply
# stays idle and reset_counter stays 0.
BRIDGE_ARGS=()
if [[ -n "${BRIDGE_INPUT_TOPIC:-}"  ]]; then BRIDGE_ARGS+=("input_topic:=${BRIDGE_INPUT_TOPIC}"); fi
if [[ -n "${BRIDGE_STATUS_TOPIC:-}" ]]; then BRIDGE_ARGS+=("status_topic:=${BRIDGE_STATUS_TOPIC}"); fi

if [[ ${#BRIDGE_ARGS[@]} -gt 0 ]]; then
  BRIDGE_CMD="exec ros2 launch oakd_vslam_bringup px4_bridge.launch.py ${BRIDGE_ARGS[*]}"
else
  BRIDGE_CMD="exec run_bridge.sh"
fi
BRIDGE_DESC="${BRIDGE_INPUT_TOPIC:-/visual_slam/tracking/odometry} -> /fmu/in/vehicle_visual_odometry"

# --- camera-only bringup (--no-slam) -----------------------------------------
# Mirrors oakd_vslam.launch.py minus the composable container: the same depthai
# v3 entry point (driver.launch.py -- v2's camera.launch.py is deprecated), the
# same params file, and the same base_link -> oak_parent_frame static TF. The
# rest of the camera TF tree comes from depthai's URDF, which driver.launch.py
# roots at oak_parent_frame, so the tree is identical to SLAM mode.
#
# The $(...) below is deliberately single-quoted here: it is typed into the
# tmux pane and evaluated there, where ROS is on PATH.
OAKD_PARAMS='$(ros2 pkg prefix --share oakd_vslam_bringup)/config/oakd_vslam.yaml'
CAMERA_CMD="exec ros2 launch depthai_ros_driver_v3 driver.launch.py name:=oak params_file:=${OAKD_PARAMS}"
CAM_TF_CMD="exec ros2 run tf2_ros static_transform_publisher \
--x ${CAMERA_X} --y ${CAMERA_Y} --z ${CAMERA_Z} \
--roll ${CAMERA_ROLL} --pitch ${CAMERA_PITCH} --yaw ${CAMERA_YAW} \
--frame-id base_link --child-frame-id oak_parent_frame"

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "[stack] session '${SESSION}' already running -- attaching"
  exec tmux attach -t "${SESSION}"
fi

# remain-on-exit keeps dead panes visible with their exit code instead of
# silently disappearing -- failures are loud, not silent.
#
# It is a WINDOW option, so `set-option -t <session>` only ever reached the
# session's *current* window and the later windows died silently (bug #6).
# `-g` sets the global window option, which every window inherits.
#
# It is set in the same invocation that starts the server: a bare
# `tmux start-server` does not survive on its own (exit-empty defaults to on,
# so a server with no session exits immediately and the next tmux command
# fails). The first window is an idle shell until the send-keys below, and
# remain-on-exit is read when a pane's process exits, not when it is created,
# so setting it here covers window 0 as well.
tmux new-session -d -s "${SESSION}" -n agent \; set-option -g remain-on-exit on
tmux send-keys   -t "${SESSION}:agent" 'exec run_agent.sh' C-m
sleep "${SLAM_DELAY_S}"

# Banner lines are collected as the windows are created so the numbering in the
# summary can never drift from the numbering tmux actually used.
BANNER=("  0 agent       MicroXRCEAgent ${XRCE_TRANSPORT:-serial} ${XRCE_DEV:-/dev/ttyTHS1} @ ${XRCE_BAUD:-921600}")

if [[ ${NO_SLAM} -eq 1 ]]; then
  tmux new-window -t "${SESSION}" -n camera
  tmux send-keys  -t "${SESSION}:camera" "${CAMERA_CMD}" C-m
  BANNER+=("  1 camera      OAK-D driver only -- cuVSLAM is NOT running (--no-slam)")

  tmux new-window -t "${SESSION}" -n cam_tf
  tmux send-keys  -t "${SESSION}:cam_tf" "${CAM_TF_CMD}" C-m
  BANNER+=("  2 cam_tf      base_link -> oak_parent_frame; FLU xyz=${CAMERA_X} ${CAMERA_Y} ${CAMERA_Z} m  rpy=${CAMERA_ROLL} ${CAMERA_PITCH} ${CAMERA_YAW} rad")

  BRIDGE_WIN=3
else
  tmux new-window -t "${SESSION}" -n slam
  tmux send-keys  -t "${SESSION}:slam" \
    "exec run_slam.sh ${CAMERA_X} ${CAMERA_Y} ${CAMERA_Z} ${CAMERA_ROLL} ${CAMERA_PITCH} ${CAMERA_YAW}" C-m
  BANNER+=("  1 slam        cuVSLAM + OAK-D; camera FLU xyz=${CAMERA_X} ${CAMERA_Y} ${CAMERA_Z} m  rpy=${CAMERA_ROLL} ${CAMERA_PITCH} ${CAMERA_YAW} rad")

  BRIDGE_WIN=2
fi
sleep "${BRIDGE_DELAY_S}"

tmux new-window  -t "${SESSION}" -n bridge
tmux send-keys   -t "${SESSION}:bridge" "${BRIDGE_CMD}" C-m
BANNER+=("  ${BRIDGE_WIN} bridge      ${BRIDGE_DESC}")
sleep "${COMPETITOR_DELAY_S}"

# The competitor package is what teams write (WP5). If it is not in this image
# the window becomes a plain shell with a one-line hint rather than a dead pane.
tmux new-window  -t "${SESSION}" -n competitor
tmux send-keys   -t "${SESSION}:competitor" \
  "if ros2 pkg prefix ${COMPETITOR_PKG} >/dev/null 2>&1; then exec ros2 launch ${COMPETITOR_PKG} ${COMPETITOR_LAUNCH}; else echo '[stack] ${COMPETITOR_PKG} is not in this image -- write it under src/, rebuild, and it launches here; shell kept.'; fi" C-m
LAST_WIN=$((BRIDGE_WIN + 1))
BANNER+=("  ${LAST_WIN} competitor  ros2 launch ${COMPETITOR_PKG} ${COMPETITOR_LAUNCH} (shell if not installed)")

echo "[stack] tmux session '${SESSION}' is up (remain-on-exit on -- dead panes keep their exit code):"
printf '%s\n' "${BANNER[@]}"

if [[ ${NO_SLAM} -eq 1 ]]; then
  cat <<EOM

  BRING YOUR OWN SLAM: cuVSLAM is not running. Nothing publishes
  /visual_slam/tracking/odometry or /oak/{left,right}/image_mono -- start your
  own tracker (another window, or host/run.sh shell) and point the bridge at it:

    BRIDGE_INPUT_TOPIC=/my_vio/odometry run_full_stack.sh --no-slam

  The output contract (ENU pose, body-FLU twist, the TF baseline, pose_frame)
  is docs/competitor_guide.md, "Bring your own SLAM". Read it before you fly.
EOM
fi

cat <<EOM

  check_topics.sh   prove the DDS link actually carries data
  record_bag.sh     mcap recording (light by default; --images costs ~36 MB/s)

Ctrl-b 0..${LAST_WIN}  switch window     Ctrl-b d  detach     exit  to tear down
EOM

exec tmux attach -t "${SESSION}"

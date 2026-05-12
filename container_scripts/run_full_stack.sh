#!/usr/bin/env bash
# Inside the skybeta container — start the full stack in a tmux session named
# 'skybeta' with three windows: agent, slam, bridge. Detaches into the session
# at the end; one Ctrl-b d to detach, `tmux kill-session -t skybeta` (or just
# exit the container) to tear everything down.
set -euo pipefail

SESSION="${SESSION:-skybeta}"
SLAM_DELAY_S="${SLAM_DELAY_S:-3}"     # let agent bind UART before SLAM starts
BRIDGE_DELAY_S="${BRIDGE_DELAY_S:-8}" # let cuVSLAM init before bridge subscribes

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "[stack] session '${SESSION}' already running — attaching"
  exec tmux attach -t "${SESSION}"
fi

# remain-on-exit keeps dead panes visible with their exit code instead of
# silently disappearing — failures are loud, not silent.
tmux new-session  -d -s "${SESSION}" -n agent
tmux set-option   -t "${SESSION}" remain-on-exit on

tmux send-keys    -t "${SESSION}:agent"  'exec run_agent.sh' C-m
sleep "${SLAM_DELAY_S}"

tmux new-window   -t "${SESSION}" -n slam
tmux send-keys    -t "${SESSION}:slam" \
  "exec run_slam.sh ${CAMERA_X:-0.0} ${CAMERA_Y:-0.0} ${CAMERA_Z:-0.0}" C-m
sleep "${BRIDGE_DELAY_S}"

tmux new-window   -t "${SESSION}" -n bridge
tmux send-keys    -t "${SESSION}:bridge" 'exec run_bridge.sh' C-m

cat <<EOM
[stack] tmux session '${SESSION}' is up:
  agent   MicroXRCEAgent on ${XRCE_DEV:-/dev/ttyTHS1}
  slam    cuVSLAM (camera offset FLU: x=${CAMERA_X:-0.0} y=${CAMERA_Y:-0.0} z=${CAMERA_Z:-0.0})
  bridge  /visual_slam/tracking/odometry -> /fmu/in/vehicle_visual_odometry

Ctrl-b 0..2  switch window     Ctrl-b d  detach     exit  to tear down
EOM

exec tmux attach -t "${SESSION}"

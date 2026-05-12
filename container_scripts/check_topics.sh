#!/usr/bin/env bash
# Inside the skybeta container — confirm the bridge is alive: list topics and
# sample timesync_status (a PX4 heartbeat that exists even with no sensors).
set -euo pipefail

# ROS 2 setup scripts touch unset vars (AMENT_TRACE_SETUP_FILES etc.);
# relax nounset just for the source then re-enable.
set +u
source /opt/ros/humble/setup.bash
source /root/skybeta_ws/install/local_setup.bash
set -u

echo "[check] ros2 topic list"
ros2 topic list

echo
echo "[check] Sampling /fmu/out/timesync_status (Ctrl-C to stop)"
exec ros2 topic echo /fmu/out/timesync_status \
  --qos-reliability best_effort \
  --qos-durability transient_local

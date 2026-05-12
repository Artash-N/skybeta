#!/usr/bin/env bash
# Inside the skybeta container — launch cuVSLAM + OAK-D driver + camera-offset
# static TF. Camera offset (FLU body, meters) comes from positional args or
# CAMERA_X/Y/Z env vars (defaults to camera-at-body-origin).
#
#   run_slam.sh                       # all offsets 0
#   run_slam.sh 0.10 0.0 -0.02        # x y z
#   CAMERA_X=0.10 CAMERA_Y=0 CAMERA_Z=-0.02 run_slam.sh
set -euo pipefail

CAMERA_X="${1:-${CAMERA_X:-0.0}}"
CAMERA_Y="${2:-${CAMERA_Y:-0.0}}"
CAMERA_Z="${3:-${CAMERA_Z:-0.0}}"

# ROS 2 setup scripts touch unset vars (AMENT_TRACE_SETUP_FILES etc.);
# relax nounset just for the source then re-enable.
set +u
source /opt/ros/humble/setup.bash
source /root/skybeta_ws/install/local_setup.bash
set -u

echo "[slam] camera offset (FLU): x=${CAMERA_X} y=${CAMERA_Y} z=${CAMERA_Z}"
exec ros2 launch oakd_vslam_bringup oakd_vslam.launch.py \
  camera_x:="${CAMERA_X}" \
  camera_y:="${CAMERA_Y}" \
  camera_z:="${CAMERA_Z}"

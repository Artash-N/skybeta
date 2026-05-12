#!/usr/bin/env bash
# Inside the skybeta container — republish /visual_slam/tracking/odometry
# (from cuVSLAM) onto /fmu/in/vehicle_visual_odometry with ENU->NED + FLU->FRD
# frame conversion and PX4 BEST_EFFORT QoS. Source: oakd_vslam_bringup.
set -euo pipefail

# ROS 2 setup scripts touch unset vars (AMENT_TRACE_SETUP_FILES etc.);
# relax nounset just for the source then re-enable.
set +u
source /opt/ros/humble/setup.bash
source /root/skybeta_ws/install/local_setup.bash
set -u

echo "[bridge] ros2 launch oakd_vslam_bringup px4_bridge.launch.py"
exec ros2 launch oakd_vslam_bringup px4_bridge.launch.py

"""Start the cuVSLAM -> PX4 VehicleOdometry bridge.

Run in a second container shell alongside `05_run_slam.sh`. The host-side
Micro-XRCE-DDS-Agent (see `06_run_px4_agent.sh`) shuttles the published
/fmu/in/vehicle_visual_odometry topic to the Pixhawk over serial.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package='oakd_vslam_bringup',
        executable='px4_odom_bridge',
        name='px4_odom_bridge',
        output='screen',
    )
    return LaunchDescription([bridge])

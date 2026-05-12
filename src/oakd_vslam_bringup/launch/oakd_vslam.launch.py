"""OAK-D stereo + IMU -> isaac_ros_visual_slam (cuVSLAM).

OAK-D-S2 has color stereo sensors (OV9782) that output bgr8. cuVSLAM needs mono8.
Two bgr_to_mono converter nodes bridge the encoding gap.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    depthai_share = get_package_share_directory('depthai_ros_driver_v3')
    bringup_share = get_package_share_directory('oakd_vslam_bringup')

    # Camera position in the drone body frame (meters, FLU).
    # Feeds a static base_link -> oak TF so cuVSLAM reports body-frame odometry
    # for the PX4 bridge (v1.14 has no EKF2_EV_POS_*).
    camera_x_arg = DeclareLaunchArgument('camera_x', default_value='0.0')
    camera_y_arg = DeclareLaunchArgument('camera_y', default_value='0.0')
    camera_z_arg = DeclareLaunchArgument('camera_z', default_value='0.0')

    # depthai's URDF is rooted at 'oak_parent_frame', not 'oak'. Attach it to
    # base_link so cuVSLAM (base_frame=base_link) can reach the camera optical
    # frames through a single connected TF tree.
    base_to_oak_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_oak',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'oak_parent_frame',
        ],
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_share, 'launch', 'driver.launch.py')
        ),
        launch_arguments={
            'name': 'oak',
            'params_file': os.path.join(bringup_share, 'config', 'oakd_vslam.yaml'),
        }.items(),
    )

    left_converter = Node(
        package='oakd_vslam_bringup',
        executable='bgr_to_mono',
        name='bgr_to_mono_left',
        parameters=[{
            'input_topic': '/oak/left/image_raw',
            'output_topic': '/oak/left/image_mono',
        }],
        output='screen',
    )

    right_converter = Node(
        package='oakd_vslam_bringup',
        executable='bgr_to_mono',
        name='bgr_to_mono_right',
        parameters=[{
            'input_topic': '/oak/right/image_raw',
            'output_topic': '/oak/right/image_mono',
        }],
        output='screen',
    )

    vslam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            'enable_image_denoising': False,
            'rectified_images': False,
            'enable_imu_fusion': False,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'imu_frame': 'oak_imu_frame',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'publish_odom_to_base_tf': True,
            'publish_map_to_odom_tf': True,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'path_max_size': 2048,
        }],
        remappings=[
            ('visual_slam/image_0', '/oak/left/image_mono'),
            ('visual_slam/camera_info_0', '/oak/left/camera_info'),
            ('visual_slam/image_1', '/oak/right/image_mono'),
            ('visual_slam/camera_info_1', '/oak/right/camera_info'),
            ('visual_slam/imu', '/oak/imu/data'),
        ],
    )

    container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[vslam],
        output='screen',
    )

    return LaunchDescription([
        camera_x_arg, camera_y_arg, camera_z_arg,
        base_to_oak_tf,
        camera, left_converter, right_converter, container,
    ])

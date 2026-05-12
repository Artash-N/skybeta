"""Bridge cuVSLAM nav_msgs/Odometry -> PX4 VehicleOdometry via uXRCE-DDS.

ROS convention (REP-103):  ENU world, FLU body.
PX4 convention:            NED world, FRD body.

This node republishes /visual_slam/tracking/odometry (ENU/FLU) on
/fmu/in/vehicle_visual_odometry (NED/FRD) with the QoS profile PX4's
uxrce_dds_client subscribers expect (BEST_EFFORT, VOLATILE, KEEP_LAST).

The frame math mirrors px4_ros_com's frame_transforms:
    q_ned_frd = NED_ENU_Q * q_enu_flu * AIRCRAFT_BASELINK_Q
where both constant quaternions are self-inverse 180 deg rotations.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry


# from_euler(pi, 0, pi/2) in wxyz
NED_ENU_Q = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
# from_euler(pi, 0, 0) in wxyz
AIRCRAFT_BASELINK_Q = (0.0, 1.0, 0.0, 0.0)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def stamp_to_us(stamp) -> int:
    return int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1000


class PX4OdomBridge(Node):

    def __init__(self):
        super().__init__('px4_odom_bridge')
        self.declare_parameter('input_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter('output_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('publish_velocity', True)
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.publish_velocity = self.get_parameter('publish_velocity').value

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(VehicleOdometry, out_topic, px4_qos)
        self.sub = self.create_subscription(Odometry, in_topic, self.cb, 10)
        self.get_logger().info(
            f"PX4 odom bridge up: {in_topic} -> {out_topic} "
            f"(velocity={'on' if self.publish_velocity else 'off'})"
        )

    def cb(self, msg: Odometry):
        out = VehicleOdometry()
        out.timestamp = self.get_clock().now().nanoseconds // 1000
        out.timestamp_sample = stamp_to_us(msg.header.stamp)

        out.pose_frame = VehicleOdometry.POSE_FRAME_NED

        p = msg.pose.pose.position
        # ENU -> NED: (x_ned, y_ned, z_ned) = (y_enu, x_enu, -z_enu)
        out.position = [float(p.y), float(p.x), float(-p.z)]

        # ROS quaternion is xyzw; convert to wxyz for internal math.
        q_ros = msg.pose.pose.orientation
        q_enu_flu = (q_ros.w, q_ros.x, q_ros.y, q_ros.z)
        q_tmp = quat_mul(NED_ENU_Q, q_enu_flu)
        q_ned_frd = quat_mul(q_tmp, AIRCRAFT_BASELINK_Q)
        out.q = [float(q_ned_frd[0]), float(q_ned_frd[1]),
                 float(q_ned_frd[2]), float(q_ned_frd[3])]

        if self.publish_velocity:
            out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
            v = msg.twist.twist.linear
            w = msg.twist.twist.angular
            # FLU -> FRD body: (x, -y, -z)
            out.velocity = [float(v.x), float(-v.y), float(-v.z)]
            out.angular_velocity = [float(w.x), float(-w.y), float(-w.z)]
        else:
            out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
            out.velocity = [float('nan')] * 3
            out.angular_velocity = [float('nan')] * 3

        # Covariance: ROS ships a full 6x6 row-major (36 floats). PX4 wants 3
        # diagonal variances per category. Variance is invariant under the
        # ENU<->NED sign flips (they square out), but the x/y swap permutes.
        pc = msg.pose.covariance
        tc = msg.twist.covariance
        # pose diag: pos=[0,7,14], rot=[21,28,35]
        out.position_variance = [float(pc[7]), float(pc[0]), float(pc[14])]
        out.orientation_variance = [float(pc[28]), float(pc[21]), float(pc[35])]
        if self.publish_velocity:
            out.velocity_variance = [float(tc[7]), float(tc[0]), float(tc[14])]
        else:
            out.velocity_variance = [float('nan')] * 3

        out.reset_counter = 0
        out.quality = 0  # 0 = unknown per px4_msgs; EKF2 tolerates this

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PX4OdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

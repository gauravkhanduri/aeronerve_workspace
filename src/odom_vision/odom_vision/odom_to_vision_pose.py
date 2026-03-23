#!/usr/bin/env python3
"""
Converts Gazebo odometry to MAVROS vision_pose format.

ros2 topic echo /odom --field pose.pose.position --once
Subscribes to: /odom (nav_msgs/msg/Odometry) for X/Y/Z
Publishes to: /mavros/vision_pose/pose (geometry_msgs/msg/PoseStamped)

Heading correction
------------------
If the drone heading in the GCS does not match the simulation, pass yaw_offset_deg:

    ros2 run odom_vision odom_to_vision_pose --ros-args -p yaw_offset_deg:=90.0

Common values to try:
    0.0   — no correction (default)
    90.0  — Gazebo X-forward but MAVROS expects East-forward
   -90.0  — opposite
   180.0  — drone appears reversed
"""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def _apply_yaw_offset(qx, qy, qz, qw, yaw_offset_rad: float):
    """
    Rotate quaternion (qx,qy,qz,qw) by yaw_offset_rad around the Z axis.
    Equivalent to: q_result = q_z_offset * q_original

    q_z_offset = (ox=0, oy=0, oz=sin(a/2), ow=cos(a/2))
    Quaternion multiply with ox=oy=0 simplifies to:
        rx = ow*qx + oz*qy
        ry = ow*qy - oz*qx
        rz = ow*qz + oz*qw
        rw = ow*qw - oz*qz
    """
    oz = math.sin(yaw_offset_rad / 2.0)
    ow = math.cos(yaw_offset_rad / 2.0)

    rx = ow * qx + oz * qy
    ry = ow * qy - oz * qx
    rz = ow * qz + oz * qw
    rw = ow * qw - oz * qz

    return rx, ry, rz, rw


class OdomToVisionPose(Node):

    def __init__(self):
        super().__init__('odom_to_vision_pose')

        self.declare_parameter('yaw_offset_deg', 0.0)
        yaw_deg = self.get_parameter('yaw_offset_deg').value
        self._yaw_offset_rad = math.radians(float(yaw_deg))

        # Subscriber for odometry from Gazebo (X/Y/Z)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publisher for MAVROS vision pose
        self.vision_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )

        self.get_logger().info('Odom to Vision Pose converter started')
        self.get_logger().info(f'yaw_offset_deg={yaw_deg:.1f}')
        self.get_logger().info('Subscribing to: /odom (X/Y/Z)')
        self.get_logger().info('Publishing to: /mavros/vision_pose/pose')

    def odom_callback(self, msg: Odometry):
        pose_stamped = PoseStamped()

        # Copy header
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = 'map'  # MAVROS expects 'map' frame

        # Copy position unchanged
        pose_stamped.pose.position = msg.pose.pose.position

        # Apply yaw offset to orientation
        q = msg.pose.pose.orientation
        if self._yaw_offset_rad == 0.0:
            pose_stamped.pose.orientation = q
        else:
            rx, ry, rz, rw = _apply_yaw_offset(
                q.x, q.y, q.z, q.w, self._yaw_offset_rad)
            pose_stamped.pose.orientation.x = rx
            pose_stamped.pose.orientation.y = ry
            pose_stamped.pose.orientation.z = rz
            pose_stamped.pose.orientation.w = rw

        # Publish
        self.vision_pub.publish(pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToVisionPose()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Author: Gourav Khanduri
# Email:  gauravkhanduri93@gmail.com
"""
send_forward_goal — tell Nav2 "go X meters forward from where you are".

Reads /mavros/local_position/pose once, computes a map-frame goal that is
`distance` meters ahead in the drone's CURRENT heading, publishes it to
/goal_pose (which bt_navigator picks up as a NavigateToPose action).

Usage:
    ros2 run odom_vision send_forward_goal               # default 5m
    ros2 run odom_vision send_forward_goal --dist 10.0   # any distance
"""

import argparse
import sys
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class ForwardGoalSender(Node):
    def __init__(self, distance: float):
        super().__init__("send_forward_goal")
        self.distance = distance
        self.pose_received = False
        self.pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, sensor_qos
        )
        self.get_logger().info(f"Waiting for pose on /mavros/local_position/pose...")

    def _pose_cb(self, msg: PoseStamped):
        if self.pose_received:
            return
        self.pose_received = True

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        gx = x + self.distance
        gy = y

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = z  # keep current altitude
        # Identity orientation — don't constrain final heading. yaw_goal_tolerance
        # in nav2_drone.yaml should be wide (e.g. 6.28) so Nav2 doesn't spin at goal.
        goal.pose.orientation.w = 1.0

        time.sleep(0.3)
        self.pub.publish(goal)
        self.get_logger().info(
            f"Sent goal: ({gx:.2f}, {gy:.2f}, {z:.2f}) "
            f"= {self.distance:.1f}m ahead along +X from ({x:.2f}, {y:.2f})"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dist", type=float, default=5.0, help="Forward distance (m)")
    # argparse can't see ros args — strip them
    known, _ = parser.parse_known_args()

    rclpy.init()
    node = ForwardGoalSender(known.dist)

    # Spin until the pose callback fires once + goal is published
    timeout_s = 5.0
    t0 = time.time()
    while rclpy.ok() and not node.pose_received:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > timeout_s:
            node.get_logger().error(
                "Timed out waiting for /mavros/local_position/pose — is MAVROS running?"
            )
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    # Brief spin so the publish is flushed
    t_end = time.time() + 0.5
    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

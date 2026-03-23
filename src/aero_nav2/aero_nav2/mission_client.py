#!/usr/bin/env python3
"""
mission_client.py — Custom Nav Stack
======================================
Sends a single navigation goal and monitors progress.

Usage
-----
  ros2 run aero_nav2 mission_client \
    --ros-args -p goal_x:=10.0 -p goal_y:=0.0 -p goal_z:=3.0

This node publishes the goal once on /goal_pose and then watches /odom
to report distance-to-goal every 2 seconds.  It exits when /goal_reached
is published as True.

Topics
------
Publishes:   /goal_pose     (geometry_msgs/PoseStamped)
Subscribes:  /odom          (nav_msgs/Odometry)
             /goal_reached  (std_msgs/Bool)
             /planning_status (std_msgs/String)

Parameters
----------
goal_x : float (default 10.0)
goal_y : float (default 0.0)
goal_z : float (default 3.0)  cruise altitude (m)
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


class MissionClientNode(Node):

    def __init__(self):
        super().__init__('mission_client')

        self.declare_parameter('goal_x', 10.0)
        self.declare_parameter('goal_y',  0.0)
        self.declare_parameter('goal_z',  3.0)

        self._goal_x = self.get_parameter('goal_x').value
        self._goal_y = self.get_parameter('goal_y').value
        self._goal_z = self.get_parameter('goal_z').value

        self._odom_x = 0.0
        self._odom_y = 0.0

        # Publishers
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscriptions
        self.create_subscription(Odometry, '/odom',           self._odom_cb,    10)
        self.create_subscription(Bool,     '/goal_reached',   self._reached_cb, 10)
        self.create_subscription(String,   '/planning_status',self._status_cb,  10)

        # Wait 1 s for subscribers to connect before publishing goal
        self.create_timer(1.0, self._send_goal_once)

        # Progress log every 2 s
        self.create_timer(2.0, self._log_progress)

        self._goal_sent   = False
        self._goal_reached = False

    def _send_goal_once(self):
        if self._goal_sent:
            return
        self._goal_sent = True

        goal                    = PoseStamped()
        goal.header.stamp       = self.get_clock().now().to_msg()
        goal.header.frame_id    = 'map'
        goal.pose.position.x    = self._goal_x
        goal.pose.position.y    = self._goal_y
        goal.pose.position.z    = self._goal_z
        goal.pose.orientation.w = 1.0

        self._goal_pub.publish(goal)
        self.get_logger().info(
            f'Goal sent: ({self._goal_x:.1f}, {self._goal_y:.1f}, {self._goal_z:.1f})')

    def _odom_cb(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y

    def _reached_cb(self, msg: Bool):
        if msg.data and not self._goal_reached:
            self._goal_reached = True
            self.get_logger().info('Goal reached! Mission complete.')

    def _status_cb(self, msg: String):
        if msg.data != 'ok':
            self.get_logger().warn(f'Planner status: {msg.data}')

    def _log_progress(self):
        if self._goal_reached:
            return
        dist = math.hypot(
            self._goal_x - self._odom_x,
            self._goal_y - self._odom_y,
        )
        self.get_logger().info(
            f'Drone: ({self._odom_x:.1f}, {self._odom_y:.1f})  '
            f'→ goal: ({self._goal_x:.1f}, {self._goal_y:.1f})  '
            f'dist={dist:.1f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Author: Gourav Khanduri
# Email:  gauravkhanduri93@gmail.com
"""
Quick test: send a goal to the global planner and watch it plan.

Run this AFTER launching:
  1. ZED camera
  2. OctoMap server  
  3. Global planner node

Usage:
    # Send drone to position (3.0, 2.0):
    python3 test_send_goal.py 3.0 2.0

    # Send drone home:
    python3 test_send_goal.py 0.0 0.0

    # Test with a mission (multiple waypoints):
    python3 test_send_goal.py --mission
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
import numpy as np


class PlannerTester(Node):
    def __init__(self):
        super().__init__('planner_tester')

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.mission_pub = self.create_publisher(Path, '/mission_waypoints', 10)

        # Subscribe to see results
        self.create_subscription(
            Path, '/planned_path', self._path_callback, 10
        )
        self.create_subscription(
            OccupancyGrid, '/projected_map', self._grid_callback, 1
        )
        

        self.grid_received = False
        self.path_received = False

    def _grid_callback(self, msg):
        if not self.grid_received:
            data = np.array(msg.data, dtype=np.int8)
            free = np.sum(data == 0)
            occupied = np.sum(data == 100)
            unknown = np.sum(data == -1)
            total = len(data)
            self.get_logger().info(
                f'Grid: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m | '
                f'Free: {free} | Occupied: {occupied} | Unknown: {unknown}'
            )
            self.grid_received = True

    def _path_callback(self, msg):
        if msg.poses:
            self.get_logger().info(f'Planned path received: {len(msg.poses)} points')
            for i, pose in enumerate(msg.poses):
                p = pose.pose.position
                self.get_logger().info(f'  [{i}] x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}')
            self.path_received = True
        else:
            self.get_logger().info('Empty path received (mission complete or no path)')

    def send_goal(self, x: float, y: float, z: float = 5.0):
        """Send a single goal to the planner."""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f}, {z:.2f})')

        self.goal_pub.publish(goal)

    def send_test_mission(self, z: float = 5.0):
        """Send a simple square mission pattern."""
        waypoints = [
            (3.0, 0.0),
            (3.0, 3.0),
            (0.0, 3.0),
            (0.0, 0.0),  # return home
        ]

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        for x, y in waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.get_logger().info(
            f'Sending test mission: {len(waypoints)} waypoints (3m square)'
        )
        self.mission_pub.publish(path)


def main():
    rclpy.init()
    node = PlannerTester()

    # Wait a moment for connections
    import time
    time.sleep(1.0)

    if len(sys.argv) >= 3 and sys.argv[1] != '--mission':
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
        node.send_goal(x, y, z)
    elif '--mission' in sys.argv:
        node.send_test_mission()
    else:
        print('Usage:')
        print('  python3 test_send_goal.py <x> <y>        # single goal')
        print('  python3 test_send_goal.py --mission       # test square mission')
        node.send_goal(3.0, 2.0)  # default test goal

    # Spin for a few seconds to receive the planned path
    try:
        end_time = time.time() + 10.0
        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.path_received:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

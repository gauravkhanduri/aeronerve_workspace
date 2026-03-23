#!/usr/bin/env python3
"""
nav2_mission_client.py
=======================
High-level mission interface that sends a single NavigateToPose goal
to the Nav2 bt_navigator action server using nav2_simple_commander.

The full Nav2 pipeline triggered by this node:
  nav2_mission_client
      │  NavigateToPose action
      ▼
  bt_navigator
      ├── planner_server (SmacPlannerHybrid)  → nav_msgs/Path
      └── controller_server (RegulatedPurePursuit) → /cmd_vel
                                                           │
                                                     nav2_mavros_bridge
                                                           │
                                               /mavros/setpoint_position/local
                                                           │
                                                     PX4 OFFBOARD

Prerequisites
-------------
1. The drone must already be armed and flying at the target altitude
   (OFFBOARD mode enabled, nav2_mavros_bridge is publishing 20 Hz setpoints).
2. The full Nav2 stack must be running (use nav2_launch.py).
3. Nav2 lifecycle nodes must be in the ACTIVE state.

Parameters (ros2 run / launch overridable)
------------------------------------------
goal_x   : float (default  5.0)   m — goal position in map frame
goal_y   : float (default  0.0)   m
goal_z   : float (default  3.0)   m — altitude (carried through to bridge)
goal_yaw : float (default  0.0)   rad — desired heading at goal (0 = East)

Usage examples
--------------
# Default goal
ros2 run aero_nav2 nav2_mission_client

# Custom goal
ros2 run aero_nav2 nav2_mission_client \\
  --ros-args -p goal_x:=10.0 -p goal_y:=5.0 -p goal_z:=4.0

# Via waypoint list (multiple goals) — not yet implemented here;
# see BasicNavigator.followWaypoints() for that use-case.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def main(args=None):
    rclpy.init(args=args)

    navigator = BasicNavigator()

    # ── Parameters ────────────────────────────────────────────────────────
    navigator.declare_parameter('goal_x', 5.0)
    navigator.declare_parameter('goal_y', 0.0)
    navigator.declare_parameter('goal_z', 3.0)
    navigator.declare_parameter('goal_yaw', 0.0)

    gx = navigator.get_parameter('goal_x').value
    gy = navigator.get_parameter('goal_y').value
    gz = navigator.get_parameter('goal_z').value
    gyaw = navigator.get_parameter('goal_yaw').value

    # ── Wait for Nav2 to become active ────────────────────────────────────
    navigator.get_logger().info('Waiting for Nav2 stack to become active...')
    # We do not run AMCL — localisation comes from MAVROS TF.
    # Pass localizer='controller_server' so BasicNavigator waits for a node
    # that actually exists in our stack instead of the default 'amcl'.
    navigator.waitUntilNav2Active(localizer='controller_server')
    navigator.get_logger().info('Nav2 is active.')

    # ── Build goal PoseStamped ─────────────────────────────────────────────
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = gx
    goal.pose.position.y = gy
    goal.pose.position.z = gz
    # Convert yaw to quaternion (rotation around Z axis)
    goal.pose.orientation.z = math.sin(gyaw / 2.0)
    goal.pose.orientation.w = math.cos(gyaw / 2.0)

    navigator.get_logger().info(
        f'Sending goal: ({gx:.2f}, {gy:.2f}, {gz:.2f}) m, yaw={math.degrees(gyaw):.1f}°'
    )

    # ── Send goal to bt_navigator ──────────────────────────────────────────
    navigator.goToPose(goal)

    # ── Monitor progress ───────────────────────────────────────────────────
    tick = 0
    while not navigator.isTaskComplete():
        tick += 1
        feedback = navigator.getFeedback()
        if feedback and tick % 20 == 0:
            dist = getattr(feedback, 'distance_remaining', '?')
            navigator.get_logger().info(f'Distance remaining: {dist}')

    # ── Report result ──────────────────────────────────────────────────────
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Goal reached successfully.')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('Navigation was cancelled.')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('Navigation FAILED.')
    else:
        navigator.get_logger().error(f'Unknown result: {result}')

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

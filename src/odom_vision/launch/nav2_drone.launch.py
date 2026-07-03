#!/usr/bin/env python3
"""
nav2_drone.launch.py — Nav2 MPPI local planner for ArduPilot drone.

What this starts:
  1. Nav2 stack (controller_server, planner_server, bt_navigator,
                 behavior_server, lifecycle_manager)
  2. cmd_vel_bridge: converts /cmd_vel -> /mavros/setpoint_raw/local

Prerequisites (start separately via drone.launch.py):
  - MAVROS
  - OctoMap publishing /projected_map
  - /mavros/local_position/pose being published
  - base_link -> camera TF (ZED driver or static_transform_publisher)

Usage:
    ros2 launch odom_vision nav2_drone.launch.py

Then send a goal:
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 0.0},
       orientation: {w: 1.0}}}"
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("odom_vision")
    nav2_config = os.path.join(pkg_share, "config", "nav2_drone.yaml")
    bt_xml = os.path.join(pkg_share, "config", "drone_nav_bt.xml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        # Nav2 controller server (runs MPPI)
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=[nav2_config, {"use_sim_time": use_sim_time}],
        ),

        # Nav2 global planner (A* navfn — provides path for MPPI to follow)
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_config, {"use_sim_time": use_sim_time}],
        ),

        # Behavior server (spin, backup, wait)
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            output="screen",
            parameters=[nav2_config, {"use_sim_time": use_sim_time}],
        ),

        # BT navigator — orchestrates planner + controller via behavior tree
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                nav2_config,
                {"use_sim_time": use_sim_time,
                 "default_nav_to_pose_bt_xml": bt_xml},
            ],
        ),

        # Lifecycle manager — brings all Nav2 nodes through configure/activate
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": True},
                {"node_names": [
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                ]},
            ],
        ),

        # Bridge: /cmd_vel (Twist from MPPI) -> /mavros/setpoint_raw/local
        Node(
            package="odom_vision",
            executable="cmd_vel_bridge",
            name="cmd_vel_bridge",
            output="screen",
        ),

        # QoS fix: MAVROS odom is BEST_EFFORT, bt_navigator wants RELIABLE
        Node(
            package="odom_vision",
            executable="odom_qos_relay",
            name="odom_qos_relay",
            output="screen",
        ),
    ])

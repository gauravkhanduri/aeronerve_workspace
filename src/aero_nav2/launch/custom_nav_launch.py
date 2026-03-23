#!/usr/bin/env python3
"""
custom_nav_launch.py — Custom Nav Stack
=========================================
Launches the from-scratch custom navigation stack for the PX4/MAVROS drone.
This replaces the Nav2 bringup entirely.

Nodes started
-------------
1. depth_to_costmap   — /depth_camera → /depth_pointcloud (PointCloud2)
2. occupancy_map      — /depth_pointcloud → /occupancy_grid (OccupancyGrid)
3. global_planner     — /occupancy_grid + /odom + /goal_pose → /global_path
4. local_planner      — /global_path + /odom + /depth_pointcloud → /cmd_vel
5. nav2_mavros_bridge — /cmd_vel → /mavros/setpoint_position/local

NOT started here (run separately first)
----------------------------------------
- PX4 SITL + Gazebo
- MAVROS node
- ros_gz_bridge / odom_to_vision_pose / tf_publisher
    → use:  ros2 launch odom_vision pose.launch.py

Usage
-----
Terminal 1 (base drone stack):
  ros2 launch odom_vision pose.launch.py

Terminal 2 (custom nav stack):
  ros2 launch aero_nav2 custom_nav_launch.py

Terminal 3 (send a goal):
  ros2 run aero_nav2 mission_client \
    --ros-args -p goal_x:=10.0 -p goal_y:=0.0 -p goal_z:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation clock",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── 1. Depth image → PointCloud2 ─────────────────────────────────────────
    depth_node = Node(
        package="aero_nav2",
        executable="depth_to_costmap",
        name="depth_to_costmap",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "image_width": 640,
                "image_height": 480,
                "hfov_deg": 68.94,  # OakD-Lite measured HFOV
                "downsample_factor": 8,  # every 8th pixel → ~9600 pts/frame
                "min_depth": 0.1,
                "max_depth": 8.0,
            }
        ],
    )

    # ── 2. Occupancy map builder ──────────────────────────────────────────────
    occ_map_node = Node(
        package="aero_nav2",
        executable="occupancy_map",
        name="occupancy_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "resolution": 0.2,
                "grid_size_m": 200.0,
                "origin_x": -100.0,
                "origin_y": -100.0,
                "min_obstacle_z": 1.0,  # ignore ground
                "max_obstacle_z": 5.5,  # ignore well above drone
                "decay_time": 5.0,  # obstacles persist 5 s after leaving FOV
                "inflation_radius": 0.8,  # safety margin (drone radius ~0.5 m)
                "publish_hz": 3.0,
            }
        ],
    )

    # ── 3. A* global planner ──────────────────────────────────────────────────
    global_planner_node = Node(
        package="aero_nav2",
        executable="global_planner",
        name="global_planner",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "replan_hz": 1.0,
                "goal_tolerance": 0.5,
            }
        ],
    )

    # ── 4. Holonomic DWA local planner ────────────────────────────────────────
    local_planner_node = Node(
        package="aero_nav2",
        executable="local_planner",
        name="local_planner",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "max_speed": 0.8,  # m/s — matches RPP desired_linear_vel
                "v_samples": 8,  # 9 × 17 = 153 candidates
                "sim_time": 1.5,  # s forward simulation
                "sim_steps": 5,
                "lookahead_dist": 2.5,  # m on global path
                "goal_tolerance": 0.5,  # m
                "obstacle_stop_dist": 0.6,  # m — hard stop margin
                "w_heading": 0.6,
                "w_clearance": 0.3,
                "w_speed": 0.1,
                "control_hz": 10.0,
            }
        ],
    )

    # ── 5. cmd_vel → MAVROS setpoint bridge ──────────────────────────────────
    bridge_node = Node(
        package="aero_nav2",
        executable="nav2_mavros_bridge",
        name="nav2_mavros_bridge",
        output="screen",
        parameters=[
            {
                "lookahead_dist": 0.8,
            }
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            depth_node,
            # cc_map_node,
            global_planner_node,
            local_planner_node,
            bridge_node,
        ]
    )

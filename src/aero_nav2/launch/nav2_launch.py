#!/usr/bin/env python3
"""
nav2_launch.py — aero_nav2
===========================
Launches the full Nav2 integration stack for PX4/MAVROS drone.

Nodes started by this file
--------------------------
1. depth_to_costmap      — /depth_camera → /depth_pointcloud (PointCloud2)
2. nav2_mavros_bridge    — /cmd_vel (Nav2) → /mavros/setpoint_position/local
3. Nav2 navigation stack (via nav2_bringup/navigation_launch.py):
     - bt_navigator
     - planner_server        (SmacPlannerHybrid)
     - controller_server     (RegulatedPurePursuitController)
     - behavior_server       (Wait only)
     - waypoint_follower
     - smoother_server
     - lifecycle_manager_navigation

NOT started here (run separately)
----------------------------------
- PX4 SITL / Gazebo
- MAVROS node
- ros_gz_bridge / odom_to_vision_pose / tf_publisher
  → use:  ros2 launch odom_vision pose.launch.py

Usage
-----
Terminal 1 (base drone stack):
  ros2 launch odom_vision pose.launch.py

Terminal 2 (Nav2 stack):
  ros2 launch aero_nav2 nav2_launch.py

Terminal 3 (send a goal):
  ros2 run aero_nav2 nav2_mission_client \\
    --ros-args -p goal_x:=5.0 -p goal_y:=0.0 -p goal_z:=3.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package paths ─────────────────────────────────────────────────────
    pkg_aero_nav2 = get_package_share_directory('aero_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(pkg_aero_nav2, 'config', 'nav2_params.yaml')

    # ── Launch arguments ──────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock (true for SITL)',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for Nav2 nodes (debug/info/warn/error)',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 with the drone Nav2 configuration',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    rviz_enabled = LaunchConfiguration('rviz')

    # Buffer stdout for cleaner terminal output
    set_log_env = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # ── Node 1: Depth image → PointCloud2 sensor bridge ──────────────────
    depth_to_costmap_node = Node(
        package='aero_nav2',
        executable='depth_to_costmap',
        name='depth_to_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_width': 640,
            'image_height': 480,
            'hfov_deg': 68.94,
            'downsample_factor': 8,   # use every 8th pixel → ~9600 pts/frame
            'min_depth': 0.1,
            'max_depth': 8.0,
        }],
    )

    # ── Node 2: Nav2 cmd_vel → MAVROS setpoint bridge ────────────────────
    nav2_mavros_bridge_node = Node(
        package='aero_nav2',
        executable='nav2_mavros_bridge',
        name='nav2_mavros_bridge',
        output='screen',
        parameters=[{
            'lookahead_dist': 0.8,
        }],
    )

    # ── Nav2 navigation stack ─────────────────────────────────────────────
    # Uses nav2_bringup's navigation_launch.py which starts:
    #   planner_server, controller_server, bt_navigator, behavior_server,
    #   waypoint_follower, smoother_server, lifecycle_manager_navigation
    #
    # We do NOT start map_server or amcl (localisation comes from MAVROS TF)
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': log_level,
        }.items(),
    )

    # ── RViz2 (optional, on by default) ──────────────────────────────────
    rviz_config = os.path.join(pkg_aero_nav2, 'config', 'nav2_drone.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz_enabled),
    )

    return LaunchDescription([
        set_log_env,
        use_sim_time_arg,
        log_level_arg,
        rviz_arg,
        depth_to_costmap_node,
        nav2_mavros_bridge_node,
        nav2_navigation,
        rviz_node,
    ])

# Author: Gourav Khanduri
# Email:  gauravkhanduri93@gmail.com
"""
Planner + Executor Launch File
Launches global planner and mission executor together.

Prerequisites: ZED + OctoMap + MAVROS must already be running.

Usage:
    ros2 launch isr_drone_bringup planner_executor.launch.py

    # For ground testing (lower altitude):
    ros2 launch isr_drone_bringup planner_executor.launch.py flight_altitude:=1.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    altitude_arg = DeclareLaunchArgument(
        'flight_altitude', default_value='5.0',
        description='Flight altitude in meters'
    )

    drone_radius_arg = DeclareLaunchArgument(
        'drone_radius', default_value='0.4',
        description='Drone radius for obstacle inflation (meters)'
    )

    optimistic_arg = DeclareLaunchArgument(
        'treat_unknown_as_free', default_value='True',
        description='Plan through unknown cells (optimistic) or avoid them'
    )

    # Global Planner
    planner_node = Node(
        package='isr_drone_bringup',
        executable='global_planner_node.py',
        name='global_planner',
        output='screen',
        parameters=[{
            'drone_radius': LaunchConfiguration('drone_radius'),
            'replan_interval': 2.0,
            'waypoint_reached_radius': 0.5,
            'flight_altitude': LaunchConfiguration('flight_altitude'),
            'treat_unknown_as_free': LaunchConfiguration('treat_unknown_as_free'),
            'max_planning_iterations': 200000,
        }],
    )

    # Mission Executor
    executor_node = Node(
        package='isr_drone_bringup',
        executable='mission_executor_node.py',
        name='mission_executor',
        output='screen',
        parameters=[{
            'acceptance_radius': 0.5,
            'flight_altitude': LaunchConfiguration('flight_altitude'),
            'setpoint_rate': 20.0,
            'tcp_port': 5555,
        }],
    )

    return LaunchDescription([
        altitude_arg,
        drone_radius_arg,
        optimistic_arg,
        planner_node,
        executor_node,
    ])

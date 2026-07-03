#!/usr/bin/env python3
"""
drone.launch.py — Real drone launch for Jetson Orin companion computer.

Starts everything needed for drone operation in one command:
  - MAVROS (serial connection to flight controller)
  - drone_state_broadcaster (streams pose + state on port 5761)
  - mission_sequencer (waits for mission upload on port 5760)

Usage:
    ros2 launch odom_vision drone.launch.py

Optional overrides:
    ros2 launch odom_vision drone.launch.py fcu_url:=/dev/ttyUSB0:921600
    ros2 launch odom_vision drone.launch.py mission_port:=5760 state_port:=5761
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    fcu_url      = LaunchConfiguration('fcu_url',      default='serial:///dev/ttyACM0:921600')
    mission_port = LaunchConfiguration('mission_port', default='5760')
    state_port   = LaunchConfiguration('state_port',   default='5761')

    mavros = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mavros', 'mavros_node',
            '--ros-args',
            '-p', ['fcu_url:=', fcu_url],
            '-p', "plugin_allowlist:=['*']",
        ],
        output='screen',
    )

    # Delay stream rate call by 5s to give MAVROS time to start up
    set_stream_rate = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/mavros/set_stream_rate',
                    'mavros_msgs/srv/StreamRate',
                    '{stream_id: 6, message_rate: 30, on_off: true}',
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([

        DeclareLaunchArgument('fcu_url',      default_value='serial:///dev/ttyACM0:921600',
                              description='Flight controller serial URL'),
        DeclareLaunchArgument('mission_port', default_value='5760',
                              description='TCP port to receive mission from GCS'),
        DeclareLaunchArgument('state_port',   default_value='5761',
                              description='TCP port to stream state to GCS'),

        mavros,
        set_stream_rate,

        # Static TF: zed2_camera_link → base_link
        # ZED VIO publishes odom → zed2_camera_link; this extends the chain to
        # base_link: map → odom → zed2_camera_link → base_link.
        # Translation is inverted (camera→base direction): camera is 0.3048m
        # forward and 0.2134m below base_link in physical mounting.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zed_to_base_link_tf',
            arguments=[
                '--x', '-0.3048',
                '--y', '0.0',
                '--z', '0.2134',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'zed2_camera_link',
                '--child-frame-id', 'base_link',
            ],
            output='screen',
        ),


        # Drone state broadcaster — streams pose + armed/mode to GCS on state_port
        Node(
            package='odom_vision',
            executable='drone_state_broadcaster',
            name='drone_state_broadcaster',
            parameters=[{'port': 5761, 'broadcast_hz': 5.0}],
            output='screen',
        ),

        # Mission sequencer — waits for GCS to send mission JSON on mission_port
        Node(
            package='odom_vision',
            executable='mission_sequencer',
            name='mission_sequencer',
            parameters=[{'tcp_port': 5760, 'arrival_radius': 0.3}],
            output='screen',
        ),

    ])

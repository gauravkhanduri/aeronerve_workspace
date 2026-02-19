#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # MAVROS node
        ExecuteProcess(
            cmd=['ros2', 'run', 'mavros', 'mavros_node', '--ros-args', '-p', 'fcu_url:=udp://127.0.0.1:14551@'],
            output='screen'
        ),
        # Gazebo bridge for odometry
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='odom_bridge',
                arguments=[
                    '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/rgb_camera@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
                ],
                output='screen'
            ),
        Node(
            package='odom_vision',
            executable='odom_to_vision_pose',
            name='odom_to_vision_pose',
            output='screen'
        ),
        Node(
            package='odom_vision',
            executable='tf_publisher',
            name='tf_publisher',
            output='screen'
        ),
    ])

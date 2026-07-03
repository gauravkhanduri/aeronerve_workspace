# Author: Gourav Khanduri
# Email:  gauravkhanduri93@gmail.com
"""
ISR Drone Full System Bringup
ROS2 Humble — Launches everything needed for autonomous flight

Launches:
1. ZED 2i camera node (VIO + pointcloud + depth)
2. OctoMap server (3D mapping → 2D occupancy grid)
3. MAVROS (bridge to PX4 flight controller)
4. Vision pose bridge (feeds ZED VIO into PX4's EKF as GPS replacement)
5. Static TF for camera mounting

Usage:
    # Full system:
    ros2 launch isr_drone_bringup full_system.launch.py

    # With custom flight altitude:
    ros2 launch isr_drone_bringup full_system.launch.py flight_altitude:=8.0

    # With custom FCU connection:
    ros2 launch isr_drone_bringup full_system.launch.py fcu_url:=/dev/ttyTHS1:921600
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================================================
    # Launch Arguments
    # ============================================================
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyTHS1:921600',
        description='FCU connection string for MAVROS. '
                    'Serial: /dev/ttyTHS1:921600 (Jetson UART) '
                    'USB: /dev/ttyACM0:57600 '
                    'UDP (SITL): udp://:14540@'
    )

    flight_altitude_arg = DeclareLaunchArgument(
        'flight_altitude',
        default_value='5.0',
        description='Planned flight altitude in meters. '
                    'Used to configure OctoMap z-filtering.'
    )

    octomap_resolution_arg = DeclareLaunchArgument(
        'octomap_resolution',
        default_value='0.3',
        description='OctoMap grid resolution in meters.'
    )

    # ============================================================
    # 1. ZED 2i Camera Node
    # ============================================================
    # The ZED wrapper provides: pointcloud, depth, VIO pose, IMU, TF
    # It publishes the odom → zed_camera_link transform chain
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
        }.items(),
    )

    # ============================================================
    # 2. OctoMap Server (delayed start — wait for ZED to initialize)
    # ============================================================
    # Give ZED 5 seconds to start publishing before OctoMap subscribes
    octomap_server_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='octomap_server',
                executable='octomap_server_node',
                name='octomap_server',
                output='screen',
                parameters=[{
                    'resolution': LaunchConfiguration('octomap_resolution'),
                    'frame_id': 'map',
                    'base_frame_id': 'base_link',
                    'sensor_model.max_range': 15.0,
                    'sensor_model.hit': 0.7,
                    'sensor_model.miss': 0.4,
                    'sensor_model.min': 0.12,
                    'sensor_model.max': 0.97,
                    'pointcloud_min_z': 0.5,
                    'pointcloud_max_z': 10.0,
                    'occupancy_min_z': 2.0,
                    'occupancy_max_z': 8.0,
                    'filter_ground': True,
                    'ground_filter.distance': 0.1,
                    'ground_filter.plane_distance': 0.07,
                    'latch': False,
                    'publish_free_space': False,
                    'colored_map.enabled': False,
                }],
                remappings=[
                    ('cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),
                ],
            )
        ]
    )

    # ============================================================
    # 3. MAVROS (bridge to PX4 FCU)
    # ============================================================
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': '',
            'system_id': 255,
            'component_id': 240,
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
        }],
    )

    # ============================================================
    # 4. Vision Pose Bridge
    # ============================================================
    # This node takes ZED's VIO pose and republishes it to MAVROS
    # so PX4 can use it as a GPS replacement.
    #
    # ZED publishes:  /zed/zed_node/pose (PoseStamped in map frame)
    # MAVROS expects: /mavros/vision_pose/pose (PoseStamped)
    #
    # PX4 parameters you MUST set on the FCU:
    #   EKF2_EV_CTRL = 15        (fuse vision position + velocity + yaw)
    #   EKF2_HGT_REF = 3         (use vision as height reference)
    #   EKF2_EV_DELAY = 0        (adjust based on your actual delay)
    #   EKF2_GPS_CTRL = 0        (disable GPS fusion since GPS-denied)
    #
    # For ArduPilot:
    #   VISO_TYPE = 1             (MAVLink vision)
    #   EK3_SRC1_POSXY = 6       (ExternalNav)
    #   EK3_SRC1_VELXY = 6       (ExternalNav)
    #   EK3_SRC1_POSZ = 6        (ExternalNav)
    #   EK3_SRC1_YAW = 6         (ExternalNav)

    vision_pose_bridge = Node(
        package='topic_tools',
        executable='relay',
        name='vision_pose_relay',
        output='screen',
        arguments=[
            '/zed/zed_node/pose',
            '/mavros/vision_pose/pose'
        ],
    )

    # ============================================================
    # 5. Static TF: base_link → zed_camera_link
    # ============================================================
    # Adjust these values to match YOUR camera mounting on the drone
    #
    # Measure from drone center (base_link) to camera center:
    #   x = forward (meters)
    #   y = left (meters)
    #   z = up (meters)
    #
    # If camera is tilted down (common for ISR), you need a rotation.
    # Example: 15° downward tilt = rotation around Y axis
    #   quaternion for -15° pitch: qx=0, qy=-0.1305, qz=0, qw=0.9914

    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            # Translation: x_forward, y_left, z_up (meters)
            '0.05', '0.0', '-0.03',
            # Rotation as quaternion: qx, qy, qz, qw
            # No tilt (camera faces forward horizontally):
            '0', '0', '0', '1',
            # Frames
            'base_link', 'zed_camera_link'
        ],
    )

    # ============================================================
    # Return Launch Description
    # ============================================================
    return LaunchDescription([
        # Arguments
        fcu_url_arg,
        flight_altitude_arg,
        octomap_resolution_arg,

        # Nodes (in startup order)
        static_tf_base_to_camera,     # TF first
        mavros_node,                   # FCU bridge
        zed_launch,                    # Camera (takes ~3s to initialize)
        vision_pose_bridge,            # VIO → MAVROS bridge
        octomap_server_node,           # OctoMap (delayed 5s, needs pointcloud + TF)
    ])

# Author: Gourav Khanduri
# Email:  gauravkhanduri93@gmail.com
"""
OctoMap Server Launch File for Outdoor ISR Drone
ROS2 Humble + ZED 2i + OctoMap

This launch file starts:
1. OctoMap server node (subscribes to ZED pointcloud, builds 3D map, publishes 2D grid)

Prerequisites:
- ZED camera node must be running (launch separately)
- TF tree must have: map → odom → base_link → zed_camera_link chain
- sudo apt install ros-humble-octomap-server

Parameters are tuned for ~5m flight altitude outdoors. See inline comments for
the rationale behind each value.

Usage:
    ros2 launch odom_vision octomap_mapping.launch.py

    # Or with custom parameters:
    ros2 launch odom_vision octomap_mapping.launch.py resolution:=0.2 max_range:=5.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.3',
        description='OctoMap voxel resolution in meters. '
                    '0.3m is a good balance for 5m altitude outdoor flight.'
    )

    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='5.0',
        description='Maximum sensor range to integrate in meters. '
                    'ZED 2i stereo noise grows quadratically with depth; '
                    '5.0m keeps the noise tail out of the map.'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Fixed frame for the OctoMap. Must match your TF tree root.'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value='base_link',
        description='Robot base frame for ground plane filtering. '
                    'Must exist in the TF tree.'
    )

    resolution    = LaunchConfiguration('resolution')
    max_range     = LaunchConfiguration('max_range')
    frame_id      = LaunchConfiguration('frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            # ---- Core map settings ----
            'resolution': resolution,
            'frame_id': frame_id,
            'base_frame_id': base_frame_id,

            # ---- Sensor model (tuned for ZED 2i outdoor) ----
            'sensor_model.max_range': max_range,
            'sensor_model.hit': 0.7,    # log-odds hit prob
            'sensor_model.miss': 0.35,  # aggressive clearing to kill stale noise
            'sensor_model.min': 0.12,   # clamping min
            'sensor_model.max': 0.97,   # clamping max

            # ---- Z filter (world frame) ----
            # At 5m altitude we want to see from the ground to ~10m up.
            # Keeping the ground in the cloud helps filter_ground work.
            'pointcloud_min_z': 0.5,
            'pointcloud_max_z': 10.0,

            # ---- 2D projection Z band ----
            # Only voxels in this band are projected into /projected_map.
            # Start at 2m to skip low ground clutter (grass, curbs, bumps)
            # that get through the ground filter outdoors.
            # 2–8m covers the drone's flight envelope at 5m altitude.
            'occupancy_min_z': 2.0,
            'occupancy_max_z': 8.0,

            # ---- Ground filtering ----
            # Outdoors with grass/uneven terrain, widen the tolerances so
            # RANSAC accepts more points as ground instead of classifying
            # low clutter as obstacles.
            'filter_ground': True,
            'ground_filter.distance': 0.2,
            'ground_filter.plane_distance': 0.12,
            'ground_filter.angle': 0.30,

            # ---- Misc ----
            'latch': False,
            'publish_free_space': False,
            'colored_map.enabled': False,
        }],
        remappings=[
            ('cloud_in', '/zed2/zed_node/point_cloud/cloud_registered'),
        ],
    )

    return LaunchDescription([
        resolution_arg,
        max_range_arg,
        frame_id_arg,
        base_frame_arg,

        octomap_server_node,
    ])

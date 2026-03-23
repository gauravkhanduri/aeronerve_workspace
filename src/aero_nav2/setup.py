from glob import glob

from setuptools import find_packages, setup

package_name = 'aero_nav2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # Install Nav2 parameter configs and RViz config
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grv22',
    maintainer_email='gauravkhanduri93@gmail.com',
    description='Nav2 integration layer for PX4/MAVROS drone',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # Sensor bridge: depth image → PointCloud2
            'depth_to_costmap = aero_nav2.depth_to_costmap:main',
            # Controller bridge: cmd_vel → MAVROS setpoint_position
            'nav2_mavros_bridge = aero_nav2.nav2_mavros_bridge:main',
            # Nav2 mission client (original Nav2 stack)
            'nav2_mission_client = aero_nav2.nav2_mission_client:main',
            # ── Custom nav stack ──────────────────────────────────────────
            # 2D occupancy grid builder from depth pointcloud
            'occupancy_map = aero_nav2.occupancy_map:main',
            # 2D A* global planner
            'global_planner = aero_nav2.global_planner:main',
            # Holonomic DWA local planner
            'local_planner = aero_nav2.local_planner:main',
            # Simple goal sender and monitor
            'mission_client = aero_nav2.mission_client:main',
        ],
    },
)

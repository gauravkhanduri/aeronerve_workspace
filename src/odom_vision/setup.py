from setuptools import find_packages, setup

package_name = 'odom_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pose.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grv22',
    maintainer_email='gauravkhanduri93@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_to_vision_pose = odom_vision.odom_to_vision_pose:main',
            'pose_comparer = odom_vision.pose_comparer:main',
            'tf_publisher = odom_vision.tf_publisher:main',
            'figure8_trajectory = odom_vision.figure8_trajectory:main',
            'astar_planner = odom_vision.astar_planner:main',
            'apf_planner = odom_vision.apf_planner:main',
            'depth_distance = odom_vision.depth_distance:main',
            'hybrid_planner = odom_vision.hybrid_planner:main',
            'keyboard_control = odom_vision.keyboard_control:main',
            'grid_control = odom_vision.grid_control:main',
        ],
    },
)

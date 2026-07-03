from setuptools import find_packages
from setuptools import setup

package_name = "odom_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/nav2_drone.yaml",
            "config/drone_nav_bt.xml",
        ]),
        ("share/" + package_name + "/launch", [
            "launch/pose.launch.py",
            "launch/drone.launch.py",
            "launch/health_check.py",
            "launch/octomap_mapping.launch.py",
            "launch/planner_executor.launch.py",
            "launch/nav2_drone.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="grv22",
    maintainer_email="gauravkhanduri93@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "odom_to_vision_pose = odom_vision.odom_to_vision_pose:main",
            "pose_comparer = odom_vision.pose_comparer:main",
            "tf_publisher = odom_vision.tf_publisher:main",
            "figure8_trajectory = odom_vision.figure8_trajectory:main",
            "astar_planner = odom_vision.astar_planner:main",
            "apf_planner = odom_vision.apf_planner:main",
            "depth_distance = odom_vision.depth_distance:main",
            "hybrid_planner = odom_vision.hybrid_planner:main",
            "keyboard_control = odom_vision.keyboard_control:main",
            "grid_control = odom_vision.grid_control:main",
            "gcs_watchdog = odom_vision.gcs_watchdog:main",
            "takeoff_land = odom_vision.takeoff_land:main",
            "square_waypoints = odom_vision.square_waypoints:main",
            "mission_sequencer = odom_vision.mission_sequencer:main",
            "drone_state_broadcaster = odom_vision.drone_state_broadcaster:main",
            "health_check = odom_vision.health_check:main",
            "global_planner_node = odom_vision.global_planner_node:main",
            "mission_executor_node = odom_vision.mission_executor_node:main",
            "test_send_goal = odom_vision.test_send_goal:main",
            "reactive_nav_node = odom_vision.reactive_nav_node:main",
            "reactive_nav_node_updated = odom_vision.reactive_nav_node_updated:main",
            "send_move = odom_vision.send_move:main",
            "send_forward_goal = odom_vision.send_forward_goal:main",
            "cmd_vel_bridge = odom_vision.cmd_vel_bridge:main",
            "odom_qos_relay = odom_vision.odom_qos_relay:main",
        ],
    },
)

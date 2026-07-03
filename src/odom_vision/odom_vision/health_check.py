#!/usr/bin/env python3
"""
ISR Drone System Health Check
Run this script after launching the full system to verify everything is working.

Usage:
    ros2 run isr_drone_bringup health_check.py
    # or simply:
    python3 health_check.py
"""

import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Header


class HealthCheckNode(Node):
    """Checks all critical topics and TF for the ISR drone system."""

    def __init__(self):
        super().__init__('health_check')

        self.results = {}
        self.start_time = time.time()
        self.timeout = 10.0  # seconds to wait for each topic

        # QoS for sensor topics (best effort for high-frequency data)
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # ---- Topics to check ----
        self.checks = {
            'zed_pointcloud': {
                'topic': '/zed/zed_node/point_cloud/cloud_registered',
                'msg_type': PointCloud2,
                'qos': sensor_qos,
                'received': False,
                'hz': 0.0,
                'last_time': None,
                'count': 0,
                'description': 'ZED 2i pointcloud',
            },
            'zed_pose': {
                'topic': '/zed/zed_node/pose',
                'msg_type': PoseStamped,
                'qos': sensor_qos,
                'received': False,
                'hz': 0.0,
                'last_time': None,
                'count': 0,
                'description': 'ZED VIO pose',
            },
            'zed_odom': {
                'topic': '/zed/zed_node/odom',
                'msg_type': Odometry,
                'qos': sensor_qos,
                'received': False,
                'hz': 0.0,
                'last_time': None,
                'count': 0,
                'description': 'ZED odometry',
            },
            'octomap_projected': {
                'topic': '/projected_map',
                'msg_type': OccupancyGrid,
                'qos': QoSProfile(depth=1),
                'received': False,
                'hz': 0.0,
                'last_time': None,
                'count': 0,
                'description': 'OctoMap 2D projected grid',
            },
            'mavros_state': {
                'topic': '/mavros/state',
                'msg_type': State,
                'qos': QoSProfile(depth=1),
                'received': False,
                'hz': 0.0,
                'last_time': None,
                'count': 0,
                'description': 'MAVROS connection state',
            },
            'mavros_local_pos': {
                'topic': '/mavros/local_position/pose',
                'msg_type': PoseStamped,
                'qos': sensor_qos,
                'received': False,
                'hz': 0.0,
                'last_time': None,
                'count': 0,
                'description': 'MAVROS local position (FCU EKF)',
            },
            'vision_pose_out': {
                'topic': '/mavros/vision_pose/pose',
                'msg_type': PoseStamped,
                'qos': QoSProfile(depth=1),
                'received': False,
                'hz': 0.0,
                'last_time': None,
                'count': 0,
                'description': 'Vision pose → MAVROS bridge',
            },
        }

        # Create subscribers
        for key, check in self.checks.items():
            self.create_subscription(
                check['msg_type'],
                check['topic'],
                lambda msg, k=key: self._callback(k, msg),
                check['qos'],
            )

        # Diagnostic data from specific topics
        self.mavros_connected = False
        self.mavros_armed = False
        self.mavros_mode = 'UNKNOWN'
        self.grid_info = None
        self.pointcloud_width = 0
        self.vio_position = None

        # Timer to print results
        self.create_timer(1.0, self._print_status)
        self.check_count = 0

    def _callback(self, key: str, msg):
        """Generic callback for all checked topics."""
        check = self.checks[key]
        now = time.time()

        check['received'] = True
        check['count'] += 1

        if check['last_time'] is not None:
            dt = now - check['last_time']
            if dt > 0:
                # Exponential moving average of Hz
                instant_hz = 1.0 / dt
                check['hz'] = 0.7 * check['hz'] + 0.3 * instant_hz
        check['last_time'] = now

        # Extract specific diagnostic info
        if key == 'mavros_state':
            self.mavros_connected = msg.connected
            self.mavros_armed = msg.armed
            self.mavros_mode = msg.mode

        elif key == 'octomap_projected':
            self.grid_info = {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin_x': msg.info.origin.position.x,
                'origin_y': msg.info.origin.position.y,
            }

        elif key == 'zed_pointcloud':
            self.pointcloud_width = msg.width * msg.height

        elif key == 'zed_pose':
            p = msg.pose.position
            self.vio_position = (p.x, p.y, p.z)

    def _print_status(self):
        """Print diagnostic status every second."""
        self.check_count += 1
        elapsed = time.time() - self.start_time

        print('\n' + '=' * 65)
        print(f'  ISR DRONE HEALTH CHECK  (elapsed: {elapsed:.0f}s)')
        print('=' * 65)

        all_ok = True

        for key, check in self.checks.items():
            if check['received']:
                status = f'\033[92mOK\033[0m'  # green
                hz_str = f'{check["hz"]:.1f} Hz'
            else:
                status = f'\033[91mNO DATA\033[0m'  # red
                hz_str = '---'
                all_ok = False

            print(f'  [{status}] {check["description"]:<35} {hz_str:<12} {check["topic"]}')

        # Additional diagnostics
        print('\n  --- System Status ---')

        if self.mavros_connected:
            print(f'  FCU: \033[92mConnected\033[0m | Mode: {self.mavros_mode} | Armed: {self.mavros_armed}')
        else:
            print(f'  FCU: \033[91mNot Connected\033[0m')
            all_ok = False

        if self.vio_position:
            x, y, z = self.vio_position
            print(f'  VIO Position: x={x:.2f}m  y={y:.2f}m  z={z:.2f}m')

        if self.pointcloud_width > 0:
            print(f'  Pointcloud: {self.pointcloud_width:,} points')

        if self.grid_info:
            g = self.grid_info
            area = g['width'] * g['height'] * g['resolution'] ** 2
            print(f'  OctoMap Grid: {g["width"]}x{g["height"]} cells, '
                  f'{g["resolution"]}m res, ~{area:.0f}m² coverage')
        else:
            print(f'  OctoMap Grid: \033[93mWaiting...\033[0m (takes 10-30s to build)')

        # Readiness check
        print('\n  --- Flight Readiness ---')
        checks_passed = sum(1 for c in self.checks.values() if c['received'])
        total_checks = len(self.checks)
        print(f'  Topics: {checks_passed}/{total_checks} active')

        critical_ok = (
            self.checks['zed_pointcloud']['received']
            and self.checks['zed_pose']['received']
            and self.checks['mavros_state']['received']
            and self.mavros_connected
            and self.checks['vision_pose_out']['received']
        )

        if critical_ok and self.grid_info:
            print(f'  \033[92m>>> SYSTEM READY FOR FLIGHT <<<\033[0m')
        elif critical_ok:
            print(f'  \033[93m>>> ALMOST READY — waiting for OctoMap to build <<<\033[0m')
        else:
            missing = [c['description'] for c in self.checks.values() if not c['received']]
            print(f'  \033[91m>>> NOT READY — missing: {", ".join(missing)} <<<\033[0m')

        # Auto-exit after 30 seconds
        if self.check_count >= 30:
            print('\n  Health check complete. Press Ctrl+C to exit.')
            if not critical_ok:
                print('  \033[91m  WARNING: System is not fully operational!\033[0m')


def main():
    rclpy.init()
    node = HealthCheckNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

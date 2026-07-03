#!/usr/bin/env python3
# Author: Gourav Khanduri
# Email:  gauravkhanduri93@gmail.com
"""
drone_state_broadcaster.py

Companion-side UDP broadcaster that streams drone state to the GCS.

Subscribes to:
    /mavros/local_position/pose  (geometry_msgs/PoseStamped)
    /mavros/state                (mavros_msgs/State)

Broadcasts JSON at ~20 Hz to the registered GCS via UDP (NDJSON — one line per message).

Registration flow:
    1. GCS sends any datagram (e.g. b'hello\\n') to drone_ip:port
    2. Drone records GCS source address and begins sending state datagrams to it

JSON format sent to GCS:
    {
      "msg_type": "drone_state",
      "timestamp_utc": "2026-03-20T15:30:45Z",
      "pose": {
        "x_m":     1.23,
        "y_m":     4.56,
        "z_m":     3.00,
        "yaw_deg": 45.2
      },
      "state": {
        "connected":     true,
        "armed":         false,
        "mode":          "GUIDED",
        "system_status": 3
      }
    }

Usage:
    ros2 run odom_vision drone_state_broadcaster \
        --ros-args -p port:=5761 -p broadcast_hz:=20.0
"""

import datetime
import json
import math
import socket
import threading

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _quat_to_yaw_deg(qx, qy, qz, qw) -> float:
    yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return math.degrees(yaw_rad)


class DroneStateBroadcaster(Node):

    def __init__(self):
        super().__init__('drone_state_broadcaster')

        self.declare_parameter('port', 5761)
        self.declare_parameter('broadcast_hz', 20.0)

        port         = self.get_parameter('port').value
        broadcast_hz = self.get_parameter('broadcast_hz').value

        self._lock     = threading.Lock()
        self._pose     = None
        self._state    = None
        self._gcs_addr = None  # (ip, port) set when GCS sends registration packet

        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, mavros_qos)
        self.create_subscription(
            State, '/mavros/state', self._state_cb, 10)

        self.create_timer(1.0 / broadcast_hz, self._broadcast_cb)

        # Single UDP socket: bound to receive GCS registration + send datagrams back
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('0.0.0.0', port))

        threading.Thread(target=self._reg_listen_loop, daemon=True).start()

        self.get_logger().info(
            f'Drone state broadcaster started — UDP port {port}, {broadcast_hz:.1f} Hz')

    def _pose_cb(self, msg: PoseStamped) -> None:
        with self._lock:
            self._pose = msg

    def _state_cb(self, msg: State) -> None:
        with self._lock:
            self._state = msg

    def _reg_listen_loop(self) -> None:
        """Wait for GCS registration datagrams and record the sender address."""
        self.get_logger().info('UDP: waiting for GCS registration packet...')
        while True:
            try:
                _, addr = self._sock.recvfrom(1024)
                with self._lock:
                    if self._gcs_addr != addr:
                        self._gcs_addr = addr
                        self.get_logger().info(f'GCS registered: {addr[0]}:{addr[1]}')
            except OSError:
                break

    def _broadcast_cb(self) -> None:
        with self._lock:
            pose     = self._pose
            state    = self._state
            gcs_addr = self._gcs_addr

        if gcs_addr is None:
            return

        payload = self._build_payload(pose, state)
        line    = (json.dumps(payload) + '\n').encode()

        try:
            self._sock.sendto(line, gcs_addr)
        except OSError as e:
            self.get_logger().warning(f'UDP send error: {e}')

    def _build_payload(self, pose: PoseStamped, state: State) -> dict:
        ts = datetime.datetime.utcnow().isoformat() + 'Z'

        pose_dict = {'x_m': 0.0, 'y_m': 0.0, 'z_m': 0.0, 'yaw_deg': 0.0}
        if pose is not None:
            p = pose.pose.position
            q = pose.pose.orientation
            pose_dict = {
                'x_m':     round(p.x, 4),
                'y_m':     round(p.y, 4),
                'z_m':     round(p.z, 4),
                'yaw_deg': round(_quat_to_yaw_deg(q.x, q.y, q.z, q.w), 2),
            }

        state_dict = {'connected': False, 'armed': False, 'mode': 'UNKNOWN', 'system_status': 0}
        if state is not None:
            state_dict = {
                'connected':     bool(state.connected),
                'armed':         bool(state.armed),
                'mode':          str(state.mode),
                'system_status': int(state.system_status),
            }

        return {
            'msg_type':      'drone_state',
            'timestamp_utc': ts,
            'pose':          pose_dict,
            'state':         state_dict,
        }


def main(args=None):
    rclpy.init(args=args)
    node = DroneStateBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
drone_state_broadcaster.py

Companion-side TCP server that streams drone state to the GCS.

Subscribes to:
    /mavros/local_position/pose  (geometry_msgs/PoseStamped)
    /mavros/state                (mavros_msgs/State)

Broadcasts JSON at ~5 Hz to all connected TCP clients (NDJSON — one line per message).

JSON format sent to GCS:
    {
      "msg_type": "drone_state",
      "timestamp_utc": "2026-03-20T15:30:45Z",
      "pose": {
        "x_m":    1.23,
        "y_m":    4.56,
        "z_m":    3.00,
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
        --ros-args -p port:=5761 -p broadcast_hz:=5.0
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
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def _quat_to_yaw_deg(qx, qy, qz, qw) -> float:
    """Extract yaw in degrees from quaternion (ENU frame)."""
    yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return math.degrees(yaw_rad)


class DroneStateBroadcaster(Node):

    def __init__(self):
        super().__init__("drone_state_broadcaster")

        self.declare_parameter("port", 5761)
        self.declare_parameter("broadcast_hz", 5.0)

        port = self.get_parameter("port").value
        broadcast_hz = self.get_parameter("broadcast_hz").value

        self._lock = threading.Lock()
        self._pose = None  # latest PoseStamped
        self._state = None  # latest mavros State
        self._clients: list[socket.socket] = []

        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, mavros_qos
        )

        self.create_subscription(State, "/mavros/state", self._state_cb, 10)

        self.create_timer(1.0 / broadcast_hz, self._broadcast_cb)

        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind(("0.0.0.0", port))
        self._server_sock.listen(5)

        threading.Thread(target=self._accept_loop, daemon=True).start()

        self.get_logger().info(
            f"Drone state broadcaster started — TCP port {port}, " f"{broadcast_hz:.1f} Hz"
        )

    def _pose_cb(self, msg: PoseStamped) -> None:
        with self._lock:
            self._pose = msg

    def _state_cb(self, msg: State) -> None:
        with self._lock:
            self._state = msg

    def _accept_loop(self) -> None:
        self.get_logger().info("TCP server: waiting for GCS connections...")
        while True:
            try:
                conn, addr = self._server_sock.accept()
                with self._lock:
                    self._clients.append(conn)
                self.get_logger().info(f"GCS connected: {addr[0]}:{addr[1]}")
            except Exception as e:
                self.get_logger().error(f"Accept error: {e}")
                break

    def _broadcast_cb(self) -> None:
        with self._lock:
            pose = self._pose
            state = self._state
            clients = list(self._clients)

        if not clients:
            return

        payload = self._build_payload(pose, state)
        line = (json.dumps(payload) + "\n").encode()

        dead = []
        for conn in clients:
            try:
                conn.sendall(line)
            except (BrokenPipeError, ConnectionResetError, OSError):
                dead.append(conn)

        if dead:
            with self._lock:
                for conn in dead:
                    self._clients.remove(conn)
                    self.get_logger().info("GCS disconnected.")

    def _build_payload(self, pose: PoseStamped, state: State) -> dict:
        ts = datetime.datetime.utcnow().isoformat() + "Z"

        pose_dict = {"x_m": 0.0, "y_m": 0.0, "z_m": 0.0, "yaw_deg": 0.0}
        if pose is not None:
            p = pose.pose.position
            q = pose.pose.orientation
            pose_dict = {
                "x_m": round(p.x, 4),
                "y_m": round(p.y, 4),
                "z_m": round(p.z, 4),
                "yaw_deg": round(_quat_to_yaw_deg(q.x, q.y, q.z, q.w), 2),
            }

        state_dict = {
            "connected": False,
            "armed": False,
            "mode": "UNKNOWN",
            "system_status": 0,
        }
        if state is not None:
            state_dict = {
                "connected": bool(state.connected),
                "armed": bool(state.armed),
                "mode": str(state.mode),
                "system_status": int(state.system_status),
            }

        return {
            "msg_type": "drone_state",
            "timestamp_utc": ts,
            "pose": pose_dict,
            "state": state_dict,
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


if __name__ == "__main__":
    main()
if __name__ == "__main__":
    main()

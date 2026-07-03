#!/usr/bin/env python3
# Author: Gourav Khanduri
# Email:  gauravkhanduri93@gmail.com
"""
mission_sequencer.py

Receives a GCS-generated mission JSON (via TCP or file) and executes it
via MAVROS setpoint_position/local on ArduPilot GUIDED mode.

Modes
-----
TCP mode (real drone):
    ros2 run odom_vision mission_sequencer --ros-args -p tcp_port:=5760
    Listens on 0.0.0.0:5760, accepts one GCS connection, receives mission
    JSON, sends ACK, then executes the mission.

File mode (simulation / testing):
    ros2 run odom_vision mission_sequencer \
        --ros-args -p mission_file:=/home/grv22/gcs/missions/xxx.json

JSON structure (gcs/core/tcp_sender.py build_mission_json):
    {
      "parameters": { "altitude_m", "speed_mps", "loiter_time_s", "return_home" },
      "waypoints":  [ survey waypoints ],
      "exit_point": { RTH waypoint }   ← also accepts "return_home" key
    }

Sequence
--------
    1. Load mission (TCP or file)
    2. Wait for MAVROS local position
    3. Arm
    4. Takeoff to mission altitude
    5. Wait until altitude confirmed
    6. Fly each survey waypoint (virtual setpoint stepping)
    7. RTH if exit_point present and return_home=true
    8. Land
"""

import datetime
import json
import math
import os
import socket
import threading
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MissionSequencer(Node):

    def __init__(self):
        super().__init__('mission_sequencer')

        self.declare_parameter('mission_file', '')
        self.declare_parameter('tcp_port', 0)              # 0 = disabled (use file)
        self.declare_parameter('arrival_radius', 0.5)    # metres
        self.declare_parameter('altitude_tolerance', 2.2)  # metres
        self.declare_parameter('step_hz', 10.0)
        self.declare_parameter('save_dir', os.path.expanduser('~/missions'))

        mission_file   = self.get_parameter('mission_file').value
        tcp_port       = self.get_parameter('tcp_port').value
        self.arrival_r = self.get_parameter('arrival_radius').value
        self.alt_tol   = self.get_parameter('altitude_tolerance').value
        self.step_hz   = self.get_parameter('step_hz').value
        self._save_dir = self.get_parameter('save_dir').value

        # ── Load mission ──────────────────────────────────────────────────────
        if tcp_port > 0:
            self._mission = self._receive_via_tcp(tcp_port)
        elif mission_file:
            self.get_logger().info(f'Loading mission file: {mission_file}')
            with open(mission_file, 'r') as f:
                self._mission = json.load(f)
        else:
            self.get_logger().fatal(
                'Neither tcp_port nor mission_file parameter set. Aborting.')
            raise SystemExit(1)

        # ── Save mission to disk ──────────────────────────────────────────────
        self._save_mission(self._mission)

        # ── Parse parameters ──────────────────────────────────────────────────
        params        = self._mission.get('parameters', {})
        self.altitude = float(params.get('altitude_m', 10.0))
        self.speed    = float(params.get('speed_mps', 1.0))
        self.loiter_s = float(params.get('loiter_time_s', 0.0))
        self.do_rth   = bool(params.get('return_home', True))

        self._wps = self._mission.get('waypoints', [])
        if not self._wps:
            self.get_logger().fatal('No waypoints in mission. Aborting.')
            raise SystemExit(1)

        # Accept both key names for the RTH waypoint
        self._rth = (self._mission.get('exit_point')
                     or self._mission.get('return_home')
                     if self.do_rth else None)
        # Guard: if return_home key was a bool (from parameters), ignore it
        if not isinstance(self._rth, dict):
            self._rth = None

        self.get_logger().info(
            f'Mission loaded: {len(self._wps)} survey waypoints, '
            f'alt={self.altitude}m, speed={self.speed}m/s, '
            f'loiter={self.loiter_s}s, RTH={self.do_rth}')

        # ── MAVROS setup ──────────────────────────────────────────────────────
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self._setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self._pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, mavros_qos)
        self._state_sub = self.create_subscription(
            State, '/mavros/state', self._state_cb, 10)

        self._arm_client     = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._takeoff_client = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')
        self._land_client    = self.create_client(CommandTOL,  '/mavros/cmd/land')

        self._current_pose   = PoseStamped()
        self._pose_received  = False
        self._current_mode   = ''
        self._state_received = False

        threading.Thread(target=self._sequence, daemon=True).start()

    # ── Save mission ──────────────────────────────────────────────────────────

    def _save_mission(self, mission: dict) -> None:
        os.makedirs(self._save_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        mission_id = mission.get('mission_id', 'unknown')
        path = os.path.join(self._save_dir, f'mission_{ts}_{mission_id}.json')
        with open(path, 'w') as f:
            json.dump(mission, f, indent=2)
        self.get_logger().info(f'Mission saved to {path}')

    # ── TCP receive ───────────────────────────────────────────────────────────

    def _receive_via_tcp(self, port: int) -> dict:
        """Block until GCS connects, receive one mission JSON, return parsed dict."""
        self.get_logger().info(f'TCP mode: listening on 0.0.0.0:{port} ...')

        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', port))
        srv.listen(1)

        conn, addr = srv.accept()
        self.get_logger().info(f'GCS connected from {addr[0]}:{addr[1]}')

        with conn:
            raw = conn.makefile().readline()
            if not raw.strip():
                raise RuntimeError('Empty mission received from GCS')

            mission = json.loads(raw)
            wp_count = len(mission.get('waypoints', []))

            # Send ACK immediately so GCS knows mission was received
            ack = json.dumps({
                'msg_type':      'ack',
                'mission_id':    mission.get('mission_id', ''),
                'status':        'accepted',
                'waypoint_count': wp_count,
            }) + '\n'
            conn.sendall(ack.encode())
            self.get_logger().info(
                f'Mission received: {wp_count} survey waypoints. ACK sent.')

        srv.close()
        return mission

    # ── MAVROS callbacks ──────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._current_pose = msg
        self._pose_received = True

    def _state_cb(self, msg: State) -> None:
        self._current_mode = msg.mode
        self._state_received = True

    def _is_guided(self) -> bool:
        return self._current_mode == 'GUIDED'

    def _call(self, client, request, name: str):
        client.wait_for_service(timeout_sec=10.0)
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        result = future.result()
        if result is None:
            self.get_logger().error(f'{name}: no response')
        else:
            self.get_logger().info(f'{name}: result={result}')
        return result

    # ── Setpoint helpers ──────────────────────────────────────────────────────

    def _send_setpoint(self, x: float, y: float, z: float, yaw_deg: float = 0.0) -> None:
        yaw_rad = math.radians(yaw_deg)
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self._setpoint_pub.publish(msg)

    def _dist_to(self, x: float, y: float, z: float) -> float:
        cx = self._current_pose.pose.position.x
        cy = self._current_pose.pose.position.y
        cz = self._current_pose.pose.position.z
        return math.sqrt((cx - x)**2 + (cy - y)**2 + (cz - z)**2)

    def _go_to(self, x: float, y: float, z: float, yaw_deg: float, label: str) -> bool:
        """Step virtual setpoint toward target at self.speed. Block until arrived.

        Returns True if waypoint reached, False if aborted (mode left GUIDED).
        """
        self.get_logger().info(
            f'-> {label}  target=({x:.2f}, {y:.2f}, {z:.2f})m  yaw={yaw_deg:.1f} deg')

        dt   = 1.0 / self.step_hz
        vx   = self._current_pose.pose.position.x
        vy   = self._current_pose.pose.position.y
        vz   = self._current_pose.pose.position.z
        yaw_r = math.radians(yaw_deg)

        while rclpy.ok():
            if not self._is_guided():
                self.get_logger().error(
                    f'Mode changed to {self._current_mode} during {label}. '
                    f'Aborting mission.')
                return False

            dx, dy, dz = x - vx, y - vy, z - vz
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            step = self.speed * dt

            if abs(dx) > 1e-3 or abs(dy) > 1e-3:
                yaw_r = math.atan2(dy, dx)

            if dist <= step:
                vx, vy, vz = x, y, z
            else:
                scale = step / dist
                vx += dx * scale
                vy += dy * scale
                vz += dz * scale

            self._send_setpoint(vx, vy, vz, math.degrees(yaw_r))

            if self._pose_received and self._dist_to(x, y, z) < self.arrival_r:
                self.get_logger().info(
                    f'Reached {label}. Loitering {self.loiter_s:.1f}s...')
                t_end = time.time() + self.loiter_s
                while rclpy.ok() and time.time() < t_end:
                    if not self._is_guided():
                        self.get_logger().error(
                            f'Mode changed to {self._current_mode} during '
                            f'loiter at {label}. Aborting mission.')
                        return False
                    self._send_setpoint(x, y, z, math.degrees(yaw_r))
                    time.sleep(dt)
                return True

            time.sleep(dt)
        return True

    # ── Main sequence ─────────────────────────────────────────────────────────

    def _sequence(self) -> None:
        time.sleep(3.0)  # let MAVROS connect

        # 1. Wait for local position and GUIDED mode
        self.get_logger().info('Waiting for /mavros/local_position/pose...')
        while rclpy.ok() and not self._pose_received:
            time.sleep(0.2)
        p = self._current_pose.pose.position
        self.get_logger().info(f'Pose received: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

        self.get_logger().info('Waiting for GUIDED mode...')
        while rclpy.ok() and not self._is_guided():
            time.sleep(0.2)
        self.get_logger().info(f'Mode confirmed: {self._current_mode}')

        # 2. Arm
        self.get_logger().info('Arming...')
        req = CommandBool.Request()
        req.value = True
        self._call(self._arm_client, req, 'arming')
        time.sleep(2.0)

        # 3. Takeoff
        self.get_logger().info(f'Taking off to {self.altitude:.1f}m AGL...')
        req = CommandTOL.Request()
        req.altitude  = self.altitude
        req.min_pitch = 0.0
        req.yaw       = 0.0
        req.latitude  = 0.0
        req.longitude = 0.0
        self._call(self._takeoff_client, req, 'takeoff')

        # 4. Wait for altitude
        self.get_logger().info(
            f'Climbing... target={self.altitude:.1f}m +/-{self.alt_tol:.2f}m')
        while rclpy.ok():
            if self._pose_received:
                z = self._current_pose.pose.position.z
                if abs(z - self.altitude) <= self.alt_tol:
                    self.get_logger().info(f'Altitude confirmed: z={z:.2f}m')
                    break
            time.sleep(0.2)

        # 5. Survey waypoints
        aborted = False
        self.get_logger().info(f'Starting survey: {len(self._wps)} waypoints...')
        for i, wp in enumerate(self._wps):
            if not self._is_guided():
                self.get_logger().error(
                    f'Mode is {self._current_mode} before WP {i+1}. '
                    f'Aborting mission.')
                aborted = True
                break
            reached = self._go_to(
                x       = float(wp['x_m']),
                y       = float(wp['y_m']),
                z       = float(wp.get('z_m', self.altitude)),
                yaw_deg = float(wp.get('yaw_deg', 0.0)),
                label   = f'WP {i+1}/{len(self._wps)} (id={wp["id"]})',
            )
            if not reached:
                aborted = True
                break

        if not aborted:
            self.get_logger().info('Survey complete.')

            # 6. RTH
            if self.do_rth and self._rth is not None:
                self.get_logger().info('Returning to home...')
                reached = self._go_to(
                    x       = float(self._rth['x_m']),
                    y       = float(self._rth['y_m']),
                    z       = float(self._rth.get('z_m', self.altitude)),
                    yaw_deg = 0.0,
                    label   = 'HOME',
                )
                if not reached:
                    aborted = True

        # 7. Land
        self.get_logger().info('Landing...')
        req = CommandTOL.Request()
        req.altitude  = 0.0
        req.min_pitch = 0.0
        req.yaw       = 0.0
        req.latitude  = 0.0
        req.longitude = 0.0
        self._call(self._land_client, req, 'land')

        if aborted:
            self.get_logger().error(
                'Mission ABORTED — mode left GUIDED. '
                'Send a new mission from GCS to retry.')
        else:
            self.get_logger().info('Mission complete.')


def main(args=None):
    rclpy.init(args=args)
    node = MissionSequencer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
mission_sequencer.py

Reads a GCS-generated mission JSON and executes it via MAVROS setpoint_position/local.
Follows the same pattern as square_waypoints.py (virtual setpoint stepping, arming service).

Usage:
    ros2 run odom_vision mission_sequencer 
        --ros-args -p mission_file:=/home/grv22/gcs/missions/20260320_212552.json

JSON structure expected (from gcs/core/tcp_sender.py build_mission_json):
    {
      "parameters": { "altitude_m", "speed_mps", "loiter_time_s", "return_home" },
      "home":        { takeoff position — always (0,0,alt) },
      "waypoints":   [ all survey waypoints ],
      "return_home": { RTH waypoint or null }
    }

Sequence:
    1. Load JSON
    2. Wait for MAVROS local position
    3. Arm
    4. Takeoff to mission altitude
    5. Wait until altitude confirmed
    6. Fly each waypoint (virtual setpoint stepping at speed_mps)
    7. RTH if return_home present
    8. Land
"""

import json
import math
import threading
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandTOL
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MissionSequencer(Node):

    def __init__(self):
        super().__init__('mission_sequencer')

        self.declare_parameter('mission_file', '')
        self.declare_parameter('arrival_radius', 0.3)    # metres — waypoint acceptance
        self.declare_parameter('altitude_tolerance', 0.3) # metres — takeoff confir # metres — waypoint acceptancemation
        self.declare_parameter('step_hz', 10.0)           # control loop rate

        mission_file     = self.get_parameter('mission_file').value
        self.arrival_r   = self.get_parameter('arrival_radius').value
        self.alt_tol     = self.get_parameter('altitude_tolerance').value
        self.step_hz     = self.get_parameter('step_hz').value

        if not mission_file:
            self.get_logger().fatal('mission_file parameter not set. Aborting.')
            raise SystemExit(1)

        self.get_logger().info(f'Loading mission: {mission_file}')
        with open(mission_file, 'r') as f:
            self._mission = json.load(f)

        params           = self._mission.get('parameters', {})
        self.altitude    = float(params.get('altitude_m', 10.0))
        self.speed       = float(params.get('speed_mps', 1.0))
        self.loiter_s    = float(params.get('loiter_time_s', 0.0))
        self.do_rth      = bool(params.get('return_home', True))

        self._wps = self._mission.get('waypoints', [])
        if not self._wps:
            self.get_logger().fatal('No waypoints in mission file. Aborting.')
            raise SystemExit(1)

        # RTH waypoint is under 'exit_point' key in GCS JSON
        self._rth = self._mission.get('exit_point')

        self.get_logger().info(
            f'Mission loaded: {len(self._wps)} survey waypoints, '
            f'alt={self.altitude}m, speed={self.speed}m/s, '
            f'loiter={self.loiter_s}s, RTH={self.do_rth}')

        
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self._setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self._pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, mavros_qos)

        self._arm_client     = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._takeoff_client = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')
        self._land_client    = self.create_client(CommandTOL,  '/mavros/cmd/land')

        
        self._current_pose = PoseStamped()
        self._pose_received = False

        
        threading.Thread(target=self._sequence, daemon=True).start()

   
    def _pose_cb(self, msg: PoseStamped) -> None:
        self._current_pose = msg
        self._pose_received = True


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

   

    def _send_setpoint(self, x: float, y: float, z: float, yaw_deg: float = 0.0) -> None:
        """Publish a PoseStamped setpoint. yaw_deg in ENU (0 = East, CCW positive)."""
        yaw_rad = math.radians(yaw_deg)
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self._setpoint_pub.publish(msg)


    def _dist_to(self, x: float, y: float, z: float) -> float:
        cx = self._current_pose.pose.position.x
        cy = self._current_pose.pose.position.y
        cz = self._current_pose.pose.position.z
        return math.sqrt((cx - x)**2 + (cy - y)**2 + (cz - z)**2)

    

    def _go_to(self, x: float, y: float, z: float, yaw_deg: float, label: str) -> None:
        """
        Move a virtual setpoint from current position toward (x, y, z) at self.speed.
        Blocks until drone arrives within arrival_radius, then dwells loiter_s seconds.
        """
        self.get_logger().info(
            f'→ {label}  target=({x:.2f}, {y:.2f}, {z:.2f})m  yaw={yaw_deg:.1f}°')

        dt = 1.0 / self.step_hz

        # Seed virtual setpoint at current drone position (no jump at start)
        vx = self._current_pose.pose.position.x
        vy = self._current_pose.pose.position.y
        vz = self._current_pose.pose.position.z
        yaw_r = math.radians(yaw_deg)

        while rclpy.ok():
            dx, dy, dz = x - vx, y - vy, z - vz
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            step = self.speed * dt

            # Update yaw toward direction of travel
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
                    self._send_setpoint(x, y, z, math.degrees(yaw_r))
                    time.sleep(dt)
                self.get_logger().info(f'Loiter done at {label}.')
                break

            time.sleep(dt)

    # ── Main sequence ──────────────────────────────────────────────────────────

    def _sequence(self) -> None:
        time.sleep(3.0)  # let MAVROS connect

        # 1. Wait for local position
        self.get_logger().info('Waiting for /mavros/local_position/pose...')
        while rclpy.ok() and not self._pose_received:
            time.sleep(0.2)
        self.get_logger().info(
            f'Pose received: ({self._current_pose.pose.position.x:.2f}, '
            f'{self._current_pose.pose.position.y:.2f}, '
            f'{self._current_pose.pose.position.z:.2f})')

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
            f'Climbing... target={self.altitude:.1f}m ±{self.alt_tol:.2f}m')
        while rclpy.ok():
            if self._pose_received:
                z = self._current_pose.pose.position.z
                if abs(z - self.altitude) <= self.alt_tol:
                    self.get_logger().info(f'Altitude confirmed: z={z:.2f}m')
                    break
            time.sleep(0.2)

        # 5. Execute survey waypoints
        self.get_logger().info(
            f'Starting survey: {len(self._wps)} waypoints...')
        for i, wp in enumerate(self._wps):
            label = f'WP {i+1}/{len(self._wps)} (id={wp["id"]})'
            self._go_to(
                x       = float(wp['x_m']),
                y       = float(wp['y_m']),
                z       = float(wp.get('z_m', self.altitude)),
                yaw_deg = float(wp.get('yaw_deg', 0.0)),
                label   = label,
            )

        self.get_logger().info('Survey complete.')

        # 6. Return to home (if enabled and present in JSON)
        if self.do_rth and self._rth is not None:
            self.get_logger().info('Returning to home...')
            self._go_to(
                x       = float(self._rth['x_m']),
                y       = float(self._rth['y_m']),
                z       = float(self._rth.get('z_m', self.altitude)),
                yaw_deg = 0.0,
                label   = 'HOME',
            )

        # 7. Land
        self.get_logger().info('Landing...')
        req = CommandTOL.Request()
        req.altitude  = 0.0
        req.min_pitch = 0.0
        req.yaw       = 0.0
        req.latitude  = 0.0
        req.longitude = 0.0
        self._call(self._land_client, req, 'land')

        self.get_logger().info('Mission complete.')


def main(args=None):
    rclpy.init(args=args)
    node = MissionSequencer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

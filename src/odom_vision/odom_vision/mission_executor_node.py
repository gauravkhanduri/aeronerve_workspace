#!/usr/bin/env python3
"""
Mission Executor Node for ISR Drone
ROS2 Humble

This is the node that actually flies the drone. It:
1. Receives the collision-free path from the global planner
2. Follows it point-by-point via /mavros/setpoint_position/local
3. Receives mission waypoints from the GUI via TCP
4. Sends telemetry back to the GUI

Subscribes:
    /planned_path               → nav_msgs/Path (from global planner)
    /mavros/local_position/pose → geometry_msgs/PoseStamped (drone position)
    /mavros/state               → mavros_msgs/State (FCU status)

Publishes:
    /mavros/setpoint_position/local → geometry_msgs/PoseStamped (flight commands)
    /goal_pose                      → geometry_msgs/PoseStamped (next goal for planner)
    /mission_waypoints              → nav_msgs/Path (full mission for planner)

Services called:
    /mavros/cmd/arming          → arm/disarm
    /mavros/set_mode            → switch to GUIDED
"""

import math
import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class MissionExecutorNode(Node):
    """Executes missions by following planner paths via MAVROS GUIDED mode (ArduPilot)."""

    def __init__(self):
        super().__init__('mission_executor')

        # ---- Parameters ----
        self.declare_parameter('acceptance_radius', 0.5)     # meters
        self.declare_parameter('flight_altitude', 5.0)        # meters
        self.declare_parameter('setpoint_rate', 20.0)         # Hz
        self.declare_parameter('tcp_port', 5555)              # GUI connection port

        self.acceptance_radius = self.get_parameter('acceptance_radius').value
        self.flight_altitude = self.get_parameter('flight_altitude').value
        self.setpoint_rate = self.get_parameter('setpoint_rate').value
        self.tcp_port = self.get_parameter('tcp_port').value

        # ---- State ----
        self.drone_pose = None                 # (x, y, z) from MAVROS
        self.mavros_state = State()            # FCU state
        self.current_path = []                 # List of PoseStamped from planner
        self.path_index = 0                    # Current index in path
        self.mission_waypoints = []            # GUI waypoints [(x,y,z,yaw), ...]
        self.mission_wp_index = 0              # Current mission waypoint
        self.mission_active = False
        self.current_setpoint = PoseStamped()  # What we're publishing to MAVROS

        # Initialize setpoint to origin (required before entering OFFBOARD)
        self.current_setpoint.pose.position.z = self.flight_altitude
        self.current_setpoint.pose.orientation.w = 1.0

        # ---- QoS ----
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ---- Subscribers ----
        self.create_subscription(
            Path, '/planned_path', self._path_callback, 10
        )
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._local_pos_callback, sensor_qos
        )
        self.create_subscription(
            State, '/mavros/state', self._state_callback, 10
        )

        # ---- Publishers ----
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
        self.mission_pub = self.create_publisher(
            Path, '/mission_waypoints', 10
        )

        # ---- Service clients ----
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # ---- Setpoint publishing timer (20Hz) ----
        period = 1.0 / self.setpoint_rate
        self.create_timer(period, self._publish_setpoint)

        # ---- Path following timer (10Hz) ----
        self.create_timer(0.1, self._follow_path)

        # ---- TCP server for GUI (runs in background thread) ----
        self.tcp_thread = threading.Thread(target=self._tcp_server, daemon=True)
        self.tcp_thread.start()

        self.get_logger().info(
            f'Mission executor started | '
            f'altitude={self.flight_altitude}m | '
            f'TCP port={self.tcp_port}'
        )

    # ================================================================
    # ROS Callbacks
    # ================================================================

    def _path_callback(self, msg: Path):
        """Receive new collision-free path from global planner."""
        if len(msg.poses) == 0:
            self.get_logger().info('Empty path received — mission may be complete')
            self.current_path = []
            self.path_index = 0
            return

        self.current_path = msg.poses
        self.path_index = 0
        self.get_logger().info(
            f'New path received: {len(self.current_path)} points'
        )

        # Set first point as current setpoint
        if self.current_path:
            self.current_setpoint = self.current_path[0]

    def _local_pos_callback(self, msg: PoseStamped):
        """Current drone position from MAVROS (EKF output)."""
        self.drone_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    def _state_callback(self, msg: State):
        """FCU state (armed, mode, connected)."""
        self.mavros_state = msg

    # ================================================================
    # Path Following Logic
    # ================================================================

    def _publish_setpoint(self):
        """Publish setpoint at 20Hz to MAVROS."""
        self.current_setpoint.header.stamp = self.get_clock().now().to_msg()
        self.current_setpoint.header.frame_id = 'map'
        self.setpoint_pub.publish(self.current_setpoint)

    def _follow_path(self):
        """Check if drone reached current path point, advance if so."""
        if not self.current_path or self.drone_pose is None:
            return

        if self.path_index >= len(self.current_path):
            # Reached end of planned path segment
            self._on_path_segment_complete()
            return

        # Current target point
        target = self.current_path[self.path_index]
        tx = target.pose.position.x
        ty = target.pose.position.y
        tz = target.pose.position.z

        # Distance to target
        dx = self.drone_pose[0] - tx
        dy = self.drone_pose[1] - ty
        dz = self.drone_pose[2] - tz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < self.acceptance_radius:
            self.path_index += 1
            if self.path_index < len(self.current_path):
                self.current_setpoint = self.current_path[self.path_index]
                self.get_logger().debug(
                    f'Path point {self.path_index}/{len(self.current_path)} reached, '
                    f'next: ({self.current_setpoint.pose.position.x:.1f}, '
                    f'{self.current_setpoint.pose.position.y:.1f})'
                )

    def _on_path_segment_complete(self):
        """Called when the planner's path is fully traversed."""
        if not self.mission_active:
            return

        self.mission_wp_index += 1

        if self.mission_wp_index < len(self.mission_waypoints):
            # Send next mission waypoint to the global planner
            wp = self.mission_waypoints[self.mission_wp_index]
            self.get_logger().info(
                f'Mission waypoint {self.mission_wp_index + 1}/'
                f'{len(self.mission_waypoints)}: '
                f'({wp["x"]:.1f}, {wp["y"]:.1f})'
            )
            self._send_goal_to_planner(wp['x'], wp['y'])
        else:
            self.get_logger().info('=== MISSION COMPLETE ===')
            self.mission_active = False
            # Hold position at last waypoint
            self._send_telemetry_to_gui({'type': 'mission_complete'})

    def _send_goal_to_planner(self, x: float, y: float):
        """Send a goal pose to the global planner node."""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = self.flight_altitude
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    # ================================================================
    # TCP Server for GUI Communication
    # ================================================================

    def _tcp_server(self):
        """Simple TCP server to receive commands from PyQt5 GUI."""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.tcp_port))
        server.listen(1)
        self.get_logger().info(f'TCP server listening on port {self.tcp_port}')

        while rclpy.ok():
            try:
                client, addr = server.accept()
                self.get_logger().info(f'GUI connected from {addr}')
                self._handle_gui_client(client)
            except Exception as e:
                self.get_logger().error(f'TCP error: {e}')

    def _handle_gui_client(self, client: socket.socket):
        """Handle messages from one GUI connection."""
        buffer = ''
        client.settimeout(0.5)

        # Start telemetry thread for this client
        telem_running = True

        def send_telemetry():
            while telem_running and rclpy.ok():
                try:
                    if self.drone_pose:
                        telem = {
                            'type': 'telemetry',
                            'x': round(self.drone_pose[0], 3),
                            'y': round(self.drone_pose[1], 3),
                            'z': round(self.drone_pose[2], 3),
                            'armed': self.mavros_state.armed,
                            'mode': self.mavros_state.mode,
                            'connected': self.mavros_state.connected,
                            'mission_active': self.mission_active,
                            'current_wp': self.mission_wp_index,
                            'total_wp': len(self.mission_waypoints),
                            'path_points': len(self.current_path),
                            'path_index': self.path_index,
                        }
                        msg = json.dumps(telem) + '\n'
                        client.sendall(msg.encode())
                except (BrokenPipeError, ConnectionResetError):
                    break
                except Exception:
                    pass
                time.sleep(0.2)  # 5Hz telemetry

        telem_thread = threading.Thread(target=send_telemetry, daemon=True)
        telem_thread.start()

        try:
            while rclpy.ok():
                try:
                    data = client.recv(4096).decode()
                    if not data:
                        break
                    buffer += data

                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip():
                            self._process_gui_command(json.loads(line))

                except socket.timeout:
                    continue
                except json.JSONDecodeError as e:
                    self.get_logger().warn(f'Invalid JSON from GUI: {e}')
        finally:
            telem_running = False
            client.close()
            self.get_logger().info('GUI disconnected')

    def _process_gui_command(self, cmd: dict):
        """Process a command from the GUI."""
        action = cmd.get('cmd', '')

        if action == 'upload_mission':
            waypoints = cmd.get('waypoints', [])
            self.mission_waypoints = waypoints
            self.mission_wp_index = 0
            self.mission_active = False
            self.get_logger().info(
                f'Mission uploaded: {len(waypoints)} waypoints'
            )

        elif action == 'start_mission':
            if not self.mission_waypoints:
                self.get_logger().warn('No mission uploaded!')
                return

            self.mission_active = True
            self.mission_wp_index = 0
            wp = self.mission_waypoints[0]
            self.get_logger().info(
                f'Mission started! First waypoint: ({wp["x"]:.1f}, {wp["y"]:.1f})'
            )
            self._send_goal_to_planner(wp['x'], wp['y'])

        elif action == 'pause_mission':
            self.mission_active = False
            self.get_logger().info('Mission paused — holding position')
            # Hold at current position
            if self.drone_pose:
                self.current_setpoint.pose.position.x = self.drone_pose[0]
                self.current_setpoint.pose.position.y = self.drone_pose[1]
                self.current_setpoint.pose.position.z = self.drone_pose[2]
            self.current_path = []

        elif action == 'abort_mission':
            self.mission_active = False
            self.current_path = []
            self.get_logger().warn('Mission ABORTED')

        elif action == 'return_home':
            self.mission_active = False
            self.get_logger().info('Returning home (0, 0)')
            self._send_goal_to_planner(0.0, 0.0)

        elif action == 'arm':
            self._arm_drone(True)

        elif action == 'disarm':
            self._arm_drone(False)

        elif action == 'guided':
            self._set_mode('GUIDED')

        else:
            self.get_logger().warn(f'Unknown command: {action}')

    def _send_telemetry_to_gui(self, data: dict):
        """Placeholder — telemetry is sent in the background thread."""
        pass

    # ================================================================
    # MAVROS Commands
    # ================================================================

    def _arm_drone(self, arm: bool):
        """Arm or disarm the drone via MAVROS service."""
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Arming service not available')
            return

        req = CommandBool.Request()
        req.value = arm
        future = self.arming_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'{"Armed" if arm else "Disarmed"}: {f.result().success}'
            )
        )

    def _set_mode(self, mode: str):
        """Set flight mode via MAVROS service."""
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Set mode service not available')
            return

        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'Mode set to {mode}: {f.result().mode_sent}'
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

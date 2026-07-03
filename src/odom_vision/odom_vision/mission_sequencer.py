#!/usr/bin/env python3
# Author: Gourav Khanduri
# Email:  gauravkhanduri93@gmail.com

import json
import math
import os
import socket
import threading
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class MissionSequencer(Node):

    def __init__(self):
        super().__init__("mission_sequencer")

        self.declare_parameter("mission_file", "")
        self.declare_parameter("tcp_port", 0)  # 0 = disabled (use file)
        self.declare_parameter("arrival_radius", 1.0)  # metres — hops shorter than this skip Nav2 (yaw+stabilize only)
        self.declare_parameter("altitude_tolerance", 0.3)  # metres
        self.declare_parameter("step_hz", 10.0)
        self.declare_parameter("stabilize_s", 2.0)
        self.declare_parameter("precision_radius", 0.3)
        self.declare_parameter("precision_timeout_s", 8.0)
        self.declare_parameter("missions_dir", os.path.expanduser("~/missions"))

        mission_file = self.get_parameter("mission_file").value
        tcp_port = self.get_parameter("tcp_port").value
        self.arrival_r = self.get_parameter("arrival_radius").value
        self.alt_tol = self.get_parameter("altitude_tolerance").value
        self.step_hz = self.get_parameter("step_hz").value
        self.stabilize_s = self.get_parameter("stabilize_s").value
        self.precision_radius = self.get_parameter("precision_radius").value
        self.precision_timeout_s = self.get_parameter("precision_timeout_s").value
        self._missions_dir = self.get_parameter("missions_dir").value
        os.makedirs(self._missions_dir, exist_ok=True)

        # ── Load mission ──────────────────────────────────────────────────────
        self._srv_sock = None  # kept alive between uploads when in TCP mode
        if tcp_port > 0:
            self._srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._srv_sock.bind(("0.0.0.0", tcp_port))
            self._srv_sock.listen(1)
            self._mission = self._accept_one_mission()
        elif mission_file:
            self.get_logger().info(f"Loading mission file: {mission_file}")
            with open(mission_file, "r") as f:
                self._mission = json.load(f)
        else:
            self.get_logger().fatal("Neither tcp_port nor mission_file parameter set. Aborting.")
            raise SystemExit(1)

        # ── Parse parameters ──────────────────────────────────────────────────
        params = self._mission.get("parameters", {})
        self.altitude = float(params.get("altitude_m", 10.0))
        self.speed = float(params.get("speed_mps", 1.0))
        self.loiter_s = float(params.get("loiter_time_s", 0.0))
        self.do_rth = bool(params.get("return_home", True))

        self._wps = self._mission.get("waypoints", [])
        if not self._wps:
            self.get_logger().fatal("No waypoints in mission. Aborting.")
            raise SystemExit(1)

        # Accept both key names for the RTH waypoint
        self._rth = (
            self._mission.get("exit_point") or self._mission.get("return_home")
            if self.do_rth
            else None
        )
        # Guard: if return_home key was a bool (from parameters), ignore it
        if not isinstance(self._rth, dict):
            self._rth = None

        self.get_logger().info(
            f"Mission loaded: {len(self._wps)} survey waypoints, "
            f"alt={self.altitude}m, speed={self.speed}m/s, "
            f"loiter={self.loiter_s}s, RTH={self.do_rth}"
        )

        # ── MAVROS setup ──────────────────────────────────────────────────────
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._setpoint_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        self._pose_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, mavros_qos
        )

        self._arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self._land_client = self.create_client(CommandTOL, "/mavros/cmd/land")

        self._nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self._current_pose = PoseStamped()
        self._pose_received = False

        threading.Thread(target=self._mission_loop, daemon=True).start()

    # ── TCP receive ───────────────────────────────────────────────────────────

    def _accept_one_mission(self) -> dict:
        """Block until GCS connects, receive one mission JSON, return parsed dict.

        Uses self._srv_sock which stays bound/listening so the port remains open
        for subsequent uploads without needing to restart the node.
        """
        self.get_logger().info("TCP: waiting for GCS mission upload...")
        conn, addr = self._srv_sock.accept()
        self.get_logger().info(f"GCS connected from {addr[0]}:{addr[1]}")

        with conn:
            raw = conn.makefile().readline()
            if not raw.strip():
                raise RuntimeError("Empty mission received from GCS")

            mission = json.loads(raw)
            wp_count = len(mission.get("waypoints", []))

            ack = (
                json.dumps(
                    {
                        "msg_type": "ack",
                        "mission_id": mission.get("mission_id", ""),
                        "status": "accepted",
                        "waypoint_count": wp_count,
                    }
                )
                + "\n"
            )
            conn.sendall(ack.encode())
            self.get_logger().info(f"Mission received: {wp_count} survey waypoints. ACK sent.")

        self._save_mission(mission)
        return mission

    def _save_mission(self, mission: dict) -> None:
        mission_id = mission.get("mission_id", f"mission_{int(time.time())}")
        path = os.path.join(self._missions_dir, f"{mission_id}.json")
        with open(path, "w") as f:
            json.dump(mission, f, indent=2)
        self.get_logger().info(f"Mission saved to {path}")

    # ── MAVROS callbacks ──────────────────────────────────────────────────────

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
            self.get_logger().error(f"{name}: no response")
        else:
            self.get_logger().info(f"{name}: result={result}")
        return result

    # ── Setpoint helpers ──────────────────────────────────────────────────────

    def _send_setpoint(self, x: float, y: float, z: float, yaw_deg: float = 0.0) -> None:
        yaw_rad = math.radians(yaw_deg)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
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
        return math.sqrt((cx - x) ** 2 + (cy - y) ** 2 + (cz - z) ** 2)

    def _current_yaw(self) -> float:
        q = self._current_pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    # ── Nav2 waypoint execution ───────────────────────────────────────────────

    def _go_to(self, x: float, y: float, z: float, yaw_deg: float, label: str) -> bool:
        """
        Two-phase waypoint execution:
          Phase A  — Travel to (x, y, z) via Nav2 NavigateToPose. Goal yaw is set
                     to direction-of-travel so MPPI doesn't fight an unrelated target.
          Phase A.5/B/C/D — precision approach, yaw in place, stabilize, loiter.
        Returns True if all phases succeeded.
        """
        self.get_logger().info(
            f"→ {label}  target=({x:.2f}, {y:.2f}, {z:.2f})m  yaw={yaw_deg:.1f}°"
        )

        # Skip Nav2 if already at the waypoint (handles WP1 == takeoff position).
        # Sending a no-op goal makes progress_checker abort with status=6.
        cx = self._current_pose.pose.position.x
        cy = self._current_pose.pose.position.y
        leg_dist = math.hypot(x - cx, y - cy)
        if leg_dist < self.arrival_r:
            self.get_logger().info(
                f"  Short hop to {label} (xy_dist={leg_dist:.2f}m < arrival_radius={self.arrival_r:.2f}m). "
                f"Skipping Nav2; yaw + stabilize only."
            )
            return self._finish_waypoint(x, y, z, yaw_deg, label)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server unavailable")
            return False

        dx, dy = x - cx, y - cy
        travel_yaw = math.atan2(dy, dx)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = z
        goal.pose.pose.orientation.z = math.sin(travel_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(travel_yaw / 2.0)

        send_future = self._nav_client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            time.sleep(0.05)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{label}: Nav2 rejected goal")
            return False

        result_future = handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.1)
        result_msg = result_future.result()
        status = result_msg.status if result_msg is not None else -1

        # action_msgs/GoalStatus.STATUS_SUCCEEDED == 4
        if status != 4:
            self.get_logger().error(f"{label}: Nav2 ended with status={status}")
            return False

        return self._finish_waypoint(x, y, z, yaw_deg, label)

    def _finish_waypoint(self, x: float, y: float, z: float, yaw_deg: float, label: str) -> bool:
        """Precision approach → yaw → stabilize → loiter."""
        self._precision_approach(x, y, z, label)
        self._yaw_to(yaw_deg, label)

        if self.stabilize_s > 0.0:
            self.get_logger().info(f"  Stabilizing at {label} for {self.stabilize_s:.1f}s...")
            self._hold_pose(yaw_deg, self.stabilize_s)

        if self.loiter_s > 0.0:
            self.get_logger().info(f"Reached {label}. Loitering {self.loiter_s:.1f}s...")
            self._hold_pose(yaw_deg, self.loiter_s)
        else:
            self.get_logger().info(f"Reached {label}.")
        return True

    def _precision_approach(self, x: float, y: float, z: float, label: str) -> bool:
        """Close the gap left by Nav2's xy_goal_tolerance to within precision_radius."""
        cur_dist = math.hypot(
            x - self._current_pose.pose.position.x,
            y - self._current_pose.pose.position.y,
        )
        if cur_dist < self.precision_radius:
            return True

        self.get_logger().info(
            f"  Precision approach to {label}: closing {cur_dist:.2f}m "
            f"to within {self.precision_radius:.2f}m..."
        )
        dt = 1.0 / self.step_hz
        deadline = time.time() + self.precision_timeout_s
        approach_yaw_deg = math.degrees(self._current_yaw())

        while rclpy.ok() and time.time() < deadline:
            self._send_setpoint(x, y, z, approach_yaw_deg)
            xy_err = math.hypot(
                x - self._current_pose.pose.position.x,
                y - self._current_pose.pose.position.y,
            )
            if xy_err < self.precision_radius:
                self.get_logger().info(
                    f"  Precision approach OK at {label} (xy_err={xy_err:.2f}m)"
                )
                return True
            time.sleep(dt)

        residual = math.hypot(
            x - self._current_pose.pose.position.x,
            y - self._current_pose.pose.position.y,
        )
        self.get_logger().warn(
            f"  Precision approach timeout at {label} (residual={residual:.2f}m). "
            f"Continuing to yaw anyway."
        )
        return False

    def _hold_pose(self, yaw_deg: float, duration_s: float) -> None:
        x = self._current_pose.pose.position.x
        y = self._current_pose.pose.position.y
        z = self._current_pose.pose.position.z
        dt = 1.0 / self.step_hz
        t_end = time.time() + duration_s
        while rclpy.ok() and time.time() < t_end:
            self._send_setpoint(x, y, z, yaw_deg)
            time.sleep(dt)

    def _yaw_to(self, target_yaw_deg: float, label: str) -> bool:
        target_yaw_r = math.radians(target_yaw_deg)
        x = self._current_pose.pose.position.x
        y = self._current_pose.pose.position.y
        z = self._current_pose.pose.position.z

        tol_rad = math.radians(3.0)
        timeout_s = 10.0
        dt = 1.0 / self.step_hz
        deadline = time.time() + timeout_s

        self.get_logger().info(f"  Yawing to {target_yaw_deg:.1f}° at {label}...")

        yaw_err = float("inf")
        while rclpy.ok() and time.time() < deadline:
            self._send_setpoint(x, y, z, target_yaw_deg)
            yaw_err = abs(
                math.atan2(
                    math.sin(target_yaw_r - self._current_yaw()),
                    math.cos(target_yaw_r - self._current_yaw()),
                )
            )
            if yaw_err < tol_rad:
                self.get_logger().info(f"  Yaw aligned at {label}")
                self._send_setpoint(x, y, z, target_yaw_deg)
                return True
            time.sleep(dt)

        self.get_logger().warn(f"  Yaw timeout at {label} (residual={math.degrees(yaw_err):.1f}°)")
        return False

    # ── Main sequence ─────────────────────────────────────────────────────────

    def _sequence(self) -> None:
        time.sleep(3.0)  # let MAVROS connect

        # 1. Wait for local position
        self.get_logger().info("Waiting for /mavros/local_position/pose...")
        while rclpy.ok() and not self._pose_received:
            time.sleep(0.2)
        p = self._current_pose.pose.position
        self.get_logger().info(f"Pose received: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")

        # 2. Arm
        self.get_logger().info("Arming...")
        req = CommandBool.Request()
        req.value = True
        self._call(self._arm_client, req, "arming")
        time.sleep(2.0)

        # 3. Takeoff
        self.get_logger().info(f"Taking off to {self.altitude:.1f}m AGL...")
        req = CommandTOL.Request()
        req.altitude = self.altitude
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        self._call(self._takeoff_client, req, "takeoff")

        # 4. Wait for altitude
        self.get_logger().info(f"Climbing... target={self.altitude:.1f}m +/-{self.alt_tol:.2f}m")
        while rclpy.ok():
            if self._pose_received:
                z = self._current_pose.pose.position.z
                if abs(z - self.altitude) <= self.alt_tol:
                    self.get_logger().info(f"Altitude confirmed: z={z:.2f}m")
                    break
            time.sleep(0.2)

        # 5. Survey waypoints
        self.get_logger().info(f"Starting survey: {len(self._wps)} waypoints...")
        mission_aborted = False
        for i, wp in enumerate(self._wps):
            label = f'WP {i+1}/{len(self._wps)} (id={wp["id"]})'
            ok = self._go_to(
                x=float(wp["x_m"]),
                y=float(wp["y_m"]),
                z=float(wp.get("z_m", self.altitude)),
                yaw_deg=float(wp.get("yaw_deg", 0.0)),
                label=label,
            )
            if not ok:
                self.get_logger().error(
                    f"Aborting mission at {label}. Skipping remaining waypoints."
                )
                mission_aborted = True
                break

        if not mission_aborted:
            self.get_logger().info("Survey complete.")

        # 6. RTH (attempted even after abort so drone returns home)
        if self.do_rth and self._rth is not None:
            self.get_logger().info("Returning to home...")
            if not self._go_to(
                x=float(self._rth["x_m"]),
                y=float(self._rth["y_m"]),
                z=float(self._rth.get("z_m", self.altitude)),
                yaw_deg=0.0,
                label="HOME",
            ):
                self.get_logger().error("RTH failed. Landing in place.")

        # 7. Land
        self.get_logger().info("Landing...")
        req = CommandTOL.Request()
        req.altitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        self._call(self._land_client, req, "land")

        self.get_logger().info("Mission complete.")

    def _mission_loop(self) -> None:
        """Run sequence then loop back to accept next upload (TCP mode only)."""
        while rclpy.ok():
            self._sequence()
            if self._srv_sock is None:
                break
            self.get_logger().info("Ready for next mission upload...")
            try:
                new_mission = self._accept_one_mission()
                self._load_mission(new_mission)
            except Exception as e:
                self.get_logger().error(f"Failed to load next mission: {e} — waiting for retry...")
                continue

    def _load_mission(self, mission: dict) -> None:
        """Update node state from a newly received mission dict."""
        self._mission = mission
        params = mission.get("parameters", {})
        self.altitude = float(params.get("altitude_m", 10.0))
        self.speed = float(params.get("speed_mps", 1.0))
        self.loiter_s = float(params.get("loiter_time_s", 0.0))
        self.do_rth = bool(params.get("return_home", True))
        self._wps = mission.get("waypoints", [])
        self._rth = (
            mission.get("exit_point") or mission.get("return_home") if self.do_rth else None
        )
        if not isinstance(self._rth, dict):
            self._rth = None
        self.get_logger().info(
            f"New mission loaded: {len(self._wps)} waypoints, alt={self.altitude}m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionSequencer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3


import math
import threading
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class SquareWaypoints(Node):

    def __init__(self):
        super().__init__("square_waypoints")

        self.declare_parameter("altitude", 5.0)
        self.declare_parameter("side", 4.0)
        self.declare_parameter("arrival_radius", 0.5)
        self.declare_parameter("altitude_tolerance", 0.3)
        self.declare_parameter("waypoint_dwell", 3.0)
        self.declare_parameter("max_speed", 0.5)

        self.altitude = self.get_parameter("altitude").value
        self.side = self.get_parameter("side").value
        self.arrival_radius = self.get_parameter("arrival_radius").value
        self.altitude_tolerance = self.get_parameter("altitude_tolerance").value
        self.waypoint_dwell = self.get_parameter("waypoint_dwell").value
        self.max_speed = self.get_parameter("max_speed").value

        # Current local position
        self.current_pose = PoseStamped()
        self.pose_received = False

        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.land_client = self.create_client(CommandTOL, "/mavros/cmd/land")

        self.setpoint_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )

        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, mavros_qos
        )

        self.get_logger().info(
            f"SquareWaypoints (ArduPilot GUIDED + VIO): altitude={self.altitude} m, side={self.side} m, "
            f"alt_tol={self.altitude_tolerance} m, dwell={self.waypoint_dwell} s, "
            f"max_speed={self.max_speed} m/s"
        )
        threading.Thread(target=self._sequence, daemon=True).start()

    # Callbacks

    def _pose_cb(self, msg):
        self.current_pose = msg
        self.pose_received = True

    def _call(self, client, request, name):
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

    def _distance_to(self, x, y, z):
        cx = self.current_pose.pose.position.x
        cy = self.current_pose.pose.position.y
        cz = self.current_pose.pose.position.z
        return math.sqrt((cx - x) ** 2 + (cy - y) ** 2 + (cz - z) ** 2)

    def _send_setpoint(self, x, y, z, yaw=0.0):
        """Publish a local position setpoint. yaw is in radians (ENU: 0=+X, CCW positive)."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        # Convert yaw to quaternion (rotation around Z axis only)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.setpoint_pub.publish(msg)

    def _go_to(self, x, y, z, label):
        """Move a virtual setpoint toward target at max_speed, block until within arrival_radius, then dwell."""
        self.get_logger().info(
            f"Going to {label} ({x:.1f}, {y:.1f}, {z:.1f}) m at {self.max_speed:.1f} m/s ..."
        )
        dt = 0.1  # 10 Hz update rate

        # Seed virtual setpoint at current drone position so there's no jump
        vx = self.current_pose.pose.position.x
        vy = self.current_pose.pose.position.y
        vz = self.current_pose.pose.position.z

        # Yaw toward the target (face direction of travel); refined each step
        yaw = math.atan2(y - vy, x - vx)

        while rclpy.ok():
            # Advance virtual setpoint toward target by at most max_speed * dt each step
            dx, dy, dz = x - vx, y - vy, z - vz
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            step = self.max_speed * dt

            # Update yaw only when there is meaningful horizontal movement
            if abs(dx) > 1e-3 or abs(dy) > 1e-3:
                yaw = math.atan2(dy, dx)

            if dist <= step:
                vx, vy, vz = x, y, z
            else:
                scale = step / dist
                vx += dx * scale
                vy += dy * scale
                vz += dz * scale

            self._send_setpoint(vx, vy, vz, yaw)

            if self.pose_received and self._distance_to(x, y, z) < self.arrival_radius:
                self.get_logger().info(f"Reached {label}. Dwelling {self.waypoint_dwell:.1f} s...")
                dwell_end = time.time() + self.waypoint_dwell
                while rclpy.ok() and time.time() < dwell_end:
                    self._send_setpoint(x, y, z, yaw)
                    time.sleep(dt)
                self.get_logger().info(f"Dwell complete at {label}.")
                break
            time.sleep(dt)

    def _sequence(self):
        time.sleep(3.0)  # let MAVROS connect

        # 1. Wait for VIO local position
        self.get_logger().info("Waiting for VIO local position...")
        while rclpy.ok() and not self.pose_received:
            time.sleep(0.2)
        self.get_logger().info("VIO position received.")

        # 2. Arm
        self.get_logger().info("Arming...")
        req = CommandBool.Request()
        req.value = True
        self._call(self.arming_client, req, "arming")
        time.sleep(2.0)

        # 4. Takeoff — ArduPilot interprets CommandTOL.altitude as relative (AGL), no AMSL needed
        self.get_logger().info(f"Taking off to {self.altitude} m AGL...")
        req = CommandTOL.Request()
        req.altitude = self.altitude
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        self._call(self.takeoff_client, req, "takeoff")

        # 5. Wait for altitude confirmation from VIO
        self.get_logger().info(
            f"Climbing... target {self.altitude:.1f} m ± {self.altitude_tolerance:.2f} m"
        )
        while rclpy.ok():
            if self.pose_received:
                z = self.current_pose.pose.position.z
                if abs(z - self.altitude) <= self.altitude_tolerance:
                    self.get_logger().info(
                        f"Altitude confirmed: z={z:.2f} m. Starting square."
                    )
                    break
            time.sleep(0.2)

        # 6. Square
        s = self.side
        alt = self.altitude

        self._go_to(s, 0.0, alt, "corner 1")
        self._go_to(s, -s, alt, "corner 2")
        self._go_to(0.0, -s, alt, "corner 3")
        self._go_to(0.0, 0.0, alt, "home")

        # 7. Land
        self.get_logger().info("Square complete. Landing...")
        req = CommandTOL.Request()
        req.altitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        self._call(self.land_client, req, "land")

        self.get_logger().info("Done.")


def main(args=None):
    rclpy.init(args=args)
    node = SquareWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

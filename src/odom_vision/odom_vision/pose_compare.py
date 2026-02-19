#!/usr/bin/env python3
"""
Pose Comparison Node for Visual Odometry Debugging.

Compares vision_pose with local_position to check alignment.

Run: python3 pose_compare.py
"""

import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


class PoseCompare(Node):

    def __init__(self):
        super().__init__("pose_compare")

        self.vision_pose = None
        self.local_pose = None

        # Subscribe to vision pose (what we send to MAVROS)
        self.create_subscription(PoseStamped, "/mavros/vision_pose/pose", self.vision_cb, 10)

        # Subscribe to local position (fused result from EKF)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.local_cb, 10)

        # Timer to print comparison every second
        self.create_timer(1.0, self.compare)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Pose Comparison Node Started")
        self.get_logger().info(
            "Comparing: /mavros/vision_pose/pose vs /mavros/local_position/pose"
        )
        self.get_logger().info("=" * 60)

    def vision_cb(self, msg: PoseStamped):
        """Store latest vision pose."""
        self.vision_pose = msg.pose

    def local_cb(self, msg: PoseStamped):
        """Store latest local pose."""
        self.local_pose = msg.pose

    def quaternion_to_euler(self, q):
        """Convert quaternion to euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def compare(self):
        """Log position and orientation difference between vision and local poses."""
        self.get_logger().info("-" * 60)

        if self.vision_pose is None:
            self.get_logger().warn("No vision_pose received! Check bridge/publisher.")
            return

        if self.local_pose is None:
            self.get_logger().warn("No local_position received! Check MAVROS connection.")
            return

        # Position comparison
        vp = self.vision_pose.position
        lp = self.local_pose.position

        dx = vp.x - lp.x
        dy = vp.y - lp.y
        dz = vp.z - lp.z
        pos_diff = math.sqrt(dx * dx + dy * dy + dz * dz)

        self.get_logger().info(f"VISION POSE:  X={vp.x:+7.3f}  Y={vp.y:+7.3f}  Z={vp.z:+7.3f}")
        self.get_logger().info(f"LOCAL POSE:   X={lp.x:+7.3f}  Y={lp.y:+7.3f}  Z={lp.z:+7.3f}")
        diff_str = f"dX={dx:+7.3f} dY={dy:+7.3f} dZ={dz:+7.3f}  |d|={pos_diff:.4f}m"
        self.get_logger().info(f"DIFFERENCE:  {diff_str}")

        # Orientation comparison
        v_roll, v_pitch, v_yaw = self.quaternion_to_euler(self.vision_pose.orientation)
        l_roll, l_pitch, l_yaw = self.quaternion_to_euler(self.local_pose.orientation)

        self.get_logger().info(f"VISION YAW:   {math.degrees(v_yaw):+7.2f}°")
        self.get_logger().info(f"LOCAL YAW:    {math.degrees(l_yaw):+7.2f}°")
        self.get_logger().info(f"YAW DIFF:     {math.degrees(v_yaw - l_yaw):+7.2f}°")

        # Status check
        if pos_diff < 0.1:
            self.get_logger().info("✓ Position alignment: GOOD")
        elif pos_diff < 0.5:
            self.get_logger().warn("⚠ Position alignment: ACCEPTABLE")
        else:
            self.get_logger().error("✗ Position alignment: POOR - Check frame transforms!")


def main(args=None):
    rclpy.init(args=args)
    node = PoseCompare()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

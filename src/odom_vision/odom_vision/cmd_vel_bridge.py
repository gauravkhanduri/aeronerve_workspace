#!/usr/bin/env python3
"""
cmd_vel_bridge — converts Nav2 /cmd_vel (Twist) to MAVROS PositionTarget.

Nav2 outputs geometry_msgs/Twist assuming a ground robot.
ArduPilot GUIDED mode takes mavros_msgs/PositionTarget on /mavros/setpoint_raw/local.

Mapping:
  twist.linear.x  -> forward velocity in drone's current heading (ENU)
  twist.angular.z -> yaw rate
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import PositionTarget
from rclpy.qos import QoSProfile, ReliabilityPolicy


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")
        self.yaw = 0.0

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._pose_cb, sensor_qos)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        self.pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10
        )
        self.get_logger().info("cmd_vel_bridge ready")

    def _pose_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _cmd_vel_cb(self, msg: Twist):
        pt = PositionTarget()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "map"
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        pt.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
        )
        v = msg.linear.x
        pt.velocity.x = v * math.cos(self.yaw)
        pt.velocity.y = v * math.sin(self.yaw)
        pt.velocity.z = 0.0
        pt.yaw_rate = msg.angular.z
        self.pub.publish(pt)


def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

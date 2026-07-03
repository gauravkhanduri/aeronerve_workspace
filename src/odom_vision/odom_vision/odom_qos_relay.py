#!/usr/bin/env python3
# Author: Gourav Khanduri
# Email:  gauravkhanduri93@gmail.com
"""
Relay /mavros/local_position/odom (BEST_EFFORT) -> /odom (RELIABLE).

Nav2's bt_navigator subscribes to its odom_topic with RELIABLE QoS.
MAVROS publishes with BEST_EFFORT (sensor QoS). The QoS mismatch means
bt_navigator receives zero messages. This bridges the two.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class OdomQosRelay(Node):
    def __init__(self):
        super().__init__("odom_qos_relay")
        be_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        rel_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(Odometry, "/odom", rel_qos)
        self.create_subscription(
            Odometry, "/mavros/local_position/odom", self._cb, be_qos
        )
        self.get_logger().info(
            "Relaying /mavros/local_position/odom (BE) -> /odom (RELIABLE)"
        )

    def _cb(self, msg: Odometry):
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = OdomQosRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
TF Publisher for Visual Odometry frames.

Publishes TF frames for RViz visualization and frame alignment checking.

Run: python3 tf_publisher.py
"""

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class TFPublisher(Node):

    def __init__(self):
        super().__init__('vo_tf_publisher')

        # TF Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # QoS profile compatible with MAVROS (BEST_EFFORT)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS profile for ros_gz_bridge (RELIABLE)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to poses with compatible QoS
        self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.vision_pose_cb,
            sensor_qos
        )

        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pose_cb,
            sensor_qos
        )

        # /odom from ros_gz_bridge uses RELIABLE QoS
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            reliable_qos
        )

        # Publish static transform for camera
        self.publish_static_transforms()

        self.get_logger().info('TF Publisher started')
        self.get_logger().info(
            'Publishing frames: map, odom, base_link, vision_pose, local_pose, camera_link'
        )

    def publish_static_transforms(self):
        """Publish static transforms."""
        transforms = []

        # map -> odom (identity for now)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)

        # base_link -> camera_link (camera mounted on drone)
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'camera_link'
        t2.transform.translation.x = 0.08  # forward
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = -0.05  # down
        # 45 degree pitch down (quaternion)
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.3827  # sin(45/2)
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 0.9239  # cos(45/2)
        transforms.append(t2)

        self.static_tf_broadcaster.sendTransform(transforms)

    def vision_pose_cb(self, msg: PoseStamped):
        """Publish TF for vision pose."""
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'vision_pose'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def local_pose_cb(self, msg: PoseStamped):
        """Publish TF for local (fused) pose."""
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'local_pose'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def odom_cb(self, msg: Odometry):
        """Publish TF for odometry -> base_link."""
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

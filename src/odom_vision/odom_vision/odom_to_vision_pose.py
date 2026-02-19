#!/usr/bin/env python3
"""
Converts Gazebo odometry to MAVROS vision_pose format.
ros2 topic echo /odom --field pose.pose.position --onceSubscribes to: /odom (nav_msgs/msg/Odometry) for X/Y/Z
Publishes to: /mavros/vision_pose/pose (geometry_msgs/msg/PoseStamped)
"""

import rclpy
from rclpy.node import Node 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomToVisionPose(Node):
    def __init__(self):
        super().__init__('odom_to_vision_pose')
        
        # Subscriber for odometry from Gazebo (X/Y/Z)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for MAVROS vision pose
        self.vision_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )
        
        self.get_logger().info('Odom to Vision Pose converter started')
        self.get_logger().info('Subscribing to: /odom (X/Y/Z)')
        self.get_logger().info('Publishing to: /mavros/vision_pose/pose')

    def odom_callback(self, msg: Odometry):
        pose_stamped = PoseStamped()
        
        # Copy header
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = 'map'  # MAVROS expects 'map' frame
        
        # Copy full pose from odometry (X/Y/Z)
        pose_stamped.pose = msg.pose.pose
        
        # Publish
        self.vision_pub.publish(pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToVisionPose()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

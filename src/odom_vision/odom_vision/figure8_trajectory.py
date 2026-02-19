#!/usr/bin/env python3
"""
Figure-8 trajectory publisher for MAVROS.
Publishes continuous setpoints to fly a figure-8 pattern.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


class Figure8Trajectory(Node):
    def __init__(self):
        super().__init__('figure8_trajectory')
        
        # Parameters
        self.declare_parameter('altitude', 10.0)  # Height in meters
        self.declare_parameter('scale_x', 10.0)   # Width of figure-8
        self.declare_parameter('scale_y', 5.0)    # Height of each loop
        self.declare_parameter('speed', 0.5)      # Speed factor (lower = slower)
        self.declare_parameter('center_x', 0.0)   # Center X position
        self.declare_parameter('center_y', 0.0)   # Center Y position
        
        self.altitude = self.get_parameter('altitude').value
        self.scale_x = self.get_parameter('scale_x').value
        self.scale_y = self.get_parameter('scale_y').value
        self.speed = self.get_parameter('speed').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        
        # Publisher
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        # Timer for 20Hz publishing
        self.timer = self.create_timer(0.05, self.publish_setpoint)  # 20Hz
        
        self.t = 0.0  # Time parameter for trajectory
        
        self.get_logger().info('Figure-8 Trajectory started')
        self.get_logger().info(f'Altitude: {self.altitude}m, Scale: ({self.scale_x}, {self.scale_y})')
        self.get_logger().info(f'Center: ({self.center_x}, {self.center_y}), Speed: {self.speed}')

    def publish_setpoint(self):
        """Publish figure-8 trajectory setpoint"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        
        # Figure-8 parametric equations (Lemniscate of Bernoulli)
        # x = a * cos(t) / (1 + sin^2(t))
        # y = a * sin(t) * cos(t) / (1 + sin^2(t))
        # Simplified version using sin(t) and sin(2t)
        
        x = self.scale_x * math.sin(self.t)
        y = self.scale_y * math.sin(2 * self.t)
        
        pose.pose.position.x = self.center_x + x
        pose.pose.position.y = self.center_y + y
        pose.pose.position.z = self.altitude
        
        # Calculate heading (yaw) to point in direction of travel
        dx = self.scale_x * math.cos(self.t)
        dy = 2 * self.scale_y * math.cos(2 * self.t)
        yaw = math.atan2(dy, dx)
        
        # Convert yaw to quaternion (only rotating around Z)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        
        self.setpoint_pub.publish(pose)
        
        # Increment time
        self.t += self.speed * 0.05  # Adjust based on timer period
        if self.t > 2 * math.pi:
            self.t -= 2 * math.pi


def main(args=None):
    rclpy.init(args=args)
    node = Figure8Trajectory()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

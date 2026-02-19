#!/usr/bin/env python3
"""
Artificial Potential Field (APF) Node for MAVROS.
Subscribes to /depth_distance and /mavros/local_position/pose, publishes setpoints to /mavros/setpoint_position/local.
Combines goal attraction and obstacle repulsion for collision avoidance.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import math

class APFPlanner(Node):
    def __init__(self):
        super().__init__('apf_planner')
        # Parameters
        self.declare_parameter('goal_x', 30.0)
        self.declare_parameter('goal_y', 5.0)
        self.declare_parameter('goal_z', 5.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('obstacle_threshold', 2.0)  # meters
        self.declare_parameter('repulsion_gain', 2.0)
        self.declare_parameter('attraction_gain', 1.0)
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_z = self.get_parameter('goal_z').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.repulsion_gain = self.get_parameter('repulsion_gain').value
        self.attraction_gain = self.get_parameter('attraction_gain').value
        self.current_pose = None
        self.obstacle_distance = None
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )
        self.distance_sub = self.create_subscription(
            Float32,
            '/depth_distance',
            self.distance_callback,
            10
        )
        # Publisher
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_setpoint)
        self.get_logger().info('APF Planner started')
        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y}, {self.goal_z})')
    def pose_callback(self, msg):
        self.current_pose = msg.pose
    def distance_callback(self, msg):
        self.obstacle_distance = msg.data
    def publish_setpoint(self):
        if self.current_pose is None:
            return
        # Attraction force towards goal
        dx = self.goal_x - self.current_pose.position.x
        dy = self.goal_y - self.current_pose.position.y
        dz = self.goal_z - self.current_pose.position.z
        dist_to_goal = math.sqrt(dx**2 + dy**2 + dz**2)
        fx = self.attraction_gain * dx
        fy = self.attraction_gain * dy
        fz = self.attraction_gain * dz
        # Repulsion force from obstacle (assume obstacle in front, along drone's heading)
        if self.obstacle_distance is not None and self.obstacle_distance < self.obstacle_threshold:
            repulsion = self.repulsion_gain * (1.0 / self.obstacle_distance - 1.0 / self.obstacle_threshold)
            repulsion = max(repulsion, 0.0)
            # Assume repulsion along drone's forward direction (X axis in local frame)
            fx -= repulsion
            self.get_logger().info(f'Obstacle detected at {self.obstacle_distance:.2f}m, repulsion: {repulsion:.2f}')
        # Normalize total force
        norm = math.sqrt(fx**2 + fy**2 + fz**2)
        if norm > 0.0: 
            fx /= norm
            fy /= norm
            fz /= norm
        # Step towards next setpoint
        step_size = 0.5  # meters per step
        next_x = self.current_pose.position.x + fx * step_size
        next_y = self.current_pose.position.y + fy * step_size
        next_z = self.current_pose.position.z + fz * step_size
        # Publish setpoint
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = next_x
        pose.pose.position.y = next_y
        pose.pose.position.z = next_z
        pose.pose.orientation.w = 1.0
        self.setpoint_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = APFPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

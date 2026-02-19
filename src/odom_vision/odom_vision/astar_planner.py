#!/usr/bin/env python3
"""
A* Path Planning Node for MAVROS.

Plans and executes paths from current position to goal.

Usage:
  ros2 run odom_vision astar_planner
  ros2 run odom_vision astar_planner --ros-args -p goal_x:=30.0 -p goal_y:=5.0 -p goal_z:=5.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
import heapq
import math


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        
        # Parameters
        self.declare_parameter('goal_x', 30.0)
        self.declare_parameter('goal_y', 5.0)
        self.declare_parameter('goal_z', 5.0)
        self.declare_parameter('grid_resolution', 1.0)  # meters per grid cell
        self.declare_parameter('waypoint_threshold', 0.5)  # meters to consider waypoint reached
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_z = self.get_parameter('goal_z').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Current position
        self.current_pose = None
        self.path = []
        self.current_waypoint_idx = 0
        self.path_planned = False
        
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            pose_qos
        )
            
        
        
        # Publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            '/astar_planner/path',
            10
        )
        
        # Timer for setpoint publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_setpoint)
        
        self.get_logger().info('A* Path Planner started')
        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y}, {self.goal_z})')
        self.get_logger().info('Waiting for current position...')

    def pose_callback(self, msg: PoseStamped):
        """Update current position."""
        self.current_pose = msg.pose
        
        # Plan path once we have current position
        if not self.path_planned and self.current_pose is not None:
            self.plan_path()

    def plan_path(self):
        """Run A* to find path from current position to goal."""
        start = (
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z
        )
        goal = (self.goal_x, self.goal_y, self.goal_z)
        
        self.get_logger().info(f'Planning path from {start} to {goal}')
        
        # Run A*
        path = self.astar(start, goal)
        
        if path:
            self.path = path
            self.current_waypoint_idx = 0
            self.path_planned = True
            self.get_logger().info(f'Path found with {len(path)} waypoints')
            self.publish_path_visualization()
        else:
            self.get_logger().error('No path found!')

    def astar(self, start, goal):
        """
        Run A* pathfinding in 3D space.

        Returns list of (x, y, z) waypoints.
        """
        # Convert to grid coordinates
        def to_grid(pos):
            return (
                round(pos[0] / self.grid_resolution),
                round(pos[1] / self.grid_resolution),
                round(pos[2] / self.grid_resolution)
            )
        
        def to_world(grid_pos):
            return (
                grid_pos[0] * self.grid_resolution,
                grid_pos[1] * self.grid_resolution,
                grid_pos[2] * self.grid_resolution
            )
        
        def heuristic(a, b):
            # Euclidean distance
            return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)
        
        def get_neighbors(pos):
            """Get 26-connected neighbors (all directions in 3D)."""
            neighbors = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    for dz in [-1, 0, 1]:
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        neighbor = (pos[0] + dx, pos[1] + dy, pos[2] + dz)
                        # Add obstacle checking here if needed
                        # if not self.is_obstacle(neighbor):
                        neighbors.append(neighbor)
            return neighbors
        
        start_grid = to_grid(start)
        goal_grid = to_grid(goal)
        
        # Priority queue: (f_score, counter, position)
        counter = 0
        open_set = [(0, counter, start_grid)]
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: heuristic(start_grid, goal_grid)}
        open_set_hash = {start_grid}
        
        while open_set:
            current = heapq.heappop(open_set)[2]
            open_set_hash.discard(current)
            
            # Check if reached goal
            if heuristic(current, goal_grid) < 1.5:
                # Reconstruct path
                path = [to_world(goal_grid)]
                while current in came_from:
                    path.append(to_world(current))
                    current = came_from[current]
                path.append(start)
                path.reverse()
                
                # Simplify path (remove intermediate collinear points)
                return self.simplify_path(path)
            
            for neighbor in get_neighbors(current):
                # Calculate tentative g_score
                move_cost = heuristic(current, neighbor)
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal_grid)
                    
                    if neighbor not in open_set_hash:
                        counter += 1
                        heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
                        open_set_hash.add(neighbor)
        
        return None  # No path found

    def simplify_path(self, path):
        """Remove unnecessary intermediate waypoints."""
        if len(path) <= 2:
            return path
        
        simplified = [path[0]]
        
        for i in range(1, len(path) - 1):
            # Check if point is necessary (not collinear with prev and next)
            prev = simplified[-1]
            curr = path[i]
            next_pt = path[i + 1]
            
            # Simple distance-based simplification
            dist_direct = math.sqrt(
                (next_pt[0]-prev[0])**2 + 
                (next_pt[1]-prev[1])**2 + 
                (next_pt[2]-prev[2])**2
            )
            dist_via = (
                math.sqrt((curr[0]-prev[0])**2 + (curr[1]-prev[1])**2 + (curr[2]-prev[2])**2) +
                math.sqrt((next_pt[0]-curr[0])**2 + (next_pt[1]-curr[1])**2 + (next_pt[2]-curr[2])**2)
            )
            
            # Keep point if path is significantly longer via intermediate
            if dist_via > dist_direct * 1.1:
                simplified.append(curr)
        
        simplified.append(path[-1])
        return simplified

    def publish_path_visualization(self):
        """Publish path for RViz visualization."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for waypoint in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

    def publish_setpoint(self):
        """Publish current waypoint setpoint."""
        if not self.path or self.current_pose is None:
            return
        
        if self.current_waypoint_idx >= len(self.path):
            self.get_logger().info('Goal reached!')
            # Keep publishing final position
            waypoint = self.path[-1]
        else:
            waypoint = self.path[self.current_waypoint_idx]
            
            # Check if we reached current waypoint
            dist = math.sqrt(
                (self.current_pose.position.x - waypoint[0])**2 +
                (self.current_pose.position.y - waypoint[1])**2 +
                (self.current_pose.position.z - waypoint[2])**2
            )
            
            if dist < self.waypoint_threshold:
                self.current_waypoint_idx += 1
                self.get_logger().info(f'Waypoint {self.current_waypoint_idx}/{len(self.path)} reached')
                if self.current_waypoint_idx < len(self.path):
                    waypoint = self.path[self.current_waypoint_idx]
        
        # Create and publish setpoint
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.position.z = waypoint[2]
        
        # Calculate yaw towards waypoint
        if self.current_waypoint_idx < len(self.path):
            dx = waypoint[0] - self.current_pose.position.x
            dy = waypoint[1] - self.current_pose.position.y
            yaw = math.atan2(dy, dx)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
        else:
            pose.pose.orientation.w = 1.0
        
        self.setpoint_pub.publish(pose)
        
        # Republish path for visualization
        self.publish_path_visualization()


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

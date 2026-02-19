#!/usr/bin/env python3
"""
Hybrid A* + APF Path Planner for MAVROS.

Combines A* global path planning with APF local obstacle avoidance.

Usage:
  ros2 run odom_vision hybrid_planner
  ros2 run odom_vision hybrid_planner --ros-args -p goal_x:=30.0 -p goal_y:=5.0 -p goal_z:=5.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import heapq
import math


class HybridPlanner(Node):
    def __init__(self):
        super().__init__('hybrid_planner')

        # Parameters
        self.declare_parameter('goal_x', 30.0)
        self.declare_parameter('goal_y', 5.0)
        self.declare_parameter('goal_z', 3.0)
        self.declare_parameter('grid_resolution', 0.5)  # meters per grid cell
        self.declare_parameter('waypoint_threshold', 1.0)  # meters to consider waypoint reached
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('obstacle_threshold', 2.5)  # meters for APF repulsion
        self.declare_parameter('repulsion_gain', 3.0)  # APF repulsion strength
        self.declare_parameter('attraction_gain', 1.0)  # APF attraction strength
        self.declare_parameter('apf_step_size', 1.0)  # meters per APF step
        self.declare_parameter('replan_distance_threshold', 2.0)  # replan if obstacle is closer

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_z = self.get_parameter('goal_z').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        publish_rate = self.get_parameter('publish_rate').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.repulsion_gain = self.get_parameter('repulsion_gain').value
        self.attraction_gain = self.get_parameter('attraction_gain').value
        self.apf_step_size = self.get_parameter('apf_step_size').value
        self.replan_distance_threshold = self.get_parameter('replan_distance_threshold').value

        # State variables
        self.current_pose = None
        self.obstacle_distance = None
        self.path = []
        self.current_waypoint_idx = 0
        self.path_planned = False

        # QoS for MAVROS pose
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

        self.distance_sub = self.create_subscription(
            Float32,
            '/depth_distance',
            self.distance_callback,
            10
        )

        # Publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/hybrid_planner/path',
            10
        )

        # Timer for setpoint publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_setpoint)

        self.get_logger().info('Hybrid A* + APF Planner started')
        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y}, {self.goal_z})')
        self.get_logger().info('Waiting for current position...')

    def pose_callback(self, msg: PoseStamped):
        """Update current position."""
        self.current_pose = msg.pose

        # Plan path once we have current position
        if not self.path_planned and self.current_pose is not None:
            self.plan_path()

    def distance_callback(self, msg: Float32):
        """Update obstacle distance from depth sensor."""
        self.obstacle_distance = msg.data

        # Replan if obstacle is very close and blocking the path
        if (self.obstacle_distance is not None
                and self.obstacle_distance < self.replan_distance_threshold
                and self.path_planned):
            self.get_logger().warn(
                f'Obstacle at {self.obstacle_distance:.2f}m - replanning...')
            self.plan_path()

    def plan_path(self):
        """Run A* to find path from current position to goal."""
        if self.current_pose is None:
            return

        start = (
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z
        )
        goal = (self.goal_x, self.goal_y, self.goal_z)

        self.get_logger().info(
            f'Planning path from ({start[0]:.2f}, {start[1]:.2f}, {start[2]:.2f}) to {goal}')

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
                        # TODO: Add obstacle map checking here if needed
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
            d1 = math.sqrt(
                (curr[0]-prev[0])**2 + (curr[1]-prev[1])**2 + (curr[2]-prev[2])**2)
            d2 = math.sqrt(
                (next_pt[0]-curr[0])**2 + (next_pt[1]-curr[1])**2 + (next_pt[2]-curr[2])**2)
            dist_via = d1 + d2

            # Keep point if path is significantly longer via intermediate
            if dist_via > dist_direct * 1.05:
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

    def compute_apf_force(self, target_pos):
        """
        Compute Artificial Potential Field force vector.

        Combines attraction to target and repulsion from obstacles.
        """
        # Attraction force towards target waypoint
        dx = target_pos[0] - self.current_pose.position.x
        dy = target_pos[1] - self.current_pose.position.y
        dz = target_pos[2] - self.current_pose.position.z

        fx = self.attraction_gain * dx
        fy = self.attraction_gain * dy
        fz = self.attraction_gain * dz

        # Repulsion force from obstacle (assume obstacle in front, along drone's heading)
        if self.obstacle_distance is not None and self.obstacle_distance < self.obstacle_threshold:
            repulsion = self.repulsion_gain * (
                1.0 / self.obstacle_distance - 1.0 / self.obstacle_threshold)
            repulsion = max(repulsion, 0.0)

            # Get drone's current yaw from orientation
            qz = self.current_pose.orientation.z
            qw = self.current_pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)

            # Apply repulsion opposite to obstacle direction (backward)
            fx -= repulsion * math.cos(yaw)
            fy -= repulsion * math.sin(yaw)

            self.get_logger().info(
                f'Obstacle at {self.obstacle_distance:.2f}m, repulsion: {repulsion:.2f}',
                throttle_duration_sec=1.0
            )

        return fx, fy, fz

    def publish_setpoint(self):
        """Publish current waypoint setpoint with APF obstacle avoidance."""
        if not self.path or self.current_pose is None:
            return

        if self.current_waypoint_idx >= len(self.path):
            self.get_logger().info('Goal reached!', throttle_duration_sec=2.0)
            # Keep publishing final position
            waypoint = self.path[-1]

            # Create and publish final setpoint
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.w = 1.0
            self.setpoint_pub.publish(pose)
            return

        # Get current target waypoint
        waypoint = self.path[self.current_waypoint_idx]

        # Check if we reached current waypoint
        dist = math.sqrt(
            (self.current_pose.position.x - waypoint[0])**2 +
            (self.current_pose.position.y - waypoint[1])**2 +
            (self.current_pose.position.z - waypoint[2])**2
        )

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        cz = self.current_pose.position.z
        self.get_logger().info(
            f'Current: ({cx:.2f}, {cy:.2f}, {cz:.2f}), '
            f'Target waypoint {self.current_waypoint_idx+1}/{len(self.path)}: '
            f'({waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}), '
            f'distance: {dist:.2f}m',
            throttle_duration_sec=1.0
        )

        if dist < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_idx}/{len(self.path)} reached')
            if self.current_waypoint_idx < len(self.path):
                waypoint = self.path[self.current_waypoint_idx]
            else:
                return

        # Compute APF force towards waypoint (with obstacle avoidance)
        fx, fy, fz = self.compute_apf_force(waypoint)

        # Normalize force vector but preserve some proportionality
        norm = math.sqrt(fx**2 + fy**2 + fz**2)
        if norm > 0.0:
            # Limit the maximum step but allow smaller steps when close
            step = min(self.apf_step_size, norm)
            fx = (fx / norm) * step
            fy = (fy / norm) * step
            fz = (fz / norm) * step

        # Compute next setpoint using APF step
        next_x = self.current_pose.position.x + fx
        next_y = self.current_pose.position.y + fy
        next_z = self.current_pose.position.z + fz

        self.get_logger().info(
            f'APF step: ({fx:.2f}, {fy:.2f}, {fz:.2f}) -> '
            f'Next setpoint: ({next_x:.2f}, {next_y:.2f}, {next_z:.2f})',
            throttle_duration_sec=1.0
        )

        # Create and publish setpoint
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = next_x
        pose.pose.position.y = next_y
        pose.pose.position.z = next_z

        # Calculate yaw towards next position
        dx = next_x - self.current_pose.position.x
        dy = next_y - self.current_pose.position.y
        if abs(dx) > 0.01 or abs(dy) > 0.01:  # Avoid division by zero
            yaw = math.atan2(dy, dx)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
        else:
            pose.pose.orientation.w = 1.0

        self.setpoint_pub.publish(pose)

        # Republish path for visualization
        if self.current_waypoint_idx % 10 == 0:  # Throttle path publishing
            self.publish_path_visualization()


def main(args=None):
    rclpy.init(args=args)
    node = HybridPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

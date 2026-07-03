#!/usr/bin/env python3
"""
Reactive Navigation Node for ISR Drone
ROS2 Humble + ArduPilot GUIDED mode

Sits BETWEEN your mission executor and the drone:

    Executor publishes /goal_pose
              ↓
    This node checks occupancy grid
              ↓
    CLEAR   → publishes /planned_path (executor picks it up and flies)
    BLOCKED → publishes /obstacle_status "BLOCKED" (executor gets no path, drone holds)

Also:
    - Continuously monitors the forward direction at 5Hz
    - If obstacle appears in front of moving drone, publishes empty /planned_path
      to make the executor stop
    - Accepts /move_command (body-frame Point) as alternative input
    - Publishes RViz markers for visualization

Your executor needs ZERO changes. Just run this node alongside it.

Subscribes:
    /projected_map              → nav_msgs/OccupancyGrid (from OctoMap)
    /zed/zed_node/pose          → geometry_msgs/PoseStamped (drone VIO)
    /goal_pose                  → geometry_msgs/PoseStamped (FROM your executor)
    /move_command               → geometry_msgs/Point (alternative body-frame input)

Publishes:
    /planned_path               → nav_msgs/Path (TO your executor)
    /obstacle_status            → std_msgs/String ("CLEAR" or "BLOCKED:0.85")
    /goal_check_markers         → visualization_msgs/MarkerArray (RViz debug)
"""

import math
import numpy as np
from typing import Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class ReactiveNavNode(Node):

    def __init__(self):
        super().__init__('reactive_nav')

        # ---- Parameters ----
        self.declare_parameter('safety_distance', 1.0)
        self.declare_parameter('check_width', 0.5)
        self.declare_parameter('check_step', 0.1)
        self.declare_parameter('drone_radius', 0.3)
        self.declare_parameter('flight_altitude', 5.0)
        self.declare_parameter('emergency_stop_enabled', True)

        self.safety_distance = self.get_parameter('safety_distance').value
        self.check_width = self.get_parameter('check_width').value
        self.check_step = self.get_parameter('check_step').value
        self.drone_radius = self.get_parameter('drone_radius').value
        self.flight_altitude = self.get_parameter('flight_altitude').value
        self.emergency_stop = self.get_parameter('emergency_stop_enabled').value

        # ---- State ----
        self.grid_array = None
        self.grid_info = None
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_yaw = 0.0
        self.current_goal = None        # (x, y) the executor wants to go to
        self.path_is_active = False     # True if we've sent a path to executor
        self.was_blocked = False        # Track state change for logging

        # ---- QoS ----
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ---- Subscribers ----
        self.create_subscription(
            OccupancyGrid, '/projected_map',
            self._grid_callback, sensor_qos,
        )

        self.create_subscription(
            PoseStamped, '/zed/zed_node/pose',
            self._pose_callback, sensor_qos,
        )

        # FROM the executor — it publishes goals here
        self.create_subscription(
            PoseStamped, '/goal_pose',
            self._goal_callback, 10,
        )

        # Alternative: body-frame move command
        self.create_subscription(
            Point, '/move_command',
            self._move_command_callback, 10,
        )

        # ---- Publishers ----
        # TO the executor — it subscribes to this
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.status_pub = self.create_publisher(String, '/obstacle_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/goal_check_markers', 10)

        # ---- Continuous monitoring timer (5Hz) ----
        self.create_timer(0.2, self._continuous_check)

        self.get_logger().info(
            f'Reactive nav started | '
            f'safety={self.safety_distance}m | '
            f'Subscribes to /goal_pose from executor | '
            f'Publishes /planned_path to executor'
        )

    # ================================================================
    # Callbacks
    # ================================================================

    def _grid_callback(self, msg: OccupancyGrid):
        """Receive 2D occupancy grid from OctoMap."""
        self.grid_info = msg.info
        width = msg.info.width
        height = msg.info.height
        self.grid_array = np.array(msg.data, dtype=np.int8).reshape((height, width))

    def _pose_callback(self, msg: PoseStamped):
        """Receive drone position and heading."""
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y

        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.drone_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _goal_callback(self, msg: PoseStamped):
        """
        Executor published a goal — check if safe and respond with a path.
        
        If clear: publish /planned_path with [current_pos, goal]
        If blocked: publish /obstacle_status BLOCKED, no path sent
        """
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.current_goal = (goal_x, goal_y)

        self.get_logger().info(
            f'Goal from executor: ({goal_x:.2f}, {goal_y:.2f})'
        )

        self._evaluate_and_respond()

    def _move_command_callback(self, msg: Point):
        """
        Alternative input: body-frame movement.
        Converts to map-frame goal and evaluates.
        """
        forward = msg.x
        left = msg.y

        goal_x = self.drone_x + forward * math.cos(self.drone_yaw) - left * math.sin(self.drone_yaw)
        goal_y = self.drone_y + forward * math.sin(self.drone_yaw) + left * math.cos(self.drone_yaw)

        self.current_goal = (goal_x, goal_y)

        self.get_logger().info(
            f'Move command: fwd={forward:.1f}m left={left:.1f}m → '
            f'goal=({goal_x:.2f}, {goal_y:.2f})'
        )

        self._evaluate_and_respond()

    # ================================================================
    # Core Logic
    # ================================================================

    def _evaluate_and_respond(self):
        """Check if path to current goal is clear, publish path or block."""
        if self.current_goal is None or self.grid_array is None:
            return

        goal_x, goal_y = self.current_goal

        # Direction and distance to goal
        dx = goal_x - self.drone_x
        dy = goal_y - self.drone_y
        distance = math.sqrt(dx * dx + dy * dy)
        direction_yaw = math.atan2(dy, dx)

        # Check for obstacles along the path
        obstacle_dist = self._check_direction(direction_yaw, distance + 0.5)

        if obstacle_dist is not None and obstacle_dist < self.safety_distance:
            # BLOCKED — don't send path
            self.get_logger().warn(
                f'BLOCKED — obstacle at {obstacle_dist:.2f}m toward goal! Holding position.'
            )
            self.path_is_active = False
            self.was_blocked = True

            status = String()
            status.data = f'BLOCKED:{obstacle_dist:.2f}'
            self.status_pub.publish(status)

            # Send empty path to make executor hold position
            empty_path = Path()
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.header.frame_id = 'map'
            self.path_pub.publish(empty_path)

        else:
            # CLEAR — send path: [current_position, goal]
            if obstacle_dist is not None:
                self.get_logger().info(
                    f'CLEAR — nearest obstacle at {obstacle_dist:.2f}m. Sending path.'
                )
            else:
                self.get_logger().info('CLEAR — no obstacles detected. Sending path.')

            self.path_is_active = True
            self.was_blocked = False

            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'

            # Start point: current drone position
            start = PoseStamped()
            start.header = path_msg.header
            start.pose.position.x = self.drone_x
            start.pose.position.y = self.drone_y
            start.pose.position.z = self.flight_altitude
            start.pose.orientation.w = 1.0
            path_msg.poses.append(start)

            # End point: the goal
            end = PoseStamped()
            end.header = path_msg.header
            end.pose.position.x = goal_x
            end.pose.position.y = goal_y
            end.pose.position.z = self.flight_altitude
            end.pose.orientation.w = 1.0
            path_msg.poses.append(end)

            self.path_pub.publish(path_msg)

            status = String()
            status.data = 'CLEAR'
            self.status_pub.publish(status)

    def _continuous_check(self):
        """
        Runs at 5Hz. While the drone is flying toward a goal:
        - Re-check the path for new obstacles
        - If something appeared, send empty path to stop the drone
        """
        if self.grid_array is None:
            return

        # Always check forward direction for RViz visualization
        obstacle_dist = self._check_direction(self.drone_yaw, self.safety_distance + 1.0)
        self._publish_check_markers(obstacle_dist)

        # If we have an active goal and the drone is flying, keep checking
        if self.current_goal is not None and self.path_is_active:
            goal_x, goal_y = self.current_goal
            dx = goal_x - self.drone_x
            dy = goal_y - self.drone_y
            distance = math.sqrt(dx * dx + dy * dy)
            direction_yaw = math.atan2(dy, dx)

            obstacle_dist_to_goal = self._check_direction(direction_yaw, distance + 0.5)

            if obstacle_dist_to_goal is not None and obstacle_dist_to_goal < self.safety_distance:
                if not self.was_blocked:
                    self.get_logger().warn(
                        f'NEW OBSTACLE detected at {obstacle_dist_to_goal:.2f}m while flying! '
                        f'Sending STOP.'
                    )
                    self.was_blocked = True
                    self.path_is_active = False

                    # Send empty path — executor will hold position
                    empty_path = Path()
                    empty_path.header.stamp = self.get_clock().now().to_msg()
                    empty_path.header.frame_id = 'map'
                    self.path_pub.publish(empty_path)

                    status = String()
                    status.data = f'BLOCKED:{obstacle_dist_to_goal:.2f}'
                    self.status_pub.publish(status)

        # Publish status even when idle
        if not self.path_is_active:
            status = String()
            if obstacle_dist is not None and obstacle_dist < self.safety_distance:
                status.data = f'BLOCKED:{obstacle_dist:.2f}'
            else:
                status.data = 'CLEAR'
            self.status_pub.publish(status)

    # ================================================================
    # Obstacle Detection
    # ================================================================

    def _check_direction(
        self, direction_yaw: float, max_distance: float
    ) -> Optional[float]:
        """
        Check for obstacles in a corridor along a direction.
        Returns distance to nearest obstacle, or None if clear.
        """
        if self.grid_array is None or self.grid_info is None:
            return None

        half_width = (self.check_width + self.drone_radius) / 2.0
        resolution = self.grid_info.resolution
        num_steps = int(max_distance / self.check_step)

        perp_yaw = direction_yaw + math.pi / 2.0
        num_lateral = max(1, int(half_width / resolution))

        for step in range(1, num_steps + 1):
            dist = step * self.check_step

            cx = self.drone_x + dist * math.cos(direction_yaw)
            cy = self.drone_y + dist * math.sin(direction_yaw)

            for lat in range(-num_lateral, num_lateral + 1):
                lateral_offset = lat * resolution
                px = cx + lateral_offset * math.cos(perp_yaw)
                py = cy + lateral_offset * math.sin(perp_yaw)

                col = int((px - self.grid_info.origin.position.x) / resolution)
                row = int((py - self.grid_info.origin.position.y) / resolution)

                rows, cols = self.grid_array.shape
                if 0 <= row < rows and 0 <= col < cols:
                    if self.grid_array[row, col] == 100:
                        return dist

        return None

    # ================================================================
    # RViz Visualization
    # ================================================================

    def _publish_check_markers(self, obstacle_dist: Optional[float]):
        """Publish markers showing safety corridor, obstacle, and drone heading."""
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        is_blocked = obstacle_dist is not None and obstacle_dist < self.safety_distance

        # Safety corridor line
        corridor = Marker()
        corridor.header.stamp = stamp
        corridor.header.frame_id = 'map'
        corridor.ns = 'safety_corridor'
        corridor.id = 0
        corridor.type = Marker.LINE_STRIP
        corridor.action = Marker.ADD
        corridor.scale.x = self.check_width + self.drone_radius
        corridor.color.a = 0.3
        corridor.color.r = 1.0 if is_blocked else 0.0
        corridor.color.g = 0.0 if is_blocked else 1.0

        p1 = Point()
        p1.x = self.drone_x
        p1.y = self.drone_y
        p1.z = 0.3
        corridor.points.append(p1)

        check_dist = self.safety_distance
        p2 = Point()
        p2.x = self.drone_x + check_dist * math.cos(self.drone_yaw)
        p2.y = self.drone_y + check_dist * math.sin(self.drone_yaw)
        p2.z = 0.3
        corridor.points.append(p2)
        markers.markers.append(corridor)

        # Path to goal line (if we have a goal)
        if self.current_goal is not None:
            goal_line = Marker()
            goal_line.header.stamp = stamp
            goal_line.header.frame_id = 'map'
            goal_line.ns = 'goal_line'
            goal_line.id = 3
            goal_line.type = Marker.LINE_STRIP
            goal_line.action = Marker.ADD
            goal_line.scale.x = 0.05
            goal_line.color.a = 0.6
            goal_line.color.b = 1.0  # blue line to goal

            gp1 = Point()
            gp1.x = self.drone_x
            gp1.y = self.drone_y
            gp1.z = 0.3
            goal_line.points.append(gp1)

            gp2 = Point()
            gp2.x = self.current_goal[0]
            gp2.y = self.current_goal[1]
            gp2.z = 0.3
            goal_line.points.append(gp2)
            markers.markers.append(goal_line)

            # Goal sphere
            goal_marker = Marker()
            goal_marker.header.stamp = stamp
            goal_marker.header.frame_id = 'map'
            goal_marker.ns = 'goal_point'
            goal_marker.id = 4
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.scale.x = 0.3
            goal_marker.scale.y = 0.3
            goal_marker.scale.z = 0.3
            goal_marker.color.b = 1.0
            goal_marker.color.a = 0.8
            goal_marker.pose.position.x = self.current_goal[0]
            goal_marker.pose.position.y = self.current_goal[1]
            goal_marker.pose.position.z = 0.3
            goal_marker.pose.orientation.w = 1.0
            markers.markers.append(goal_marker)

        # Obstacle hit sphere
        if obstacle_dist is not None:
            obs = Marker()
            obs.header.stamp = stamp
            obs.header.frame_id = 'map'
            obs.ns = 'obstacle_hit'
            obs.id = 1
            obs.type = Marker.SPHERE
            obs.action = Marker.ADD
            obs.scale.x = 0.3
            obs.scale.y = 0.3
            obs.scale.z = 0.3
            obs.color.r = 1.0
            obs.color.a = 0.8
            obs.pose.position.x = self.drone_x + obstacle_dist * math.cos(self.drone_yaw)
            obs.pose.position.y = self.drone_y + obstacle_dist * math.sin(self.drone_yaw)
            obs.pose.position.z = 0.3
            obs.pose.orientation.w = 1.0
            markers.markers.append(obs)
        else:
            obs = Marker()
            obs.header.stamp = stamp
            obs.header.frame_id = 'map'
            obs.ns = 'obstacle_hit'
            obs.id = 1
            obs.action = Marker.DELETE
            markers.markers.append(obs)

        # Drone arrow
        drone_marker = Marker()
        drone_marker.header.stamp = stamp
        drone_marker.header.frame_id = 'map'
        drone_marker.ns = 'drone_pos'
        drone_marker.id = 2
        drone_marker.type = Marker.ARROW
        drone_marker.action = Marker.ADD
        drone_marker.scale.x = 0.6
        drone_marker.scale.y = 0.1
        drone_marker.scale.z = 0.1
        drone_marker.color.b = 1.0
        drone_marker.color.a = 1.0
        drone_marker.pose.position.x = self.drone_x
        drone_marker.pose.position.y = self.drone_y
        drone_marker.pose.position.z = 0.3
        drone_marker.pose.orientation.z = math.sin(self.drone_yaw / 2.0)
        drone_marker.pose.orientation.w = math.cos(self.drone_yaw / 2.0)
        markers.markers.append(drone_marker)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNavNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

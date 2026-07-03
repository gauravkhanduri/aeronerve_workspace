#!/usr/bin/env python3
"""
Reactive Navigation Node for ISR Drone
ROS2 Humble + ArduPilot GUIDED mode

Simple behavior:
    1. Receive a movement command (e.g., "move 1m forward")
    2. Check the occupancy grid in that direction
    3. If clear → move to the target
    4. If obstacle within safety distance → stop and hover
    5. Continuously monitor while moving — stop if obstacle appears

No A* path planning. Just look-before-you-move with live OctoMap data.

Subscribes:
    /projected_map              → nav_msgs/OccupancyGrid (from OctoMap)
    /zed/zed_node/pose          → geometry_msgs/PoseStamped (drone position + heading)

Publishes:
    /obstacle_status            → std_msgs/String (for GUI: "CLEAR" or "BLOCKED")
    /goal_check_markers         → visualization_msgs/MarkerArray (RViz debug)

Usage:
    # Terminal 1: ZED + OctoMap already running
    # Terminal 2:
    python3 reactive_nav_node.py

    # Terminal 3: test commands
    ros2 topic pub /move_command geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 0.0}" --once
"""

import math
import numpy as np
from typing import Tuple, Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class ReactiveNavNode(Node):
    """
    Simple reactive navigation:
    - Check if the path ahead is clear using the OctoMap grid
    - Move if clear, hover if blocked
    """

    def __init__(self):
        super().__init__('reactive_nav')

        # ---- Parameters ----
        self.declare_parameter('safety_distance', 1.0)     # meters — stop if obstacle within this
        self.declare_parameter('check_width', 0.5)          # meters — how wide a corridor to check
        self.declare_parameter('check_step', 0.1)           # meters — step size for ray check
        self.declare_parameter('drone_radius', 0.3)         # meters — inflate check corridor

        self.safety_distance = self.get_parameter('safety_distance').value
        self.check_width = self.get_parameter('check_width').value
        self.check_step = self.get_parameter('check_step').value
        self.drone_radius = self.get_parameter('drone_radius').value

        # ---- State ----
        self.grid_array = None          # numpy 2D array
        self.grid_info = None           # MapMetaData
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_yaw = 0.0           # radians

        # ---- QoS ----
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Try multiple QoS profiles for projected_map since it varies
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ---- Subscribers ----
        self.create_subscription(
            OccupancyGrid, '/projected_map',
            self._grid_callback, map_qos,
        )

        self.create_subscription(
            PoseStamped, '/zed/zed_node/pose',
            self._pose_callback, sensor_qos,
        )

        # Movement command: Point(x=forward, y=left, z=0) in drone body frame
        self.create_subscription(
            Point, '/move_command',
            self._move_command_callback, 10,
        )

        # ---- Publishers ----
        self.status_pub = self.create_publisher(String, '/obstacle_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/goal_check_markers', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # ---- Continuous forward check timer (5Hz) ----
        self.create_timer(0.2, self._check_forward)

        self.get_logger().info(
            f'Reactive nav started | '
            f'safety_distance={self.safety_distance}m | '
            f'check_width={self.check_width}m'
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
        """Receive drone position and extract yaw."""
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.drone_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _move_command_callback(self, msg: Point):
        """
        Receive a movement command in body frame.
        
        msg.x = forward distance (meters, positive = forward)
        msg.y = left distance (meters, positive = left)
        msg.z = ignored (altitude is constant)
        
        Example: move 1m forward → Point(x=1.0, y=0.0, z=0.0)
        Example: move 2m left   → Point(x=0.0, y=2.0, z=0.0)
        Example: move diag      → Point(x=1.0, y=1.0, z=0.0)
        """
        forward = msg.x
        left = msg.y

        # Convert body frame movement to map frame goal
        goal_x = self.drone_x + forward * math.cos(self.drone_yaw) - left * math.sin(self.drone_yaw)
        goal_y = self.drone_y + forward * math.sin(self.drone_yaw) + left * math.cos(self.drone_yaw)

        distance = math.sqrt(forward * forward + left * left)

        self.get_logger().info(
            f'Move command: forward={forward:.1f}m, left={left:.1f}m → '
            f'goal=({goal_x:.2f}, {goal_y:.2f})'
        )

        # Check if the path is clear
        direction_yaw = math.atan2(goal_y - self.drone_y, goal_x - self.drone_x)
        obstacle_dist = self._check_direction(direction_yaw, distance + 0.5)

        if obstacle_dist is not None and obstacle_dist < self.safety_distance:
            self.get_logger().warn(
                f'BLOCKED — obstacle at {obstacle_dist:.2f}m in movement direction! '
                f'Hovering in place.'
            )
            status = String()
            status.data = f'BLOCKED:{obstacle_dist:.2f}'
            self.status_pub.publish(status)
        else:
            if obstacle_dist is not None:
                self.get_logger().info(
                    f'Path clear (nearest obstacle at {obstacle_dist:.2f}m). Moving to goal.'
                )
            else:
                self.get_logger().info('Path clear (no obstacles detected). Moving to goal.')

            # Publish goal — your executor picks this up
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            goal.pose.position.z = 0.0  # your executor handles altitude
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)

            status = String()
            status.data = 'CLEAR'
            self.status_pub.publish(status)

    # ================================================================
    # Obstacle Checking
    # ================================================================

    def _check_forward(self):
        """
        Continuously check the forward direction for obstacles.
        Publishes status and RViz markers at 5Hz.
        """
        if self.grid_array is None:
            return

        # Check forward along the drone's current heading
        obstacle_dist = self._check_direction(self.drone_yaw, self.safety_distance + 1.0)

        # Publish status
        status = String()
        if obstacle_dist is not None and obstacle_dist < self.safety_distance:
            status.data = f'BLOCKED:{obstacle_dist:.2f}'
        else:
            dist_str = f'{obstacle_dist:.2f}' if obstacle_dist else 'clear'
            status.data = f'CLEAR:{dist_str}'
        self.status_pub.publish(status)

        # Publish RViz visualization
        self._publish_check_markers(obstacle_dist)

    def _check_direction(
        self, direction_yaw: float, max_distance: float
    ) -> Optional[float]:
        """
        Cast rays in a direction and return distance to nearest obstacle.
        
        Checks a corridor of width (check_width + drone_radius) centered
        on the direction line. Returns the distance to the first occupied
        cell, or None if no obstacle within max_distance.
        
        Args:
            direction_yaw: Direction to check in map frame (radians)
            max_distance: How far ahead to check (meters)
            
        Returns:
            Distance to nearest obstacle in meters, or None if clear
        """
        if self.grid_array is None or self.grid_info is None:
            return None

        half_width = (self.check_width + self.drone_radius) / 2.0
        resolution = self.grid_info.resolution
        nearest_obstacle = None

        # Sample points along the direction
        num_steps = int(max_distance / self.check_step)

        for step in range(1, num_steps + 1):
            dist = step * self.check_step

            # Center point at this distance
            cx = self.drone_x + dist * math.cos(direction_yaw)
            cy = self.drone_y + dist * math.sin(direction_yaw)

            # Check across the corridor width (perpendicular to direction)
            perp_yaw = direction_yaw + math.pi / 2.0
            num_lateral = max(1, int(half_width / resolution))

            for lat in range(-num_lateral, num_lateral + 1):
                lateral_offset = lat * resolution
                px = cx + lateral_offset * math.cos(perp_yaw)
                py = cy + lateral_offset * math.sin(perp_yaw)

                # Convert to grid coordinates
                col = int((px - self.grid_info.origin.position.x) / resolution)
                row = int((py - self.grid_info.origin.position.y) / resolution)

                rows, cols = self.grid_array.shape
                if 0 <= row < rows and 0 <= col < cols:
                    cell = self.grid_array[row, col]
                    if cell == 100:  # occupied
                        if nearest_obstacle is None or dist < nearest_obstacle:
                            nearest_obstacle = dist
                        return nearest_obstacle  # first hit = nearest

        return nearest_obstacle

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert map coordinates to grid row, col."""
        info = self.grid_info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        return (row, col)

    # ================================================================
    # RViz Visualization
    # ================================================================

    def _publish_check_markers(self, obstacle_dist: Optional[float]):
        """
        Publish markers showing:
        - Green corridor = checked area (clear)
        - Red sphere = obstacle location
        - Yellow line = safety boundary
        """
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Safety zone corridor (green if clear, red if blocked)
        is_blocked = obstacle_dist is not None and obstacle_dist < self.safety_distance

        # Forward check line
        corridor = Marker()
        corridor.header.stamp = stamp
        corridor.header.frame_id = 'map'
        corridor.ns = 'safety_corridor'
        corridor.id = 0
        corridor.type = Marker.LINE_STRIP
        corridor.action = Marker.ADD
        corridor.scale.x = self.check_width + self.drone_radius  # line width
        corridor.color.a = 0.3

        if is_blocked:
            corridor.color.r = 1.0  # red
            corridor.color.g = 0.0
        else:
            corridor.color.r = 0.0
            corridor.color.g = 1.0  # green

        # Start point (drone position)
        p1 = Point()
        p1.x = self.drone_x
        p1.y = self.drone_y
        p1.z = 0.3
        corridor.points.append(p1)

        # End point (safety distance ahead)
        check_dist = self.safety_distance
        p2 = Point()
        p2.x = self.drone_x + check_dist * math.cos(self.drone_yaw)
        p2.y = self.drone_y + check_dist * math.sin(self.drone_yaw)
        p2.z = 0.3
        corridor.points.append(p2)

        markers.markers.append(corridor)

        # Obstacle sphere (if detected)
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
            # Delete old obstacle marker
            obs = Marker()
            obs.header.stamp = stamp
            obs.header.frame_id = 'map'
            obs.ns = 'obstacle_hit'
            obs.id = 1
            obs.action = Marker.DELETE
            markers.markers.append(obs)

        # Drone position marker
        drone_marker = Marker()
        drone_marker.header.stamp = stamp
        drone_marker.header.frame_id = 'map'
        drone_marker.ns = 'drone_pos'
        drone_marker.id = 2
        drone_marker.type = Marker.ARROW
        drone_marker.action = Marker.ADD
        drone_marker.scale.x = 0.6   # arrow length
        drone_marker.scale.y = 0.1   # arrow width
        drone_marker.scale.z = 0.1   # arrow height
        drone_marker.color.b = 1.0
        drone_marker.color.a = 1.0
        drone_marker.pose.position.x = self.drone_x
        drone_marker.pose.position.y = self.drone_y
        drone_marker.pose.position.z = 0.3

        # Yaw as quaternion
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

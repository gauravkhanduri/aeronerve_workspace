#!/usr/bin/env python3
# Author: Gourav Khanduri
# Email:  gauravkhanduri93@gmail.com
"""
Reactive Navigation Node for ISR Drone
ROS2 Humble + ArduPilot GUIDED mode

Graduated speed-limiting behavior:
    1. Receive a movement command (e.g., "move 1m forward")
    2. Check the occupancy grid in that direction
    3. If clear → move to the target at cruise_speed
    4. As obstacle gets closer → smoothly decelerate (graduated zones)
    5. If obstacle within stop_distance → hold position, preserve goal
    6. When obstacle clears → automatically resume toward original goal

Speed zones:
    > slow_distance      → 100% cruise_speed
    stop..slow_distance  → linear ramp from min_speed_fraction to 100%
    < stop_distance      → full stop (hover in place)

No A* path planning. Just look-before-you-move with live OctoMap data.

Subscribes:
    /projected_map              → nav_msgs/OccupancyGrid (from OctoMap)
    /mavros/local_position/pose → geometry_msgs/PoseStamped (drone position + heading)
    /mavros/state               → mavros_msgs/State (FCU mode + armed)
    /move_command               → geometry_msgs/Point (body-frame movement command)
    /cancel_goal                → std_msgs/String (cancel current goal)

Publishes:
    /obstacle_status            → std_msgs/String ("STATUS:fraction:distance")
    /speed_fraction             → std_msgs/Float32 (0.0–1.0)
    /obstacle_distance          → std_msgs/Float32 (-1.0 if none)
    /goal_check_markers         → visualization_msgs/MarkerArray (RViz debug)
    /mavros/setpoint_position/local → geometry_msgs/PoseStamped

Usage:
    # Terminal 1: ZED + OctoMap already running
    # Terminal 2:
    python3 reactive_nav_node.py

    # Terminal 3: test commands
    ros2 topic pub /move_command geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 0.0}" --once
"""

import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Float32
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ReactiveNavNode(Node):
    """
    Reactive navigation with graduated speed limiting.
    Smoothly decelerates as obstacles get closer instead of binary stop/go.
    """

    def __init__(self):
        super().__init__("reactive_nav")

        # ---- Parameters ----
        self.declare_parameter("slow_distance", 5.0)  # meters — start slowing here
        self.declare_parameter("stop_distance", 1.0)  # meters — full stop below this
        self.declare_parameter("min_speed_fraction", 0.1)  # 10% of cruise before full stop
        self.declare_parameter("check_width", 0.5)  # meters — corridor width
        self.declare_parameter("check_step", 0.1)  # meters — ray step size
        self.declare_parameter("drone_radius", 0.4)  # meters — inflate corridor
        self.declare_parameter("cruise_speed", 0.5)  # m/s — max speed toward goal

        self.slow_distance = self.get_parameter("slow_distance").value
        self.stop_distance = self.get_parameter("stop_distance").value
        self.min_speed_fraction = self.get_parameter("min_speed_fraction").value
        self.check_width = self.get_parameter("check_width").value
        self.check_step = self.get_parameter("check_step").value
        self.drone_radius = self.get_parameter("drone_radius").value
        self.cruise_speed = self.get_parameter("cruise_speed").value

        # ---- State ----
        self.grid_array = None  # numpy 2D array
        self.grid_info = None  # MapMetaData
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_yaw = 0.0  # radians
        self.fcu_mode = ""
        self.fcu_armed = False

        # ---- QoS ----
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ---- Subscribers ----
        self.create_subscription(
            OccupancyGrid,
            "/projected_map",
            self._grid_callback,
            map_qos,
        )

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose_callback,
            sensor_qos,
        )

        self.create_subscription(
            State,
            "/mavros/state",
            self._state_callback,
            10,
        )

        # Movement command: Point(x=forward, y=left, z=0) in drone body frame
        self.create_subscription(
            Point,
            "/move_command",
            self._move_command_callback,
            10,
        )

        # Cancel current goal
        self.create_subscription(
            String,
            "/cancel_goal",
            self._cancel_goal_callback,
            10,
        )

        # ---- Navigation state ----
        self.active_setpoint: Optional[PoseStamped] = None  # streamed to MAVROS
        self.final_goal: Optional[Tuple[float, float, float]] = None  # (x, y, z)
        self.setpoint_x = 0.0  # independent setpoint position
        self.setpoint_y = 0.0
        self.current_speed_fraction = 1.0  # 0.0–1.0, updated by _check_forward
        self._last_zone = ""  # for zone transition logging

        # ---- Publishers ----
        self.status_pub = self.create_publisher(String, "/obstacle_status", 10)
        self.speed_pub = self.create_publisher(Float32, "/speed_fraction", 10)
        self.obs_dist_pub = self.create_publisher(Float32, "/obstacle_distance", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/goal_check_markers", 10)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )

        # ---- Timers ----
        self.create_timer(0.2, self._check_forward)   # 5Hz obstacle check
        self.create_timer(0.1, self._stream_setpoint)  # 10Hz setpoint streaming

        self.get_logger().info(
            f"Reactive nav started | "
            f"slow_distance={self.slow_distance}m | "
            f"stop_distance={self.stop_distance}m | "
            f"cruise_speed={self.cruise_speed}m/s"
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
        self.drone_z = msg.pose.position.z

        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.drone_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _state_callback(self, msg: State):
        self.fcu_mode = msg.mode
        self.fcu_armed = msg.armed

    def _cancel_goal_callback(self, msg: String):
        """Cancel current goal and hover in place."""
        if self.final_goal is not None:
            self.get_logger().info("Goal cancelled by operator.")
        self.final_goal = None
        self.current_speed_fraction = 1.0
        hover = PoseStamped()
        hover.header.stamp = self.get_clock().now().to_msg()
        hover.header.frame_id = "map"
        hover.pose.position.x = self.drone_x
        hover.pose.position.y = self.drone_y
        hover.pose.position.z = self.drone_z
        hover.pose.orientation.w = 1.0
        self.active_setpoint = hover

    def _move_command_callback(self, msg: Point):
        """
        Receive a movement command in body frame.

        msg.x = forward distance (meters, positive = forward)
        msg.y = left distance (meters, positive = left)
        msg.z = ignored (altitude is constant)
        """
        if self.fcu_mode != "GUIDED" or not self.fcu_armed:
            self.get_logger().warn(
                f"Move command ignored — FCU mode={self.fcu_mode}, armed={self.fcu_armed}. "
                f"Must be GUIDED and armed."
            )
            return

        forward = msg.x
        left = msg.y

        # Convert body frame movement to map frame goal
        goal_x = (
            self.drone_x + forward * math.cos(self.drone_yaw) - left * math.sin(self.drone_yaw)
        )
        goal_y = (
            self.drone_y + forward * math.sin(self.drone_yaw) + left * math.cos(self.drone_yaw)
        )

        distance = math.sqrt(forward * forward + left * left)

        self.get_logger().info(
            f"Move command: forward={forward:.1f}m, left={left:.1f}m → "
            f"goal=({goal_x:.2f}, {goal_y:.2f})"
        )

        # Check obstacle in the requested direction
        direction_yaw = math.atan2(goal_y - self.drone_y, goal_x - self.drone_x)
        obstacle_dist = self._check_direction(direction_yaw, distance + 0.5)
        fraction = self._compute_speed_fraction(obstacle_dist)

        if fraction == 0.0:
            # Completely blocked — reject command
            self.get_logger().warn(
                f"STOPPED — obstacle at {obstacle_dist:.2f}m (below stop_distance "
                f"{self.stop_distance}m). Command rejected."
            )
            status = String()
            status.data = f"STOP:0.00:{obstacle_dist:.2f}"
            self.status_pub.publish(status)
        else:
            # Accept command — drone will move at reduced speed if needed
            if fraction < 1.0:
                self.get_logger().info(
                    f"Obstacle at {obstacle_dist:.2f}m — accepting at "
                    f"{fraction * 100:.0f}% speed."
                )
            else:
                if obstacle_dist is not None:
                    self.get_logger().info(
                        f"Path clear (nearest obstacle at {obstacle_dist:.2f}m). "
                        f"Moving to goal."
                    )
                else:
                    self.get_logger().info(
                        "Path clear (no obstacles detected). Moving to goal."
                    )

            self.final_goal = (goal_x, goal_y, self.drone_z)
            self.setpoint_x = self.drone_x
            self.setpoint_y = self.drone_y
            self.current_speed_fraction = fraction

    # ================================================================
    # Speed Fraction
    # ================================================================

    def _compute_speed_fraction(self, obstacle_distance: Optional[float]) -> float:
        """
        Compute speed fraction (0.0 to 1.0) based on obstacle proximity.
        Linear interpolation between stop_distance and slow_distance.

        Returns:
            0.0 = full stop
            1.0 = full cruise_speed allowed
        """
        if obstacle_distance is None:
            return 1.0
        if obstacle_distance >= self.slow_distance:
            return 1.0
        if obstacle_distance <= self.stop_distance:
            return 0.0

        # Linear interpolation: stop_distance → min_speed_fraction, slow_distance → 1.0
        t = (obstacle_distance - self.stop_distance) / (self.slow_distance - self.stop_distance)
        return self.min_speed_fraction + t * (1.0 - self.min_speed_fraction)

    def _get_zone_label(self, fraction: float) -> str:
        """Get status label from speed fraction."""
        if fraction == 0.0:
            return "STOP"
        if fraction <= 0.25:
            return "CREEP"
        if fraction <= 0.75:
            return "SLOW"
        return "CLEAR"

    # ================================================================
    # Setpoint Streaming
    # ================================================================

    def _stream_setpoint(self):
        """
        Step active_setpoint toward final_goal at cruise_speed * speed_fraction.
        Called at 10Hz — each tick advances the setpoint by (cruise_speed * fraction * 0.1)m.
        When speed_fraction is 0.0, the setpoint freezes but goal is preserved for auto-resume.
        """
        if self.fcu_mode != "GUIDED" or not self.fcu_armed:
            return

        if self.final_goal is not None:
            gx, gy, gz = self.final_goal

            # Check if drone has reached the goal
            drone_dx = gx - self.drone_x
            drone_dy = gy - self.drone_y
            drone_dist = math.sqrt(drone_dx * drone_dx + drone_dy * drone_dy)

            if drone_dist <= 0.15:
                self.final_goal = None
                self.active_setpoint = None
                self.current_speed_fraction = 1.0
                self.get_logger().info("Goal reached.")
            elif self.current_speed_fraction == 0.0:
                # Stopped — hold position but keep goal for auto-resume
                hover = PoseStamped()
                hover.header.stamp = self.get_clock().now().to_msg()
                hover.header.frame_id = "map"
                hover.pose.position.x = self.drone_x
                hover.pose.position.y = self.drone_y
                hover.pose.position.z = gz
                hover.pose.orientation.w = 1.0
                self.active_setpoint = hover
                # Reset setpoint to drone pos so it doesn't jump ahead on resume
                self.setpoint_x = self.drone_x
                self.setpoint_y = self.drone_y
            else:
                # Advance setpoint toward goal at scaled speed
                dx = gx - self.setpoint_x
                dy = gy - self.setpoint_y
                dist = math.sqrt(dx * dx + dy * dy)
                step = self.cruise_speed * self.current_speed_fraction * 0.1

                if dist <= step:
                    sp_x, sp_y = gx, gy
                else:
                    sp_x = self.setpoint_x + (dx / dist) * step
                    sp_y = self.setpoint_y + (dy / dist) * step

                self.setpoint_x = sp_x
                self.setpoint_y = sp_y

                sp = PoseStamped()
                sp.header.stamp = self.get_clock().now().to_msg()
                sp.header.frame_id = "map"
                sp.pose.position.x = sp_x
                sp.pose.position.y = sp_y
                sp.pose.position.z = gz
                sp.pose.orientation.w = 1.0
                self.active_setpoint = sp

        if self.active_setpoint is None:
            return
        self.active_setpoint.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.active_setpoint)

    # ================================================================
    # Obstacle Checking
    # ================================================================

    def _check_forward(self):
        """
        Continuously check toward the goal (or drone heading) for obstacles.
        Computes graduated speed fraction and publishes telemetry at 5Hz.
        """
        if self.grid_array is None:
            return

        if self.final_goal is not None:
            gx, gy, _ = self.final_goal
            dx = gx - self.drone_x
            dy = gy - self.drone_y
            remaining = math.sqrt(dx * dx + dy * dy)
            check_yaw = math.atan2(dy, dx)
        else:
            check_yaw = self.drone_yaw
            remaining = self.slow_distance + 1.0

        obstacle_dist = self._check_direction(
            check_yaw, min(remaining, self.slow_distance + 1.0)
        )

        # Compute graduated speed fraction
        fraction = self._compute_speed_fraction(obstacle_dist)
        self.current_speed_fraction = fraction

        # Zone transition logging (only log changes)
        zone = self._get_zone_label(fraction)
        if zone != self._last_zone and self.final_goal is not None:
            dist_str = f"{obstacle_dist:.2f}m" if obstacle_dist is not None else "none"
            self.get_logger().info(
                f"Zone: {zone} | speed={fraction * 100:.0f}% | obstacle={dist_str}"
            )
            self._last_zone = zone

        # Publish obstacle_status
        status = String()
        dist_str = f"{obstacle_dist:.2f}" if obstacle_dist is not None else "none"
        status.data = f"{zone}:{fraction:.2f}:{dist_str}"
        self.status_pub.publish(status)

        # Publish speed_fraction
        speed_msg = Float32()
        speed_msg.data = fraction
        self.speed_pub.publish(speed_msg)

        # Publish obstacle_distance
        obs_msg = Float32()
        obs_msg.data = obstacle_dist if obstacle_dist is not None else -1.0
        self.obs_dist_pub.publish(obs_msg)

        # Publish RViz visualization
        self._publish_check_markers(obstacle_dist, fraction)

    def _check_direction(self, direction_yaw: float, max_distance: float) -> Optional[float]:
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

    def _publish_check_markers(self, obstacle_dist: Optional[float], fraction: float):
        """
        Publish markers showing:
        - Color-coded corridor (green/yellow/orange/red based on speed fraction)
        - Red sphere at obstacle location
        - Status text above drone
        - Drone position arrow
        """
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # ---- Corridor color based on speed fraction ----
        if fraction > 0.75:
            cor_r, cor_g, cor_b = 0.0, 1.0, 0.0      # green
        elif fraction > 0.25:
            cor_r, cor_g, cor_b = 1.0, 1.0, 0.0      # yellow
        elif fraction > 0.0:
            cor_r, cor_g, cor_b = 1.0, 0.5, 0.0      # orange
        else:
            cor_r, cor_g, cor_b = 1.0, 0.0, 0.0      # red

        # Direction: toward final_goal if active, else drone heading
        viz_yaw = self.drone_yaw
        if self.final_goal is not None:
            gx, gy, _ = self.final_goal
            viz_yaw = math.atan2(gy - self.drone_y, gx - self.drone_x)

        # Forward check corridor
        corridor = Marker()
        corridor.header.stamp = stamp
        corridor.header.frame_id = "map"
        corridor.ns = "safety_corridor"
        corridor.id = 0
        corridor.type = Marker.LINE_STRIP
        corridor.action = Marker.ADD
        corridor.scale.x = self.check_width + self.drone_radius
        corridor.color.r = cor_r
        corridor.color.g = cor_g
        corridor.color.b = cor_b
        corridor.color.a = 0.3

        p1 = Point()
        p1.x = self.drone_x
        p1.y = self.drone_y
        p1.z = self.drone_z
        corridor.points.append(p1)

        check_dist = self.slow_distance
        p2 = Point()
        p2.x = self.drone_x + check_dist * math.cos(viz_yaw)
        p2.y = self.drone_y + check_dist * math.sin(viz_yaw)
        p2.z = self.drone_z
        corridor.points.append(p2)

        markers.markers.append(corridor)

        # ---- Obstacle sphere ----
        if obstacle_dist is not None:
            obs = Marker()
            obs.header.stamp = stamp
            obs.header.frame_id = "map"
            obs.ns = "obstacle_hit"
            obs.id = 1
            obs.type = Marker.SPHERE
            obs.action = Marker.ADD
            obs.scale.x = 0.3
            obs.scale.y = 0.3
            obs.scale.z = 0.3
            obs.color.r = 1.0
            obs.color.a = 0.8
            obs.pose.position.x = self.drone_x + obstacle_dist * math.cos(viz_yaw)
            obs.pose.position.y = self.drone_y + obstacle_dist * math.sin(viz_yaw)
            obs.pose.position.z = self.drone_z
            obs.pose.orientation.w = 1.0
            markers.markers.append(obs)
        else:
            obs = Marker()
            obs.header.stamp = stamp
            obs.header.frame_id = "map"
            obs.ns = "obstacle_hit"
            obs.id = 1
            obs.action = Marker.DELETE
            markers.markers.append(obs)

        # ---- Drone position arrow ----
        drone_marker = Marker()
        drone_marker.header.stamp = stamp
        drone_marker.header.frame_id = "map"
        drone_marker.ns = "drone_pos"
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
        drone_marker.pose.position.z = self.drone_z
        drone_marker.pose.orientation.z = math.sin(self.drone_yaw / 2.0)
        drone_marker.pose.orientation.w = math.cos(self.drone_yaw / 2.0)
        markers.markers.append(drone_marker)

        # ---- Status text above drone ----
        text_marker = Marker()
        text_marker.header.stamp = stamp
        text_marker.header.frame_id = "map"
        text_marker.ns = "status_text"
        text_marker.id = 3
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.scale.z = 0.3
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.pose.position.x = self.drone_x
        text_marker.pose.position.y = self.drone_y
        text_marker.pose.position.z = self.drone_z + 1.0
        text_marker.pose.orientation.w = 1.0
        dist_str = f"{obstacle_dist:.1f}m" if obstacle_dist is not None else "none"
        text_marker.text = f"SPD: {fraction * 100:.0f}% | OBS: {dist_str}"
        markers.markers.append(text_marker)

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


if __name__ == "__main__":
    main()

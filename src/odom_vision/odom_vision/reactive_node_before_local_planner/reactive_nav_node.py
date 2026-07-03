#!/usr/bin/env python3
# Author: Gourav Khanduri
# Email:  gauravkhanduri93@gmail.com
"""
Reactive Navigation Node for ISR Drone
ROS2 Humble + ArduPilot GUIDED mode

Graduated speed-limiting + Left/Right Avoidance:
    1. Receive a movement command (e.g., "move 10m forward")
    2. Check the occupancy grid in that direction
    3. If clear → move to the target at cruise_speed (DIRECT state)
    4. As obstacle gets closer → smoothly decelerate (graduated zones)
    5. If blocked → check left (90°) and right (90°) of goal direction
    6. Pick the clearer side, fly sideways by avoidance_step (AVOIDING state)
    7. On arrival, re-check direct path to goal → resume or shift again
    8. If both sides blocked → hover and wait (IDLE)

State machine:
    IDLE          → no goal, hovering
    DIRECT        → flying straight toward final_goal
    AVOIDING      → flying toward a temporary avoidance waypoint
    GOAL_REACHED  → arrived at final_goal (transitions to IDLE)

Speed zones (unchanged):
    > slow_distance      → 100% cruise_speed
    stop..slow_distance  → linear ramp from min_speed_fraction to 100%
    < stop_distance      → full stop (hover in place)

Layer 2 — Raw Depth Emergency Stop:
    Reads raw depth image from ZED 2i at 15Hz, bypasses OctoMap.
    If any pixel in center ROI < emergency_distance → zero velocity brake.
    Stays in GUIDED mode. Recovery after obstacle clears + delay.

Subscribes:
    /projected_map              → nav_msgs/OccupancyGrid (from OctoMap)
    /mavros/local_position/pose → geometry_msgs/PoseStamped (drone position + heading)
    /mavros/state               → mavros_msgs/State (FCU mode + armed)
    /move_command               → geometry_msgs/Point (body-frame movement command)
    /cancel_goal                → std_msgs/String (cancel current goal)
    /zed/zed_node/depth/depth_registered → sensor_msgs/Image (32FC1 raw depth)

Publishes:
    /obstacle_status            → std_msgs/String ("NAV_STATE:SPEED_STATUS:fraction:distance")
    /speed_fraction             → std_msgs/Float32 (0.0–1.0)
    /obstacle_distance          → std_msgs/Float32 (-1.0 if none)
    /nav_state                  → std_msgs/String (IDLE|DIRECT|AVOIDING|GOAL_REACHED)
    /goal_check_markers         → visualization_msgs/MarkerArray (RViz debug)
    /mavros/setpoint_position/local → geometry_msgs/PoseStamped
    /mavros/setpoint_raw/local  → mavros_msgs/PositionTarget (zero velocity for emergency)
    /emergency_active           → std_msgs/Bool
    /min_depth                  → std_msgs/Float32 (closest pixel in ROI)

Usage:
    # Terminal 1: ZED + OctoMap already running
    # Terminal 2:
    python3 reactive_nav_node.py

    # Terminal 3: test commands
    ros2 topic pub /move_command geometry_msgs/msg/Point "{x: 10.0, y: 0.0, z: 0.0}" --once
"""

import math
from typing import Optional, Tuple

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ReactiveNavNode(Node):
    """
    Reactive navigation with graduated speed limiting and
    simple left/right obstacle avoidance.
    """

    def __init__(self):
        super().__init__("reactive_nav")

        # ---- Parameters ----
        self.declare_parameter("cruise_speed", 4.0)
        self.declare_parameter("min_speed_fraction", 0.1)
        self.declare_parameter("check_width", 0.5)
        self.declare_parameter("check_step", 0.1)
        self.declare_parameter("drone_radius", 0.4)

        self.cruise_speed = self.get_parameter("cruise_speed").value
        self.min_speed_fraction = self.get_parameter("min_speed_fraction").value
        self.check_width = self.get_parameter("check_width").value
        self.check_step = self.get_parameter("check_step").value
        self.drone_radius = self.get_parameter("drone_radius").value

        # Auto-scale distances from cruise_speed:
        #   slow_distance = 2x cruise_speed (at 1 m/s → 2m, at 5 m/s → 10m)
        #   stop_distance = 0.75x cruise_speed, min 1.5m
        #   avoidance_step scales below
        self.slow_distance = max(3.0, 2.0 * self.cruise_speed)
        self.stop_distance = max(1.5, 0.75 * self.cruise_speed)

        # ---- Avoidance Parameters ----
        self.declare_parameter("avoid_threshold", 0.3)
        self.declare_parameter("max_avoidance_attempts", 10)

        self.avoid_threshold = self.get_parameter("avoid_threshold").value
        # Auto-scale from cruise_speed
        self.scan_range = max(4.0, self.slow_distance + 2.0)
        self.min_gap_depth = self.stop_distance
        self.avoidance_step = max(2.0, 0.75 * self.cruise_speed)
        self.max_avoidance_attempts = self.get_parameter("max_avoidance_attempts").value

        # ---- Emergency (Layer 2) Parameters ----
        self.declare_parameter("emergency_distance_base", 1.5)  # base meters (below stop_distance)
        self.declare_parameter("emergency_speed_factor", 0.8)  # seconds of travel added
        self.declare_parameter("emergency_recovery_delay", 1.0)
        self.declare_parameter("depth_roi_width_pct", 0.3)
        self.declare_parameter("depth_roi_height_pct", 0.5)
        self.declare_parameter("depth_min_valid", 0.3)
        self.declare_parameter("depth_max_valid", 20.0)

        self.emergency_distance_base = self.get_parameter("emergency_distance_base").value
        self.emergency_speed_factor = self.get_parameter("emergency_speed_factor").value
        self.emergency_recovery_delay = self.get_parameter("emergency_recovery_delay").value
        self.depth_roi_width_pct = self.get_parameter("depth_roi_width_pct").value
        self.depth_roi_height_pct = self.get_parameter("depth_roi_height_pct").value
        self.depth_min_valid = self.get_parameter("depth_min_valid").value
        self.depth_max_valid = self.get_parameter("depth_max_valid").value

        # ---- State ----
        self.grid_array = None
        self.grid_info = None
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_yaw = 0.0
        self._prev_drone_x = 0.0
        self._prev_drone_y = 0.0
        self._prev_pose_time = None
        self.drone_speed = 0.0  # actual speed from position delta (m/s)
        self.fcu_mode = ""
        self.fcu_armed = False

        # ---- Navigation state ----
        self.active_setpoint: Optional[PoseStamped] = None
        self.final_goal: Optional[Tuple[float, float, float]] = None
        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        self.current_speed_fraction = 1.0
        self._last_zone = ""

        # ---- Avoidance state ----
        self.nav_state = "IDLE"
        self.avoidance_waypoint: Optional[Tuple[float, float]] = None
        self.avoidance_attempts = 0
        self._avoidance_phase = (
            "SHIFT"  # "SHIFT" = moving sideways, "FORWARD" = moving past obstacle
        )
        self._goal_direction_yaw = 0.0  # goal yaw at time of avoidance trigger (for forward phase)
        self.last_lr_check: Optional[dict] = None
        self._committed_avoidance_side = None  # "LEFT" or "RIGHT", locked on first trigger
        self._goal_reached_timer = None

        # ---- Emergency (Layer 2) state ----
        self.emergency_active = False
        self.emergency_start_time = None
        self.last_min_depth = float("inf")

        # ---- QoS ----
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ---- Subscribers ----
        self.create_subscription(OccupancyGrid, "/projected_map", self._grid_callback, map_qos)
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose_callback,
            sensor_qos,
        )
        self.create_subscription(State, "/mavros/state", self._state_callback, 10)
        self.create_subscription(Point, "/move_command", self._move_command_callback, 10)
        self.create_subscription(String, "/cancel_goal", self._cancel_goal_callback, 10)
        # Use /depth_camera for sim (Gazebo), /zed2/zed_node/depth/depth_registered for real
        self.declare_parameter("depth_topic", "/zed2/zed_node/depth/depth_registered")
        depth_topic = self.get_parameter("depth_topic").value
        self.create_subscription(
            Image,
            depth_topic,
            self._depth_callback,
            sensor_qos,
        )

        # ---- Publishers ----
        self.status_pub = self.create_publisher(String, "/obstacle_status", 10)
        self.speed_pub = self.create_publisher(Float32, "/speed_fraction", 10)
        self.obs_dist_pub = self.create_publisher(Float32, "/obstacle_distance", 10)
        self.nav_state_pub = self.create_publisher(String, "/nav_state", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/goal_check_markers", 10)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        self.raw_setpoint_pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10
        )
        self.emergency_status_pub = self.create_publisher(Bool, "/emergency_active", 10)
        self.min_depth_pub = self.create_publisher(Float32, "/min_depth", 10)

        # ---- Timers ----
        self.create_timer(0.2, self._check_forward)  # 5Hz obstacle check
        self.create_timer(0.1, self._stream_setpoint)  # 10Hz setpoint streaming
        self.create_timer(0.2, self._publish_nav_state)  # 5Hz nav state

        self.get_logger().info(
            f"Reactive nav started | cruise={self.cruise_speed}m/s | "
            f"slow={self.slow_distance:.1f}m | stop={self.stop_distance:.1f}m | "
            f"avoidance_step={self.avoidance_step:.1f}m | "
            f"scan_range={self.scan_range:.1f}m | "
            f"emergency={self.emergency_distance_base}+{self.emergency_speed_factor}*speed"
        )

    # ================================================================
    # Callbacks
    # ================================================================

    def _grid_callback(self, msg: OccupancyGrid):
        self.grid_info = msg.info
        width = msg.info.width
        height = msg.info.height
        self.grid_array = np.array(msg.data, dtype=np.int8).reshape((height, width))

    def _pose_callback(self, msg: PoseStamped):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        self.drone_z = msg.pose.position.z
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.drone_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Compute actual speed from position delta
        now = self.get_clock().now()
        if self._prev_pose_time is not None:
            dt = (now - self._prev_pose_time).nanoseconds / 1e9
            if dt > 0.001:
                dx = self.drone_x - self._prev_drone_x
                dy = self.drone_y - self._prev_drone_y
                raw_speed = math.sqrt(dx * dx + dy * dy) / dt
                raw_speed = min(raw_speed, 15.0)  # sanity clamp
                self.drone_speed = 0.7 * self.drone_speed + 0.3 * raw_speed
        self._prev_drone_x = self.drone_x
        self._prev_drone_y = self.drone_y
        self._prev_pose_time = now

    def _state_callback(self, msg: State):
        self.fcu_mode = msg.mode
        self.fcu_armed = msg.armed

    def _cancel_goal_callback(self, msg: String):
        if self.final_goal is not None:
            self.get_logger().info("Goal cancelled by operator.")
        self.final_goal = None
        self.avoidance_waypoint = None
        self.avoidance_attempts = 0
        self.nav_state = "IDLE"
        self._avoidance_phase = "SHIFT"
        self.last_lr_check = None
        self._committed_avoidance_side = None
        self.current_speed_fraction = 1.0
        hover = PoseStamped()
        hover.header.stamp = self.get_clock().now().to_msg()
        hover.header.frame_id = "map"
        hover.pose.position.x = self.drone_x
        hover.pose.position.y = self.drone_y
        hover.pose.position.z = self.drone_z
        hover.pose.orientation.z = math.sin(self.drone_yaw / 2.0)
        hover.pose.orientation.w = math.cos(self.drone_yaw / 2.0)
        self.active_setpoint = hover

    def _move_command_callback(self, msg: Point):
        if self.emergency_active:
            self.get_logger().warn("Move command rejected — emergency stop is active!")
            return
        if self.fcu_mode != "GUIDED" or not self.fcu_armed:
            self.get_logger().warn(
                f"Move command ignored — FCU mode={self.fcu_mode}, "
                f"armed={self.fcu_armed}. Must be GUIDED and armed."
            )
            return

        if self._goal_reached_timer is not None:
            self._goal_reached_timer.cancel()
            self._goal_reached_timer = None

        forward = msg.x
        left = msg.y

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

        self.final_goal = (goal_x, goal_y, self.drone_z)
        self.setpoint_x = self.drone_x
        self.setpoint_y = self.drone_y
        self.avoidance_waypoint = None
        self.avoidance_attempts = 0
        self._avoidance_phase = "SHIFT"
        self.last_lr_check = None
        self._committed_avoidance_side = None

        # Check initial path
        direction_yaw = math.atan2(goal_y - self.drone_y, goal_x - self.drone_x)
        obstacle_dist = self._check_direction(direction_yaw, distance + 0.5)
        fraction = self._compute_speed_fraction(obstacle_dist)

        if fraction <= self.avoid_threshold:
            self.get_logger().info(
                f"Direct path blocked (fraction={fraction:.2f}). Scanning for gaps..."
            )
            self.current_speed_fraction = fraction
            self._trigger_avoidance()
        else:
            self.nav_state = "DIRECT"
            self.current_speed_fraction = fraction
            if fraction < 1.0:
                self.get_logger().info(
                    f"Obstacle at {obstacle_dist:.2f}m — moving at "
                    f"{fraction * 100:.0f}% speed."
                )
            else:
                if obstacle_dist is not None:
                    self.get_logger().info(
                        f"Path clear (nearest obstacle at {obstacle_dist:.2f}m). "
                        f"Moving to goal."
                    )
                else:
                    self.get_logger().info("Path clear (no obstacles detected). Moving to goal.")

    def _publish_nav_state(self):
        msg = String()
        msg.data = self.nav_state
        self.nav_state_pub.publish(msg)

    # ================================================================
    # Speed Fraction
    # ================================================================

    def _compute_speed_fraction(self, obstacle_distance: Optional[float]) -> float:
        if obstacle_distance is None:
            return 1.0
        if obstacle_distance >= self.slow_distance:
            return 1.0
        if obstacle_distance <= self.stop_distance:
            return 0.0
        t = (obstacle_distance - self.stop_distance) / (self.slow_distance - self.stop_distance)
        return self.min_speed_fraction + t * (1.0 - self.min_speed_fraction)

    def _get_zone_label(self, fraction: float) -> str:
        if fraction == 0.0:
            return "STOP"
        if fraction <= 0.25:
            return "CREEP"
        if fraction <= 0.75:
            return "SLOW"
        return "CLEAR"

    # ================================================================
    # Left/Right Avoidance
    # ================================================================

    def _find_avoidance_direction(self) -> Optional[float]:
        """
        Simple left/right check. No full scan.
        Returns the yaw to steer toward, or None if both sides blocked.
        Once a side is chosen, it is locked for subsequent attempts on the
        same goal to prevent zigzagging.
        """
        goal_yaw = math.atan2(
            self.final_goal[1] - self.drone_y,
            self.final_goal[0] - self.drone_x,
        )

        left_yaw = goal_yaw + math.pi / 2  # 90° left of goal direction
        right_yaw = goal_yaw - math.pi / 2  # 90° right of goal direction

        left_dist = self._check_direction(left_yaw, self.scan_range)
        right_dist = self._check_direction(right_yaw, self.scan_range)

        # Treat None (no obstacle) as max range
        left_clear = left_dist if left_dist is not None else self.scan_range
        right_clear = right_dist if right_dist is not None else self.scan_range

        # Store for RViz visualization
        self.last_lr_check = {
            "left_yaw": left_yaw,
            "left_dist": left_clear,
            "right_yaw": right_yaw,
            "right_dist": right_clear,
            "chosen": None,
        }

        # Both blocked
        if left_clear < self.min_gap_depth and right_clear < self.min_gap_depth:
            return None

        # If already committed to a side, keep it (prevent zigzag)
        if self._committed_avoidance_side == "LEFT":
            if left_clear >= self.min_gap_depth:
                self.last_lr_check["chosen"] = left_yaw
                return left_yaw
            # Committed side now blocked — fall back to other side
            self._committed_avoidance_side = "RIGHT"
            self.last_lr_check["chosen"] = right_yaw
            return right_yaw

        if self._committed_avoidance_side == "RIGHT":
            if right_clear >= self.min_gap_depth:
                self.last_lr_check["chosen"] = right_yaw
                return right_yaw
            # Committed side now blocked — fall back to other side
            self._committed_avoidance_side = "LEFT"
            self.last_lr_check["chosen"] = left_yaw
            return left_yaw

        # First time — pick the clearer side and lock it
        if left_clear >= right_clear:
            self._committed_avoidance_side = "LEFT"
            self.last_lr_check["chosen"] = left_yaw
            return left_yaw
        else:
            self._committed_avoidance_side = "RIGHT"
            self.last_lr_check["chosen"] = right_yaw
            return right_yaw

    def _trigger_avoidance(self):
        """Check left/right and generate avoidance waypoint."""
        if self.final_goal is None:
            return

        if self.avoidance_attempts >= self.max_avoidance_attempts:
            self.get_logger().error(
                f"Max avoidance attempts ({self.max_avoidance_attempts}) reached. "
                f"Giving up — hovering in place."
            )
            self.nav_state = "IDLE"
            self.avoidance_waypoint = None
            self.final_goal = None
            self.avoidance_attempts = 0
            self._avoidance_phase = "SHIFT"
            self.last_lr_check = None
            self._committed_avoidance_side = None
            return

        best_yaw = self._find_avoidance_direction()

        if best_yaw is None:
            self.get_logger().warn("Both left and right blocked. Hovering.")
            self.nav_state = "IDLE"
            self.avoidance_waypoint = None
            return

        goal_yaw = math.atan2(
            self.final_goal[1] - self.drone_y,
            self.final_goal[0] - self.drone_x,
        )

        avoid_x = self.drone_x + self.avoidance_step * math.cos(best_yaw)
        avoid_y = self.drone_y + self.avoidance_step * math.sin(best_yaw)
        self.avoidance_waypoint = (avoid_x, avoid_y)
        self.nav_state = "AVOIDING"
        self._avoidance_phase = "SHIFT"
        self._goal_direction_yaw = goal_yaw  # remember goal direction for forward phase
        self.avoidance_attempts += 1
        self.setpoint_x = self.drone_x
        self.setpoint_y = self.drone_y

        direction = "left" if self._normalize_angle(best_yaw - goal_yaw) > 0 else "right"
        self.get_logger().info(
            f"Avoiding: shifting {direction} by {self.avoidance_step}m | "
            f"attempt {self.avoidance_attempts}/{self.max_avoidance_attempts} | "
            f"waypoint=({avoid_x:.2f}, {avoid_y:.2f})"
        )

    def _avoidance_waypoint_reached(self):
        """
        Reached avoidance waypoint. Two-phase L-shaped avoidance:

        Phase SHIFT:  drone just moved sideways. Now go FORWARD (parallel
                      to the original goal direction) to clear the obstacle
                      before cutting back diagonally.
        Phase FORWARD: drone moved forward past the obstacle. Now check if
                       the direct path to goal is clear.
        """
        if self.final_goal is None:
            self.nav_state = "IDLE"
            self.avoidance_waypoint = None
            return

        gx, gy, _ = self.final_goal

        # Check if we've also reached the final goal
        goal_dist = math.sqrt((gx - self.drone_x) ** 2 + (gy - self.drone_y) ** 2)
        if goal_dist <= 0.3:
            self.get_logger().info("Goal reached during avoidance!")
            self._enter_goal_reached()
            return

        if self._avoidance_phase == "SHIFT":
            # Just finished shifting sideways — now move FORWARD
            # Use the goal direction saved at trigger time so we go
            # parallel to the original approach, not diagonal.
            fwd_yaw = self._goal_direction_yaw
            fwd_dist = self._check_direction(fwd_yaw, self.avoidance_step + 0.5)
            fwd_fraction = self._compute_speed_fraction(fwd_dist)

            if fwd_fraction > self.avoid_threshold:
                # Forward is clear — place waypoint straight ahead
                fwd_x = self.drone_x + self.avoidance_step * math.cos(fwd_yaw)
                fwd_y = self.drone_y + self.avoidance_step * math.sin(fwd_yaw)
                self.avoidance_waypoint = (fwd_x, fwd_y)
                self._avoidance_phase = "FORWARD"
                self.setpoint_x = self.drone_x
                self.setpoint_y = self.drone_y
                self.get_logger().info(
                    f"Shifted sideways. Now moving forward past obstacle → "
                    f"({fwd_x:.2f}, {fwd_y:.2f})"
                )
            else:
                # Forward also blocked at this lateral offset — shift more
                self.get_logger().info("Forward still blocked after shift. Shifting further...")
                self._trigger_avoidance()

        elif self._avoidance_phase == "FORWARD":
            # Just moved forward — now check direct path to goal
            direction_yaw = math.atan2(gy - self.drone_y, gx - self.drone_x)
            obstacle_dist = self._check_direction(
                direction_yaw, min(goal_dist, self.slow_distance + 1.0)
            )
            fraction = self._compute_speed_fraction(obstacle_dist)

            if fraction > self.avoid_threshold:
                self.get_logger().info(
                    f"Past the obstacle! Resuming direct approach " f"(fraction={fraction:.2f})."
                )
                self.nav_state = "DIRECT"
                self.avoidance_waypoint = None
                self.avoidance_attempts = 0
                self.last_lr_check = None
                self._avoidance_phase = "SHIFT"
                self.current_speed_fraction = fraction
                self.setpoint_x = self.drone_x
                self.setpoint_y = self.drone_y
            else:
                # Still blocked — need another L-shaped maneuver
                self.get_logger().info("Goal still blocked after forward pass. Shifting again...")
                self._trigger_avoidance()

    # ================================================================
    # Setpoint Streaming
    # ================================================================

    def _stream_setpoint(self):
        """
        Step active_setpoint toward current target at cruise_speed * speed_fraction.
        In DIRECT → target is final_goal.
        In AVOIDING → target is avoidance_waypoint.
        """
        if self.fcu_mode != "GUIDED" or not self.fcu_armed:
            return

        # Layer 2: if emergency active, publish zero velocity instead
        if self.emergency_active:
            self._publish_zero_velocity()
            return

        # Select target based on nav_state
        if self.nav_state == "AVOIDING" and self.avoidance_waypoint is not None:
            current_target = (
                self.avoidance_waypoint[0],
                self.avoidance_waypoint[1],
                self.drone_z,
            )
        elif self.nav_state == "DIRECT" and self.final_goal is not None:
            current_target = self.final_goal
        else:
            # IDLE or no target — hold position
            if self.active_setpoint is not None:
                self.active_setpoint.header.stamp = self.get_clock().now().to_msg()
                self.setpoint_pub.publish(self.active_setpoint)
            return

        tx, ty, tz = current_target

        # Check if drone has reached the current target
        drone_dx = tx - self.drone_x
        drone_dy = ty - self.drone_y
        drone_dist = math.sqrt(drone_dx * drone_dx + drone_dy * drone_dy)

        if self.nav_state == "DIRECT" and drone_dist <= 0.15:
            self.get_logger().info("Goal reached.")
            self._enter_goal_reached()
            return

        if self.nav_state == "AVOIDING" and drone_dist <= 0.3:
            self._avoidance_waypoint_reached()
            return

        if self.current_speed_fraction == 0.0:
            if self.nav_state == "DIRECT":
                # In DIRECT mode, speed_fraction 0 triggers avoidance
                # (handled in _check_forward), just hold position here
                pass
            # Hold position
            if self.active_setpoint is not None:
                self.active_setpoint.header.stamp = self.get_clock().now().to_msg()
                self.setpoint_pub.publish(self.active_setpoint)
            return

        # Advance setpoint toward target at scaled speed
        dx = tx - self.setpoint_x
        dy = ty - self.setpoint_y
        dist = math.sqrt(dx * dx + dy * dy)
        step = self.cruise_speed * self.current_speed_fraction * 0.1

        if dist <= step:
            sp_x, sp_y = tx, ty
        else:
            sp_x = self.setpoint_x + (dx / dist) * step
            sp_y = self.setpoint_y + (dy / dist) * step

        self.setpoint_x = sp_x
        self.setpoint_y = sp_y

        # Use final_goal altitude if available, else current target altitude
        altitude = self.final_goal[2] if self.final_goal is not None else tz

        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = sp_x
        sp.pose.position.y = sp_y
        sp.pose.position.z = altitude
        # During SHIFT phase, keep facing goal direction (not sideways waypoint)
        # so the depth camera stays forward-looking and the drone doesn't snap 90°
        if (
            self.nav_state == "AVOIDING"
            and self._avoidance_phase == "SHIFT"
            and self.final_goal is not None
        ):
            target_yaw = math.atan2(
                self.final_goal[1] - self.drone_y,
                self.final_goal[0] - self.drone_x,
            )
        else:
            target_yaw = math.atan2(ty - self.drone_y, tx - self.drone_x)
        sp.pose.orientation.z = math.sin(target_yaw / 2.0)
        sp.pose.orientation.w = math.cos(target_yaw / 2.0)
        self.active_setpoint = sp
        self.setpoint_pub.publish(self.active_setpoint)

    # ================================================================
    # Obstacle Checking
    # ================================================================

    def _check_forward(self):
        """
        Continuously check toward current target for obstacles.
        Triggers avoidance when speed_fraction drops below avoid_threshold in DIRECT mode.
        Re-scans if a new obstacle appears in the avoidance path.
        """
        if self.grid_array is None:
            return

        # Determine which direction to check
        if self.nav_state == "AVOIDING" and self.avoidance_waypoint is not None:
            ax, ay = self.avoidance_waypoint
            dx = ax - self.drone_x
            dy = ay - self.drone_y
            remaining = math.sqrt(dx * dx + dy * dy)
            check_yaw = math.atan2(dy, dx)
        elif self.final_goal is not None:
            gx, gy, _ = self.final_goal
            dx = gx - self.drone_x
            dy = gy - self.drone_y
            remaining = math.sqrt(dx * dx + dy * dy)
            check_yaw = math.atan2(dy, dx)
        else:
            check_yaw = self.drone_yaw
            remaining = self.slow_distance + 1.0

        obstacle_dist = self._check_direction(check_yaw, min(remaining, self.slow_distance + 1.0))
        fraction = self._compute_speed_fraction(obstacle_dist)
        self.current_speed_fraction = fraction

        # Avoidance trigger logic
        if self.nav_state == "DIRECT" and fraction <= self.avoid_threshold:
            self.get_logger().info(
                f"Direct path blocked (fraction={fraction:.2f}). " f"Triggering avoidance..."
            )
            self._trigger_avoidance()

        # If avoiding and a NEW obstacle blocks the avoidance path completely
        if self.nav_state == "AVOIDING" and fraction == 0.0:
            self.get_logger().warn("Obstacle in avoidance path! Re-scanning...")
            self._trigger_avoidance()

        # Zone transition logging
        zone = self._get_zone_label(fraction)
        if zone != self._last_zone and (
            self.final_goal is not None or self.avoidance_waypoint is not None
        ):
            dist_str = f"{obstacle_dist:.2f}m" if obstacle_dist is not None else "none"
            self.get_logger().info(
                f"Zone: {zone} | speed={fraction * 100:.0f}% | obstacle={dist_str}"
            )
            self._last_zone = zone

        # Publish obstacle_status: "NAV_STATE:SPEED_STATUS:fraction:distance"
        status = String()
        dist_str = f"{obstacle_dist:.2f}" if obstacle_dist is not None else "none"
        status.data = f"{self.nav_state}:{zone}:{fraction:.2f}:{dist_str}"
        self.status_pub.publish(status)

        speed_msg = Float32()
        speed_msg.data = fraction
        self.speed_pub.publish(speed_msg)

        obs_msg = Float32()
        obs_msg.data = obstacle_dist if obstacle_dist is not None else -1.0
        self.obs_dist_pub.publish(obs_msg)

        self._publish_check_markers(obstacle_dist, fraction)

    def _check_direction(self, direction_yaw: float, max_distance: float) -> Optional[float]:
        """
        Cast rays in a direction and return distance to nearest obstacle.

        Checks a corridor of width (check_width + drone_radius) centered
        on the direction line. Returns the distance to the first occupied
        cell, or None if no obstacle within max_distance.
        """
        if self.grid_array is None or self.grid_info is None:
            return None

        half_width = (self.check_width + self.drone_radius) / 2.0
        resolution = self.grid_info.resolution
        nearest_obstacle = None

        num_steps = int(max_distance / self.check_step)

        for step in range(1, num_steps + 1):
            dist = step * self.check_step
            cx = self.drone_x + dist * math.cos(direction_yaw)
            cy = self.drone_y + dist * math.sin(direction_yaw)

            perp_yaw = direction_yaw + math.pi / 2.0
            num_lateral = max(1, int(half_width / resolution))

            for lat in range(-num_lateral, num_lateral + 1):
                lateral_offset = lat * resolution
                px = cx + lateral_offset * math.cos(perp_yaw)
                py = cy + lateral_offset * math.sin(perp_yaw)

                col = int((px - self.grid_info.origin.position.x) / resolution)
                row = int((py - self.grid_info.origin.position.y) / resolution)

                rows, cols = self.grid_array.shape
                if 0 <= row < rows and 0 <= col < cols:
                    cell = self.grid_array[row, col]
                    if cell == 100:
                        if nearest_obstacle is None or dist < nearest_obstacle:
                            nearest_obstacle = dist
                        return nearest_obstacle

        return nearest_obstacle

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        info = self.grid_info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        return (row, col)

    # ================================================================
    # Helpers
    # ================================================================

    def _enter_goal_reached(self):
        """Transition to GOAL_REACHED, then auto-transition to IDLE after ~1s."""
        self.nav_state = "GOAL_REACHED"
        self.final_goal = None
        self.active_setpoint = None
        self.current_speed_fraction = 1.0
        self.avoidance_waypoint = None
        self.avoidance_attempts = 0
        self._avoidance_phase = "SHIFT"
        self.last_lr_check = None
        self._goal_reached_timer = self.create_timer(1.0, self._goal_reached_to_idle)

    def _goal_reached_to_idle(self):
        """One-shot callback: transition GOAL_REACHED → IDLE."""
        if self.nav_state == "GOAL_REACHED":
            self.nav_state = "IDLE"
        if self._goal_reached_timer is not None:
            self._goal_reached_timer.cancel()
            self._goal_reached_timer = None

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ================================================================
    # Layer 2: Raw Depth Emergency Stop
    # ================================================================

    def _depth_callback(self, msg: Image):
        """
        Layer 2: Process raw depth image for emergency obstacle detection.
        Runs at camera framerate (~15Hz). Bypasses OctoMap completely.
        Checks center ROI for dangerously close obstacles.
        """
        if self.fcu_mode != "GUIDED" or not self.fcu_armed:
            return

        # Convert 32FC1 Image to numpy
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

        # Extract center ROI
        h, w = depth.shape
        roi_y_start = int(h * (0.5 - self.depth_roi_height_pct / 2))
        roi_y_end = int(h * (0.5 + self.depth_roi_height_pct / 2))
        roi_x_start = int(w * (0.5 - self.depth_roi_width_pct / 2))
        roi_x_end = int(w * (0.5 + self.depth_roi_width_pct / 2))
        roi = depth[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

        # Filter invalid pixels (nan, inf, out of range)
        valid_mask = (
            np.isfinite(roi) & (roi >= self.depth_min_valid) & (roi <= self.depth_max_valid)
        )
        valid_depths = roi[valid_mask]

        if len(valid_depths) == 0:
            return

        min_depth = float(np.min(valid_depths))
        self.last_min_depth = min_depth

        # Publish min depth for monitoring
        depth_msg = Float32()
        depth_msg.data = min_depth
        self.min_depth_pub.publish(depth_msg)

        # Layer 2 emergency is a SAFETY NET for high-speed flight only.
        # At low speed, Layer 1 (OctoMap graduated slowdown) handles it.
        # Only trigger when drone is moving fast enough that Layer 1
        # can't stop in time.
        #
        # Speed-scaled threshold:
        #   At < 1 m/s: DON'T TRIGGER (Layer 1 handles it)
        #   At 2 m/s: 1.5 + 0.8*2 = 3.1m
        #   At 5 m/s: 1.5 + 0.8*5 = 5.5m
        emergency_dist = (
            self.emergency_distance_base + self.emergency_speed_factor * self.drone_speed
        )

        if not self.emergency_active:
            # Only trigger if actually moving fast (> 1 m/s)
            if (
                self.drone_speed > 1.0
                and min_depth < emergency_dist
                and self.nav_state in ("DIRECT", "AVOIDING")
            ):
                self._trigger_emergency(min_depth, emergency_dist)
        else:
            # Keep braking every depth frame (15Hz)
            self._publish_zero_velocity()
            elapsed = (self.get_clock().now() - self.emergency_start_time).nanoseconds / 1e9
            # Release once drone has stopped (speed < 0.3 m/s) and
            # had time to decelerate
            if elapsed > 10.0:
                self.get_logger().error("Emergency timeout — force releasing after 10s")
                self._release_emergency(min_depth)
            elif elapsed > self.emergency_recovery_delay and self.drone_speed < 0.3:
                self._release_emergency(min_depth)

    def _trigger_emergency(self, min_depth: float, trigger_dist: float):
        """
        Activate emergency stop. Publish zero velocity to brake instantly.
        Stays in GUIDED mode — no flight mode switching.
        """
        self.emergency_active = True
        self.emergency_start_time = self.get_clock().now()

        bool_msg = Bool()
        bool_msg.data = True
        self.emergency_status_pub.publish(bool_msg)

        self.get_logger().error(
            f"EMERGENCY STOP — obstacle at {min_depth:.2f}m "
            f"(threshold {trigger_dist:.1f}m at "
            f"{self.drone_speed:.1f}m/s actual). "
            f"Zero velocity."
        )

        self._publish_zero_velocity()

    def _release_emergency(self, min_depth: float):
        """
        Release emergency stop. Resume previous navigation state —
        don't cancel goal so avoidance can continue.
        """
        self.emergency_active = False
        self.emergency_start_time = None

        # Reset setpoint to current position so _stream_setpoint doesn't
        # jump back to a stale waypoint the drone overshot during braking
        self.setpoint_x = self.drone_x
        self.setpoint_y = self.drone_y

        bool_msg = Bool()
        bool_msg.data = False
        self.emergency_status_pub.publish(bool_msg)

        if self.final_goal is not None:
            # Goal still active — let the nav state machine continue
            # (DIRECT will re-check obstacles, AVOIDING will resume toward waypoint)
            self.get_logger().warn(
                f"Emergency released — clear at {min_depth:.2f}m. "
                f"Resuming {self.nav_state} toward goal."
            )
        else:
            # No goal was active — just hover
            self.nav_state = "IDLE"
            hover = PoseStamped()
            hover.header.stamp = self.get_clock().now().to_msg()
            hover.header.frame_id = "map"
            hover.pose.position.x = self.drone_x
            hover.pose.position.y = self.drone_y
            hover.pose.position.z = self.drone_z
            hover.pose.orientation.z = math.sin(self.drone_yaw / 2.0)
            hover.pose.orientation.w = math.cos(self.drone_yaw / 2.0)
            self.active_setpoint = hover
            self.get_logger().warn(f"Emergency released — clear at {min_depth:.2f}m. Hovering.")

    def _publish_zero_velocity(self):
        """
        Publish zero velocity via /mavros/setpoint_raw/local.
        Velocity-only PositionTarget — ignores position and acceleration.
        Stops the drone instantly without leaving GUIDED mode.
        """
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0
        msg.yaw = self.drone_yaw
        self.raw_setpoint_pub.publish(msg)

    # ================================================================
    # RViz Visualization
    # ================================================================

    def _publish_check_markers(self, obstacle_dist: Optional[float], fraction: float):
        """
        Publish markers:
        - Color-coded corridor (green/yellow/orange/red)
        - Red sphere at obstacle location
        - Drone position arrow
        - Status text with nav_state
        - Left/right check rays (green=clear, red=blocked, cyan=chosen)
        - Yellow sphere at avoidance waypoint
        - Yellow path line: drone → avoidance_waypoint → final_goal
        - Green sphere at final_goal
        """
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # ---- Corridor color based on speed fraction ----
        if fraction > 0.75:
            cor_r, cor_g, cor_b = 0.0, 1.0, 0.0
        elif fraction > 0.25:
            cor_r, cor_g, cor_b = 1.0, 1.0, 0.0
        elif fraction > 0.0:
            cor_r, cor_g, cor_b = 1.0, 0.5, 0.0
        else:
            cor_r, cor_g, cor_b = 1.0, 0.0, 0.0

        # Direction: toward current target
        viz_yaw = self.drone_yaw
        if self.nav_state == "AVOIDING" and self.avoidance_waypoint is not None:
            ax, ay = self.avoidance_waypoint
            viz_yaw = math.atan2(ay - self.drone_y, ax - self.drone_x)
        elif self.final_goal is not None:
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
        if self.nav_state == "AVOIDING":
            phase_label = "shift" if self._avoidance_phase == "SHIFT" else "fwd"
            text_marker.text = (
                f"AVOIDING {phase_label} [{self.avoidance_attempts}/{self.max_avoidance_attempts}] | "
                f"SPD: {fraction * 100:.0f}% | OBS: {dist_str}"
            )
        elif self.nav_state == "DIRECT":
            text_marker.text = f"DIRECT | SPD: {fraction * 100:.0f}% | OBS: {dist_str}"
        else:
            text_marker.text = f"{self.nav_state} | hovering"
        markers.markers.append(text_marker)

        # ---- Left/right check rays ----
        if self.last_lr_check is not None:
            lr = self.last_lr_check
            for side_yaw, side_dist, side_id in [
                (lr["left_yaw"], lr["left_dist"], 100),
                (lr["right_yaw"], lr["right_dist"], 101),
            ]:
                ray = Marker()
                ray.header.stamp = stamp
                ray.header.frame_id = "map"
                ray.ns = "lr_rays"
                ray.id = side_id
                ray.type = Marker.LINE_STRIP
                ray.action = Marker.ADD

                is_chosen = (
                    lr["chosen"] is not None
                    and abs(self._normalize_angle(side_yaw - lr["chosen"])) < 0.01
                )
                is_clear = side_dist >= self.min_gap_depth

                if is_chosen:
                    ray.scale.x = 0.05
                    ray.color.r = 0.0
                    ray.color.g = 1.0
                    ray.color.b = 1.0  # cyan
                    ray.color.a = 1.0
                elif is_clear:
                    ray.scale.x = 0.03
                    ray.color.r = 0.0
                    ray.color.g = 1.0
                    ray.color.b = 0.0  # green
                    ray.color.a = 0.5
                else:
                    ray.scale.x = 0.03
                    ray.color.r = 1.0
                    ray.color.g = 0.0
                    ray.color.b = 0.0  # red
                    ray.color.a = 0.5

                p_start = Point()
                p_start.x = self.drone_x
                p_start.y = self.drone_y
                p_start.z = self.drone_z
                ray.points.append(p_start)

                p_end = Point()
                p_end.x = self.drone_x + side_dist * math.cos(side_yaw)
                p_end.y = self.drone_y + side_dist * math.sin(side_yaw)
                p_end.z = self.drone_z
                ray.points.append(p_end)

                markers.markers.append(ray)
        else:
            # Delete left/right rays
            for side_id in (100, 101):
                ray = Marker()
                ray.header.stamp = stamp
                ray.header.frame_id = "map"
                ray.ns = "lr_rays"
                ray.id = side_id
                ray.action = Marker.DELETE
                markers.markers.append(ray)

        # ---- Avoidance waypoint sphere (yellow) ----
        if self.avoidance_waypoint is not None:
            wp_marker = Marker()
            wp_marker.header.stamp = stamp
            wp_marker.header.frame_id = "map"
            wp_marker.ns = "avoidance_wp"
            wp_marker.id = 50
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.scale.x = 0.3
            wp_marker.scale.y = 0.3
            wp_marker.scale.z = 0.3
            wp_marker.color.r = 1.0
            wp_marker.color.g = 1.0
            wp_marker.color.b = 0.0
            wp_marker.color.a = 0.8
            wp_marker.pose.position.x = self.avoidance_waypoint[0]
            wp_marker.pose.position.y = self.avoidance_waypoint[1]
            wp_marker.pose.position.z = self.drone_z
            wp_marker.pose.orientation.w = 1.0
            markers.markers.append(wp_marker)
        else:
            wp_marker = Marker()
            wp_marker.header.stamp = stamp
            wp_marker.header.frame_id = "map"
            wp_marker.ns = "avoidance_wp"
            wp_marker.id = 50
            wp_marker.action = Marker.DELETE
            markers.markers.append(wp_marker)

        # ---- Final goal sphere (green) ----
        if self.final_goal is not None:
            goal_marker = Marker()
            goal_marker.header.stamp = stamp
            goal_marker.header.frame_id = "map"
            goal_marker.ns = "final_goal"
            goal_marker.id = 51
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.scale.x = 0.3
            goal_marker.scale.y = 0.3
            goal_marker.scale.z = 0.3
            goal_marker.color.r = 0.0
            goal_marker.color.g = 1.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 0.8
            goal_marker.pose.position.x = self.final_goal[0]
            goal_marker.pose.position.y = self.final_goal[1]
            goal_marker.pose.position.z = self.final_goal[2]
            goal_marker.pose.orientation.w = 1.0
            markers.markers.append(goal_marker)
        else:
            goal_marker = Marker()
            goal_marker.header.stamp = stamp
            goal_marker.header.frame_id = "map"
            goal_marker.ns = "final_goal"
            goal_marker.id = 51
            goal_marker.action = Marker.DELETE
            markers.markers.append(goal_marker)

        # ---- Path line: drone → avoidance_waypoint → final_goal (yellow) ----
        if self.avoidance_waypoint is not None and self.final_goal is not None:
            path_marker = Marker()
            path_marker.header.stamp = stamp
            path_marker.header.frame_id = "map"
            path_marker.ns = "avoidance_path"
            path_marker.id = 52
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.05
            path_marker.color.r = 1.0
            path_marker.color.g = 1.0
            path_marker.color.b = 0.0
            path_marker.color.a = 0.8

            p_drone = Point()
            p_drone.x = self.drone_x
            p_drone.y = self.drone_y
            p_drone.z = self.drone_z
            path_marker.points.append(p_drone)

            p_wp = Point()
            p_wp.x = self.avoidance_waypoint[0]
            p_wp.y = self.avoidance_waypoint[1]
            p_wp.z = self.drone_z
            path_marker.points.append(p_wp)

            p_goal = Point()
            p_goal.x = self.final_goal[0]
            p_goal.y = self.final_goal[1]
            p_goal.z = self.final_goal[2]
            path_marker.points.append(p_goal)

            markers.markers.append(path_marker)
        else:
            path_marker = Marker()
            path_marker.header.stamp = stamp
            path_marker.header.frame_id = "map"
            path_marker.ns = "avoidance_path"
            path_marker.id = 52
            path_marker.action = Marker.DELETE
            markers.markers.append(path_marker)

        # ---- Emergency alert sphere (magenta) ----
        if self.emergency_active:
            em = Marker()
            em.header.stamp = stamp
            em.header.frame_id = "map"
            em.ns = "emergency_alert"
            em.id = 99
            em.type = Marker.SPHERE
            em.action = Marker.ADD
            em.scale.x = 1.5
            em.scale.y = 1.5
            em.scale.z = 1.5
            em.color.r = 1.0
            em.color.b = 1.0
            em.color.a = 0.4
            em.pose.position.x = self.drone_x
            em.pose.position.y = self.drone_y
            em.pose.position.z = self.drone_z
            em.pose.orientation.w = 1.0
            markers.markers.append(em)
        else:
            em = Marker()
            em.header.stamp = stamp
            em.header.frame_id = "map"
            em.ns = "emergency_alert"
            em.id = 99
            em.action = Marker.DELETE
            markers.markers.append(em)

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

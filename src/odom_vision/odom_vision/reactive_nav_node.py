#!/usr/bin/env python3
# Author: Gourav Khanduri
# Email:  gauravkhanduri93@gmail.com

import heapq
import math
import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ReactiveNavNode(Node):
    """
    Reactive navigation with DWA (Dynamic Window Approach) local planner.
    """

    def __init__(self):
        super().__init__("reactive_nav")

        # ---- Legacy Parameters (kept for speed fraction / zone logging) ----
        self.declare_parameter("cruise_speed", 0.5)
        self.declare_parameter("min_speed_fraction", 0.1)
        self.declare_parameter("check_width", 0.5)
        self.declare_parameter("check_step", 0.1)

        self.cruise_speed = self.get_parameter("cruise_speed").value
        self.min_speed_fraction = self.get_parameter("min_speed_fraction").value
        self.check_width = self.get_parameter("check_width").value
        self.check_step = self.get_parameter("check_step").value

        self.slow_distance = max(3.0, 2.0 * self.cruise_speed)
        self.stop_distance = max(1.5, 0.75 * self.cruise_speed)

        # ---- DWA Parameters ----
        # First-flight defaults — conservative. Override in launch file for faster flights.
        self.declare_parameter("max_linear_velocity", 0.8)
        self.declare_parameter("max_angular_velocity", 0.8)
        self.declare_parameter("max_linear_accel", 1.0)
        self.declare_parameter("max_angular_accel", 1.0)
        self.declare_parameter("min_linear_velocity", 0.0)

        self.declare_parameter("num_v_samples", 7)
        self.declare_parameter("num_w_samples", 11)
        self.declare_parameter("sim_time", 4.0)
        self.declare_parameter("sim_dt", 0.2)

        self.declare_parameter("heading_weight", 0.30)
        self.declare_parameter("clearance_weight", 0.3)
        self.declare_parameter("velocity_weight", 0.30)

        self.declare_parameter(
            "drone_radius", 0.50
        )  # meters, half the drone's width plus small margin
        self.declare_parameter("arrival_threshold", 0.5)
        self.declare_parameter("max_clearance", 3.0)
        self.declare_parameter("dwa_rate", 10.0)
        self.declare_parameter("occupied_threshold", 80.0)  # OctoMap cell value to treat as solid
        self.declare_parameter("inflation_radius", 0.8)  # meters — lethal + soft inflation extent
        self.declare_parameter("inflation_decay", 3.0)  # higher = steeper cost drop-off
        self.declare_parameter(
            "safety_margin", 0.3
        )  # meters added to drone_radius for lethal zone

        self.max_v = self.get_parameter("max_linear_velocity").value
        self.max_w = self.get_parameter("max_angular_velocity").value
        self.max_a = self.get_parameter("max_linear_accel").value
        self.max_w_a = self.get_parameter("max_angular_accel").value
        self.min_v = self.get_parameter("min_linear_velocity").value

        self.num_v_samples = self.get_parameter("num_v_samples").value
        self.num_w_samples = self.get_parameter("num_w_samples").value
        self.sim_time = self.get_parameter("sim_time").value
        self.sim_dt = self.get_parameter("sim_dt").value

        self.heading_weight = self.get_parameter("heading_weight").value
        self.clearance_weight = self.get_parameter("clearance_weight").value
        self.velocity_weight = self.get_parameter("velocity_weight").value

        self.drone_radius = self.get_parameter("drone_radius").value
        self.arrival_threshold = self.get_parameter("arrival_threshold").value
        self.max_clearance = self.get_parameter("max_clearance").value
        self.dwa_rate = self.get_parameter("dwa_rate").value
        self.occupied_threshold = self.get_parameter("occupied_threshold").value
        self.inflation_radius = self.get_parameter("inflation_radius").value
        self.inflation_decay = self.get_parameter("inflation_decay").value
        self.safety_margin = self.get_parameter("safety_margin").value

        # ---- Emergency (Layer 2) Parameters ----
        self.declare_parameter("max_plausible_speed", 15.0)
        self.declare_parameter("emergency_distance_base", 2.0)
        self.declare_parameter("emergency_speed_factor", 0.8)
        self.declare_parameter("emergency_recovery_delay", 1.0)
        self.declare_parameter("depth_roi_width_pct", 0.3)
        self.declare_parameter("depth_roi_height_pct", 0.5)
        self.declare_parameter("depth_min_valid", 0.3)
        self.declare_parameter("depth_max_valid", 20.0)

        self.max_plausible_speed = self.get_parameter("max_plausible_speed").value
        self.emergency_distance_base = self.get_parameter("emergency_distance_base").value
        self.emergency_speed_factor = self.get_parameter("emergency_speed_factor").value
        self.emergency_recovery_delay = self.get_parameter("emergency_recovery_delay").value
        self.depth_roi_width_pct = self.get_parameter("depth_roi_width_pct").value
        self.depth_roi_height_pct = self.get_parameter("depth_roi_height_pct").value
        self.depth_min_valid = self.get_parameter("depth_min_valid").value
        self.depth_max_valid = self.get_parameter("depth_max_valid").value

        # ---- A* Hybrid Parameters ----
        self.declare_parameter("astar_lookahead_dist", 2.0)  # carrot distance along path
        self.declare_parameter("astar_cost_weight", 5.0)  # penalty for high-cost cells
        self.declare_parameter("astar_replan_period", 3.0)  # periodic replan interval (s)
        self.declare_parameter("astar_stuck_replan", 2.0)  # stuck-time before replan (s)
        self.declare_parameter("astar_max_iterations", 20000)  # safety cap on node expansions
        self.lookahead_dist = self.get_parameter("astar_lookahead_dist").value
        self.astar_cost_weight = self.get_parameter("astar_cost_weight").value
        self.replan_period = self.get_parameter("astar_replan_period").value
        self.stuck_replan_threshold = self.get_parameter("astar_stuck_replan").value
        self.astar_max_iterations = self.get_parameter("astar_max_iterations").value

        # ---- State ----
        self.grid_array = None
        self.grid_info = None
        self.cost_grid = None  # inflated cost map (0.0 free .. 1.0 occupied/lethal)
        self.lethal_grid = None  # bool mask: True = drone footprint would collide
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_yaw = 0.0
        self._prev_drone_x = 0.0
        self._prev_drone_y = 0.0
        self._prev_pose_time = None
        self.drone_speed = 0.0
        self.fcu_mode = ""
        self.fcu_armed = False

        # ---- Navigation state ----
        self.final_goal: Optional[Tuple[float, float, float]] = None
        self.current_speed_fraction = 1.0
        self._last_zone = ""
        self.nav_state = "IDLE"
        self._goal_reached_timer = None

        # ---- DWA state ----
        self.current_v = 0.0
        self.current_w = 0.0
        self.best_trajectory = None
        self.all_trajectories = []

        # ---- A* Hybrid state ----
        self.global_path: List[Tuple[float, float]] = []  # world (x,y) waypoints
        self.local_goal: Optional[Tuple[float, float]] = None  # carrot fed to DWA scoring
        self._last_replan_time: float = 0.0
        self._stuck_start: Optional[float] = None

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
        self.raw_setpoint_pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10
        )
        self.emergency_status_pub = self.create_publisher(Bool, "/emergency_active", 10)
        self.min_depth_pub = self.create_publisher(Float32, "/min_depth", 10)
        self.dwa_markers_pub = self.create_publisher(MarkerArray, "/dwa_trajectories", 10)
        self.inflation_pub = self.create_publisher(OccupancyGrid, "/inflated_grid", 1)
        self.global_path_pub = self.create_publisher(Path, "/global_path", 1)

        # ---- Timers ----
        self.dwa_timer = self.create_timer(1.0 / self.dwa_rate, self._dwa_cycle)
        self.create_timer(0.2, self._publish_nav_state)
        self.create_timer(0.1, self._publish_dwa_markers)

        self.get_logger().info(
            f"DWA reactive nav started | max_v={self.max_v}m/s | "
            f"max_w={self.max_w}rad/s | "
            f"samples={self.num_v_samples}x{self.num_w_samples} | "
            f"sim_time={self.sim_time}s | "
            f"weights=H{self.heading_weight}/C{self.clearance_weight}/V{self.velocity_weight} | "
            f"emergency={self.emergency_distance_base}+{self.emergency_speed_factor}*speed"
        )

    def _grid_callback(self, msg: OccupancyGrid):
        self.grid_info = msg.info
        width = msg.info.width
        height = msg.info.height
        self.grid_array = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self._rebuild_cost_grid()

    def _rebuild_cost_grid(self):
        """
        Build an inflated cost grid from the raw occupancy grid.

        lethal_grid[y, x] = True  ↔ a drone of drone_radius centered here would collide.
        cost_grid[y, x]  ∈ [0, 1] — 1 at occupied cells, decays to 0 beyond inflation_radius.

        Uses a distance transform (O(N) for the whole grid) so the per-cycle
        DWA checks become single-cell lookups instead of nested footprint loops.
        """
        if self.grid_array is None or self.grid_info is None:
            return

        resolution = self.grid_info.resolution

        # Occupied cells only (treat unknown=-1 as free for planning purposes)
        occupied_mask = (self.grid_array >= self.occupied_threshold) & (self.grid_array > 0)

        # Distance (in cells) from each cell to the nearest occupied cell
        if occupied_mask.any():
            dist_cells = distance_transform_edt(~occupied_mask)
        else:
            dist_cells = np.full_like(self.grid_array, 1e6, dtype=np.float32)

        dist_m = dist_cells * resolution

        # Lethal: drone footprint (plus safety margin) would overlap an obstacle
        lethal_dist = self.drone_radius + self.safety_margin
        self.lethal_grid = dist_m < lethal_dist

        # Soft inflation: exponential decay from lethal boundary out to inflation_radius
        # cost = 1 inside lethal, decays to 0 at inflation_radius, clipped 0 beyond
        extra_dist = np.clip(dist_m - lethal_dist, 0.0, None)
        decay = np.exp(-self.inflation_decay * extra_dist)
        decay[dist_m >= self.inflation_radius] = 0.0
        cost = np.where(self.lethal_grid, 1.0, decay).astype(np.float32)
        self.cost_grid = cost
        self._publish_inflated_grid()

    def _clear_drone_footprint(self):
        """
        Clear the lethal classification in a small radius around the drone.
        Prevents deadlock when inflation (ground plane etc.) overlaps the
        drone's current position.
        """
        if self.lethal_grid is None or self.grid_info is None:
            return
        resolution = self.grid_info.resolution
        origin_x = self.grid_info.origin.position.x
        origin_y = self.grid_info.origin.position.y
        height, width = self.lethal_grid.shape

        gx = int((self.drone_x - origin_x) / resolution)
        gy = int((self.drone_y - origin_y) / resolution)
        if not (0 <= gx < width and 0 <= gy < height):
            return

        # Clear only the drone's physical footprint — NOT the safety halo.
        # Must equal skip_radius in _is_trajectory_safe: the gap between
        # them becomes a zone that is cleared here but then checked there,
        # which silently passes every trajectory through the inflation.
        clear_dist = self.drone_radius
        clear_r = max(1, int(math.ceil(clear_dist / resolution)))
        for dy in range(-clear_r, clear_r + 1):
            for dx in range(-clear_r, clear_r + 1):
                if dx * dx + dy * dy > clear_r * clear_r:
                    continue
                cx, cy = gx + dx, gy + dy
                if 0 <= cx < width and 0 <= cy < height:
                    # Clear derived grids only — leave raw grid_array intact
                    self.lethal_grid[cy, cx] = False
                    if self.cost_grid is not None:
                        self.cost_grid[cy, cx] = 0.0

    def _publish_inflated_grid(self):
        """Publish the inflated cost grid as an OccupancyGrid for RViz."""
        if self.cost_grid is None or self.grid_info is None:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info = self.grid_info
        # 0..100 scale, lethal=100, free=0, gradient in between
        # Clamp to [0, 1] before scaling — int8 overflows above 127
        data = (np.clip(self.cost_grid, 0.0, 1.0) * 100.0).astype(np.int8).flatten()
        msg.data = data.tolist()
        self.inflation_pub.publish(msg)

    def _pose_callback(self, msg: PoseStamped):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        self.drone_z = msg.pose.position.z
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.drone_yaw = math.atan2(siny_cosp, cosy_cosp)

        now = self.get_clock().now()
        if self._prev_pose_time is not None:
            dt = (now - self._prev_pose_time).nanoseconds / 1e9
            if dt > 0.001:
                dx = self.drone_x - self._prev_drone_x
                dy = self.drone_y - self._prev_drone_y
                raw_speed = math.sqrt(dx * dx + dy * dy) / dt
                raw_speed = min(raw_speed, self.max_plausible_speed)
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
        self.nav_state = "IDLE"
        self.current_speed_fraction = 1.0
        self.current_v = 0.0
        self.current_w = 0.0
        self._publish_zero_velocity()

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

        # Body frame to map frame
        goal_x = self.drone_x + msg.x * math.cos(self.drone_yaw) - msg.y * math.sin(self.drone_yaw)
        goal_y = self.drone_y + msg.x * math.sin(self.drone_yaw) + msg.y * math.cos(self.drone_yaw)
        goal_z = self.drone_z + msg.z

        self.final_goal = (goal_x, goal_y, goal_z)
        self.nav_state = "NAVIGATING"
        self.get_logger().info(f"New goal: ({goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f})")
        self._trigger_replan(reason="new goal")

    def _publish_nav_state(self):
        msg = String()
        msg.data = self.nav_state
        self.nav_state_pub.publish(msg)

    # ================================================================
    # Speed Fraction (kept for status publishing)
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
    # DWA Local Planner
    # ================================================================

    def _compute_dynamic_window(self):
        """
        Velocity window for DWA target sampling — full capability range.

        We do NOT apply a physics brake cap here, because that would shorten
        the simulated trajectories below obstacle distance, hiding collisions
        from the safety check and letting the straight-ahead candidate always
        score best. Brake behaviour is enforced later via the executed best_v,
        not by restricting the search space.
        """
        dt = 1.0 / self.dwa_rate
        w_lo = max(-self.max_w, self.current_w - self.max_w_a * dt)
        w_hi = min(self.max_w, self.current_w + self.max_w_a * dt)
        return self.min_v, self.max_v, w_lo, w_hi

    def _simulate_trajectory(self, v_target, w_target):
        """
        Predict drone trajectory assuming it accelerates from current
        velocity toward (v_target, w_target) at the configured accel limits.
        This produces longer, more realistic trajectories than constant-vel.
        """
        trajectory = []
        x = self.drone_x
        y = self.drone_y
        yaw = self.drone_yaw
        v = self.current_v
        w = self.current_w

        latency = 0.15
        x += v * math.cos(yaw) * latency
        y += v * math.sin(yaw) * latency
        yaw = math.atan2(math.sin(yaw + w * latency), math.cos(yaw + w * latency))

        t = 0.0
        while t < self.sim_time:
            # Accelerate toward target velocity at accel limits
            dv = v_target - v
            dw = w_target - w
            max_dv = self.max_a * self.sim_dt
            max_dw = self.max_w_a * self.sim_dt
            v += max(-max_dv, min(max_dv, dv))
            w += max(-max_dw, min(max_dw, dw))

            x += v * math.cos(yaw) * self.sim_dt
            y += v * math.sin(yaw) * self.sim_dt
            yaw += w * self.sim_dt
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            trajectory.append((x, y, yaw))
            t += self.sim_dt

        return trajectory

    def _is_trajectory_safe(self, trajectory):
        """
        Check every grid cell that the trajectory crosses, not just sample
        points. Otherwise, at velocities where trajectory points are spaced
        further apart than grid cells, the trajectory can straddle a single
        occupied cell without hitting a sample point.
        """
        if self.lethal_grid is None or self.grid_info is None:
            return False

        resolution = self.grid_info.resolution
        origin_x = self.grid_info.origin.position.x
        origin_y = self.grid_info.origin.position.y
        height, width = self.lethal_grid.shape
        inv_res = 1.0 / resolution

        # Skip just the drone's physical footprint to avoid phantom lethal at
        # the current cell. The lethal_grid already inflates obstacles by
        # drone_radius + safety_margin, so no extra padding is needed here.
        skip_radius = self.drone_radius
        skip_r2 = skip_radius * skip_radius

        prev_x = self.drone_x
        prev_y = self.drone_y

        for x, y, _ in trajectory:
            dxd = x - self.drone_x
            dyd = y - self.drone_y
            if dxd * dxd + dyd * dyd < skip_r2:
                prev_x, prev_y = x, y
                continue

            # Walk from prev_x,prev_y → x,y in resolution/2 sized steps so
            # we hit every cell the segment crosses.
            seg_dx = x - prev_x
            seg_dy = y - prev_y
            seg_len = math.sqrt(seg_dx * seg_dx + seg_dy * seg_dy)
            n_steps = max(1, int(seg_len / (resolution * 0.5)) + 1)
            for i in range(1, n_steps + 1):
                t = i / n_steps
                px = prev_x + seg_dx * t
                py = prev_y + seg_dy * t
                gx = int((px - origin_x) * inv_res)
                gy = int((py - origin_y) * inv_res)
                if gx < 0 or gy < 0 or gx >= width or gy >= height:
                    continue  # outside map = unknown = treat as safe
                if self.lethal_grid[gy, gx]:
                    return False

            prev_x, prev_y = x, y
        return True

    def _point_cost(self, x, y):
        """Lookup cost in the inflated grid. 0 = free, 1 = lethal/occupied."""
        if self.cost_grid is None or self.grid_info is None:
            return 0.0

        resolution = self.grid_info.resolution
        origin_x = self.grid_info.origin.position.x
        origin_y = self.grid_info.origin.position.y
        height, width = self.cost_grid.shape

        gx = int((x - origin_x) / resolution)
        gy = int((y - origin_y) / resolution)
        if gx < 0 or gy < 0 or gx >= width or gy >= height:
            return 0.0  # outside grid = unknown = assume free
        return float(self.cost_grid[gy, gx])

    def _score_trajectory(self, trajectory, v):
        """Score a trajectory: progress + alignment + clearance + velocity."""
        if len(trajectory) == 0:
            return -float("inf")

        end_x, end_y, end_yaw = trajectory[-1]

        # 1. Goal-progress: how much closer to goal does end position get?
        # Normalise by the GLOBAL maximum (max_v * sim_time), not per-trajectory
        # — otherwise a slow/short trajectory that creeps 0.5m scores the same
        # as a fast trajectory covering its full reach, and DWA never commits
        # to longer curved paths around obstacles.
        # Use A* carrot as the scoring target when a global path is available,
        # so DWA tracks the planned route around large obstacles instead of
        # driving straight at the raw goal.
        target = (
            self.local_goal
            if self.local_goal is not None
            else (self.final_goal[0], self.final_goal[1])
        )
        start_dist = math.hypot(target[0] - self.drone_x, target[1] - self.drone_y)
        end_dist = math.hypot(target[0] - end_x, target[1] - end_y)
        progress = max(0.0, start_dist - end_dist)
        global_max_progress = max(1e-3, self.max_v * self.sim_time)
        progress_score = min(1.0, progress / global_max_progress)

        # 2. Alignment: end yaw vs bearing from end position to carrot/goal.
        # Rewards trajectories that leave the drone facing the target after
        # going around an obstacle, discouraging snap-back oscillation.
        goal_dx = target[0] - end_x
        goal_dy = target[1] - end_y
        goal_bearing = math.atan2(goal_dy, goal_dx)
        angle_err = abs(
            math.atan2(math.sin(goal_bearing - end_yaw), math.cos(goal_bearing - end_yaw))
        )
        alignment_score = (math.pi - angle_err) / math.pi

        # 3. Clearance: mean cost along the trajectory (not just max).
        # Gives a smooth gradient — trajectories skirting near the inflation
        # halo are penalised even if they don't cross the lethal boundary.
        total_cost = 0.0
        for x, y, _ in trajectory:
            total_cost += self._point_cost(x, y)
        mean_cost = total_cost / len(trajectory)
        clearance_score = 1.0 - mean_cost

        # 4. Velocity: prefer faster forward motion
        velocity_score = v / self.max_v if self.max_v > 0 else 0.0

        # Velocity-dependent weights: as the drone slows (near obstacle),
        # clearance dominates so the planner actively turns to find an opening.
        # At full speed in open space, progress dominates for efficient travel.
        v_ratio = v / self.max_v if self.max_v > 0 else 0.0  # 0=stopped, 1=full speed
        w_progress = 0.10 + 0.25 * v_ratio  # 0.10 near obstacle → 0.35 open space
        w_alignment = 0.10 + 0.10 * v_ratio  # 0.10 → 0.20
        w_clearance = 0.75 - 0.35 * v_ratio  # 0.75 near obstacle → 0.40 open space
        w_velocity = 0.05

        return (
            w_progress * progress_score
            + w_alignment * alignment_score
            + w_clearance * clearance_score
            + w_velocity * velocity_score
        )

    # ================================================================
    # A* Hybrid: global planner + carrot follower
    # ================================================================

    def _plan_astar(
        self, start_xy: Tuple[float, float], goal_xy: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        """
        8-connected A* over the lethal_grid, with cost_grid used as a soft
        penalty so paths prefer clearance. Returns a list of world (x,y)
        waypoints from start to goal, or [] if no path exists.
        """
        if self.lethal_grid is None or self.cost_grid is None or self.grid_info is None:
            return []

        resolution = self.grid_info.resolution
        origin_x = self.grid_info.origin.position.x
        origin_y = self.grid_info.origin.position.y
        height, width = self.lethal_grid.shape

        def w2g(x, y):
            return int((x - origin_x) / resolution), int((y - origin_y) / resolution)

        def g2w(gx, gy):
            return origin_x + (gx + 0.5) * resolution, origin_y + (gy + 0.5) * resolution

        sx, sy = w2g(*start_xy)
        gx, gy = w2g(*goal_xy)

        if not (0 <= sx < width and 0 <= sy < height):
            return []
        if not (0 <= gx < width and 0 <= gy < height):
            return []

        # If start is lethal (drone footprint overlap), clear just start cell
        # — matches _clear_drone_footprint behaviour.
        lethal = self.lethal_grid.copy()
        lethal[sy, sx] = False

        if lethal[gy, gx]:
            # Goal cell is blocked; snap goal to nearest free cell within 1m.
            snap_r = max(1, int(1.0 / resolution))
            best = None
            best_d = float("inf")
            for dy in range(-snap_r, snap_r + 1):
                for dx in range(-snap_r, snap_r + 1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < width and 0 <= ny < height and not lethal[ny, nx]:
                        d = dx * dx + dy * dy
                        if d < best_d:
                            best_d = d
                            best = (nx, ny)
            if best is None:
                return []
            gx, gy = best

        NEIGHBORS = [
            (-1, -1, math.sqrt(2)),
            (0, -1, 1.0),
            (1, -1, math.sqrt(2)),
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (-1, 1, math.sqrt(2)),
            (0, 1, 1.0),
            (1, 1, math.sqrt(2)),
        ]

        open_heap = [(0.0, (sx, sy))]
        came_from = {}
        g_score = {(sx, sy): 0.0}
        iterations = 0

        while open_heap and iterations < self.astar_max_iterations:
            iterations += 1
            _, current = heapq.heappop(open_heap)
            if current == (gx, gy):
                break
            cx, cy = current
            for dx, dy, step_cost in NEIGHBORS:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if lethal[ny, nx]:
                    continue
                tentative = (
                    g_score[current]
                    + step_cost
                    + self.astar_cost_weight * float(self.cost_grid[ny, nx]) * step_cost
                )
                if tentative < g_score.get((nx, ny), float("inf")):
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative
                    h = math.hypot(gx - nx, gy - ny)
                    heapq.heappush(open_heap, (tentative + h, (nx, ny)))

        if (gx, gy) not in came_from and (sx, sy) != (gx, gy):
            self.get_logger().warn(
                f"A*: no path start=({sx},{sy}) goal=({gx},{gy}) " f"iterations={iterations}"
            )
            return []

        # Reconstruct grid path
        path_cells = [(gx, gy)]
        while path_cells[-1] != (sx, sy):
            prev = came_from.get(path_cells[-1])
            if prev is None:
                break
            path_cells.append(prev)
        path_cells.reverse()

        # Convert to world coords and prepend exact start / append exact goal
        path_world = [start_xy] + [g2w(cx, cy) for cx, cy in path_cells] + [goal_xy]
        return path_world

    def _get_local_goal(self) -> Optional[Tuple[float, float]]:
        """Return a carrot point `lookahead_dist` ahead of the drone on the path."""
        if not self.global_path:
            return None

        # Closest path index to current drone position
        closest_idx = 0
        closest_d = float("inf")
        for i, (px, py) in enumerate(self.global_path):
            d = math.hypot(px - self.drone_x, py - self.drone_y)
            if d < closest_d:
                closest_d = d
                closest_idx = i

        # Walk forward along the path accumulating arc length
        accumulated = 0.0
        for i in range(closest_idx, len(self.global_path) - 1):
            x0, y0 = self.global_path[i]
            x1, y1 = self.global_path[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if accumulated + seg >= self.lookahead_dist:
                t = (self.lookahead_dist - accumulated) / max(seg, 1e-6)
                return (x0 + t * (x1 - x0), y0 + t * (y1 - y0))
            accumulated += seg

        # Path shorter than lookahead — aim at final waypoint
        return self.global_path[-1]

    def _trigger_replan(self, reason: str = ""):
        """Run A* from current drone position to final_goal, publish, timestamp."""
        if self.final_goal is None:
            return
        path = self._plan_astar(
            (self.drone_x, self.drone_y),
            (self.final_goal[0], self.final_goal[1]),
        )
        self.global_path = path
        self._last_replan_time = time.time()
        self._stuck_start = None
        if path:
            self.get_logger().info(
                f"A* replan ({reason}): {len(path)} waypoints, "
                f"length={self._path_length(path):.1f}m"
            )
        else:
            self.get_logger().warn(f"A* replan ({reason}): FAILED (no path)")
        self._publish_global_path()

    def _path_length(self, path: List[Tuple[float, float]]) -> float:
        total = 0.0
        for i in range(len(path) - 1):
            total += math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        return total

    def _publish_global_path(self):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.global_path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = self.final_goal[2] if self.final_goal else 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.global_path_pub.publish(msg)

    def _dwa_cycle(self):
        """Main DWA loop, runs at dwa_rate Hz."""
        if not self.fcu_armed or self.fcu_mode != "GUIDED":
            return

        # Layer 2: if emergency active, publish zero velocity
        if self.emergency_active:
            self._publish_zero_velocity()
            return

        if self.nav_state == "IDLE" or self.final_goal is None:
            self._publish_cmd_zero()
            self._publish_status(None, 0.0)
            return

        # Keep drone's current footprint clear of inflation bleed-over
        self._clear_drone_footprint()

        # ---- A* Hybrid: replan triggers ----
        now = time.time()
        need_replan = False
        if not self.global_path:
            need_replan = True
        elif now - self._last_replan_time > self.replan_period:
            need_replan = True
        if need_replan:
            self._trigger_replan(reason="periodic" if self.global_path else "no path")

        # Carrot for DWA scoring — falls back to final_goal if path empty
        self.local_goal = self._get_local_goal()

        # Check goal reached
        dist_to_goal = math.sqrt(
            (self.drone_x - self.final_goal[0]) ** 2 + (self.drone_y - self.final_goal[1]) ** 2
        )
        if dist_to_goal < self.arrival_threshold:
            self._enter_goal_reached()
            self.get_logger().info(f"Goal reached! Distance: {dist_to_goal:.2f}m")
            return

        # Obstacle check along drone's current heading (what the camera sees).
        # Non-holonomic: the drone must yaw to face the goal, so the relevant
        # forward clearance is along drone_yaw, not the goal bearing.
        obstacle_dist = self._check_direction(
            self.drone_yaw, min(dist_to_goal, self.slow_distance + 1.0)
        )
        fraction = self._compute_speed_fraction(obstacle_dist)
        self.current_speed_fraction = fraction
        self._publish_status(obstacle_dist, fraction)

        # Compute dynamic window
        v_min, v_max, w_min, w_max = self._compute_dynamic_window()

        if w_max <= w_min:
            w_samples = np.array([0.0])
        else:
            w_samples = np.linspace(w_min, w_max, self.num_w_samples)

        # When braked to a stop by physics cap, inject turn-in-place samples
        # so DWA can rotate to find an opening rather than freezing.
        if v_max < 0.05:
            v_samples = np.array([0.0])
        else:
            min_useful_v = max(v_min, 0.05)
            v_samples = np.linspace(min_useful_v, v_max, self.num_v_samples)

        # Evaluate all (v, w) combinations
        best_score = -float("inf")
        best_v, best_w = 0.0, 0.0
        best_traj = None
        self.all_trajectories = []
        safe_count = 0

        for v in v_samples:
            for w in w_samples:
                traj = self._simulate_trajectory(v, w)

                if not self._is_trajectory_safe(traj):
                    self.all_trajectories.append((traj, False, -float("inf")))
                    continue

                safe_count += 1
                score = self._score_trajectory(traj, v)
                self.all_trajectories.append((traj, True, score))

                if score > best_score:
                    best_score = score
                    best_v = v
                    best_w = w
                    best_traj = traj

        # Execute best trajectory or stop
        if best_traj is None:
            self._log_dwa_debug()
            self._publish_cmd_zero()
            self.current_v = 0.0
            self.current_w = 0.0
            # Track how long DWA has been stuck — trigger A* replan if prolonged
            if self._stuck_start is None:
                self._stuck_start = time.time()
            elif time.time() - self._stuck_start > self.stuck_replan_threshold:
                self._trigger_replan(reason="stuck")
                self._stuck_start = time.time()  # debounce
        else:
            # Execution-time brake cap: bound forward speed by braking distance
            # along the drone's heading.
            forward_dist = self._check_direction(self.drone_yaw, 8.0)
            v_cap = self.max_v
            if forward_dist is not None and forward_dist > self.drone_radius:
                usable_dist = forward_dist - self.drone_radius - self.safety_margin
                if usable_dist > 0.0:
                    v_cap = min(self.max_v, math.sqrt(2.0 * self.max_a * usable_dist))
                else:
                    v_cap = 0.0
            exec_v = min(best_v, v_cap)

            # If brake has fully zeroed forward motion AND DWA didn't pick a
            # meaningful yaw, override into explicit turn-in-place: scan yaw
            # rates, pick the one whose simulated ROTATED heading yields the
            # most forward clearance.
            if v_cap < 0.05 and abs(best_w) < 0.05:
                # Full 180° sweep around the drone's heading, not constrained
                # by one tick of max_w rotation. We're looking for the widest
                # opening anywhere — the rate we command is just whatever
                # rotates us toward that bearing fastest.
                best_rot_bearing = self.drone_yaw
                best_rot_clearance = -1.0
                max_raycast = 6.0
                for bearing in np.linspace(self.drone_yaw - math.pi, self.drone_yaw + math.pi, 24):
                    clearance = self._check_direction(bearing, max_raycast)
                    c_val = clearance if clearance is not None else max_raycast
                    if c_val > best_rot_clearance:
                        best_rot_clearance = c_val
                        best_rot_bearing = bearing

                # Rotate toward the best bearing at max yaw rate.
                delta = math.atan2(
                    math.sin(best_rot_bearing - self.drone_yaw),
                    math.cos(best_rot_bearing - self.drone_yaw),
                )
                best_w = self.max_w * (1.0 if delta > 0 else -1.0)
                exec_v = 0.0
                self.get_logger().warn(
                    f"DWA: TURN-IN-PLACE — forward blocked. "
                    f"target_bearing={math.degrees(best_rot_bearing):+.0f}° "
                    f"delta={math.degrees(delta):+.0f}° "
                    f"clearance={best_rot_clearance:.2f}m w={best_w:+.2f}",
                    throttle_duration_sec=1.0,
                )

            self.best_trajectory = best_traj
            self._stuck_start = None
            # Low-pass smooth yaw rate to prevent fast fluctuation on the FCU.
            # Skip smoothing during TURN-IN-PLACE (exec_v==0 override) — want full rate.
            if exec_v > 0.0:
                alpha = 0.4
                cmd_w = alpha * best_w + (1.0 - alpha) * self.current_w
            else:
                cmd_w = best_w
            self._publish_velocity(exec_v, cmd_w)
            self.current_v = exec_v
            self.current_w = cmd_w
            brake_note = f" BRAKE={v_cap:.2f}" if v_cap < self.max_v - 0.05 else ""
            self.get_logger().info(
                f"DWA: sim_v={best_v:.2f} exec_v={exec_v:.2f} w={best_w:.2f} "
                f"score={best_score:.3f} safe={safe_count}/{len(self.all_trajectories)}{brake_note}",
                throttle_duration_sec=2.0,
            )

        self._publish_check_markers(obstacle_dist, fraction)

    def _log_dwa_debug(self):
        """Log diagnostic info when all trajectories fail."""
        if self.grid_array is None or self.grid_info is None:
            self.get_logger().warn("DWA: no safe trajectory — grid is None!")
            return

        resolution = self.grid_info.resolution
        origin_x = self.grid_info.origin.position.x
        origin_y = self.grid_info.origin.position.y
        height, width = self.grid_array.shape

        gx = int((self.drone_x - origin_x) / resolution)
        gy = int((self.drone_y - origin_y) / resolution)

        # Check if drone position is even inside the grid
        in_grid = 0 <= gx < width and 0 <= gy < height

        # Sample cell values in an 11x11 area around the drone (covers lethal_dist)
        nearby_vals = []
        for dy in range(-5, 6):
            for dx in range(-5, 6):
                cx, cy = gx + dx, gy + dy
                if 0 <= cx < width and 0 <= cy < height:
                    nearby_vals.append(int(self.grid_array[cy, cx]))

        # Count unique values
        unique, counts = np.unique(nearby_vals, return_counts=True)
        val_summary = ", ".join(f"{v}:{c}" for v, c in zip(unique, counts))

        v_min, v_max, w_min, w_max = self._compute_dynamic_window()

        # Lethal cell count near drone
        lethal_here = False
        lethal_count = 0
        if self.lethal_grid is not None and in_grid:
            lethal_here = bool(self.lethal_grid[gy, gx])
            lethal_count = int(self.lethal_grid.sum())

        self.get_logger().warn(
            f"DWA: no safe trajectory! "
            f"drone=({self.drone_x:.2f},{self.drone_y:.2f}) "
            f"grid_cell=({gx},{gy}) in_grid={in_grid} "
            f"grid_size={width}x{height} res={resolution:.2f} "
            f"nearby_raw=[{val_summary}] "
            f"lethal_here={lethal_here} lethal_total={lethal_count} "
            f"dw=[v:{v_min:.2f}..{v_max:.2f} w:{w_min:.2f}..{w_max:.2f}] "
            f"inflation_radius={self.inflation_radius}"
        )

    # ================================================================
    # Velocity Publishing
    # ================================================================

    def _publish_velocity(self, v, w):
        """Publish non-holonomic velocity: forward in drone's heading, with yaw rate.

        The drone's camera stays pointed in the direction of motion because we
        command a yaw rate that matches how DWA simulates the trajectory.
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
            | PositionTarget.IGNORE_YAW
        )
        msg.velocity.x = v * math.cos(self.drone_yaw)
        msg.velocity.y = v * math.sin(self.drone_yaw)
        msg.velocity.z = 0.0
        msg.yaw_rate = w
        self.raw_setpoint_pub.publish(msg)

    def _publish_cmd_zero(self):
        """Publish zero velocity via raw setpoint to hold position."""
        self._publish_zero_velocity()

    def _publish_zero_velocity(self):
        """
        Publish zero velocity via /mavros/setpoint_raw/local.
        Velocity-only PositionTarget with zero yaw rate — stops the drone instantly.
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
            | PositionTarget.IGNORE_YAW
        )
        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0
        msg.yaw_rate = 0.0
        self.raw_setpoint_pub.publish(msg)

    # ================================================================
    # Status Publishing
    # ================================================================

    def _publish_status(self, obstacle_dist, fraction):
        """Publish obstacle status, speed fraction, and obstacle distance."""
        zone = self._get_zone_label(fraction)

        if zone != self._last_zone and self.final_goal is not None:
            dist_str = f"{obstacle_dist:.2f}m" if obstacle_dist is not None else "none"
            self.get_logger().info(
                f"Zone: {zone} | speed={fraction * 100:.0f}% | obstacle={dist_str}"
            )
            self._last_zone = zone

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

    # ================================================================
    # Obstacle Checking (kept for status + DWA sanity)
    # ================================================================

    def _check_direction(self, direction_yaw: float, max_distance: float) -> Optional[float]:
        """
        Cast rays in a direction and return distance to nearest obstacle.
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
                    if cell >= self.occupied_threshold:
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
        self.current_speed_fraction = 1.0
        self.current_v = 0.0
        self.current_w = 0.0
        self._publish_cmd_zero()
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
        return math.atan2(math.sin(angle), math.cos(angle))

    # ================================================================
    # Layer 2: Raw Depth Emergency Stop
    # ================================================================

    def _depth_callback(self, msg: Image):
        """
        Layer 2: Process raw depth image for emergency obstacle detection.
        """
        if self.fcu_mode != "GUIDED" or not self.fcu_armed:
            return

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

        h, w = depth.shape
        roi_y_start = int(h * (0.5 - self.depth_roi_height_pct / 2))
        roi_y_end = int(h * (0.5 + self.depth_roi_height_pct / 2))
        roi_x_start = int(w * (0.5 - self.depth_roi_width_pct / 2))
        roi_x_end = int(w * (0.5 + self.depth_roi_width_pct / 2))
        roi = depth[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

        valid_mask = (
            np.isfinite(roi) & (roi >= self.depth_min_valid) & (roi <= self.depth_max_valid)
        )
        valid_depths = roi[valid_mask]

        if len(valid_depths) == 0:
            return

        min_depth = float(np.min(valid_depths))
        self.last_min_depth = min_depth

        depth_msg = Float32()
        depth_msg.data = min_depth
        self.min_depth_pub.publish(depth_msg)

        emergency_dist = (
            self.emergency_distance_base + self.emergency_speed_factor * self.drone_speed
        )

        if not self.emergency_active:
            if (
                self.drone_speed > 0.2
                and min_depth < emergency_dist
                and self.nav_state == "NAVIGATING"
            ):
                self._trigger_emergency(min_depth, emergency_dist)
        else:
            self._publish_zero_velocity()
            if self.emergency_start_time is None:
                self.emergency_start_time = self.get_clock().now()
            elapsed = (self.get_clock().now() - self.emergency_start_time).nanoseconds / 1e9
            if elapsed > 10.0:
                self.get_logger().error("Emergency timeout — force releasing after 10s")
                self._release_emergency(min_depth)
            elif elapsed > self.emergency_recovery_delay and self.drone_speed < 0.3:
                self._release_emergency(min_depth)

    def _trigger_emergency(self, min_depth: float, trigger_dist: float):
        """Activate emergency stop."""
        self.emergency_active = True
        self.emergency_start_time = self.get_clock().now()
        self.current_v = 0.0
        self.current_w = 0.0

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
        """Release emergency stop. Resume previous navigation state."""
        self.emergency_active = False
        self.emergency_start_time = None

        bool_msg = Bool()
        bool_msg.data = False
        self.emergency_status_pub.publish(bool_msg)

        if self.final_goal is not None:
            self.get_logger().warn(
                f"Emergency released — clear at {min_depth:.2f}m. "
                f"Resuming {self.nav_state} toward goal."
            )
        else:
            self.nav_state = "IDLE"
            self.get_logger().warn(f"Emergency released — clear at {min_depth:.2f}m. Hovering.")

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
        - Green sphere at final_goal
        - Emergency alert sphere (magenta)
        """
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        if fraction > 0.75:
            cor_r, cor_g, cor_b = 0.0, 1.0, 0.0
        elif fraction > 0.25:
            cor_r, cor_g, cor_b = 1.0, 1.0, 0.0
        elif fraction > 0.0:
            cor_r, cor_g, cor_b = 1.0, 0.5, 0.0
        else:
            cor_r, cor_g, cor_b = 1.0, 0.0, 0.0

        # Corridor follows the drone's heading (where the camera is looking),
        # so it always matches what _check_direction scans.
        viz_yaw = self.drone_yaw

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

        # Obstacle sphere
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

        # Drone position arrow
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

        # Status text above drone
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
        if self.nav_state == "NAVIGATING":
            yaw_deg = math.degrees(self.current_w)
            text_marker.text = (
                f"DWA | v={self.current_v:.2f}m/s yaw_rate={yaw_deg:+.0f}°/s | " f"OBS: {dist_str}"
            )
        else:
            text_marker.text = f"{self.nav_state} | hovering"
        markers.markers.append(text_marker)

        # Delete old avoidance markers (cleanup from old version)
        for ns, mid in [
            ("lr_rays", 100),
            ("lr_rays", 101),
            ("avoidance_wp", 50),
            ("avoidance_path", 52),
        ]:
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = "map"
            m.ns = ns
            m.id = mid
            m.action = Marker.DELETE
            markers.markers.append(m)

        # Final goal sphere (green)
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

        # Emergency alert sphere (magenta)
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

    def _publish_dwa_markers(self):
        """Publish DWA candidate trajectories for RViz visualization."""
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, (traj, safe, score) in enumerate(self.all_trajectories):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "dwa_candidates"
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.05

            if safe:
                m.color.r = 0.2
                m.color.g = 0.8
                m.color.b = 0.2
                m.color.a = 0.6
            else:
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 0.5

            for x, y, _ in traj:
                p = Point()
                p.x = x
                p.y = y
                p.z = self.drone_z
                m.points.append(p)

            markers.markers.append(m)

        # Delete stale markers from previous cycle (fixed range)
        MAX_DWA_MARKER_ID = 150
        for j in range(len(self.all_trajectories), MAX_DWA_MARKER_ID):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "dwa_candidates"
            m.id = j
            m.action = Marker.DELETE
            markers.markers.append(m)

        # Best trajectory in green, thicker
        if self.best_trajectory is not None:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "dwa_best"
            m.id = 9999
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.08
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            for x, y, _ in self.best_trajectory:
                p = Point()
                p.x = x
                p.y = y
                p.z = self.drone_z
                m.points.append(p)
            markers.markers.append(m)
        else:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "dwa_best"
            m.id = 9999
            m.action = Marker.DELETE
            markers.markers.append(m)

        self.dwa_markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNavNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted — publishing zero velocity and shutting down.")
    finally:
        try:
            node._publish_zero_velocity()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

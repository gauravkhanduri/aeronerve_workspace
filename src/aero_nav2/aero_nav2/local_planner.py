#!/usr/bin/env python3
"""
local_planner.py — Custom Nav Stack
=====================================
Holonomic Dynamic Window Approach (DWA) local planner for a drone.

Unlike DWB (which is designed for differential-drive robots and always
produces angular.z=0 for lateral motion), this node samples (vx, vy) directly
in the robot body frame — the drone can move sideways without rotating.

Data flow
---------
/global_path    (Path)          — from global_planner.py
/odom           (Odometry)      — drone position + yaw
/depth_pointcloud (PointCloud2) — real-time obstacle proximity check
  →  find lookahead waypoint on global path
  →  compute desired (vx_body, vy_body) toward waypoint
  →  DWA: score N candidate velocities by heading + obstacle clearance
  →  publish best /cmd_vel (Twist, body frame)
  →  publish /goal_reached (Bool) when within goal_tolerance

DWA Sampling
------------
  Candidates: vx ∈ [0, max_speed], vy ∈ [-max_speed, max_speed]
  Grid: (2*v_samples+1) × v_samples combinations
  Simulation horizon: sim_time seconds at each candidate velocity
  Obstacle check: nearest point in depth cloud (body frame) along trajectory

Scoring (all weights normalised to [0,1])
-----------------------------------------
  heading   : cos(angle between candidate vel and desired direction)
  clearance : min distance to obstacle / max_obstacle_dist (clipped to [0,1])
  speed     : |v| / max_speed  (prefer faster motion)
  total     : w_heading*heading + w_clearance*clearance + w_speed*speed

Body-frame convention (same as RPP → nav2_mavros_bridge)
----------------------------------------------------------
  x = forward   (positive = move forward)
  y = left      (positive = move left)
  angular.z     = unused (drone doesn't rotate; kept 0)

Parameters
----------
max_speed          : float (default 0.8)  m/s maximum linear speed
v_samples          : int   (default 8)    velocity samples per axis
sim_time           : float (default 1.5)  s simulation horizon
sim_steps          : int   (default 5)    discretisation steps per trajectory
lookahead_dist     : float (default 2.5)  m look-ahead on global path
goal_tolerance     : float (default 0.5)  m to declare goal reached
obstacle_stop_dist : float (default 0.6)  m — stop if obstacle closer than this
w_heading          : float (default 0.6)  heading weight
w_clearance        : float (default 0.3)  clearance weight
w_speed            : float (default 0.1)  speed weight
control_hz         : float (default 10.0) control loop frequency
"""

import math
import struct

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import tf2_ros
import rclpy.duration
import rclpy.time


class LocalPlannerNode(Node):

    def __init__(self):
        super().__init__('local_planner')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('max_speed',          0.6)
        self.declare_parameter('v_samples',          8)
        self.declare_parameter('sim_time',           4.0)   # was 1.5 → checks 2.4m ahead at 0.6m/s
        self.declare_parameter('sim_steps',          10)    # was 5  → finer trajectory resolution
        self.declare_parameter('lookahead_dist',     2.5)
        self.declare_parameter('goal_tolerance',     0.5)
        self.declare_parameter('obstacle_stop_dist', 1.2)   # was 0.6 → stop 1.2m from obstacle
        self.declare_parameter('w_heading',          0.5)   # was 0.6 → less heading bias
        self.declare_parameter('w_clearance',        0.4)   # was 0.3 → weight clearance more
        self.declare_parameter('w_speed',            0.1)
        self.declare_parameter('control_hz',         10.0)

        self._max_v    = self.get_parameter('max_speed').value
        self._v_samp   = self.get_parameter('v_samples').value
        self._sim_t    = self.get_parameter('sim_time').value
        self._sim_n    = self.get_parameter('sim_steps').value
        self._look     = self.get_parameter('lookahead_dist').value
        self._goal_tol = self.get_parameter('goal_tolerance').value
        self._stop_d   = self.get_parameter('obstacle_stop_dist').value
        self._w_head   = self.get_parameter('w_heading').value
        self._w_clr    = self.get_parameter('w_clearance').value
        self._w_spd    = self.get_parameter('w_speed').value

        # ── Pre-build velocity candidates ────────────────────────────────────
        # vx: [0, max_v] — always move forward or stop (never backward in practice)
        # vy: [-max_v, max_v] — full lateral range
        vx_vals = np.linspace(0.0, self._max_v, self._v_samp + 1)
        vy_vals = np.linspace(-self._max_v, self._max_v, 2 * self._v_samp + 1)
        vx_grid, vy_grid = np.meshgrid(vx_vals, vy_vals)
        self._candidates = np.stack(
            [vx_grid.ravel(), vy_grid.ravel()], axis=1)  # (N, 2)

        # ── State ─────────────────────────────────────────────────────────────
        self._path:  Path | None     = None
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_z = 3.0
        self._yaw    = 0.0
        self._goal_z = 3.0
        self._goal_reached = False

        # Latest depth cloud in base_link frame (N, 3) — updated by cloud_cb
        self._cloud_pts: np.ndarray = np.empty((0, 3))
        self._cloud_frame = 'camera_link'

        # Cached camera_link → base_link transform (updated at 5 Hz)
        # Keeping TF out of the 30 Hz cloud callback prevents executor starvation
        # that blocks the 10 Hz control timer on a SingleThreadedExecutor.
        self._T_cam_to_base: np.ndarray | None = None

        # ── TF ───────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(Path,        '/global_path',      self._path_cb,  10)
        self.create_subscription(Odometry,    '/odom',             self._odom_cb,  10)
        self.create_subscription(PointCloud2, '/depth_pointcloud', self._cloud_cb, sensor_qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self._cmd_pub     = self.create_publisher(Twist, '/cmd_vel',      10)
        self._reached_pub = self.create_publisher(Bool,  '/goal_reached', 10)

        # Use wall clock for timers — sim_time timers starve when depth cloud
        # callbacks (30 Hz) saturate the SingleThreadedExecutor
        _wall = Clock(clock_type=ClockType.STEADY_TIME)
        hz = self.get_parameter('control_hz').value
        self.create_timer(1.0 / hz, self._control_loop, clock=_wall)
        self.create_timer(0.2, self._update_tf_cache,   clock=_wall)

        self.get_logger().info(
            f'local_planner ready — max_speed={self._max_v} m/s, '
            f'{len(self._candidates)} velocity candidates'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _path_cb(self, msg: Path):
        self._path         = msg
        self._goal_reached = False
        if msg.poses:
            self._goal_z = msg.poses[-1].pose.position.z
        self.get_logger().info(f'Received global path with {len(msg.poses)} waypoints')

    def _odom_cb(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def _update_tf_cache(self):
        """Refresh cached camera_link → base_link transform at 5 Hz."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'base_link', self._cloud_frame, rclpy.time.Time())
            self._T_cam_to_base = _tf_to_matrix(tf)
        except Exception as exc:
            self.get_logger().warn(
                f'TF cache update failed: {exc}', throttle_duration_sec=5.0)

    def _cloud_cb(self, msg: PointCloud2):
        """Transform depth cloud to base_link frame using cached TF."""
        self._cloud_frame = msg.header.frame_id
        if self._T_cam_to_base is None:
            return                          # TF not ready yet — skip
        pts = _unpack_cloud(msg)
        if len(pts) == 0:
            self._cloud_pts = np.empty((0, 3))
            return
        pts_h = np.hstack([pts, np.ones((len(pts), 1))])
        self._cloud_pts = (self._T_cam_to_base @ pts_h.T).T[:, :3]

    # ── Control loop ─────────────────────────────────────────────────────────

    def _control_loop(self):
        self.get_logger().info(
            f'control tick — path={self._path is not None} '
            f'pos=({self._odom_x:.1f},{self._odom_y:.1f})',
            throttle_duration_sec=2.0)
        if self._path is None or len(self._path.poses) == 0:
            return

        # --- Check goal reached ---
        goal = self._path.poses[-1]
        dist_to_goal = math.hypot(
            self._odom_x - goal.pose.position.x,
            self._odom_y - goal.pose.position.y,
        )
        if dist_to_goal < self._goal_tol:
            if not self._goal_reached:
                self._goal_reached = True
                self.get_logger().info(
                    f'Goal reached! (dist={dist_to_goal:.2f} m)')
                msg      = Bool()
                msg.data = True
                self._reached_pub.publish(msg)
            self._publish_cmd(0.0, 0.0)
            return

        # --- Find lookahead waypoint ---
        target_x, target_y = self._find_lookahead()
        if target_x is None:
            # All waypoints are behind us; head directly to goal
            target_x = goal.pose.position.x
            target_y = goal.pose.position.y

        # Desired direction in world frame
        dxw = target_x - self._odom_x
        dyw = target_y - self._odom_y
        dist_to_target = math.hypot(dxw, dyw)

        if dist_to_target < 0.1:
            self._publish_cmd(0.0, 0.0)
            return

        # Rotate desired direction from world → body frame
        c = math.cos(-self._yaw)
        s = math.sin(-self._yaw)
        des_vx_body =  c * dxw + s * dyw
        des_vy_body = -s * dxw + c * dyw

        # Normalise to unit vector
        mag = math.hypot(des_vx_body, des_vy_body)
        des_vx_body /= mag
        des_vy_body /= mag

        # --- DWA scoring ---
        best_score  = -float('inf')
        best_vx     = 0.0
        best_vy     = 0.0

        for vx, vy in self._candidates:
            speed = math.hypot(vx, vy)
            if speed < 1e-4:
                continue

            # 1. Heading score: cos(angle between candidate and desired)
            heading = (vx * des_vx_body + vy * des_vy_body) / speed

            # 2. Simulate trajectory and find minimum clearance
            min_clear = self._trajectory_clearance(vx, vy)

            # 3. Reject if too close to obstacle
            if min_clear < self._stop_d:
                continue

            # 4. Normalise clearance to [0, 1]
            # Use 3.0 m as the "fully clear" threshold so obstacles at 1-2 m
            # still receive a meaningful penalty and the drone steers away sooner.
            max_d   = 3.0
            clr_n   = min(min_clear / max_d, 1.0)

            # 5. Speed score
            spd_n   = speed / self._max_v

            score = (self._w_head  * max(0.0, heading) +
                     self._w_clr   * clr_n              +
                     self._w_spd   * spd_n)

            if score > best_score:
                best_score = score
                best_vx    = vx
                best_vy    = vy

        if best_score == -float('inf'):
            # All candidates blocked — stop and let global planner replan
            self.get_logger().warn(
                'All DWA candidates blocked — stopping', throttle_duration_sec=1.0)
            self._publish_cmd(0.0, 0.0)
        else:
            self.get_logger().info(
                f'cmd_vel → vx={best_vx:.2f} vy={best_vy:.2f} '
                f'target=({target_x:.1f},{target_y:.1f}) dist={dist_to_goal:.1f}m',
                throttle_duration_sec=1.0)
            self._publish_cmd(best_vx, best_vy)

    def _find_lookahead(self):
        """
        Find the lookahead point on the global path.

        Step 1: find the closest waypoint to the drone (the 'progress' index).
        Step 2: from there, walk forward and return the first waypoint that is
                >= lookahead_dist away.

        Without step 1, once the drone moves past early waypoints those
        waypoints become far away again and get returned as the target,
        causing the drone to reverse back toward the start.
        """
        poses = self._path.poses

        # Step 1 — closest waypoint (tracks progress along the path)
        closest_idx = 0
        closest_d   = float('inf')
        for i, pose in enumerate(poses):
            d = math.hypot(pose.pose.position.x - self._odom_x,
                           pose.pose.position.y - self._odom_y)
            if d < closest_d:
                closest_d = d
                closest_idx = i

        # Step 2 — first waypoint beyond lookahead_dist, starting from closest
        for pose in poses[closest_idx:]:
            wp_x = pose.pose.position.x
            wp_y = pose.pose.position.y
            if math.hypot(wp_x - self._odom_x, wp_y - self._odom_y) >= self._look:
                return wp_x, wp_y

        # All remaining waypoints are within lookahead — head for the last one
        last = poses[-1]
        return last.pose.position.x, last.pose.position.y

    def _trajectory_clearance(self, vx_body: float, vy_body: float) -> float:
        """
        Simulate straight-line trajectory at (vx, vy) body-frame for sim_time
        and return the minimum Euclidean distance to any obstacle point.
        """
        if len(self._cloud_pts) == 0:
            return float('inf')

        dt     = self._sim_t / self._sim_n
        min_d  = float('inf')
        px, py = 0.0, 0.0   # trajectory starts at drone origin (body frame)

        for _ in range(self._sim_n):
            px += vx_body * dt
            py += vy_body * dt
            # Distance from trajectory point to every obstacle point in XY plane
            dx = self._cloud_pts[:, 0] - px
            dy = self._cloud_pts[:, 1] - py
            dists = np.sqrt(dx**2 + dy**2)
            min_d  = min(min_d, float(dists.min()))

        return min_d

    def _publish_cmd(self, vx: float, vy: float):
        msg            = Twist()
        msg.linear.x   = vx
        msg.linear.y   = vy
        msg.linear.z   = 0.0
        msg.angular.z  = 0.0
        self._cmd_pub.publish(msg)


# ── Utility helpers (shared with occupancy_map.py) ───────────────────────────

def _tf_to_matrix(tf) -> np.ndarray:
    t = tf.transform.translation
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - w*z),      2*(x*z + w*y)     ],
        [2*(x*y + w*z),       1 - 2*(x*x + z*z),  2*(y*z - w*x)     ],
        [2*(x*z - w*y),       2*(y*z + w*x),       1 - 2*(x*x + y*y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [t.x, t.y, t.z]
    return T


def _unpack_cloud(msg: PointCloud2) -> np.ndarray:
    step = msg.point_step
    raw  = bytes(msg.data)
    n    = len(raw) // step
    if n == 0:
        return np.empty((0, 3), dtype=np.float64)
    arr = np.frombuffer(raw, dtype=np.uint8).reshape(n, step)
    xyz = np.frombuffer(arr[:, :12].tobytes(), dtype=np.float32).reshape(n, 3)
    return xyz.astype(np.float64)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
occupancy_map.py — Custom Nav Stack
=====================================
Builds a persistent 2D occupancy grid from the depth point cloud.

Data flow
---------
/depth_pointcloud (PointCloud2, camera_link frame)
  → TF transform to map frame
  → project 3D → 2D (keep only drone-altitude band: z ∈ [min_z, max_z])
  → mark grid cells as occupied (store last-seen timestamp)
  → cells not refreshed within decay_time are freed
  → inflate obstacle cells by inflation_radius
  → publish /occupancy_grid (OccupancyGrid, map frame)

Grid layout
-----------
  200 × 200 m, 0.2 m/cell → 1000 × 1000 cells
  origin at (-100, -100) in map frame (covers the full Gazebo SITL arena)
  occupancy value: 0=free, 50=inflated (unsafe), 100=obstacle

Parameters
----------
resolution       : float  (default 0.2)   m/cell
grid_size_m      : float  (default 200.0) total grid side length (m)
origin_x         : float  (default -100)  bottom-left X in map frame
origin_y         : float  (default -100)  bottom-left Y in map frame
min_obstacle_z   : float  (default 1.0)   ignore points below this (m, map frame)
max_obstacle_z   : float  (default 5.5)   ignore points above this (m, map frame)
decay_time       : float  (default 5.0)   seconds before unrefreshed cell freed
inflation_radius : float  (default 0.8)   safety bubble around obstacles (m)
publish_hz       : float  (default 3.0)   grid publication rate
"""

import array as _array
import math
import struct

import numpy as np
import rclpy
import rclpy.time
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import tf2_ros


class OccupancyMapNode(Node):

    def __init__(self):
        super().__init__('occupancy_map')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('resolution',       0.2)
        self.declare_parameter('grid_size_m',      200.0)
        self.declare_parameter('origin_x',        -100.0)
        self.declare_parameter('origin_y',        -100.0)
        self.declare_parameter('min_obstacle_z',   1.0)
        self.declare_parameter('max_obstacle_z',   5.5)
        self.declare_parameter('decay_time',       5.0)
        self.declare_parameter('inflation_radius', 0.8)
        self.declare_parameter('publish_hz',       3.0)

        self._res   = self.get_parameter('resolution').value
        size_m      = self.get_parameter('grid_size_m').value
        self._ncols = int(size_m / self._res)   # x → cols
        self._nrows = int(size_m / self._res)   # y → rows
        self._ox    = self.get_parameter('origin_x').value
        self._oy    = self.get_parameter('origin_y').value
        self._zmin  = self.get_parameter('min_obstacle_z').value
        self._zmax  = self.get_parameter('max_obstacle_z').value
        self._decay = self.get_parameter('decay_time').value

        # ── Pre-compute inflation kernel offsets (sparse) ────────────────────
        # Store only the (dy, dx) offsets of True cells in the circular kernel.
        # _numpy_dilate_sparse uses these to shift occupied-cell indices directly
        # instead of rolling the entire 1M-cell grid (which blocks the spin thread).
        infl_cells = int(math.ceil(
            self.get_parameter('inflation_radius').value / self._res))
        ys, xs = np.mgrid[-infl_cells:infl_cells + 1, -infl_cells:infl_cells + 1]
        kernel = (xs**2 + ys**2) <= (infl_cells**2)
        dy_idx, dx_idx = np.where(kernel)
        # Shift from [0, 2*infl_cells] → [-infl_cells, infl_cells]
        self._kernel_offsets = np.stack(
            [dy_idx - infl_cells, dx_idx - infl_cells], axis=1)  # (K, 2)

        # ── State ─────────────────────────────────────────────────────────────
        # last timestamp each cell was observed as occupied (0.0 = never)
        self._obs_time = np.zeros((self._nrows, self._ncols), dtype=np.float64)
        # Cached camera_link→map transform matrix (updated at 1 Hz, used in cloud_cb)
        # Avoids calling lookup_transform inside the high-rate cloud callback which
        # would block the single-threaded executor and starve the publish timer.
        self._T_cam_to_map: np.ndarray | None = None
        self._cloud_frame: str = 'camera_link'

        # ── TF ───────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS / Pub / Sub ───────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            PointCloud2, '/depth_pointcloud', self._cloud_cb, sensor_qos)
        self._grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Wall clock timers — prevent sim_time timer starvation from
        # the 30 Hz depth cloud callbacks in the SingleThreadedExecutor
        _wall = Clock(clock_type=ClockType.STEADY_TIME)
        hz = self.get_parameter('publish_hz').value
        self.create_timer(1.0 / hz, self._publish_cb,      clock=_wall)
        self.create_timer(0.2,      self._update_tf_cache, clock=_wall)

        self.get_logger().info(
            f'occupancy_map ready — {self._ncols}×{self._nrows} cells @ {self._res} m, '
            f'decay={self._decay} s, inflation kernel={infl_cells} cells'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _update_tf_cache(self):
        """Refresh the cached camera_link→map transform (called at 5 Hz)."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', self._cloud_frame, rclpy.time.Time())
            self._T_cam_to_map = _tf_to_matrix(tf)
        except Exception as exc:
            self.get_logger().warn(
                f'TF cache update failed: {exc}', throttle_duration_sec=5.0)

    def _cloud_cb(self, msg: PointCloud2):
        """Transform depth cloud to map frame; stamp occupied cells."""
        self._cloud_frame = msg.header.frame_id  # track actual frame_id

        # Use cached transform — no blocking TF lookup in the hot path
        if self._T_cam_to_map is None:
            return
        T = self._T_cam_to_map

        # Unpack XYZ from PointCloud2 → numpy (N, 3)
        pts = _unpack_cloud(msg)
        if len(pts) == 0:
            return

        # Transform to map frame
        pts_h   = np.hstack([pts, np.ones((len(pts), 1))])  # (N, 4)
        pts_map = (T @ pts_h.T).T                            # (N, 4)

        # Filter to drone-altitude band (ignores ground and sky)
        mask  = (pts_map[:, 2] >= self._zmin) & (pts_map[:, 2] <= self._zmax)
        xy    = pts_map[mask, :2]
        if len(xy) == 0:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        # World XY → grid (col, row)
        cols = ((xy[:, 0] - self._ox) / self._res).astype(int)
        rows = ((xy[:, 1] - self._oy) / self._res).astype(int)

        # Keep only in-bounds indices
        valid = (cols >= 0) & (cols < self._ncols) & (rows >= 0) & (rows < self._nrows)
        cols, rows = cols[valid], rows[valid]

        # Stamp all hit cells with current time
        self._obs_time[rows, cols] = now

    def _publish_cb(self):
        """Decay stale observations, inflate, and publish OccupancyGrid."""
        try:
            now = self.get_clock().now().nanoseconds * 1e-9

            # Cells that were seen recently enough to still be obstacles
            raw_occ = (self._obs_time > 0.0) & ((now - self._obs_time) < self._decay)

            # Sparse dilation: shift only the occupied-cell indices by each kernel
            # offset. O(N_occupied × K) instead of O(N_grid × K) — 100-1000× faster
            # when obstacles are sparse relative to the full 1000×1000 grid.
            inflated = _numpy_dilate_sparse(raw_occ, self._kernel_offsets,
                                            self._nrows, self._ncols)

            # Encode: 0=free, 50=inflated-only (unsafe margin), 100=obstacle
            data = np.zeros(self._nrows * self._ncols, dtype=np.int8)
            data[inflated.ravel() & ~raw_occ.ravel()] = 50
            data[raw_occ.ravel()] = 100

            msg                           = OccupancyGrid()
            msg.header.stamp              = self.get_clock().now().to_msg()
            msg.header.frame_id           = 'map'
            msg.info.resolution           = self._res
            msg.info.width                = self._ncols
            msg.info.height               = self._nrows
            msg.info.origin.position.x    = self._ox
            msg.info.origin.position.y    = self._oy
            msg.info.origin.orientation.w = 1.0
            msg.data                      = _array.array('b', data.tobytes())

            self._grid_pub.publish(msg)
            self.get_logger().info(
                f'Grid published — obstacles={int(raw_occ.sum())}',
                throttle_duration_sec=5.0)

        except Exception as exc:
            self.get_logger().error(f'_publish_cb crashed: {exc}')


# ── Utility helpers ───────────────────────────────────────────────────────────

def _numpy_dilate_sparse(arr: np.ndarray, kernel_offsets: np.ndarray,
                          nrows: int, ncols: int) -> np.ndarray:
    """
    Sparse binary dilation — O(N_occupied × K) instead of O(N_grid × K).

    For each occupied cell, marks all cells within the kernel radius as
    inflated by shifting the occupied-cell index list by each kernel offset.
    When obstacles are sparse (typical drone nav), this is 100-1000× faster
    than rolling the entire grid.

    Parameters
    ----------
    arr            : (nrows, ncols) bool — raw occupied cells
    kernel_offsets : (K, 2) int — pre-computed (dy, dx) offsets of True kernel cells
    nrows, ncols   : grid dimensions (passed to avoid re-reading arr.shape)
    """
    result = np.zeros_like(arr)
    rows_occ, cols_occ = np.where(arr)
    if len(rows_occ) == 0:
        return result
    for dy, dx in kernel_offsets:
        nr = rows_occ + dy
        nc = cols_occ + dx
        valid = (nr >= 0) & (nr < nrows) & (nc >= 0) & (nc < ncols)
        result[nr[valid], nc[valid]] = True
    return result


def _tf_to_matrix(tf) -> np.ndarray:
    """Convert geometry_msgs/TransformStamped to 4×4 numpy matrix."""
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
    """
    Extract XYZ points from a sensor_msgs/PointCloud2 as (N, 3) float64 array.
    Assumes X=offset 0, Y=offset 4, Z=offset 8, float32 each.
    """
    step = msg.point_step
    raw  = bytes(msg.data)
    n    = len(raw) // step
    if n == 0:
        return np.empty((0, 3), dtype=np.float64)
    # Use numpy stride tricks for efficiency
    arr = np.frombuffer(raw, dtype=np.uint8).reshape(n, step)
    xyz = np.frombuffer(arr[:, :12].tobytes(), dtype=np.float32).reshape(n, 3)
    return xyz.astype(np.float64)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapNode()
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

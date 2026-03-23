#!/usr/bin/env python3
"""
global_planner.py — Custom Nav Stack
======================================
2D A* global planner on the occupancy grid.

Data flow
---------
/occupancy_grid  (OccupancyGrid) — obstacle map from occupancy_map.py
/odom            (Odometry)      — current drone position
/goal_pose       (PoseStamped)   — target set by mission_client.py
  →  A* search from drone position to goal on the 2D grid
  →  path smoothing (remove redundant collinear waypoints)
  →  publish /global_path (nav_msgs/Path, map frame)

Replanning
----------
- Triggers immediately on a new goal
- Checks path validity at 1 Hz: if any waypoint falls on an obstacle cell,
  replans from the current drone position

Obstacle threshold
------------------
Cells with OccupancyGrid value ≥ 50 are treated as impassable (this includes
both raw obstacles=100 and the inflation band=50).

Parameters
----------
replan_hz     : float  (default 1.0)   how often to check path validity
goal_tolerance: float  (default 0.5)   m — within this distance = goal reached
"""

import heapq
import math

import numpy as np
import rclpy
import rclpy.duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String


# Cells with this value or above are impassable
_OBSTACLE_THRESHOLD = 50


class GlobalPlannerNode(Node):

    def __init__(self):
        super().__init__('global_planner')

        self.declare_parameter('replan_hz',     1.0)
        self.declare_parameter('goal_tolerance', 0.5)

        self._goal_tol = self.get_parameter('goal_tolerance').value

        # ── State ─────────────────────────────────────────────────────────────
        self._grid: OccupancyGrid | None = None   # latest occupancy grid
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._goal: PoseStamped | None   = None   # current navigation goal
        self._path: Path | None          = None   # last planned path
        self._new_goal = False                     # flag: replan immediately

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(
            OccupancyGrid, '/occupancy_grid', self._grid_cb, 10)
        self.create_subscription(
            Odometry,  '/odom',      self._odom_cb, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._path_pub   = self.create_publisher(Path,   '/global_path',     10)
        self._status_pub = self.create_publisher(String, '/planning_status', 10)

        hz = self.get_parameter('replan_hz').value
        self.create_timer(1.0 / hz, self._replan_timer)

        self.get_logger().info('global_planner ready — waiting for goal on /goal_pose')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _grid_cb(self, msg: OccupancyGrid):
        self._grid = msg

    def _odom_cb(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y

    def _goal_cb(self, msg: PoseStamped):
        self._goal     = msg
        self._new_goal = True
        self.get_logger().info(
            f'New goal received: ({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f})')
        self._do_plan()

    # ── Periodic path-validity check ──────────────────────────────────────────

    def _replan_timer(self):
        if self._goal is None or self._grid is None:
            return

        if self._path is None or self._new_goal:
            self._do_plan()
            return

        # Check if any existing waypoint is now on an obstacle cell
        if self._path_blocked():
            self.get_logger().info('Path blocked — replanning')
            self._do_plan()

    def _path_blocked(self) -> bool:
        """Return True if any waypoint in the current path hits an obstacle."""
        if self._path is None or self._grid is None:
            return True
        grid_data, info = self._grid_as_array()
        for pose in self._path.poses:
            col, row = self._world_to_grid(
                pose.pose.position.x, pose.pose.position.y, info)
            if col is None:
                continue
            if grid_data[row, col] >= _OBSTACLE_THRESHOLD:
                return True
        return False

    # ── Planning ─────────────────────────────────────────────────────────────

    def _do_plan(self):
        self._new_goal = False

        if self._grid is None:
            self.get_logger().warn('No occupancy grid yet — cannot plan')
            return
        if self._goal is None:
            return

        grid_data, info = self._grid_as_array()

        start_col, start_row = self._world_to_grid(
            self._odom_x, self._odom_y, info)
        goal_col,  goal_row  = self._world_to_grid(
            self._goal.pose.position.x, self._goal.pose.position.y, info)

        if start_col is None or goal_col is None:
            self.get_logger().warn('Start or goal is outside the grid!')
            self._publish_status('no_path')
            return

        self.get_logger().info(
            f'Planning from grid ({start_col},{start_row}) to ({goal_col},{goal_row})')

        path_cells = _astar(grid_data, (start_row, start_col), (goal_row, goal_col))

        if path_cells is None:
            self.get_logger().warn('A* found no path — goal unreachable in current map')
            self._publish_status('no_path')
            return

        # Convert grid cells → world coordinates
        waypoints = []
        for row, col in path_cells:
            wx = info.origin.position.x + (col + 0.5) * info.resolution
            wy = info.origin.position.y + (row + 0.5) * info.resolution
            waypoints.append((wx, wy))

        # Smooth: remove collinear waypoints
        waypoints = _smooth_path(waypoints)

        # Build nav_msgs/Path
        path                    = Path()
        path.header.stamp       = self.get_clock().now().to_msg()
        path.header.frame_id    = 'map'
        goal_z = self._goal.pose.position.z

        for wx, wy in waypoints:
            ps                      = PoseStamped()
            ps.header.frame_id      = 'map'
            ps.header.stamp         = path.header.stamp
            ps.pose.position.x      = wx
            ps.pose.position.y      = wy
            ps.pose.position.z      = goal_z
            ps.pose.orientation.w   = 1.0
            path.poses.append(ps)

        self._path = path
        self._path_pub.publish(path)
        self._publish_status('ok')

        self.get_logger().info(f'Path published — {len(waypoints)} waypoints')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _grid_as_array(self):
        """Return (ndarray[nrows,ncols], MapMetaData) from the cached grid."""
        info = self._grid.info
        data = np.array(self._grid.data, dtype=np.int8).reshape(
            info.height, info.width)
        return data, info

    def _world_to_grid(self, wx, wy, info):
        """Convert world (x,y) to (col, row). Returns (None, None) if out of bounds."""
        col = int((wx - info.origin.position.x) / info.resolution)
        row = int((wy - info.origin.position.y) / info.resolution)
        if col < 0 or col >= info.width or row < 0 or row >= info.height:
            return None, None
        return col, row

    def _publish_status(self, status: str):
        msg      = String()
        msg.data = status
        self._status_pub.publish(msg)


# ── A* implementation ─────────────────────────────────────────────────────────

def _astar(grid: np.ndarray, start: tuple, goal: tuple):
    """
    8-connected A* on a 2D numpy grid.

    Parameters
    ----------
    grid  : (nrows, ncols) int8 — cells >= OBSTACLE_THRESHOLD are blocked
    start : (row, col)
    goal  : (row, col)

    Returns
    -------
    List of (row, col) tuples from start to goal inclusive, or None if no path.
    """
    nrows, ncols = grid.shape

    def h(r, c):
        # Octile heuristic (consistent for 8-connectivity)
        dr = abs(r - goal[0])
        dc = abs(c - goal[1])
        return max(dr, dc) + (math.sqrt(2) - 1) * min(dr, dc)

    # open set: (f, g, row, col)
    open_heap = []
    g_score   = {start: 0.0}
    came_from = {}

    heapq.heappush(open_heap, (h(*start), 0.0, start[0], start[1]))

    # 8-connected neighbours
    MOVES = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1,-1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
        ( 1,-1, math.sqrt(2)), ( 1, 1, math.sqrt(2)),
    ]

    while open_heap:
        _, g, r, c = heapq.heappop(open_heap)

        if (r, c) == goal:
            # Reconstruct path
            path = []
            cur  = goal
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            path.reverse()
            return path

        if g > g_score.get((r, c), float('inf')):
            continue  # stale entry

        for dr, dc, cost in MOVES:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= nrows or nc < 0 or nc >= ncols:
                continue
            if grid[nr, nc] >= _OBSTACLE_THRESHOLD:
                continue
            ng = g + cost
            if ng < g_score.get((nr, nc), float('inf')):
                g_score[(nr, nc)] = ng
                came_from[(nr, nc)] = (r, c)
                heapq.heappush(open_heap, (ng + h(nr, nc), ng, nr, nc))

    return None  # no path found


def _smooth_path(waypoints: list) -> list:
    """
    Remove redundant waypoints that lie on the same line as their neighbours.
    Uses angle threshold: if direction change < 5°, skip the middle point.
    """
    if len(waypoints) <= 2:
        return waypoints

    result = [waypoints[0]]
    for i in range(1, len(waypoints) - 1):
        x0, y0 = result[-1]
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        # Vectors: prev→curr and curr→next
        d1 = math.hypot(x1 - x0, y1 - y0)
        d2 = math.hypot(x2 - x1, y2 - y1)
        if d1 < 1e-6 or d2 < 1e-6:
            continue
        cos_a = ((x1 - x0) * (x2 - x1) + (y1 - y0) * (y2 - y1)) / (d1 * d2)
        cos_a = max(-1.0, min(1.0, cos_a))
        angle = math.acos(cos_a)
        if angle > math.radians(5.0):
            result.append(waypoints[i])
    result.append(waypoints[-1])
    return result


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
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

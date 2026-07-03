#!/usr/bin/env python3
# Author: Gourav Khanduri
# Email:  gauravkhanduri93@gmail.com
"""
Global Planner Node for ISR Drone
ROS2 Humble

Subscribes to:
    /projected_map              → nav_msgs/OccupancyGrid (from OctoMap)
    /zed/zed_node/pose          → geometry_msgs/PoseStamped (drone position from VIO)

Provides service:
    /plan_path                  → isr_drone_interfaces/srv/PlanPath (custom)
    
    Since we don't have the custom service yet, we use a simpler approach:
    Subscribe to /goal_pose and publish the planned path.

Publishes:
    /planned_path               → nav_msgs/Path (collision-free path for executor)
    /planning_grid_debug        → nav_msgs/OccupancyGrid (inflated grid for RViz debug)

Architecture:
    1. OctoMap builds 2D grid from stereo camera in real-time
    2. This node receives the grid, inflates obstacles, runs A* 
    3. Executor follows the planned path via /mavros/setpoint_position/local
    4. When grid updates significantly or path is blocked, replan automatically
"""

import math
import heapq
import numpy as np
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header


class GlobalPlannerNode(Node):
    """A* global planner that works with live OctoMap data."""

    # 8-connected grid neighbors: (delta_row, delta_col, cost)
    NEIGHBORS = [
        (-1, 0, 1.0),    # up
        (1, 0, 1.0),     # down
        (0, -1, 1.0),    # left
        (0, 1, 1.0),     # right
        (-1, -1, 1.414), # diagonals
        (-1, 1, 1.414),
        (1, -1, 1.414),
        (1, 1, 1.414),
    ]

    def __init__(self):
        super().__init__('global_planner')

        # ---- Parameters ----
        self.declare_parameter('drone_radius', 0.4)          # meters, for inflation
        self.declare_parameter('replan_interval', 2.0)        # seconds
        self.declare_parameter('waypoint_reached_radius', 0.5)# meters
        self.declare_parameter('flight_altitude', 5.0)        # meters (constant z)
        self.declare_parameter('treat_unknown_as_free', True)  # optimistic planning
        self.declare_parameter('max_planning_iterations', 200000)

        self.drone_radius = self.get_parameter('drone_radius').value
        self.replan_interval = self.get_parameter('replan_interval').value
        self.wp_reached_radius = self.get_parameter('waypoint_reached_radius').value
        self.flight_altitude = self.get_parameter('flight_altitude').value
        self.optimistic = self.get_parameter('treat_unknown_as_free').value
        self.max_iterations = self.get_parameter('max_planning_iterations').value

        # ---- State ----
        self.grid_data = None           # Raw OccupancyGrid from OctoMap
        self.grid_array = None          # numpy array (rows x cols)
        self.inflated_grid = None       # numpy array after inflation
        self.grid_info = None           # MapMetaData (resolution, origin, size)

        self.drone_pose = None          # Current (x, y, z) in map frame
        self.current_goal = None        # Current (x, y) goal
        self.current_path = None        # List of PoseStamped
        self.goal_waypoints = []        # Full mission waypoint list
        self.current_wp_index = 0       # Index into goal_waypoints

        self.needs_replan = False
        self.last_plan_time = self.get_clock().now()

        # ---- QoS ----
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ---- Subscribers ----
        self.create_subscription(
            OccupancyGrid,
            '/projected_map',
            self._grid_callback,
            map_qos,
        )

        self.create_subscription(
            PoseStamped,
            '/zed/zed_node/pose',
            self._pose_callback,
            sensor_qos,
        )

        # Goal input — operator sends a single goal or the executor sends next waypoint
        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._goal_callback,
            10,
        )

        # Mission waypoint list input (from executor or GUI bridge)
        self.create_subscription(
            Path,
            '/mission_waypoints',
            self._mission_callback,
            10,
        )

        # ---- Publishers ----
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.debug_grid_pub = self.create_publisher(
            OccupancyGrid, '/planning_grid_debug', 1
        )

        # ---- Replan timer ----
        self.create_timer(self.replan_interval, self._replan_timer)

        # ---- Startup ----
        self.get_logger().info(
            f'Global planner started | '
            f'drone_radius={self.drone_radius}m | '
            f'optimistic={self.optimistic} | '
            f'replan_interval={self.replan_interval}s'
        )

    # ================================================================
    # Callbacks
    # ================================================================

    def _grid_callback(self, msg: OccupancyGrid):
        """Receive updated 2D occupancy grid from OctoMap."""
        self.grid_info = msg.info
        width = msg.info.width
        height = msg.info.height

        # Convert flat array to 2D numpy grid
        # OccupancyGrid data: -1 = unknown, 0 = free, 100 = occupied
        raw = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.grid_array = raw
        self.grid_data = msg

        # Inflate obstacles by drone radius
        self._inflate_grid()

        # Check if current path is still valid
        if self.current_path is not None:
            if not self._is_path_valid():
                self.get_logger().warn('Current path is blocked! Requesting replan.')
                self.needs_replan = True

    def _pose_callback(self, msg: PoseStamped):
        """Receive current drone position from VIO."""
        self.drone_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    def _goal_callback(self, msg: PoseStamped):
        """Receive a single goal pose — plan a path to it."""
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f'New goal received: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})'
        )
        self._plan_and_publish()

    def _mission_callback(self, msg: Path):
        """
        Receive a full mission as a Path message.
        Each pose in the path is a survey waypoint from the GUI.
        The planner will plan to each one sequentially.
        """
        self.goal_waypoints = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        self.current_wp_index = 0
        self.get_logger().info(
            f'Mission received: {len(self.goal_waypoints)} waypoints'
        )

        if self.goal_waypoints:
            self.current_goal = self.goal_waypoints[0]
            self._plan_and_publish()

    def _replan_timer(self):
        """Periodic replan check."""
        if self.current_goal is None or self.drone_pose is None:
            return

        # Check if we've reached the current goal waypoint
        if self.goal_waypoints:
            dx = self.drone_pose[0] - self.current_goal[0]
            dy = self.drone_pose[1] - self.current_goal[1]
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < self.wp_reached_radius:
                self.current_wp_index += 1
                if self.current_wp_index < len(self.goal_waypoints):
                    self.current_goal = self.goal_waypoints[self.current_wp_index]
                    self.get_logger().info(
                        f'Waypoint {self.current_wp_index} reached! '
                        f'Planning to waypoint {self.current_wp_index + 1}: '
                        f'({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})'
                    )
                    self.needs_replan = True
                else:
                    self.get_logger().info('Mission complete! All waypoints reached.')
                    self.current_goal = None
                    self.current_path = None
                    # Publish empty path to signal completion
                    empty_path = Path()
                    empty_path.header.stamp = self.get_clock().now().to_msg()
                    empty_path.header.frame_id = 'map'
                    self.path_pub.publish(empty_path)
                    return

        # Replan if needed (grid changed, path blocked, or periodic)
        if self.needs_replan:
            self._plan_and_publish()
            self.needs_replan = False

    # ================================================================
    # Grid Processing
    # ================================================================

    def _inflate_grid(self):
        """Inflate obstacles by drone radius on the current grid."""
        if self.grid_array is None or self.grid_info is None:
            return

        resolution = self.grid_info.resolution
        inflate_cells = int(math.ceil(self.drone_radius / resolution))

        if inflate_cells <= 0:
            self.inflated_grid = self.grid_array.copy()
            return

        # Create binary obstacle mask (occupied = True)
        obstacle_mask = (self.grid_array == 100)

        # Create circular kernel
        y, x = np.ogrid[
            -inflate_cells:inflate_cells + 1,
            -inflate_cells:inflate_cells + 1,
        ]
        kernel = (x * x + y * y) <= inflate_cells * inflate_cells

        # Dilate using convolution (faster than scipy on small kernels)
        from scipy.ndimage import binary_dilation
        inflated_mask = binary_dilation(obstacle_mask, structure=kernel)

        # Build inflated grid: keep unknown/free status, add inflated obstacles
        self.inflated_grid = self.grid_array.copy()
        # Only mark as occupied cells that were free or unknown before
        new_obstacles = inflated_mask & (self.grid_array != 100)
        self.inflated_grid[new_obstacles] = 100

        # Publish debug grid for RViz visualization
        self._publish_debug_grid()

    def _publish_debug_grid(self):
        """Publish the inflated grid so you can visualize it in RViz."""
        if self.grid_data is None or self.inflated_grid is None:
            return

        debug_msg = OccupancyGrid()
        debug_msg.header = self.grid_data.header
        debug_msg.info = self.grid_data.info
        debug_msg.data = self.inflated_grid.flatten().tolist()
        self.debug_grid_pub.publish(debug_msg)

    # ================================================================
    # Coordinate Transforms
    # ================================================================

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert map frame (x, y) to grid (row, col)."""
        info = self.grid_info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        return (row, col)

    def _grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Convert grid (row, col) to map frame (x, y)."""
        info = self.grid_info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return (x, y)

    def _is_cell_free(self, row: int, col: int) -> bool:
        """Check if a cell is passable for the planner."""
        grid = self.inflated_grid if self.inflated_grid is not None else self.grid_array
        if grid is None:
            return False

        rows, cols = grid.shape
        if row < 0 or row >= rows or col < 0 or col >= cols:
            return False

        val = grid[row, col]
        if val == 0:
            return True      # Known free
        if val == 100:
            return False     # Known occupied (or inflated)
        # val == -1: unknown
        return self.optimistic  # True = assume passable, False = avoid

    # ================================================================
    # A* Planner
    # ================================================================

    def _plan_astar(
        self,
        start_xy: Tuple[float, float],
        goal_xy: Tuple[float, float],
    ) -> Optional[List[Tuple[float, float]]]:
        """
        A* search on the inflated occupancy grid.
        Returns list of (x, y) in map frame, or None.
        """
        if self.inflated_grid is None:
            self.get_logger().warn('No grid available for planning')
            return None

        sr, sc = self._world_to_grid(*start_xy)
        gr, gc = self._world_to_grid(*goal_xy)

        # Validate start
        if not self._is_cell_free(sr, sc):
            self.get_logger().warn(
                f'Start ({sr},{sc}) is occupied! Finding nearest free cell...'
            )
            result = self._find_nearest_free(sr, sc)
            if result is None:
                return None
            sr, sc = result

        # Validate goal
        if not self._is_cell_free(gr, gc):
            self.get_logger().warn(
                f'Goal ({gr},{gc}) is occupied! Finding nearest free cell...'
            )
            result = self._find_nearest_free(gr, gc)
            if result is None:
                return None
            gr, gc = result

        # A* search
        counter = 0
        open_set = []
        h0 = self._heuristic(sr, sc, gr, gc)
        heapq.heappush(open_set, (h0, counter, sr, sc))
        counter += 1

        came_from = {}
        g_score = {(sr, sc): 0.0}
        closed = set()

        iterations = 0

        while open_set and iterations < self.max_iterations:
            iterations += 1
            f, _, cr, cc = heapq.heappop(open_set)

            if cr == gr and cc == gc:
                # Reconstruct path
                path_grid = []
                current = (gr, gc)
                while current != (sr, sc):
                    path_grid.append(current)
                    current = came_from[current]
                path_grid.append((sr, sc))
                path_grid.reverse()

                # Convert to world coordinates
                path_world = [self._grid_to_world(r, c) for r, c in path_grid]

                # Ensure exact start/goal
                path_world[0] = start_xy
                path_world[-1] = goal_xy

                self.get_logger().info(
                    f'A* found path: {len(path_world)} raw points, '
                    f'{iterations} iterations'
                )
                return path_world

            if (cr, cc) in closed:
                continue
            closed.add((cr, cc))

            for dr, dc, cost in self.NEIGHBORS:
                nr, nc = cr + dr, cc + dc

                if not self._is_cell_free(nr, nc):
                    continue
                if (nr, nc) in closed:
                    continue

                # Corner-cutting check for diagonals
                if dr != 0 and dc != 0:
                    if not self._is_cell_free(cr + dr, cc) or not self._is_cell_free(cr, cc + dc):
                        continue

                tg = g_score[(cr, cc)] + cost
                if tg < g_score.get((nr, nc), float('inf')):
                    came_from[(nr, nc)] = (cr, cc)
                    g_score[(nr, nc)] = tg
                    fn = tg + self._heuristic(nr, nc, gr, gc)
                    heapq.heappush(open_set, (fn, counter, nr, nc))
                    counter += 1

        self.get_logger().warn(f'A* failed after {iterations} iterations')
        return None

    @staticmethod
    def _heuristic(r1: int, c1: int, r2: int, c2: int) -> float:
        """Octile distance heuristic for 8-connected grid."""
        dr = abs(r1 - r2)
        dc = abs(c1 - c2)
        return 1.414 * min(dr, dc) + abs(dr - dc)

    def _find_nearest_free(
        self, row: int, col: int, max_search: int = 50
    ) -> Optional[Tuple[int, int]]:
        """BFS to find nearest free cell."""
        from collections import deque
        queue = deque([(row, col, 0)])
        visited = {(row, col)}

        while queue:
            r, c, dist = queue.popleft()
            if dist > max_search:
                return None
            if self._is_cell_free(r, c):
                return (r, c)
            for dr, dc, _ in self.NEIGHBORS:
                nr, nc = r + dr, c + dc
                grid = self.inflated_grid if self.inflated_grid is not None else self.grid_array
                if grid is not None:
                    rows, cols = grid.shape
                    if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        queue.append((nr, nc, dist + 1))
        return None

    # ================================================================
    # Path Smoothing
    # ================================================================

    def _smooth_path(
        self, path: List[Tuple[float, float]]
    ) -> List[Tuple[float, float]]:
        """Line-of-sight path simplification."""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            farthest = current_idx + 1

            for check_idx in range(len(path) - 1, current_idx, -1):
                r1, c1 = self._world_to_grid(*path[current_idx])
                r2, c2 = self._world_to_grid(*path[check_idx])
                if self._is_line_free(r1, c1, r2, c2):
                    farthest = check_idx
                    break

            smoothed.append(path[farthest])
            current_idx = farthest

        return smoothed

    def _is_line_free(self, r1: int, c1: int, r2: int, c2: int) -> bool:
        """Bresenham line check — is the straight line collision-free?"""
        dr = abs(r2 - r1)
        dc = abs(c2 - c1)
        sr = 1 if r1 < r2 else -1
        sc = 1 if c1 < c2 else -1
        err = dr - dc

        r, c = r1, c1
        while True:
            if not self._is_cell_free(r, c):
                return False
            if r == r2 and c == c2:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc
        return True

    # ================================================================
    # Path Validation & Publishing
    # ================================================================

    def _is_path_valid(self) -> bool:
        """Check if the current planned path is still collision-free."""
        if self.current_path is None:
            return False

        for pose in self.current_path:
            x = pose.pose.position.x
            y = pose.pose.position.y
            r, c = self._world_to_grid(x, y)
            grid = self.inflated_grid if self.inflated_grid is not None else self.grid_array
            if grid is not None:
                rows, cols = grid.shape
                if 0 <= r < rows and 0 <= c < cols:
                    if grid[r, c] == 100:
                        return False  # A cell on the path is now occupied
        return True

    def _plan_and_publish(self):
        """Run the full planning pipeline and publish the result."""
        if self.drone_pose is None:
            self.get_logger().warn('Cannot plan: no drone pose available')
            return

        if self.current_goal is None:
            return

        if self.inflated_grid is None:
            self.get_logger().warn('Cannot plan: no occupancy grid available')
            return

        start = (self.drone_pose[0], self.drone_pose[1])
        goal = self.current_goal

        self.get_logger().info(
            f'Planning: ({start[0]:.2f}, {start[1]:.2f}) → '
            f'({goal[0]:.2f}, {goal[1]:.2f})'
        )

        # Run A*
        raw_path = self._plan_astar(start, goal)

        if raw_path is None:
            self.get_logger().error('No path found to goal!')
            return

        # Smooth
        smooth_path = self._smooth_path(raw_path)

        self.get_logger().info(
            f'Path: {len(raw_path)} raw → {len(smooth_path)} smoothed points'
        )

        # Convert to nav_msgs/Path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y in smooth_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = self.flight_altitude
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        # Store and publish
        self.current_path = path_msg.poses
        self.path_pub.publish(path_msg)
        self.last_plan_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

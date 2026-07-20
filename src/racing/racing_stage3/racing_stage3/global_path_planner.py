"""A* Global Path Planner for Stage3 Return Navigation

Ported from Stage2 A* planner to avoid forbidden zones (black obstacles) in the map.
Features:
- Reads /map occupancy grid
- A* planning from start to goal
- Obstacle inflation
- Static-map collision checks
"""

import heapq
import json
import math

import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class GlobalPathPlanner:
    """A* global path planner"""

    def __init__(self, node, config):
        """
        Args:
            node: ROS2 node instance
            config: Configuration dict with the following parameters:
                - planner_downsample: int, map downsampling factor
                - planner_occupied_threshold: int, obstacle threshold (0-100)
                - planner_unknown_is_occupied: bool, treat unknown as obstacle
                - planner_obstacle_inflation_m: float, obstacle inflation radius (m)
                - global_frame_id: str, global coordinate frame name (usually 'map')
        """
        self.node = node

        # Configuration parameters
        self.planner_downsample = config.get('planner_downsample', 4)
        self.planner_occupied_threshold = config.get('planner_occupied_threshold', 50)
        self.planner_unknown_is_occupied = config.get('planner_unknown_is_occupied', False)
        self.planner_obstacle_inflation_m = config.get('planner_obstacle_inflation_m', 0.14)
        self.global_frame_id = config.get('global_frame_id', 'map')
        self.forbidden_rectangles = self._parse_forbidden_rectangles(
            config.get('planner_forbidden_rectangles_json', '[]')
        )
        self.clear_rectangles = self._parse_forbidden_rectangles(
            config.get('planner_clear_rectangles_json', '[]')
        )

        # Map cache
        self.latest_map = None
        self.static_planner_grid = None
        self.static_planner_resolution = None
        self.static_planner_origin = None

        # Path cache
        self.last_plan_points = []
        self.last_plan_signature = None
        self.last_plan_at = 0.0

        # Subscribe to the static map only. Lidar belongs to local avoidance.
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.node.create_subscription(
            OccupancyGrid,
            config.get('map_topic', '/map'),
            self._map_callback,
            qos_latched,
        )

    def _map_callback(self, msg):
        """map callback"""
        self.latest_map = msg
        self.static_planner_grid = None
        self.static_planner_resolution = None
        self.static_planner_origin = None

    def plan_path(self, start_position, goal_position, now_sec):
        """
        A* path from start to goal
        Args:
            start_position: tuple(x, y), map start
            goal_position: tuple(x, y), map goal
            now_sec: float, current time
        Returns:
            list of tuple(x, y): path points
            None: map not loaded
            []: planning failed
        """
        planner_grid = self._build_static_planner_grid()
        if planner_grid is None:
            return None

        occupied, resolution, origin_x, origin_y = planner_grid
        height, width = occupied.shape

        start_cell = self._world_to_planner_cell(
            start_position[0], start_position[1], resolution, origin_x, origin_y, width, height
        )
        goal_cell = self._world_to_planner_cell(
            goal_position[0], goal_position[1], resolution, origin_x, origin_y, width, height
        )

        if start_cell is None or goal_cell is None:
            self.last_plan_points = []
            self.last_plan_signature = None
            return []

        # A black map pixel is a hard forbidden area. Do not silently move an
        # occupied start/goal to a nearby free cell or clear the occupied bit.
        if occupied[start_cell[1], start_cell[0]] or occupied[goal_cell[1], goal_cell[0]]:
            self.last_plan_points = []
            self.last_plan_signature = None
            return []

        signature = (start_cell, goal_cell)

        # A* 
        cell_path = self._a_star_grid_path(occupied, start_cell, goal_cell)
        if not cell_path:
            self.last_plan_points = []
            self.last_plan_signature = signature
            self.last_plan_at = now_sec
            return []

        world_points = [start_position]
        for cell_x, cell_y in cell_path[1:-1]:
            world_points.append(
                self._planner_cell_to_world(cell_x, cell_y, resolution, origin_x, origin_y)
            )
        world_points.append(goal_position)

        self.last_plan_points = list(world_points)
        self.last_plan_signature = signature
        self.last_plan_at = now_sec
        return world_points

    def is_world_segment_free(self, start_position, end_position):
        """Return whether a world-space segment stays in free planner cells."""
        planner_grid = self._build_static_planner_grid()
        if planner_grid is None:
            return None

        occupied, resolution, origin_x, origin_y = planner_grid
        height, width = occupied.shape
        start = self._world_to_planner_cell(
            start_position[0], start_position[1], resolution, origin_x, origin_y, width, height
        )
        end = self._world_to_planner_cell(
            end_position[0], end_position[1], resolution, origin_x, origin_y, width, height
        )
        if start is None or end is None:
            return False

        steps = max(abs(end[0] - start[0]), abs(end[1] - start[1]), 1)
        for index in range(steps + 1):
            ratio = index / steps
            cell_x = round(start[0] + (end[0] - start[0]) * ratio)
            cell_y = round(start[1] + (end[1] - start[1]) * ratio)
            if occupied[cell_y, cell_x]:
                return False
        return True

    def describe_world_occupancy(self, position):
        """Return the planner layer that occupies a world-space position."""
        planner_grid = self._build_static_planner_grid()
        if planner_grid is None:
            return 'map_unavailable'

        static_occupied, resolution, origin_x, origin_y = planner_grid
        height, width = static_occupied.shape
        cell = self._world_to_planner_cell(
            position[0], position[1], resolution, origin_x, origin_y, width, height
        )
        if cell is None:
            return 'outside_planner_grid'
        if static_occupied[cell[1], cell[0]]:
            return 'static_forbidden_rectangle_or_inflation'
        return 'free'

    def _build_static_planner_grid(self):
        """build static planner grid"""
        if self.latest_map is None:
            return None

        if self.static_planner_grid is not None:
            return (
                self.static_planner_grid.copy(),
                self.static_planner_resolution,
                self.static_planner_origin[0],
                self.static_planner_origin[1],
            )

        info = self.latest_map.info
        width = int(info.width)
        height = int(info.height)
        if width <= 0 or height <= 0:
            return None

        if self.forbidden_rectangles:
            # Stage3 only treats the explicitly configured competition zones as
            # static forbidden space. The remaining black artwork in /map is
            # visual reference, not a navigation obstacle.
            x_centers = float(info.origin.position.x) + (np.arange(width) + 0.5) * float(info.resolution)
            y_centers = float(info.origin.position.y) + (np.arange(height) + 0.5) * float(info.resolution)
            occupied = np.zeros((height, width), dtype=bool)
            for x_min, x_max, y_min, y_max in self.forbidden_rectangles:
                x_mask = (x_centers >= x_min) & (x_centers < x_max)
                y_mask = (y_centers >= y_min) & (y_centers < y_max)
                occupied |= y_mask[:, np.newaxis] & x_mask[np.newaxis, :]
        else:
            raw = np.asarray(self.latest_map.data, dtype=np.int16).reshape((height, width))
            occupied = raw >= self.planner_occupied_threshold
            if self.planner_unknown_is_occupied:
                occupied |= raw < 0

            # Only explicitly reviewed visual artwork may be removed from the
            # real map occupancy layer used by Stage3.
            if self.clear_rectangles:
                x_centers = (
                    float(info.origin.position.x)
                    + (np.arange(width) + 0.5) * float(info.resolution)
                )
                y_centers = (
                    float(info.origin.position.y)
                    + (np.arange(height) + 0.5) * float(info.resolution)
                )
                for x_min, x_max, y_min, y_max in self.clear_rectangles:
                    x_mask = (x_centers >= x_min) & (x_centers < x_max)
                    y_mask = (y_centers >= y_min) & (y_centers < y_max)
                    occupied[y_mask[:, np.newaxis] & x_mask[np.newaxis, :]] = False

        stride = self.planner_downsample
        padded_height = int(math.ceil(height / stride) * stride)
        padded_width = int(math.ceil(width / stride) * stride)
        padded = np.ones((padded_height, padded_width), dtype=bool)
        padded[:height, :width] = occupied
        coarse = padded.reshape(
            padded_height // stride,
            stride,
            padded_width // stride,
            stride,
        ).max(axis=(1, 3))

        coarse_resolution = float(info.resolution) * stride
        inflation_cells = int(
            math.ceil(self.planner_obstacle_inflation_m / max(coarse_resolution, 1e-6))
        )
        inflated = self._inflate_binary_grid(coarse, inflation_cells)

        self.static_planner_grid = inflated
        self.static_planner_resolution = coarse_resolution
        self.static_planner_origin = (
            float(info.origin.position.x),
            float(info.origin.position.y),
        )

        return (
            inflated.copy(),
            coarse_resolution,
            self.static_planner_origin[0],
            self.static_planner_origin[1],
        )

    def _a_star_grid_path(self, occupied, start_cell, goal_cell):
        """a star grid path"""
        neighbors = [
            (-1, -1, math.sqrt(2.0)),
            (0, -1, 1.0),
            (1, -1, math.sqrt(2.0)),
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (-1, 1, math.sqrt(2.0)),
            (0, 1, 1.0),
            (1, 1, math.sqrt(2.0)),
        ]

        height, width = occupied.shape
        open_heap = []
        g_cost = {start_cell: 0.0}
        came_from = {}
        heapq.heappush(open_heap, (0.0, start_cell))

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal_cell:
                return self._reconstruct_a_star_path(came_from, current)

            current_cost = g_cost[current]
            for dx, dy, step_cost in neighbors:
                next_x = current[0] + dx
                next_y = current[1] + dy
                if next_x < 0 or next_y < 0 or next_x >= width or next_y >= height:
                    continue
                if occupied[next_y, next_x]:
                    continue
                # Do not squeeze diagonally through two obstacle corners.
                if dx and dy and (occupied[current[1], next_x] or occupied[next_y, current[0]]):
                    continue

                next_cell = (next_x, next_y)
                next_cost = current_cost + step_cost
                if next_cost >= g_cost.get(next_cell, float('inf')):
                    continue

                came_from[next_cell] = current
                g_cost[next_cell] = next_cost
                heuristic = math.hypot(goal_cell[0] - next_x, goal_cell[1] - next_y)
                heapq.heappush(open_heap, (next_cost + heuristic, next_cell))

        return None

    def _reconstruct_a_star_path(self, came_from, current):
        """reconstruct a star path"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _world_to_planner_cell(self, x_value, y_value, resolution, origin_x, origin_y, width, height):
        """world to planner cell"""
        cell_x = int(math.floor((x_value - origin_x) / resolution))
        cell_y = int(math.floor((y_value - origin_y) / resolution))
        if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
            return None
        return (cell_x, cell_y)

    def _planner_cell_to_world(self, cell_x, cell_y, resolution, origin_x, origin_y):
        """planner cell to world"""
        return (
            origin_x + (cell_x + 0.5) * resolution,
            origin_y + (cell_y + 0.5) * resolution,
        )

    def _nearest_free_planner_cell(self, occupied, cell, max_radius_cells=12):
        """nearest free planner cell"""
        if cell is None:
            return None

        cell_x, cell_y = cell
        height, width = occupied.shape
        if 0 <= cell_x < width and 0 <= cell_y < height and not occupied[cell_y, cell_x]:
            return cell

        for radius in range(1, max_radius_cells + 1):
            min_x = max(0, cell_x - radius)
            max_x = min(width - 1, cell_x + radius)
            min_y = max(0, cell_y - radius)
            max_y = min(height - 1, cell_y + radius)
            for y_index in range(min_y, max_y + 1):
                for x_index in range(min_x, max_x + 1):
                    if max(abs(x_index - cell_x), abs(y_index - cell_y)) != radius:
                        continue
                    if not occupied[y_index, x_index]:
                        return (x_index, y_index)

        return None

    @staticmethod
    def _parse_forbidden_rectangles(raw):
        """Parse half-open [x_min, x_max) x [y_min, y_max) static zones."""
        try:
            data = json.loads(raw) if isinstance(raw, str) else raw
        except (TypeError, ValueError):
            return []
        if not isinstance(data, list):
            return []

        rectangles = []
        for item in data:
            if not isinstance(item, dict):
                continue
            try:
                x_min = float(item['x_min'])
                x_max = float(item['x_max'])
                y_min = float(item['y_min'])
                y_max = float(item['y_max'])
            except (KeyError, TypeError, ValueError):
                continue
            if x_max > x_min and y_max > y_min:
                rectangles.append((x_min, x_max, y_min, y_max))
        return rectangles

    def _inflate_binary_grid(self, grid, radius_cells):
        """inflate binary grid"""
        if radius_cells <= 0:
            return grid.copy()

        kernel_size = radius_cells * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        return cv2.dilate(grid.astype(np.uint8), kernel) > 0

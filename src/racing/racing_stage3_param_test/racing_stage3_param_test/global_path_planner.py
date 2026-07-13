"""A* Global Path Planner for Stage3 Return Navigation

从 Stage2 移植的 A* 路径规划器，用于避开地图中的禁区（黑色障碍物）。

功能：
- 读取 /map occupancy grid
- A* 规划从起点到终点的路径
- 障碍物膨胀（inflation）
- 动态障碍物叠加（来自激光雷达）
- 路径缓存与周期性重规划
"""

import heapq
import math

import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class GlobalPathPlanner:
    """A* 全局路径规划器"""

    def __init__(self, node, config):
        """
        Args:
            node: ROS2 节点实例
            config: 配置字典，包含以下参数：
                - planner_downsample: int, 地图下采样倍数
                - planner_occupied_threshold: int, 障碍物阈值（0-100）
                - planner_unknown_is_occupied: bool, 未知区域视为障碍物
                - planner_obstacle_inflation_m: float, 障碍物膨胀半径（m）
                - planner_dynamic_obstacle_box_size_m: float, 动态障碍物盒大小（m）
                - planner_dynamic_obstacle_inflation_m: float, 动态障碍物膨胀半径（m）
                - planner_dynamic_obstacle_range_m: float, 动态障碍物检测范围（m）
                - planner_replan_period_sec: float, 重规划周期（s）
                - global_frame_id: str, 全局坐标系名称（通常是 'map'）
        """
        self.node = node

        # 配置参数
        self.planner_downsample = config.get('planner_downsample', 4)
        self.planner_occupied_threshold = config.get('planner_occupied_threshold', 50)
        self.planner_unknown_is_occupied = config.get('planner_unknown_is_occupied', False)
        self.planner_obstacle_inflation_m = config.get('planner_obstacle_inflation_m', 0.14)
        self.planner_dynamic_obstacle_box_size_m = config.get('planner_dynamic_obstacle_box_size_m', 0.25)
        self.planner_dynamic_obstacle_inflation_m = config.get('planner_dynamic_obstacle_inflation_m', 0.04)
        self.planner_dynamic_obstacle_range_m = config.get('planner_dynamic_obstacle_range_m', 0.7)
        self.planner_replan_period_sec = config.get('planner_replan_period_sec', 0.25)
        self.global_frame_id = config.get('global_frame_id', 'map')

        # 地图缓存
        self.latest_map = None
        self.static_planner_grid = None
        self.static_planner_resolution = None
        self.static_planner_origin = None

        # 激光扫描缓存
        self.latest_scan = None
        self.scan_frame_id = None

        # 路径缓存
        self.last_plan_points = []
        self.last_plan_signature = None
        self.last_plan_at = 0.0

        # TF 缓存
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.cached_2d_transforms = {}

        # 订阅地图和激光扫描
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
        self.node.create_subscription(
            LaserScan,
            config.get('scan_topic', '/scan'),
            self._scan_callback,
            10,
        )

    def _map_callback(self, msg):
        """地图回调，清除静态规划网格缓存"""
        self.latest_map = msg
        self.static_planner_grid = None
        self.static_planner_resolution = None
        self.static_planner_origin = None

    def _scan_callback(self, msg):
        """激光扫描回调"""
        self.latest_scan = msg
        if msg.header.frame_id:
            self.scan_frame_id = msg.header.frame_id

    def plan_path(self, start_position, goal_position, now_sec):
        """
        A* 规划从起点到终点的路径

        Args:
            start_position: tuple(x, y), map 坐标系起点
            goal_position: tuple(x, y), map 坐标系终点
            now_sec: float, 当前时间（秒）

        Returns:
            list of tuple(x, y): 路径点列表（map 坐标），
            None: 地图未加载，
            []: 路径规划失败
        """
        planner_grid = self._build_static_planner_grid()
        if planner_grid is None:
            return None

        occupied, resolution, origin_x, origin_y = planner_grid
        occupied = self._overlay_scan_obstacles(occupied, resolution, origin_x, origin_y)
        height, width = occupied.shape

        start_cell = self._world_to_planner_cell(
            start_position[0], start_position[1], resolution, origin_x, origin_y, width, height
        )
        goal_cell = self._world_to_planner_cell(
            goal_position[0], goal_position[1], resolution, origin_x, origin_y, width, height
        )

        start_cell = self._nearest_free_planner_cell(occupied, start_cell)
        goal_cell = self._nearest_free_planner_cell(occupied, goal_cell)

        if start_cell is None or goal_cell is None:
            self.last_plan_points = []
            self.last_plan_signature = None
            return []

        # 强制设置起点和终点为自由
        occupied = occupied.copy()
        occupied[start_cell[1], start_cell[0]] = False
        occupied[goal_cell[1], goal_cell[0]] = False

        # 路径缓存检查
        signature = (start_cell, goal_cell)
        if (
            self.last_plan_points
            and self.last_plan_signature == signature
            and now_sec - self.last_plan_at < self.planner_replan_period_sec
        ):
            return list(self.last_plan_points)

        # A* 规划
        cell_path = self._a_star_grid_path(occupied, start_cell, goal_cell)
        if not cell_path:
            self.last_plan_points = []
            self.last_plan_signature = signature
            self.last_plan_at = now_sec
            return []

        # 转换为世界坐标
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

    def _build_static_planner_grid(self):
        """构建静态规划网格（下采样 + 膨胀）"""
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

        raw = np.asarray(self.latest_map.data, dtype=np.int16).reshape((height, width))
        occupied = raw >= self.planner_occupied_threshold
        if self.planner_unknown_is_occupied:
            occupied |= raw < 0

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

    def _overlay_scan_obstacles(self, occupied, resolution, origin_x, origin_y):
        """叠加激光扫描的动态障碍物"""
        if self.latest_scan is None or not self.scan_frame_id:
            return occupied

        transform = self._lookup_2d_transform(self.global_frame_id, self.scan_frame_id)
        if transform is None:
            return occupied

        height, width = occupied.shape
        dynamic_mask = np.zeros((height, width), dtype=np.uint8)
        trans_x, trans_y, trans_yaw = transform
        cos_yaw = math.cos(trans_yaw)
        sin_yaw = math.sin(trans_yaw)
        dynamic_box_cells = max(
            1,
            int(math.ceil(self.planner_dynamic_obstacle_box_size_m / max(resolution, 1e-6))),
        )

        max_scan_range = self.latest_scan.range_max
        if (
            math.isfinite(self.planner_dynamic_obstacle_range_m)
            and self.planner_dynamic_obstacle_range_m > 0.0
        ):
            max_scan_range = min(max_scan_range, self.planner_dynamic_obstacle_range_m)

        for index, distance in enumerate(self.latest_scan.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance <= 0.0:
                continue
            if distance > max_scan_range:
                continue

            angle = self.latest_scan.angle_min + index * self.latest_scan.angle_increment
            scan_x = distance * math.cos(angle)
            scan_y = distance * math.sin(angle)
            world_x = trans_x + cos_yaw * scan_x - sin_yaw * scan_y
            world_y = trans_y + sin_yaw * scan_x + cos_yaw * scan_y
            cell = self._world_to_planner_cell(
                world_x, world_y, resolution, origin_x, origin_y, width, height
            )
            if cell is None:
                continue
            self._stamp_square_cells(dynamic_mask, cell[0], cell[1], dynamic_box_cells)

        inflation_cells = int(
            math.ceil(self.planner_dynamic_obstacle_inflation_m / max(resolution, 1e-6))
        )
        if inflation_cells > 0:
            dynamic_mask = self._inflate_binary_grid(
                dynamic_mask > 0, inflation_cells
            ).astype(np.uint8)

        return occupied | (dynamic_mask > 0)

    def _a_star_grid_path(self, occupied, start_cell, goal_cell):
        """A* 网格路径规划（8连通）"""
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
        """重建 A* 路径"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _world_to_planner_cell(self, x_value, y_value, resolution, origin_x, origin_y, width, height):
        """世界坐标 → 规划网格单元"""
        cell_x = int(math.floor((x_value - origin_x) / resolution))
        cell_y = int(math.floor((y_value - origin_y) / resolution))
        if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
            return None
        return (cell_x, cell_y)

    def _planner_cell_to_world(self, cell_x, cell_y, resolution, origin_x, origin_y):
        """规划网格单元 → 世界坐标"""
        return (
            origin_x + (cell_x + 0.5) * resolution,
            origin_y + (cell_y + 0.5) * resolution,
        )

    def _nearest_free_planner_cell(self, occupied, cell, max_radius_cells=12):
        """查找最近的自由单元"""
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

    def _inflate_binary_grid(self, grid, radius_cells):
        """二值网格膨胀"""
        if radius_cells <= 0:
            return grid.copy()

        kernel_size = radius_cells * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        return cv2.dilate(grid.astype(np.uint8), kernel) > 0

    def _stamp_square_cells(self, grid, center_x, center_y, box_cells):
        """在网格上标记方形区域"""
        if box_cells <= 1:
            if 0 <= center_y < grid.shape[0] and 0 <= center_x < grid.shape[1]:
                grid[center_y, center_x] = 1
            return

        if box_cells % 2 == 0:
            box_cells += 1

        half_cells = box_cells // 2
        min_x = max(0, center_x - half_cells)
        max_x = min(grid.shape[1], center_x + half_cells + 1)
        min_y = max(0, center_y - half_cells)
        max_y = min(grid.shape[0], center_y + half_cells + 1)
        grid[min_y:max_y, min_x:max_x] = 1

    def _lookup_2d_transform(self, target_frame, source_frame):
        """查询 2D 坐标变换"""
        if not target_frame or not source_frame:
            return None
        if target_frame == source_frame:
            return (0.0, 0.0, 0.0)

        cache_key = (target_frame, source_frame)
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except TransformException:
            return self.cached_2d_transforms.get(cache_key)

        translation = transform.transform.translation
        yaw = self._quaternion_to_yaw(transform.transform.rotation)
        transform_2d = (float(translation.x), float(translation.y), yaw)
        self.cached_2d_transforms[cache_key] = transform_2d
        return transform_2d

    @staticmethod
    def _quaternion_to_yaw(q):
        """四元数转 yaw 角"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

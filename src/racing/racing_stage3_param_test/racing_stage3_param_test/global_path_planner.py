"""Global map-frame path planning (A* on /map + dynamic scan overlay).

Adapted from ``racing_stage2`` corridor navigation for stage3 param test.
"""

from __future__ import annotations

import heapq
import math

import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class GlobalPathPlannerMixin:
    """TF lookup, occupancy-grid A*, and pure-pursuit helpers in map frame."""

    def init_global_path_planner(self):
        self.latest_map: OccupancyGrid | None = None
        self.latest_scan: LaserScan | None = None
        self.scan_frame_id = ''
        self.static_planner_grid = None
        self.static_planner_resolution = None
        self.static_planner_origin = None
        self.last_plan_points = []
        self.last_plan_signature = None
        self.last_plan_at = 0.0
        self.cached_2d_transforms = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lookup_2d_transform(self, target_frame, source_frame):
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
        yaw = self.quaternion_to_yaw(transform.transform.rotation)
        transform_2d = (float(translation.x), float(translation.y), yaw)
        self.cached_2d_transforms[cache_key] = transform_2d
        return transform_2d

    def transform_point_2d(self, point, target_frame, source_frame):
        if point is None:
            return None

        transform = self.lookup_2d_transform(target_frame, source_frame)
        if transform is None:
            return None

        x_value, y_value = point
        trans_x, trans_y, trans_yaw = transform
        target_x = trans_x + math.cos(trans_yaw) * x_value - math.sin(trans_yaw) * y_value
        target_y = trans_y + math.sin(trans_yaw) * x_value + math.cos(trans_yaw) * y_value
        return (target_x, target_y)

    def transform_yaw_2d(self, yaw, target_frame, source_frame):
        if yaw is None:
            return None

        transform = self.lookup_2d_transform(target_frame, source_frame)
        if transform is None:
            return None

        return self.normalize_angle(yaw + transform[2])

    def current_global_position(self):
        if self.current_position is None:
            return None
        mapped = self.transform_point_2d(
            self.current_position,
            self.global_frame_id,
            self.odom_frame_id,
        )
        if mapped is None:
            return self.current_position
        return mapped

    def selected_global_yaw(self):
        odom_yaw = self.current_odom_yaw
        imu_yaw = self.current_yaw

        if self.global_yaw_source == 'odom':
            return odom_yaw if odom_yaw is not None else imu_yaw
        if self.global_yaw_source == 'imu':
            return imu_yaw if imu_yaw is not None else odom_yaw
        if odom_yaw is None:
            return imu_yaw
        if imu_yaw is None:
            return odom_yaw
        if abs(self.angle_error(imu_yaw, odom_yaw)) > self.global_yaw_disagreement:
            return imu_yaw
        return odom_yaw

    def current_global_yaw(self):
        source_yaw = self.selected_global_yaw()
        mapped = self.transform_yaw_2d(
            source_yaw,
            self.global_frame_id,
            self.odom_frame_id,
        )
        if mapped is None:
            return source_yaw
        return mapped

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg
        self.static_planner_grid = None
        self.static_planner_resolution = None
        self.static_planner_origin = None

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.scan_frame_id = msg.header.frame_id or self.scan_frame_id

    def inflate_binary_grid(self, grid, radius_cells):
        if radius_cells <= 0:
            return grid.copy()

        kernel_size = radius_cells * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        return cv2.dilate(grid.astype(np.uint8), kernel) > 0

    def stamp_square_cells(self, grid, center_x, center_y, box_cells):
        if box_cells <= 1:
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

    def build_static_planner_grid(self):
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
        inflation_cells = int(math.ceil(self.planner_obstacle_inflation_m / max(coarse_resolution, 1e-6)))
        inflated = self.inflate_binary_grid(coarse, inflation_cells)

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

    def world_to_planner_cell(self, x_value, y_value, resolution, origin_x, origin_y, width, height):
        cell_x = int(math.floor((x_value - origin_x) / resolution))
        cell_y = int(math.floor((y_value - origin_y) / resolution))
        if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
            return None
        return (cell_x, cell_y)

    def planner_cell_to_world(self, cell_x, cell_y, resolution, origin_x, origin_y):
        return (
            origin_x + (cell_x + 0.5) * resolution,
            origin_y + (cell_y + 0.5) * resolution,
        )

    def nearest_free_planner_cell(self, occupied, cell, max_radius_cells=12):
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

    def overlay_scan_obstacles(self, occupied, resolution, origin_x, origin_y):
        if self.latest_scan is None or not self.scan_frame_id:
            return occupied

        transform = self.lookup_2d_transform(self.global_frame_id, self.scan_frame_id)
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
        if math.isfinite(self.planner_dynamic_obstacle_range_m) and self.planner_dynamic_obstacle_range_m > 0.0:
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
            cell = self.world_to_planner_cell(world_x, world_y, resolution, origin_x, origin_y, width, height)
            if cell is None:
                continue
            self.stamp_square_cells(dynamic_mask, cell[0], cell[1], dynamic_box_cells)

        inflation_cells = int(
            math.ceil(self.planner_dynamic_obstacle_inflation_m / max(resolution, 1e-6))
        )
        if inflation_cells > 0:
            dynamic_mask = self.inflate_binary_grid(dynamic_mask > 0, inflation_cells).astype(np.uint8)

        return occupied | (dynamic_mask > 0)

    def reconstruct_a_star_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def a_star_grid_path(self, occupied, start_cell, goal_cell):
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
                return self.reconstruct_a_star_path(came_from, current)

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

    def plan_global_path(self, start_position, goal_position, now_sec):
        planner_grid = self.build_static_planner_grid()
        if planner_grid is None:
            return None

        occupied, resolution, origin_x, origin_y = planner_grid
        occupied = self.overlay_scan_obstacles(occupied, resolution, origin_x, origin_y)
        height, width = occupied.shape
        start_cell = self.world_to_planner_cell(
            start_position[0], start_position[1], resolution, origin_x, origin_y, width, height
        )
        goal_cell = self.world_to_planner_cell(
            goal_position[0], goal_position[1], resolution, origin_x, origin_y, width, height
        )
        start_cell = self.nearest_free_planner_cell(occupied, start_cell)
        goal_cell = self.nearest_free_planner_cell(occupied, goal_cell)
        if start_cell is None or goal_cell is None:
            self.last_plan_points = []
            self.last_plan_signature = None
            return []

        occupied = occupied.copy()
        occupied[start_cell[1], start_cell[0]] = False
        occupied[goal_cell[1], goal_cell[0]] = False

        signature = (start_cell, goal_cell)
        if (
            self.last_plan_points
            and self.last_plan_signature == signature
            and now_sec - self.last_plan_at < self.planner_replan_period_sec
        ):
            return list(self.last_plan_points)

        cell_path = self.a_star_grid_path(occupied, start_cell, goal_cell)
        if not cell_path:
            self.last_plan_points = []
            self.last_plan_signature = signature
            self.last_plan_at = now_sec
            return []

        world_points = [start_position]
        for cell_x, cell_y in cell_path[1:-1]:
            world_points.append(
                self.planner_cell_to_world(cell_x, cell_y, resolution, origin_x, origin_y)
            )
        world_points.append(goal_position)

        self.last_plan_points = list(world_points)
        self.last_plan_signature = signature
        self.last_plan_at = now_sec
        return world_points

    def select_path_lookahead_point(self, path_points, lookahead_distance):
        if not path_points:
            return None
        if len(path_points) == 1:
            return path_points[0]

        traveled = 0.0
        previous_point = path_points[0]
        for point in path_points[1:]:
            traveled += math.hypot(point[0] - previous_point[0], point[1] - previous_point[1])
            if traveled >= lookahead_distance:
                return point
            previous_point = point

        return path_points[-1]

    def publish_path_points(self, points, frame_id=None):
        path_msg = Path()
        path_msg.header.frame_id = frame_id or self.global_frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for index, point in enumerate(points):
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = path_msg.header.frame_id
            pose_msg.header.stamp = path_msg.header.stamp
            pose_msg.pose.position.x = float(point[0])
            pose_msg.pose.position.y = float(point[1])

            pose_yaw = 0.0
            if index < len(points) - 1:
                next_point = points[index + 1]
                pose_yaw = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
            elif index > 0:
                previous_point = points[index - 1]
                pose_yaw = math.atan2(point[1] - previous_point[1], point[0] - previous_point[0])

            orientation_z, orientation_w = self.yaw_to_quaternion(pose_yaw)
            pose_msg.pose.orientation.z = orientation_z
            pose_msg.pose.orientation.w = orientation_w
            path_msg.poses.append(pose_msg)

        self.return_path_pub.publish(path_msg)

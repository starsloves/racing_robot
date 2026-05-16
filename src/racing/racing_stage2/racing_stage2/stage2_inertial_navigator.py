import heapq
import json
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformException, TransformListener


class Stage2InertialNavigator(Node):
    def __init__(self):
        super().__init__('stage2_inertial_navigator')

        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('task_topic', 'competition_qr_task')
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/stage2_cmd_vel')
        self.declare_parameter('corridor_path_topic', '/stage2_corridor_path')
        self.declare_parameter('feedback_topic', 'competition_feedback')
        self.declare_parameter('state_topic', 'stage2_state')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('start_delay_sec', 0.5)
        self.declare_parameter('corridor_linear_speed', 0.18)
        self.declare_parameter('ring_linear_speed', 0.20)
        self.declare_parameter('turn_linear_speed', 0.08)
        self.declare_parameter('turn_angular_speed', 0.75)
        self.declare_parameter('turn_min_angular_speed', 0.45)
        self.declare_parameter('turn_kp', 1.8)
        self.declare_parameter('heading_kp', 1.6)
        self.declare_parameter('max_angular_speed', 0.9)
        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('heading_tolerance_deg', 4.0)
        self.declare_parameter('segment_timeout', 12.0)
        self.declare_parameter('odd_is_clockwise', True)
        self.declare_parameter('clockwise_keywords', '顺时针,cw,clockwise')
        self.declare_parameter('counterclockwise_keywords', '逆时针,ccw,counterclockwise')
        self.declare_parameter('pre_loop_plan_json', '[]')
        self.declare_parameter('use_corridor_path', False)
        self.declare_parameter('corridor_waypoints_are_global', False)
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('global_yaw_source', 'odom')
        self.declare_parameter('global_yaw_disagreement_deg', 45.0)
        self.declare_parameter('corridor_waypoints_json', '[]')
        self.declare_parameter('corridor_waypoint_tolerance', 0.20)
        self.declare_parameter('corridor_goal_tolerance', 0.12)
        self.declare_parameter('corridor_goal_standoff_m', 0.0)
        self.declare_parameter('corridor_goal_yaw_tolerance_deg', 6.0)
        self.declare_parameter('corridor_path_timeout_sec', 45.0)
        self.declare_parameter('pure_pursuit_lookahead_m', 0.45)
        self.declare_parameter('pure_pursuit_heading_stop_deg', 70.0)
        self.declare_parameter('pure_pursuit_turn_kp', 1.8)
        self.declare_parameter('use_occupancy_grid_planner', True)
        self.declare_parameter('planner_downsample', 4)
        self.declare_parameter('planner_occupied_threshold', 50)
        self.declare_parameter('planner_unknown_is_occupied', True)
        self.declare_parameter('planner_obstacle_inflation_m', 0.14)
        self.declare_parameter('planner_dynamic_obstacle_box_size_m', 0.25)
        self.declare_parameter('planner_dynamic_obstacle_inflation_m', 0.12)
        self.declare_parameter('planner_dynamic_obstacle_range_m', 2.5)
        self.declare_parameter('planner_replan_period_sec', 0.25)
        self.declare_parameter('corridor_path_skip_pre_loop_plan', True)
        self.declare_parameter('post_corridor_path_plan_json', '[]')
        self.declare_parameter('loop_long_length_m', 3.5)
        self.declare_parameter('loop_short_length_m', 1.15)
        self.declare_parameter('exit_distance_m', 1.0)
        self.declare_parameter('detour_enabled', True)
        self.declare_parameter('detour_obstacle_distance', 0.48)
        self.declare_parameter('detour_front_angle_deg', 18.0)
        self.declare_parameter('detour_side_center_deg', 65.0)
        self.declare_parameter('detour_side_window_deg', 30.0)
        self.declare_parameter('detour_min_side_clearance', 0.55)
        self.declare_parameter('detour_lateral_distance_m', 0.32)
        self.declare_parameter('detour_forward_distance_m', 0.75)
        self.declare_parameter('detour_cooldown_sec', 2.0)

        self.phase_topic = self.get_parameter('phase_topic').value
        self.task_topic = self.get_parameter('task_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.corridor_path_topic = self.get_parameter('corridor_path_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.corridor_linear_speed = float(self.get_parameter('corridor_linear_speed').value)
        self.ring_linear_speed = float(self.get_parameter('ring_linear_speed').value)
        self.turn_linear_speed = float(self.get_parameter('turn_linear_speed').value)
        self.turn_angular_speed = float(self.get_parameter('turn_angular_speed').value)
        self.turn_min_angular_speed = float(self.get_parameter('turn_min_angular_speed').value)
        self.turn_kp = float(self.get_parameter('turn_kp').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.distance_tolerance = float(self.get_parameter('distance_tolerance').value)
        self.heading_tolerance = math.radians(float(self.get_parameter('heading_tolerance_deg').value))
        self.segment_timeout = float(self.get_parameter('segment_timeout').value)
        self.odd_is_clockwise = bool(self.get_parameter('odd_is_clockwise').value)
        self.clockwise_keywords = self.parse_keywords(self.get_parameter('clockwise_keywords').value)
        self.counterclockwise_keywords = self.parse_keywords(self.get_parameter('counterclockwise_keywords').value)
        self.pre_loop_plan_json = self.get_parameter('pre_loop_plan_json').value
        self.use_corridor_path = bool(self.get_parameter('use_corridor_path').value)
        self.corridor_waypoints_are_global = bool(
            self.get_parameter('corridor_waypoints_are_global').value
        )
        self.global_frame_id = str(self.get_parameter('global_frame_id').value).strip() or 'map'
        self.global_yaw_source = str(self.get_parameter('global_yaw_source').value).strip().lower() or 'odom'
        if self.global_yaw_source not in ('auto', 'odom', 'imu'):
            self.global_yaw_source = 'odom'
        self.global_yaw_disagreement = math.radians(
            float(self.get_parameter('global_yaw_disagreement_deg').value)
        )
        self.corridor_waypoints_json = self.get_parameter('corridor_waypoints_json').value
        self.corridor_waypoint_tolerance = float(self.get_parameter('corridor_waypoint_tolerance').value)
        self.corridor_goal_tolerance = float(self.get_parameter('corridor_goal_tolerance').value)
        self.corridor_goal_standoff_m = float(self.get_parameter('corridor_goal_standoff_m').value)
        self.corridor_goal_yaw_tolerance = math.radians(
            float(self.get_parameter('corridor_goal_yaw_tolerance_deg').value)
        )
        self.corridor_path_timeout_sec = float(self.get_parameter('corridor_path_timeout_sec').value)
        self.pure_pursuit_lookahead_m = float(self.get_parameter('pure_pursuit_lookahead_m').value)
        self.pure_pursuit_heading_stop = math.radians(
            float(self.get_parameter('pure_pursuit_heading_stop_deg').value)
        )
        self.pure_pursuit_turn_kp = float(self.get_parameter('pure_pursuit_turn_kp').value)
        self.use_occupancy_grid_planner = bool(
            self.get_parameter('use_occupancy_grid_planner').value
        )
        self.planner_downsample = max(1, int(self.get_parameter('planner_downsample').value))
        self.planner_occupied_threshold = int(self.get_parameter('planner_occupied_threshold').value)
        self.planner_unknown_is_occupied = bool(
            self.get_parameter('planner_unknown_is_occupied').value
        )
        self.planner_obstacle_inflation_m = float(
            self.get_parameter('planner_obstacle_inflation_m').value
        )
        self.planner_dynamic_obstacle_box_size_m = max(
            0.0,
            float(self.get_parameter('planner_dynamic_obstacle_box_size_m').value),
        )
        self.planner_dynamic_obstacle_inflation_m = float(
            self.get_parameter('planner_dynamic_obstacle_inflation_m').value
        )
        self.planner_dynamic_obstacle_range_m = float(
            self.get_parameter('planner_dynamic_obstacle_range_m').value
        )
        self.planner_replan_period_sec = float(
            self.get_parameter('planner_replan_period_sec').value
        )
        self.corridor_path_skip_pre_loop_plan = bool(
            self.get_parameter('corridor_path_skip_pre_loop_plan').value
        )
        self.post_corridor_path_plan_json = self.get_parameter('post_corridor_path_plan_json').value
        self.loop_long_length_m = float(self.get_parameter('loop_long_length_m').value)
        self.loop_short_length_m = float(self.get_parameter('loop_short_length_m').value)
        self.exit_distance_m = float(self.get_parameter('exit_distance_m').value)
        self.detour_enabled = bool(self.get_parameter('detour_enabled').value)
        self.detour_obstacle_distance = float(self.get_parameter('detour_obstacle_distance').value)
        self.detour_front_angle_deg = float(self.get_parameter('detour_front_angle_deg').value)
        self.detour_side_center_deg = float(self.get_parameter('detour_side_center_deg').value)
        self.detour_side_window_deg = float(self.get_parameter('detour_side_window_deg').value)
        self.detour_min_side_clearance = float(self.get_parameter('detour_min_side_clearance').value)
        self.detour_lateral_distance_m = float(self.get_parameter('detour_lateral_distance_m').value)
        self.detour_forward_distance_m = float(self.get_parameter('detour_forward_distance_m').value)
        self.detour_cooldown_sec = float(self.get_parameter('detour_cooldown_sec').value)

        self.phase = 1
        self.task_raw = ''
        self.direction = None
        self.current_yaw = None
        self.current_position = None
        self.current_odom_yaw = None
        self.latest_map = None
        self.latest_scan = None
        self.scan_frame_id = ''
        self.static_planner_grid = None
        self.static_planner_resolution = None
        self.static_planner_origin = None
        self.last_corridor_plan_points = []
        self.last_corridor_plan_signature = None
        self.last_corridor_plan_at = 0.0
        self.plan = []
        self.plan_index = -1
        self.current_segment = None
        self.segment_started_at = None
        self.segment_start_pose = None
        self.segment_start_yaw = None
        self.segment_target_yaw = None
        self.segment_heading = None
        self.segment_state_label = 'idle'
        self.start_after_time = None
        self.mission_active = False
        self.mission_finished = False
        self.reported_start = False
        self.front_obstacle_distance = float('inf')
        self.left_clearance_distance = float('inf')
        self.right_clearance_distance = float('inf')
        self.detour_cooldown_until = 0.0
        self.pending_segment_start_pose = None
        self.pending_segment_start_yaw = None
        self.odom_frame_id = 'odom'
        self.corridor_waypoints = self.parse_waypoints_json(
            self.corridor_waypoints_json,
            'corridor_waypoints_json',
            self.corridor_linear_speed,
        )
        self.corridor_path_active = False
        self.corridor_path_index = 0
        self.corridor_path_origin_pose = None
        self.corridor_path_origin_yaw = None
        self.corridor_path_started_at = None
        self.cached_2d_transforms = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = ReliabilityPolicy.RELIABLE

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        event_qos = QoSProfile(depth=1)
        event_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        event_qos.reliability = ReliabilityPolicy.RELIABLE

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.corridor_path_pub = self.create_publisher(Path, self.corridor_path_topic, path_qos)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.create_subscription(Int32, self.phase_topic, self.phase_callback, event_qos)
        self.create_subscription(String, self.task_topic, self.task_callback, event_qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.publish_state('idle')
        self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)

        self.get_logger().info('stage2 inertial navigator ready')

    def parse_keywords(self, raw_value):
        return [item.strip().lower() for item in raw_value.split(',') if item.strip()]

    def parse_waypoints_json(self, raw_json, param_name, default_speed):
        try:
            raw_waypoints = json.loads(raw_json)
        except json.JSONDecodeError:
            self.get_logger().error(f'{param_name} is invalid, fallback to empty waypoints')
            return []

        if not isinstance(raw_waypoints, list):
            self.get_logger().error(f'{param_name} must decode to a list, fallback to empty waypoints')
            return []

        sanitized_waypoints = []
        for index, raw_waypoint in enumerate(raw_waypoints):
            if not isinstance(raw_waypoint, dict):
                continue

            yaw_deg = raw_waypoint.get('yaw_deg')
            sanitized_waypoints.append({
                'x': float(raw_waypoint.get('x', 0.0)),
                'y': float(raw_waypoint.get('y', 0.0)),
                'speed': float(raw_waypoint.get('speed', default_speed)),
                'yaw_deg': None if yaw_deg is None else float(yaw_deg),
                'description': raw_waypoint.get('description', f'corridor_wp_{index}'),
            })

        return sanitized_waypoints

    def quaternion_to_yaw(self, orientation):
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def angle_error(self, target_angle, current_angle):
        return self.normalize_angle(target_angle - current_angle)

    def yaw_to_quaternion(self, yaw):
        half_yaw = yaw / 2.0
        return (math.sin(half_yaw), math.cos(half_yaw))

    def create_twist(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        return msg

    def clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def publish_feedback(self, text):
        self.feedback_pub.publish(String(data=text))
        self.get_logger().info(text)

    def publish_state(self, text):
        self.segment_state_label = text
        self.state_pub.publish(String(data=text))

    def clear_corridor_path(self):
        self.publish_path_points([])

    def publish_path_points(self, points, frame_id=None):
        path_msg = Path()
        path_msg.header.frame_id = frame_id or (
            self.global_frame_id if self.corridor_waypoints_are_global else self.odom_frame_id
        )
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

        self.corridor_path_pub.publish(path_msg)

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

        return self.transform_point_2d(
            self.current_position,
            self.global_frame_id,
            self.odom_frame_id,
        )

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
        return self.transform_yaw_2d(
            source_yaw,
            self.global_frame_id,
            self.odom_frame_id,
        )

    def corridor_waypoint_target_yaw(self, index):
        waypoint = self.corridor_waypoints[index]
        yaw_deg = waypoint.get('yaw_deg')
        if yaw_deg is not None:
            return math.radians(float(yaw_deg))

        if index < len(self.corridor_waypoints) - 1:
            next_waypoint = self.corridor_waypoints[index + 1]
            return math.atan2(
                next_waypoint['y'] - waypoint['y'],
                next_waypoint['x'] - waypoint['x'],
            )

        if index > 0:
            previous_waypoint = self.corridor_waypoints[index - 1]
            return math.atan2(
                waypoint['y'] - previous_waypoint['y'],
                waypoint['x'] - previous_waypoint['x'],
            )

        return None

    def corridor_waypoint_target_position(self, index):
        waypoint = self.corridor_waypoints[index]
        target_x = waypoint['x']
        target_y = waypoint['y']

        if index == len(self.corridor_waypoints) - 1 and self.corridor_goal_standoff_m > 1e-6:
            target_yaw = self.corridor_waypoint_target_yaw(index)
            if target_yaw is not None:
                target_x -= self.corridor_goal_standoff_m * math.cos(target_yaw)
                target_y -= self.corridor_goal_standoff_m * math.sin(target_yaw)

        return (target_x, target_y)

    def map_callback(self, msg):
        self.latest_map = msg
        self.static_planner_grid = None
        self.static_planner_resolution = None
        self.static_planner_origin = None

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

    def plan_corridor_path(self, start_position, goal_position, now_sec):
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
            self.last_corridor_plan_points = []
            self.last_corridor_plan_signature = None
            return []

        occupied = occupied.copy()
        occupied[start_cell[1], start_cell[0]] = False
        occupied[goal_cell[1], goal_cell[0]] = False

        signature = (start_cell, goal_cell)
        if (
            self.last_corridor_plan_points
            and self.last_corridor_plan_signature == signature
            and now_sec - self.last_corridor_plan_at < self.planner_replan_period_sec
        ):
            return list(self.last_corridor_plan_points)

        cell_path = self.a_star_grid_path(occupied, start_cell, goal_cell)
        if not cell_path:
            self.last_corridor_plan_points = []
            self.last_corridor_plan_signature = signature
            self.last_corridor_plan_at = now_sec
            return []

        world_points = [start_position]
        for cell_x, cell_y in cell_path[1:-1]:
            world_points.append(
                self.planner_cell_to_world(cell_x, cell_y, resolution, origin_x, origin_y)
            )
        world_points.append(goal_position)

        self.last_corridor_plan_points = list(world_points)
        self.last_corridor_plan_signature = signature
        self.last_corridor_plan_at = now_sec
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

    def publish_corridor_path(self):
        if not self.corridor_waypoints:
            return

        if (
            not self.corridor_waypoints_are_global
            and (self.corridor_path_origin_pose is None or self.corridor_path_origin_yaw is None)
        ):
            return

        path_msg = Path()
        path_msg.header.frame_id = self.global_frame_id if self.corridor_waypoints_are_global else self.odom_frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        if self.corridor_waypoints_are_global:
            start_position = self.current_global_position()
            start_yaw = self.current_global_yaw()
            if start_position is None:
                return

            start_pose_msg = PoseStamped()
            start_pose_msg.header.frame_id = path_msg.header.frame_id
            start_pose_msg.header.stamp = path_msg.header.stamp
            start_pose_msg.pose.position.x = start_position[0]
            start_pose_msg.pose.position.y = start_position[1]
            start_yaw = 0.0 if start_yaw is None else start_yaw
            orientation_z, orientation_w = self.yaw_to_quaternion(start_yaw)
            start_pose_msg.pose.orientation.z = orientation_z
            start_pose_msg.pose.orientation.w = orientation_w
            path_msg.poses.append(start_pose_msg)

        origin_x, origin_y = (0.0, 0.0)
        origin_yaw = 0.0
        if not self.corridor_waypoints_are_global:
            origin_x, origin_y = self.corridor_path_origin_pose
            origin_yaw = self.corridor_path_origin_yaw

        for index, waypoint in enumerate(self.corridor_waypoints):
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = path_msg.header.frame_id
            pose_msg.header.stamp = path_msg.header.stamp
            waypoint_x, waypoint_y = self.corridor_waypoint_target_position(index)
            if self.corridor_waypoints_are_global:
                pose_msg.pose.position.x = waypoint_x
                pose_msg.pose.position.y = waypoint_y
            else:
                pose_msg.pose.position.x = (
                    origin_x
                    + math.cos(origin_yaw) * waypoint_x
                    - math.sin(origin_yaw) * waypoint_y
                )
                pose_msg.pose.position.y = (
                    origin_y
                    + math.sin(origin_yaw) * waypoint_x
                    + math.cos(origin_yaw) * waypoint_y
                )

            target_yaw = self.corridor_waypoint_target_yaw(index)
            if target_yaw is not None:
                pose_yaw = target_yaw if self.corridor_waypoints_are_global else origin_yaw + target_yaw
            else:
                pose_yaw = start_yaw if self.corridor_waypoints_are_global and start_yaw is not None else origin_yaw

            orientation_z, orientation_w = self.yaw_to_quaternion(pose_yaw)
            pose_msg.pose.orientation.z = orientation_z
            pose_msg.pose.orientation.w = orientation_w
            path_msg.poses.append(pose_msg)

        self.corridor_path_pub.publish(path_msg)

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_frame_id = msg.header.frame_id
        self.front_obstacle_distance = self.sector_min_distance(
            msg,
            -self.detour_front_angle_deg,
            self.detour_front_angle_deg,
        )
        half_window = self.detour_side_window_deg / 2.0
        self.left_clearance_distance = self.sector_min_distance(
            msg,
            self.detour_side_center_deg - half_window,
            self.detour_side_center_deg + half_window,
        )
        self.right_clearance_distance = self.sector_min_distance(
            msg,
            -self.detour_side_center_deg - half_window,
            -self.detour_side_center_deg + half_window,
        )

    def sector_min_distance(self, scan_msg, min_angle_deg, max_angle_deg):
        min_distance = float('inf')
        for index, distance in enumerate(scan_msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance <= 0.0:
                continue

            angle_deg = math.degrees(scan_msg.angle_min + index * scan_msg.angle_increment)
            angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
            if angle_deg < min_angle_deg or angle_deg > max_angle_deg:
                continue

            if distance < min_distance:
                min_distance = distance

        return min_distance

    def phase_callback(self, msg):
        previous_phase = self.phase
        self.phase = int(msg.data)
        if previous_phase != self.phase and self.phase != 2:
            self.reset_mission(clear_task=False)
        self.try_start_mission()

    def task_callback(self, msg):
        self.task_raw = msg.data.strip()
        self.direction = self.parse_direction(self.task_raw)
        self.try_start_mission()

    def imu_callback(self, msg):
        self.current_yaw = self.quaternion_to_yaw(msg.orientation)
        self.try_start_mission()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.current_position = (float(position.x), float(position.y))
        self.current_odom_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_frame_id = msg.header.frame_id or self.odom_frame_id
        self.try_start_mission()

    def parse_direction(self, task_text):
        if not task_text:
            return None

        normalized = task_text.lower()
        for keyword in self.clockwise_keywords:
            if keyword and keyword in normalized:
                return 'clockwise'
        for keyword in self.counterclockwise_keywords:
            if keyword and keyword in normalized:
                return 'counterclockwise'

        try:
            numeric_value = int(task_text)
        except ValueError:
            return None

        is_odd = numeric_value % 2 == 1
        if self.odd_is_clockwise:
            return 'clockwise' if is_odd else 'counterclockwise'
        return 'counterclockwise' if is_odd else 'clockwise'

    def parse_pre_loop_plan(self):
        return self.parse_plan_json(self.pre_loop_plan_json, 'pre_loop_plan_json')

    def parse_post_corridor_path_plan(self):
        return self.parse_plan_json(
            self.post_corridor_path_plan_json,
            'post_corridor_path_plan_json',
        )

    def parse_plan_json(self, raw_json, param_name):
        try:
            raw_segments = json.loads(raw_json)
        except json.JSONDecodeError:
            self.get_logger().error(f'{param_name} is invalid, fallback to empty plan')
            return []

        if not isinstance(raw_segments, list):
            self.get_logger().error(f'{param_name} must decode to a list, fallback to empty plan')
            return []

        sanitized_segments = []
        for raw_segment in raw_segments:
            if not isinstance(raw_segment, dict):
                continue
            segment_type = raw_segment.get('type')
            if segment_type == 'turn':
                sanitized_segments.append({
                    'type': 'turn',
                    'angle_deg': float(raw_segment.get('angle_deg', 0.0)),
                    'description': raw_segment.get('description', 'pre_turn'),
                })
            elif segment_type == 'move':
                sanitized_segments.append({
                    'type': 'move',
                    'distance_m': max(0.0, float(raw_segment.get('distance_m', 0.0))),
                    'speed': float(raw_segment.get('speed', self.corridor_linear_speed)),
                    'description': raw_segment.get('description', 'pre_move'),
                    'allow_detour': bool(raw_segment.get('allow_detour', True)),
                })
            elif segment_type == 'pause':
                sanitized_segments.append({
                    'type': 'pause',
                    'duration': max(0.0, float(raw_segment.get('duration', 0.0))),
                    'description': raw_segment.get('description', 'pre_pause'),
                })
        return sanitized_segments

    def build_ring_plan(self):
        half_long = self.loop_long_length_m / 2.0
        initial_turn = 90.0 if self.direction == 'clockwise' else -90.0
        corner_turn = -initial_turn

        return [
            {'type': 'turn', 'angle_deg': initial_turn, 'description': 'loop_enter'},
            {'type': 'move', 'distance_m': half_long, 'speed': self.ring_linear_speed, 'description': 'loop_bottom_half_1', 'allow_detour': True},
            {'type': 'turn', 'angle_deg': corner_turn, 'description': 'corner_1'},
            {'type': 'move', 'distance_m': self.loop_short_length_m, 'speed': self.ring_linear_speed, 'description': 'loop_short_1', 'allow_detour': True},
            {'type': 'turn', 'angle_deg': corner_turn, 'description': 'corner_2'},
            {'type': 'move', 'distance_m': self.loop_long_length_m, 'speed': self.ring_linear_speed, 'description': 'loop_long_top', 'allow_detour': True},
            {'type': 'turn', 'angle_deg': corner_turn, 'description': 'corner_3'},
            {'type': 'move', 'distance_m': self.loop_short_length_m, 'speed': self.ring_linear_speed, 'description': 'loop_short_2', 'allow_detour': True},
            {'type': 'turn', 'angle_deg': corner_turn, 'description': 'corner_4'},
            {'type': 'move', 'distance_m': half_long, 'speed': self.ring_linear_speed, 'description': 'loop_bottom_half_2', 'allow_detour': True},
            {'type': 'turn', 'angle_deg': initial_turn, 'description': 'loop_exit_align'},
            {'type': 'move', 'distance_m': self.exit_distance_m, 'speed': self.corridor_linear_speed, 'description': 'corridor_exit', 'allow_detour': True},
        ]

    def current_segment_allows_detour(self):
        if not self.detour_enabled or self.current_segment is None:
            return False
        if self.current_segment.get('type') != 'move':
            return False
        if not bool(self.current_segment.get('allow_detour', True)):
            return False
        if bool(self.current_segment.get('is_detour', False)):
            return False
        current_time = self.get_clock().now().nanoseconds / 1e9
        return current_time >= self.detour_cooldown_until

    def select_detour_side(self):
        left_clear = self.left_clearance_distance
        right_clear = self.right_clearance_distance
        left_ok = math.isfinite(left_clear) and left_clear >= self.detour_min_side_clearance
        right_ok = math.isfinite(right_clear) and right_clear >= self.detour_min_side_clearance

        if left_ok and right_ok:
            return 'left' if left_clear >= right_clear else 'right'
        if left_ok:
            return 'left'
        if right_ok:
            return 'right'
        return None

    def build_detour_segments(self, side, forward_distance, resume_distance):
        side_sign = 1.0 if side == 'left' else -1.0
        turn_angle = 90.0 * side_sign
        segments = [
            {'type': 'turn', 'angle_deg': turn_angle, 'description': f'detour_{side}_shift_out_turn'},
            {
                'type': 'move',
                'distance_m': self.detour_lateral_distance_m,
                'speed': self.corridor_linear_speed,
                'description': f'detour_{side}_shift_out_move',
                'allow_detour': False,
                'is_detour': True,
            },
            {'type': 'turn', 'angle_deg': -turn_angle, 'description': f'detour_{side}_forward_align'},
            {
                'type': 'move',
                'distance_m': forward_distance,
                'speed': self.corridor_linear_speed,
                'description': f'detour_{side}_pass_obstacle',
                'allow_detour': False,
                'is_detour': True,
            },
            {'type': 'turn', 'angle_deg': -turn_angle, 'description': f'detour_{side}_return_turn'},
            {
                'type': 'move',
                'distance_m': self.detour_lateral_distance_m,
                'speed': self.corridor_linear_speed,
                'description': f'detour_{side}_return_move',
                'allow_detour': False,
                'is_detour': True,
            },
            {'type': 'turn', 'angle_deg': turn_angle, 'description': f'detour_{side}_resume_align'},
        ]
        if resume_distance > self.distance_tolerance:
            segments.append({
                'type': 'move',
                'distance_m': resume_distance,
                'speed': float(self.current_segment.get('speed', self.corridor_linear_speed)),
                'description': f'{self.current_segment.get("description", "segment")}_resume',
                'allow_detour': False,
            })
        return segments

    def maybe_inject_detour(self):
        if not self.current_segment_allows_detour():
            return False
        if not math.isfinite(self.front_obstacle_distance) or self.front_obstacle_distance > self.detour_obstacle_distance:
            return False

        side = self.select_detour_side()
        if side is None:
            self.publish_state('detour_waiting')
            self.cmd_pub.publish(Twist())
            return True

        progress = self.projected_distance()
        target_distance = float(self.current_segment['distance_m'])
        remaining_distance = max(0.0, target_distance - progress)
        if remaining_distance <= self.distance_tolerance:
            return False

        forward_distance = min(self.detour_forward_distance_m, remaining_distance)
        resume_distance = max(0.0, remaining_distance - forward_distance)
        detour_segments = self.build_detour_segments(side, forward_distance, resume_distance)
        self.plan = self.plan[:self.plan_index] + detour_segments + self.plan[self.plan_index + 1:]
        self.detour_cooldown_until = self.get_clock().now().nanoseconds / 1e9 + self.detour_cooldown_sec
        self.publish_feedback(
            f'检测到通道障碍，向{"左" if side == "left" else "右"}侧绿色区域短暂绕行'
        )
        self.start_segment(self.plan_index)
        return True

    def try_start_mission(self):
        if self.mission_active or self.mission_finished:
            return
        if self.phase != 2 or self.direction is None:
            return
        if self.current_yaw is None or self.current_position is None:
            return

        if self.start_after_time is None:
            self.start_after_time = self.get_clock().now().nanoseconds / 1e9 + self.start_delay_sec
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time < self.start_after_time:
            return

        self.mission_active = True
        self.reported_start = True
        if self.use_corridor_path and self.corridor_waypoints:
            self.publish_feedback(f'第二阶段启动，方向: {self.direction}，先沿固定航点路径到通道入口')
            self.start_corridor_path()
            return

        self.publish_feedback(f'第二阶段启动，方向: {self.direction}')
        self.begin_inertial_plan_after_nav(nav_succeeded=False)

    def build_inertial_plan(self, nav_succeeded):
        pre_loop_plan = self.parse_pre_loop_plan()
        if nav_succeeded and self.corridor_path_skip_pre_loop_plan:
            pre_loop_plan = self.parse_post_corridor_path_plan()
        return pre_loop_plan + self.build_ring_plan()

    def begin_inertial_plan_after_nav(self, nav_succeeded):
        self.reset_corridor_path_state()
        self.pending_segment_start_pose = self.current_position
        self.pending_segment_start_yaw = self.current_yaw

        self.plan = self.build_inertial_plan(nav_succeeded)
        if not self.plan:
            self.publish_feedback('第二阶段没有可执行段，直接结束')
            self.finish_mission()
            return

        self.start_segment(0)

    def reset_corridor_path_state(self):
        self.corridor_path_active = False
        self.corridor_path_index = 0
        self.corridor_path_origin_pose = None
        self.corridor_path_origin_yaw = None
        self.corridor_path_started_at = None
        self.last_corridor_plan_points = []
        self.last_corridor_plan_signature = None
        self.last_corridor_plan_at = 0.0

    def start_corridor_path(self):
        if self.current_position is None or self.current_yaw is None:
            return

        self.reset_corridor_path_state()
        self.corridor_path_active = True
        if self.corridor_waypoints_are_global:
            self.corridor_path_origin_pose = None
            self.corridor_path_origin_yaw = None
        else:
            self.corridor_path_origin_pose = self.current_position
            self.corridor_path_origin_yaw = self.current_yaw
        self.corridor_path_started_at = self.get_clock().now().nanoseconds / 1e9
        self.publish_corridor_path()
        self.publish_state('corridor_path_running')
        self.publish_feedback(f'固定航点路径已启动，共 {len(self.corridor_waypoints)} 个航点')

    def corridor_local_pose(self):
        if self.corridor_waypoints_are_global:
            return self.current_global_position()
        if self.current_position is None or self.corridor_path_origin_pose is None or self.corridor_path_origin_yaw is None:
            return None

        dx = self.current_position[0] - self.corridor_path_origin_pose[0]
        dy = self.current_position[1] - self.corridor_path_origin_pose[1]
        x_local = math.cos(self.corridor_path_origin_yaw) * dx + math.sin(self.corridor_path_origin_yaw) * dy
        y_local = -math.sin(self.corridor_path_origin_yaw) * dx + math.cos(self.corridor_path_origin_yaw) * dy
        return (x_local, y_local)

    def corridor_local_yaw(self):
        if self.corridor_waypoints_are_global:
            return self.current_global_yaw()
        if self.current_yaw is None or self.corridor_path_origin_yaw is None:
            return None
        return self.normalize_angle(self.current_yaw - self.corridor_path_origin_yaw)

    def maybe_advance_corridor_waypoint(self, local_pose):
        while self.corridor_path_index < len(self.corridor_waypoints) - 1:
            waypoint = self.corridor_waypoints[self.corridor_path_index]
            distance = math.hypot(waypoint['x'] - local_pose[0], waypoint['y'] - local_pose[1])
            if distance > self.corridor_waypoint_tolerance:
                return
            self.corridor_path_index += 1

    def finish_corridor_path(self):
        self.cmd_pub.publish(Twist())
        self.publish_feedback('固定航点路径到达通道入口，切换惯导绕圈')
        self.begin_inertial_plan_after_nav(nav_succeeded=True)

    def reset_mission(self, clear_task):
        self.cmd_pub.publish(Twist())
        self.plan = []
        self.plan_index = -1
        self.current_segment = None
        self.segment_started_at = None
        self.segment_start_pose = None
        self.segment_start_yaw = None
        self.segment_target_yaw = None
        self.segment_heading = None
        self.start_after_time = None
        self.mission_active = False
        self.mission_finished = False
        self.reported_start = False
        self.detour_cooldown_until = 0.0
        self.pending_segment_start_pose = None
        self.pending_segment_start_yaw = None
        self.reset_corridor_path_state()
        self.clear_corridor_path()
        self.publish_state('idle')
        if clear_task:
            self.task_raw = ''
            self.direction = None

    def start_segment(self, index):
        if index >= len(self.plan):
            self.finish_mission()
            return

        self.plan_index = index
        self.current_segment = self.plan[index]
        self.segment_started_at = self.get_clock().now().nanoseconds / 1e9
        self.segment_start_pose = self.pending_segment_start_pose if self.pending_segment_start_pose is not None else self.current_position
        self.segment_start_yaw = self.pending_segment_start_yaw if self.pending_segment_start_yaw is not None else self.current_yaw
        self.pending_segment_start_pose = None
        self.pending_segment_start_yaw = None

        if self.current_segment['type'] == 'turn':
            self.segment_target_yaw = self.normalize_angle(
                self.segment_start_yaw + math.radians(self.current_segment['angle_deg'])
            )
            self.publish_state(self.current_segment['description'])
        elif self.current_segment['type'] == 'move':
            self.segment_heading = self.segment_start_yaw
            self.publish_state(self.current_segment['description'])
        elif self.current_segment['type'] == 'pause':
            self.publish_state(self.current_segment['description'])

    def finish_mission(self):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self.reset_corridor_path_state()
        self.publish_state('complete')
        self.publish_feedback('第二阶段完成，已离开环形通道并到达出口')

    def fail_mission(self, reason):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self.reset_corridor_path_state()
        self.publish_state('failed')
        self.publish_feedback(reason)

    def projected_distance(self):
        if self.segment_start_pose is None or self.current_position is None or self.segment_heading is None:
            return 0.0
        dx = self.current_position[0] - self.segment_start_pose[0]
        dy = self.current_position[1] - self.segment_start_pose[1]
        return dx * math.cos(self.segment_heading) + dy * math.sin(self.segment_heading)

    def control_loop(self):
        if self.corridor_path_active:
            self.run_corridor_path_stage()
            return

        if not self.mission_active or self.current_segment is None:
            if not self.mission_active:
                self.cmd_pub.publish(Twist())
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.segment_started_at is not None and now_sec - self.segment_started_at > self.segment_timeout:
            self.publish_feedback(f'段超时，强制切换: {self.current_segment.get("description", "unknown")}')
            self.start_segment(self.plan_index + 1)
            return

        segment_type = self.current_segment['type']
        if segment_type == 'turn':
            self.run_turn_segment()
            return
        if segment_type == 'move':
            self.run_move_segment()
            return
        if segment_type == 'pause':
            self.run_pause_segment(now_sec)
            return

        self.start_segment(self.plan_index + 1)

    def run_turn_segment(self):
        if self.current_yaw is None or self.segment_target_yaw is None:
            self.cmd_pub.publish(Twist())
            return

        error = self.angle_error(self.segment_target_yaw, self.current_yaw)
        if abs(error) <= self.heading_tolerance:
            self.cmd_pub.publish(Twist())
            self.start_segment(self.plan_index + 1)
            return

        angular = self.clamp(self.turn_kp * error, self.turn_angular_speed)
        if abs(error) > self.heading_tolerance and abs(angular) < self.turn_min_angular_speed:
            angular = math.copysign(self.turn_min_angular_speed, error)

        self.cmd_pub.publish(self.create_twist(self.turn_linear_speed, angular))

    def run_move_segment(self):
        if self.current_position is None or self.segment_heading is None:
            self.cmd_pub.publish(Twist())
            return

        if self.maybe_inject_detour():
            return

        progress = self.projected_distance()
        target_distance = float(self.current_segment['distance_m'])
        if progress >= target_distance - self.distance_tolerance:
            self.cmd_pub.publish(Twist())
            self.start_segment(self.plan_index + 1)
            return

        heading_error = 0.0 if self.current_yaw is None else self.angle_error(self.segment_heading, self.current_yaw)
        angular = self.clamp(self.heading_kp * heading_error, self.max_angular_speed)
        linear = float(self.current_segment.get('speed', self.corridor_linear_speed))
        self.cmd_pub.publish(self.create_twist(linear, angular))

    def run_pause_segment(self, now_sec):
        duration = float(self.current_segment.get('duration', 0.0))
        self.cmd_pub.publish(Twist())
        if self.segment_started_at is not None and now_sec - self.segment_started_at >= duration:
            self.start_segment(self.plan_index + 1)

    def run_corridor_path_stage(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9

        if (
            self.corridor_path_started_at is not None
            and now_sec - self.corridor_path_started_at > self.corridor_path_timeout_sec
        ):
            self.fail_mission('固定航点路径超时，阶段二停止')
            return

        local_pose = self.corridor_local_pose()
        local_yaw = self.corridor_local_yaw()
        if local_pose is None or local_yaw is None:
            self.cmd_pub.publish(Twist())
            return

        self.maybe_advance_corridor_waypoint(local_pose)
        final_waypoint = self.corridor_waypoints[-1]
        final_target_x, final_target_y = self.corridor_waypoint_target_position(len(self.corridor_waypoints) - 1)
        final_distance = math.hypot(final_target_x - local_pose[0], final_target_y - local_pose[1])
        if self.corridor_path_index >= len(self.corridor_waypoints) - 1 and final_distance <= self.corridor_goal_tolerance:
            target_yaw = self.corridor_waypoint_target_yaw(len(self.corridor_waypoints) - 1)
            if target_yaw is not None:
                yaw_error = self.angle_error(target_yaw, local_yaw)
                if abs(yaw_error) > self.corridor_goal_yaw_tolerance:
                    angular = self.clamp(self.pure_pursuit_turn_kp * yaw_error, self.turn_angular_speed)
                    if abs(angular) < self.turn_min_angular_speed:
                        angular = math.copysign(self.turn_min_angular_speed, yaw_error)
                    align_linear = min(self.turn_linear_speed, 0.04)
                    self.publish_state('corridor_path_align')
                    self.cmd_pub.publish(self.create_twist(align_linear, angular))
                    return

            self.finish_corridor_path()
            return

        target_waypoint = self.corridor_waypoints[self.corridor_path_index]
        target_x, target_y = self.corridor_waypoint_target_position(self.corridor_path_index)
        if self.use_occupancy_grid_planner and self.corridor_waypoints_are_global:
            planned_points = self.plan_corridor_path(local_pose, (target_x, target_y), now_sec)
            if planned_points is None:
                self.publish_state('planner_waiting_for_map')
                self.cmd_pub.publish(Twist())
                return
            if not planned_points:
                self.publish_state('corridor_planner_blocked')
                self.cmd_pub.publish(Twist())
                return

            self.publish_path_points(planned_points, self.global_frame_id)
            lookahead_point = self.select_path_lookahead_point(
                planned_points,
                self.pure_pursuit_lookahead_m,
            )
            if lookahead_point is not None:
                target_x, target_y = lookahead_point

        target_dx = target_x - local_pose[0]
        target_dy = target_y - local_pose[1]
        target_x_robot = math.cos(local_yaw) * target_dx + math.sin(local_yaw) * target_dy
        target_y_robot = -math.sin(local_yaw) * target_dx + math.cos(local_yaw) * target_dy
        target_distance = math.hypot(target_x_robot, target_y_robot)
        heading_error = math.atan2(target_y_robot, target_x_robot if abs(target_x_robot) > 1e-6 else 1e-6)

        self.publish_state(target_waypoint['description'])
        if target_x_robot <= 0.0:
            angular = self.clamp(self.pure_pursuit_turn_kp * heading_error, self.turn_angular_speed)
            if abs(angular) < self.turn_min_angular_speed:
                angular = math.copysign(self.turn_min_angular_speed, heading_error)
            self.cmd_pub.publish(self.create_twist(min(self.turn_linear_speed, 0.04), angular))
            return

        if abs(heading_error) > self.pure_pursuit_heading_stop:
            angular = self.clamp(self.pure_pursuit_turn_kp * heading_error, self.turn_angular_speed)
            if abs(angular) < self.turn_min_angular_speed:
                angular = math.copysign(self.turn_min_angular_speed, heading_error)
            self.cmd_pub.publish(self.create_twist(self.turn_linear_speed, angular))
            return

        pursuit_distance = max(target_distance, self.pure_pursuit_lookahead_m)
        curvature = 0.0 if pursuit_distance <= 1e-6 else 2.0 * target_y_robot / (pursuit_distance * pursuit_distance)
        linear_speed = min(float(target_waypoint.get('speed', self.corridor_linear_speed)), self.corridor_linear_speed)
        if target_distance < self.pure_pursuit_lookahead_m:
            linear_speed *= max(0.4, target_distance / max(self.pure_pursuit_lookahead_m, 1e-6))
        angular_speed = self.clamp(linear_speed * curvature, self.max_angular_speed)
        self.cmd_pub.publish(self.create_twist(linear_speed, angular_speed))


def main(args=None):
    rclpy.init(args=args)
    node = Stage2InertialNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

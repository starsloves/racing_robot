"""Stage3 官方返程导航：通道对中 + 地图粗导航 + P 视觉最终到达 + Stage1 4态避障
- 状态机: idle → armed → pre_return_channel_yolo → running(map_search_p) → p_approach → complete
- running 时可中断：avoiding → countersteer → recovering → running
- 仅在 competition_phase=3 时启动
- 输出 /cmd_vel（phase3 由 Stage1 礼让）
"""

import json
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from racing_common.racing_logger import RacingLogger
from racing_common.yolo_bbox_detector import YoloBBoxDetector
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32, String

from .cmd_vel_stop import (
    emergency_cli_stop_async,
    init_without_ros_signal_handler,
    install_stop_event,
    publish_stop,
    spin_until_stop,
)
from .vision_p_detector import VisionPDetector


class Stage3ReturnNavigator(Node):
    def __init__(self):
        super().__init__('stage3_return_navigator')

        self._declare_params()
        self._read_params()

        # ── 日志文件 ~/dev_ws/log/enhanced_return_test/latest.log ──
        self.log = RacingLogger(
            self, log_subdir='competition_stage3',
            log_filename='latest.log', session_title='Stage3 return navigator',
        )

        # ── 路点（map 全局坐标系）──
        self.return_waypoints = self._parse_waypoints_json(
            self.return_waypoints_json, 'return_waypoints_json',
            self.pursuit_linear_speed,
        )

        # ── 状态 ──
        self.phase = 1
        self.phase_initialized = False
        self.mission_active = False
        self.mission_finished = False
        self.start_after_time = None

        # 位姿（odom_combined，map 坐标系）
        self.current_position = None
        self.current_yaw = None
        self.odom_frame_id = 'odom'
        self._last_raw_odom_xy = None
        self._last_raw_odom_yaw = None
        self._imu_yaw = None
        self._imu_yaw_offset = 0.0

        # 路径状态
        self.path_started_at = None
        self.path_index = 0
        self._settled_start = None
        self._filtered_heading_err = 0.0

        # P 视觉最终接管：地图只用于粗导航，P 视觉决定最终到达。
        self._p_detector = None
        self._p_approaching = False
        self._p_consecutive_hits = 0
        self._p_offset_filtered = 0.0

        # 激光扫描
        self.latest_scan = None

        # ── 避障状态 ──
        self.avoid_state = 'forward'
        self.avoid_turn_direction = 0.0
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None

        # ── Stage3 前置通道 YOLO 重定位 ──
        self._pre_return_state = 'idle'
        self._pre_return_started_at = None
        self._channel_detector = None
        self._channel_hits = 0
        self._channel_offset_filtered = 0.0

        # ── Pub/Sub ──
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, qos_latched)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)

        self.create_subscription(Int32, self.phase_topic, self._phase_cb, qos_latched)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)

        self._init_p_detector()
        self._init_channel_detector()

        self._publish_state('idle')
        self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.log.startup(
            f'enhanced return navigator ready | waypoints={len(self.return_waypoints)} '
            f'cmd={self.cmd_topic} odom={self.odom_topic}'
        )

    # ══════════════ 参数 ══════════════

    def _declare_params(self):
        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('state_topic', 'stage3_state')
        self.declare_parameter('feedback_topic', 'competition_feedback')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('start_delay_sec', 0.5)

        # ── 路点 ──
        self.declare_parameter('return_waypoints_json', '[]')
        self.declare_parameter('waypoint_tolerance', 0.18)
        self.declare_parameter('goal_box_x_min', 0.1)
        self.declare_parameter('goal_box_x_max', 0.3)
        self.declare_parameter('goal_box_y_min', 0.1)
        self.declare_parameter('goal_box_y_max', 0.2)
        self.declare_parameter('goal_center_stop_distance_m', 0.10)
        self.declare_parameter('path_timeout_sec', 60.0)

        # ── P 点视觉最终到达 ──
        self.declare_parameter('p_model_path', '')
        self.declare_parameter('p_conf_thres', 0.25)
        self.declare_parameter('p_iou_thres', 0.45)
        self.declare_parameter('p_crop_ratio', 0.4)
        self.declare_parameter('p_approach_conf_threshold', 0.5)
        self.declare_parameter('p_approach_consecutive_hits', 3)
        self.declare_parameter('p_approach_linear_speed', 0.50)
        self.declare_parameter('p_approach_angular_kp', 0.8)
        self.declare_parameter('p_web_port', 8083)
        self.declare_parameter('p_detection_timeout_sec', 0.35)

        # ── Pure Pursuit ──
        self.declare_parameter('pursuit_linear_speed', 0.18)
        self.declare_parameter('pursuit_lookahead_m', 0.45)
        self.declare_parameter('pursuit_heading_stop_deg', 70.0)
        self.declare_parameter('pursuit_turn_kp', 1.8)
        self.declare_parameter('pursuit_turn_linear_speed', 0.08)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('min_angular_speed', 0.45)

        # ── 避障状态（同 Stage1）──
        self.declare_parameter('avoid_linear_speed', 0.10)
        self.declare_parameter('avoid_angular_speed', 0.80)
        self.declare_parameter('avoid_min_duration_sec', 0.70)
        self.declare_parameter('avoid_clear_hold_sec', 0.25)
        self.declare_parameter('avoid_min_turn_angle_deg', 18.0)
        self.declare_parameter('avoid_safe_distance', 0.50)
        self.declare_parameter('avoid_clear_distance', 0.65)
        self.declare_parameter('emergency_stop_distance', 0.22)

        self.declare_parameter('recovery_linear_speed', 0.12)
        self.declare_parameter('recovery_turn_linear_speed', 0.08)
        self.declare_parameter('recovery_angular_speed', 0.75)
        self.declare_parameter('recovery_heading_kp', 2.4)
        self.declare_parameter('recovery_max_angular_speed', 1.1)
        self.declare_parameter('recovery_min_angular_speed', 0.5)
        self.declare_parameter('recovery_in_place_angle_deg', 8.0)
        self.declare_parameter('recovery_timeout', 2.5)
        self.declare_parameter('recovery_duration_scale', 0.9)

        self.declare_parameter('counter_steer_linear_speed', 0.10)
        self.declare_parameter('counter_steer_angular_speed', 0.95)
        self.declare_parameter('counter_steer_duration_scale', 1.35)
        self.declare_parameter('counter_steer_min_duration_sec', 0.45)
        self.declare_parameter('counter_steer_max_duration_sec', 1.20)

        # ── 激光聚类窗口（避障用）──
        self.declare_parameter('window_min_x', 0.18)
        self.declare_parameter('window_max_x', 0.85)
        self.declare_parameter('window_half_width', 0.22)
        self.declare_parameter('cluster_gap_tolerance', 0.12)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('min_cluster_width', 0.06)
        self.declare_parameter('max_cluster_width', 0.40)
        self.declare_parameter('min_valid_range', 0.15)

        # ── A* 全局路径规划（避开地图禁区）──
        self.declare_parameter('use_global_planner', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('planner_downsample', 4)
        self.declare_parameter('planner_occupied_threshold', 50)
        self.declare_parameter('planner_unknown_is_occupied', False)
        self.declare_parameter('planner_obstacle_inflation_m', 0.14)
        self.declare_parameter('planner_dynamic_obstacle_box_size_m', 0.25)
        self.declare_parameter('planner_dynamic_obstacle_inflation_m', 0.04)
        self.declare_parameter('planner_dynamic_obstacle_range_m', 0.7)
        self.declare_parameter('planner_replan_period_sec', 0.25)

        # ── map→odom 偏移参数（同 Stage2 map_overlay，直接传参避免 TF 依赖）──
        self.declare_parameter('test_direction', 'clockwise')
        self.declare_parameter('map_to_odom_x', 0.0)
        self.declare_parameter('map_to_odom_y', 0.0)
        self.declare_parameter('map_to_odom_yaw', 0.0)

        # ── Stage3 前置通道 YOLO 对中 + 内部 map 初始值重置 ──
        self.declare_parameter('stage3_channel_yolo_enabled', True)
        self.declare_parameter('stage3_channel_model_path', '/home/sunrise/dev_ws/best_rdk_tongdao.bin')
        self.declare_parameter('stage3_channel_camera_topic', '/aurora/rgb/image_raw')
        self.declare_parameter('stage3_channel_conf_thres', 0.25)
        self.declare_parameter('stage3_channel_iou_thres', 0.45)
        self.declare_parameter('stage3_channel_preview_path', '/tmp/stage3_channel_yolo.jpg')
        self.declare_parameter('stage3_channel_yaw_deg', -90.0)
        self.declare_parameter('stage3_channel_yaw_tolerance_deg', 5.0)
        self.declare_parameter('stage3_channel_reset_map_x', 2.5)
        self.declare_parameter('stage3_channel_reset_map_y', 2.5)
        self.declare_parameter('stage3_channel_reset_yaw_deg', -90.0)
        self.declare_parameter('stage3_channel_linear_speed', 0.10)
        self.declare_parameter('stage3_channel_angular_kp', 0.55)
        self.declare_parameter('stage3_channel_max_angular', 0.25)
        self.declare_parameter('stage3_channel_offset_deadband', 0.05)
        self.declare_parameter('stage3_channel_offset_tolerance', 0.08)
        self.declare_parameter('stage3_channel_fill_ratio', 0.20)
        self.declare_parameter('stage3_channel_consecutive_hits', 3)
        self.declare_parameter('stage3_channel_timeout_sec', 14.0)

    def _read_params(self):
        self.phase_topic = str(self.get_parameter('phase_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.feedback_topic = str(self.get_parameter('feedback_topic').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)

        self.return_waypoints_json = self.get_parameter('return_waypoints_json').value
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.goal_box_x_min = float(self.get_parameter('goal_box_x_min').value)
        self.goal_box_x_max = float(self.get_parameter('goal_box_x_max').value)
        self.goal_box_y_min = float(self.get_parameter('goal_box_y_min').value)
        self.goal_box_y_max = float(self.get_parameter('goal_box_y_max').value)
        self.goal_center_stop_distance_m = max(
            0.0, float(self.get_parameter('goal_center_stop_distance_m').value)
        )
        self.path_timeout_sec = float(self.get_parameter('path_timeout_sec').value)

        self.p_model_path = str(self.get_parameter('p_model_path').value)
        self.p_conf_thres = float(self.get_parameter('p_conf_thres').value)
        self.p_iou_thres = float(self.get_parameter('p_iou_thres').value)
        self.p_crop_ratio = float(self.get_parameter('p_crop_ratio').value)
        self.p_approach_conf = float(self.get_parameter('p_approach_conf_threshold').value)
        self.p_approach_hits_required = max(
            1, int(self.get_parameter('p_approach_consecutive_hits').value)
        )
        self.p_approach_linear = float(self.get_parameter('p_approach_linear_speed').value)
        self.p_approach_angular_kp = float(self.get_parameter('p_approach_angular_kp').value)
        self.p_web_port = int(self.get_parameter('p_web_port').value)
        self.p_detection_timeout = max(
            0.0, float(self.get_parameter('p_detection_timeout_sec').value)
        )

        self.pursuit_linear_speed = float(self.get_parameter('pursuit_linear_speed').value)
        self.pursuit_lookahead = float(self.get_parameter('pursuit_lookahead_m').value)
        self.pursuit_heading_stop = math.radians(float(self.get_parameter('pursuit_heading_stop_deg').value))
        self.pursuit_turn_kp = float(self.get_parameter('pursuit_turn_kp').value)
        self.pursuit_turn_linear = float(self.get_parameter('pursuit_turn_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.min_angular = float(self.get_parameter('min_angular_speed').value)

        self.avoid_linear_speed = float(self.get_parameter('avoid_linear_speed').value)
        self.avoid_angular_speed = float(self.get_parameter('avoid_angular_speed').value)
        self.avoid_min_duration = float(self.get_parameter('avoid_min_duration_sec').value)
        self.avoid_clear_hold = float(self.get_parameter('avoid_clear_hold_sec').value)
        self.avoid_min_turn_angle = math.radians(float(self.get_parameter('avoid_min_turn_angle_deg').value))
        self.avoid_safe_dist = float(self.get_parameter('avoid_safe_distance').value)
        self.avoid_clear_dist = float(self.get_parameter('avoid_clear_distance').value)
        self.emergency_stop_dist = float(self.get_parameter('emergency_stop_distance').value)

        self.recovery_linear = float(self.get_parameter('recovery_linear_speed').value)
        self.recovery_turn_linear = float(self.get_parameter('recovery_turn_linear_speed').value)
        self.recovery_angular = float(self.get_parameter('recovery_angular_speed').value)
        self.recovery_kp = float(self.get_parameter('recovery_heading_kp').value)
        self.recovery_max_angular = float(self.get_parameter('recovery_max_angular_speed').value)
        self.recovery_min_angular = float(self.get_parameter('recovery_min_angular_speed').value)
        self.recovery_in_place = math.radians(float(self.get_parameter('recovery_in_place_angle_deg').value))
        self.recovery_timeout = float(self.get_parameter('recovery_timeout').value)
        self.recovery_duration_scale = float(self.get_parameter('recovery_duration_scale').value)

        self.counter_linear = float(self.get_parameter('counter_steer_linear_speed').value)
        self.counter_angular = float(self.get_parameter('counter_steer_angular_speed').value)
        self.counter_duration_scale = float(self.get_parameter('counter_steer_duration_scale').value)
        self.counter_min_dur = float(self.get_parameter('counter_steer_min_duration_sec').value)
        self.counter_max_dur = float(self.get_parameter('counter_steer_max_duration_sec').value)

        self.window_min_x = float(self.get_parameter('window_min_x').value)
        self.window_max_x = float(self.get_parameter('window_max_x').value)
        self.window_half_width = float(self.get_parameter('window_half_width').value)
        self.cluster_gap = float(self.get_parameter('cluster_gap_tolerance').value)
        self.min_cluster_pts = int(self.get_parameter('min_cluster_points').value)
        self.min_cluster_w = float(self.get_parameter('min_cluster_width').value)
        self.max_cluster_w = float(self.get_parameter('max_cluster_width').value)
        self.min_range = float(self.get_parameter('min_valid_range').value)

        self.use_global_planner = bool(self.get_parameter('use_global_planner').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.global_frame_id = str(self.get_parameter('global_frame_id').value)
        self.planner_downsample = int(self.get_parameter('planner_downsample').value)
        self.planner_occupied_threshold = int(self.get_parameter('planner_occupied_threshold').value)
        self.planner_unknown_is_occupied = bool(self.get_parameter('planner_unknown_is_occupied').value)
        self.planner_obstacle_inflation_m = float(self.get_parameter('planner_obstacle_inflation_m').value)
        self.planner_dynamic_obstacle_box_size_m = float(self.get_parameter('planner_dynamic_obstacle_box_size_m').value)
        self.planner_dynamic_obstacle_inflation_m = float(self.get_parameter('planner_dynamic_obstacle_inflation_m').value)
        self.planner_dynamic_obstacle_range_m = float(self.get_parameter('planner_dynamic_obstacle_range_m').value)
        self.planner_replan_period_sec = float(self.get_parameter('planner_replan_period_sec').value)

        self.test_direction = str(self.get_parameter('test_direction').value)
        self.map_odom_x = float(self.get_parameter('map_to_odom_x').value)
        self.map_odom_y = float(self.get_parameter('map_to_odom_y').value)
        self.map_odom_yaw = float(self.get_parameter('map_to_odom_yaw').value)
        self._default_map_odom_x = self.map_odom_x
        self._default_map_odom_y = self.map_odom_y
        self._default_map_odom_yaw = self.map_odom_yaw

        self.stage3_channel_yolo_enabled = bool(self.get_parameter('stage3_channel_yolo_enabled').value)
        self.stage3_channel_model_path = str(self.get_parameter('stage3_channel_model_path').value)
        self.stage3_channel_camera_topic = str(self.get_parameter('stage3_channel_camera_topic').value)
        self.stage3_channel_conf_thres = float(self.get_parameter('stage3_channel_conf_thres').value)
        self.stage3_channel_iou_thres = float(self.get_parameter('stage3_channel_iou_thres').value)
        self.stage3_channel_preview_path = str(self.get_parameter('stage3_channel_preview_path').value)
        self.stage3_channel_yaw = math.radians(float(self.get_parameter('stage3_channel_yaw_deg').value))
        self.stage3_channel_yaw_tolerance = math.radians(
            float(self.get_parameter('stage3_channel_yaw_tolerance_deg').value)
        )
        self.stage3_channel_reset_map_x = float(self.get_parameter('stage3_channel_reset_map_x').value)
        self.stage3_channel_reset_map_y = float(self.get_parameter('stage3_channel_reset_map_y').value)
        self.stage3_channel_reset_yaw = math.radians(
            float(self.get_parameter('stage3_channel_reset_yaw_deg').value)
        )
        self.stage3_channel_linear_speed = float(self.get_parameter('stage3_channel_linear_speed').value)
        self.stage3_channel_angular_kp = float(self.get_parameter('stage3_channel_angular_kp').value)
        self.stage3_channel_max_angular = float(self.get_parameter('stage3_channel_max_angular').value)
        self.stage3_channel_offset_deadband = float(self.get_parameter('stage3_channel_offset_deadband').value)
        self.stage3_channel_offset_tolerance = float(self.get_parameter('stage3_channel_offset_tolerance').value)
        self.stage3_channel_fill_ratio = float(self.get_parameter('stage3_channel_fill_ratio').value)
        self.stage3_channel_consecutive_hits = max(
            1, int(self.get_parameter('stage3_channel_consecutive_hits').value)
        )
        self.stage3_channel_timeout_sec = float(self.get_parameter('stage3_channel_timeout_sec').value)

    # ══════════════ 工具 ══════════════

    def _in_goal_region(self, x, y):
        """判断 (x,y) 是否在目标矩形区域内"""
        return (self.goal_box_x_min <= x <= self.goal_box_x_max and
                self.goal_box_y_min <= y <= self.goal_box_y_max)

    def _goal_center(self):
        """返回 P 矩形区域中心点（map 坐标系）。"""
        return (
            (self.goal_box_x_min + self.goal_box_x_max) / 2.0,
            (self.goal_box_y_min + self.goal_box_y_max) / 2.0,
        )

    def _distance_to_goal_center(self):
        if self.current_position is None:
            return None
        goal_x, goal_y = self._goal_center()
        return math.hypot(
            self.current_position[0] - goal_x,
            self.current_position[1] - goal_y,
        )

    def _check_goal_center_stop(self):
        """到 P 矩形中心前按配置提前停车并完成 Stage3。"""
        dist = self._distance_to_goal_center()
        if dist is None:
            return False
        if dist > self.goal_center_stop_distance_m:
            return False
        goal_x, goal_y = self._goal_center()
        self.log.segment(
            f'goal center early stop: dist={dist:.2f}/'
            f'{self.goal_center_stop_distance_m:.2f}m '
            f'center=({goal_x:.2f},{goal_y:.2f}) '
            f'pos=({self.current_position[0]:.2f},{self.current_position[1]:.2f})'
        )
        self._finish_mission(
            f'return complete, stopped {dist:.2f}m from P center '
            f'(target early stop {self.goal_center_stop_distance_m:.2f}m)'
        )
        return True

    @staticmethod
    def _normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _angle_error(target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    @staticmethod
    def _clamp(v, limit):
        return max(-limit, min(limit, v))

    @staticmethod
    def _twist(linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        return t

    @staticmethod
    def _quat_to_yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _select_lookahead_point(path_points, lookahead_distance):
        """
        从 A* 路径中选择预瞄点（距离起点 lookahead_distance 处）

        Args:
            path_points: list of tuple(x, y), 路径点列表
            lookahead_distance: float, 预瞄距离（m）
        Returns:
            tuple(x, y): 预瞄点坐标，如果路径太短则返回终点
        """
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

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_feedback(self, text):
        self.feedback_pub.publish(String(data=text))
        self.log.feedback(text)
        self.get_logger().info(text)

    def _publish_state(self, text):
        self.state_pub.publish(String(data=text))

    def _parse_waypoints_json(self, raw, param_name, default_speed):
        try:
            data = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().error(f'{param_name} invalid, empty waypoints')
            return []
        if not isinstance(data, list):
            return []
        wps = []
        for i, item in enumerate(data):
            if not isinstance(item, dict):
                continue
            yaw_d = item.get('yaw_deg')
            wps.append({
                'x': float(item.get('x', 0.0)),
                'y': float(item.get('y', 0.0)),
                'speed': float(item.get('speed', default_speed)),
                'yaw_deg': None if yaw_d is None else float(yaw_d),
                'desc': str(item.get('description', f'wp_{i}')),
            })
        return wps

    def _init_p_detector(self):
        if not self.p_model_path:
            self.log.startup('P model path not set, P detection disabled')
            return
        import os
        if not os.path.exists(self.p_model_path):
            self.log.warn('P_MODEL', f'model not found: {self.p_model_path}, P detection disabled')
            return
        self._p_detector = VisionPDetector(
            self, self.p_model_path,
            conf_thres=self.p_conf_thres,
            iou_thres=self.p_iou_thres,
            crop_ratio=self.p_crop_ratio,
            http_port=self.p_web_port,
        )
        self._set_p_inference_active(False)
        self.log.startup(
            f'P detector enabled, model={self.p_model_path}, '
            f'HTTP port={self.p_web_port}, endpoint=/vision_latest.jpg'
        )

    def _init_channel_detector(self):
        if not self.stage3_channel_yolo_enabled:
            self.log.startup('Stage3 channel YOLO pre-return disabled')
            return
        import os
        if not os.path.exists(self.stage3_channel_model_path):
            self.stage3_channel_yolo_enabled = False
            self.log.warn('CHANNEL_YOLO', f'model not found: {self.stage3_channel_model_path}')
            return
        try:
            self._channel_detector = YoloBBoxDetector(
                self,
                model_path=self.stage3_channel_model_path,
                camera_topic=self.stage3_channel_camera_topic,
                target_name='stage3_channel',
                conf_thres=self.stage3_channel_conf_thres,
                iou_thres=self.stage3_channel_iou_thres,
                jpeg_output_path=self.stage3_channel_preview_path,
                http_port=0,
            )
            self._set_channel_inference_active(False)
            self.log.startup(
                f'Stage3 channel YOLO enabled model={self.stage3_channel_model_path} '
                f'reset=({self.stage3_channel_reset_map_x:.2f},{self.stage3_channel_reset_map_y:.2f})'
            )
        except Exception as e:
            self.stage3_channel_yolo_enabled = False
            self._channel_detector = None
            self.log.warn('CHANNEL_YOLO', f'init failed, disabled: {e}')

    # ══════════════ 回调 ══════════════

    def _set_p_inference_active(self, active: bool):
        detector = getattr(self, '_p_detector', None)
        if detector is not None and hasattr(detector, 'set_inference_active'):
            detector.set_inference_active(active)

    def _set_stage3_http_active(self, active: bool):
        detector = getattr(self, '_p_detector', None)
        if detector is None:
            return
        method = getattr(detector, 'start_http_server' if active else 'stop_http_server', None)
        if method is not None:
            method()

    def _set_channel_inference_active(self, active: bool):
        detector = getattr(self, '_channel_detector', None)
        if detector is not None and hasattr(detector, 'set_inference_active'):
            detector.set_inference_active(active)

    def _update_p_inference_gate(self):
        # map 位置存在偏移；Phase3 全程启用 P 检测，由视觉决定最终到达。
        self._set_p_inference_active(self.phase == 3 and not self.mission_finished)

    def _phase_cb(self, msg):
        prev = self.phase
        incoming = int(msg.data)
        self.get_logger().info(
            f'[PHASE] 收到 competition_phase={incoming} (之前={prev}, initialized={self.phase_initialized})'
        )

        if not self.phase_initialized:
            if incoming == 1:
                self.phase = 1
                self.phase_initialized = True
                self._set_p_inference_active(False)
                self._set_stage3_http_active(False)
                self.get_logger().info('[PHASE] ✓ Phase 初始化完成: phase=1')
                return
            if incoming == 3:
                self.phase = 1
                self._set_p_inference_active(False)
                self.get_logger().warn('[PHASE] ⚠ 忽略启动时的 phase=3（可能是旧消息），等待 phase=1')
                return
            self.phase = incoming
            self._set_p_inference_active(False)
            self._set_stage3_http_active(False)
            return

        self.phase = incoming
        if prev == 3 and self.phase != 3:
            self._reset_mission()
            self._set_p_inference_active(False)
            self._set_stage3_http_active(False)
        elif prev != 3 and self.phase == 3:
            self._set_stage3_http_active(True)
            self._arm_mission()

    def _odom_cb(self, msg):
        raw_x = float(msg.pose.pose.position.x)
        raw_y = float(msg.pose.pose.position.y)
        raw_yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        self._last_raw_odom_xy = (raw_x, raw_y)
        self._last_raw_odom_yaw = raw_yaw
        self.odom_frame_id = msg.header.frame_id or self.odom_frame_id
        # 经 map_overlay 静态 TF：map_pos = R(odom_pos) + translation
        cos_y = math.cos(self.map_odom_yaw)
        sin_y = math.sin(self.map_odom_yaw)
        self.current_position = (
            cos_y * raw_x - sin_y * raw_y + self.map_odom_x,
            sin_y * raw_x + cos_y * raw_y + self.map_odom_y,
        )
        if self._imu_yaw is None:
            self.current_yaw = self._normalize_angle(raw_yaw + self.map_odom_yaw)

    def _imu_cb(self, msg):
        self._imu_yaw = self._quat_to_yaw(msg.orientation)
        self.current_yaw = self._normalize_angle(self._imu_yaw + self._imu_yaw_offset)

    def _scan_cb(self, msg):
        self.latest_scan = msg

    # ══════════════ 任务生命周期 ══════════════

    def _arm_mission(self):
        self.mission_active = False
        self.mission_finished = False
        self.map_odom_x = self._default_map_odom_x
        self.map_odom_y = self._default_map_odom_y
        self.map_odom_yaw = self._default_map_odom_yaw
        self._imu_yaw_offset = 0.0
        self.path_started_at = None
        self.path_index = 0
        self._settled_start = None
        self.avoid_state = 'forward'
        self.start_after_time = self._now_sec() + self.start_delay_sec
        self._p_approaching = False
        self._p_consecutive_hits = 0
        self._p_offset_filtered = 0.0
        self._set_p_inference_active(False)
        self._set_channel_inference_active(False)
        self._pre_return_state = 'turn_to_channel_yaw' if self.stage3_channel_yolo_enabled else 'done'
        self._pre_return_started_at = self._now_sec()
        self._channel_hits = 0
        self._channel_offset_filtered = 0.0
        self._publish_state('armed')
        self.log.mission(
            f'phase=3 detected, direction={self.test_direction}, '
            f'pre_return={self._pre_return_state}'
        )

    def _reset_mission(self):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = False
        self.map_odom_x = self._default_map_odom_x
        self.map_odom_y = self._default_map_odom_y
        self.map_odom_yaw = self._default_map_odom_yaw
        self._imu_yaw_offset = 0.0
        self.start_after_time = None
        self.path_started_at = None
        self.path_index = 0
        self._settled_start = None
        self.avoid_state = 'forward'
        self._p_approaching = False
        self._p_consecutive_hits = 0
        self._p_offset_filtered = 0.0
        self._pre_return_state = 'idle'
        self._channel_hits = 0
        self._channel_offset_filtered = 0.0
        self._set_channel_inference_active(False)
        self._set_p_inference_active(False)
        self._publish_state('idle')

    def _start_mission(self):
        if self.current_position is None or self.current_yaw is None:
            self.log.warn('ODOM', 'no odom yet, cannot start (waiting for /odom_combined)')
            return
        if not self.return_waypoints:
            self._publish_feedback('no waypoints configured, cannot start')
            self._fail_mission('no return waypoints')
            return
        self.mission_active = True
        self.path_started_at = self._now_sec()
        self.path_index = 0
        self._publish_state('running')
        self._update_p_inference_gate()
        self.log.mission(
            f'return started, {len(self.return_waypoints)} waypoints (map coords), '
            f'current=({self.current_position[0]:.2f},{self.current_position[1]:.2f}) '
            f'yaw={math.degrees(self.current_yaw):.1f}°'
        )
        self.get_logger().info(f'mission_active=True, will publish cmd_vel now')

    def _finish_mission(self, feedback_text='return complete, reached P point'):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self._set_p_inference_active(False)
        self._publish_state('complete')
        self._publish_feedback(feedback_text)
        sys.stderr.write('\n=== STAGE3 RETURN COMPLETE ===\n\n')

    def _fail_mission(self, reason):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self._set_channel_inference_active(False)
        self._set_p_inference_active(False)
        self._publish_state('failed')
        self._publish_feedback(f'return failed: {reason}')
        sys.stderr.write(f'\n=== STAGE3 RETURN FAILED: {reason} ===\n\n')


    # ══════════════ Stage3 前置通道重定位 ══════════════

    def _reset_stage3_map_origin_to_channel(self):
        if self._last_raw_odom_xy is None:
            return False
        raw_x, raw_y = self._last_raw_odom_xy
        yaw_now = self._imu_yaw
        if yaw_now is None:
            yaw_now = self.current_yaw
        if yaw_now is None:
            yaw_now = (
                self._normalize_angle(self._last_raw_odom_yaw + self.map_odom_yaw)
                if self._last_raw_odom_yaw is not None else self.stage3_channel_reset_yaw
            )
        yaw_offset = self._normalize_angle(self.stage3_channel_reset_yaw - yaw_now)
        cos_y = math.cos(yaw_offset)
        sin_y = math.sin(yaw_offset)
        self.map_odom_x = self.stage3_channel_reset_map_x - (cos_y * raw_x - sin_y * raw_y)
        self.map_odom_y = self.stage3_channel_reset_map_y - (sin_y * raw_x + cos_y * raw_y)
        self.map_odom_yaw = yaw_offset
        self._imu_yaw_offset = yaw_offset
        self.current_position = (
            self.stage3_channel_reset_map_x,
            self.stage3_channel_reset_map_y,
        )
        self.current_yaw = self.stage3_channel_reset_yaw
        self.path_index = 0
        self.path_started_at = None
        self._filtered_heading_err = 0.0
        self.avoid_state = 'forward'
        self._p_approaching = False
        self._p_consecutive_hits = 0
        self._p_offset_filtered = 0.0
        self._p_extra_forward_active = False
        self.log.mission(
            f'Stage3 local map origin reset: odom=({raw_x:.3f},{raw_y:.3f}) '
            f'imu_yaw={math.degrees(yaw_now):.1f}deg -> '
            f'map=({self.stage3_channel_reset_map_x:.2f},{self.stage3_channel_reset_map_y:.2f}) '
            f'map_odom=({self.map_odom_x:.3f},{self.map_odom_y:.3f},'
            f'{math.degrees(self.map_odom_yaw):.1f}deg)'
        )
        return True

    def _run_pre_return_handoff(self):
        if not self.stage3_channel_yolo_enabled or self._channel_detector is None:
            self._pre_return_state = 'done'
            return True
        now = self._now_sec()
        started = self._pre_return_started_at or now
        if now - started > self.stage3_channel_timeout_sec:
            self.log.warn('CHANNEL_YOLO', 'pre-return timeout, resetting local map and starting return')
            self._set_channel_inference_active(False)
            self._reset_stage3_map_origin_to_channel()
            self._pre_return_state = 'done'
            return True

        if self.current_yaw is None:
            self.cmd_pub.publish(self._twist(0.0, 0.0))
            return False

        if self._pre_return_state in ('idle', 'turn_to_channel_yaw'):
            self._pre_return_state = 'turn_to_channel_yaw'
            self._publish_state('pre_return_turn_to_channel')
            err = self._angle_error(self.stage3_channel_yaw, self.current_yaw)
            if abs(err) <= self.stage3_channel_yaw_tolerance:
                self._pre_return_state = 'yolo_centering'
                self._channel_hits = 0
                self._channel_offset_filtered = 0.0
                self._set_channel_inference_active(True)
                self.stop_robot()
                self.log.mission(
                    f'pre-return channel yaw aligned yaw={math.degrees(self.current_yaw):.1f}deg, YOLO on'
                )
                return False
            angular = self._clamp(2.0 * err, self.max_angular)
            if abs(angular) < self.min_angular:
                angular = math.copysign(self.min_angular, err)
            linear = self.pursuit_turn_linear
            self.cmd_pub.publish(self._twist(linear, angular))
            if now - getattr(self, '_pre_return_log_time', 0.0) >= 0.5:
                self._pre_return_log_time = now
                self.log.segment(
                    f'pre_return_turn yaw={math.degrees(self.current_yaw):.1f}deg '
                    f'target={math.degrees(self.stage3_channel_yaw):.1f}deg '
                    f'err={math.degrees(err):.1f}deg v={linear:.2f} w={angular:.2f}'
                )
            return False

        self._publish_state('pre_return_channel_yolo')
        detected, conf, bbox, ts, offset, fill_ratio = self._channel_detector.get_detection()
        age = now - float(ts or 0.0)
        if not detected or bbox is None or age > 0.8:
            self.cmd_pub.publish(self._twist(0.0, 0.0))
            if now - getattr(self, '_pre_return_log_time', 0.0) >= 0.5:
                self._pre_return_log_time = now
                self.log.segment(f'pre_return_yolo waiting detection age={age:.2f}s')
            return False

        alpha = 0.35
        self._channel_offset_filtered = (
            alpha * float(offset) + (1.0 - alpha) * self._channel_offset_filtered
        )
        filt = self._channel_offset_filtered
        yaw_err = self._angle_error(self.stage3_channel_yaw, self.current_yaw)
        centered = (
            abs(filt) <= self.stage3_channel_offset_tolerance
            and fill_ratio >= self.stage3_channel_fill_ratio
            and abs(yaw_err) <= self.stage3_channel_yaw_tolerance
        )
        self._channel_hits = self._channel_hits + 1 if centered else 0
        if self._channel_hits >= self.stage3_channel_consecutive_hits:
            self.stop_robot()
            self._set_channel_inference_active(False)
            self._reset_stage3_map_origin_to_channel()
            self._pre_return_state = 'done'
            self.log.mission(
                f'pre-return channel center reached conf={conf:.2f} off={filt:+.3f} '
                f'fill={fill_ratio:.2%}; start return'
            )
            return True

        if abs(filt) <= self.stage3_channel_offset_deadband:
            angular = 0.0
        else:
            effective = math.copysign(abs(filt) - self.stage3_channel_offset_deadband, filt)
            angular = -self.stage3_channel_angular_kp * effective
        angular += 0.4 * yaw_err
        angular = self._clamp(angular, self.stage3_channel_max_angular)
        speed = self.stage3_channel_linear_speed
        if abs(filt) > 0.30:
            speed *= 0.5
        self.cmd_pub.publish(self._twist(speed, angular))
        if now - getattr(self, '_pre_return_log_time', 0.0) >= 0.5:
            self._pre_return_log_time = now
            self.log.segment(
                f'pre_return_yolo conf={conf:.2f} off={offset:+.3f} filt={filt:+.3f} '
                f'fill={fill_ratio:.2%} hits={self._channel_hits}/'
                f'{self.stage3_channel_consecutive_hits} yaw_err={math.degrees(yaw_err):.1f}deg '
                f'v={speed:.2f} w={angular:.2f}'
            )
        return False

    # ══════════════ 主控制循环 ══════════════

    def _control_loop(self):
        if self.phase != 3 or self.mission_finished:
            return

        now = self._now_sec()
        if not self.mission_active:
            if self.start_after_time is None or now < self.start_after_time:
                return
            if self._pre_return_state != 'done':
                if not self._run_pre_return_handoff():
                    return
            self._start_mission()
            return

        # 1. 紧急停止
        if self._check_emergency_stop():
            return

        # 2. P 已确认后由视觉完全接管；不再使用 map 位置判定到达。
        if self._p_approaching:
            self._run_p_approach()
            return

        # 3. 地图仅用于粗导航和搜索 P；检测到 P 后立即切入视觉终段。
        self._update_p_detection()
        if self._p_approaching:
            self._run_p_approach()
            return

        # 4. 避障检测（仅在地图粗导航态）
        if self.avoid_state == 'forward' and self.latest_scan is not None:
            self._check_obstacle()

        # 5. 若在避障状态，运行避障
        if self.avoid_state != 'forward':
            self._run_avoidance()
            return

        # 6. 尚未识别 P，继续地图粗导航。
        self._run_center_drive()

    def _check_emergency_stop(self):
        if self.latest_scan is None:
            return False
        min_dist = float('inf')
        for i, d in enumerate(self.latest_scan.ranges):
            if math.isinf(d) or math.isnan(d) or d < self.min_range:
                continue
            if d < min_dist:
                min_dist = d
        if min_dist <= self.emergency_stop_dist:
            self.stop_robot()
            self._publish_feedback(f'emergency stop, closest={min_dist:.2f}m')
            return True
        return False

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # ══════════════ 地图粗导航 + P 视觉最终到达 ══════════════

    def _p_detection(self):
        if self._p_detector is None:
            return False, 0.0, None, 0.0, 0.0, 0.0
        detected, conf, bbox, stamp, offset, fill = self._p_detector.get_p_detection_geometry()
        fresh = time.time() - float(stamp or 0.0) <= self.p_detection_timeout
        return bool(detected and bbox is not None and fresh), conf, bbox, stamp, offset, fill

    def _update_p_detection(self):
        detected, conf, _bbox, _stamp, offset, _fill = self._p_detection()
        confirmed = detected and conf >= self.p_approach_conf
        self._p_consecutive_hits = self._p_consecutive_hits + 1 if confirmed else 0
        if self._p_consecutive_hits < self.p_approach_hits_required:
            return
        self._p_approaching = True
        self._p_offset_filtered = float(offset)
        self._publish_state('p_approach')
        self.log.segment(
            f'P acquired conf={conf:.2f} offset={offset:+.3f}; '
            f'visual approach v={self.p_approach_linear:.2f}'
        )
        self._publish_feedback('P acquired, visual final approach started')

    def _run_p_approach(self):
        detected, conf, _bbox, _stamp, offset, fill = self._p_detection()
        if not detected or conf < self.p_approach_conf:
            self.cmd_pub.publish(Twist())
            self.log.segment('P lost after visual acquisition: v=0, complete and coast into P')
            self._finish_mission('P lost after acquisition: v=0, inertial coast into P')
            return

        alpha = 0.35
        self._p_offset_filtered = alpha * float(offset) + (1.0 - alpha) * self._p_offset_filtered
        angular = self._clamp(-self.p_approach_angular_kp * self._p_offset_filtered, self.max_angular)
        self._publish_state('p_approach')
        self.log.telemetry(
            'P_APPROACH',
            f'conf={conf:.2f} off={self._p_offset_filtered:+.3f} fill={fill:.2%} '
            f'spd={self.p_approach_linear:.2f} ang={angular:.2f}',
        )
        self.cmd_pub.publish(self._twist(self.p_approach_linear, angular))


    def _advance_waypoint(self, pose):
        """跳过已到达的中间路点"""
        while self.path_index < len(self.return_waypoints) - 1:
            wp = self.return_waypoints[self.path_index]
            dist = math.hypot(wp['x'] - pose[0], wp['y'] - pose[1])
            if dist > self.waypoint_tolerance:
                return
            self.path_index += 1

    def _run_center_drive(self):
        now = self._now_sec()
        if self.path_started_at is not None and now - self.path_started_at > self.path_timeout_sec:
            self.log.timeout(f'path timeout after {self.path_timeout_sec}s')
            self._fail_mission('path timeout')
            return

        if self.current_position is None or self.current_yaw is None:
            self.log.warn('ODOM', 'no pose, waiting for odom')
            return

        # P 尚未识别时用地图目标作粗导航；地图坐标不参与最终完成判定。
        target_x, target_y = self._goal_center()

        # ── 计算目标相对车体坐标 ──
        dx = target_x - self.current_position[0]
        dy = target_y - self.current_position[1]
        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)
        tx = cos_y * dx + sin_y * dy
        ty = -sin_y * dx + cos_y * dy
        target_dist = math.hypot(tx, ty)
        heading_err = math.atan2(ty, tx if abs(tx) > 1e-6 else 1e-6)

        # ── 航向误差低通滤波（消除抖动）──
        alpha = 0.3
        self._filtered_heading_err = alpha * heading_err + (1.0 - alpha) * self._filtered_heading_err
        heading_err = self._filtered_heading_err

        self._publish_state('map_search_p')

        angular = self._clamp(self.pursuit_turn_kp * heading_err, self.max_angular)
        if abs(angular) < 1e-4:
            angular = 0.0

        speed = self.pursuit_linear_speed
        if abs(heading_err) > math.radians(30.0):
            speed = self.pursuit_turn_linear
        elif abs(heading_err) > math.radians(5.0):
            speed = self.pursuit_linear_speed * 0.5

        self.log.telemetry('MAP_SEARCH_P',
            f'dist={target_dist:.2f} '
            f'err={math.degrees(heading_err):.1f}° '
            f'spd={speed:.2f} ang={angular:.2f} '
            f'pos=({self.current_position[0]:.2f},{self.current_position[1]:.2f}) '
            f'center=({target_x:.2f},{target_y:.2f})'
        )
        self.cmd_pub.publish(self._twist(speed, angular))

    # ══════════════ 避障状态（同 Stage1）═════════════
    def _clusters_in_window(self, scan_msg):
        clusters = []
        cur = []
        prev_pt = None
        for i, d in enumerate(scan_msg.ranges):
            if math.isinf(d) or math.isnan(d) or d < self.min_range:
                if cur:
                    clusters.append(cur)
                    cur = []
                prev_pt = None
                continue
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = d * math.cos(angle)
            y = d * math.sin(angle)
            if x < self.window_min_x or x > self.window_max_x or abs(y) > self.window_half_width:
                if cur:
                    clusters.append(cur)
                    cur = []
                prev_pt = None
                continue
            pt = (x, y, d)
            if prev_pt is None or math.hypot(prev_pt[0] - pt[0], prev_pt[1] - pt[1]) <= self.cluster_gap:
                cur.append(pt)
            else:
                if cur:
                    clusters.append(cur)
                cur = [pt]
            prev_pt = pt
        if cur:
            clusters.append(cur)
        return clusters

    def _classify_cluster(self, cluster):
        """
        分类聚类：'obstacle' 或 'wall'

        墙壁特征（根据实测数据调整）：
        - 点数 > 40（实测墙壁 50-60 点）
        - 角度跨度 > 30度（实测墙壁 39-42度）
        - 点密集连续（相邻点平均间距 < 0.012 m，实测墙壁 0.007-0.010m）
        Returns:
            str: 'obstacle' 或 'wall'
        """
        if len(cluster) < 40:
            return 'obstacle'
        
        # 角度跨度
        angles = [math.atan2(pt[1], pt[0]) for pt in cluster]
        angle_span = max(angles) - min(angles)
        if angle_span < math.radians(30.0):
            return 'obstacle'
        
        distances = []
        for i in range(len(cluster) - 1):
            d = math.hypot(cluster[i+1][0] - cluster[i][0], 
                           cluster[i+1][1] - cluster[i][1])
            distances.append(d)
        
        if distances:
            avg_gap = sum(distances) / len(distances)
            if avg_gap > 0.012:
                return 'obstacle'
        
        return 'wall'

    def _find_nearest_obstacle(self, scan_msg):
        """返回最近的障碍物（墙壁已过滤）"""
        clusters = self._clusters_in_window(scan_msg)
        best = None
        wall_count = 0
        obstacle_count = 0
        
        for c in clusters:
            if len(c) < self.min_cluster_pts:
                continue
            
            # 计算聚类属性用于分类和日志
            nearest = min(p[2] for p in c)
            angles = [math.atan2(pt[1], pt[0]) for pt in c]
            angle_span = max(angles) - min(angles)
            angle_span_deg = math.degrees(angle_span)
            xs = [pt[0] for pt in c]
            ys = [pt[1] for pt in c]
            width = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            
            # 计算平均间距
            distances = []
            for i in range(len(c) - 1):
                d = math.hypot(c[i+1][0] - c[i][0], c[i+1][1] - c[i][1])
                distances.append(d)
            avg_gap = sum(distances) / len(distances) if distances else 0.0
            
            # 分类聚类
            cluster_type = self._classify_cluster(c)
            
            if cluster_type == 'obstacle':
                obstacle_count += 1
                if best is None or nearest < best['dist']:
                    nearest_pt = min(c, key=lambda p: p[2])
                    danger_angle = math.atan2(nearest_pt[1], nearest_pt[0])
                    danger_deg = math.degrees(danger_angle)
                    best = {
                        'dist': nearest,
                        'danger_deg': danger_deg,
                        'width': width,
                        'pts': len(c),
                    }
            else:
                wall_count += 1
            
            self.log.telemetry('CLUSTER',
                f'type={cluster_type} dist={nearest:.2f} span={angle_span_deg:.1f} '
                f'pts={len(c)} w={width:.2f} gap={avg_gap:.3f}')
        
        self.log.telemetry('FILTER_SUMMARY',
            f'walls={wall_count} obstacles={obstacle_count}')
        
        return best

    def _check_obstacle(self):
        obs = self._find_nearest_obstacle(self.latest_scan)
        if obs is not None and obs['dist'] < self.avoid_safe_dist:
            self._begin_avoidance(obs['danger_deg'])

    def _begin_avoidance(self, danger_deg):
        self.avoid_state = 'avoiding'
        self.avoid_turn_direction = -1.0 if danger_deg > 0.0 else 1.0
        self.avoid_started_time = self.get_clock().now()
        self.avoid_clear_since = None
        self.avoid_entry_yaw = self.current_yaw
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self._publish_state('avoiding')
        self.log.corner_avoid(f'start dir={self.avoid_turn_direction:.1f} danger={danger_deg:.1f}°')
        self._publish_feedback(f'avoid start dir={self.avoid_turn_direction:.1f} danger={danger_deg:.1f}°')

    def _run_avoidance(self):
        now = self.get_clock().now()

        if self.avoid_state == 'avoiding':
            avoid_elapsed = 0.0
            if self.avoid_started_time is not None:
                avoid_elapsed = (now - self.avoid_started_time).nanoseconds / 1e9

            turned = False
            if self.current_yaw is not None and self.avoid_entry_yaw is not None:
                turned = abs(self._angle_error(self.current_yaw, self.avoid_entry_yaw)) >= self.avoid_min_turn_angle

            cone_clear = True
            if self.latest_scan is not None:
                obs = self._find_nearest_obstacle(self.latest_scan)
                if obs is not None and obs['dist'] < self.avoid_clear_dist:
                    cone_clear = False

            if cone_clear and self.avoid_clear_since is None:
                self.avoid_clear_since = now
            elif not cone_clear:
                self.avoid_clear_since = None

            clear_elapsed = 0.0
            if self.avoid_clear_since is not None:
                clear_elapsed = (now - self.avoid_clear_since).nanoseconds / 1e9

            if avoid_elapsed >= self.avoid_min_duration and clear_elapsed >= self.avoid_clear_hold and turned:
                self._begin_counter_steer()
                return

            self.cmd_pub.publish(self._twist(self.avoid_linear_speed,
                                             self.avoid_turn_direction * self.avoid_angular_speed))

        elif self.avoid_state == 'countersteering':
            if self.counter_steer_deadline is not None and now >= self.counter_steer_deadline:
                self._begin_recovery()
                return
            self.cmd_pub.publish(self._twist(self.counter_linear,
                                             -self.avoid_turn_direction * self.counter_angular))

        elif self.avoid_state == 'recovering':
            if self._recovery_complete():
                self._finish_recovery()
                return

            if self.current_yaw is not None:
                target_yaw = self.avoid_entry_yaw if self.avoid_entry_yaw is not None else 0.0
                error = self._angle_error(target_yaw, self.current_yaw)
                angular = self._clamp(self.recovery_kp * error, self.recovery_max_angular)
                if abs(error) > self.recovery_in_place and abs(angular) < self.recovery_min_angular:
                    angular = math.copysign(self.recovery_min_angular, error)
                linear = self.recovery_turn_linear if abs(error) > self.recovery_in_place else self.recovery_linear
                self.cmd_pub.publish(self._twist(linear, angular))
            else:
                self.cmd_pub.publish(self._twist(self.recovery_linear,
                                                 -self.avoid_turn_direction * self.recovery_angular))

    def _begin_counter_steer(self):
        if self.avoid_state != 'avoiding':
            return
        now = self.get_clock().now()
        avoid_dur = 0.0
        if self.avoid_started_time is not None:
            avoid_dur = (now - self.avoid_started_time).nanoseconds / 1e9
        self.last_avoid_duration = avoid_dur
        dur = max(self.counter_min_dur, min(self.counter_max_dur, avoid_dur * self.counter_duration_scale))
        self.avoid_state = 'countersteering'
        self.avoid_clear_since = None
        self.counter_steer_deadline = now + Duration(seconds=dur)
        self.recovery_deadline = None

    def _begin_recovery(self):
        if self.avoid_state not in ('avoiding', 'countersteering'):
            return
        self.avoid_state = 'recovering'
        self.avoid_clear_since = None
        self.counter_steer_deadline = None
        dur = max(0.15, min(self.recovery_timeout, self.last_avoid_duration * self.recovery_duration_scale))
        self.recovery_deadline = self.get_clock().now() + Duration(seconds=dur)

    def _recovery_complete(self):
        now = self.get_clock().now()
        if self.current_yaw is not None and self.avoid_entry_yaw is not None:
            if abs(self._angle_error(self.avoid_entry_yaw, self.current_yaw)) <= self.recovery_in_place:
                return True
        if self.recovery_deadline is not None and now >= self.recovery_deadline:
            return True
        return False

    def _finish_recovery(self):
        self.avoid_state = 'forward'
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self._publish_state('running')

    def destroy_node(self):
        try:
            self._set_p_inference_active(False)
            self._set_stage3_http_active(False)
            self.log.close()
            if rclpy.ok():
                self.cmd_pub.publish(Twist())
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    init_without_ros_signal_handler(args)
    node = Stage3ReturnNavigator()
    stop_event = threading.Event()
    
    def _stop_callback():
        publish_stop(node.cmd_pub, repeat=10)
    
    install_stop_event(stop_event, _stop_callback, cli_topics=['/cmd_vel'])
    
    try:
        spin_until_stop(node, stop_event)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                publish_stop(node.cmd_pub, repeat=15)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

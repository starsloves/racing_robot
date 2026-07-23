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
from collections import deque

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from racing_common.racing_logger import RacingLogger
from racing_common.yolo_bbox_detector import YoloBBoxDetector
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformException, TransformListener

from .cmd_vel_stop import (
    emergency_cli_stop_async,
    init_without_ros_signal_handler,
    install_stop_event,
    publish_stop,
    spin_until_stop,
)
from .global_path_planner import GlobalPathPlanner
from .vision_p_detector import VisionPDetector


class Stage3ReturnNavigator(Node):
    def __init__(self):
        super().__init__('stage3_return_navigator')

        self._declare_params()
        self._read_params()

        # 文件会话在 phase=3 激活时才打开，避免 Stage1 覆盖上一轮 Stage3 日志。
        self.log = RacingLogger(
            self, log_subdir='competition_stage3',
            log_filename='latest.log', session_title='Stage3 return navigator',
            defer_file=True,
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

        # Position is anchored by the Stage2 handoff map pose, then propagated
        # by the /odom_combined displacement rotated into the map frame. This
        # prevents a later map TF jump from moving Stage3's whole A* frame.
        # Heading always comes from IMU, never odometry orientation.
        self.current_position = None
        self.current_yaw = None
        self.odom_frame_id = 'odom'
        self._last_raw_odom_xy = None
        self._last_raw_odom_yaw = None
        self._imu_yaw = None
        self._imu_yaw_offset = 0.0
        self._awaiting_entry_yaw_alignment = False
        self._entry_anchor_map = None
        self._entry_anchor_odom = None
        self._entry_anchor_map_from_odom_yaw = None
        self._pending_entry_anchor_map = None
        self._entry_anchor_stamp_sec = None
        self._last_tf_position = None

        # 路径状态
        self.path_started_at = None
        self.path_index = 0
        self._settled_start = None
        self._filtered_heading_err = 0.0
        self._active_planned_path = []
        self._active_path_cursor = 0
        self._planner_reverse_start = None
        self._planner_reverse_started_at = None
        self._planner_forbidden_reverse_attempts = 0
        self._map_pose_warned = False
        self._entry_anchor_tf_warned = False
        self._visual_search_gate_reached = False
        self._initial_align_required = False

        # P 视觉最终接管：地图只用于粗导航，P 视觉决定最终到达。
        self._p_detector = None
        self._p_approaching = False
        self._p_consecutive_hits = 0
        self._p_last_confirmed_stamp = None
        self._p_offset_filtered = 0.0
        self._p_target_yaw = None
        self._p_heading_lock_offset = None
        self._p_last_heading_reacquire_at = None
        self._p_lost_since = None
        self._p_lost_reverse_started_at = None
        self._p_last_angular = 0.0
        # Once P is within the final approach distance, lidar must not detour around it.
        self._p_final_approach_latched = False
        self._p_visible_yaw_history = deque()
        self._p_recovery_target_yaw = None
        # Preserve the visual heading while lidar temporarily takes control.
        self._p_avoidance_recovery_yaw = None

        # P 终段深度当前仅用于实车标定日志，不参与控制或完成判定。
        self._depth_bridge = CvBridge()
        self._depth_lock = threading.Lock()
        self._last_depth_image = None
        self._last_depth_received_at = 0.0
        self._last_depth_encoding = ''

        # 激光扫描
        self.latest_scan = None

        # ── 避障状态 ──
        self.avoid_state = 'forward'
        self.avoid_turn_direction = 0.0
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.desired_heading = None
        self.recovery_uses_heading = False
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.emergency_reverse_deadline = None

        # 保留旧通道 YOLO 对象，仅用于诊断；它不允许阻塞生产返程。
        self._pre_return_state = 'idle'
        self._pre_return_started_at = None
        self._channel_detector = None
        self._channel_hits = 0
        self._channel_offset_filtered = 0.0
        self._preplan_start = None
        self._preplanned_path = []

        # ── Pub/Sub ──
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, qos_latched)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Int32, self.phase_topic, self._phase_cb, qos_latched)
        self.create_subscription(String, self.direction_topic, self._direction_cb, qos_latched)
        self.create_subscription(
            PointStamped, self.stage3_entry_anchor_topic,
            self._stage3_entry_anchor_cb, qos_latched,
        )
        self.create_subscription(
            PointStamped, self.stage3_preplan_pose_topic,
            self._stage3_preplan_pose_cb, qos_latched,
        )
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        if self.p_depth_logging_enabled:
            self.create_subscription(
                Image, self.p_depth_topic, self._depth_cb,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
            )

        self.global_planner = None
        if self.use_global_planner:
            self.global_planner = GlobalPathPlanner(self, {
                'map_topic': self.map_topic,
                'global_frame_id': self.global_frame_id,
                'planner_downsample': self.planner_downsample,
                'planner_occupied_threshold': self.planner_occupied_threshold,
                'planner_unknown_is_occupied': self.planner_unknown_is_occupied,
                'planner_obstacle_inflation_m': self.planner_obstacle_inflation_m,
                'planner_forbidden_rectangles_json': self.planner_forbidden_rectangles_json,
                'planner_clear_rectangles_json': self.planner_clear_rectangles_json,
            })
            self.log.startup('A* global planner enabled: static map only; lidar reserved for avoidance')

        self._init_p_detector()
        self._init_channel_detector()

        self._publish_state('idle')
        self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.log.startup(
            f'enhanced return navigator ready | waypoints={len(self.return_waypoints)} '
            f'cmd={self.cmd_topic} odom={self.odom_topic}'
        )
        if self.p_depth_logging_enabled:
            self.log.startup(
                f'P depth logging enabled: topic={self.p_depth_topic} '
                f'scale={self.p_depth_unit_scale_m:.6f}m/unit'
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
        self.declare_parameter('direction_topic', 'competition_qr_task')
        self.declare_parameter('stage3_entry_anchor_topic', 'stage3_entry_anchor')
        self.declare_parameter('stage3_preplan_pose_topic', 'stage3_preplan_pose')
        self.declare_parameter('require_stage3_entry_anchor', True)
        self.declare_parameter('stage3_entry_anchor_max_age_sec', 3.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('start_delay_sec', 0.0)
        self.declare_parameter('stage3_entry_map_yaw_deg', -90.0)

        # ── 路点 ──
        self.declare_parameter('return_waypoints_json', '[]')
        self.declare_parameter('waypoint_tolerance', 0.18)
        self.declare_parameter('path_timeout_sec', 60.0)

        # ── P 点视觉最终到达 ──
        self.declare_parameter('p_model_path', '')
        self.declare_parameter('p_conf_thres', 0.25)
        self.declare_parameter('p_iou_thres', 0.45)
        self.declare_parameter('p_crop_ratio', 0.4)
        self.declare_parameter('p_approach_conf_threshold', 0.5)
        self.declare_parameter('p_approach_consecutive_hits', 3)
        self.declare_parameter('p_approach_linear_speed', 0.50)
        self.declare_parameter('p_approach_offset_filter_alpha', 0.55)
        self.declare_parameter('p_heading_bearing_gain_rad', 0.55)
        self.declare_parameter('p_heading_kp', 1.4)
        self.declare_parameter('p_heading_tolerance_deg', 3.0)
        self.declare_parameter('p_heading_max_angular_speed', 0.45)
        self.declare_parameter('p_heading_reacquire_offset', 0.18)
        self.declare_parameter('p_heading_reacquire_interval_sec', 0.35)
        self.declare_parameter('p_loss_reverse_speed', 0.10)
        self.declare_parameter('p_loss_reverse_duration_sec', 0.80)
        self.declare_parameter('p_loss_reverse_max_angular', 0.35)
        self.declare_parameter('p_loss_heading_lookback_sec', 1.0)
        self.declare_parameter('p_loss_heading_tolerance_deg', 6.0)
        self.declare_parameter('p_loss_heading_kp', 1.2)
        self.declare_parameter('p_web_port', 8083)
        self.declare_parameter('p_detection_timeout_sec', 0.35)
        self.declare_parameter('p_depth_logging_enabled', True)
        self.declare_parameter('p_depth_topic', '/aurora/depth/image_raw')
        self.declare_parameter('p_depth_unit_scale_m', 0.001)
        self.declare_parameter('p_depth_max_age_sec', 0.25)
        self.declare_parameter('p_depth_min_m', 0.10)
        self.declare_parameter('p_depth_max_m', 4.00)
        self.declare_parameter('p_depth_roi_fraction', 0.50)
        self.declare_parameter('p_depth_stop_distance_m', 0.50)
        self.declare_parameter('p_approach_disable_avoidance_distance_m', 0.0)

        # ── Pure Pursuit ──
        self.declare_parameter('pursuit_linear_speed', 0.18)
        self.declare_parameter('pursuit_lookahead_m', 0.45)
        self.declare_parameter('pursuit_heading_stop_deg', 70.0)
        self.declare_parameter('pursuit_turn_kp', 1.8)
        self.declare_parameter('pursuit_turn_linear_speed', 0.08)
        self.declare_parameter('pursuit_min_turn_radius_m', 0.42)
        self.declare_parameter('initial_align_trigger_deg', 30.0)
        self.declare_parameter('initial_align_tolerance_deg', 8.0)
        self.declare_parameter('initial_align_angular_speed', 1.5)
        self.declare_parameter('initial_align_linear_speed', 0.0)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('min_angular_speed', 0.45)

        # ── 避障状态（同 Stage1）──
        self.declare_parameter('avoid_linear_speed', 0.10)
        self.declare_parameter('avoid_angular_speed', 0.80)
        self.declare_parameter('avoid_min_duration_sec', 0.70)
        self.declare_parameter('avoid_clear_hold_sec', 0.25)
        self.declare_parameter('avoid_min_turn_angle_deg', 18.0)
        self.declare_parameter('avoid_right_turn_left_obstacle_angle_deg', 15.0)
        self.declare_parameter('avoid_goal_bias_enabled', True)
        self.declare_parameter('avoid_obstacle_side_penalty', 3.5)
        self.declare_parameter('avoid_safe_distance', 0.50)
        self.declare_parameter('avoid_clear_distance', 0.65)
        self.declare_parameter('emergency_stop_distance', 0.22)
        self.declare_parameter('emergency_reverse_speed', 0.10)
        self.declare_parameter('emergency_reverse_duration_sec', 0.80)
        self.declare_parameter('emergency_reverse_angular_speed', 0.35)

        self.declare_parameter('recovery_linear_speed', 0.12)
        self.declare_parameter('recovery_turn_linear_speed', 0.08)
        self.declare_parameter('recovery_angular_speed', 0.75)
        self.declare_parameter('recovery_heading_kp', 2.4)
        self.declare_parameter('recovery_max_angular_speed', 1.1)
        self.declare_parameter('recovery_min_angular_speed', 0.5)
        self.declare_parameter('recovery_in_place_angle_deg', 8.0)
        self.declare_parameter('heading_tolerance_deg', 6.0)
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
        self.declare_parameter('emergency_window_min_x', 0.08)
        self.declare_parameter('emergency_window_max_x', 0.45)
        self.declare_parameter('emergency_window_half_width', 0.12)
        self.declare_parameter('emergency_min_cluster_points', 2)
        self.declare_parameter('cluster_gap_tolerance', 0.12)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('min_cluster_width', 0.06)
        self.declare_parameter('max_cluster_width', 0.55)
        self.declare_parameter('min_valid_range', 0.15)

        # ── A* 全局路径规划（避开地图禁区）──
        self.declare_parameter('use_global_planner', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('planner_downsample', 4)
        self.declare_parameter('planner_occupied_threshold', 50)
        self.declare_parameter('planner_unknown_is_occupied', False)
        self.declare_parameter('planner_obstacle_inflation_m', 0.14)
        self.declare_parameter('planner_path_deviation_replan_m', 0.35)
        self.declare_parameter('planner_forbidden_rectangles_json', '[]')
        self.declare_parameter('planner_clear_rectangles_json', '[]')
        self.declare_parameter('planner_forbidden_reverse_speed', 0.10)
        self.declare_parameter('planner_forbidden_reverse_distance_m', 0.30)
        self.declare_parameter('planner_forbidden_reverse_timeout_sec', 5.0)
        self.declare_parameter('planner_forbidden_reverse_max_attempts', 3)

        self.declare_parameter('test_direction', 'clockwise')

        # ── Stage3 通道 YOLO（生产返程不使用）──
        self.declare_parameter('stage3_channel_yolo_enabled', False)
        self.declare_parameter('stage3_channel_model_path', '/home/sunrise/dev_ws/best_rdk_tongdao.bin')
        self.declare_parameter('stage3_channel_camera_topic', '/aurora/rgb/image_raw')
        self.declare_parameter('stage3_channel_conf_thres', 0.25)
        self.declare_parameter('stage3_channel_iou_thres', 0.45)
        self.declare_parameter('stage3_channel_preview_path', '/tmp/stage3_channel_yolo.jpg')
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
        self.direction_topic = str(self.get_parameter('direction_topic').value)
        self.stage3_entry_anchor_topic = str(
            self.get_parameter('stage3_entry_anchor_topic').value
        )
        self.stage3_preplan_pose_topic = str(
            self.get_parameter('stage3_preplan_pose_topic').value
        )
        self.require_stage3_entry_anchor = bool(
            self.get_parameter('require_stage3_entry_anchor').value
        )
        self.stage3_entry_anchor_max_age = float(
            self.get_parameter('stage3_entry_anchor_max_age_sec').value
        )
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.stage3_entry_map_yaw = math.radians(float(
            self.get_parameter('stage3_entry_map_yaw_deg').value
        ))

        self.return_waypoints_json = self.get_parameter('return_waypoints_json').value
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
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
        self.p_approach_offset_filter_alpha = min(1.0, max(0.05, float(
            self.get_parameter('p_approach_offset_filter_alpha').value
        )))
        self.p_heading_bearing_gain = max(0.0, float(
            self.get_parameter('p_heading_bearing_gain_rad').value
        ))
        self.p_heading_kp = float(self.get_parameter('p_heading_kp').value)
        self.p_heading_tolerance = math.radians(max(0.1, float(
            self.get_parameter('p_heading_tolerance_deg').value
        )))
        self.p_heading_max_angular = max(0.0, float(
            self.get_parameter('p_heading_max_angular_speed').value
        ))
        self.p_heading_reacquire_offset = min(1.0, max(0.0, float(
            self.get_parameter('p_heading_reacquire_offset').value
        )))
        self.p_heading_reacquire_interval = max(0.0, float(
            self.get_parameter('p_heading_reacquire_interval_sec').value
        ))
        self.p_loss_reverse_speed = abs(float(self.get_parameter('p_loss_reverse_speed').value))
        self.p_loss_reverse_duration = max(
            0.0, float(self.get_parameter('p_loss_reverse_duration_sec').value)
        )
        self.p_loss_reverse_max_angular = abs(float(
            self.get_parameter('p_loss_reverse_max_angular').value
        ))
        self.p_loss_heading_lookback = max(0.0, float(
            self.get_parameter('p_loss_heading_lookback_sec').value
        ))
        self.p_loss_heading_tolerance = math.radians(max(
            0.1, float(self.get_parameter('p_loss_heading_tolerance_deg').value)
        ))
        self.p_loss_heading_kp = float(self.get_parameter('p_loss_heading_kp').value)
        self.p_web_port = int(self.get_parameter('p_web_port').value)
        self.p_detection_timeout = max(
            0.0, float(self.get_parameter('p_detection_timeout_sec').value)
        )
        self.p_depth_logging_enabled = bool(
            self.get_parameter('p_depth_logging_enabled').value
        )
        self.p_depth_topic = str(self.get_parameter('p_depth_topic').value)
        self.p_depth_unit_scale_m = float(
            self.get_parameter('p_depth_unit_scale_m').value
        )
        self.p_depth_max_age = max(
            0.0, float(self.get_parameter('p_depth_max_age_sec').value)
        )
        self.p_depth_min_m = float(self.get_parameter('p_depth_min_m').value)
        self.p_depth_max_m = float(self.get_parameter('p_depth_max_m').value)
        self.p_depth_roi_fraction = min(
            1.00,
            max(0.10, float(self.get_parameter('p_depth_roi_fraction').value)),
        )
        self.p_depth_stop_distance = max(
            0.0, float(self.get_parameter('p_depth_stop_distance_m').value)
        )
        self.p_approach_disable_avoidance_distance = max(
            0.0, float(self.get_parameter(
                'p_approach_disable_avoidance_distance_m'
            ).value)
        )

        self.pursuit_linear_speed = float(self.get_parameter('pursuit_linear_speed').value)
        self.pursuit_lookahead = float(self.get_parameter('pursuit_lookahead_m').value)
        self.pursuit_heading_stop = math.radians(float(self.get_parameter('pursuit_heading_stop_deg').value))
        self.pursuit_turn_kp = float(self.get_parameter('pursuit_turn_kp').value)
        self.pursuit_turn_linear = float(self.get_parameter('pursuit_turn_linear_speed').value)
        self.pursuit_min_turn_radius = max(
            0.05, float(self.get_parameter('pursuit_min_turn_radius_m').value)
        )
        self.initial_align_trigger = math.radians(max(
            0.0, float(self.get_parameter('initial_align_trigger_deg').value)
        ))
        self.initial_align_tolerance = math.radians(max(
            0.1, float(self.get_parameter('initial_align_tolerance_deg').value)
        ))
        self.initial_align_angular = abs(float(
            self.get_parameter('initial_align_angular_speed').value
        ))
        self.initial_align_linear = float(
            self.get_parameter('initial_align_linear_speed').value
        )
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.min_angular = float(self.get_parameter('min_angular_speed').value)

        self.avoid_linear_speed = float(self.get_parameter('avoid_linear_speed').value)
        self.avoid_angular_speed = float(self.get_parameter('avoid_angular_speed').value)
        self.avoid_min_duration = float(self.get_parameter('avoid_min_duration_sec').value)
        self.avoid_clear_hold = float(self.get_parameter('avoid_clear_hold_sec').value)
        self.avoid_min_turn_angle = math.radians(float(self.get_parameter('avoid_min_turn_angle_deg').value))
        self.avoid_right_turn_left_obstacle_angle = math.radians(float(
            self.get_parameter('avoid_right_turn_left_obstacle_angle_deg').value
        ))
        self.avoid_goal_bias_enabled = bool(
            self.get_parameter('avoid_goal_bias_enabled').value
        )
        self.avoid_obstacle_side_penalty = float(
            self.get_parameter('avoid_obstacle_side_penalty').value
        )
        self.avoid_safe_dist = float(self.get_parameter('avoid_safe_distance').value)
        self.avoid_clear_dist = float(self.get_parameter('avoid_clear_distance').value)
        self.emergency_stop_dist = float(self.get_parameter('emergency_stop_distance').value)
        self.emergency_reverse_speed = abs(float(
            self.get_parameter('emergency_reverse_speed').value
        ))
        self.emergency_reverse_duration = max(0.0, float(
            self.get_parameter('emergency_reverse_duration_sec').value
        ))
        self.emergency_reverse_angular = abs(float(
            self.get_parameter('emergency_reverse_angular_speed').value
        ))

        self.recovery_linear = float(self.get_parameter('recovery_linear_speed').value)
        self.recovery_turn_linear = float(self.get_parameter('recovery_turn_linear_speed').value)
        self.recovery_angular = float(self.get_parameter('recovery_angular_speed').value)
        self.recovery_kp = float(self.get_parameter('recovery_heading_kp').value)
        self.recovery_max_angular = float(self.get_parameter('recovery_max_angular_speed').value)
        self.recovery_min_angular = float(self.get_parameter('recovery_min_angular_speed').value)
        self.recovery_in_place = math.radians(float(self.get_parameter('recovery_in_place_angle_deg').value))
        self.heading_tolerance = math.radians(
            float(self.get_parameter('heading_tolerance_deg').value)
        )
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
        self.emergency_window_min_x = float(self.get_parameter('emergency_window_min_x').value)
        self.emergency_window_max_x = float(self.get_parameter('emergency_window_max_x').value)
        self.emergency_window_half_width = float(self.get_parameter('emergency_window_half_width').value)
        self.emergency_min_cluster_pts = int(
            self.get_parameter('emergency_min_cluster_points').value
        )
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
        self.planner_path_deviation_replan = max(0.05, float(
            self.get_parameter('planner_path_deviation_replan_m').value
        ))
        self.planner_forbidden_rectangles_json = str(
            self.get_parameter('planner_forbidden_rectangles_json').value
        )
        self.planner_clear_rectangles_json = str(
            self.get_parameter('planner_clear_rectangles_json').value
        )
        self.planner_forbidden_reverse_speed = abs(float(
            self.get_parameter('planner_forbidden_reverse_speed').value
        ))
        self.planner_forbidden_reverse_distance = max(0.0, float(
            self.get_parameter('planner_forbidden_reverse_distance_m').value
        ))
        self.planner_forbidden_reverse_timeout = max(0.1, float(
            self.get_parameter('planner_forbidden_reverse_timeout_sec').value
        ))
        self.planner_forbidden_reverse_max_attempts = max(1, int(
            self.get_parameter('planner_forbidden_reverse_max_attempts').value
        ))
        self.test_direction = str(self.get_parameter('test_direction').value)
        self.return_direction = self._normalize_direction(self.test_direction)

        self.stage3_channel_yolo_enabled = bool(self.get_parameter('stage3_channel_yolo_enabled').value)
        self.stage3_channel_model_path = str(self.get_parameter('stage3_channel_model_path').value)
        self.stage3_channel_camera_topic = str(self.get_parameter('stage3_channel_camera_topic').value)
        self.stage3_channel_conf_thres = float(self.get_parameter('stage3_channel_conf_thres').value)
        self.stage3_channel_iou_thres = float(self.get_parameter('stage3_channel_iou_thres').value)
        self.stage3_channel_preview_path = str(self.get_parameter('stage3_channel_preview_path').value)
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

    def _goal_center(self):
        """Return the last map waypoint used only while searching for P."""
        waypoint = self.return_waypoints[-1]
        return waypoint['x'], waypoint['y']

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
                '(visual alignment only; map entry reset is direction-based)'
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

    def _clear_depth_cache(self):
        """Drop the full depth frame outside the Stage3 visual approach."""
        with self._depth_lock:
            self._last_depth_image = None
            self._last_depth_received_at = 0.0
            self._last_depth_encoding = ''

    def _set_channel_inference_active(self, active: bool):
        detector = getattr(self, '_channel_detector', None)
        if detector is not None and hasattr(detector, 'set_inference_active'):
            detector.set_inference_active(active)

    def _update_p_inference_gate(self):
        self._set_p_inference_active(
            self.phase == 3 and not self.mission_finished
        )

    @staticmethod
    def _normalize_direction(value):
        text = str(value).strip().lower()
        if any(token in text for token in ('counterclockwise', 'counter_clockwise', 'ccw', '逆时针')):
            return 'counterclockwise'
        return 'clockwise'

    def _direction_cb(self, msg):
        direction = self._normalize_direction(msg.data)
        if direction == self.return_direction:
            return
        self.return_direction = direction
        self.log.mission(f'Stage3 direction updated from QR: {self.return_direction}')

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
                self._clear_depth_cache()
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
            self._clear_depth_cache()
            return

        self.phase = incoming
        if prev == 3 and self.phase != 3:
            self._reset_mission()
            self._set_p_inference_active(False)
            self._set_stage3_http_active(False)
            self._clear_depth_cache()
        elif prev != 3 and self.phase == 3:
            self.log.start_session()
            self.log.startup('phase=3 activated; Stage3 file log session opened')
            self._set_stage3_http_active(True)
            self._arm_mission()

    def _odom_cb(self, msg):
        raw_x = float(msg.pose.pose.position.x)
        raw_y = float(msg.pose.pose.position.y)
        raw_yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        self._last_raw_odom_xy = (raw_x, raw_y)
        self._last_raw_odom_yaw = raw_yaw
        self.odom_frame_id = msg.header.frame_id or self.odom_frame_id
        if self._pending_entry_anchor_map is not None:
            pending_anchor = self._pending_entry_anchor_map
            self._pending_entry_anchor_map = None
            self._bind_stage3_entry_anchor(pending_anchor)
        if self._entry_anchor_map is not None and self._entry_anchor_odom is not None:
            self.current_position = self._position_from_entry_anchor(
                self._entry_anchor_map, self._entry_anchor_odom, self._last_raw_odom_xy,
                self._entry_anchor_map_from_odom_yaw,
            )
        else:
            # Only diagnostic/fallback before a Stage2 handoff anchor arrives.
            self.current_position = self._lookup_map_xy_from_tf()

    @staticmethod
    def _position_from_entry_anchor(
        entry_map, entry_odom, current_odom, map_from_odom_yaw,
    ):
        """Propagate the map anchor using an odometry delta rotated into map."""
        dx_odom = current_odom[0] - entry_odom[0]
        dy_odom = current_odom[1] - entry_odom[1]
        cos_yaw = math.cos(map_from_odom_yaw)
        sin_yaw = math.sin(map_from_odom_yaw)
        return (
            entry_map[0] + cos_yaw * dx_odom - sin_yaw * dy_odom,
            entry_map[1] + sin_yaw * dx_odom + cos_yaw * dy_odom,
        )

    def _stage3_entry_anchor_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.log.warn(
                'ENTRY_ANCHOR',
                f'ignored anchor frame={msg.header.frame_id}, expected={self.map_frame}',
            )
            return
        stamp_sec = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        if stamp_sec > 0.0 and self._now_sec() - stamp_sec > self.stage3_entry_anchor_max_age:
            self.log.warn(
                'ENTRY_ANCHOR',
                f'ignored stale anchor age={self._now_sec() - stamp_sec:.2f}s '
                f'max={self.stage3_entry_anchor_max_age:.2f}s',
            )
            return
        anchor_map = (float(msg.point.x), float(msg.point.y))
        self._entry_anchor_stamp_sec = stamp_sec if stamp_sec > 0.0 else self._now_sec()
        if self._last_raw_odom_xy is None:
            self._pending_entry_anchor_map = anchor_map
            self.log.mission(
                f'Stage2 entry anchor received map={anchor_map}; waiting for {self.odom_topic}'
            )
            return
        self._bind_stage3_entry_anchor(anchor_map)

    def _stage3_preplan_pose_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.log.warn('PREPLAN', f'ignored frame={msg.header.frame_id}')
            return
        self._preplan_start = (float(msg.point.x), float(msg.point.y))
        self._preplanned_path = []
        self.log.mission(
            f'Stage2 preplan start received map=({self._preplan_start[0]:.3f},'
            f'{self._preplan_start[1]:.3f})'
        )

    def _maybe_build_preplanned_path(self):
        if self.phase != 2 or self._preplan_start is None or self._preplanned_path:
            return
        if not self.use_global_planner or self.global_planner is None or not self.return_waypoints:
            return
        path = self.global_planner.plan_path(
            self._preplan_start, self._goal_center(), self._now_sec()
        )
        if path is None:
            return
        if not path:
            self.log.warn('PREPLAN', 'A* failed from predicted Stage3 start; will retry')
            return
        self._preplanned_path = list(path)
        self.log.mission(
            f'Stage3 preplan ready: points={len(path)} '
            f'start=({self._preplan_start[0]:.2f},{self._preplan_start[1]:.2f})'
        )

    def _bind_stage3_entry_anchor(self, anchor_map):
        map_from_odom_yaw = self._lookup_map_from_odom_yaw()
        if map_from_odom_yaw is None:
            self._pending_entry_anchor_map = anchor_map
            if not self._entry_anchor_tf_warned:
                self._entry_anchor_tf_warned = True
                self.log.warn(
                    'ENTRY_ANCHOR',
                    f'waiting for TF {self.map_frame}->{self.odom_frame_id} to bind Stage2 anchor',
                )
            return False

        self._entry_anchor_tf_warned = False
        self._entry_anchor_map = anchor_map
        self._entry_anchor_odom = self._last_raw_odom_xy
        self._entry_anchor_map_from_odom_yaw = map_from_odom_yaw
        self.current_position = anchor_map
        tf_position = self._lookup_map_xy_from_tf()
        self._last_tf_position = tf_position
        if tf_position is None:
            tf_text = 'tf=unavailable'
        else:
            tf_text = (
                f'tf=({tf_position[0]:.3f},{tf_position[1]:.3f}) '
                f'delta=({tf_position[0] - anchor_map[0]:+.3f},'
                f'{tf_position[1] - anchor_map[1]:+.3f})'
            )
        self.log.mission(
            f'Stage2 entry anchor bound map=({anchor_map[0]:.3f},{anchor_map[1]:.3f}) '
            f'odom=({self._entry_anchor_odom[0]:.3f},{self._entry_anchor_odom[1]:.3f}) '
            f'map_from_odom_yaw={math.degrees(map_from_odom_yaw):+.2f}deg '
            f'{tf_text}'
        )
        return True

    def _lookup_map_from_odom_yaw(self):
        """Lock map<-odom rotation at handoff; do not use odometry orientation."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame_id, Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException:
            return None
        return self._quat_to_yaw(transform.transform.rotation)

    def _lookup_map_xy_from_tf(self):
        """Use the Stage1-owned map transform so A* and the map share one origin."""
        candidates = []
        for frame in (self.base_frame, 'base_footprint', 'base_link'):
            if frame and frame not in candidates:
                candidates.append(frame)
        for frame in candidates:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, frame, Time(), timeout=Duration(seconds=0.05)
                )
            except TransformException:
                continue
            translation = transform.transform.translation
            if self._map_pose_warned:
                self.log.mission(
                    f'map pose TF recovered: {self.map_frame}->{frame}'
                )
                self._map_pose_warned = False
            return float(translation.x), float(translation.y)
        if not self._map_pose_warned:
            self._map_pose_warned = True
            self.log.warn(
                'POSE',
                f'cannot get TF {self.map_frame}->base; Stage3 position is unavailable',
            )
        return None

    def _position_source_text(self):
        if self._entry_anchor_map is None or self._entry_anchor_odom is None:
            return 'source=tf_fallback'
        return (
            'source=s2_anchor '
            f'anchor=({self._entry_anchor_map[0]:.2f},{self._entry_anchor_map[1]:.2f}) '
            f'odom0=({self._entry_anchor_odom[0]:.2f},{self._entry_anchor_odom[1]:.2f}) '
            f'odom=({self._last_raw_odom_xy[0]:.2f},{self._last_raw_odom_xy[1]:.2f}) '
            f'odom_to_map={math.degrees(self._entry_anchor_map_from_odom_yaw):+.1f}deg'
        )

    def _imu_cb(self, msg):
        self._imu_yaw = self._quat_to_yaw(msg.orientation)
        if self._awaiting_entry_yaw_alignment:
            self._imu_yaw_offset = self._normalize_angle(
                self.stage3_entry_map_yaw - self._imu_yaw
            )
            self._awaiting_entry_yaw_alignment = False
            self.log.mission(
                'Stage3 IMU aligned at handoff: '
                f'raw={math.degrees(self._imu_yaw):.1f}deg '
                f'map={math.degrees(self.stage3_entry_map_yaw):.1f}deg '
                f'offset={math.degrees(self._imu_yaw_offset):+.1f}deg'
            )
        self.current_yaw = self._normalize_angle(self._imu_yaw + self._imu_yaw_offset)

    def _depth_cb(self, msg):
        """Cache the newest RGB-aligned Aurora depth frame for P diagnostics."""
        if self.phase != 3 or not self.p_depth_logging_enabled:
            return
        try:
            depth = self._depth_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth is None or depth.ndim != 2:
                return
            with self._depth_lock:
                self._last_depth_image = np.asarray(depth).copy()
                self._last_depth_received_at = time.time()
                self._last_depth_encoding = str(msg.encoding)
        except Exception as exc:
            self.log.warn('P_DEPTH', f'cannot decode depth frame: {exc}')

    def _scan_cb(self, msg):
        self.latest_scan = msg

    # ══════════════ 任务生命周期 ══════════════

    def _arm_mission(self):
        self.mission_active = False
        self.mission_finished = False
        self._imu_yaw_offset = 0.0
        # S2's final handoff line is fixed along map -Y.  Establish the map
        # heading from IMU at handoff, then retain IMU-only relative heading.
        self._awaiting_entry_yaw_alignment = True
        if self._imu_yaw is not None:
            self._imu_yaw_offset = self._normalize_angle(
                self.stage3_entry_map_yaw - self._imu_yaw
            )
            self.current_yaw = self.stage3_entry_map_yaw
            self._awaiting_entry_yaw_alignment = False
            self.log.mission(
                'Stage3 IMU aligned at handoff: '
                f'raw={math.degrees(self._imu_yaw):.1f}deg '
                f'map={math.degrees(self.stage3_entry_map_yaw):.1f}deg '
                f'offset={math.degrees(self._imu_yaw_offset):+.1f}deg'
            )
        self.path_started_at = None
        self.path_index = 0
        self._settled_start = None
        self._planner_reverse_start = None
        self._planner_reverse_started_at = None
        self._planner_forbidden_reverse_attempts = 0
        self._active_planned_path = list(self._preplanned_path)
        self._active_path_cursor = 0
        self._visual_search_gate_reached = False
        self._initial_align_required = False
        if self._entry_anchor_map is not None and self._entry_anchor_odom is not None:
            self.current_position = self._position_from_entry_anchor(
                self._entry_anchor_map, self._entry_anchor_odom, self._last_raw_odom_xy,
                self._entry_anchor_map_from_odom_yaw,
            )
        else:
            self.current_position = self._lookup_map_xy_from_tf()
        if self.current_position is None:
            self.log.warn('POSE', 'phase=3 waiting for map<-base_footprint TF')
        self.avoid_state = 'forward'
        self.desired_heading = None
        self.recovery_uses_heading = False
        self.start_after_time = self._now_sec() + self.start_delay_sec
        self._p_approaching = False
        self._p_consecutive_hits = 0
        self._p_last_confirmed_stamp = None
        self._p_offset_filtered = 0.0
        self._p_target_yaw = None
        self._p_heading_lock_offset = None
        self._p_last_heading_reacquire_at = None
        self._p_lost_since = None
        self._p_lost_reverse_started_at = None
        self._p_last_angular = 0.0
        self._p_final_approach_latched = False
        self._p_visible_yaw_history.clear()
        self._p_recovery_target_yaw = None
        self._p_avoidance_recovery_yaw = None
        self._set_p_inference_active(True)
        # Phase3 直接进入返程；P YOLO 从本阶段开始即可确认并接管。
        self._set_channel_inference_active(False)
        self._pre_return_state = 'done'
        self._pre_return_started_at = self._now_sec()
        self._channel_hits = 0
        self._channel_offset_filtered = 0.0
        self._publish_state('armed')
        self.log.mission(
            f'phase=3 detected, direction={self.return_direction}, '
            f'tf_map={self.current_position}; '
            'starting return; P YOLO may take over immediately after confirmation'
        )

    def _reset_mission(self):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = False
        self._awaiting_entry_yaw_alignment = False
        self._imu_yaw_offset = 0.0
        self.start_after_time = None
        self.path_started_at = None
        self.path_index = 0
        self._settled_start = None
        self._planner_reverse_start = None
        self._planner_reverse_started_at = None
        self._planner_forbidden_reverse_attempts = 0
        self._active_planned_path = []
        self._active_path_cursor = 0
        self._visual_search_gate_reached = False
        self._initial_align_required = False
        self.avoid_state = 'forward'
        self.desired_heading = None
        self.recovery_uses_heading = False
        self._p_approaching = False
        self._p_consecutive_hits = 0
        self._p_last_confirmed_stamp = None
        self._p_offset_filtered = 0.0
        self._p_target_yaw = None
        self._p_heading_lock_offset = None
        self._p_last_heading_reacquire_at = None
        self._p_lost_since = None
        self._p_lost_reverse_started_at = None
        self._p_last_angular = 0.0
        self._p_final_approach_latched = False
        self._p_visible_yaw_history.clear()
        self._p_recovery_target_yaw = None
        self._p_avoidance_recovery_yaw = None
        self._pre_return_state = 'idle'
        self._channel_hits = 0
        self._channel_offset_filtered = 0.0
        self._set_channel_inference_active(False)
        self._set_p_inference_active(False)
        detector = getattr(self, '_p_detector', None)
        if detector is not None and hasattr(detector, 'release_model'):
            detector.release_model('phase_exit')
        self._publish_state('idle')

    def _start_mission(self):
        if self.require_stage3_entry_anchor and (
            self._entry_anchor_map is None or self._entry_anchor_odom is None
        ):
            self.stop_robot()
            self._publish_state('waiting_for_stage2_anchor')
            return
        if self.current_position is None or self.current_yaw is None:
            self.log.warn('POSE', 'cannot start: waiting for map TF and IMU yaw')
            return
        if not self.return_waypoints:
            self._publish_feedback('no waypoints configured, cannot start')
            self._fail_mission('no return waypoints')
            return
        self.mission_active = True
        self.path_started_at = self._now_sec()
        self.path_index = 0
        target_x, target_y = self._goal_center()
        target_yaw = math.atan2(
            target_y - self.current_position[1], target_x - self.current_position[0]
        )
        initial_error = self._angle_error(target_yaw, self.current_yaw)
        self._initial_align_required = abs(initial_error) >= self.initial_align_trigger
        self._publish_state('initial_align' if self._initial_align_required else 'running')
        self._update_p_inference_gate()
        self.log.mission(
            f'return started, {len(self.return_waypoints)} waypoints (map coords), '
            f'current=({self.current_position[0]:.2f},{self.current_position[1]:.2f}) '
            f'yaw={math.degrees(self.current_yaw):.1f}° '
            f'goal_yaw={math.degrees(target_yaw):.1f}° '
            f'initial_error={math.degrees(initial_error):+.1f}° '
            f'pre_align={self._initial_align_required} {self._position_source_text()}'
        )
        self.get_logger().info(f'mission_active=True, will publish cmd_vel now')

    def _finish_mission(self, feedback_text='return complete, reached P point'):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self._set_p_inference_active(False)
        detector = getattr(self, '_p_detector', None)
        if detector is not None and hasattr(detector, 'release_model'):
            detector.release_model('mission_complete')
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

    def _run_pre_return_handoff(self):
        if not self.stage3_channel_yolo_enabled or self._channel_detector is None:
            self._pre_return_state = 'done'
            return True
        now = self._now_sec()
        started = self._pre_return_started_at or now
        if now - started > self.stage3_channel_timeout_sec:
            self.log.warn('CHANNEL_YOLO', 'pre-return timeout, preserving entry map reset and starting return')
            self._set_channel_inference_active(False)
            self._pre_return_state = 'done'
            return True

        self._publish_state('pre_return_channel_yolo')
        if self._pre_return_state != 'yolo_centering':
            self._pre_return_state = 'yolo_centering'
            self._channel_hits = 0
            self._channel_offset_filtered = 0.0
            self._set_channel_inference_active(True)
            self.stop_robot()
            self.log.mission('pre-return channel YOLO started without IMU yaw alignment')
            return False
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
        centered = (
            abs(filt) <= self.stage3_channel_offset_tolerance
            and fill_ratio >= self.stage3_channel_fill_ratio
        )
        self._channel_hits = self._channel_hits + 1 if centered else 0
        if self._channel_hits >= self.stage3_channel_consecutive_hits:
            self.stop_robot()
            self._set_channel_inference_active(False)
            self._pre_return_state = 'done'
            self.log.mission(
                f'pre-return channel center reached conf={conf:.2f} off={filt:+.3f} '
                f'fill={fill_ratio:.2%}; preserving entry map reset and starting return'
            )
            return True

        if abs(filt) <= self.stage3_channel_offset_deadband:
            angular = 0.0
        else:
            effective = math.copysign(abs(filt) - self.stage3_channel_offset_deadband, filt)
            angular = -self.stage3_channel_angular_kp * effective
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
                f'{self.stage3_channel_consecutive_hits} '
                f'v={speed:.2f} w={angular:.2f}'
            )
        return False

    # ══════════════ 主控制循环 ══════════════

    def _control_loop(self):
        if self.phase == 2:
            self._maybe_build_preplanned_path()
            return
        if self.phase != 3 or self.mission_finished:
            return

        now = self._now_sec()
        if not self.mission_active:
            if self.start_after_time is None or now < self.start_after_time:
                return
            self._start_mission()
            return

        # 1. 紧急近障时倒车脱离，再重新进入常规避障。
        if self._check_emergency_stop():
            return

        # 2. 雷达避障高于 P 视觉和 A* 导航；P 终段可按深度禁用避障。
        if self.avoid_state == 'forward' and self.latest_scan is not None:
            self._check_obstacle()

        if self.avoid_state != 'forward':
            self._run_avoidance()
            return

        # 3. 大航向差先按目标方位摆正，避免直接单点追踪走出大弧。
        if self._initial_align_required:
            self._run_initial_align()
            return

        # 4. Phase3 开始即允许 P YOLO 确认并接管。
        self._update_p_inference_gate()
        self._update_p_detection()
        if self._p_approaching:
            self._run_p_approach()
            return

        # 5. 尚未识别 P，才允许使用漂移敏感的 map/A* 粗导航。
        if self.current_position is None:
            self.stop_robot()
            self._publish_state('waiting_for_map_tf')
            return
        if self.use_global_planner and self._run_planner_forbidden_reverse():
            return
        self._run_center_drive()

    def _check_emergency_stop(self):
        if self.avoid_state == 'emergency_reversing':
            self._run_emergency_reverse()
            return True
        if self.latest_scan is None:
            return False
        obstacle = self._find_nearest_obstacle(self.latest_scan, emergency=True)
        if obstacle is not None and obstacle['dist'] <= self.emergency_stop_dist:
            self._begin_emergency_reverse(obstacle)
            self._run_emergency_reverse()
            return True
        return False

    def _begin_emergency_reverse(self, obstacle):
        self.avoid_state = 'emergency_reversing'
        self.avoid_turn_direction, selection_detail = self._choose_avoid_turn_direction(
            obstacle['danger_deg']
        )
        self.emergency_reverse_deadline = self.get_clock().now() + Duration(
            seconds=self.emergency_reverse_duration
        )
        self._publish_state('emergency_reversing')
        self.log.warn(
            'EMERGENCY_AVOID',
            f'near obstacle dist={obstacle["dist"]:.2f}m danger='
            f'{obstacle["danger_deg"]:.1f}deg; reverse for '
            f'{self.emergency_reverse_duration:.2f}s {selection_detail}',
        )
        self._publish_feedback(
            f'emergency obstacle {obstacle["dist"]:.2f}m: reverse and replan avoidance'
        )

    def _run_emergency_reverse(self):
        if (
            self.emergency_reverse_deadline is not None
            and self.get_clock().now() < self.emergency_reverse_deadline
        ):
            self.cmd_pub.publish(self._twist(
                -self.emergency_reverse_speed,
                -self.avoid_turn_direction * self.emergency_reverse_angular,
            ))
            return
        self.avoid_state = 'forward'
        self.emergency_reverse_deadline = None
        self._publish_state('running')
        self.log.mission('emergency reverse complete; rechecking lidar avoidance')

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # ══════════════ 地图粗导航 + P 视觉最终到达 ══════════════

    def _run_initial_align(self):
        """Turn toward the single map goal before enabling P takeover or pursuit."""
        if self.current_position is None or self.current_yaw is None:
            self.stop_robot()
            self._publish_state('waiting_for_map_tf')
            return

        target_x, target_y = self._goal_center()
        target_yaw = math.atan2(
            target_y - self.current_position[1], target_x - self.current_position[0]
        )
        heading_error = self._angle_error(target_yaw, self.current_yaw)
        self.desired_heading = target_yaw
        if abs(heading_error) <= self.initial_align_tolerance:
            self._initial_align_required = False
            self._filtered_heading_err = 0.0
            self.stop_robot()
            self._publish_state('running')
            self.log.mission(
                f'initial align complete: target={math.degrees(target_yaw):.1f}° '
                f'yaw={math.degrees(self.current_yaw):.1f}° '
                f'error={math.degrees(heading_error):+.1f}°'
            )
            return

        angular = math.copysign(self.initial_align_angular, heading_error)
        self._publish_state('initial_align')
        self.log.telemetry(
            'INITIAL_ALIGN',
            f'target={math.degrees(target_yaw):.1f}° '
            f'yaw={math.degrees(self.current_yaw):.1f}° '
            f'error={math.degrees(heading_error):+.1f}° '
            f'v={self.initial_align_linear:.2f} w={angular:.2f}'
        )
        self.cmd_pub.publish(self._twist(self.initial_align_linear, angular))

    def _p_detection(self):
        if self._p_detector is None:
            return False, 0.0, None, 0.0, 0.0, 0.0
        detected, conf, bbox, stamp, offset, fill = self._p_detector.get_p_detection_geometry()
        fresh = time.time() - float(stamp or 0.0) <= self.p_detection_timeout
        return bool(detected and bbox is not None and fresh), conf, bbox, stamp, offset, fill

    def _p_depth_measurement(self, bbox):
        """Return a robust depth median from the center of the detected P box."""
        if not self.p_depth_logging_enabled:
            return None, 0, 'disabled'
        if bbox is None:
            return None, 0, 'no_bbox'
        with self._depth_lock:
            depth = self._last_depth_image
            received_at = self._last_depth_received_at
            encoding = self._last_depth_encoding
            if depth is not None:
                depth = depth.copy()
        if depth is None:
            return None, 0, 'no_depth_frame'
        age = time.time() - received_at
        if age > self.p_depth_max_age:
            return None, 0, f'stale:{age:.2f}s'

        # P detector currently uses the complete RGB frame. Aurora aligned
        # depth has the same 640x400 geometry, so index it directly.
        height, width = depth.shape
        x1, y1, x2, y2 = bbox
        box_w = max(1, x2 - x1)
        box_h = max(1, y2 - y1)
        inset_x = int(box_w * (1.0 - self.p_depth_roi_fraction) / 2.0)
        inset_y = int(box_h * (1.0 - self.p_depth_roi_fraction) / 2.0)
        left = max(0, min(width, x1 + inset_x))
        right = max(0, min(width, x2 - inset_x))
        top = max(0, min(height, y1 + inset_y))
        bottom = max(0, min(height, y2 - inset_y))
        if right <= left or bottom <= top:
            return None, 0, f'bad_roi:{encoding}'

        values_m = depth[top:bottom, left:right].astype(np.float32).reshape(-1)
        values_m *= self.p_depth_unit_scale_m
        valid = values_m[np.isfinite(values_m)]
        valid = valid[(valid >= self.p_depth_min_m) & (valid <= self.p_depth_max_m)]
        if valid.size == 0:
            return None, 0, f'no_valid:{encoding},age={age:.2f}s'
        return float(np.median(valid)), int(valid.size), f'{encoding},age={age:.2f}s'

    def _p_depth_text(self, bbox):
        depth_m, samples, status = self._p_depth_measurement(bbox)
        if depth_m is None:
            return f'depth=invalid samples={samples} ({status})'
        return f'depth={depth_m:.3f}m samples={samples} ({status})'

    def _record_p_visible_yaw(self, now):
        if self.current_yaw is None:
            return
        self._p_visible_yaw_history.append((now, self.current_yaw))
        history_limit = now - max(3.0, self.p_loss_heading_lookback + 1.0)
        while self._p_visible_yaw_history and self._p_visible_yaw_history[0][0] < history_limit:
            self._p_visible_yaw_history.popleft()

    def _p_heading_before_loss(self, now):
        if self._p_avoidance_recovery_yaw is not None:
            return self._p_avoidance_recovery_yaw
        if self._p_target_yaw is not None:
            return self._p_target_yaw
        desired_time = now - self.p_loss_heading_lookback
        for stamp, yaw in reversed(self._p_visible_yaw_history):
            if stamp <= desired_time:
                return yaw
        if self._p_visible_yaw_history:
            return self._p_visible_yaw_history[0][1]
        return self.current_yaw

    def _update_p_detection(self):
        detected, conf, bbox, stamp, offset, _fill = self._p_detection()
        confirmed = (
            self.phase == 3
            and detected
            and conf >= self.p_approach_conf
        )
        if not confirmed:
            self._p_consecutive_hits = 0
            self._p_last_confirmed_stamp = None
            return
        if stamp != self._p_last_confirmed_stamp:
            self._p_consecutive_hits += 1
            self._p_last_confirmed_stamp = stamp
        if self._p_consecutive_hits < self.p_approach_hits_required:
            return
        if self._p_approaching:
            return
        self._p_approaching = True
        self._p_offset_filtered = float(offset)
        self._p_target_yaw = self._visual_target_yaw(offset)
        self._p_heading_lock_offset = self._p_offset_filtered
        self._p_last_heading_reacquire_at = self._now_sec()
        self.desired_heading = self._p_target_yaw
        self._p_lost_since = None
        self._p_lost_reverse_started_at = None
        self._p_last_angular = 0.0
        self._p_visible_yaw_history.clear()
        self._p_recovery_target_yaw = None
        self._publish_state('p_approach')
        map_pos = self.current_position
        map_text = (
            f'map=({map_pos[0]:.2f},{map_pos[1]:.2f})'
            if map_pos is not None else 'map=unavailable'
        )
        self.log.segment(
            f'P acquired conf={conf:.2f} offset={offset:+.3f}; '
            f'heading_lock={math.degrees(self._p_target_yaw):.1f}deg '
            f'visual approach v={self.p_approach_linear:.2f} {map_text} '
            f'{self._position_source_text()} {self._p_depth_text(bbox)}'
        )
        self._publish_feedback('P acquired, visual final approach started')

    def _run_p_approach(self):
        detected, conf, bbox, _stamp, offset, fill = self._p_detection()
        if not detected or conf < self.p_approach_conf:
            now = self._now_sec()
            if self._p_lost_since is None:
                self._p_lost_since = now
                self._p_lost_reverse_started_at = now
                self._p_recovery_target_yaw = self._p_heading_before_loss(now)
                self.desired_heading = self._p_recovery_target_yaw
                target_deg = (
                    math.degrees(self._p_recovery_target_yaw)
                    if self._p_recovery_target_yaw is not None else float('nan')
                )
                self.log.warn(
                    'P_DETECTION',
                    'P lost after acquisition: reversing toward the locked visual heading '
                    f'({target_deg:.1f}deg)'
                )
                self._publish_feedback('P lost: reverse toward the locked visual heading')

            reverse_elapsed = now - self._p_lost_reverse_started_at
            target_yaw = self._p_recovery_target_yaw
            heading_error = (
                self._angle_error(target_yaw, self.current_yaw)
                if target_yaw is not None and self.current_yaw is not None else 0.0
            )
            heading_restored = abs(heading_error) <= self.p_loss_heading_tolerance
            if reverse_elapsed < self.p_loss_reverse_duration and not heading_restored:
                reverse_angular = self._clamp(
                    self.p_loss_heading_kp * heading_error,
                    self.p_loss_reverse_max_angular,
                )
                self._publish_state('p_visual_recover')
                self.log.telemetry(
                    'P_RECOVER',
                    f'loss_t={reverse_elapsed:.2f}s v={-self.p_loss_reverse_speed:.2f} '
                    f'target={math.degrees(target_yaw):.1f}° '
                    f'err={math.degrees(heading_error):+.1f}° w={reverse_angular:.2f}'
                )
                self.cmd_pub.publish(self._twist(-self.p_loss_reverse_speed, reverse_angular))
            else:
                self._resume_map_search_after_p_loss(
                    reverse_elapsed, heading_restored, heading_error,
                )
            return

        self._p_lost_since = None
        self._p_lost_reverse_started_at = None
        self._p_recovery_target_yaw = None
        self._p_avoidance_recovery_yaw = None
        self._record_p_visible_yaw(self._now_sec())
        alpha = self.p_approach_offset_filter_alpha
        self._p_offset_filtered = alpha * float(offset) + (1.0 - alpha) * self._p_offset_filtered
        now = self._now_sec()
        reacquire_due = (
            self._p_heading_lock_offset is None
            or abs(self._p_offset_filtered - self._p_heading_lock_offset)
            >= self.p_heading_reacquire_offset
        ) and (
            self._p_last_heading_reacquire_at is None
            or now - self._p_last_heading_reacquire_at >= self.p_heading_reacquire_interval
        )
        if self._p_target_yaw is None or reacquire_due:
            self._p_target_yaw = self._visual_target_yaw(self._p_offset_filtered)
            self._p_heading_lock_offset = self._p_offset_filtered
            self._p_last_heading_reacquire_at = now
            self.log.segment(
                f'P heading reacquire offset={self._p_offset_filtered:+.3f} '
                f'target={math.degrees(self._p_target_yaw):.1f}deg'
            )
        self.desired_heading = self._p_target_yaw
        heading_error = self._angle_error(self._p_target_yaw, self.current_yaw)
        angular = self._clamp(
            self.p_heading_kp * heading_error,
            self.p_heading_max_angular,
        )
        if abs(heading_error) <= self.p_heading_tolerance:
            angular = 0.0
        linear = self.p_approach_linear
        self._p_last_angular = angular
        depth_m, samples, depth_status = self._p_depth_measurement(bbox)
        if (
            depth_m is not None
            and depth_m <= self.p_approach_disable_avoidance_distance
        ):
            self._p_final_approach_latched = True
        if depth_m is not None and depth_m < self.p_depth_stop_distance:
            self.log.segment(
                f'P depth stop: depth={depth_m:.3f}m < '
                f'{self.p_depth_stop_distance:.3f}m samples={samples} ({depth_status})'
            )
            self._finish_mission('return complete, P depth stop threshold reached')
            return
        self._publish_state('p_approach')
        self.log.telemetry(
            'P_APPROACH',
            f'conf={conf:.2f} raw_off={offset:+.3f} off={self._p_offset_filtered:+.3f} '
            f'fill={fill:.2%} target={math.degrees(self._p_target_yaw):.1f}deg '
            f'err={math.degrees(heading_error):+.1f}deg spd={linear:.2f} ang={angular:.2f} '
            f'{self._p_depth_text(bbox)}',
        )
        self.cmd_pub.publish(self._twist(linear, angular))

    def _resume_map_search_after_p_loss(self, reverse_elapsed, heading_restored, heading_error):
        """Return to the calibrated search goal when an obstacle hides P."""
        target_deg = (
            math.degrees(self._p_recovery_target_yaw)
            if self._p_recovery_target_yaw is not None else float('nan')
        )
        self._p_approaching = False
        self._p_consecutive_hits = 0
        self._p_last_confirmed_stamp = None
        self._p_target_yaw = None
        self._p_heading_lock_offset = None
        self._p_last_heading_reacquire_at = None
        self._p_lost_since = None
        self._p_lost_reverse_started_at = None
        self._p_recovery_target_yaw = None
        self._p_avoidance_recovery_yaw = None
        self._p_final_approach_latched = False
        self._p_visible_yaw_history.clear()
        self.desired_heading = None
        self._filtered_heading_err = 0.0
        recovery_reason = 'heading_restored' if heading_restored else 'reverse_timeout'
        self._publish_state('map_search_p')
        self.log.mission(
            f'P lost recovery complete ({recovery_reason}): elapsed={reverse_elapsed:.2f}s '
            f'target={target_deg:.1f}deg error={math.degrees(heading_error):+.1f}deg; '
            'resume lidar-protected navigation to visual search goal'
        )
        self._publish_feedback('P occluded: resume navigation to visual search goal')

    def _visual_target_yaw(self, offset):
        """Turn one P-frame offset into a heading that IMU can hold straight."""
        base_yaw = self.current_yaw if self.current_yaw is not None else 0.0
        return self._normalize_angle(base_yaw - self.p_heading_bearing_gain * float(offset))

    def _start_planner_forbidden_reverse(self):
        if self.current_position is None:
            return True
        self._planner_forbidden_reverse_attempts += 1
        if self._planner_forbidden_reverse_attempts > self.planner_forbidden_reverse_max_attempts:
            self.stop_robot()
            self._publish_state('planner_forbidden_recovery_failed')
            self._publish_feedback('planner forbidden recovery failed: repeated forbidden position')
            self.log.warn(
                'PLANNER',
                f'forbidden recovery exhausted attempts={self._planner_forbidden_reverse_attempts - 1}/'
                f'{self.planner_forbidden_reverse_max_attempts}; {self._planner_pose_diagnostic()}'
            )
            self.mission_active = False
            self.mission_finished = True
            return True
        self._planner_reverse_start = self.current_position
        self._planner_reverse_started_at = self._now_sec()
        self._filtered_heading_err = 0.0
        # 禁区恢复只允许急停打断，不能与通用避障状态机并发控制 cmd_vel。
        self.avoid_state = 'forward'
        self._publish_state('planner_forbidden_reverse')
        self.log.warn(
            'PLANNER',
            f'position in forbidden planner cell at ({self.current_position[0]:.2f},'
            f'{self.current_position[1]:.2f}); reversing {self.planner_forbidden_reverse_distance:.2f}m; '
            f'attempt={self._planner_forbidden_reverse_attempts}/'
            f'{self.planner_forbidden_reverse_max_attempts}; {self._planner_pose_diagnostic()}',
        )
        return False

    def _run_planner_forbidden_reverse(self):
        if self._planner_reverse_start is None or self._planner_reverse_started_at is None:
            return False
        if self.current_position is None:
            self.stop_robot()
            return True
        moved = math.hypot(
            self.current_position[0] - self._planner_reverse_start[0],
            self.current_position[1] - self._planner_reverse_start[1],
        )
        elapsed = self._now_sec() - self._planner_reverse_started_at
        if moved >= self.planner_forbidden_reverse_distance:
            self.stop_robot()
            self.log.mission(
                f'planner forbidden reverse complete: moved={moved:.2f}m; replanning; '
                f'{self._planner_pose_diagnostic()}'
            )
            self._planner_reverse_start = None
            self._planner_reverse_started_at = None
            return False
        if elapsed >= self.planner_forbidden_reverse_timeout:
            self.stop_robot()
            self._fail_mission('planner forbidden reverse timeout')
            return True
        self.cmd_pub.publish(self._twist(-self.planner_forbidden_reverse_speed, 0.0))
        return True

    def _planner_pose_diagnostic(self):
        """Describe the live-TF pose used by forbidden-zone recovery."""
        occupancy = 'planner_unavailable'
        if self.global_planner is not None and self.current_position is not None:
            occupancy = self.global_planner.describe_world_occupancy(self.current_position)
        return (
            f'pose_source=tf_map odom_now={self._last_raw_odom_xy} '
            f'map_now={self.current_position} occupancy={occupancy}'
        )

    def _path_deviation(self, path_points):
        if self.current_position is None or not path_points:
            return float('inf')
        start = min(self._active_path_cursor, len(path_points) - 1)
        return min(
            math.hypot(point[0] - self.current_position[0], point[1] - self.current_position[1])
            for point in path_points[start:]
        )

    def _select_stable_lookahead_point(self, path_points):
        """Advance monotonically on one cached A* path and select a safe lookahead."""
        if self.current_position is None or self.global_planner is None or not path_points:
            return None
        start = min(self._active_path_cursor, len(path_points) - 1)
        nearest_index = min(
            range(start, len(path_points)),
            key=lambda index: math.hypot(
                path_points[index][0] - self.current_position[0],
                path_points[index][1] - self.current_position[1],
            ),
        )
        self._active_path_cursor = max(self._active_path_cursor, nearest_index)

        selected = path_points[nearest_index]
        traveled = 0.0
        previous = selected
        for index in range(nearest_index + 1, len(path_points)):
            point = path_points[index]
            traveled += math.hypot(point[0] - previous[0], point[1] - previous[1])
            if not self.global_planner.is_world_segment_free(self.current_position, point):
                break
            selected = point
            if traveled >= self.pursuit_lookahead:
                break
            previous = point
        return selected


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

        if self.use_global_planner and self._run_planner_forbidden_reverse():
            return

        # P 未识别时只驶向已标定的视觉搜索入口。入口处停车等待
        # 视觉接管，避免用漂移的里程计继续追逐 P 的名义 map 中心。
        target_x, target_y = self._goal_center()
        gate_dist = math.hypot(
            target_x - self.current_position[0], target_y - self.current_position[1]
        )
        if gate_dist <= self.waypoint_tolerance:
            if not self._visual_search_gate_reached:
                self._visual_search_gate_reached = True
                self._filtered_heading_err = 0.0
                self.log.mission(
                    f'visual search gate reached map=({self.current_position[0]:.2f},'
                    f'{self.current_position[1]:.2f}) target=({target_x:.2f},'
                    f'{target_y:.2f}) dist={gate_dist:.2f}m; waiting for P vision'
                )
            self._publish_state('visual_search_wait')
            self.stop_robot()
            return

        if self.use_global_planner:
            if self.global_planner is None:
                self.log.warn('PLANNER', 'global planner requested but unavailable')
                self._publish_state('planner_unavailable')
                self.stop_robot()
                return
            deviation = self._path_deviation(self._active_planned_path)
            if self._active_planned_path and deviation > self.planner_path_deviation_replan:
                self.log.warn(
                    'PLANNER',
                    f'path deviation {deviation:.2f}m exceeds '
                    f'{self.planner_path_deviation_replan:.2f}m; replanning',
                )
                self._active_planned_path = []
                self._active_path_cursor = 0
                self._filtered_heading_err = 0.0

            planned_points = self._active_planned_path
            if not planned_points:
                planned_points = self.global_planner.plan_path(
                    self.current_position, (target_x, target_y), now
                )
            if planned_points is None:
                self._publish_state('planner_waiting_for_map')
                self.stop_robot()
                return
            if not planned_points:
                position_is_free = self.global_planner.is_world_segment_free(
                    self.current_position, self.current_position
                )
                if position_is_free is False:
                    if self._start_planner_forbidden_reverse():
                        return
                    self._run_planner_forbidden_reverse()
                    return
                self.log.warn(
                    'PLANNER',
                    f'blocked: no free path from ({self.current_position[0]:.2f},'
                    f'{self.current_position[1]:.2f}) to ({target_x:.2f},{target_y:.2f})'
                )
                self._publish_state('planner_blocked')
                self.stop_robot()
                return
            if not self._active_planned_path:
                self._active_planned_path = list(planned_points)
                self._active_path_cursor = 0
                self.log.mission(
                    f'stable A* path accepted: points={len(planned_points)} '
                    f'start=({self.current_position[0]:.2f},{self.current_position[1]:.2f}) '
                    f'goal=({target_x:.2f},{target_y:.2f})'
                )
            self._planner_forbidden_reverse_attempts = 0
            lookahead_point = self._select_stable_lookahead_point(self._active_planned_path)
            if lookahead_point is None:
                self.log.warn('PLANNER', 'no collision-free local segment on A* path')
                self._publish_state('planner_local_segment_blocked')
                self.stop_robot()
                return
            if lookahead_point is not None:
                target_x, target_y = lookahead_point
            self.log.telemetry(
                'ASTAR',
                f'path_pts={len(self._active_planned_path)} cursor={self._active_path_cursor} '
                f'lookahead=({target_x:.2f},{target_y:.2f})'
            )

        # ── 计算目标相对车体坐标 ──
        dx = target_x - self.current_position[0]
        dy = target_y - self.current_position[1]
        if abs(dx) + abs(dy) > 1e-6:
            # Match S1 corridor recovery: preserve the active navigation-leg
            # heading while the fixed four-state avoidance maneuver runs.
            self.desired_heading = math.atan2(dy, dx)
        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)
        tx = cos_y * dx + sin_y * dy
        ty = -sin_y * dx + cos_y * dy
        target_dist = math.hypot(tx, ty)
        heading_err = math.atan2(ty, tx if abs(tx) > 1e-6 else 1e-6)

        # ── 航向误差环形低通滤波（跨 +/-180 度时不走长路）──
        alpha = 0.3
        self._filtered_heading_err = self._normalize_angle(
            self._filtered_heading_err + alpha * self._angle_error(
                heading_err, self._filtered_heading_err
            )
        )
        heading_err = self._filtered_heading_err

        self._publish_state('map_search_p')

        angular = self._clamp(self.pursuit_turn_kp * heading_err, self.max_angular)
        if abs(heading_err) > math.radians(5.0) and abs(angular) < self.min_angular:
            angular = math.copysign(self.min_angular, heading_err)
        elif abs(angular) < 1e-4:
            angular = 0.0

        speed = self.pursuit_linear_speed
        turn_mode = 'track'
        if abs(heading_err) > math.radians(5.0):
            # This is an Ackermann chassis: a near-zero linear speed with a
            # saturated angular command only chatters the steering.  Couple
            # speed to yaw rate so v / |w| never asks for an unrealistically
            # tight turning radius, including during large heading recovery.
            arc_speed = abs(angular) * self.pursuit_min_turn_radius
            speed = max(self.pursuit_turn_linear, min(self.pursuit_linear_speed, arc_speed))
            if abs(heading_err) > self.pursuit_heading_stop:
                turn_mode = 'arc_align'
            else:
                turn_mode = 'arc_track'

        self.log.telemetry('MAP_SEARCH_P',
            f'dist={target_dist:.2f} '
            f'err={math.degrees(heading_err):.1f}° '
            f'mode={turn_mode} spd={speed:.2f} ang={angular:.2f} '
            f'pos=({self.current_position[0]:.2f},{self.current_position[1]:.2f}) '
            f'center=({target_x:.2f},{target_y:.2f}) {self._position_source_text()}'
        )
        self.cmd_pub.publish(self._twist(speed, angular))

    # ══════════════ 避障状态（同 Stage1）═════════════
    def _clusters_in_window(self, scan_msg):
        return self._clusters_in_window_with_limits(
            scan_msg,
            self.window_min_x,
            self.window_max_x,
            self.window_half_width,
        )

    def _find_nearest_obstacle(self, scan_msg, emergency=False):
        """S1-style nearest cluster detection; static walls are valid obstacles."""
        if emergency:
            clusters = self._clusters_in_window_with_limits(
                scan_msg,
                self.emergency_window_min_x,
                self.emergency_window_max_x,
                self.emergency_window_half_width,
            )
            min_points = self.emergency_min_cluster_pts
        else:
            clusters = self._clusters_in_window(scan_msg)
            min_points = self.min_cluster_pts

        best = None
        for cluster in clusters:
            if len(cluster) < min_points:
                continue
            nearest = min(cluster, key=lambda point: point[2])
            span = math.hypot(
                cluster[0][0] - cluster[-1][0],
                cluster[0][1] - cluster[-1][1],
            )
            if not emergency and (span < self.min_cluster_w or span > self.max_cluster_w):
                continue
            center_x = sum(point[0] for point in cluster) / len(cluster)
            center_y = sum(point[1] for point in cluster) / len(cluster)
            candidate = {
                'dist': float(nearest[2]),
                'danger_deg': math.degrees(math.atan2(center_y, max(center_x, 1e-6))),
                'width': span,
                'pts': len(cluster),
            }
            if best is None or candidate['dist'] < best['dist']:
                best = candidate
        return best

    def _clusters_in_window_with_limits(self, scan_msg, min_x, max_x, half_width):
        """Collect scan clusters using the requested S1 detection window."""
        clusters = []
        current = []
        previous = None
        for index, distance in enumerate(scan_msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance < self.min_range:
                if current:
                    clusters.append(current)
                current = []
                previous = None
                continue
            angle = scan_msg.angle_min + index * scan_msg.angle_increment
            point = (
                distance * math.cos(angle),
                distance * math.sin(angle),
                distance,
            )
            if point[0] < min_x or point[0] > max_x or abs(point[1]) > half_width:
                if current:
                    clusters.append(current)
                current = []
                previous = None
                continue
            if previous is None or math.hypot(point[0] - previous[0], point[1] - previous[1]) <= self.cluster_gap:
                current.append(point)
            else:
                if current:
                    clusters.append(current)
                current = [point]
            previous = point
        if current:
            clusters.append(current)
        return clusters

    def _check_obstacle(self):
        if self._p_approach_avoidance_disabled():
            return
        obs = self._find_nearest_obstacle(self.latest_scan)
        if obs is not None and obs['dist'] < self.avoid_safe_dist:
            self.log.segment(
                f'lidar obstacle trigger: dist={obs["dist"]:.2f}m '
                f'danger={obs["danger_deg"]:.1f}deg width={obs["width"]:.2f}m '
                f'points={obs["pts"]} threshold={self.avoid_safe_dist:.2f}m'
            )
            self._begin_avoidance(obs['danger_deg'])

    def _p_approach_avoidance_disabled(self):
        """Skip normal lidar detours once the P final approach has begun."""
        if not self._p_approaching or self.p_approach_disable_avoidance_distance <= 0.0:
            return False
        if self._p_final_approach_latched:
            return True
        detected, conf, bbox, _stamp, _offset, _fill = self._p_detection()
        if detected and conf >= self.p_approach_conf:
            depth_m, _samples, _status = self._p_depth_measurement(bbox)
            if (
                depth_m is not None
                and depth_m <= self.p_approach_disable_avoidance_distance
            ):
                self._p_final_approach_latched = True
                return True
        return False

    def _begin_avoidance(self, danger_deg):
        if self._p_approaching and self._p_target_yaw is not None:
            self._p_avoidance_recovery_yaw = self._p_target_yaw
        self.avoid_state = 'avoiding'
        self.avoid_turn_direction, selection_detail = self._choose_avoid_turn_direction(
            danger_deg
        )
        self.avoid_started_time = self.get_clock().now()
        self.avoid_clear_since = None
        self.avoid_entry_yaw = self.current_yaw
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.recovery_uses_heading = False
        self._update_avoidance_desired_heading()
        self._publish_state('avoiding')
        turn_name = 'RIGHT' if self.avoid_turn_direction < 0.0 else 'LEFT'
        desired_deg = (
            math.degrees(self.desired_heading)
            if self.desired_heading is not None else float('nan')
        )
        self.log.corner_avoid(
            f'{turn_name}: dir={self.avoid_turn_direction:.0f} '
            f'danger={danger_deg:.1f}deg {selection_detail} desired='
            f'{desired_deg:.1f}deg'
        )
        self._publish_feedback(f'avoid start dir={self.avoid_turn_direction:.1f} danger={danger_deg:.1f}°')

    def _update_avoidance_desired_heading(self):
        """Restore the P visual heading after a visual-approach lidar detour."""
        if self._p_avoidance_recovery_yaw is not None:
            self.desired_heading = self._p_avoidance_recovery_yaw
            return
        if self.current_position is not None:
            goal_x, goal_y = self._goal_center()
            dx = goal_x - self.current_position[0]
            dy = goal_y - self.current_position[1]
            if abs(dx) + abs(dy) > 1e-6:
                self.desired_heading = math.atan2(dy, dx)
                return
        if self.desired_heading is None and self.current_yaw is not None:
            self.desired_heading = self.current_yaw

    def _choose_avoid_turn_direction(self, danger_deg):
        """Choose the safe detour that also leaves the chassis closest to the search goal."""
        fallback = (
            -1.0
            if math.radians(danger_deg) >= self.avoid_right_turn_left_obstacle_angle
            else 1.0
        )
        if (
            not self.avoid_goal_bias_enabled
            or self.current_position is None
            or self.current_yaw is None
        ):
            return fallback, 'local_fallback'

        goal_x, goal_y = self._goal_center()
        dx = goal_x - self.current_position[0]
        dy = goal_y - self.current_position[1]
        if abs(dx) + abs(dy) <= 1e-6:
            return fallback, 'local_at_goal'

        target_bearing = math.atan2(dy, dx)
        danger_angle = math.radians(danger_deg)
        scores = {}
        for direction in (-1.0, 1.0):
            turned_yaw = self._normalize_angle(
                self.current_yaw + direction * self.avoid_min_turn_angle
            )
            score = abs(self._angle_error(target_bearing, turned_yaw))
            if (
                abs(danger_angle) >= self.avoid_right_turn_left_obstacle_angle
                and math.copysign(1.0, danger_angle) == direction
            ):
                score += self.avoid_obstacle_side_penalty
            scores[direction] = score

        direction = min(scores, key=scores.get)
        relative_goal = math.degrees(self._angle_error(target_bearing, self.current_yaw))
        detail = (
            f'goal_bias target=({goal_x:.2f},{goal_y:.2f}) '
            f'goal_angle={relative_goal:.1f}deg '
            f'score_right={scores[-1.0]:.2f} score_left={scores[1.0]:.2f}'
        )
        return direction, detail

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
                # S1 switches to a tighter emergency window while turning;
                # once that window is clear, the fixed maneuver can progress.
                obs = self._find_nearest_obstacle(self.latest_scan, emergency=True)
                cone_clear = obs is None

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

            if self.recovery_uses_heading and self.current_yaw is not None and self.desired_heading is not None:
                error = self._angle_error(self.desired_heading, self.current_yaw)
                angular = self._clamp(self.recovery_kp * error, self.recovery_max_angular)
                if abs(error) > self.heading_tolerance and abs(angular) < self.recovery_min_angular:
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
        self.recovery_uses_heading = False

    def _begin_recovery(self):
        if self.avoid_state not in ('avoiding', 'countersteering'):
            return
        self.avoid_state = 'recovering'
        self.avoid_clear_since = None
        self.counter_steer_deadline = None
        self._update_avoidance_desired_heading()
        self.recovery_uses_heading = (
            self.current_yaw is not None and self.desired_heading is not None
        )
        if self.recovery_uses_heading:
            heading_error = abs(self._angle_error(self.desired_heading, self.current_yaw))
            dur = max(
                0.6,
                heading_error / max(self.recovery_max_angular, 0.1) * 1.6,
            )
        else:
            dur = max(0.15, self.last_avoid_duration * self.recovery_duration_scale)
        dur = min(self.recovery_timeout, dur)
        self.recovery_deadline = self.get_clock().now() + Duration(seconds=dur)

    def _recovery_complete(self):
        now = self.get_clock().now()
        if self.recovery_uses_heading and self.current_yaw is not None and self.desired_heading is not None:
            if abs(self._angle_error(self.desired_heading, self.current_yaw)) <= self.heading_tolerance:
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
        self.recovery_uses_heading = False
        self._publish_state('running')

    def destroy_node(self):
        try:
            self._set_p_inference_active(False)
            self._set_stage3_http_active(False)
            self._clear_depth_cache()
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

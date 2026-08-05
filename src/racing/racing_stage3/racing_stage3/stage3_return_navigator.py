"""Stage3 官方返程导航：地图找 P + P 视觉接管 + 终端里程段 + Stage1 4态避障
- 状态机: idle → armed → running(map_search_p) → p_approach → p_final_odometry → complete
- running 时可中断：avoiding → countersteer → recovering → running
- 由 Supervisor activate 后启动运动
- 输出 /cmd_vel（自然交接后独占）
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
from racing_common.process_lifecycle import install_parent_death_signal
from racing_common.yolo_bbox_detector import YoloBBoxDetector
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
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

        # 文件会话在 Supervisor activate 时才打开，避免预热覆盖比赛日志。
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
        # Prewarming is silent; Supervisor activation grants motion authority.
        self._activated = False
        self._released = False
        self._handoff_command_announced = False
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
        self._imu_yaw = None
        self._imu_yaw_offset = 0.0
        self._awaiting_entry_yaw_alignment = False
        self._entry_anchor_map = None
        self._entry_anchor_odom = None
        self._entry_anchor_map_from_odom_yaw = None
        self._pending_entry_anchor_map = None
        self._entry_anchor_stamp_sec = None
        self._last_tf_position = None
        # The terminal corner supplies a physical map correction before the
        # P mark and close-range lidar become unreliable in the final run.
        self._terminal_corner_map_correction = (0.0, 0.0)
        self._terminal_corner_candidate_since = None
        self._terminal_corner_source_axes = None
        self._terminal_corner_reference_correction = None
        self._terminal_corner_lock = None
        self._terminal_corner_committed = False
        self._terminal_precommitting = False
        self._terminal_corner_target_yaw = None
        self._terminal_corner_start_odom = None
        self._terminal_corner_start_distance = None
        self._terminal_last_progress_odom = None
        self._terminal_last_progress_at = None
        self._terminal_p_confirmed = False
        self._p_ever_confirmed = False
        self._terminal_completion_candidate_since = None
        self._terminal_zone_announced = False
        self._terminal_fallback_hold_since = None

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
        self._p_ever_confirmed = False
        self._terminal_fallback_hold_since = None
        self._initial_align_required = False
        self._initial_align_target_yaw = None

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
        self._p_final_run_start_odom = None
        self._p_final_run_target_yaw = None
        self._p_final_run_depth_m = None
        self._p_final_run_trigger = ''
        self._p_final_last_progress_odom = None
        self._p_final_last_progress_at = None
        self._p_last_fill_ratio = 0.0
        self._p_last_fill_at = None
        self._p_last_valid_depth_m = None
        self._p_last_valid_depth_at = None
        self._p_last_valid_depth_status = ''
        self._p_visual_near_since = None
        self._p_visual_near_last_at = None
        self._p_visual_only_near_since = None
        self._p_visual_only_near_last_at = None
        self._p_visible_yaw_history = deque()
        self._p_recovery_target_yaw = None
        # Once P is large, centered, and stable, its disappearance means the
        # chassis has reached the physical terminal.  Do not fall back to a
        # drift-prone map coordinate and drive past the finish.
        self._p_terminal_pass_candidate_since = None
        self._p_terminal_pass_armed = False
        # Preserve the visual heading while lidar temporarily takes control.
        self._p_avoidance_recovery_yaw = None
        self._p_approach_progress_odom = None
        self._p_approach_progress_at = None

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
        self.declare_parameter('lifecycle_service_prefix', '/competition/stage3')
        lifecycle_prefix = str(self.get_parameter('lifecycle_service_prefix').value).rstrip('/')
        self._activate_srv = self.create_service(Trigger, f'{lifecycle_prefix}/activate', self._activate_cb)
        self._release_srv = self.create_service(Trigger, f'{lifecycle_prefix}/release', self._release_cb)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        self._publish_state('standby')
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
        self.declare_parameter('standby', True)
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
        self.declare_parameter('p_terminal_pass_enabled', True)
        self.declare_parameter('p_terminal_pass_fill_ratio', 0.35)
        self.declare_parameter('p_terminal_pass_center_offset', 0.10)
        self.declare_parameter('p_terminal_pass_evidence_hold_sec', 0.20)
        self.declare_parameter('p_heading_bearing_gain_rad', 0.55)
        self.declare_parameter('p_heading_kp', 1.4)
        self.declare_parameter('p_heading_tolerance_deg', 3.0)
        self.declare_parameter('p_heading_max_angular_speed', 0.45)
        self.declare_parameter('p_heading_reacquire_offset', 0.18)
        self.declare_parameter('p_heading_reacquire_interval_sec', 0.35)
        self.declare_parameter('p_heading_reacquire_max_delta_deg', 8.0)
        self.declare_parameter('p_loss_reverse_speed', 0.10)
        self.declare_parameter('p_loss_reverse_duration_sec', 0.80)
        self.declare_parameter('p_loss_grace_period_sec', 0.45)
        self.declare_parameter('p_loss_reverse_max_angular', 0.35)
        self.declare_parameter('p_loss_heading_lookback_sec', 1.0)
        self.declare_parameter('p_loss_heading_tolerance_deg', 6.0)
        self.declare_parameter('p_loss_heading_kp', 1.2)
        self.declare_parameter('p_approach_stall_reverse_enabled', True)
        self.declare_parameter('p_approach_progress_min_delta_m', 0.015)
        self.declare_parameter('p_approach_progress_timeout_sec', 0.75)
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
        self.declare_parameter('p_final_visual_fill_trigger_ratio', 0.45)
        self.declare_parameter('p_final_visual_depth_assist_m', 0.75)
        self.declare_parameter('p_final_visual_evidence_hold_sec', 0.10)
        self.declare_parameter('p_final_visual_evidence_timeout_sec', 1.20)
        self.declare_parameter('p_final_visual_no_depth_fill_trigger_ratio', 0.45)
        self.declare_parameter('p_approach_slow_linear_speed', 0.12)
        self.declare_parameter('p_final_odom_travel_m', 0.50)
        self.declare_parameter('p_final_brake_response_sec', 0.12)
        self.declare_parameter('p_final_brake_decel_mps2', 0.80)
        self.declare_parameter('p_final_brake_margin_m', 0.015)
        self.declare_parameter('p_final_completion_tolerance_m', 0.040)
        self.declare_parameter('p_final_heading_tolerance_deg', 10.0)
        self.declare_parameter('p_final_heading_max_angular_speed', 0.12)
        self.declare_parameter('p_final_progress_min_delta_m', 0.015)
        self.declare_parameter('p_final_progress_timeout_sec', 0.75)
        self.declare_parameter('p_final_stall_completion_min_m', 0.30)
        self.declare_parameter('p_approach_disable_avoidance_distance_m', 0.0)

        # ── P 墙角终端校正 ──
        # The fixed field corner is a metric reference.  It is deliberately
        # independent from odometry orientation; yaw remains IMU-only.
        self.declare_parameter('terminal_corner_enabled', True)
        self.declare_parameter('terminal_corner_map_x', 0.0)
        self.declare_parameter('terminal_corner_map_y', 0.0)
        self.declare_parameter('terminal_target_map_x', 0.25)
        self.declare_parameter('terminal_target_map_y', 0.10)
        self.declare_parameter('terminal_corner_activation_distance_m', 1.20)
        self.declare_parameter('terminal_corner_max_range_m', 1.80)
        self.declare_parameter('terminal_corner_cluster_gap_m', 0.18)
        self.declare_parameter('terminal_corner_min_points', 8)
        self.declare_parameter('terminal_corner_min_span_m', 0.25)
        self.declare_parameter('terminal_corner_fit_residual_m', 0.035)
        self.declare_parameter('terminal_corner_perpendicular_tolerance_deg', 25.0)
        self.declare_parameter('terminal_corner_max_correction_m', 0.80)
        self.declare_parameter('terminal_corner_source_axis_jump_deg', 18.0)
        self.declare_parameter('terminal_corner_expected_axes_deg_json', '[0.0, 90.0]')
        self.declare_parameter('terminal_corner_expected_axis_tolerance_deg', 15.0)
        self.declare_parameter('terminal_corner_lock_hold_sec', 0.20)
        self.declare_parameter('terminal_corner_filter_alpha', 0.35)
        self.declare_parameter('terminal_corner_correction_stability_m', 0.06)
        self.declare_parameter('terminal_acquire_distance_m', 1.20)
        self.declare_parameter('terminal_corner_commit_max_distance_m', 0.65)
        self.declare_parameter('terminal_precommit_linear_speed', 0.06)
        self.declare_parameter('terminal_precommit_heading_kp', 1.2)
        self.declare_parameter('terminal_precommit_heading_max_angular_speed', 0.25)
        self.declare_parameter('terminal_precommit_heading_tolerance_deg', 6.0)
        self.declare_parameter('terminal_corner_approach_speed', 0.12)
        self.declare_parameter('terminal_corner_heading_kp', 1.4)
        self.declare_parameter('terminal_corner_heading_max_angular_speed', 0.30)
        self.declare_parameter('terminal_corner_stop_tolerance_m', 0.035)
        self.declare_parameter('terminal_corner_lateral_tolerance_m', 0.060)
        self.declare_parameter('terminal_lateral_guard_m', 0.09)
        self.declare_parameter('terminal_corner_max_extra_travel_m', 0.08)
        self.declare_parameter('terminal_progress_min_delta_m', 0.015)
        self.declare_parameter('terminal_progress_timeout_sec', 0.75)
        self.declare_parameter('terminal_completion_hold_sec', 0.30)
        self.declare_parameter('terminal_corner_disable_emergency_avoidance', True)
        self.declare_parameter('terminal_fallback_enabled', True)
        self.declare_parameter('terminal_fallback_tolerance_m', 0.12)
        self.declare_parameter('terminal_fallback_linear_speed', 0.24)
        self.declare_parameter('terminal_fallback_max_angular_speed', 0.35)
        self.declare_parameter('terminal_fallback_heading_kp', 1.4)
        self.declare_parameter('terminal_fallback_completion_hold_sec', 0.30)

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
        self.p_terminal_pass_enabled = bool(
            self.get_parameter('p_terminal_pass_enabled').value
        )
        self.p_terminal_pass_fill_ratio = min(1.0, max(0.05, float(
            self.get_parameter('p_terminal_pass_fill_ratio').value
        )))
        self.p_terminal_pass_center_offset = min(1.0, max(0.0, float(
            self.get_parameter('p_terminal_pass_center_offset').value
        )))
        self.p_terminal_pass_evidence_hold = max(0.0, float(
            self.get_parameter('p_terminal_pass_evidence_hold_sec').value
        ))
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
        self.p_heading_reacquire_max_delta = math.radians(max(0.0, float(
            self.get_parameter('p_heading_reacquire_max_delta_deg').value
        )))
        self.p_loss_reverse_speed = abs(float(self.get_parameter('p_loss_reverse_speed').value))
        self.p_loss_reverse_duration = max(
            0.0, float(self.get_parameter('p_loss_reverse_duration_sec').value)
        )
        self.p_loss_grace_period = max(
            0.0, float(self.get_parameter('p_loss_grace_period_sec').value)
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
        self.p_approach_stall_reverse_enabled = bool(
            self.get_parameter('p_approach_stall_reverse_enabled').value
        )
        self.p_approach_progress_min_delta = max(
            0.001,
            float(self.get_parameter('p_approach_progress_min_delta_m').value),
        )
        self.p_approach_progress_timeout = max(
            0.1,
            float(self.get_parameter('p_approach_progress_timeout_sec').value),
        )
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
        self.p_final_visual_fill_trigger = min(
            1.0,
            max(0.0, float(
                self.get_parameter('p_final_visual_fill_trigger_ratio').value
            )),
        )
        self.p_final_visual_depth_assist = max(
            self.p_depth_stop_distance,
            float(self.get_parameter('p_final_visual_depth_assist_m').value),
        )
        self.p_final_visual_evidence_hold = max(
            0.0,
            float(self.get_parameter('p_final_visual_evidence_hold_sec').value),
        )
        self.p_final_visual_evidence_timeout = max(
            0.0,
            float(self.get_parameter('p_final_visual_evidence_timeout_sec').value),
        )
        self.p_final_visual_no_depth_fill_trigger = min(
            1.0,
            max(0.0, float(
                self.get_parameter('p_final_visual_no_depth_fill_trigger_ratio').value
            )),
        )
        self.p_approach_slow_linear = max(
            0.02, float(self.get_parameter('p_approach_slow_linear_speed').value)
        )
        self.p_final_odom_travel = max(
            0.05, float(self.get_parameter('p_final_odom_travel_m').value)
        )
        self.p_final_brake_response = max(
            0.0, float(self.get_parameter('p_final_brake_response_sec').value)
        )
        self.p_final_brake_decel = max(
            0.05, float(self.get_parameter('p_final_brake_decel_mps2').value)
        )
        self.p_final_brake_margin = max(
            0.0, float(self.get_parameter('p_final_brake_margin_m').value)
        )
        self.p_final_completion_tolerance = min(
            self.p_final_odom_travel,
            max(0.005, float(self.get_parameter('p_final_completion_tolerance_m').value)),
        )
        self.p_final_heading_tolerance = math.radians(max(
            0.1, float(self.get_parameter('p_final_heading_tolerance_deg').value)
        ))
        self.p_final_heading_max_angular = max(
            0.0, float(self.get_parameter('p_final_heading_max_angular_speed').value)
        )
        self.p_final_progress_min_delta = max(
            0.001, float(self.get_parameter('p_final_progress_min_delta_m').value)
        )
        self.p_final_progress_timeout = max(
            0.1, float(self.get_parameter('p_final_progress_timeout_sec').value)
        )
        self.p_final_stall_completion_min = min(
            self.p_final_odom_travel,
            max(0.0, float(self.get_parameter('p_final_stall_completion_min_m').value)),
        )
        self.p_approach_disable_avoidance_distance = max(
            0.0, float(self.get_parameter(
                'p_approach_disable_avoidance_distance_m'
            ).value)
        )
        self.terminal_corner_enabled = bool(
            self.get_parameter('terminal_corner_enabled').value
        )
        self.terminal_corner_map = (
            float(self.get_parameter('terminal_corner_map_x').value),
            float(self.get_parameter('terminal_corner_map_y').value),
        )
        self.terminal_target_map = (
            float(self.get_parameter('terminal_target_map_x').value),
            float(self.get_parameter('terminal_target_map_y').value),
        )
        self.terminal_corner_activation_distance = max(0.1, float(
            self.get_parameter('terminal_corner_activation_distance_m').value
        ))
        self.terminal_corner_max_range = max(0.2, float(
            self.get_parameter('terminal_corner_max_range_m').value
        ))
        self.terminal_corner_cluster_gap = max(0.01, float(
            self.get_parameter('terminal_corner_cluster_gap_m').value
        ))
        self.terminal_corner_min_points = max(3, int(
            self.get_parameter('terminal_corner_min_points').value
        ))
        self.terminal_corner_min_span = max(0.05, float(
            self.get_parameter('terminal_corner_min_span_m').value
        ))
        self.terminal_corner_fit_residual = max(0.001, float(
            self.get_parameter('terminal_corner_fit_residual_m').value
        ))
        self.terminal_corner_perpendicular_tolerance = math.radians(max(1.0, float(
            self.get_parameter('terminal_corner_perpendicular_tolerance_deg').value
        )))
        self.terminal_corner_max_correction = max(0.05, float(
            self.get_parameter('terminal_corner_max_correction_m').value
        ))
        self.terminal_corner_source_axis_jump = math.radians(max(1.0, float(
            self.get_parameter('terminal_corner_source_axis_jump_deg').value
        )))
        try:
            expected_axes_deg = json.loads(str(self.get_parameter(
                'terminal_corner_expected_axes_deg_json'
            ).value))
        except (json.JSONDecodeError, TypeError):
            expected_axes_deg = []
        self.terminal_corner_expected_axes = tuple(sorted(
            self._undirected_axis(math.radians(float(axis)))
            for axis in expected_axes_deg
        ))
        self.terminal_corner_expected_axis_tolerance = math.radians(max(1.0, float(
            self.get_parameter('terminal_corner_expected_axis_tolerance_deg').value
        )))
        self.terminal_corner_lock_hold = max(0.0, float(
            self.get_parameter('terminal_corner_lock_hold_sec').value
        ))
        self.terminal_corner_filter_alpha = min(1.0, max(0.05, float(
            self.get_parameter('terminal_corner_filter_alpha').value
        )))
        self.terminal_corner_correction_stability = max(0.005, float(
            self.get_parameter('terminal_corner_correction_stability_m').value
        ))
        self.terminal_acquire_distance = max(0.1, float(
            self.get_parameter('terminal_acquire_distance_m').value
        ))
        self.terminal_corner_commit_max_distance = max(0.05, float(
            self.get_parameter('terminal_corner_commit_max_distance_m').value
        ))
        self.terminal_precommit_linear = max(0.02, float(
            self.get_parameter('terminal_precommit_linear_speed').value
        ))
        self.terminal_precommit_heading_kp = float(
            self.get_parameter('terminal_precommit_heading_kp').value
        )
        self.terminal_precommit_heading_max_angular = max(0.0, float(
            self.get_parameter('terminal_precommit_heading_max_angular_speed').value
        ))
        self.terminal_precommit_heading_tolerance = math.radians(max(0.1, float(
            self.get_parameter('terminal_precommit_heading_tolerance_deg').value
        )))
        self.terminal_corner_approach_speed = max(0.02, float(
            self.get_parameter('terminal_corner_approach_speed').value
        ))
        self.terminal_corner_heading_kp = float(
            self.get_parameter('terminal_corner_heading_kp').value
        )
        self.terminal_corner_heading_max_angular = max(0.0, float(
            self.get_parameter('terminal_corner_heading_max_angular_speed').value
        ))
        self.terminal_corner_stop_tolerance = max(0.005, float(
            self.get_parameter('terminal_corner_stop_tolerance_m').value
        ))
        self.terminal_corner_lateral_tolerance = max(0.005, float(
            self.get_parameter('terminal_corner_lateral_tolerance_m').value
        ))
        self.terminal_lateral_guard = max(
            self.terminal_corner_lateral_tolerance,
            float(self.get_parameter('terminal_lateral_guard_m').value),
        )
        self.terminal_corner_max_extra_travel = max(0.0, float(
            self.get_parameter('terminal_corner_max_extra_travel_m').value
        ))
        self.terminal_progress_min_delta = max(0.001, float(
            self.get_parameter('terminal_progress_min_delta_m').value
        ))
        self.terminal_progress_timeout = max(0.1, float(
            self.get_parameter('terminal_progress_timeout_sec').value
        ))
        self.terminal_completion_hold = max(0.0, float(
            self.get_parameter('terminal_completion_hold_sec').value
        ))
        self.terminal_corner_disable_emergency_avoidance = bool(
            self.get_parameter('terminal_corner_disable_emergency_avoidance').value
        )
        self.terminal_fallback_enabled = bool(
            self.get_parameter('terminal_fallback_enabled').value
        )
        self.terminal_fallback_tolerance = max(0.03, float(
            self.get_parameter('terminal_fallback_tolerance_m').value
        ))
        self.terminal_fallback_linear_speed = max(0.04, float(
            self.get_parameter('terminal_fallback_linear_speed').value
        ))
        self.terminal_fallback_max_angular = max(0.05, float(
            self.get_parameter('terminal_fallback_max_angular_speed').value
        ))
        self.terminal_fallback_heading_kp = max(0.1, float(
            self.get_parameter('terminal_fallback_heading_kp').value
        ))
        self.terminal_fallback_completion_hold = max(0.0, float(
            self.get_parameter('terminal_fallback_completion_hold_sec').value
        ))

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

    def _twist(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        if (
            self._activated
            and not self._handoff_command_announced
            and (abs(t.linear.x) > 1e-4 or abs(t.angular.z) > 1e-4)
        ):
            self._handoff_command_announced = True
            self._publish_state('handoff_command_ready')
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

    def _activate_cb(self, _request, response):
        if self._released:
            response.success = False
            response.message = 'stage3 already released'
            return response
        self._activated = True
        self.log.start_session()
        self._set_stage3_http_active(True)
        self._arm_mission()
        response.success = True
        response.message = 'stage3 activated'
        return response

    def _release_cb(self, _request, response):
        if self._released:
            response.success = True
            response.message = 'stage3 already released'
            return response
        self._released = True
        self._activated = False
        self.mission_active = False
        self._publish_state('complete')
        response.success = True
        response.message = 'stage3 released; process will exit'
        self._release_shutdown_timer = self.create_timer(0.15, self._shutdown_after_release)
        return response

    def _shutdown_after_release(self):
        if rclpy.ok():
            rclpy.shutdown()

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
            self._activated and not self.mission_finished
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

    def _odom_cb(self, msg):
        raw_x = float(msg.pose.pose.position.x)
        raw_y = float(msg.pose.pose.position.y)
        self._last_raw_odom_xy = (raw_x, raw_y)
        self.odom_frame_id = msg.header.frame_id or self.odom_frame_id
        real_map = self._lookup_map_xy_from_tf()
        if real_map is not None and self.log.path is not None:
            self.log.real_pose(real_map[0], real_map[1], source='map_tf')
        if self._pending_entry_anchor_map is not None:
            pending_anchor = self._pending_entry_anchor_map
            self._pending_entry_anchor_map = None
            if self._bind_stage3_entry_anchor(pending_anchor) and not self._activated:
                self._publish_state('ready')
        if self._entry_anchor_map is not None and self._entry_anchor_odom is not None:
            base_position = self._position_from_entry_anchor(
                self._entry_anchor_map, self._entry_anchor_odom, self._last_raw_odom_xy,
                self._entry_anchor_map_from_odom_yaw,
            )
            self.current_position = self._apply_terminal_corner_correction(base_position)
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

    def _apply_terminal_corner_correction(self, position):
        """Apply only the translation established from a physical wall corner."""
        return (
            position[0] + self._terminal_corner_map_correction[0],
            position[1] + self._terminal_corner_map_correction[1],
        )

    def _uncorrected_entry_anchor_position(self):
        if (
            self._entry_anchor_map is None
            or self._entry_anchor_odom is None
            or self._last_raw_odom_xy is None
        ):
            return None
        return self._position_from_entry_anchor(
            self._entry_anchor_map,
            self._entry_anchor_odom,
            self._last_raw_odom_xy,
            self._entry_anchor_map_from_odom_yaw,
        )

    def _stage3_entry_anchor_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.log.warn(
                'ENTRY_ANCHOR',
                f'ignored anchor frame={msg.header.frame_id}, expected={self.map_frame}',
            )
            return
        stamp_sec = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        anchor_age = self._now_sec() - stamp_sec if stamp_sec > 0.0 else 0.0
        if anchor_age < -0.25:
            self.log.warn(
                'ENTRY_ANCHOR',
                f'ignored future anchor age={anchor_age:.2f}s',
            )
            return
        if anchor_age > self.stage3_entry_anchor_max_age:
            # S2 publishes a physical anchor before it keeps moving.  A
            # delayed/transient-local delivery is still usable because the
            # current /odom_combined sample is propagated from that anchor.
            self.log.warn(
                'ENTRY_ANCHOR',
                f'accepted delayed anchor age={anchor_age:.2f}s '
                f'max={self.stage3_entry_anchor_max_age:.2f}s',
            )
        anchor_map = (float(msg.point.x), float(msg.point.y))
        self._entry_anchor_stamp_sec = stamp_sec if stamp_sec > 0.0 else self._now_sec()
        if self._last_raw_odom_xy is None:
            self._pending_entry_anchor_map = anchor_map
            self.log.mission(
                f'Stage2 entry anchor received map={anchor_map}; waiting for {self.odom_topic}'
            )
            return
        self._bind_stage3_entry_anchor(anchor_map)
        if not self._activated:
            self._publish_state('ready')

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
        if self._activated or self._preplan_start is None or self._preplanned_path:
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
        if not self._activated or not self.p_depth_logging_enabled:
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
        self._handoff_command_announced = False
        self._reset_terminal_corner_state()
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
        self._p_ever_confirmed = False
        self._terminal_fallback_hold_since = None
        self._initial_align_required = False
        self._initial_align_target_yaw = None
        if self._entry_anchor_map is not None and self._entry_anchor_odom is not None:
            self.current_position = self._apply_terminal_corner_correction(
                self._position_from_entry_anchor(
                self._entry_anchor_map, self._entry_anchor_odom, self._last_raw_odom_xy,
                self._entry_anchor_map_from_odom_yaw,
                )
            )
        else:
            self.current_position = self._lookup_map_xy_from_tf()
        if self.current_position is None:
            self.log.warn('POSE', 'waiting for map<-base_footprint TF')
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
        self._p_final_run_start_odom = None
        self._p_final_run_target_yaw = None
        self._p_final_run_depth_m = None
        self._p_final_run_trigger = ''
        self._p_final_last_progress_odom = None
        self._p_final_last_progress_at = None
        self._p_terminal_pass_candidate_since = None
        self._p_terminal_pass_armed = False
        self._reset_p_final_visual_evidence()
        self._p_visible_yaw_history.clear()
        self._p_recovery_target_yaw = None
        self._p_avoidance_recovery_yaw = None
        self._p_approach_progress_odom = None
        self._p_approach_progress_at = None
        self._set_p_inference_active(True)
        # Phase3 starts the return immediately; P is terminal semantic confirmation only.
        self._set_channel_inference_active(False)
        self._pre_return_state = 'done'
        self._pre_return_started_at = self._now_sec()
        self._channel_hits = 0
        self._channel_offset_filtered = 0.0
        self._publish_state('armed')
        self.log.mission(
            f'S3 activated, direction={self.return_direction}, '
            f'tf_map={self.current_position}; '
            'starting return; P is terminal semantic confirmation only'
        )

    def _reset_mission(self):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = False
        self._reset_terminal_corner_state()
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
        self._initial_align_target_yaw = None
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
        self._p_final_run_start_odom = None
        self._p_final_run_target_yaw = None
        self._p_final_run_depth_m = None
        self._p_final_run_trigger = ''
        self._p_final_last_progress_odom = None
        self._p_final_last_progress_at = None
        self._reset_p_final_visual_evidence()
        self._p_visible_yaw_history.clear()
        self._p_recovery_target_yaw = None
        self._p_avoidance_recovery_yaw = None
        self._p_approach_progress_odom = None
        self._p_approach_progress_at = None
        self._pre_return_state = 'idle'
        self._channel_hits = 0
        self._channel_offset_filtered = 0.0
        self._set_channel_inference_active(False)
        self._set_p_inference_active(False)
        detector = getattr(self, '_p_detector', None)
        if detector is not None and hasattr(detector, 'release_model'):
            detector.release_model('phase_exit')
        self._publish_state('idle')

    def _reset_terminal_corner_state(self):
        self._terminal_corner_map_correction = (0.0, 0.0)
        self._terminal_corner_candidate_since = None
        self._terminal_corner_source_axes = None
        self._terminal_corner_reference_correction = None
        self._terminal_corner_lock = None
        self._terminal_corner_committed = False
        self._terminal_precommitting = False
        self._terminal_corner_target_yaw = None
        self._terminal_corner_start_odom = None
        self._terminal_corner_start_distance = None
        self._terminal_last_progress_odom = None
        self._terminal_last_progress_at = None
        self._terminal_p_confirmed = False
        self._terminal_completion_candidate_since = None
        self._terminal_zone_announced = False
        self._terminal_fallback_hold_since = None

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
        self._initial_align_target_yaw = target_yaw
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
        real_map = self._lookup_map_xy_from_tf()
        if real_map is not None:
            self.log.real_pose(real_map[0], real_map[1], source='map_tf', force=True)
        self.log.task(f'Stage3 完成，原因={feedback_text}')
        # P point is terminal for the competition.  Stage3 owns its normal
        # shutdown; Supervisor only verifies that it has disappeared before
        # closing the common base stack.
        self._complete_shutdown_timer = self.create_timer(0.50, self._shutdown_after_complete)

    def _shutdown_after_complete(self):
        timer = getattr(self, '_complete_shutdown_timer', None)
        if timer is not None:
            timer.cancel()
        self.cmd_pub.publish(Twist())
        self.get_logger().info('Stage3 complete; shutting down process')
        rclpy.shutdown()

    def _fail_mission(self, reason):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self._set_channel_inference_active(False)
        self._set_p_inference_active(False)
        self._publish_state('failed')
        self._publish_feedback(f'return failed: {reason}')


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
        if self._released:
            return
        if not self._activated:
            self._maybe_build_preplanned_path()
            return
        if self.mission_finished:
            return

        now = self._now_sec()
        if not self.mission_active:
            if self.start_after_time is None or now < self.start_after_time:
                return
            self._start_mission()
            return

        # First align the vehicle sufficiently for both map navigation and
        # visual takeover. This is still a moving Ackermann maneuver, so it
        # must retain the same lidar safety arbitration as map search.
        if self._initial_align_required:
            self.desired_heading = self._initial_align_target_yaw
            if self._check_emergency_stop():
                return
            if self.avoid_state == 'forward' and self.latest_scan is not None:
                self._check_obstacle()
            if self.avoid_state != 'forward':
                self._run_avoidance()
                return
            self._run_initial_align()
            return

        self._update_p_inference_gate()
        self._update_p_detection()
        if self._p_final_run_start_odom is not None:
            self._run_p_final_odometry()
            return
        if self._p_approaching and self._try_arm_p_final_odometry():
            self._run_p_final_odometry()
            return

        # Preserve lidar protection during the visual approach until final
        # approach evidence confirms that the P board/terminal wall is the
        # expected close object.  Before that point it can still be a wall or
        # another obstacle directly ahead.
        if self._p_approaching:
            if not self._p_approach_avoidance_disabled():
                if self._check_emergency_stop():
                    return
                if self.avoid_state == 'forward' and self.latest_scan is not None:
                    self._check_obstacle()
                if self.avoid_state != 'forward':
                    self._run_avoidance()
                    return
            self._run_p_approach()
            return

        # After a confirmed P detection, terminal geometry is the remaining
        # objective. Do not let the P board/finish wall enter the generic
        # avoidance state machine or trigger a reverse maneuver.
        if self._terminal_fallback_ready():
            self._cancel_avoidance_for_terminal_zone()
            self._run_terminal_fallback_drive()
            return

        # MAP_SEARCH keeps the normal lidar safety state machine.
        if self._check_emergency_stop():
            return
        if self.avoid_state == 'forward' and self.latest_scan is not None:
            self._check_obstacle()
        if self.avoid_state != 'forward':
            self._run_avoidance()
            return

        # P is absent or was deliberately recovered from. Use the anchored
        # map route to return to the visual search point and try again.
        if self.current_position is None:
            self.stop_robot()
            self._publish_state('waiting_for_map_tf')
            return
        if self.use_global_planner and self._run_planner_forbidden_reverse():
            return
        self._run_center_drive()

    def _in_terminal_acquisition_zone(self):
        if self.current_position is None:
            return False
        return math.hypot(
            self.current_position[0] - self.terminal_target_map[0],
            self.current_position[1] - self.terminal_target_map[1],
        ) <= self.terminal_acquire_distance

    def _cancel_avoidance_for_terminal_zone(self):
        if self.avoid_state != 'forward':
            self.log.mission(
                f'terminal acquisition cancels lidar state={self.avoid_state}; '
                'expected terminal walls must not trigger a detour'
            )
        self.avoid_state = 'forward'
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.emergency_reverse_deadline = None
        self.recovery_uses_heading = False
        if not self._terminal_zone_announced:
            self._terminal_zone_announced = True
            self.log.mission(
                f'terminal acquisition zone entered: distance<='
                f'{self.terminal_acquire_distance:.2f}m; lidar avoidance disabled '
                'until terminal commit or mission release'
            )

    def _try_start_terminal_corner_approach(self):
        if (
            self._terminal_corner_committed
            or not self._terminal_p_confirmed
            or self._terminal_corner_lock is None
        ):
            return False
        if self.current_position is None or self.current_yaw is None:
            return False
        dx = self.terminal_target_map[0] - self.current_position[0]
        dy = self.terminal_target_map[1] - self.current_position[1]
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx) if distance > 1e-6 else self.current_yaw
        heading_error = self._angle_error(target_yaw, self.current_yaw)
        if (
            distance <= self.terminal_corner_commit_max_distance
            and abs(heading_error) <= self.terminal_precommit_heading_tolerance
        ):
            return self._start_terminal_corner_approach(target_yaw)
        self._terminal_precommitting = True
        return False

    def _check_emergency_stop(self):
        if self.avoid_state == 'emergency_reversing':
            self._run_emergency_reverse()
            return True
        if self._p_approach_avoidance_disabled():
            return False
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
        if self._p_approaching:
            self._reset_p_approach_progress_watchdog()
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

        target_yaw = self._initial_align_target_yaw
        if target_yaw is None:
            target_x, target_y = self._goal_center()
            target_yaw = math.atan2(
                target_y - self.current_position[1], target_x - self.current_position[0]
            )
            self._initial_align_target_yaw = target_yaw
        heading_error = self._angle_error(target_yaw, self.current_yaw)
        self.desired_heading = target_yaw
        if abs(heading_error) <= self.initial_align_tolerance:
            self._initial_align_required = False
            self._initial_align_target_yaw = None
            self._filtered_heading_err = 0.0
            self.stop_robot()
            self._publish_state('running')
            self.log.mission(
                f'initial align complete: target={math.degrees(target_yaw):.1f}° '
                f'yaw={math.degrees(self.current_yaw):.1f}° '
                f'error={math.degrees(heading_error):+.1f}°'
            )
            return

        # An Ackermann chassis cannot safely execute an arbitrary angular
        # command at a nonzero speed.  Keep its commanded radius no tighter
        # than the pursuit radius, including when a config value is too high.
        angular_limit = min(
            self.initial_align_angular,
            abs(self.initial_align_linear) / self.pursuit_min_turn_radius,
        )
        angular = math.copysign(angular_limit, heading_error)
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

    def _reset_p_final_visual_evidence(self):
        self._p_last_fill_ratio = 0.0
        self._p_last_fill_at = None
        self._p_last_valid_depth_m = None
        self._p_last_valid_depth_at = None
        self._p_last_valid_depth_status = ''
        self._p_visual_near_since = None
        self._p_visual_near_last_at = None
        self._p_visual_only_near_since = None
        self._p_visual_only_near_last_at = None

    def _update_p_final_visual_evidence(self, fill, depth_m, depth_status):
        now = self._now_sec()
        self._p_last_fill_ratio = float(fill or 0.0)
        self._p_last_fill_at = now
        if depth_m is not None:
            self._p_last_valid_depth_m = float(depth_m)
            self._p_last_valid_depth_at = now
            self._p_last_valid_depth_status = str(depth_status or '')

        recent_depth_ok = (
            self._p_last_valid_depth_m is not None
            and self._p_last_valid_depth_at is not None
            and now - self._p_last_valid_depth_at <= self.p_final_visual_evidence_timeout
            and self._p_last_valid_depth_m <= self.p_final_visual_depth_assist
        )
        near_now = (
            self._p_last_fill_ratio >= self.p_final_visual_fill_trigger
            and recent_depth_ok
        )
        if near_now:
            if self._p_visual_near_since is None:
                self._p_visual_near_since = now
            self._p_visual_near_last_at = now
        elif (
            self._p_visual_near_last_at is None
            or now - self._p_visual_near_last_at > self.p_final_visual_evidence_timeout
        ):
            self._p_visual_near_since = None
            self._p_visual_near_last_at = None

        visual_only_now = (
            self.p_final_visual_no_depth_fill_trigger > 0.0
            and self._p_last_fill_ratio >= self.p_final_visual_no_depth_fill_trigger
            and depth_m is None
        )
        if visual_only_now:
            if self._p_visual_only_near_since is None:
                self._p_visual_only_near_since = now
            self._p_visual_only_near_last_at = now
        elif (
            self._p_visual_only_near_last_at is None
            or now - self._p_visual_only_near_last_at > self.p_final_visual_evidence_timeout
        ):
            self._p_visual_only_near_since = None
            self._p_visual_only_near_last_at = None

    def _p_visual_near_ready(self):
        if self.p_final_visual_fill_trigger <= 0.0:
            return False
        now = self._now_sec()
        if self._p_visual_near_since is None or self._p_visual_near_last_at is None:
            return False
        if now - self._p_visual_near_last_at > self.p_final_visual_evidence_timeout:
            return False
        if now - self._p_visual_near_since < self.p_final_visual_evidence_hold:
            return False
        return True

    def _p_visual_no_depth_near_ready(self):
        """Allow a stable large P box to commit when Aurora has no usable ROI."""
        if self.p_final_visual_no_depth_fill_trigger <= 0.0:
            return False
        now = self._now_sec()
        if (
            self._p_visual_only_near_since is None
            or self._p_visual_only_near_last_at is None
        ):
            return False
        if now - self._p_visual_only_near_last_at > self.p_final_visual_evidence_timeout:
            return False
        return now - self._p_visual_only_near_since >= self.p_final_visual_evidence_hold

    def _p_final_depth_start_text(self):
        if self._p_final_run_depth_m is None:
            return 'visual_fallback'
        return f'{self._p_final_run_depth_m:.3f}m'

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

    @staticmethod
    def _undirected_axis(angle):
        return float(angle % math.pi)

    @staticmethod
    def _undirected_axis_error(first, second):
        difference = abs(first - second) % math.pi
        return min(difference, math.pi - difference)

    def _cluster_terminal_corner_points(self, scan_msg):
        clusters = []
        active = []
        previous = None
        max_range = self.terminal_corner_max_range
        if scan_msg.range_max > 0.0:
            max_range = min(max_range, float(scan_msg.range_max))

        def finish_cluster():
            nonlocal active, previous
            if active:
                clusters.append(active)
            active = []
            previous = None

        for index, distance in enumerate(scan_msg.ranges):
            if (
                math.isinf(distance)
                or math.isnan(distance)
                or distance < self.min_range
                or distance > max_range
            ):
                finish_cluster()
                continue
            angle = scan_msg.angle_min + index * scan_msg.angle_increment
            point = (distance * math.cos(angle), distance * math.sin(angle))
            if (
                previous is not None
                and math.hypot(point[0] - previous[0], point[1] - previous[1])
                > self.terminal_corner_cluster_gap
            ):
                finish_cluster()
            active.append(point)
            previous = point
        finish_cluster()
        return clusters

    def _fit_terminal_corner_line(self, points):
        if len(points) < self.terminal_corner_min_points:
            return None
        values = np.asarray(points, dtype=float)
        center = np.mean(values, axis=0)
        centered = values - center
        try:
            _unused, _singular, vectors = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return None
        tangent = vectors[0]
        normal = np.array((-tangent[1], tangent[0]))
        projections = centered @ tangent
        span = float(np.max(projections) - np.min(projections))
        residual = float(np.sqrt(np.mean((centered @ normal) ** 2)))
        if (
            span < self.terminal_corner_min_span
            or residual > self.terminal_corner_fit_residual
        ):
            return None
        return {
            'center': center,
            'tangent': tangent,
            'span': span,
            'rms': residual,
            'points': int(values.shape[0]),
        }

    def _find_terminal_corner(self, scan_msg):
        best = None
        clusters = self._cluster_terminal_corner_points(scan_msg)
        lines = []

        def consider_pair(first, second):
            nonlocal best
            dot = float(np.clip(np.dot(first['tangent'], second['tangent']), -1.0, 1.0))
            perpendicular_error = abs(math.pi / 2.0 - math.acos(abs(dot)))
            if perpendicular_error > self.terminal_corner_perpendicular_tolerance:
                return
            matrix = np.column_stack((first['tangent'], -second['tangent']))
            if abs(float(np.linalg.det(matrix))) < 1e-4:
                return
            try:
                distances = np.linalg.solve(matrix, second['center'] - first['center'])
            except np.linalg.LinAlgError:
                return
            corner = first['center'] + distances[0] * first['tangent']
            corner_range = float(np.linalg.norm(corner))
            if corner_range > self.terminal_corner_max_range:
                return
            rank = (
                min(first['span'], second['span']),
                first['points'] + second['points'],
                -max(first['rms'], second['rms']),
                -corner_range,
            )
            candidate = {
                'corner_body': corner,
                'first': first,
                'second': second,
                'perpendicular_error': perpendicular_error,
                'range': corner_range,
                'rank': rank,
            }
            if best is None or candidate['rank'] > best['rank']:
                best = candidate

        for cluster in clusters:
            if (line := self._fit_terminal_corner_line(cluster)) is not None:
                lines.append(line)
            # Adjacent returns from an L corner can form one continuous V
            # cluster.  Test every viable scan-order split as two wall faces.
            for split in range(
                self.terminal_corner_min_points,
                len(cluster) - self.terminal_corner_min_points + 1,
            ):
                first = self._fit_terminal_corner_line(cluster[:split])
                second = self._fit_terminal_corner_line(cluster[split:])
                if first is not None and second is not None:
                    consider_pair(first, second)

        for first_index, first in enumerate(lines):
            for second in lines[first_index + 1:]:
                consider_pair(first, second)
        return best

    def _update_terminal_corner_lock(self):
        if (
            not self.terminal_corner_enabled
            or self.latest_scan is None
            or self.current_position is None
            or self.current_yaw is None
        ):
            return
        # A commit must use one immutable physical reference.  Continuing to
        # filter scan data after lock would move the endpoint while the final
        # odometry-only run is in progress.
        if self._terminal_corner_lock is not None:
            return
        if math.hypot(
            self.current_position[0] - self.terminal_target_map[0],
            self.current_position[1] - self.terminal_target_map[1],
        ) > self.terminal_corner_activation_distance:
            self._terminal_corner_candidate_since = None
            self._terminal_corner_reference_correction = None
            return
        base_position = self._uncorrected_entry_anchor_position()
        if base_position is None:
            return
        candidate = self._find_terminal_corner(self.latest_scan)
        if candidate is None:
            self._terminal_corner_candidate_since = None
            self._terminal_corner_reference_correction = None
            return

        corner_x, corner_y = candidate['corner_body']
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        measured_corner_map = (
            base_position[0] + cos_yaw * corner_x - sin_yaw * corner_y,
            base_position[1] + sin_yaw * corner_x + cos_yaw * corner_y,
        )
        raw_correction = (
            self.terminal_corner_map[0] - measured_corner_map[0],
            self.terminal_corner_map[1] - measured_corner_map[1],
        )
        if math.hypot(*raw_correction) > self.terminal_corner_max_correction:
            self._terminal_corner_candidate_since = None
            self._terminal_corner_reference_correction = None
            return

        axes = tuple(sorted((
            self._undirected_axis(self.current_yaw + math.atan2(
                candidate['first']['tangent'][1], candidate['first']['tangent'][0],
            )),
            self._undirected_axis(self.current_yaw + math.atan2(
                candidate['second']['tangent'][1], candidate['second']['tangent'][0],
            )),
        )))
        if (
            self.terminal_corner_expected_axes
            and (
                len(self.terminal_corner_expected_axes) != 2
                or len(axes) != 2
                or any(
                    self._undirected_axis_error(axis, expected)
                    > self.terminal_corner_expected_axis_tolerance
                    for axis, expected in zip(axes, self.terminal_corner_expected_axes)
                )
            )
        ):
            self._terminal_corner_candidate_since = None
            self._terminal_corner_reference_correction = None
            self._terminal_corner_source_axes = None
            return
        if self._terminal_corner_source_axes is not None and any(
            self._undirected_axis_error(axis, source_axis)
            > self.terminal_corner_source_axis_jump
            for axis, source_axis in zip(axes, self._terminal_corner_source_axes)
        ):
            self._terminal_corner_candidate_since = None
            self._terminal_corner_reference_correction = None
            return

        now = self._now_sec()
        if self._terminal_corner_reference_correction is None:
            self._terminal_corner_reference_correction = raw_correction
        elif math.hypot(
            raw_correction[0] - self._terminal_corner_reference_correction[0],
            raw_correction[1] - self._terminal_corner_reference_correction[1],
        ) > self.terminal_corner_correction_stability:
            self._terminal_corner_reference_correction = raw_correction
            self._terminal_corner_candidate_since = None
            return
        if self._terminal_corner_candidate_since is None:
            self._terminal_corner_candidate_since = now
        alpha = self.terminal_corner_filter_alpha
        old_x, old_y = self._terminal_corner_map_correction
        self._terminal_corner_map_correction = (
            old_x + alpha * (raw_correction[0] - old_x),
            old_y + alpha * (raw_correction[1] - old_y),
        )
        self.current_position = self._apply_terminal_corner_correction(base_position)
        self._terminal_corner_source_axes = axes
        if (
            self._terminal_corner_lock is None
            and now - self._terminal_corner_candidate_since >= self.terminal_corner_lock_hold
        ):
            self._terminal_corner_lock = {
                'stamp': now,
                'correction': self._terminal_corner_map_correction,
                'range': candidate['range'],
                'span': min(candidate['first']['span'], candidate['second']['span']),
                'rms': max(candidate['first']['rms'], candidate['second']['rms']),
            }
            self.log.segment(
                'P corner lock: '
                f'corner_body=({corner_x:.2f},{corner_y:.2f}) '
                f'correction=({self._terminal_corner_map_correction[0]:+.3f},'
                f'{self._terminal_corner_map_correction[1]:+.3f}) '
                f'range={candidate["range"]:.2f}m '
                f'span={self._terminal_corner_lock["span"]:.2f}m '
                f'rms={self._terminal_corner_lock["rms"]:.3f}m'
            )

    def _run_terminal_precommit(self):
        """Close the frozen-anchor geometry before allowing blind short travel."""
        if self.current_position is None or self.current_yaw is None:
            self.stop_robot()
            self._publish_state('p_corner_terminal_guard')
            return
        dx = self.terminal_target_map[0] - self.current_position[0]
        dy = self.terminal_target_map[1] - self.current_position[1]
        distance = math.hypot(dx, dy)
        if distance <= self.terminal_corner_stop_tolerance:
            self._start_terminal_corner_approach(self.current_yaw)
            return
        target_yaw = math.atan2(dy, dx)
        heading_error = self._angle_error(target_yaw, self.current_yaw)
        angular = self._clamp(
            self.terminal_precommit_heading_kp * heading_error,
            self.terminal_precommit_heading_max_angular,
        )
        if abs(heading_error) <= self.terminal_precommit_heading_tolerance:
            angular = 0.0
        self.desired_heading = target_yaw
        self._publish_state('terminal_precommit')
        self.log.telemetry(
            'TERMINAL_PRECOMMIT',
            f'dist={distance:.3f}m target={math.degrees(target_yaw):.1f}deg '
            f'heading_err={math.degrees(heading_error):+.1f}deg '
            f'v={self.terminal_precommit_linear:.2f} w={angular:.2f}',
        )
        self.cmd_pub.publish(self._twist(self.terminal_precommit_linear, angular))

    def _start_terminal_corner_approach(self, target_yaw=None):
        if (
            not self.terminal_corner_enabled
            or self._terminal_corner_lock is None
            or self.current_position is None
            or self.current_yaw is None
            or self._last_raw_odom_xy is None
        ):
            return False
        dx = self.terminal_target_map[0] - self.current_position[0]
        dy = self.terminal_target_map[1] - self.current_position[1]
        distance = math.hypot(dx, dy)
        if distance > self.terminal_corner_commit_max_distance:
            return False
        self._terminal_corner_committed = True
        self._terminal_precommitting = False
        self._terminal_corner_target_yaw = target_yaw if target_yaw is not None else (
            self.current_yaw
            if distance <= self.terminal_corner_stop_tolerance
            else math.atan2(dy, dx)
        )
        self._terminal_corner_start_odom = self._last_raw_odom_xy
        self._terminal_corner_start_distance = distance
        self._terminal_last_progress_odom = self._last_raw_odom_xy
        self._terminal_last_progress_at = self._now_sec()
        self.desired_heading = self._terminal_corner_target_yaw
        self._p_final_approach_latched = True
        self._publish_state('p_corner_approach')
        self.log.segment(
            f'P corner terminal commit: target=({self.terminal_target_map[0]:.2f},'
            f'{self.terminal_target_map[1]:.2f}) distance={distance:.3f}m '
            f'yaw={math.degrees(self._terminal_corner_target_yaw):.1f}deg '
            'visual/lidar loss is now expected; IMU plus short odometry runout'
        )
        return True

    def _run_terminal_corner_approach(self):
        if (
            self.current_position is None
            or self.current_yaw is None
            or self._terminal_corner_target_yaw is None
            or self._terminal_corner_start_odom is None
        ):
            self.stop_robot()
            self._publish_state('p_corner_terminal_guard')
            return
        dx = self.terminal_target_map[0] - self.current_position[0]
        dy = self.terminal_target_map[1] - self.current_position[1]
        cos_target = math.cos(self._terminal_corner_target_yaw)
        sin_target = math.sin(self._terminal_corner_target_yaw)
        remaining = cos_target * dx + sin_target * dy
        lateral = -sin_target * dx + cos_target * dy
        travelled = math.hypot(
            self._last_raw_odom_xy[0] - self._terminal_corner_start_odom[0],
            self._last_raw_odom_xy[1] - self._terminal_corner_start_odom[1],
        )
        max_travel = self._terminal_corner_start_distance + self.terminal_corner_max_extra_travel
        now = self._now_sec()
        if self._terminal_last_progress_odom is not None:
            progress_delta = math.hypot(
                self._last_raw_odom_xy[0] - self._terminal_last_progress_odom[0],
                self._last_raw_odom_xy[1] - self._terminal_last_progress_odom[1],
            )
            if progress_delta >= self.terminal_progress_min_delta:
                self._terminal_last_progress_odom = self._last_raw_odom_xy
                self._terminal_last_progress_at = now
            elif (
                self._terminal_last_progress_at is not None
                and now - self._terminal_last_progress_at >= self.terminal_progress_timeout
            ):
                self.stop_robot()
                self._publish_state('p_corner_terminal_guard')
                self.log.warn(
                    'P_CORNER',
                    f'terminal guard: no odom progress for '
                    f'{now - self._terminal_last_progress_at:.2f}s '
                    f'while remain={remaining:.3f}m',
                )
                return
        if abs(lateral) > self.terminal_lateral_guard:
            self.stop_robot()
            self._publish_state('p_corner_terminal_guard')
            self.log.warn(
                'P_CORNER',
                f'terminal guard: lateral={lateral:.3f}m exceeds '
                f'{self.terminal_lateral_guard:.3f}m',
            )
            return
        if abs(remaining) <= self.terminal_corner_stop_tolerance:
            if abs(lateral) <= self.terminal_corner_lateral_tolerance:
                if self._terminal_completion_candidate_since is None:
                    self._terminal_completion_candidate_since = now
                    self.stop_robot()
                    self._publish_state('terminal_done_hold')
                    self.log.segment(
                        f'terminal target reached; holding for '
                        f'{self.terminal_completion_hold:.2f}s before complete'
                    )
                    return
                if now - self._terminal_completion_candidate_since >= self.terminal_completion_hold:
                    self._finish_mission('return complete, terminal anchor short-run verified')
                else:
                    self.stop_robot()
                    self._publish_state('terminal_done_hold')
            else:
                self._terminal_completion_candidate_since = None
                self.stop_robot()
                self._publish_state('p_corner_terminal_guard')
                self.log.warn(
                    'P_CORNER',
                    f'terminal guard: remaining={remaining:.3f}m lateral={lateral:.3f}m',
                )
            return
        self._terminal_completion_candidate_since = None
        if remaining < -self.terminal_corner_stop_tolerance:
            self.stop_robot()
            self._publish_state('p_corner_terminal_guard')
            self.log.warn(
                'P_CORNER',
                f'terminal guard: overshot={-remaining:.3f}m '
                f'lateral={lateral:.3f}m',
            )
            return
        if travelled > max_travel:
            self.stop_robot()
            self._publish_state('p_corner_terminal_guard')
            self.log.warn(
                'P_CORNER',
                f'terminal guard: travelled={travelled:.3f}m max={max_travel:.3f}m',
            )
            return
        heading_error = self._angle_error(self._terminal_corner_target_yaw, self.current_yaw)
        angular = self._clamp(
            self.terminal_corner_heading_kp * heading_error,
            self.terminal_corner_heading_max_angular,
        )
        if abs(heading_error) <= self.p_heading_tolerance:
            angular = 0.0
        self._publish_state('p_corner_approach')
        self.log.telemetry(
            'P_CORNER_APPROACH',
            f'remain={remaining:.3f}m lateral={lateral:+.3f}m travelled={travelled:.3f}m '
            f'heading_err={math.degrees(heading_error):+.1f}deg '
            f'v={self.terminal_corner_approach_speed:.2f} w={angular:.2f}',
        )
        self.cmd_pub.publish(self._twist(self.terminal_corner_approach_speed, angular))

    def _update_p_detection(self):
        detected, conf, bbox, stamp, offset, _fill = self._p_detection()
        confirmed = (
            self._activated
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
        self._p_ever_confirmed = True
        self._p_approaching = True
        self._p_offset_filtered = float(offset)
        self._p_target_yaw = self._visual_target_yaw(offset)
        self._p_heading_lock_offset = self._p_offset_filtered
        self._p_last_heading_reacquire_at = self._now_sec()
        self.desired_heading = self._p_target_yaw
        self._p_lost_since = None
        self._p_lost_reverse_started_at = None
        self._p_last_angular = 0.0
        self._p_final_approach_latched = False
        self._p_final_run_start_odom = None
        self._p_final_run_target_yaw = None
        self._p_final_run_depth_m = None
        self._p_final_run_trigger = ''
        self._p_final_last_progress_odom = None
        self._p_final_last_progress_at = None
        self._reset_p_final_visual_evidence()
        self._p_visible_yaw_history.clear()
        self._p_recovery_target_yaw = None
        self._reset_p_approach_progress_watchdog()
        self._publish_state('p_approach')
        map_pos = self.current_position
        map_text = (
            f'map=({map_pos[0]:.2f},{map_pos[1]:.2f})'
            if map_pos is not None else 'map=unavailable'
        )
        self.log.segment(
            f'P acquired conf={conf:.2f} offset={offset:+.3f} '
            f'hits={self._p_consecutive_hits}; '
            f'heading_lock={math.degrees(self._p_target_yaw):.1f}deg '
            f'visual approach v={self.p_approach_linear:.2f} {map_text} '
            f'{self._position_source_text()} {self._p_depth_text(bbox)}'
        )
        self._publish_feedback('P acquired, visual final approach started')

    def _run_p_approach(self):
        detected, conf, bbox, _stamp, offset, fill = self._p_detection()
        if not detected or conf < self.p_approach_conf:
            now = self._now_sec()
            if self._p_terminal_pass_armed:
                self.stop_robot()
                self.log.segment(
                    'P terminal pass confirmed: stable near-and-centered P disappeared; '
                    'stop at the physical terminal without map-coordinate fallback'
                )
                self._finish_mission('return complete, P terminal pass confirmed')
                return
            self._p_terminal_pass_candidate_since = None
            if self._p_lost_since is None:
                self._p_lost_since = now
                self._p_lost_reverse_started_at = None
                self._p_recovery_target_yaw = self._p_heading_before_loss(now)
                self.desired_heading = self._p_recovery_target_yaw
                self.log.telemetry(
                    'P_LOSS_GRACE',
                    f'P temporarily lost; hold for {self.p_loss_grace_period:.2f}s '
                    'before recovery reverse',
                )

            loss_elapsed = now - self._p_lost_since
            # Once P has been confirmed, a loss inside the terminal zone is
            # usually caused by the board filling the camera or depth/vision
            # dropout. Keep converging to the calibrated terminal point; do
            # not back away from the finish and let lidar classify the board.
            if self._p_ever_confirmed and self._terminal_fallback_ready():
                self._resume_map_search_after_p_loss(0.0, True, 0.0)
                self.log.mission(
                    'P lost inside terminal zone; skip recovery reverse and '
                    'continue smooth terminal convergence'
                )
                return
            if loss_elapsed < self.p_loss_grace_period:
                self._publish_state('p_visual_loss_grace')
                self.cmd_pub.publish(self._twist(0.0, 0.0))
                return

            if self._p_lost_reverse_started_at is None:
                self._p_lost_reverse_started_at = now
                self.log.warn(
                    'P_DETECTION',
                    'P remained lost after grace period; reversing toward the locked '
                    'visual heading '
                    f'({self._format_optional_yaw_deg(self._p_recovery_target_yaw)})'
                )
                self._publish_feedback('P lost: reverse toward the locked visual heading')

            reverse_elapsed = now - self._p_lost_reverse_started_at
            target_yaw = self._p_recovery_target_yaw
            target_text = self._format_optional_yaw_deg(target_yaw)
            heading_error = (
                self._angle_error(target_yaw, self.current_yaw)
                if target_yaw is not None and self.current_yaw is not None else 0.0
            )
            heading_restored = abs(heading_error) <= self.p_loss_heading_tolerance
            if reverse_elapsed < self.p_loss_reverse_duration:
                reverse_angular = self._clamp(
                    self.p_loss_heading_kp * heading_error,
                    self.p_loss_reverse_max_angular,
                )
                self._publish_state('p_visual_recover')
                self.log.telemetry(
                    'P_RECOVER',
                    f'loss_t={reverse_elapsed:.2f}s v={-self.p_loss_reverse_speed:.2f} '
                    f'target={target_text} '
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
            applied_delta = 0.0
            if self._p_target_yaw is None or self._p_heading_lock_offset is None:
                self._p_target_yaw = self._visual_target_yaw(self._p_offset_filtered)
            else:
                # Apply only the image-space delta. Rebuilding from current
                # IMU yaw makes the terminal target drift while the chassis turns.
                offset_delta = self._p_offset_filtered - self._p_heading_lock_offset
                requested_delta = -self.p_heading_bearing_gain * offset_delta
                applied_delta = self._clamp(
                    requested_delta,
                    self.p_heading_reacquire_max_delta,
                )
                self._p_target_yaw = self._normalize_angle(
                    self._p_target_yaw + applied_delta
                )
            self._p_heading_lock_offset = self._p_offset_filtered
            self._p_last_heading_reacquire_at = now
            self.log.segment(
                f'P heading reacquire offset={self._p_offset_filtered:+.3f} '
                f'target={math.degrees(self._p_target_yaw):.1f}deg '
                f'delta={math.degrees(applied_delta):+.1f}deg'
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
        self._update_p_terminal_pass_evidence(now, conf, fill)
        depth_m, samples, depth_status = self._p_depth_measurement(bbox)
        self._update_p_final_visual_evidence(fill, depth_m, depth_status)
        if (
            depth_m is not None
            and depth_m <= self.p_approach_disable_avoidance_distance
        ):
            self._p_final_approach_latched = True
        if self._try_arm_p_final_odometry(depth_m, samples, depth_status, fill):
            self._run_p_final_odometry()
            return
        if self._check_p_approach_stall(linear):
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

    def _update_p_terminal_pass_evidence(self, now, conf, fill):
        """Arm terminal completion only from stable, close, centered P evidence."""
        if not self.p_terminal_pass_enabled or self._p_terminal_pass_armed:
            return
        close_and_centered = (
            conf >= self.p_approach_conf
            and fill >= self.p_terminal_pass_fill_ratio
            and abs(self._p_offset_filtered) <= self.p_terminal_pass_center_offset
        )
        if not close_and_centered:
            self._p_terminal_pass_candidate_since = None
            return
        if self._p_terminal_pass_candidate_since is None:
            self._p_terminal_pass_candidate_since = now
            return
        if now - self._p_terminal_pass_candidate_since < self.p_terminal_pass_evidence_hold:
            return
        self._p_terminal_pass_armed = True
        self.log.segment(
            'P terminal pass armed: '
            f'conf={conf:.2f} fill={fill:.2%} off={self._p_offset_filtered:+.3f} '
            f'hold={now - self._p_terminal_pass_candidate_since:.2f}s; '
            'P disappearance will complete without map-coordinate fallback'
        )

    def _try_arm_p_final_odometry(
        self, depth_m=None, samples=None, depth_status=None, fill=None,
    ):
        """Start the final run before any lidar avoidance can arbitrate a command."""
        if self._p_final_run_start_odom is not None:
            return True
        trigger_text = None
        if depth_m is None:
            detected, conf, bbox, _stamp, _offset, detected_fill = self._p_detection()
            if detected and conf >= self.p_approach_conf:
                depth_m, samples, depth_status = self._p_depth_measurement(bbox)
                fill = detected_fill
                self._update_p_final_visual_evidence(fill, depth_m, depth_status)
        if depth_m is not None and depth_m <= self.p_depth_stop_distance:
            trigger_text = (
                f'depth={depth_m:.3f}m <= {self.p_depth_stop_distance:.3f}m '
                f'samples={samples} ({depth_status})'
            )
        elif self._p_visual_near_ready():
            depth_m = self._p_last_valid_depth_m
            trigger_text = (
                f'visual_fill={self._p_last_fill_ratio:.2%} >= '
                f'{self.p_final_visual_fill_trigger:.2%}; recent_depth='
                f'{self._p_last_valid_depth_m:.3f}m <= '
                f'{self.p_final_visual_depth_assist:.3f}m '
                f'({self._p_last_valid_depth_status})'
            )
        elif self._p_visual_no_depth_near_ready():
            trigger_text = (
                f'visual_only_fill={self._p_last_fill_ratio:.2%} >= '
                f'{self.p_final_visual_no_depth_fill_trigger:.2%}; '
                'depth_roi_unavailable'
            )
        else:
            return False
        if self._last_raw_odom_xy is None or self._p_target_yaw is None:
            self.stop_robot()
            self._publish_state('p_final_waiting_for_odom')
            return True

        self._p_final_approach_latched = True
        self._p_final_run_start_odom = self._last_raw_odom_xy
        self._p_final_run_target_yaw = self._p_target_yaw
        self._p_final_run_depth_m = depth_m
        self._p_final_run_trigger = trigger_text
        self._p_final_last_progress_odom = self._last_raw_odom_xy
        self._p_final_last_progress_at = self._now_sec()
        self.desired_heading = self._p_final_run_target_yaw
        self.avoid_state = 'forward'
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.emergency_reverse_deadline = None
        self.recovery_uses_heading = False
        self.log.segment(
            f'P final odometry run armed: {trigger_text} '
            f'runout={self.p_final_odom_travel:.3f}m '
            f'v={self.p_approach_slow_linear:.2f}; lidar avoidance disabled'
        )
        self._publish_feedback('P is within 0.5m: final low-speed odometry run started')
        return True

    def _run_p_final_odometry(self):
        """Complete the calibrated final 0.5m after P depth enters its close zone."""
        if (
            self._p_final_run_start_odom is None
            or self._p_final_run_target_yaw is None
            or self._last_raw_odom_xy is None
            or self.current_yaw is None
        ):
            self.stop_robot()
            self._publish_state('p_final_waiting_for_odom')
            return
        travelled = math.hypot(
            self._last_raw_odom_xy[0] - self._p_final_run_start_odom[0],
            self._last_raw_odom_xy[1] - self._p_final_run_start_odom[1],
        )
        final_speed = self.p_approach_slow_linear
        brake_distance = (
            final_speed * self.p_final_brake_response
            + final_speed * final_speed / (2.0 * self.p_final_brake_decel)
            + self.p_final_brake_margin
        )
        completion_floor = max(
            0.0,
            self.p_final_odom_travel - max(
                brake_distance, self.p_final_completion_tolerance,
            ),
        )
        now = self._now_sec()
        if self._p_final_last_progress_odom is None:
            self._p_final_last_progress_odom = self._last_raw_odom_xy
            self._p_final_last_progress_at = now
        else:
            progress_delta = math.hypot(
                self._last_raw_odom_xy[0] - self._p_final_last_progress_odom[0],
                self._last_raw_odom_xy[1] - self._p_final_last_progress_odom[1],
            )
            if progress_delta >= self.p_final_progress_min_delta:
                self._p_final_last_progress_odom = self._last_raw_odom_xy
                self._p_final_last_progress_at = now
            elif (
                self._p_final_last_progress_at is not None
                and now - self._p_final_last_progress_at >= self.p_final_progress_timeout
            ):
                self.stop_robot()
                if travelled >= self.p_final_stall_completion_min:
                    self.log.segment(
                        f'P final stall complete: travelled={travelled:.3f}m '
                        f'min={self.p_final_stall_completion_min:.3f}m '
                        f'no_progress={now - self._p_final_last_progress_at:.2f}s '
                        f'target={self.p_final_odom_travel:.3f}m '
                        f'depth_start={self._p_final_depth_start_text()} '
                        f'trigger={self._p_final_run_trigger}'
                    )
                    self._finish_mission('return complete, P final run reached terminal wall')
                else:
                    self._publish_state('p_final_terminal_guard')
                    self.log.warn(
                        'P_FINAL_ODOMETRY',
                        f'no odom progress for {now - self._p_final_last_progress_at:.2f}s '
                        f'before minimum terminal run: travelled={travelled:.3f}m '
                        f'min={self.p_final_stall_completion_min:.3f}m',
                    )
                return
        if travelled >= completion_floor:
            self.stop_robot()
            self.log.segment(
                f'P final brake: travelled={travelled:.3f}m '
                f'target={self.p_final_odom_travel:.3f}m '
                f'completion_floor={completion_floor:.3f}m '
                f'brake_distance={brake_distance:.3f}m '
                f'depth_start={self._p_final_depth_start_text()} '
                f'trigger={self._p_final_run_trigger}'
            )
            self._finish_mission('return complete, P visual final braking window reached')
            return
        heading_error = self._angle_error(self._p_final_run_target_yaw, self.current_yaw)
        angular = self._clamp(
            self.p_heading_kp * heading_error,
            self.p_final_heading_max_angular,
        )
        if abs(heading_error) <= self.p_final_heading_tolerance:
            angular = 0.0
        self._publish_state('p_final_odometry')
        self.log.telemetry(
            'P_FINAL_ODOMETRY',
            f'travelled={travelled:.3f}/{self.p_final_odom_travel:.3f}m '
            f'remain={self.p_final_odom_travel - travelled:.3f}m '
            f'brake_at={completion_floor:.3f}m brake_distance={brake_distance:.3f}m '
            f'depth_start={self._p_final_depth_start_text()} '
            f'heading_err={math.degrees(heading_error):+.1f}deg '
            f'v={final_speed:.2f} w={angular:.2f}',
        )
        self.cmd_pub.publish(self._twist(final_speed, angular))

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
        self._p_final_run_start_odom = None
        self._p_final_run_target_yaw = None
        self._p_final_run_depth_m = None
        self._p_final_run_trigger = ''
        self._p_final_last_progress_odom = None
        self._p_final_last_progress_at = None
        self._reset_p_final_visual_evidence()
        self._p_visible_yaw_history.clear()
        self.desired_heading = None
        self._filtered_heading_err = 0.0
        self._reset_p_approach_progress_watchdog()
        recovery_reason = 'heading_restored' if heading_restored else 'reverse_timeout'
        self._publish_state('map_search_p')
        self.log.mission(
            f'P lost recovery complete ({recovery_reason}): elapsed={reverse_elapsed:.2f}s '
            f'target={target_deg:.1f}deg error={math.degrees(heading_error):+.1f}deg; '
            'resume lidar-protected navigation to visual search goal'
        )
        self._publish_feedback('P occluded: resume navigation to visual search goal')

    def _terminal_fallback_ready(self):
        if (
            not self.terminal_fallback_enabled
            or not self._p_ever_confirmed
            or self.current_position is None
        ):
            return False
        distance = math.hypot(
            self.terminal_target_map[0] - self.current_position[0],
            self.terminal_target_map[1] - self.current_position[1],
        )
        return distance <= self.terminal_acquire_distance

    def _run_terminal_fallback_drive(self):
        """Smoothly converge to the terminal point after P was confirmed."""
        if self.current_position is None or self.current_yaw is None:
            self.stop_robot()
            return
        dx = self.terminal_target_map[0] - self.current_position[0]
        dy = self.terminal_target_map[1] - self.current_position[1]
        distance = math.hypot(dx, dy)
        now = self._now_sec()
        if distance <= self.terminal_fallback_tolerance:
            self.stop_robot()
            if self._terminal_fallback_hold_since is None:
                self._terminal_fallback_hold_since = now
                self._publish_state('terminal_done_hold')
                self.log.segment(
                    f'terminal fallback reached: distance={distance:.3f}m; '
                    f'holding {self.terminal_fallback_completion_hold:.2f}s'
                )
                return
            if now - self._terminal_fallback_hold_since >= self.terminal_fallback_completion_hold:
                self._finish_mission('return complete, smooth terminal fallback reached')
            return

        self._terminal_fallback_hold_since = None
        target_yaw = math.atan2(dy, dx)
        heading_error = self._angle_error(target_yaw, self.current_yaw)
        angular = self._clamp(
            self.terminal_fallback_heading_kp * heading_error,
            self.terminal_fallback_max_angular,
        )
        # Couple speed to both remaining distance and heading error. This
        # makes large errors turn decisively while small errors settle gently.
        speed = min(self.terminal_fallback_linear_speed, max(0.05, distance * 0.8))
        if abs(heading_error) > math.radians(45.0):
            speed = min(speed, 0.10)
        self.desired_heading = target_yaw
        self._publish_state('terminal_fallback_drive')
        self.log.telemetry(
            'TERMINAL_FALLBACK',
            f'dist={distance:.3f}m target=({self.terminal_target_map[0]:.2f},'
            f'{self.terminal_target_map[1]:.2f}) heading_err='
            f'{math.degrees(heading_error):+.1f}deg v={speed:.2f} w={angular:.2f}',
        )
        self.cmd_pub.publish(self._twist(speed, angular))

    @staticmethod
    def _format_optional_yaw_deg(yaw_rad):
        if yaw_rad is None:
            return 'nan'
        return f'{math.degrees(yaw_rad):.1f}deg'

    def _reset_p_approach_progress_watchdog(self):
        self._p_approach_progress_odom = self._last_raw_odom_xy
        self._p_approach_progress_at = self._now_sec()

    def _check_p_approach_stall(self, commanded_linear):
        if (
            not self.p_approach_stall_reverse_enabled
            or commanded_linear <= 0.0
            or self._p_final_run_start_odom is not None
            or self._p_final_approach_latched
            or self._last_raw_odom_xy is None
        ):
            self._reset_p_approach_progress_watchdog()
            return False
        now = self._now_sec()
        if self._p_approach_progress_odom is None:
            self._reset_p_approach_progress_watchdog()
            return False
        progress_delta = math.hypot(
            self._last_raw_odom_xy[0] - self._p_approach_progress_odom[0],
            self._last_raw_odom_xy[1] - self._p_approach_progress_odom[1],
        )
        if progress_delta >= self.p_approach_progress_min_delta:
            self._p_approach_progress_odom = self._last_raw_odom_xy
            self._p_approach_progress_at = now
            return False
        if (
            self._p_approach_progress_at is None
            or now - self._p_approach_progress_at < self.p_approach_progress_timeout
        ):
            return False
        self._begin_p_approach_stall_reverse(
            now - self._p_approach_progress_at,
            progress_delta,
        )
        self._run_emergency_reverse()
        return True

    def _begin_p_approach_stall_reverse(self, stalled_sec, progress_delta):
        obstacle = None
        if self.latest_scan is not None:
            obstacle = self._find_nearest_obstacle(self.latest_scan, emergency=True)
            if obstacle is None:
                obstacle = self._find_nearest_obstacle(self.latest_scan, emergency=False)

        if obstacle is not None:
            self._begin_emergency_reverse(obstacle)
            self.log.warn(
                'P_APPROACH_STALL',
                f'forward command stalled for {stalled_sec:.2f}s '
                f'progress={progress_delta:.3f}m; use lidar obstacle '
                f'dist={obstacle["dist"]:.2f}m danger={obstacle["danger_deg"]:.1f}deg',
            )
            return

        if abs(self._p_offset_filtered) > 0.05:
            self.avoid_turn_direction = -math.copysign(1.0, self._p_offset_filtered)
            direction_source = f'p_offset={self._p_offset_filtered:+.3f}'
        elif abs(self._p_last_angular) > 1e-3:
            self.avoid_turn_direction = math.copysign(1.0, self._p_last_angular)
            direction_source = f'last_angular={self._p_last_angular:+.2f}'
        else:
            self.avoid_turn_direction = 1.0
            direction_source = 'fallback_left'
        self.avoid_state = 'emergency_reversing'
        self.emergency_reverse_deadline = self.get_clock().now() + Duration(
            seconds=self.emergency_reverse_duration
        )
        self._publish_state('emergency_reversing')
        self.log.warn(
            'P_APPROACH_STALL',
            f'forward command stalled for {stalled_sec:.2f}s '
            f'progress={progress_delta:.3f}m; no lidar cluster in window, '
            f'reverse for {self.emergency_reverse_duration:.2f}s '
            f'dir={self.avoid_turn_direction:.0f} {direction_source}',
        )
        self._publish_feedback('P approach stalled: reverse and retry obstacle clearance')

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

        # The anchored map route is only a repeatable visual search route. It
        # never declares success; P detection takes over whenever it appears.
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
            return self._p_final_run_start_odom is not None
        if self._p_final_approach_latched:
            return True
        if self._p_final_run_start_odom is not None or self._p_visual_near_ready():
            self._p_final_approach_latched = True
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
        if self._p_approaching:
            self._reset_p_approach_progress_watchdog()
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
    install_parent_death_signal()
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

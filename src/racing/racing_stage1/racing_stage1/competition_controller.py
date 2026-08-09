"""Production Stage1 navigation controller.

The node deliberately owns the only motion publisher used by Stage1.  It
implements the same safety boundaries as the Nav2 Humble combination used on
the board, but keeps the stage lifecycle in one process:

* a heading-aware, footprint-inflated grid search for the global route;
* an MPPI-style short-horizon trajectory sampler using the live scan;
* an independent TTC/footprint collision monitor applied to the final command.

The static map is never modified by scan returns.  IMU gyro rate supplies the
short-term heading delta, while lidar/map wall geometry supplies the absolute
heading correction; odometry/TF provide position only.
"""

import heapq
import json
import math
import os
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from racing_common.imu_distance_pose import ImuDistancePose
from racing_common.process_lifecycle import install_parent_death_signal
from racing_common.racing_logger import RacingLogger, terminal_write


class CompetitionController(Node):
    """Single S1 lifecycle node and single /cmd_vel publisher."""

    MISSION_STANDBY = 'standby'
    MISSION_SEARCH_QR = 'search_qr'
    MISSION_QR_LOCKED = 'qr_locked'
    MISSION_RETURN_TO_ENTRY = 'return_to_entry'
    MISSION_HANDOFF_WAIT = 'handoff_wait'

    def __init__(self):
        super().__init__('competition_controller')

        self._declare_parameters()
        self._read_parameters()

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        latest_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._sensor_group = ReentrantCallbackGroup()
        self._control_group = MutuallyExclusiveCallbackGroup()
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.stage1_state_topic, latched)
        self.task_pub = self.create_publisher(String, self.task_topic, latched)
        self.entry_pose_pub = self.create_publisher(PoseStamped, self.entry_pose_topic, latched)
        self.imu_offset_pub = self.create_publisher(Float64, self.imu_offset_topic, latched)
        self.map_heading_pub = self.create_publisher(Float64, self.map_heading_topic, 10)
        self.map_pose_pub = self.create_publisher(PoseStamped, self.map_pose_topic, 10)
        self.route_pub = self.create_publisher(Path, self.route_topic, latched)
        self.mission_route_pub = self.create_publisher(Path, self.mission_route_topic, latched)

        prefix = self.lifecycle_service_prefix.rstrip('/')
        self._activate_srv = self.create_service(
            Trigger, f'{prefix}/activate', self._activate_cb, callback_group=self._control_group)
        self._release_srv = self.create_service(
            Trigger, f'{prefix}/release', self._release_cb, callback_group=self._control_group)

        self.create_subscription(
            LaserScan, self.scan_topic, self._scan_cb, latest_sensor,
            callback_group=self._sensor_group)
        self.create_subscription(
            Imu, self.imu_topic, self._imu_cb, latest_sensor,
            callback_group=self._sensor_group)
        self.create_subscription(
            Odometry, self.raw_odom_topic, self._raw_odom_cb, latest_sensor,
            callback_group=self._sensor_group)
        self.create_subscription(
            Odometry, self.odom_topic, self._odom_cb, latest_sensor,
            callback_group=self._sensor_group)
        self.create_subscription(
            Float64, self.lidar_heading_topic, self._lidar_heading_cb, 10,
            callback_group=self._control_group)
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_cb, latched,
            callback_group=self._control_group)
        self.create_subscription(
            String, self.qr_result_topic, self._qr_cb, 10,
            callback_group=self._control_group)
        self.create_subscription(
            String, self.diagnostic_topic, self._diagnostic_cb, latched,
            callback_group=self._control_group)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.log = RacingLogger(self, log_subdir='competition_stage1',
                                log_filename='latest.log', session_title='Stage1 Nav2-style navigation')
        self._localization_trace_path = os.path.join(
            os.path.dirname(self.log.path), 'localization_trace.jsonl'
        )
        self._localization_trace_file = None
        try:
            self._localization_trace_file = open(
                self._localization_trace_path, 'w', encoding='utf-8', buffering=1
            )
            self._localization_trace_file.write(json.dumps({
                'type': 'header',
                'schema': 'stage1_localization_consistency_v3',
                'note': 'Compact raw-wheel, EKF and lidar/map consistency records; rejected localization never updates navigation.',
            }, separators=(',', ':')) + '\n')
            self.log.config(f'localization trace path={self._localization_trace_path}')
        except OSError as exc:
            self.log.warn('LOCALIZATION_TRACE', f'cannot open trace file: {exc}')
        self._lock = threading.RLock()
        self._trace_lock = threading.Lock()
        self._map = None
        self._map_blocked = None
        self._lidar_distance_map = None
        self._map_signature = None
        self._scan = None
        self._odom_xy = None
        self._last_scan_stamp = None
        self._last_map_time = None
        self._current_raw_yaw = None
        self._gyro_relative_yaw = 0.0
        self._gyro_anchor_relative_yaw = 0.0
        self._last_imu_stamp = None
        self._last_imu_rate_z = None
        self._last_odom_stamp = None
        self._last_odom_twist = None
        self._raw_odom = None
        self._odom_fault_reason = None
        self._map_xy_correction = (0.0, 0.0)
        self._lidar_position_fault_reason = None
        self._lidar_position_mismatch_count = 0
        self._last_lidar_position_check_stamp = None
        self._lidar_position_check_in_progress = False
        self._lidar_position_pending = deque(maxlen=self.lidar_position_stable_frames)
        # Sensor callbacks can be delayed while the Python planners run. Keep
        # enough history to project each scan at the pose when it was taken.
        self._pose_history = deque(maxlen=256)
        self._yaw_history = deque(maxlen=512)
        self._lidar_corrected_yaw = None
        self._last_lidar_heading_time = None
        self._initial_raw_yaw = None
        self._start_map_xy = None
        self._start_map_yaw = None
        self._start_odom_xy = None
        self._map_pose_xy = None
        self._map_motion_direction = 1.0
        self._map_pose_integrator = ImuDistancePose(
            max_step_m=self.map_pose_odom_max_step,
            min_step_m=self.map_pose_odom_min_step,
        )
        self.current_yaw = None
        self._heading_anchor_yaw = None
        self._heading_motion_active = False
        self._last_pose = None
        self._last_pose_time = 0.0
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._last_motion_cmd = Twist()
        self._last_motion_cmd_time = 0.0
        self._last_plan_time = 0.0
        self._route_target_signature = None
        self._route_locked = False
        self._local_failure_since = None
        self._last_failure_replan_time = 0.0
        self._plan_in_progress = False
        self._plan_generation = 0
        self._plan_thread = None
        self._local_plan_in_progress = False
        self._local_plan_generation = 0
        self._local_plan_thread = None
        self._local_result = None
        self._local_result_time = 0.0
        self._last_safe_result_time = 0.0
        self._last_local_request_time = 0.0
        self._last_local_request_scan_key = None
        self._last_local_request_generation = -1
        self._local_replan_period = 0.10
        self._local_command_hold = 0.55
        self._path = []
        self._path_headings = []
        self._path_gears = []
        self._path_progress_index = 0
        self._target_name = ''
        self._target_xy = None
        self._target_yaw = None
        self._mission_state = self.MISSION_STANDBY
        self._qr_pose_xy = None
        self._qr_pose_yaw = None
        self._qr_pose_odom_xy = None
        self._qr_locked_at = None
        self._entry_stable_since = None
        self._qr_task = ''
        self._qr_latched = False
        self._entry_announced = False
        self._activation_requested = False
        self._motion_enabled = False
        self._released = False
        self._start_after = None
        self._ready_published = False
        self._running_published = False
        self._handoff_wait = False
        self._last_safety_log = 0.0
        self._scan_tf_reported = False
        self._scan_transform_cache = None
        self._scan_transform_frame = ''
        self._scan_transform_time = 0.0
        self._base_link_tf_reported = False
        self._base_link_offset = (0.0, 0.0, 0.0)
        self._footprint_samples_cache = None
        self._footprint_samples_signature = None
        self._last_scan_diagnostic = None
        self._last_local_failure = None
        self._last_route_heading = None
        self._last_local_gear = 'F'
        self._last_route_gear = 1
        self._diagnostic_state = 'waiting'
        self._control_timer = self.create_timer(
            1.0 / self.control_rate_hz, self._control_loop,
            callback_group=self._control_group)
        self._lidar_position_timer = self.create_timer(
            self.lidar_position_check_period, self._check_lidar_position,
            callback_group=self._sensor_group)

        self.log.startup(
            f'S1 production controller: Hybrid-A* style global search + MPPI local sampler + '
            f'collision monitor; map={self.map_topic} scan={self.scan_topic}'
        )
        terminal_write('[STARTUP] S1 Nav2-style 三层控制器已启动，等待定位与 Supervisor activate')

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        defaults = {
            'output_cmd_topic': '/cmd_vel', 'scan_topic': '/scan',
            'imu_topic': '/imu/data', 'raw_odom_topic': '/odom',
            'odom_topic': '/odom_combined',
            'map_topic': '/map', 'map_frame': 'map', 'base_frame': 'base_footprint',
            'odom_frame': 'odom_combined', 'stage1_state_topic': 'stage1_state',
            'qr_result_topic': 'qr_scan_result', 'task_topic': 'competition_qr_task',
            'stage2_entry_pose_topic': 'stage2_entry_pose',
            'imu_map_yaw_offset_topic': 'imu_map_yaw_offset',
            'map_heading_topic': 'map_heading',
            'map_pose_topic': 'stage1_map_pose',
            'lidar_heading_topic': 'map_heading_lidar',
            'lidar_heading_max_age_sec': 0.80,
            'map_pose_odom_min_step_m': 0.002,
            'map_pose_odom_max_step_m': 0.30,
            'route_topic': 'stage1_route',
            'mission_route_topic': 'stage1_mission_route',
            'start_corner_diagnostic_topic': 'start_corner_pose_diagnostic',
            'heading_motion_linear_threshold_mps': 0.001,
            'heading_motion_angular_threshold_rad_s': 0.005,
            'imu_max_integration_gap_sec': 2.50,
            'angular_reversal_deadband_rad_s': 0.08,
            'lifecycle_service_prefix': '/competition/stage1', 'control_rate_hz': 20.0,
            'start_delay_sec': 0.0,
            'startup_straight_distance_m': 0.0,
            'localization_max_age_sec': 1.50,
            'odom_max_speed_mps': 1.20,
            'odom_jump_tolerance_m': 0.12,
            'odom_validation_max_gap_sec': 0.50,
            'lidar_position_check_period_sec': 0.50,
            'lidar_position_search_range_m': 0.80,
            'lidar_position_coarse_step_m': 0.08,
            'lidar_position_fine_range_m': 0.10,
            'lidar_position_fine_step_m': 0.01,
            'lidar_position_global_step_m': 0.10,
            'lidar_position_min_scan_points': 40,
            'lidar_position_max_scan_points': 160,
            'lidar_position_max_score_distance_m': 0.35,
            'lidar_position_inlier_distance_m': 0.10,
            'lidar_position_min_inlier_ratio': 0.45,
            'lidar_position_max_mean_distance_m': 0.12,
            'lidar_position_stable_frames': 3,
            'lidar_position_stable_spread_m': 0.08,
            'lidar_position_min_correction_m': 0.03,
            'lidar_position_max_auto_correction_m': 0.25,
            'lidar_position_fault_threshold_m': 0.25,
            'qr_goal_x_m': 4.50, 'qr_goal_y_m': 1.60,
            'qr_search_radius_m': 0.45,
            'channel_entry_x_m': 2.50, 'channel_entry_y_m': 2.50,
            'channel_entry_yaw_deg': 90.0, 'channel_entry_tolerance_m': 0.16,
            'channel_entry_yaw_tolerance_deg': 12.0,
            'entry_stable_sec': 0.25,
            'global_plan_retry_period_sec': 1.50,
            'global_replan_after_local_failure_sec': 2.00,
            'global_route_deviation_threshold_m': 0.45,
            'global_replan_cooldown_sec': 3.00,
            'planner_grid_step_m': 0.16, 'planner_heading_bins': 32,
            'planner_motion_step_m': 0.20, 'planner_max_expansions': 30000,
            'planner_fast_grid_step_m': 0.12,
            'planner_fast_corridor_length_m': 0.70,
            'planner_occupied_threshold': 50, 'planner_unknown_is_occupied': True,
            'planner_robot_radius_m': 0.26, 'planner_min_turn_radius_m': 0.62,
            'planner_turn_penalty': 0.35, 'planner_steer_change_penalty': 0.40,
            'planner_goal_yaw_tolerance_deg': 18.0,
            'robot_body_length_m': 0.276, 'robot_body_width_m': 0.164,
            'robot_footprint_margin_m': 0.02, 'scan_self_filter_margin_m': 0.04,
            'scan_static_match_tolerance_m': 0.16,
            'local_horizon_sec': 1.00, 'local_dt_sec': 0.15,
            'local_samples_speed': 3, 'local_samples_steer': 7,
            'local_nominal_speed_mps': 0.35, 'local_min_speed_mps': 0.12,
            'local_max_speed_mps': 0.45, 'local_min_turn_radius_m': 0.62,
            'local_goal_speed_floor_mps': 0.20,
            'local_goal_speed_gain_mps': 0.50,
            'local_goal_speed_cap_mps': 0.45,
            'local_max_angular_speed_rad_s': 0.75,
            'local_path_lookahead_m': 0.65, 'local_footprint_radius_m': 0.28,
            'local_obstacle_clearance_m': 0.10, 'local_dynamic_obstacle_max_range_m': 2.2,
            'local_scan_max_points': 32,
            'local_steer_change_penalty': 0.80,
            'local_heading_control_weight': 20.0, 'local_angular_effort_weight': 12.0,
            'local_path_distance_weight': 12.0, 'local_heading_weight': 2.5,
            'local_goal_weight': 4.0, 'local_clearance_weight': 3.0,
            'local_obstacle_cost_weight': 1000.0,
            'local_replan_period_sec': 0.20,
            'local_command_hold_sec': 1.20,
            'local_max_linear_accel_mps2': 1.50,
            'local_max_angular_accel_rad_s2': 2.50,
            'collision_stop_distance_m': 0.24, 'collision_slow_distance_m': 0.62,
            'collision_stop_ttc_sec': 0.55, 'collision_slow_ttc_sec': 1.40,
            'collision_slow_scale': 0.35, 'collision_forward_half_angle_deg': 38.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        get = lambda name: self.get_parameter(name).value
        self.output_cmd_topic = str(get('output_cmd_topic'))
        self.scan_topic = str(get('scan_topic'))
        self.imu_topic = str(get('imu_topic'))
        self.raw_odom_topic = str(get('raw_odom_topic'))
        self.odom_topic = str(get('odom_topic'))
        self.map_topic = str(get('map_topic'))
        self.map_frame = str(get('map_frame'))
        self.base_frame = str(get('base_frame'))
        self.odom_frame = str(get('odom_frame'))
        self.stage1_state_topic = str(get('stage1_state_topic'))
        self.qr_result_topic = str(get('qr_result_topic'))
        self.task_topic = str(get('task_topic'))
        self.entry_pose_topic = str(get('stage2_entry_pose_topic'))
        self.imu_offset_topic = str(get('imu_map_yaw_offset_topic'))
        self.map_heading_topic = str(get('map_heading_topic'))
        self.map_pose_topic = str(get('map_pose_topic'))
        self.lidar_heading_topic = str(get('lidar_heading_topic'))
        self.lidar_heading_max_age = max(0.2, float(get('lidar_heading_max_age_sec')))
        self.map_pose_odom_min_step = max(0.0, float(get('map_pose_odom_min_step_m')))
        self.map_pose_odom_max_step = max(
            self.map_pose_odom_min_step,
            float(get('map_pose_odom_max_step_m')),
        )
        self.route_topic = str(get('route_topic'))
        self.mission_route_topic = str(get('mission_route_topic'))
        self.diagnostic_topic = str(get('start_corner_diagnostic_topic'))
        self.heading_motion_linear_threshold = max(
            0.0, float(get('heading_motion_linear_threshold_mps'))
        )
        self.heading_motion_angular_threshold = max(
            0.0, float(get('heading_motion_angular_threshold_rad_s'))
        )
        self.imu_max_integration_gap = max(
            0.05, float(get('imu_max_integration_gap_sec'))
        )
        self.lifecycle_service_prefix = str(get('lifecycle_service_prefix'))
        self.control_rate_hz = max(5.0, float(get('control_rate_hz')))
        self.start_delay_sec = max(0.0, float(get('start_delay_sec')))
        self.startup_straight_distance = max(0.0, float(get('startup_straight_distance_m')))
        self.localization_max_age = max(0.1, float(get('localization_max_age_sec')))
        self.odom_max_speed = max(0.1, float(get('odom_max_speed_mps')))
        self.odom_jump_tolerance = max(0.0, float(get('odom_jump_tolerance_m')))
        self.odom_validation_max_gap = max(
            0.05, float(get('odom_validation_max_gap_sec')))
        self.lidar_position_check_period = max(
            0.1, float(get('lidar_position_check_period_sec')))
        self.lidar_position_search_range = max(
            0.1, float(get('lidar_position_search_range_m')))
        self.lidar_position_coarse_step = max(
            0.01, float(get('lidar_position_coarse_step_m')))
        self.lidar_position_fine_range = max(
            0.01, float(get('lidar_position_fine_range_m')))
        self.lidar_position_fine_step = max(
            0.005, float(get('lidar_position_fine_step_m')))
        self.lidar_position_global_step = max(
            0.05, float(get('lidar_position_global_step_m')))
        self.lidar_position_min_scan_points = max(
            16, int(get('lidar_position_min_scan_points')))
        self.lidar_position_max_scan_points = max(
            self.lidar_position_min_scan_points,
            int(get('lidar_position_max_scan_points')))
        self.lidar_position_max_score_distance = max(
            0.1, float(get('lidar_position_max_score_distance_m')))
        self.lidar_position_inlier_distance = max(
            0.02, float(get('lidar_position_inlier_distance_m')))
        self.lidar_position_min_inlier_ratio = min(
            1.0, max(0.1, float(get('lidar_position_min_inlier_ratio'))))
        self.lidar_position_max_mean_distance = max(
            0.02, float(get('lidar_position_max_mean_distance_m')))
        self.lidar_position_stable_frames = max(
            2, int(get('lidar_position_stable_frames')))
        self.lidar_position_stable_spread = max(
            0.01, float(get('lidar_position_stable_spread_m')))
        self.lidar_position_min_correction = max(
            0.0, float(get('lidar_position_min_correction_m')))
        self.lidar_position_max_auto_correction = max(
            self.lidar_position_min_correction,
            float(get('lidar_position_max_auto_correction_m')))
        self.lidar_position_fault_threshold = max(
            self.lidar_position_max_auto_correction,
            float(get('lidar_position_fault_threshold_m')))
        self.qr_goal = (float(get('qr_goal_x_m')), float(get('qr_goal_y_m')))
        self.qr_search_radius = max(0.05, float(get('qr_search_radius_m')))
        self.entry_goal = (float(get('channel_entry_x_m')), float(get('channel_entry_y_m')))
        self.entry_yaw = math.radians(float(get('channel_entry_yaw_deg')))
        self.entry_tolerance = max(0.05, float(get('channel_entry_tolerance_m')))
        self.entry_yaw_tolerance = math.radians(float(get('channel_entry_yaw_tolerance_deg')))
        self.entry_stable_sec = max(0.0, float(get('entry_stable_sec')))
        self.plan_retry_period = max(0.2, float(get('global_plan_retry_period_sec')))
        self.replan_after_local_failure = max(
            0.5, float(get('global_replan_after_local_failure_sec'))
        )
        self.route_deviation_threshold = max(
            0.10, float(get('global_route_deviation_threshold_m'))
        )
        self.replan_cooldown = max(0.5, float(get('global_replan_cooldown_sec')))
        self.grid_step = max(0.03, float(get('planner_grid_step_m')))
        self.heading_bins = max(8, int(get('planner_heading_bins')))
        self.motion_step = max(0.05, float(get('planner_motion_step_m')))
        self.max_expansions = max(1000, int(get('planner_max_expansions')))
        self.fast_grid_step = max(0.05, float(get('planner_fast_grid_step_m')))
        self.fast_corridor_length = max(
            self.fast_grid_step, float(get('planner_fast_corridor_length_m'))
        )
        self.occupied_threshold = int(get('planner_occupied_threshold'))
        self.unknown_occupied = bool(get('planner_unknown_is_occupied'))
        self.robot_radius = max(0.10, float(get('planner_robot_radius_m')))
        self.min_turn_radius = max(0.20, float(get('planner_min_turn_radius_m')))
        self.turn_penalty = float(get('planner_turn_penalty'))
        self.planner_steer_change_penalty = float(get('planner_steer_change_penalty'))
        self.goal_yaw_tolerance = math.radians(float(get('planner_goal_yaw_tolerance_deg')))
        self.body_length = max(0.10, float(get('robot_body_length_m')))
        self.body_width = max(0.08, float(get('robot_body_width_m')))
        self.footprint_margin = max(0.0, float(get('robot_footprint_margin_m')))
        self.scan_self_filter_margin = max(0.0, float(get('scan_self_filter_margin_m')))
        self.scan_static_match_tolerance = max(
            0.0, float(get('scan_static_match_tolerance_m'))
        )
        self.horizon = max(0.5, float(get('local_horizon_sec')))
        self.local_dt = max(0.03, float(get('local_dt_sec')))
        self.speed_samples = max(1, int(get('local_samples_speed')))
        self.steer_samples = max(3, int(get('local_samples_steer')))
        self.nominal_speed = max(0.05, float(get('local_nominal_speed_mps')))
        self.min_speed = max(0.03, float(get('local_min_speed_mps')))
        self.max_speed = max(self.min_speed, float(get('local_max_speed_mps')))
        self.goal_speed_floor = max(self.min_speed, float(get('local_goal_speed_floor_mps')))
        self.goal_speed_gain = max(0.0, float(get('local_goal_speed_gain_mps')))
        self.goal_speed_cap = max(self.goal_speed_floor, float(get('local_goal_speed_cap_mps')))
        self.local_min_turn_radius = max(self.min_turn_radius, float(get('local_min_turn_radius_m')))
        self.max_angular = max(0.05, float(get('local_max_angular_speed_rad_s')))
        self.lookahead = max(0.10, float(get('local_path_lookahead_m')))
        self.footprint_radius = max(self.robot_radius, float(get('local_footprint_radius_m')))
        self.clearance = max(0.02, float(get('local_obstacle_clearance_m')))
        self.dynamic_max_range = max(0.5, float(get('local_dynamic_obstacle_max_range_m')))
        self.local_scan_max_points = max(24, int(get('local_scan_max_points')))
        self.steer_change_penalty = float(get('local_steer_change_penalty'))
        self.heading_control_weight = max(0.0, float(get('local_heading_control_weight')))
        self.angular_effort_weight = max(0.0, float(get('local_angular_effort_weight')))
        self.path_weight = float(get('local_path_distance_weight'))
        self.heading_weight = float(get('local_heading_weight'))
        self.goal_weight = float(get('local_goal_weight'))
        self.clearance_weight = float(get('local_clearance_weight'))
        self.obstacle_cost_weight = float(get('local_obstacle_cost_weight'))
        self._local_replan_period = max(0.05, float(get('local_replan_period_sec')))
        self._local_command_hold = max(0.20, float(get('local_command_hold_sec')))
        self.max_linear_accel = max(0.05, float(get('local_max_linear_accel_mps2')))
        self.max_angular_accel = max(0.10, float(get('local_max_angular_accel_rad_s2')))
        self.angular_reversal_deadband = max(
            0.0, float(get('angular_reversal_deadband_rad_s')))
        self.stop_distance = max(0.05, float(get('collision_stop_distance_m')))
        self.slow_distance = max(self.stop_distance + 0.05, float(get('collision_slow_distance_m')))
        self.stop_ttc = max(0.1, float(get('collision_stop_ttc_sec')))
        self.slow_ttc = max(self.stop_ttc + 0.1, float(get('collision_slow_ttc_sec')))
        self.slow_scale = min(1.0, max(0.05, float(get('collision_slow_scale'))))
        self.forward_half_angle = math.radians(float(get('collision_forward_half_angle_deg')))

    # ------------------------------------------------------------------
    # Sensor and lifecycle callbacks
    # ------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _yaw_from_quaternion(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _norm(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _stamp_sec(msg):
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    @staticmethod
    def _history_value(history, stamp, angular=False):
        if not history:
            return None
        if stamp <= history[0][0]:
            return history[0][1]
        if stamp >= history[-1][0]:
            return history[-1][1]
        for before, after in zip(history, history[1:]):
            if stamp > after[0]:
                continue
            span = max(after[0] - before[0], 1e-9)
            ratio = (stamp - before[0]) / span
            if angular:
                return CompetitionController._norm(
                    before[1] + ratio * CompetitionController._norm(after[1] - before[1])
                )
            return tuple(
                before[1][axis] + ratio * (after[1][axis] - before[1][axis])
                for axis in range(len(before[1]))
            )
        return history[-1][1]

    def _scan_pose(self, scan):
        """Return the map pose that belongs to a scan's header timestamp."""
        if scan is None:
            return None
        stamp = self._stamp_sec(scan)
        with self._lock:
            pose_xy = self._history_value(list(self._pose_history), stamp)
            yaw = self._history_value(list(self._yaw_history), stamp, angular=True)
            if pose_xy is None:
                pose_xy = self._map_pose_xy or self._last_pose
            if yaw is None:
                yaw = self.current_yaw
        if pose_xy is None or yaw is None:
            return None
        return pose_xy, yaw

    def _imu_cb(self, msg):
        stamp = self._stamp_sec(msg)
        age = self._now() - stamp
        if age > self.localization_max_age or age < -0.20:
            return
        with self._lock:
            raw = self._yaw_from_quaternion(msg.orientation)
            self._current_raw_yaw = raw
            rate_z = float(msg.angular_velocity.z)
            if self._last_imu_stamp is None:
                self._last_imu_stamp = stamp
            else:
                dt = stamp - self._last_imu_stamp
                if 1e-4 <= dt <= self.imu_max_integration_gap:
                    # The board publishes IMU frames at an uneven rate.  Do
                    # not discard real turns just because one callback was
                    # delayed; trapezoidal integration limits a single noisy
                    # sample's influence across the gap.
                    previous_rate_z = self._last_imu_rate_z
                    if previous_rate_z is None:
                        previous_rate_z = rate_z
                    self._gyro_relative_yaw += 0.5 * (previous_rate_z + rate_z) * dt
                    self._last_imu_stamp = stamp
                elif dt > self.imu_max_integration_gap:
                    # A genuinely stale interval is a new integration segment;
                    # keeping it would turn sensor silence into a large yaw jump.
                    self._last_imu_stamp = stamp
            if self._start_map_yaw is None:
                self.current_yaw = None
            elif self._initial_raw_yaw is None:
                self._initial_raw_yaw = raw
                self._heading_anchor_yaw = self._start_map_yaw
                offset = self._norm(self._start_map_yaw - raw)
                self.imu_offset_pub.publish(Float64(data=offset))
                self.log.config(
                    f'IMU heading anchored to radar map={math.degrees(self._start_map_yaw):.1f}deg; '
                    f'offset={math.degrees(offset):+.1f}deg'
                )
                self.current_yaw = self._start_map_yaw
            else:
                if self._heading_motion_active:
                    self.current_yaw = self._norm(
                        self._heading_anchor_yaw + self._norm(
                            self._gyro_relative_yaw - self._gyro_anchor_relative_yaw
                        )
                    )
                else:
                    # A stationary IMU can drift substantially.  Keep the
                    # last radar/IMU-corrected map heading and continuously
                    # re-anchor the raw yaw while no motion command is being
                    # issued, so startup drift never becomes a fake turn.
                    self._initial_raw_yaw = raw
                    self._gyro_anchor_relative_yaw = self._gyro_relative_yaw
                    if self._heading_anchor_yaw is None:
                        self._heading_anchor_yaw = (
                            self.current_yaw if self.current_yaw is not None else self._start_map_yaw
                        )
                    self.current_yaw = self._heading_anchor_yaw
            self._publish_map_heading_locked()
            self._last_imu_rate_z = rate_z
            if self.current_yaw is not None:
                self._yaw_history.append((stamp, self.current_yaw))

    def _lidar_heading_cb(self, msg):
        try:
            corrected = self._norm(float(msg.data))
        except (TypeError, ValueError):
            return
        with self._lock:
            if self._start_map_yaw is None:
                return
            if self._heading_motion_active:
                # The localizer is expected to gate this already, but keep a
                # second boundary at the motion owner so a delayed or rogue
                # wall-pair message cannot reset the in-flight IMU integral.
                return
            self._lidar_corrected_yaw = corrected
            self._last_lidar_heading_time = self._now()
            self._heading_anchor_yaw = corrected
            if self._current_raw_yaw is not None:
                self._initial_raw_yaw = self._current_raw_yaw
            self._gyro_anchor_relative_yaw = self._gyro_relative_yaw
            self.current_yaw = corrected
            self._publish_map_heading_locked()

    def _raw_odom_cb(self, msg):
        now = self._now()
        record = {
            'type': 'raw_odom_frame',
            'recorded_at_sec': now,
            'stamp_sec': self._stamp_sec(msg),
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'vx': float(msg.twist.twist.linear.x),
            'vy': float(msg.twist.twist.linear.y),
            'wz': float(msg.twist.twist.angular.z),
        }
        record['message_age_sec'] = now - record['stamp_sec']
        record['speed_valid'] = math.hypot(record['vx'], record['vy']) <= self.odom_max_speed
        with self._lock:
            self._raw_odom = record
        self._write_localization_trace(record)

    def _odom_cb(self, msg):
        now = self._now()
        stamp = self._stamp_sec(msg)
        position = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        twist = (
            float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y),
            float(msg.twist.twist.angular.z),
        )
        with self._lock:
            dt = None if self._last_odom_stamp is None else stamp - self._last_odom_stamp
            displacement = None if self._odom_xy is None else math.hypot(
                position[0] - self._odom_xy[0], position[1] - self._odom_xy[1])
            # Validate displacement against the real message interval.  The
            # old cap made a normal delayed frame (0.757m over 1.7s) fail a
            # 0.72m limit, after which every later frame stayed rejected.
            allowed = None if dt is None else (
                self.odom_jump_tolerance + self.odom_max_speed * max(dt, 0.0)
            )
            values = (*position, *twist)
            reason = None
            if not all(math.isfinite(value) for value in values):
                reason = 'non_finite'
            elif now - stamp > self.localization_max_age:
                reason = f'stale_age={now - stamp:.3f}s'
            elif now - stamp < -0.20:
                reason = f'future_stamp={stamp - now:.3f}s'
            elif dt is not None and dt <= 0.0:
                reason = f'non_monotonic_dt={dt:.6f}s'
            elif math.hypot(twist[0], twist[1]) > self.odom_max_speed:
                reason = f'speed={math.hypot(twist[0], twist[1]):.3f}m/s'
            elif displacement is not None and allowed is not None and displacement > allowed:
                reason = f'jump={displacement:.3f}m allowed={allowed:.3f}m dt={dt:.3f}s'

            accepted = reason is None
            self._odom_fault_reason = reason
            odom_yaw = None
            if accepted:
                self._odom_xy = position
                self._last_odom_stamp = stamp
                self._last_odom_twist = twist
                odom_yaw = self._history_value(
                    list(self._yaw_history), stamp, angular=True)
                if odom_yaw is None:
                    odom_yaw = self.current_yaw
                if self._start_map_xy is not None and odom_yaw is not None:
                    self._map_pose_integrator.update(
                        self._odom_xy, odom_yaw, self._map_motion_direction
                    )
                    self._map_pose_xy = self._project_odom_xy_to_map_locked()
                    self._publish_map_pose_locked()
                    self._pose_history.append((stamp, self._map_pose_xy))
            raw_odom = None if self._raw_odom is None else dict(self._raw_odom)

        record = {
            'type': 'ekf_odom_frame',
            'recorded_at_sec': now,
            'stamp_sec': stamp,
            'message_age_sec': now - stamp,
            'x': position[0], 'y': position[1],
            'vx': twist[0], 'vy': twist[1], 'wz': twist[2],
            'dt_sec': dt,
            'displacement_m': displacement,
            'allowed_displacement_m': allowed,
            'accepted': accepted,
            'reason': reason,
            'projection_yaw_rad': None if not accepted or odom_yaw is None else odom_yaw,
            'raw_odom': raw_odom,
        }
        self._write_localization_trace(record)
        if reason is not None:
            self.log.telemetry('odom_consistency', f'rejected EKF frame: {reason}')

    def _scan_cb(self, msg):
        stamp = self._stamp_sec(msg)
        age = self._now() - stamp
        if age > self.localization_max_age or age < -0.20:
            return
        with self._lock:
            self._scan = msg
            self._last_scan_stamp = stamp

    def _lidar_match_score(self, rotated_x, rotated_y, x, y, info, distance_map):
        map_x = rotated_x + x
        map_y = rotated_y + y
        gx = np.floor((map_x - info.origin.position.x) / info.resolution).astype(np.int32)
        gy = np.floor((map_y - info.origin.position.y) / info.resolution).astype(np.int32)
        valid = (gx >= 0) & (gy >= 0) & (gx < info.width) & (gy < info.height)
        distances = np.full(rotated_x.shape, self.lidar_position_max_score_distance,
                            dtype=np.float32)
        if np.any(valid):
            distances[valid] = np.minimum(
                distance_map[gy[valid], gx[valid]], self.lidar_position_max_score_distance)
        return {
            'x': float(x),
            'y': float(y),
            'mean_distance_m': float(np.mean(distances)),
            'inlier_ratio': float(np.mean(distances <= self.lidar_position_inlier_distance)),
        }

    def _search_lidar_position(self, rotated_x, rotated_y, center_xy, search_range, step,
                               info, distance_map):
        count = max(1, int(round(2.0 * search_range / step)) + 1)
        xs = center_xy[0] - search_range + np.arange(count, dtype=np.float32) * step
        ys = center_xy[1] - search_range + np.arange(count, dtype=np.float32) * step
        best = None
        for x in xs:
            for y in ys:
                candidate = self._lidar_match_score(
                    rotated_x, rotated_y, x, y, info, distance_map)
                rank = (
                    self._lidar_match_valid(candidate),
                    candidate['inlier_ratio'],
                    -candidate['mean_distance_m'],
                )
                if best is None or rank > best[0]:
                    best = (rank, candidate)
        return None if best is None else best[1]

    def _lidar_match_valid(self, result):
        return (
            result is not None
            and result['inlier_ratio'] >= self.lidar_position_min_inlier_ratio
            and result['mean_distance_m'] <= self.lidar_position_max_mean_distance
        )

    def _check_lidar_position(self):
        with self._lock:
            if self._lidar_position_check_in_progress:
                return
            self._lidar_position_check_in_progress = True

        def run_check():
            try:
                self._check_lidar_position_worker()
            finally:
                with self._lock:
                    self._lidar_position_check_in_progress = False

        threading.Thread(target=run_check, name='s1_lidar_match', daemon=True).start()

    def _check_lidar_position_worker(self):
        with self._lock:
            scan = self._scan
            distance_map = self._lidar_distance_map
            info = None if self._map is None else self._map.info
        if scan is None or distance_map is None or info is None:
            return
        # Let the radar anchor and the independently checked odometry carry
        # the validated startup corridor.  A full-map scan search here is both
        # ambiguous and expensive while the robot is still at the corner.
        if self._map_pose_integrator.total_distance_m < self.startup_straight_distance:
            return
        stamp = self._stamp_sec(scan)
        if stamp == self._last_lidar_position_check_stamp:
            return
        # A queued scan describes an old robot pose.  Matching it against the
        # current map pose can produce a convincing, but completely unrelated,
        # full-map match and falsely trip the localization fault latch.
        if self._now() - stamp > self.localization_max_age:
            return
        scan_pose = self._scan_pose(scan)
        base_points = self._scan_points_base(scan)
        if (scan_pose is None or not base_points or
                len(base_points) < self.lidar_position_min_scan_points):
            return
        self._last_lidar_position_check_stamp = stamp
        if len(base_points) > self.lidar_position_max_scan_points:
            stride = int(math.ceil(len(base_points) / self.lidar_position_max_scan_points))
            base_points = base_points[::stride][:self.lidar_position_max_scan_points]
        scan_xy, scan_yaw = scan_pose
        points = np.asarray([(item[0], item[1]) for item in base_points], dtype=np.float32)
        cos_yaw, sin_yaw = math.cos(scan_yaw), math.sin(scan_yaw)
        rotated_x = cos_yaw * points[:, 0] - sin_yaw * points[:, 1]
        rotated_y = sin_yaw * points[:, 0] + cos_yaw * points[:, 1]
        current = self._lidar_match_score(
            rotated_x, rotated_y, scan_xy[0], scan_xy[1], info, distance_map)
        coarse = self._search_lidar_position(
            rotated_x, rotated_y, scan_xy, self.lidar_position_search_range,
            self.lidar_position_coarse_step, info, distance_map)
        best = self._search_lidar_position(
            rotated_x, rotated_y, (coarse['x'], coarse['y']), self.lidar_position_fine_range,
            self.lidar_position_fine_step, info, distance_map)
        local_dx = best['x'] - scan_xy[0]
        local_dy = best['y'] - scan_xy[1]
        local_error = math.hypot(local_dx, local_dy)
        search_kind = 'local'
        # Do not fall back to an unrestricted full-map search.  Repeated wall
        # geometry can produce a high-scoring but unrelated pose, and the
        # Python scan over a 5m map can block sensor callbacks for many seconds.

        correction = (best['x'] - scan_xy[0], best['y'] - scan_xy[1])
        correction_m = math.hypot(*correction)
        accepted = self._lidar_match_valid(best)
        current_valid = self._lidar_match_valid(current)
        action = 'insufficient_evidence'
        with self._lock:
            gross_mismatch = (
                accepted
                and not current_valid
                and correction_m >= self.lidar_position_fault_threshold
            )
            if gross_mismatch:
                self._lidar_position_pending.clear()
                self._lidar_position_mismatch_count += 1
                action = 'mismatch_pending'
                if self._lidar_position_mismatch_count >= self.lidar_position_stable_frames:
                    self._lidar_position_fault_reason = (
                        f'dx={correction[0]:+.3f}m,dy={correction[1]:+.3f}m,'
                        f'error={correction_m:.3f}m')
                    action = 'fault'
                    self.log.telemetry(
                        'lidar_position',
                        f'wall/map mismatch {self._lidar_position_fault_reason}; zero command')
            elif accepted and not current_valid:
                self._lidar_position_mismatch_count = 0
                if (self._lidar_position_pending and max(
                        math.hypot(correction[0] - item[0], correction[1] - item[1])
                        for item in self._lidar_position_pending
                ) > self.lidar_position_stable_spread):
                    self._lidar_position_pending.clear()
                self._lidar_position_pending.append(correction)
                if len(self._lidar_position_pending) == self.lidar_position_stable_frames:
                    correction = (
                        sum(item[0] for item in self._lidar_position_pending) /
                        self.lidar_position_stable_frames,
                        sum(item[1] for item in self._lidar_position_pending) /
                        self.lidar_position_stable_frames,
                    )
                    correction_m = math.hypot(*correction)
                    if correction_m < self.lidar_position_min_correction:
                        self._lidar_position_fault_reason = None
                        action = 'consistent'
                    elif correction_m <= self.lidar_position_max_auto_correction:
                        self._map_xy_correction = (
                            self._map_xy_correction[0] + correction[0],
                            self._map_xy_correction[1] + correction[1],
                        )
                        if self._map_pose_xy is not None:
                            self._map_pose_xy = (
                                self._map_pose_xy[0] + correction[0],
                                self._map_pose_xy[1] + correction[1],
                            )
                            self._pose_history = deque((
                                (history_stamp, (
                                    history_pose[0] + correction[0],
                                    history_pose[1] + correction[1],
                                )) if history_stamp >= stamp else (history_stamp, history_pose)
                                for history_stamp, history_pose in self._pose_history
                            ), maxlen=self._pose_history.maxlen)
                            self._publish_map_pose_locked()
                        self._lidar_position_fault_reason = None
                        action = 'corrected'
                        self.log.telemetry(
                            'lidar_position',
                            f'corrected dx={correction[0]:+.3f}m dy={correction[1]:+.3f}m '
                            f'error={correction_m:.3f}m')
                    elif correction_m >= self.lidar_position_fault_threshold:
                        self._lidar_position_fault_reason = (
                            f'dx={correction[0]:+.3f}m,dy={correction[1]:+.3f}m,'
                            f'error={correction_m:.3f}m')
                        action = 'fault'
                        self.log.telemetry(
                            'lidar_position',
                            f'wall/map mismatch {self._lidar_position_fault_reason}; zero command')
                    self._lidar_position_pending.clear()
            elif current_valid:
                self._lidar_position_pending.clear()
                self._lidar_position_mismatch_count = 0
                self._lidar_position_fault_reason = None
                action = 'consistent'
            else:
                self._lidar_position_pending.clear()
                self._lidar_position_mismatch_count = 0

        self._write_localization_trace({
            'type': 'lidar_position_match',
            'recorded_at_sec': self._now(),
            'stamp_sec': stamp,
            'search': search_kind,
            'scan_pose_x': scan_xy[0],
            'scan_pose_y': scan_xy[1],
            'scan_yaw': scan_yaw,
            'matched_x': best['x'],
            'matched_y': best['y'],
            'dx': correction[0],
            'dy': correction[1],
            'error_m': correction_m,
            'current_mean_distance_m': current['mean_distance_m'],
            'current_inlier_ratio': current['inlier_ratio'],
            'current_valid': current_valid,
            'matched_mean_distance_m': best['mean_distance_m'],
            'matched_inlier_ratio': best['inlier_ratio'],
            'accepted': accepted,
            'action': action,
            'points_used': int(points.shape[0]),
        })

    def _project_odom_xy_to_map_locked(self):
        """Project odometry distance using the IMU-only heading integrator."""
        if self._start_map_xy is None or self._map_pose_integrator.pose is None:
            return self._map_pose_xy
        # /odom already integrates wheel motion with its own chassis yaw.
        # Rotating that world XY again mixes two heading sources and mirrored
        # the observed forward Y displacement.  The shared integrator uses
        # only displacement magnitude plus the IMU heading.
        dx, dy = self._map_pose_integrator.pose
        return (
            self._start_map_xy[0] + dx + self._map_xy_correction[0],
            self._start_map_xy[1] + dy + self._map_xy_correction[1],
        )

    def _write_localization_trace(self, record):
        """Persist compact physical-consistency evidence outside the navigation lock."""
        if self._localization_trace_file is None:
            return
        try:
            with self._trace_lock:
                self._localization_trace_file.write(
                    json.dumps(record, separators=(',', ':')) + '\n')
        except OSError as exc:
            self.log.warn('LOCALIZATION_TRACE', f'write failed: {exc}')
            with self._trace_lock:
                self._localization_trace_file.close()
                self._localization_trace_file = None

    def _map_cb(self, msg):
        with self._lock:
            signature = (msg.info.width, msg.info.height, msg.info.resolution,
                         msg.info.origin.position.x, msg.info.origin.position.y,
                         len(msg.data), hash(bytes((int(v) & 0xff for v in msg.data))))
            self._map = msg
            self._last_map_time = self._now()
            if signature != self._map_signature:
                self._map_signature = signature
                self._map_blocked = self._build_inflated_map(msg)
                self._lidar_distance_map = self._build_lidar_distance_map(msg)
                self._path = []
                self._path_headings = []
                self._path_gears = []
                self._path_progress_index = 0
                self._route_target_signature = None
                self._route_locked = False
                self._local_failure_since = None
                self._publish_route_locked()
                self._plan_generation += 1
                self._plan_in_progress = False
                self._local_plan_generation += 1
                self._local_result = None
                self._local_result_time = 0.0
                self.log.config(f'static restricted map loaded {msg.info.width}x{msg.info.height} '
                                f'resolution={msg.info.resolution:.3f}m; footprint inflation={self.robot_radius:.2f}m')

    def _diagnostic_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            state = str(payload.get('state', 'unknown'))
            with self._lock:
                self._diagnostic_state = state
                if state == 'valid' and self._start_map_xy is None:
                    self._map_xy_correction = (0.0, 0.0)
                    self._lidar_position_fault_reason = None
                    self._lidar_position_mismatch_count = 0
                    self._lidar_position_pending.clear()
                    self._start_map_xy = (
                        float(payload['map_x']), float(payload['map_y'])
                    )
                    self._start_map_yaw = math.radians(float(payload['map_yaw_deg']))
                    self._start_odom_xy = (
                        float(payload['odom_x']), float(payload['odom_y'])
                    )
                    anchor_raw = self._current_raw_yaw
                    if anchor_raw is None:
                        anchor_raw = math.radians(float(payload['imu_raw_yaw_deg']))
                    self._initial_raw_yaw = anchor_raw
                    self._gyro_anchor_relative_yaw = self._gyro_relative_yaw
                    self._heading_anchor_yaw = self._start_map_yaw
                    self._heading_motion_active = False
                    self.current_yaw = self._start_map_yaw
                    self._map_pose_xy = self._start_map_xy
                    if self._odom_xy is not None:
                        self._map_pose_integrator.reset(self._odom_xy, self.current_yaw)
                        stamp = self._last_odom_stamp or self._now()
                        self._pose_history.append((stamp, self._map_pose_xy))
                    self._yaw_history.append((self._last_imu_stamp or self._now(), self.current_yaw))
                    self._lidar_corrected_yaw = self._start_map_yaw
                    self._last_lidar_heading_time = self._now()
                    offset = self._norm(self._start_map_yaw - anchor_raw)
                    self.imu_offset_pub.publish(Float64(data=offset))
                    self._publish_map_heading_locked()
                    self._publish_map_pose_locked()
                    self._publish_mission_route_locked()
                    self.log.config(
                        f'RADAR_START locked map=({self._start_map_xy[0]:.3f},'
                        f'{self._start_map_xy[1]:.3f}) '
                        f'yaw={math.degrees(self._start_map_yaw):.1f}deg '
                        f'odom_anchor=({self._start_odom_xy[0]:.3f},'
                        f'{self._start_odom_xy[1]:.3f}) '
                        f'imu_offset={math.degrees(offset):+.1f}deg'
                    )
            self.log.config(f'START_CORNER_DIAG state={state} '
                            f'reason={payload.get("reason", "log-only")}')
        except (TypeError, json.JSONDecodeError):
            self.log.warn('START_CORNER_DIAG', 'invalid diagnostic payload; ignored')
        except (KeyError, TypeError, ValueError):
            self.log.warn('START_CORNER_DIAG', 'valid payload missing startup pose fields; ignored')

    def _qr_cb(self, msg):
        task = msg.data.strip()
        if (not task or self._qr_latched or self._released or
                self._mission_state != self.MISSION_SEARCH_QR):
            return
        with self._lock:
            pose_xy = self._last_pose or self._lookup_map_pose_xy()
            if pose_xy is None or self.current_yaw is None:
                self.log.warn('QR', 'QR result received before a valid realtime pose; ignored')
                return
            self._qr_latched = True
            self._qr_task = task
            self._qr_pose_xy = tuple(pose_xy)
            self._qr_pose_yaw = float(self.current_yaw)
            self._qr_pose_odom_xy = None if self._odom_xy is None else tuple(self._odom_xy)
            self._qr_locked_at = self._now()
            self.task_pub.publish(String(data=task))
            self._publish_mission_event_locked(self.MISSION_QR_LOCKED)
            self._start_entry_route(pose_xy)
            self.log.task(
                f'S1 QR locked direction={task}; '
                f'qr_pose=({pose_xy[0]:.3f},{pose_xy[1]:.3f}) '
                f'yaw={math.degrees(self.current_yaw):.1f}deg; '
                f'replanning from scan pose to entry '
                f'({self.entry_goal[0]:.2f},{self.entry_goal[1]:.2f}) '
                f'target_yaw={math.degrees(self.entry_yaw):.1f}deg'
            )

    def _activate_cb(self, _request, response):
        if self._released:
            response.success = False
            response.message = 'stage1 already released'
            return response
        self._activation_requested = True
        response.success = True
        response.message = 'stage1 activation armed; waiting for prewarmed safe command'
        self.log.task('S1 activate received; arming until global route and local safe command are ready')
        return response

    def _release_cb(self, _request, response):
        if self._released:
            response.success = True
            response.message = 'stage1 already released'
            return response
        self._released = True
        self._motion_enabled = False
        self._plan_generation += 1
        self._plan_in_progress = False
        self._handoff_wait = False
        self._publish_zero()
        self._mission_state = 'complete'
        self.state_pub.publish(String(data='complete'))
        response.success = True
        response.message = 'stage1 released; process will exit'
        self.create_timer(0.15, self._shutdown_after_release)
        self.log.task('S1 released after S2 continuous handoff')
        return response

    def _shutdown_after_release(self):
        if rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # Localization and map model
    # ------------------------------------------------------------------
    def _publish_mission_event_locked(self, state):
        self._mission_state = state
        self.state_pub.publish(String(data=state))

    def _enter_search_qr_locked(self):
        if self._mission_state == self.MISSION_SEARCH_QR:
            return
        self._publish_mission_event_locked(self.MISSION_SEARCH_QR)
        self._publish_mission_route_locked()
        self._target_name = 'qr_search'
        self._target_xy = self.qr_goal
        self._target_yaw = None
        self.log.task(
            f'S1 mission SEARCH_QR: route reference=({self.qr_goal[0]:.2f},'
            f'{self.qr_goal[1]:.2f}), radius={self.qr_search_radius:.2f}m; '
            'QR event, not exact coordinate, completes this phase'
        )

    def _invalidate_route_locked(self, pose_xy=None):
        """Cancel stale planning work and install an optional safe connector."""
        self._route_target_signature = None
        self._route_locked = False
        self._local_failure_since = None
        self._plan_generation += 1
        self._plan_in_progress = False
        self._local_plan_generation += 1
        self._local_result = None
        self._local_result_time = 0.0
        # A QR result changes the mission segment immediately.  Do not let a
        # command produced for the old segment survive while the new local
        # sampler is still warming up.
        if self._command_is_motion(self._last_cmd) or self._heading_motion_active:
            self._publish_zero()
        else:
            self._last_cmd = Twist()
            self._last_cmd_time = self._now()
        self._last_motion_cmd = Twist()
        self._last_motion_cmd_time = 0.0
        self._last_safe_result_time = 0.0
        # Route changes now expose only the next validated global route.
        # Installing a temporary connector here made the web route and the
        # actual planner disagree during every QR handoff.
        self._path = []
        self._path_headings = []
        self._path_gears = []
        self._path_progress_index = 0
        self._publish_route_locked()

    def _build_connector_path(self, start_xy, goal_xy):
        """Build a short, statically validated bridge while A* replans.

        This is deliberately only a straight local bridge.  If any sampled
        footprint touches a static occupied cell, no bridge is returned and the
        controller waits for the full global planner; it never drives through
        an unvalidated wall just to avoid a pause.
        """
        dx = float(goal_xy[0]) - float(start_xy[0])
        dy = float(goal_xy[1]) - float(start_xy[1])
        distance = math.hypot(dx, dy)
        if distance < 0.04:
            return [], [], []
        heading = math.atan2(dy, dx)
        count = max(1, int(math.ceil(distance / 0.10)))
        path = [(float(start_xy[0]), float(start_xy[1]))]
        for index in range(1, count + 1):
            ratio = index / count
            point = (float(start_xy[0]) + ratio * dx,
                     float(start_xy[1]) + ratio * dy)
            if self._is_blocked_world(point[0], point[1], heading):
                return [], [], []
            path.append(point)
        return path, [heading] * len(path), [1] * len(path)

    def _publish_map_heading_locked(self):
        if self.current_yaw is not None:
            self.map_heading_pub.publish(Float64(data=float(self.current_yaw)))

    def _publish_map_pose_locked(self):
        if self._map_pose_xy is None or self.current_yaw is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = self._map_pose_xy
        msg.pose.orientation.z = math.sin(self.current_yaw * 0.5)
        msg.pose.orientation.w = math.cos(self.current_yaw * 0.5)
        self.map_pose_pub.publish(msg)

    def _publish_route_locked(self):
        """Publish the currently validated planner route for diagnostics/UI."""
        message = Path()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        for index, (x, y) in enumerate(self._path):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            heading = (
                self._path_headings[index]
                if index < len(self._path_headings) else self.current_yaw
            )
            if heading is not None:
                pose.pose.orientation.z = math.sin(float(heading) * 0.5)
                pose.pose.orientation.w = math.cos(float(heading) * 0.5)
            message.poses.append(pose)
        self.route_pub.publish(message)

    def _publish_mission_route_locked(self):
        """Publish logical task targets separately from the active safe route."""
        message = Path()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        points = []
        if self._start_map_xy is not None:
            points.append(self._start_map_xy)
        points.append(self.qr_goal)
        points.append(self.entry_goal)
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            if index + 1 < len(points):
                heading = math.atan2(
                    points[index + 1][1] - point[1],
                    points[index + 1][0] - point[0],
                )
            else:
                heading = self.entry_yaw
            pose.pose.orientation.z = math.sin(heading * 0.5)
            pose.pose.orientation.w = math.cos(heading * 0.5)
            message.poses.append(pose)
        self.mission_route_pub.publish(message)

    def _command_is_motion(self, command):
        return (
            abs(float(command.linear.x)) >= self.heading_motion_linear_threshold
            or abs(float(command.angular.z)) >= self.heading_motion_angular_threshold
        )

    def _update_heading_motion_state(self, command):
        """Gate IMU integration on actual commanded motion.

        The radar heading is the absolute map anchor.  IMU deltas are used
        only while a non-zero command is really being sent.  When the command
        becomes zero, preserve the last corrected heading and reset the raw
        IMU baseline before the next movement segment.
        """
        moving = self._command_is_motion(command)
        if moving and not self._heading_motion_active:
            self._heading_anchor_yaw = (
                self.current_yaw if self.current_yaw is not None else self._start_map_yaw
            )
            if self._heading_anchor_yaw is None or self._last_imu_stamp is None:
                return
            self._gyro_anchor_relative_yaw = self._gyro_relative_yaw
            self._heading_motion_active = True
            self.log.config(
                f'IMU dynamic heading enabled from radar anchor '
                f'{math.degrees(self._heading_anchor_yaw):.1f}deg'
            )
        elif not moving and self._heading_motion_active:
            if self.current_yaw is not None:
                self._heading_anchor_yaw = self.current_yaw
            if self._current_raw_yaw is not None:
                self._initial_raw_yaw = self._current_raw_yaw
            self._gyro_anchor_relative_yaw = self._gyro_relative_yaw
            self._heading_motion_active = False
            self.current_yaw = self._heading_anchor_yaw
            self._publish_map_heading_locked()

    def _lookup_map_pose_xy(self):
        return self._map_pose_xy

    def _localization_status(self, now):
        ages = (self._last_scan_stamp, self._last_imu_stamp, self._last_odom_stamp)
        names = ('scan', 'imu', 'odom')
        stale = []
        for name, stamp in zip(names, ages):
            if stamp is None:
                stale.append(f'{name}=missing')
            elif now - stamp > self.localization_max_age:
                stale.append(f'{name}_age={now - stamp:.3f}s')
        if stale:
            return False, ','.join(stale)
        if self._odom_fault_reason is not None:
            return False, f'odom_invalid={self._odom_fault_reason}'
        if self._lidar_position_fault_reason is not None:
            return False, f'lidar_map_mismatch={self._lidar_position_fault_reason}'
        if self._map is None or self._map_blocked is None or self._last_map_time is None:
            return False, 'map=missing'
        # /map is transient-local static data and normally arrives once.  Do
        # not treat its age as sensor staleness; only a new map callback
        # replaces the cached static layer.
        if self._lookup_map_pose_xy() is None:
            return False, 'odom_anchor=missing'
        if self.current_yaw is None:
            return False, 'heading=missing'
        return True, 'ok'

    def _localization_ok(self, now):
        return self._localization_status(now)[0]

    def _build_inflated_map(self, msg):
        width, height = int(msg.info.width), int(msg.info.height)
        radius = int(math.ceil(self.robot_radius / max(msg.info.resolution, 1e-6)))
        source = list(msg.data)
        blocked = [False] * (width * height)
        frontier = deque()
        for gy in range(height):
            for gx in range(width):
                value = source[gy * width + gx]
                if value < 0:
                    occupied = self.unknown_occupied
                else:
                    occupied = value >= self.occupied_threshold
                if not occupied:
                    continue
                index = gy * width + gx
                blocked[index] = True
                frontier.append((gx, gy, 0))

        # Multi-source breadth-first inflation is O(width*height), unlike the
        # old occupied-cell x disk-area loop.  The 8-connected frontier is a
        # conservative footprint envelope and keeps the callback short enough
        # for the S1 control timer to continue running on the same executor.
        neighbours = ((-1, -1), (-1, 0), (-1, 1), (0, -1),
                      (0, 1), (1, -1), (1, 0), (1, 1))
        while frontier:
            gx, gy, distance = frontier.popleft()
            if distance >= radius:
                continue
            next_distance = distance + 1
            for dx, dy in neighbours:
                nx, ny = gx + dx, gy + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                index = ny * width + nx
                if blocked[index]:
                    continue
                blocked[index] = True
                frontier.append((nx, ny, next_distance))
        return blocked

    def _build_lidar_distance_map(self, msg):
        """Distance to occupied edges; filled restricted areas are not fake walls."""
        data = np.asarray(msg.data, dtype=np.int16).reshape(
            (int(msg.info.height), int(msg.info.width)))
        occupied = data >= self.occupied_threshold
        if self.unknown_occupied:
            occupied |= data < 0
        occupied_image = occupied.astype(np.uint8)
        eroded = cv2.erode(occupied_image, np.ones((3, 3), dtype=np.uint8))
        edges = occupied & (eroded == 0)
        free_image = np.where(edges, 0, 255).astype(np.uint8)
        return cv2.distanceTransform(free_image, cv2.DIST_L2, 5).astype(np.float32) * float(
            msg.info.resolution)

    def _world_to_grid(self, x, y, step=None):
        if self._map is None:
            return None
        info = self._map.info
        resolution = float(info.resolution)
        step = resolution if step is None else float(step)
        return (int(math.floor((x - info.origin.position.x) / step)),
                int(math.floor((y - info.origin.position.y) / step)))

    def _grid_to_world(self, gx, gy, step=None):
        info = self._map.info
        resolution = float(info.resolution)
        step = resolution if step is None else float(step)
        return (info.origin.position.x + (gx + 0.5) * step,
                info.origin.position.y + (gy + 0.5) * step)

    def _lookup_base_link_offset(self):
        """Return the physical body origin in the laser/base_footprint frame."""
        if self._base_link_tf_reported:
            return self._base_link_offset
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, 'base_link', Time(), timeout=Duration(seconds=0.03))
            t = transform.transform.translation
            yaw = self._yaw_from_quaternion(transform.transform.rotation)
            self._base_link_offset = (float(t.x), float(t.y), yaw)
            if not self._base_link_tf_reported:
                self._base_link_tf_reported = True
                self.log.config(
                    f'Robot footprint TF {self.base_frame}->base_link: '
                    f'offset=({t.x:.3f},{t.y:.3f}) yaw={math.degrees(yaw):+.1f}deg; '
                    f'body=({self.body_length:.3f}x{self.body_width:.3f})')
        except TransformException:
            # A missing static body TF must not turn the local layer into an
            # unbounded circular obstacle.  The laser frame is still checked
            # independently; the conservative zero-offset fallback is only
            # used until the robot model TF arrives.
            pass
        return self._base_link_offset

    def _body_corners_base(self, extra=0.0):
        """Physical rectangular footprint expressed in base_footprint axes."""
        ox, oy, body_yaw = self._lookup_base_link_offset()
        half_length = 0.5 * self.body_length + self.footprint_margin + extra
        half_width = 0.5 * self.body_width + self.footprint_margin + extra
        local = ((-half_length, -half_width), (-half_length, half_width),
                 (half_length, half_width), (half_length, -half_width))
        return tuple((ox + math.cos(body_yaw) * x - math.sin(body_yaw) * y,
                      oy + math.sin(body_yaw) * x + math.cos(body_yaw) * y)
                     for x, y in local)

    def _footprint_samples_base(self):
        """Return a cached perimeter sample set for the rectangular body.

        The global planner evaluates thousands of poses.  Rebuilding the TF
        geometry and trigonometric values for every pose made the Python
        worker spend seconds in collision checks and starved the ROS executor.
        The static base_link transform and body dimensions only change when a
        robot model is reloaded, so cache the same conservative corner/edge
        samples used by the previous raw-map check.
        """
        ox, oy, body_yaw = self._lookup_base_link_offset()
        signature = (
            ox, oy, body_yaw, self.body_length, self.body_width,
            self.footprint_margin,
        )
        if signature == self._footprint_samples_signature and self._footprint_samples_cache:
            return self._footprint_samples_cache
        half_length = 0.5 * self.body_length + self.footprint_margin
        half_width = 0.5 * self.body_width + self.footprint_margin
        local = ((-half_length, -half_width), (-half_length, half_width),
                 (half_length, half_width), (half_length, -half_width))
        corners = tuple(
            (ox + math.cos(body_yaw) * x - math.sin(body_yaw) * y,
             oy + math.sin(body_yaw) * x + math.cos(body_yaw) * y)
            for x, y in local
        )
        samples = list(corners)
        for index in range(4):
            ax, ay = corners[index]
            bx, by = corners[(index + 1) % 4]
            for fraction in (0.25, 0.50, 0.75):
                samples.append((ax + fraction * (bx - ax),
                                ay + fraction * (by - ay)))
        self._footprint_samples_signature = signature
        self._footprint_samples_cache = tuple(samples)
        return self._footprint_samples_cache

    def _point_in_body_base(self, base_x, base_y, extra=0.0):
        ox, oy, body_yaw = self._lookup_base_link_offset()
        dx, dy = base_x - ox, base_y - oy
        local_x = math.cos(body_yaw) * dx + math.sin(body_yaw) * dy
        local_y = -math.sin(body_yaw) * dx + math.cos(body_yaw) * dy
        half_length = 0.5 * self.body_length + self.footprint_margin + extra
        half_width = 0.5 * self.body_width + self.footprint_margin + extra
        return abs(local_x) <= half_length and abs(local_y) <= half_width

    def _body_clearance_base(self, base_x, base_y):
        """Signed distance from a base-frame point to the actual body box."""
        ox, oy, body_yaw = self._lookup_base_link_offset()
        dx, dy = base_x - ox, base_y - oy
        local_x = math.cos(body_yaw) * dx + math.sin(body_yaw) * dy
        local_y = -math.sin(body_yaw) * dx + math.cos(body_yaw) * dy
        half_length = 0.5 * self.body_length + self.footprint_margin
        half_width = 0.5 * self.body_width + self.footprint_margin
        outside_x = max(abs(local_x) - half_length, 0.0)
        outside_y = max(abs(local_y) - half_width, 0.0)
        outside = math.hypot(outside_x, outside_y)
        if abs(local_x) <= half_length and abs(local_y) <= half_width:
            return -min(half_length - abs(local_x), half_width - abs(local_y))
        return outside

    def _raw_occupied_world(self, x, y):
        """Check the uninflated map; out-of-map is a hard static boundary."""
        if self._map is None:
            return True
        info = self._map.info
        gx = int(math.floor((x - info.origin.position.x) / info.resolution))
        gy = int(math.floor((y - info.origin.position.y) / info.resolution))
        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            return True
        value = self._map.data[gy * info.width + gx]
        if value < 0:
            return self.unknown_occupied
        return value >= self.occupied_threshold

    def _footprint_collision_world(self, x, y, yaw):
        """Collision test for the real rectangular body, not a laser-centered disk."""
        if self._map is None or self._map_blocked is None:
            return True
        info = self._map.info
        samples = self._footprint_samples_base()
        cy, sy = math.cos(yaw), math.sin(yaw)
        for base_x, base_y in samples:
            map_x = x + cy * base_x - sy * base_y
            map_y = y + sy * base_x + cy * base_y
            gx = int(math.floor((map_x - info.origin.position.x) / info.resolution))
            gy = int(math.floor((map_y - info.origin.position.y) / info.resolution))
            if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
                return True
            # The full footprint is already sampled here.  Use the raw map
            # rather than the radius-inflated map: applying both would add
            # the robot radius twice and close the real corridor entrance.
            value = self._map.data[gy * info.width + gx]
            if value < 0:
                return self.unknown_occupied
            if value >= self.occupied_threshold:
                return True
        return False

    def _is_blocked_world(self, x, y, yaw=None):
        if self._map is None or self._map_blocked is None:
            return True
        if yaw is not None:
            return self._footprint_collision_world(x, y, yaw)
        info = self._map.info
        gx = int(math.floor((x - info.origin.position.x) / info.resolution))
        gy = int(math.floor((y - info.origin.position.y) / info.resolution))
        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            return True
        return bool(self._map_blocked[gy * info.width + gx])

    def _is_static_occupied_world(self, x, y):
        """Return the uninflated static occupancy at a world point.

        Scan returns that land on a known map wall are already represented by
        the inflated static layer.  They must not be inserted a second time as
        dynamic obstacles, otherwise a robot travelling close to a wall can
        make every local trajectory look like a collision.
        """
        return self._static_cell_kind_world(x, y) != 'free'

    def _static_cell_kind_world(self, x, y):
        """Classify a map sample for the dynamic hard-safety layer."""
        if self._map is None:
            return 'outside'
        info = self._map.info
        gx = int(math.floor((x - info.origin.position.x) / info.resolution))
        gy = int(math.floor((y - info.origin.position.y) / info.resolution))
        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            return 'outside'
        value = self._map.data[gy * info.width + gx]
        if value < 0:
            return 'unknown'
        return 'known_occupied' if value >= self.occupied_threshold else 'free'

    def _near_static_map_boundary(self, x, y):
        """Treat laser returns on the known map rectangle as static walls."""
        if self._map is None:
            return False
        info = self._map.info
        min_x = float(info.origin.position.x)
        min_y = float(info.origin.position.y)
        max_x = min_x + float(info.width) * float(info.resolution)
        max_y = min_y + float(info.height) * float(info.resolution)
        outside_dx = max(min_x - x, 0.0, x - max_x)
        outside_dy = max(min_y - y, 0.0, y - max_y)
        if outside_dx > 0.0 or outside_dy > 0.0:
            return math.hypot(outside_dx, outside_dy) <= self.scan_static_match_tolerance
        return min(x - min_x, y - min_y, max_x - x, max_y - y) <= self.scan_static_match_tolerance

    def _scan_points_base(self, scan=None):
        """Convert the current LaserScan from its frame into base coordinates.

        The production TF chain places ``laser`` at ``base_link``, while
        ``base_link`` is offset from ``base_footprint``.  Treating raw scan
        angles as if the laser were at the footprint origin shifts every
        obstacle by that offset and can reject every local trajectory.
        """
        scan = self._scan if scan is None else scan
        if scan is None:
            return None
        source_frame = (scan.header.frame_id or 'laser').lstrip('/')
        now = self._now()
        if (self._scan_transform_cache is None or self._scan_transform_frame != source_frame or
                now - self._scan_transform_time > 1.0):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, source_frame, Time(), timeout=Duration(seconds=0.03))
            except TransformException:
                return None
            rotation = transform.transform.rotation
            self._scan_transform_cache = (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                self._yaw_from_quaternion(rotation))
            self._scan_transform_frame = source_frame
            self._scan_transform_time = now
        tx, ty, transform_yaw = self._scan_transform_cache
        points = []
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < max(0.03, scan.range_min):
                continue
            if distance > self.dynamic_max_range:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            laser_x = distance * math.cos(angle)
            laser_y = distance * math.sin(angle)
            base_x = math.cos(transform_yaw) * laser_x - math.sin(transform_yaw) * laser_y + tx
            base_y = math.sin(transform_yaw) * laser_x + math.cos(transform_yaw) * laser_y + ty
            points.append((base_x, base_y, distance, laser_x))
        if not self._scan_tf_reported:
            self._scan_tf_reported = True
            self.log.config(
                f'LaserScan TF {source_frame}->{self.base_frame}: '
                f'offset=({tx:.3f},{ty:.3f}) yaw={math.degrees(transform_yaw):+.1f}deg; '
                f'points={len(points)}'
            )
        return points

    # ------------------------------------------------------------------
    # Heading-aware global route search
    # ------------------------------------------------------------------
    def _plan_fast_corridor(self, start_xy, goal_xy, goal_tolerance):
        """Return a fast footprint-checked corridor while Hybrid-A* refines.

        This planner intentionally has no heading state.  Its job is limited
        to avoiding a zero-command gap when the QR callback changes the route:
        it gives the local controller a statically validated first corridor in
        a few milliseconds.  The regular heading-aware planner is still
        launched immediately afterwards and replaces this temporary route
        before a tight corner needs to be negotiated.
        """
        if self._map is None:
            return [], [], []
        step = max(self.fast_grid_step, float(self._map.info.resolution))
        start = self._world_to_grid(start_xy[0], start_xy[1], step)
        goal = self._world_to_grid(goal_xy[0], goal_xy[1], step)
        if start is None or goal is None:
            return [], [], []
        max_x = int(math.ceil(self._map.info.width * self._map.info.resolution / step))
        max_y = int(math.ceil(self._map.info.height * self._map.info.resolution / step))
        if not (0 <= start[0] < max_x and 0 <= start[1] < max_y and
                0 <= goal[0] < max_x and 0 <= goal[1] < max_y):
            return [], [], []
        open_heap = [(0.0, 0.0, start)]
        g_score = {start: 0.0}
        parent = {}
        directions = ((-1, -1), (-1, 0), (-1, 1), (0, -1),
                      (0, 1), (1, -1), (1, 0), (1, 1))
        goal_state = None
        while open_heap:
            _, cost, state = heapq.heappop(open_heap)
            if cost > g_score.get(state, float('inf')) + 1e-9:
                continue
            wx, wy = self._grid_to_world(state[0], state[1], step)
            if math.hypot(wx - goal_xy[0], wy - goal_xy[1]) <= goal_tolerance:
                goal_state = state
                break
            for dx, dy in directions:
                nx, ny = state[0] + dx, state[1] + dy
                if nx < 0 or ny < 0 or nx >= max_x or ny >= max_y:
                    continue
                next_xy = self._grid_to_world(nx, ny, step)
                heading = math.atan2(dy, dx)
                # Checking both the successor and segment midpoint prevents
                # the temporary grid corridor from cutting a wall corner.
                midpoint = ((wx + next_xy[0]) * 0.5, (wy + next_xy[1]) * 0.5)
                if (self._is_blocked_world(next_xy[0], next_xy[1], heading) or
                        self._is_blocked_world(midpoint[0], midpoint[1], heading)):
                    continue
                next_state = (nx, ny)
                step_cost = math.hypot(dx, dy)
                tentative = cost + step_cost
                if tentative >= g_score.get(next_state, float('inf')):
                    continue
                g_score[next_state] = tentative
                parent[next_state] = state
                heuristic = math.hypot(nx - goal[0], ny - goal[1])
                heapq.heappush(open_heap, (tentative + heuristic, tentative, next_state))
        if goal_state is None:
            return [], [], []
        states = []
        cursor = goal_state
        while cursor != start:
            states.append(cursor)
            cursor = parent[cursor]
        states.append(start)
        states.reverse()
        path = [self._grid_to_world(gx, gy, step) for gx, gy in states]
        if path:
            path[0] = (float(start_xy[0]), float(start_xy[1]))
            # Only expose the first straight run.  The remainder of a grid
            # route can contain 90-degree corners that violate the chassis
            # turn radius; Hybrid-A* will replace this prefix asynchronously.
            prefix = [path[0]]
            first_heading = None
            travelled = 0.0
            for index in range(len(path) - 1):
                ax, ay = path[index]
                bx, by = path[index + 1]
                segment_heading = math.atan2(by - ay, bx - ax)
                if first_heading is None:
                    first_heading = segment_heading
                elif abs(self._norm(segment_heading - first_heading)) > math.radians(35.0):
                    break
                segment_length = math.hypot(bx - ax, by - ay)
                prefix.append(path[index + 1])
                travelled += segment_length
                if travelled >= self.fast_corridor_length:
                    break
            path = prefix
            if len(path) < 2:
                return [], [], []
            if self.current_yaw is not None:
                # Replace the grid prefix with a short, real heading-aligned
                # bridge.  A grid route can look straight on the map while
                # its first tangent ignores the robot's 18-degree placement.
                aligned = [path[0]]
                bridge_length = min(
                    self.fast_corridor_length,
                    math.hypot(float(goal_xy[0]) - float(start_xy[0]),
                               float(goal_xy[1]) - float(start_xy[1])),
                )
                samples = max(1, int(math.ceil(bridge_length / step)))
                for sample_index in range(1, samples + 1):
                    travelled = bridge_length * sample_index / samples
                    aligned.append((
                        float(start_xy[0] + travelled * math.cos(self.current_yaw)),
                        float(start_xy[1] + travelled * math.sin(self.current_yaw)),
                    ))
                if all(not self._is_blocked_world(x, y, self.current_yaw)
                       for x, y in aligned[1:]):
                    path = aligned
            for index in range(len(path) - 1):
                ax, ay = path[index]
                bx, by = path[index + 1]
                segment_heading = math.atan2(by - ay, bx - ax)
                samples = max(1, int(math.ceil(math.hypot(bx - ax, by - ay) / 0.05)))
                for sample_index in range(1, samples + 1):
                    ratio = sample_index / samples
                    sx = ax + ratio * (bx - ax)
                    sy = ay + ratio * (by - ay)
                    if self._is_blocked_world(sx, sy, segment_heading):
                        return [], [], []
        headings = []
        for index in range(len(path)):
            if index == 0 and self.current_yaw is not None:
                # The temporary bridge must inherit the live chassis heading;
                # otherwise prewarming installs an immediate 0-degree turn.
                headings.append(self.current_yaw)
                continue
            if index + 1 < len(path):
                a, b = path[index], path[index + 1]
            elif index > 0:
                a, b = path[index - 1], path[index]
            else:
                headings.append(self.current_yaw)
                continue
            headings.append(math.atan2(b[1] - a[1], b[0] - a[0]))
        if headings and self.current_yaw is not None:
            # The lookahead can skip the first short segment.  Keep the whole
            # temporary bridge aligned with the live heading so prewarming
            # cannot silently revert to the grid segment's 0-degree tangent.
            headings = [self.current_yaw] * len(headings)
        return path, headings, [1] * len(path)

    def _plan_global(self, start_xy, start_yaw, goal_xy, goal_yaw, goal_tolerance=None):
        if self._map is None or self._map_blocked is None:
            return [], [], []
        step = max(self.grid_step, float(self._map.info.resolution))
        start = self._world_to_grid(start_xy[0], start_xy[1], step)
        goal = self._world_to_grid(goal_xy[0], goal_xy[1], step)
        if start is None or goal is None:
            return [], [], []
        start_bin = int(round(self._norm(start_yaw) / (2.0 * math.pi) * self.heading_bins)) % self.heading_bins
        goal_bin = None if goal_yaw is None else int(round(self._norm(goal_yaw) / (2.0 * math.pi) * self.heading_bins)) % self.heading_bins
        if goal_tolerance is None:
            goal_tolerance = self.entry_tolerance if goal_yaw is not None else self.qr_search_radius
        # Keep the previous steering primitive in the search state.  Without
        # it, equal-cost grid successors can alternate left/straight/right
        # on every expansion, producing a route whose tangent flips while the
        # vehicle is travelling on an otherwise straight section.
        start_state = (start[0], start[1], start_bin, 1, 0)
        open_heap = [(0.0, 0.0, start_state)]
        g_score = {start_state: 0.0}
        parent = {}
        expansions = 0
        goal_state = None
        primitives = (-1, 0, 1)
        while open_heap and expansions < self.max_expansions:
            _, cost, state = heapq.heappop(open_heap)
            if cost > g_score.get(state, float('inf')) + 1e-9:
                continue
            expansions += 1
            if expansions % 64 == 0:
                time.sleep(0)
            gx, gy, hbin, gear, previous_steer = state
            wx, wy = self._grid_to_world(gx, gy, step)
            yaw = (hbin / self.heading_bins) * 2.0 * math.pi
            if math.hypot(wx - goal_xy[0], wy - goal_xy[1]) <= goal_tolerance:
                if goal_bin is None or abs((hbin - goal_bin + self.heading_bins // 2) % self.heading_bins - self.heading_bins // 2) <= max(1, int(self.goal_yaw_tolerance * self.heading_bins / (2.0 * math.pi))):
                    goal_state = state
                    break
            for steer in primitives:
                curvature = steer / self.min_turn_radius
                nx = wx + self.motion_step * math.cos(yaw)
                ny = wy + self.motion_step * math.sin(yaw)
                nyaw = self._norm(yaw + curvature * self.motion_step)
                ns_grid = self._world_to_grid(nx, ny, step)
                if ns_grid is None:
                    continue
                ngx, ngy = ns_grid
                if self._is_blocked_world(nx, ny, nyaw):
                    continue
                nhbin = int(round(nyaw / (2.0 * math.pi) * self.heading_bins)) % self.heading_bins
                nstate = (ngx, ngy, nhbin, 1, steer)
                step_cost = self.motion_step
                step_cost += self.turn_penalty * abs(steer)
                step_cost += self.planner_steer_change_penalty * abs(steer - previous_steer)
                tentative = cost + step_cost
                if tentative >= g_score.get(nstate, float('inf')):
                    continue
                g_score[nstate] = tentative
                parent[nstate] = state
                heuristic = math.hypot(nx - goal_xy[0], ny - goal_xy[1])
                heapq.heappush(open_heap, (tentative + heuristic, tentative, nstate))
        if goal_state is None:
            self.log.warn('PLAN', f'global search failed expansions={expansions} '
                          f'start=({start_xy[0]:.2f},{start_xy[1]:.2f}) '
                          f'goal=({goal_xy[0]:.2f},{goal_xy[1]:.2f})')
            return [], [], []
        states = []
        cursor = goal_state
        while cursor != start_state:
            states.append(cursor)
            cursor = parent[cursor]
        states.append(start_state)
        states.reverse()
        path = [self._grid_to_world(s[0], s[1], step) for s in states]
        if path:
            path[0] = (float(start_xy[0]), float(start_xy[1]))
            if len(path) > 1:
                first_distance = math.hypot(path[1][0] - path[0][0],
                                            path[1][1] - path[0][1])
                bridge_distance = min(self.motion_step, max(first_distance, 0.05))
                aligned = (
                    path[0][0] + bridge_distance * math.cos(start_yaw),
                    path[0][1] + bridge_distance * math.sin(start_yaw),
                )
                if not self._is_blocked_world(aligned[0], aligned[1], start_yaw):
                    path.insert(1, aligned)
        gears = [s[3] for s in states]
        headings = [self._norm(start_yaw)]
        for index in range(1, len(path)):
            previous = path[index - 1]
            current = path[index]
            headings.append(math.atan2(current[1] - previous[1],
                                       current[0] - previous[0]))
        # The exact goal point is appended only after the search state.  Keep
        # the final tangent/gear explicit so the local controller never
        # invents a heading from the robot pose-to-goal vector.
        if path:
            path.append((float(goal_xy[0]), float(goal_xy[1])))
            previous, final = path[-2], path[-1]
            headings.append(math.atan2(final[1] - previous[1], final[0] - previous[0]))
            gears.append(gears[-1])
        self.log.plan(f'global hybrid search success points={len(path)} expansions={expansions} '
                      f'goal={self._target_name} '
                      f'route_start_yaw={math.degrees(headings[0]):+.1f}deg '
                      f'route_end_yaw={math.degrees(headings[-1]):+.1f}deg '
                      f'route_first=({path[0][0]:.2f},{path[0][1]:.2f})->'
                      f'({path[1][0]:.2f},{path[1][1]:.2f})')
        return path, headings, gears

    def _nearest_path(self, xy, start_index=0):
        if not self._path:
            return None
        start_index = max(0, min(int(start_index), len(self._path) - 1))
        best = min(enumerate(self._path[start_index:], start=start_index),
                   key=lambda item: math.hypot(item[1][0] - xy[0], item[1][1] - xy[1]))
        return best[0], best[1], math.hypot(best[1][0] - xy[0], best[1][1] - xy[1])

    def _path_target(self, xy):
        # Reconnect to the route monotonically.  A global replan can move the
        # first few samples behind the vehicle; allowing the nearest-point
        # search to jump backwards is what caused the local sampler to select
        # long reverse arcs and oscillate around x=3.2m.
        nearest = self._nearest_path(xy, max(0, self._path_progress_index - 2))
        if nearest is None:
            return None
        index = nearest[0]
        self._path_progress_index = max(self._path_progress_index, index)
        remaining = self.lookahead
        for i in range(index, len(self._path) - 1):
            a, b = self._path[i], self._path[i + 1]
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            if length >= remaining:
                ratio = remaining / max(length, 1e-6)
                tangent = (self._path_headings[i] if i < len(self._path_headings)
                           else math.atan2(b[1] - a[1], b[0] - a[0]))
                return (a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1]), tangent)
            remaining -= length
        if len(self._path) >= 2:
            a, b = self._path[-2], self._path[-1]
            tangent = self._path_headings[-1] if self._path_headings else math.atan2(b[1] - a[1], b[0] - a[0])
            return b[0], b[1], tangent
        return self._path[-1][0], self._path[-1][1], self.current_yaw

    # ------------------------------------------------------------------
    # MPPI-style local trajectory sampler and hard collision monitor
    # ------------------------------------------------------------------
    def _scan_points_map(self, pose_xy, yaw):
        base_points = self._scan_points_base()
        if base_points is None:
            self._last_scan_diagnostic = {'raw': 0, 'self': 0, 'static': 0, 'dynamic': 0,
                                          'nearest_dynamic_m': None}
            return []
        scan_pose = self._scan_pose(self._scan)
        if scan_pose is None:
            self._last_scan_diagnostic = {'raw': len(base_points), 'self': 0, 'static': 0,
                                          'dynamic': 0, 'nearest_dynamic_m': None}
            return []
        scan_xy, scan_yaw = scan_pose
        points = []
        self_filtered = 0
        static_filtered = 0
        for base_x, base_y, distance, laser_x in base_points:
            # A delayed scan must be projected with the historical pose at its
            # own timestamp. Applying the current pose shifts static walls into
            # free cells and turns them into fake dynamic obstacles.
            map_x = scan_xy[0] + math.cos(scan_yaw) * base_x - math.sin(scan_yaw) * base_y
            map_y = scan_xy[1] + math.sin(scan_yaw) * base_x + math.cos(scan_yaw) * base_y
            # Ignore returns inside the physical chassis.  Returns on map
            # walls or outside the known map are static boundary evidence;
            # only returns in known free space become dynamic obstacles.
            if self._point_in_body_base(base_x, base_y, self.scan_self_filter_margin):
                self_filtered += 1
                continue
            if self._near_static_map_boundary(map_x, map_y):
                # Keep the local sampler consistent with Collision Monitor:
                # the physical map rectangle is static structure, not a
                # temporary obstacle that invalidates every turn candidate.
                static_filtered += 1
                continue
            if self._is_static_occupied_world(map_x, map_y):
                static_filtered += 1
                continue
            points.append((map_x, map_y, distance, laser_x))
        full_dynamic_count = len(points)
        if full_dynamic_count > self.local_scan_max_points:
            stride = int(math.ceil(full_dynamic_count / self.local_scan_max_points))
            sampled_points = points[::stride][:self.local_scan_max_points]
        else:
            sampled_points = points
        self._last_scan_diagnostic = {
            'raw': len(base_points), 'self': self_filtered, 'static': static_filtered,
            'dynamic': full_dynamic_count, 'sampled': len(sampled_points),
            'nearest_dynamic_m': None if not points else min(item[2] for item in points),
        }
        return sampled_points

    def _trajectory(self, pose_xy, yaw, v, w):
        samples = []
        x, y, heading = pose_xy[0], pose_xy[1], yaw
        steps = max(1, int(math.ceil(self.horizon / self.local_dt)))
        for _ in range(steps):
            x += v * math.cos(heading) * self.local_dt
            y += v * math.sin(heading) * self.local_dt
            heading = self._norm(heading + w * self.local_dt)
            samples.append((x, y, heading))
        return samples

    def _trajectory_cost(self, pose_xy, yaw, v, w, scan_points):
        if abs(w) > abs(v) / self.local_min_turn_radius + 1e-6:
            return float('inf')
        trajectory = self._trajectory(pose_xy, yaw, v, w)
        total = 0.0
        min_clearance = float('inf')
        for x, y, heading in trajectory:
            if self._is_blocked_world(x, y, heading):
                return float('inf')
            nearest = self._nearest_path((x, y), max(0, self._path_progress_index - 2))
            if nearest is not None:
                total += self.path_weight * nearest[2] ** 2
                path_index = nearest[0]
                if path_index < len(self._path_headings):
                    path_heading = self._path_headings[path_index]
                    total += self.heading_weight * abs(self._norm(path_heading - heading))
            if self._target_xy is not None:
                total += self.goal_weight * math.hypot(x - self._target_xy[0], y - self._target_xy[1]) ** 2
            for ox, oy, _, _ in scan_points:
                dx, dy = ox - x, oy - y
                local_x = math.cos(heading) * dx + math.sin(heading) * dy
                local_y = -math.sin(heading) * dx + math.cos(heading) * dy
                # The scan point is tested against the oriented body box in
                # the candidate pose.  A radial disk around the laser frame
                # incorrectly treats a side wall as a frontal collision.
                distance = self._body_clearance_base(local_x, local_y)
                min_clearance = min(min_clearance, distance)
                if distance <= self.clearance:
                    return float('inf')
                total += self.clearance_weight / max(distance, 0.04)
        if self._target_xy is not None and trajectory:
            final_heading = trajectory[-1][2]
            target_heading = self._target_yaw
            if target_heading is None:
                target_heading = math.atan2(self._target_xy[1] - trajectory[-1][1],
                                            self._target_xy[0] - trajectory[-1][0])
            total += 0.5 * self.heading_weight * abs(self._norm(target_heading - final_heading))
        total += self.steer_change_penalty * abs(w - self._last_cmd.angular.z)
        if self._last_route_heading is not None:
            heading_error = self._norm(self._last_route_heading - yaw)
            reference_w = heading_error / max(0.70 * self.horizon, self.local_dt)
            max_reference_w = min(self.max_angular, abs(v) / self.local_min_turn_radius)
            reference_w = max(-max_reference_w, min(max_reference_w, reference_w))
            # Keep the sampled command aligned with the route tangent.  The
            # previous implementation rewarded whichever high-curvature
            # sample happened to reduce path distance, so a straight section
            # could alternate left/right after every replan.
            total += self.heading_control_weight * (w - reference_w) ** 2
            total += self.angular_effort_weight * w * w
        if min_clearance != float('inf'):
            total += self.obstacle_cost_weight * max(0.0, self.clearance + 0.08 - min_clearance) ** 2
        return total

    def _local_command(self, pose_xy):
        if not self._path or self._scan is None or self.current_yaw is None:
            return None
        target = self._path_target(pose_xy)
        if target is None:
            return None
        target_heading = target[2]
        self._last_route_heading = target_heading
        heading_error = self._norm(target_heading - self.current_yaw)
        speed = min(self.max_speed, max(self.min_speed, self.nominal_speed))
        if abs(heading_error) > math.radians(55.0):
            speed = max(self.min_speed, speed * 0.55)
        if self._target_xy is not None:
            distance = math.hypot(self._target_xy[0] - pose_xy[0], self._target_xy[1] - pose_xy[1])
            speed = min(
                speed,
                max(
                    self.min_speed,
                    min(self.goal_speed_cap,
                        self.goal_speed_floor + self.goal_speed_gain * min(1.0, distance)),
                ),
            )
        speeds = ([speed] if self.speed_samples == 1 else [
            speed * (0.55 + 0.45 * i / (self.speed_samples - 1))
            for i in range(self.speed_samples)
        ])
        forward_candidates = []
        for v in speeds:
            max_w = min(self.max_angular, abs(v) / self.local_min_turn_radius)
            for j in range(self.steer_samples):
                ratio = 0.0 if self.steer_samples == 1 else (2.0 * j / (self.steer_samples - 1) - 1.0)
                forward_candidates.append((v, ratio * max_w))
        scan_points = self._scan_points_map(pose_xy, self.current_yaw)
        scored = [(self._trajectory_cost(pose_xy, self.current_yaw, v, w, scan_points), v, w)
                  for v, w in forward_candidates]
        safe = [item for item in scored if math.isfinite(item[0])]
        if not safe:
            diag = self._last_scan_diagnostic or {}
            self._last_local_failure = (
                f"raw_scan={diag.get('raw', 0)} self_returns={diag.get('self', 0)} "
                f"static_returns={diag.get('static', 0)} dynamic_returns={diag.get('dynamic', 0)} "
                f"sampled={diag.get('sampled', 0)} "
                f"nearest_dynamic={diag.get('nearest_dynamic_m')} "
                f"body_offset=({self._base_link_offset[0]:.3f},{self._base_link_offset[1]:.3f})")
            return None
        safe.sort(key=lambda item: item[0])
        best = safe[:max(3, min(8, len(safe)))]
        weights = [math.exp(-min(50.0, item[0] - best[0][0])) for item in best]
        total_weight = sum(weights)
        v = sum(weight * item[1] for weight, item in zip(weights, best)) / total_weight
        w = sum(weight * item[2] for weight, item in zip(weights, best)) / total_weight
        self._last_local_gear = 'F'
        if abs(v) < self.min_speed:
            v = math.copysign(self.min_speed, best[0][1] if best[0][1] else 1.0)
        w = max(-abs(v) / self.local_min_turn_radius, min(abs(v) / self.local_min_turn_radius, w))
        return v, max(-self.max_angular, min(self.max_angular, w)), best[0][0]

    def _collision_monitor(self, cmd):
        if self._scan is None or abs(cmd.linear.x) < 1e-6:
            return cmd
        nearest = float('inf')
        static_known_ignored = 0
        static_side_ignored = 0
        static_boundary_ignored = 0
        pose_xy = self._last_pose
        scan_pose = self._scan_pose(self._scan)
        scan_xy, scan_yaw = scan_pose if scan_pose is not None else (pose_xy, self.current_yaw)
        scan_points = self._scan_points_base()
        if scan_points is None:
            self.log.warn('COLLISION_MONITOR', 'laser TF unavailable; hard stop')
            return Twist()
        for base_x, base_y, distance, _ in scan_points:
            if self._point_in_body_base(base_x, base_y, self.scan_self_filter_margin):
                continue
            static_kind = 'free'
            if scan_xy is not None and scan_yaw is not None:
                map_x = scan_xy[0] + math.cos(scan_yaw) * base_x - math.sin(scan_yaw) * base_y
                map_y = scan_xy[1] + math.sin(scan_yaw) * base_x + math.cos(scan_yaw) * base_y
                static_kind = self._static_cell_kind_world(map_x, map_y)
                if self._near_static_map_boundary(map_x, map_y):
                    static_boundary_ignored += 1
                    continue
            if static_kind == 'known_occupied':
                # Known static walls are already checked by the inflated-map
                # footprint validator.  Feeding them into TTC double-counts
                # the wall and stops a chassis that is intentionally beside it.
                static_known_ignored += 1
                continue
            if static_kind == 'free':
                pass  # a return in known free space is a dynamic obstacle
            elif static_kind in ('unknown', 'outside'):
                # Unknown/map-outside returns remain hard safety boundaries.
                static_side_ignored += 1
            current_x, current_y = base_x, base_y
            if (pose_xy is not None and self.current_yaw is not None and
                    scan_xy is not None and scan_yaw is not None):
                map_dx = map_x - pose_xy[0]
                map_dy = map_y - pose_xy[1]
                current_x = math.cos(self.current_yaw) * map_dx + math.sin(self.current_yaw) * map_dy
                current_y = -math.sin(self.current_yaw) * map_dx + math.cos(self.current_yaw) * map_dy
            norm = max(math.hypot(current_x, current_y), 1e-6)
            forward = (current_x / norm) * (1.0 if cmd.linear.x >= 0.0 else -1.0)
            if forward < math.cos(self.forward_half_angle):
                continue
            nearest = min(nearest, max(0.0, self._body_clearance_base(current_x, current_y)))
        effective = nearest
        ttc = effective / max(abs(cmd.linear.x), 1e-3)
        if effective <= self.stop_distance or ttc <= self.stop_ttc:
            self.log.warn('COLLISION_MONITOR', f'hard stop distance={effective:.2f}m ttc={ttc:.2f}s '
                          f'static_known_ignored={static_known_ignored} '
                          f'static_boundary_ignored={static_boundary_ignored} '
                          f'unknown_or_outside={static_side_ignored}')
            return Twist()
        if effective <= self.slow_distance or ttc <= self.slow_ttc:
            scale = self.slow_scale
            cmd.linear.x *= scale
            cmd.angular.z = max(-abs(cmd.linear.x) / self.local_min_turn_radius,
                                 min(abs(cmd.linear.x) / self.local_min_turn_radius, cmd.angular.z))
        return cmd

    # ------------------------------------------------------------------
    # Control state machine
    # ------------------------------------------------------------------
    def _publish_zero(self):
        zero = Twist()
        self.cmd_pub.publish(zero)
        self._last_cmd = zero
        self._last_cmd_time = self._now()
        self._update_heading_motion_state(zero)

    def _rate_limit_command(self, v, w):
        """Bound command jumps so replans do not become steering jolts."""
        previous_v = float(self._last_cmd.linear.x)
        previous_w = float(self._last_cmd.angular.z)
        now = self._now()
        dt = now - self._last_cmd_time if self._last_cmd_time > 0.0 else 1.0 / self.control_rate_hz
        dt = min(0.20, max(0.01, dt))
        max_dv = self.max_linear_accel * dt
        max_dw = self.max_angular_accel * dt
        if (self.angular_reversal_deadband > 0.0 and
                abs(previous_w) >= self.angular_reversal_deadband and
                abs(w) >= self.angular_reversal_deadband and
                previous_w * float(w) < 0.0):
            # Cross zero before changing steering direction.  This keeps a
            # delayed local-plan result from alternating left/right at full
            # curvature during a large turn.
            w = 0.0
        v = previous_v + max(-max_dv, min(max_dv, float(v) - previous_v))
        w = previous_w + max(-max_dw, min(max_dw, float(w) - previous_w))
        return v, w

    def _publish_cmd(self, v, w):
        requested = Twist()
        requested.linear.x = float(v)
        requested.angular.z = float(w)
        safe = self._collision_monitor(requested)
        hard_stopped = abs(requested.linear.x) > 1e-6 and abs(safe.linear.x) <= 1e-6
        if hard_stopped:
            v, w = 0.0, 0.0
        else:
            v, w = self._rate_limit_command(safe.linear.x, safe.angular.z)
            # Linear and angular rate limits act independently.  Reapply the
            # vehicle curvature bound after that step, otherwise a braking
            # ramp can leave a large turn rate on an almost-zero speed command.
            if abs(v) <= 1e-6:
                w = 0.0
            else:
                max_curvature_w = abs(v) / self.local_min_turn_radius
                w = max(-max_curvature_w, min(max_curvature_w, w))
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)
        self._last_cmd = msg
        self._last_cmd_time = self._now()
        if abs(msg.linear.x) >= self.heading_motion_linear_threshold:
            self._map_motion_direction = math.copysign(1.0, msg.linear.x)
        if self._command_is_motion(msg):
            self._last_motion_cmd = msg
            self._last_motion_cmd_time = self._last_cmd_time
        self._update_heading_motion_state(msg)
        return msg

    def _ensure_target(self):
        if self._target_xy is not None:
            return
        self._target_name = 'qr_search'
        self._target_xy = self.qr_goal
        self._target_yaw = None

    def _route_signature(self):
        """Return the immutable task-segment identity for the active route."""
        if self._target_xy is None:
            return None
        target = (round(float(self._target_xy[0]), 3),
                  round(float(self._target_xy[1]), 3))
        yaw = None if self._target_yaw is None else round(float(self._target_yaw), 4)
        if self._mission_state in (self.MISSION_STANDBY, self.MISSION_SEARCH_QR):
            segment = 'search_qr'
        elif self._mission_state == self.MISSION_RETURN_TO_ENTRY:
            segment = 'return_to_entry'
        else:
            segment = self._mission_state
        return segment, target, yaw

    def _route_deviation(self, pose_xy):
        """Distance from the live pose to the currently validated route."""
        if not self._path:
            return float('inf')
        px, py = float(pose_xy[0]), float(pose_xy[1])
        best = float('inf')
        for first, second in zip(self._path, self._path[1:]):
            ax, ay = first
            bx, by = second
            dx, dy = bx - ax, by - ay
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-9:
                distance = math.hypot(px - ax, py - ay)
            else:
                ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
                distance = math.hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))
            best = min(best, distance)
        return min(best, math.hypot(px - self._path[-1][0], py - self._path[-1][1]))

    def _plan_worker(self, generation, route_signature, start_xy, start_yaw,
                     goal_xy, goal_yaw, goal_tolerance):
        """Compute a global route away from the 20 Hz command publisher."""
        started = time.perf_counter()
        try:
            new_path, new_headings, new_gears = self._plan_global(
                start_xy, start_yaw, goal_xy, goal_yaw, goal_tolerance
            )
        except Exception as exc:  # keep a planner fault from killing S1
            self.log.warn('PLAN', f'global worker exception: {exc}')
            new_path, new_headings, new_gears = [], [], []
        with self._lock:
            if generation != self._plan_generation or self._released:
                return
            self._plan_in_progress = False
            if new_path:
                self._path = new_path
                self._path_headings = new_headings
                self._path_gears = new_gears
                self._path_progress_index = 0
                self._local_plan_generation += 1
                self._local_result = None
                self._local_result_time = 0.0
                self._route_target_signature = route_signature
                self._route_locked = True
                self._local_failure_since = None
                self._publish_route_locked()
                self.log.target_pose(self._target_xy[0], self._target_xy[1], self._target_name)
            self.log.plan(f'global plan worker elapsed={time.perf_counter() - started:.3f}s '
                          f'path_points={len(new_path)}')

    def _start_global_plan_locked(self, pose_xy):
        """Start one global worker; caller holds ``_lock``."""
        if self._plan_in_progress or self._target_xy is None or self.current_yaw is None:
            return
        route_signature = self._route_signature()
        if route_signature is None:
            return
        self._last_plan_time = self._now()
        self._plan_generation += 1
        generation = self._plan_generation
        self._plan_in_progress = True
        if self._target_yaw is not None:
            goal_tolerance = self.entry_tolerance
        else:
            goal_tolerance = self.qr_search_radius
        self._plan_thread = threading.Thread(
            target=self._plan_worker,
            args=(generation, route_signature, tuple(pose_xy), float(self.current_yaw),
                  tuple(self._target_xy), self._target_yaw, goal_tolerance),
            name='s1_global_planner', daemon=True)
        self._plan_thread.start()

    def _refresh_plan(self, pose_xy):
        self._ensure_target()
        now = self._now()
        route_signature = self._route_signature()
        if route_signature is None:
            return False

        # A route belongs to one immutable mission segment.  Ordinary motion
        # never invalidates it; the local sampler follows it continuously.
        route_changed = self._route_target_signature != route_signature
        if route_changed:
            self._route_locked = False
            self._route_target_signature = route_signature
            self._local_failure_since = None
            # Keep the displayed and executed route identical: wait for the
            # single heading-aware global planner instead of exposing a
            # temporary straight prefix.

        failure_replan = False
        if self._route_locked and self._local_failure_since is not None:
            failure_age = now - self._local_failure_since
            route_deviated = self._route_deviation(pose_xy) >= self.route_deviation_threshold
            failure_replan = (
                failure_age >= self.replan_after_local_failure
                and (route_deviated or failure_age >= 2.0 * self.replan_after_local_failure)
                and now - self._last_failure_replan_time >= self.replan_cooldown
            )
            if failure_replan:
                self._route_locked = False
                self._route_target_signature = None
                self._last_failure_replan_time = now
                self._local_failure_since = None
                self._plan_generation += 1
                self._plan_in_progress = False
                self._local_plan_generation += 1
                self._local_result = None
                self._local_result_time = 0.0
                self.log.plan(
                    f'global route invalidated after sustained local failure '
                    f'duration={failure_age:.1f}s deviation={self._route_deviation(pose_xy):.2f}m'
                )

        if self._route_locked or self._plan_in_progress:
            return bool(self._path)
        if self._last_plan_time and now - self._last_plan_time < self.plan_retry_period:
            return bool(self._path)
        self._start_global_plan_locked(pose_xy)
        # Keep following the previous validated path while the replacement
        # plan is computed.  On the first plan there is no path yet, so the
        # control loop safely holds zero until the worker publishes one.
        return bool(self._path)

    def _local_worker(self, generation, pose_xy):
        """Evaluate one local command without blocking ROS sensor callbacks."""
        started = time.perf_counter()
        try:
            result = self._local_command(pose_xy)
        except Exception as exc:
            self.log.warn('LOCAL_PLAN', f'local worker exception: {exc}')
            result = None
        elapsed = time.perf_counter() - started
        with self._lock:
            if generation != self._local_plan_generation or self._released:
                self._local_plan_in_progress = False
                return
            self._local_plan_in_progress = False
            self._local_result = result
            self._local_result_time = self._now()
            if result is not None:
                self._last_safe_result_time = self._local_result_time
            if elapsed > 0.20:
                self.log.plan(f'local sampler elapsed={elapsed:.3f}s result={"safe" if result else "none"}')

    def _request_local_plan(self, pose_xy, now):
        with self._lock:
            if not self._path or self._local_plan_in_progress:
                return
            if now - self._last_local_request_time < self._local_replan_period:
                return
            self._last_local_request_time = now
            generation = self._local_plan_generation
            self._local_plan_in_progress = True
            self._local_plan_thread = threading.Thread(
                target=self._local_worker, args=(generation, tuple(pose_xy)),
                name='s1_local_sampler', daemon=True)
            self._local_plan_thread.start()

    def _publish_held_command(self, now):
        """Refresh the last command while a replacement plan is being computed."""
        held_command = (
            self._last_motion_cmd
            if self._command_is_motion(self._last_motion_cmd)
            else self._last_cmd
        )
        if (self._command_is_motion(held_command) and
                now - self._last_safe_result_time <= self._local_command_hold):
            self._publish_cmd(held_command.linear.x, held_command.angular.z)
            return True
        self._publish_zero()
        return False

    def _entry_ready(self, pose_xy, now):
        """Return true only after the live pose has settled at the handoff pose."""
        if self._mission_state != self.MISSION_RETURN_TO_ENTRY:
            self._entry_stable_since = None
            return False
        distance = math.hypot(
            pose_xy[0] - self.entry_goal[0], pose_xy[1] - self.entry_goal[1]
        )
        yaw_error = abs(self._norm(self.current_yaw - self.entry_yaw))
        if distance > self.entry_tolerance or yaw_error > self.entry_yaw_tolerance:
            self._entry_stable_since = None
            return False
        if self._entry_stable_since is None:
            self._entry_stable_since = now
            self.log.task(
                f'S1 entry gate entered distance={distance:.3f}m '
                f'yaw_error={math.degrees(yaw_error):.1f}deg; '
                f'settling={self.entry_stable_sec:.2f}s'
            )
            return self.entry_stable_sec <= 0.0
        return now - self._entry_stable_since >= self.entry_stable_sec

    def _start_entry_route(self, pose_xy):
        self._publish_mission_event_locked(self.MISSION_RETURN_TO_ENTRY)
        self._target_name = 'channel_entry'
        self._target_xy = self.entry_goal
        self._target_yaw = self.entry_yaw
        self._publish_mission_route_locked()
        self._invalidate_route_locked(pose_xy)
        self._entry_stable_since = None
        self.log.task(
            f'QR route interrupted at realtime pose=({pose_xy[0]:.3f},'
            f'{pose_xy[1]:.3f}); return route target='
            f'({self.entry_goal[0]:.2f},{self.entry_goal[1]:.2f},'
            f'{math.degrees(self.entry_yaw):.1f}deg)'
        )

    def _publish_entry_pose_and_handoff(self, pose_xy):
        if not self._entry_announced:
            msg = PoseStamped()
            msg.header.frame_id = self.map_frame
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = pose_xy[0]
            msg.pose.position.y = pose_xy[1]
            msg.pose.orientation.z = math.sin(self.current_yaw / 2.0)
            msg.pose.orientation.w = math.cos(self.current_yaw / 2.0)
            self.entry_pose_pub.publish(msg)
            self._publish_mission_event_locked(self.MISSION_HANDOFF_WAIT)
            self.state_pub.publish(String(data='handoff_ready'))
            self._entry_announced = True
            self._handoff_wait = True
            handoff_cmd = self._last_motion_cmd if self._command_is_motion(self._last_motion_cmd) else self._last_cmd
            self._last_cmd = handoff_cmd
            self._last_cmd_time = self._now()
            self.log.task(f'S1 handoff_ready entry=({pose_xy[0]:.3f},{pose_xy[1]:.3f}) '
                          f'map_yaw={math.degrees(self.current_yaw):.1f}deg '
                          f'target_yaw={math.degrees(self.entry_yaw):.1f}deg; '
                          'holding last valid motion')

    def _control_loop(self):
        if self._released:
            return
        now = self._now()
        with self._lock:
            pose_xy = self._lookup_map_pose_xy()
            if not self._ready_published and self._localization_ok(now):
                self._ready_published = True
                self.state_pub.publish(String(data='ready'))
                self.log.startup('S1 ready: IMU, odom, scan, map and map->base localization quality gate passed')
            if pose_xy is not None and self._ready_published:
                self._last_pose = pose_xy
                self._last_pose_time = now
                self.log.real_pose(pose_xy[0], pose_xy[1], source='radar_odom')
            if not self._ready_published or pose_xy is None or self.current_yaw is None:
                return
            localization_ok, localization_reason = self._localization_status(now)
            if not localization_ok:
                self._publish_zero()
                detail = f'quality gate failed ({localization_reason}); zero command'
                self.log.telemetry('localization_quality', detail)
                return
            # Warm the first QR-search route while S1 is still standby.  The
            # first activate command can then use an already validated path
            # and local safe command instead of spending several control
            # ticks at zero while the Python sampler is still starting.
            if not self._motion_enabled:
                # Prewarm before activate as well.  The route and first safe
                # command are computed while the stage is still stationary;
                # activate then only grants motion authority.
                self._refresh_plan(pose_xy)
                self._request_local_plan(pose_xy, now)
                if (not self._activation_requested or not self._path or
                        self._local_result is None or
                        now - self._local_result_time > self._local_command_hold):
                    return
                self._motion_enabled = True
                self._start_after = now + self.start_delay_sec
                self._running_published = False
                self.log.task('S1 prewarm complete; first motion command may be published continuously')
            if self._start_after is not None and now < self._start_after:
                return
            self._start_after = None
            if not self._running_published:
                self._running_published = True
                self.state_pub.publish(String(data='running'))
                self._enter_search_qr_locked()
            if self._handoff_wait:
                handoff_cmd = self._last_motion_cmd if self._command_is_motion(self._last_motion_cmd) else self._last_cmd
                self.cmd_pub.publish(handoff_cmd)
                self._last_cmd = handoff_cmd
                self._last_cmd_time = now
                return
            if self._entry_ready(pose_xy, now):
                self._publish_entry_pose_and_handoff(pose_xy)
                handoff_cmd = self._last_motion_cmd if self._command_is_motion(self._last_motion_cmd) else self._last_cmd
                self.cmd_pub.publish(handoff_cmd)
                return
            if not self._refresh_plan(pose_xy):
                self._publish_held_command(now)
                self.log.telemetry('no_safe_global_path',
                                   'holding last safe command briefly until static-map path exists')
                return
            self._request_local_plan(pose_xy, now)
            command = self._local_result
            if command is None or now - self._local_result_time > self._local_command_hold:
                if self._local_failure_since is None:
                    self._local_failure_since = now
                self._publish_held_command(now)
                detail = self._last_local_failure or 'candidate rejection details unavailable'
                self.log.telemetry('no_safe_local_trajectory',
                                   f'local sampler is computing or returned no safe trajectory; {detail}')
                return
            self._local_failure_since = None
            v, w, score = command
            published = self._publish_cmd(v, w)
            self.log.telemetry('nav', f'target={self._target_name} v={published.linear.x:.3f} '
                               f'w={published.angular.z:.3f} '
                               f'curvature={abs(published.angular.z)/max(abs(published.linear.x),1e-3):.2f} '
                               f'score={score:.2f} path_i={self._path_progress_index} '
                               f'route_yaw={math.degrees(self._last_route_heading or 0.0):+.1f}deg '
                               f'gear={self._last_local_gear}')

    def destroy_node(self):
        try:
            self._publish_zero()
        except Exception:
            pass
        with self._trace_lock:
            if self._localization_trace_file is not None:
                self._localization_trace_file.close()
                self._localization_trace_file = None
        self.log.close()
        super().destroy_node()


def main(args=None):
    install_parent_death_signal()
    rclpy.init(args=args)
    node = CompetitionController()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_zero()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

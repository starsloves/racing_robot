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

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

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
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.stage1_state_topic, latched)
        self.task_pub = self.create_publisher(String, self.task_topic, latched)
        self.entry_pose_pub = self.create_publisher(PoseStamped, self.entry_pose_topic, latched)
        self.imu_offset_pub = self.create_publisher(Float64, self.imu_offset_topic, latched)
        self.map_heading_pub = self.create_publisher(Float64, self.map_heading_topic, 10)
        self.route_pub = self.create_publisher(Path, self.route_topic, latched)
        self.mission_route_pub = self.create_publisher(Path, self.mission_route_topic, latched)

        prefix = self.lifecycle_service_prefix.rstrip('/')
        self._activate_srv = self.create_service(Trigger, f'{prefix}/activate', self._activate_cb)
        self._release_srv = self.create_service(Trigger, f'{prefix}/release', self._release_cb)

        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, 10)
        self.create_subscription(Float64, self.lidar_heading_topic, self._lidar_heading_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, latched)
        self.create_subscription(String, self.qr_result_topic, self._qr_cb, 10)
        self.create_subscription(String, self.diagnostic_topic, self._diagnostic_cb, latched)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.log = RacingLogger(self, log_subdir='competition_stage1',
                                log_filename='latest.log', session_title='Stage1 Nav2-style navigation')
        self._lock = threading.RLock()
        self._map = None
        self._map_blocked = None
        self._map_signature = None
        self._scan = None
        self._odom_xy = None
        self._last_scan_time = None
        self._last_imu_time = None
        self._last_odom_time = None
        self._last_map_time = None
        self._current_raw_yaw = None
        self._gyro_relative_yaw = 0.0
        self._gyro_anchor_relative_yaw = 0.0
        self._last_imu_stamp = None
        self._lidar_corrected_yaw = None
        self._last_lidar_heading_time = None
        self._initial_raw_yaw = None
        self._start_map_xy = None
        self._start_map_yaw = None
        self._start_odom_xy = None
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
        self._last_plan_pose = None
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
        self._diagnostic_state = 'waiting'
        self._qr_search_waypoints = []
        self._qr_search_index = 0
        self._control_timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)

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
            'imu_topic': '/imu/data', 'odom_topic': '/odom_combined',
            'map_topic': '/map', 'map_frame': 'map', 'base_frame': 'base_footprint',
            'odom_frame': 'odom_combined', 'stage1_state_topic': 'stage1_state',
            'qr_result_topic': 'qr_scan_result', 'task_topic': 'competition_qr_task',
            'stage2_entry_pose_topic': 'stage2_entry_pose',
            'imu_map_yaw_offset_topic': 'imu_map_yaw_offset',
            'map_heading_topic': 'map_heading',
            'lidar_heading_topic': 'map_heading_lidar',
            'lidar_heading_max_age_sec': 0.80,
            'route_topic': 'stage1_route',
            'mission_route_topic': 'stage1_mission_route',
            'start_corner_diagnostic_topic': 'start_corner_pose_diagnostic',
            'heading_motion_linear_threshold_mps': 0.015,
            'heading_motion_angular_threshold_rad_s': 0.03,
            'lifecycle_service_prefix': '/competition/stage1', 'control_rate_hz': 20.0,
            'start_delay_sec': 0.0,
            'localization_max_age_sec': 0.40,
            'qr_goal_x_m': 4.50, 'qr_goal_y_m': 1.60,
            'qr_search_radius_m': 0.45,
            'qr_search_waypoint_offset_m': 0.28,
            'qr_search_waypoint_tolerance_m': 0.16,
            'channel_entry_x_m': 2.50, 'channel_entry_y_m': 2.50,
            'channel_entry_yaw_deg': 90.0, 'channel_entry_tolerance_m': 0.16,
            'channel_entry_yaw_tolerance_deg': 12.0,
            'entry_stable_sec': 0.25,
            'global_replan_period_sec': 1.50, 'global_replan_position_delta_m': 0.65,
            'planner_grid_step_m': 0.16, 'planner_heading_bins': 16,
            'planner_motion_step_m': 0.20, 'planner_max_expansions': 30000,
            'planner_fast_grid_step_m': 0.12,
            'planner_fast_corridor_length_m': 0.70,
            'planner_occupied_threshold': 50, 'planner_unknown_is_occupied': True,
            'planner_robot_radius_m': 0.26, 'planner_min_turn_radius_m': 0.62,
            'planner_reverse_penalty': 2.8, 'planner_change_gear_penalty': 4.0,
            'planner_turn_penalty': 0.35, 'planner_steer_change_penalty': 0.40,
            'planner_goal_yaw_tolerance_deg': 18.0,
            'robot_body_length_m': 0.276, 'robot_body_width_m': 0.164,
            'robot_footprint_margin_m': 0.02, 'scan_self_filter_margin_m': 0.04,
            'local_horizon_sec': 1.00, 'local_dt_sec': 0.15,
            'local_samples_speed': 3, 'local_samples_steer': 5,
            'local_nominal_speed_mps': 0.42, 'local_min_speed_mps': 0.12,
            'local_max_speed_mps': 0.55, 'local_min_turn_radius_m': 0.62,
            'local_max_angular_speed_rad_s': 0.75,
            'local_path_lookahead_m': 0.42, 'local_footprint_radius_m': 0.28,
            'local_obstacle_clearance_m': 0.10, 'local_dynamic_obstacle_max_range_m': 2.2,
            'local_allow_reverse': True, 'local_reverse_speed_mps': -0.22,
            'local_reverse_only_when_forward_unsafe': True,
            'local_reverse_heading_threshold_deg': 105.0,
            'local_scan_max_points': 32,
            'local_reverse_penalty': 3.0, 'local_steer_change_penalty': 0.25,
            'local_heading_control_weight': 60.0, 'local_angular_effort_weight': 5.0,
            'local_path_distance_weight': 12.0, 'local_heading_weight': 2.5,
            'local_goal_weight': 4.0, 'local_clearance_weight': 3.0,
            'local_obstacle_cost_weight': 1000.0,
            'local_replan_period_sec': 0.20,
            'local_command_hold_sec': 0.80,
            'local_max_linear_accel_mps2': 0.80,
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
        self.lidar_heading_topic = str(get('lidar_heading_topic'))
        self.lidar_heading_max_age = max(0.2, float(get('lidar_heading_max_age_sec')))
        self.route_topic = str(get('route_topic'))
        self.mission_route_topic = str(get('mission_route_topic'))
        self.diagnostic_topic = str(get('start_corner_diagnostic_topic'))
        self.heading_motion_linear_threshold = max(
            0.0, float(get('heading_motion_linear_threshold_mps'))
        )
        self.heading_motion_angular_threshold = max(
            0.0, float(get('heading_motion_angular_threshold_rad_s'))
        )
        self.lifecycle_service_prefix = str(get('lifecycle_service_prefix'))
        self.control_rate_hz = max(5.0, float(get('control_rate_hz')))
        self.start_delay_sec = max(0.0, float(get('start_delay_sec')))
        self.localization_max_age = max(0.1, float(get('localization_max_age_sec')))
        self.qr_goal = (float(get('qr_goal_x_m')), float(get('qr_goal_y_m')))
        self.qr_search_radius = max(0.05, float(get('qr_search_radius_m')))
        self.qr_search_waypoint_offset = min(
            max(0.05, float(get('qr_search_waypoint_offset_m'))),
            max(0.05, self.qr_search_radius * 0.80),
        )
        self.qr_search_waypoint_tolerance = min(
            max(0.08, float(get('qr_search_waypoint_tolerance_m'))),
            max(0.08, self.qr_search_waypoint_offset * 0.70),
        )
        self.entry_goal = (float(get('channel_entry_x_m')), float(get('channel_entry_y_m')))
        self.entry_yaw = math.radians(float(get('channel_entry_yaw_deg')))
        self.entry_tolerance = max(0.05, float(get('channel_entry_tolerance_m')))
        self.entry_yaw_tolerance = math.radians(float(get('channel_entry_yaw_tolerance_deg')))
        self.entry_stable_sec = max(0.0, float(get('entry_stable_sec')))
        self.replan_period = max(0.2, float(get('global_replan_period_sec')))
        self.replan_delta = max(0.05, float(get('global_replan_position_delta_m')))
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
        self.reverse_penalty = float(get('planner_reverse_penalty'))
        self.change_gear_penalty = float(get('planner_change_gear_penalty'))
        self.turn_penalty = float(get('planner_turn_penalty'))
        self.planner_steer_change_penalty = float(get('planner_steer_change_penalty'))
        self.goal_yaw_tolerance = math.radians(float(get('planner_goal_yaw_tolerance_deg')))
        self.body_length = max(0.10, float(get('robot_body_length_m')))
        self.body_width = max(0.08, float(get('robot_body_width_m')))
        self.footprint_margin = max(0.0, float(get('robot_footprint_margin_m')))
        self.scan_self_filter_margin = max(0.0, float(get('scan_self_filter_margin_m')))
        self.horizon = max(0.5, float(get('local_horizon_sec')))
        self.local_dt = max(0.03, float(get('local_dt_sec')))
        self.speed_samples = max(3, int(get('local_samples_speed')))
        self.steer_samples = max(3, int(get('local_samples_steer')))
        self.nominal_speed = max(0.05, float(get('local_nominal_speed_mps')))
        self.min_speed = max(0.03, float(get('local_min_speed_mps')))
        self.max_speed = max(self.min_speed, float(get('local_max_speed_mps')))
        self.local_min_turn_radius = max(self.min_turn_radius, float(get('local_min_turn_radius_m')))
        self.max_angular = max(0.05, float(get('local_max_angular_speed_rad_s')))
        self.lookahead = max(0.10, float(get('local_path_lookahead_m')))
        self.footprint_radius = max(self.robot_radius, float(get('local_footprint_radius_m')))
        self.clearance = max(0.02, float(get('local_obstacle_clearance_m')))
        self.dynamic_max_range = max(0.5, float(get('local_dynamic_obstacle_max_range_m')))
        self.allow_reverse = bool(get('local_allow_reverse'))
        self.reverse_only_when_forward_unsafe = bool(get('local_reverse_only_when_forward_unsafe'))
        self.reverse_heading_threshold = math.radians(float(get('local_reverse_heading_threshold_deg')))
        self.local_scan_max_points = max(24, int(get('local_scan_max_points')))
        self.reverse_speed = -abs(float(get('local_reverse_speed_mps')))
        self.reverse_local_penalty = float(get('local_reverse_penalty'))
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

    def _imu_cb(self, msg):
        with self._lock:
            raw = self._yaw_from_quaternion(msg.orientation)
            self._current_raw_yaw = raw
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            if self._last_imu_stamp is None:
                self._last_imu_stamp = stamp
            else:
                dt = stamp - self._last_imu_stamp
                if 1e-4 <= dt <= 0.25:
                    self._gyro_relative_yaw += float(msg.angular_velocity.z) * dt
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
            self._last_imu_time = self._now()

    def _lidar_heading_cb(self, msg):
        try:
            corrected = self._norm(float(msg.data))
        except (TypeError, ValueError):
            return
        with self._lock:
            if self._start_map_yaw is None:
                return
            self._lidar_corrected_yaw = corrected
            self._last_lidar_heading_time = self._now()
            self._heading_anchor_yaw = corrected
            if self._current_raw_yaw is not None:
                self._initial_raw_yaw = self._current_raw_yaw
            self._gyro_anchor_relative_yaw = self._gyro_relative_yaw
            self.current_yaw = corrected
            self._publish_map_heading_locked()

    def _odom_cb(self, msg):
        with self._lock:
            self._odom_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
            self._last_odom_time = self._now()

    def _scan_cb(self, msg):
        with self._lock:
            self._scan = msg
            self._last_scan_time = self._now()

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
                self._path = []
                self._path_headings = []
                self._path_gears = []
                self._path_progress_index = 0
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
                    self._lidar_corrected_yaw = self._start_map_yaw
                    self._last_lidar_heading_time = self._now()
                    offset = self._norm(self._start_map_yaw - anchor_raw)
                    self.imu_offset_pub.publish(Float64(data=offset))
                    self._publish_map_heading_locked()
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
        self._motion_enabled = True
        self._start_after = self._now() + self.start_delay_sec
        self._running_published = False
        response.success = True
        response.message = 'stage1 activated; single /cmd_vel owner enabled'
        self.log.task(f'S1 activate received; start_delay={self.start_delay_sec:.2f}s')
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
        x, y = self.qr_goal
        d = self.qr_search_waypoint_offset
        # The first point is the map reference supplied by the course setup.
        # If the code is not visible there, the remaining points form a small
        # bounded lawn-mower loop instead of waiting forever at a zero-speed
        # endpoint.  The QR event, not completion of this loop, ends S1's
        # first mission segment.
        self._qr_search_waypoints = [
            (x, y), (x + d, y), (x + d, y + d), (x, y + d),
            (x - d, y + d), (x - d, y), (x - d, y - d),
            (x, y - d), (x + d, y - d),
        ]
        self._qr_search_index = 0
        self._publish_mission_route_locked()
        warmed_reference_path = bool(self._path) and self._target_xy == self.qr_goal
        if warmed_reference_path:
            self._target_name = 'qr_search_1'
            self._target_yaw = None
        else:
            self._set_qr_search_target_locked(None)
        self.log.task(
            f'S1 mission SEARCH_QR: route reference=({self.qr_goal[0]:.2f},'
            f'{self.qr_goal[1]:.2f}), radius={self.qr_search_radius:.2f}m; '
            f'bounded search offset={d:.2f}m; QR event, not exact coordinate, '
            'completes this phase'
        )

    def _set_qr_search_target_locked(self, pose_xy):
        if not self._qr_search_waypoints:
            self._target_name = 'qr_search'
            self._target_xy = self.qr_goal
            self._target_yaw = None
            return
        self._target_name = f'qr_search_{self._qr_search_index + 1}'
        self._target_xy = self._qr_search_waypoints[self._qr_search_index]
        self._target_yaw = None
        self._invalidate_route_locked(pose_xy)

    def _invalidate_route_locked(self, pose_xy=None):
        """Cancel stale planning work and install an optional safe connector."""
        self._last_plan_pose = None
        self._plan_generation += 1
        self._plan_in_progress = False
        self._local_plan_generation += 1
        self._local_result = None
        self._local_result_time = 0.0
        connector = []
        if pose_xy is not None and self._target_xy is not None:
            connector = self._build_connector_path(pose_xy, self._target_xy)
        if connector:
            self._path, self._path_headings, self._path_gears = connector
            self._path_progress_index = 0
            now = self._now()
            if abs(self._last_cmd.linear.x) > 1e-6:
                self._last_safe_result_time = now
        else:
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

    def _advance_qr_search_waypoint_locked(self, pose_xy):
        if (self._mission_state != self.MISSION_SEARCH_QR or
                not self._qr_search_waypoints or self._qr_latched):
            return False
        target = self._qr_search_waypoints[self._qr_search_index]
        tolerance = (self.qr_search_radius if self._qr_search_index == 0
                     else self.qr_search_waypoint_tolerance)
        if math.hypot(pose_xy[0] - target[0], pose_xy[1] - target[1]) > tolerance:
            return False
        previous = self._qr_search_index
        self._qr_search_index = (self._qr_search_index + 1) % len(self._qr_search_waypoints)
        self._set_qr_search_target_locked(pose_xy)
        self.log.task(
            f'QR search waypoint reached index={previous + 1}; '
            f'next={self._qr_search_index + 1} '
            f'target=({self._target_xy[0]:.2f},{self._target_xy[1]:.2f})'
        )
        return True

    def _publish_map_heading_locked(self):
        if self.current_yaw is not None:
            self.map_heading_pub.publish(Float64(data=float(self.current_yaw)))

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
        if self._qr_search_waypoints:
            points.extend(self._qr_search_waypoints[1:])
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
        if (
            self._start_map_xy is None
            or self._start_map_yaw is None
            or self._start_odom_xy is None
            or self._odom_xy is None
        ):
            return None
        dx = self._odom_xy[0] - self._start_odom_xy[0]
        dy = self._odom_xy[1] - self._start_odom_xy[1]
        cos_yaw = math.cos(self._start_map_yaw)
        sin_yaw = math.sin(self._start_map_yaw)
        return (
            self._start_map_xy[0] + cos_yaw * dx - sin_yaw * dy,
            self._start_map_xy[1] + sin_yaw * dx + cos_yaw * dy,
        )

    def _localization_ok(self, now):
        ages = (self._last_scan_time, self._last_imu_time, self._last_odom_time)
        if any(stamp is None or now - stamp > self.localization_max_age for stamp in ages):
            return False
        if self._map is None or self._map_blocked is None or self._last_map_time is None:
            return False
        # /map is transient-local static data and normally arrives once.  Do
        # not treat its age as sensor staleness; only a new map callback
        # replaces the cached static layer.
        return self._lookup_map_pose_xy() is not None and self.current_yaw is not None

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

    def _scan_points_base(self):
        """Convert the current LaserScan from its frame into base coordinates.

        The production TF chain places ``laser`` at ``base_link``, while
        ``base_link`` is offset from ``base_footprint``.  Treating raw scan
        angles as if the laser were at the footprint origin shifts every
        obstacle by that offset and can reject every local trajectory.
        """
        scan = self._scan
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
            if index + 1 < len(path):
                a, b = path[index], path[index + 1]
            elif index > 0:
                a, b = path[index - 1], path[index]
            else:
                headings.append(self.current_yaw)
                continue
            headings.append(math.atan2(b[1] - a[1], b[0] - a[0]))
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
            gx, gy, hbin, gear, previous_steer = state
            wx, wy = self._grid_to_world(gx, gy, step)
            yaw = (hbin / self.heading_bins) * 2.0 * math.pi
            if math.hypot(wx - goal_xy[0], wy - goal_xy[1]) <= goal_tolerance:
                if goal_bin is None or abs((hbin - goal_bin + self.heading_bins // 2) % self.heading_bins - self.heading_bins // 2) <= max(1, int(self.goal_yaw_tolerance * self.heading_bins / (2.0 * math.pi))):
                    goal_state = state
                    break
            for steer in primitives:
                for next_gear in ((1, -1) if self.allow_reverse else (1,)):
                    curvature = steer / self.min_turn_radius
                    direction = float(next_gear)
                    nx = wx + direction * self.motion_step * math.cos(yaw)
                    ny = wy + direction * self.motion_step * math.sin(yaw)
                    nyaw = self._norm(yaw + direction * curvature * self.motion_step)
                    ns_grid = self._world_to_grid(nx, ny, step)
                    if ns_grid is None:
                        continue
                    ngx, ngy = ns_grid
                    if self._is_blocked_world(nx, ny, nyaw):
                        continue
                    nhbin = int(round(nyaw / (2.0 * math.pi) * self.heading_bins)) % self.heading_bins
                    nstate = (ngx, ngy, nhbin, next_gear, steer)
                    step_cost = self.motion_step * (self.reverse_penalty if next_gear < 0 else 1.0)
                    step_cost += self.turn_penalty * abs(steer)
                    step_cost += self.planner_steer_change_penalty * abs(steer - previous_steer)
                    if next_gear != gear:
                        step_cost += self.change_gear_penalty
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
        gears = [s[3] for s in states]
        # Store the direction of travel, not merely the chassis yaw.  For a
        # reverse primitive the route tangent is yaw+pi; this distinction is
        # essential when the local controller decides whether to back into a
        # tight reconnect or drive forward along it.
        # The local tracker follows the actual direction of each polyline
        # segment.  Discrete Hybrid-A* chassis headings are only search
        # states; using them directly made the route tangent jump by heading
        # bin (e.g. 0->30->0 degrees) even when the path points were nearly
        # straight.
        headings = []
        for index in range(len(path)):
            if index + 1 < len(path):
                a, b = path[index], path[index + 1]
            elif index > 0:
                a, b = path[index - 1], path[index]
            else:
                headings.append(self.current_yaw)
                continue
            if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-6:
                headings.append(math.atan2(b[1] - a[1], b[0] - a[0]))
            else:
                headings.append(headings[-1] if headings else self.current_yaw)
        # The exact goal point is appended only after the search state.  Keep
        # the final tangent/gear explicit so the local controller never
        # invents a heading from the robot pose-to-goal vector.
        if path:
            path.append((float(goal_xy[0]), float(goal_xy[1])))
            if len(path) >= 2:
                previous, final = path[-2], path[-1]
                headings.append(math.atan2(final[1] - previous[1], final[0] - previous[0]))
            else:
                headings.append(self.current_yaw)
            gears.append(gears[-1])
        self.log.plan(f'global hybrid search success points={len(path)} expansions={expansions} '
                      f'goal={self._target_name}')
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
        points = []
        self_filtered = 0
        static_filtered = 0
        for base_x, base_y, distance, laser_x in base_points:
            map_x = pose_xy[0] + math.cos(yaw) * base_x - math.sin(yaw) * base_y
            map_y = pose_xy[1] + math.sin(yaw) * base_x + math.cos(yaw) * base_y
            # Ignore returns inside the physical chassis.  Returns on map
            # walls or outside the known map are static boundary evidence;
            # only returns in known free space become dynamic obstacles.
            if self._point_in_body_base(base_x, base_y, self.scan_self_filter_margin):
                self_filtered += 1
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
                    # A reverse trajectory follows the route tangent with the
                    # chassis facing the opposite way.  Penalise misalignment
                    # with that signed tangent, rather than the direction to
                    # the final goal, which otherwise rewards backing away
                    # from the route and then turning in place.
                    desired_heading = path_heading if v >= 0.0 else self._norm(path_heading + math.pi)
                    total += self.heading_weight * abs(self._norm(desired_heading - heading))
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
        if v < 0.0:
            total += self.reverse_local_penalty
        total += self.steer_change_penalty * abs(w - self._last_cmd.angular.z)
        if self._last_route_heading is not None:
            desired_heading = (self._last_route_heading if v >= 0.0 else
                               self._norm(self._last_route_heading + math.pi))
            heading_error = self._norm(desired_heading - yaw)
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
            speed = min(speed, max(self.min_speed, 0.18 + 0.30 * min(1.0, distance)))
        speeds = [speed * (0.55 + 0.45 * i / max(1, self.speed_samples - 1)) for i in range(self.speed_samples)]
        forward_candidates = []
        for v in speeds:
            max_w = min(self.max_angular, abs(v) / self.local_min_turn_radius)
            for j in range(self.steer_samples):
                ratio = 0.0 if self.steer_samples == 1 else (2.0 * j / (self.steer_samples - 1) - 1.0)
                forward_candidates.append((v, ratio * max_w))
        scan_points = self._scan_points_map(pose_xy, self.current_yaw)
        scored = [(self._trajectory_cost(pose_xy, self.current_yaw, v, w, scan_points), v, w)
                  for v, w in forward_candidates]
        safe_forward = [item for item in scored if item[1] >= 0.0 and math.isfinite(item[0])]
        safe_reverse = []
        prefer_reverse = abs(heading_error) > self.reverse_heading_threshold
        # Reverse is a recovery gear.  Do not spend half of every local
        # planning cycle scoring it when the forward set is already safe.
        if self.allow_reverse and (not safe_forward or prefer_reverse):
            reverse_candidates = []
            max_w = min(self.max_angular, abs(self.reverse_speed) / self.local_min_turn_radius)
            for j in range(self.steer_samples):
                ratio = 0.0 if self.steer_samples == 1 else (2.0 * j / (self.steer_samples - 1) - 1.0)
                reverse_candidates.append((self.reverse_speed, ratio * max_w))
            reverse_scored = [
                (self._trajectory_cost(pose_xy, self.current_yaw, v, w, scan_points), v, w)
                for v, w in reverse_candidates
            ]
            safe_reverse = [item for item in reverse_scored
                            if item[1] < 0.0 and math.isfinite(item[0])]
        # Reverse remains available, but it is a recovery gear.  It may not
        # win merely because its endpoint is closer to the goal: first use a
        # safe forward candidate, except when the route tangent is genuinely
        # behind the current chassis and backing along that tangent is the
        # intended reconnection.
        if self.reverse_only_when_forward_unsafe and safe_forward and not prefer_reverse:
            safe = safe_forward
        elif self.reverse_only_when_forward_unsafe and safe_forward and prefer_reverse:
            reverse_best = min(safe_reverse, default=None, key=lambda item: item[0])
            forward_best = min(safe_forward, key=lambda item: item[0])
            safe = safe_reverse if reverse_best is not None and reverse_best[0] + self.reverse_local_penalty < forward_best[0] else safe_forward
        else:
            safe = safe_forward + safe_reverse
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
        self._last_local_gear = 'R' if v < 0.0 else 'F'
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
        pose_xy = self._last_pose
        scan_points = self._scan_points_base()
        if scan_points is None:
            self.log.warn('COLLISION_MONITOR', 'laser TF unavailable; hard stop')
            return Twist()
        for base_x, base_y, distance, _ in scan_points:
            if self._point_in_body_base(base_x, base_y, self.scan_self_filter_margin):
                continue
            static_kind = 'free'
            if pose_xy is not None and self.current_yaw is not None:
                map_x = pose_xy[0] + math.cos(self.current_yaw) * base_x - math.sin(self.current_yaw) * base_y
                map_y = pose_xy[1] + math.sin(self.current_yaw) * base_x + math.cos(self.current_yaw) * base_y
                static_kind = self._static_cell_kind_world(map_x, map_y)
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
            norm = max(math.hypot(base_x, base_y), 1e-6)
            forward = (base_x / norm) * (1.0 if cmd.linear.x >= 0.0 else -1.0)
            if forward < math.cos(self.forward_half_angle):
                continue
            nearest = min(nearest, max(0.0, self._body_clearance_base(base_x, base_y)))
        effective = nearest
        ttc = effective / max(abs(cmd.linear.x), 1e-3)
        if effective <= self.stop_distance or ttc <= self.stop_ttc:
            self.log.warn('COLLISION_MONITOR', f'hard stop distance={effective:.2f}m ttc={ttc:.2f}s '
                          f'static_known_ignored={static_known_ignored} '
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
        v = previous_v + max(-max_dv, min(max_dv, float(v) - previous_v))
        w = previous_w + max(-max_dw, min(max_dw, float(w) - previous_w))
        return v, w

    def _publish_cmd(self, v, w):
        v, w = self._rate_limit_command(v, w)
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        msg = self._collision_monitor(msg)
        self.cmd_pub.publish(msg)
        self._last_cmd = msg
        self._last_cmd_time = self._now()
        if self._command_is_motion(msg):
            self._last_motion_cmd = msg
            self._last_motion_cmd_time = self._last_cmd_time
        self._update_heading_motion_state(msg)
        return msg

    def _ensure_target(self):
        if self._target_xy is not None:
            return
        if self._mission_state == self.MISSION_SEARCH_QR and self._qr_search_waypoints:
            self._set_qr_search_target_locked(None)
            return
        self._target_name = 'qr_search'
        self._target_xy = self.qr_goal
        self._target_yaw = None

    def _plan_worker(self, generation, start_xy, start_yaw, goal_xy, goal_yaw,
                     goal_tolerance):
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
                self._publish_route_locked()
                self.log.target_pose(self._target_xy[0], self._target_xy[1], self._target_name)
            self.log.plan(f'global plan worker elapsed={time.perf_counter() - started:.3f}s '
                          f'path_points={len(new_path)}')

    def _start_global_plan_locked(self, pose_xy):
        """Start one global worker; caller holds ``_lock``."""
        if self._plan_in_progress or self._target_xy is None or self.current_yaw is None:
            return
        self._last_plan_time = self._now()
        self._last_plan_pose = tuple(pose_xy)
        self._plan_generation += 1
        generation = self._plan_generation
        self._plan_in_progress = True
        if self._target_yaw is not None:
            goal_tolerance = self.entry_tolerance
        elif self._mission_state == self.MISSION_SEARCH_QR and self._qr_search_index > 0:
            goal_tolerance = self.qr_search_waypoint_tolerance
        else:
            goal_tolerance = self.qr_search_radius
        self._plan_thread = threading.Thread(
            target=self._plan_worker,
            args=(generation, tuple(pose_xy), float(self.current_yaw),
                  tuple(self._target_xy), self._target_yaw, goal_tolerance),
            name='s1_global_planner', daemon=True)
        self._plan_thread.start()

    def _refresh_plan(self, pose_xy):
        now = self._now()
        moved = (self._last_plan_pose is None or
                 math.hypot(pose_xy[0] - self._last_plan_pose[0], pose_xy[1] - self._last_plan_pose[1]) >= self.replan_delta)
        if (self._last_plan_pose is not None and
                now - self._last_plan_time < self.replan_period and not moved):
            return bool(self._path)
        self._ensure_target()
        self._last_plan_time = now
        self._last_plan_pose = pose_xy
        if not self._plan_in_progress:
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
        if (abs(self._last_cmd.linear.x) > 1e-6 and
                now - self._last_safe_result_time <= self._local_command_hold):
            self._publish_cmd(self._last_cmd.linear.x, self._last_cmd.angular.z)
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
        self._qr_search_waypoints = []
        self._qr_search_index = 0
        self._publish_mission_route_locked()
        self._invalidate_route_locked(pose_xy)
        fast_path, fast_headings, fast_gears = self._plan_fast_corridor(
            pose_xy, self.entry_goal, self.entry_tolerance
        )
        if fast_path:
            self._path = fast_path
            self._path_headings = fast_headings
            self._path_gears = fast_gears
            self._path_progress_index = 0
            self._publish_route_locked()
            self.log.plan(
                f'entry fast corridor points={len(fast_path)}; '
                'heading-aware global refinement queued'
            )
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
            if not self._localization_ok(now):
                self._publish_zero()
                self.log.telemetry('localization_quality', 'live IMU/odom/scan/TF quality gate failed; zero command')
                return
            # Warm the first QR-search route while S1 is still standby.  The
            # first activate command can then use an already validated path
            # instead of spending several control ticks at zero.
            if not self._motion_enabled:
                self._refresh_plan(pose_xy)
                return
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
            self._advance_qr_search_waypoint_locked(pose_xy)
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
                self._publish_held_command(now)
                detail = self._last_local_failure or 'candidate rejection details unavailable'
                self.log.telemetry('no_safe_local_trajectory',
                                   f'local sampler is computing or returned no safe trajectory; {detail}')
                return
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

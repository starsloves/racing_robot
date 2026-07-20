import math

import os
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PointStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Empty
from tf2_ros import TransformException

from racing_stage2.stage2_inertial_base import Stage2InertialBase
from racing_stage2.avoid_controller import AvoidConfig, AvoidController, NavState
from racing_stage2.avoid_geometry import cross_segment_m
from racing_stage2.scan_processor import ScanProcessor
from racing_stage2.session_file_log import SessionFileLog
from racing_common.obstacle_marker_publisher import ObstacleMarkerPublisher
from racing_stage2.stage2_vision_mixin import Stage2VisionMixin
from racing_stage2.track_controller import ImuDistancePose, Stage2TrackController


class Stage2InertialNavigator(Stage2InertialBase, Stage2VisionMixin):
    def __init__(self):
        super().__init__(node_name='stage2_inertial_navigator')
        
        self._last_wheel_odom_msg = None  # 保存最新的 wheel odom 消息用于 marker 坐标转换
        self._last_turn_target_yaw = None  # 上一个转弯的目标 yaw，用于直行段 heading 基准

        self.declare_parameter('test_direction', 'clockwise')
        self.declare_parameter('use_test_direction_fallback', False)
        self.declare_parameter('test_start_mode', 'auto')
        self.declare_parameter('test_feedback_prefix', '第二阶段')
        self.declare_parameter('start_stationary_speed_mps', 0.03)
        self.declare_parameter('start_stationary_yaw_rate_rps', 0.08)
        self.declare_parameter('start_stationary_hold_sec', 0.0)
        self.declare_parameter('track_max_speed', 0.34)
        self.declare_parameter('track_corner_speed', 0.12)
        self.declare_parameter('track_max_lateral_accel', 0.55)
        self.declare_parameter('track_heading_kp', 1.8)
        self.declare_parameter('track_line_heading_kp', 0.80)
        self.declare_parameter('track_cross_kp', 2.2)
        self.declare_parameter('track_curvature_kp', 1.0)
        self.declare_parameter('track_max_angular', 0.75)
        self.declare_parameter('track_entry_angular', 0.75)
        self.declare_parameter('track_corner_angular', 0.75)
        self.declare_parameter('track_entry_linear', 0.08)
        self.declare_parameter('track_entry_min_linear', 0.04)
        self.declare_parameter('track_entry_radius', 0.18)
        self.declare_parameter('track_entry_medium_distance_m', 0.85)
        self.declare_parameter('track_entry_boundary_trigger_enabled', False)
        self.declare_parameter('track_entry_boundary_guard_half_width_m', 0.15)
        self.declare_parameter('track_entry_boundary_top_y_ratio', 0.18)
        self.declare_parameter('track_entry_boundary_max_angle_deg', 20.0)
        self.declare_parameter('track_entry_boundary_confirm_frames', 3)
        self.declare_parameter('track_top_long_distance_m', 2.59)
        self.declare_parameter('track_exit_medium_distance_m', 1.49)
        self.declare_parameter('track_entry_tolerance_deg', 2.0)
        self.declare_parameter('track_settle_sec', 0.25)
        self.declare_parameter('track_entry_heading_kp', 1.0)
        self.declare_parameter('track_yaw_rate_damping', 0.30)
        self.declare_parameter('track_entry_yaw_rate_tolerance', 0.10)
        self.declare_parameter('track_entry_align_max_distance_m', 0.45)
        self.declare_parameter('track_entry_align_error_tolerance', 0.08)
        self.declare_parameter('track_entry_align_hold_sec', 0.20)
        self.declare_parameter('track_entry_align_visual_kp', 0.55)
        self.declare_parameter('track_arc_min_completion_ratio', 0.70)
        self.declare_parameter('track_arc_finish_predict_sec', 0.18)
        self.declare_parameter('track_arc_mismatch_angle_deg', 14.0)
        self.declare_parameter('track_entry_stop_prepare_deg', 0.0)
        self.declare_parameter('track_entry_arc_complete_lead_deg', 0.0)
        self.declare_parameter('track_corner_arc_complete_lead_deg', 0.0)
        self.declare_parameter('track_corner_radius', 0.18)
        self.declare_parameter('track_vision_lateral_scale_m', 0.30)
        self.declare_parameter('track_vision_lateral_weight', 0.35)
        self.declare_parameter('track_vision_correction_max_angular', 0.10)
        self.declare_parameter('track_lookahead_m', 0.45)
        self.declare_parameter('track_heading_slowdown_deg', 10.0)
        self.declare_parameter('track_finish_tolerance_m', 0.10)
        self.declare_parameter('track_odom_combined_step_max_m', 0.12)
        self.declare_parameter('track_stage3_handoff_map_y', 2.0)
        self.declare_parameter('stage3_entry_anchor_topic', 'stage3_entry_anchor')
        self.declare_parameter('stage3_entry_anchor_base_frame', 'base_footprint')
        self.declare_parameter('stage2_ai_capture_enabled', True)
        self.declare_parameter('stage2_ai_capture_lead_m', 0.50)
        self.declare_parameter('stage2_ai_trigger_topic', 'stage2_ai_capture')
        # 避障参数（yaml 配置，直行避障使用）
        self.declare_parameter('avoid_turn_away_deg', 30.0)
        self.declare_parameter('avoid_turn_back_deg', 40.0)
        self.declare_parameter('avoid_recover_deg', 40.0)
        self.declare_parameter('avoid_leg1_distance_m', 0.30)
        self.declare_parameter('avoid_leg2_distance_m', 0.60)
        self.declare_parameter('avoid_leg_linear_speed', 0.10)
        self.declare_parameter('avoid_turn_linear_speed', 0.08)
        self.declare_parameter('avoid_leg_distance_tol_m', 0.04)
        self.declare_parameter('avoid_turn_angular_speed', 0.40)
        self.declare_parameter('side_detour_threshold_m', 0.18)
        self.declare_parameter('avoider_heading_tolerance_deg', 1.5)
        # 转弯障碍检测参数
        self.declare_parameter('turn_obstacle_stop_m', 0.25)
        self.declare_parameter('corner_approach_m', 0.15)
        self.declare_parameter('turn_obstacle_creep_speed', 0.02)
        # 转弯减速参数
        self.declare_parameter('turn_slowdown_threshold_deg', 15.0)
        self.declare_parameter('turn_min_speed_ratio', 0.4)
        self.declare_parameter('turn_inertia_compensation_deg', 0.0)  # 已减速，无惯性补偿
        # 转角系统性补偿（IMU/机械零点偏差）
        self.declare_parameter('turn_angle_compensation_deg', 0.0)  # 每次转弯额外补偿角度
        # 加速渐变参数（转弯后平滑过渡）
        self.declare_parameter('move_accel_ramp_sec', 0.5)  # 转弯后加速渐变时长（秒）
        self.declare_parameter('command_heartbeat_rate_hz', 20.0)

        self.test_direction_raw = str(self.get_parameter('test_direction').value).strip()
        self.test_direction = self.resolve_test_direction(self.test_direction_raw)
        self.use_test_direction_fallback = bool(self.get_parameter('use_test_direction_fallback').value)
        self.test_start_mode = str(self.get_parameter('test_start_mode').value).strip().lower() or 'auto'
        self.test_feedback_prefix = str(self.get_parameter('test_feedback_prefix').value).strip() or '第二阶段'
        self._track_config_name = 'stage2_controller.yaml::track_*'

        # 比赛默认待命：只有 competition_phase=2 后才启动
        self.phase = 1
        self.phase_initialized = False
        self.waiting_for_phase2_start = False
        # 独立测试可用 test_direction；比赛中方向以 competition_qr_task 为准
        self.task_raw = ''
        self.direction = None

        self.reported_waiting_pose = False
        self.reported_start_delay = False
        self.last_progress_bucket = -1
        self.active_turn_heading_tolerance = self.heading_tolerance

        # 独立模块：雷达处理 + 避障
        self._scan_processor = ScanProcessor(
            front_angle_deg=self.detour_front_angle_deg,
            side_window_deg=self.detour_side_window_deg,
            side_center_deg=self.detour_side_center_deg,
        )
        self.front_obstacle_distance = float('inf')
        self.front_obstacle_angle_deg = 0.0
        self.left_clearance_distance = float('inf')
        self.right_clearance_distance = float('inf')

        # 视觉模块初始化（必须在 avoider 之前，因为 avoider 需要视觉回调）
        # 默认关闭推理：Stage1 期间不跑视觉，等 phase=2 再启用
        self._setup_vision_centering()
        if getattr(self, '_vision_node', None) is not None:
            self._vision_node.set_inference_active(False)
        
        _avoid_cfg = AvoidConfig(
            detour_obstacle_distance=self.detour_obstacle_distance,
            avoid_turn_away_deg=float(self.get_parameter('avoid_turn_away_deg').value),
            avoid_turn_back_deg=float(self.get_parameter('avoid_turn_back_deg').value),
            avoid_recover_deg=float(self.get_parameter('avoid_recover_deg').value),
            avoid_leg1_distance_m=max(0.05, float(self.get_parameter('avoid_leg1_distance_m').value)),
            avoid_leg2_distance_m=max(0.05, float(self.get_parameter('avoid_leg2_distance_m').value)),
            avoid_leg_linear_speed=max(0.02, float(self.get_parameter('avoid_leg_linear_speed').value)),
            avoid_turn_linear_speed=max(0.02, float(self.get_parameter('avoid_turn_linear_speed').value)),
            avoid_leg_distance_tol_m=max(0.0, float(self.get_parameter('avoid_leg_distance_tol_m').value)),
            avoid_turn_angular_speed=max(0.1, float(self.get_parameter('avoid_turn_angular_speed').value)),
            distance_tolerance=self.distance_tolerance,
            heading_kp=self.heading_kp,
            side_detour_threshold_m=float(self.get_parameter('side_detour_threshold_m').value),
            avoider_heading_tolerance_deg=float(self.get_parameter('avoider_heading_tolerance_deg').value),
        )
        self._avoider = AvoidController(
            self.cmd_pub, 
            self.get_logger(), 
            self.get_clock(), 
            _avoid_cfg,
            vision_callback=self._get_vision_angular_for_avoider  # 传递视觉回调
        )

        # 障碍物可视化（rviz2 调试用）
        # 使用 laser 帧，和 LaserScan 点云保持一致，依赖 TF 自动转换
        self.obstacle_markers = ObstacleMarkerPublisher(
            self, topic='/stage2_obstacle_markers', frame_id='laser', radius=0.13
        )
        self.all_clusters = []  # 缓存当前帧的所有聚类
        self._cluster_window_config = {
            'min_x': 0.30,
            'max_x': 2.50,
            'half_y': 0.50,
            'gap_tolerance': 0.12
        }

        self._setup_wheel_odom_position()
        self._setup_session_log()
        self._track_controller = Stage2TrackController(
            max_speed=float(self.get_parameter('track_max_speed').value),
            corner_speed=float(self.get_parameter('track_corner_speed').value),
            max_lateral_accel=float(self.get_parameter('track_max_lateral_accel').value),
            stanley_heading_kp=float(self.get_parameter('track_heading_kp').value),
            line_heading_kp=float(self.get_parameter('track_line_heading_kp').value),
            stanley_cross_kp=float(self.get_parameter('track_cross_kp').value),
            curvature_kp=float(self.get_parameter('track_curvature_kp').value),
            max_angular=float(self.get_parameter('track_max_angular').value),
            entry_angular=float(self.get_parameter('track_entry_angular').value),
            corner_angular=float(self.get_parameter('track_corner_angular').value),
            entry_linear=float(self.get_parameter('track_entry_linear').value),
            entry_min_linear=float(self.get_parameter('track_entry_min_linear').value),
            entry_radius=float(self.get_parameter('track_entry_radius').value),
            entry_medium_distance_m=float(
                self.get_parameter('track_entry_medium_distance_m').value
            ),
            entry_boundary_trigger_enabled=bool(
                self.get_parameter('track_entry_boundary_trigger_enabled').value
            ),
            entry_boundary_guard_half_width_m=float(
                self.get_parameter('track_entry_boundary_guard_half_width_m').value
            ),
            entry_boundary_top_y_ratio=float(
                self.get_parameter('track_entry_boundary_top_y_ratio').value
            ),
            entry_boundary_max_angle_deg=float(
                self.get_parameter('track_entry_boundary_max_angle_deg').value
            ),
            entry_boundary_confirm_frames=int(
                self.get_parameter('track_entry_boundary_confirm_frames').value
            ),
            top_long_distance_m=float(
                self.get_parameter('track_top_long_distance_m').value
            ),
            exit_medium_distance_m=float(
                self.get_parameter('track_exit_medium_distance_m').value
            ),
            entry_tolerance_deg=float(self.get_parameter('track_entry_tolerance_deg').value),
            settle_sec=float(self.get_parameter('track_settle_sec').value),
            entry_heading_kp=float(self.get_parameter('track_entry_heading_kp').value),
            yaw_rate_damping=float(self.get_parameter('track_yaw_rate_damping').value),
            entry_yaw_rate_tolerance=float(
                self.get_parameter('track_entry_yaw_rate_tolerance').value
            ),
            entry_align_max_distance_m=float(
                self.get_parameter('track_entry_align_max_distance_m').value
            ),
            entry_align_error_tolerance=float(
                self.get_parameter('track_entry_align_error_tolerance').value
            ),
            entry_align_hold_sec=float(
                self.get_parameter('track_entry_align_hold_sec').value
            ),
            entry_align_visual_kp=float(
                self.get_parameter('track_entry_align_visual_kp').value
            ),
            arc_min_completion_ratio=float(
                self.get_parameter('track_arc_min_completion_ratio').value
            ),
            arc_finish_predict_sec=float(
                self.get_parameter('track_arc_finish_predict_sec').value
            ),
            arc_mismatch_angle_deg=float(
                self.get_parameter('track_arc_mismatch_angle_deg').value
            ),
            entry_stop_prepare_deg=float(
                self.get_parameter('track_entry_stop_prepare_deg').value
            ),
            entry_arc_complete_lead_deg=float(
                self.get_parameter('track_entry_arc_complete_lead_deg').value
            ),
            corner_arc_complete_lead_deg=float(
                self.get_parameter('track_corner_arc_complete_lead_deg').value
            ),
            corner_radius=float(self.get_parameter('track_corner_radius').value),
            vision_lateral_scale_m=float(self.get_parameter('track_vision_lateral_scale_m').value),
            vision_lateral_weight=float(self.get_parameter('track_vision_lateral_weight').value),
            vision_correction_max_angular=float(
                self.get_parameter('track_vision_correction_max_angular').value
            ),
            lookahead_m=float(self.get_parameter('track_lookahead_m').value),
            heading_slowdown_deg=float(
                self.get_parameter('track_heading_slowdown_deg').value
            ),
            finish_tolerance_m=float(self.get_parameter('track_finish_tolerance_m').value),
        )
        self._track_mission_active = False
        self._stage2_ai_capture_enabled = bool(
            self.get_parameter('stage2_ai_capture_enabled').value
        )
        self._stage2_ai_capture_lead_m = max(
            0.0, float(self.get_parameter('stage2_ai_capture_lead_m').value)
        )
        self._stage2_ai_capture_sent = False
        self._stage2_ai_trigger_topic = str(
            self.get_parameter('stage2_ai_trigger_topic').value
        ).strip() or 'stage2_ai_capture'
        self._stage2_ai_trigger_pub = self.create_publisher(
            Empty, self._stage2_ai_trigger_topic, 1
        )
        anchor_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self._stage3_anchor_pub = self.create_publisher(
            PointStamped,
            str(self.get_parameter('stage3_entry_anchor_topic').value),
            anchor_qos,
        )
        self._track_stage3_handoff_map_y = float(
            self.get_parameter('track_stage3_handoff_map_y').value
        )
        self._stage3_entry_anchor_base_frame = str(
            self.get_parameter('stage3_entry_anchor_base_frame').value
        ).strip() or 'base_footprint'
        # The track controller works in an IMU-aligned local frame.  The EKF
        # odometry contributes only travelled distance; its orientation is
        # deliberately never used by Stage2 navigation.
        self._track_pose = None
        self._track_pose_integrator = ImuDistancePose(
            max_step_m=float(self.get_parameter('track_odom_combined_step_max_m').value)
        )
        self._track_distance_m = 0.0
        self._stationary_since = None
        self._last_stationary_log = 0.0
        self._pure_linear_after_avoid = False  # 避障完成后纯线速度直行
        
        # 加速渐变状态（转弯后平滑过渡）
        self._just_finished_turn = False
        self._turn_finish_time = 0.0
        self._accel_ramp_duration = float(self.get_parameter('move_accel_ramp_sec').value)
        self._last_ramp_pct = -1  # 记录上次日志的进度百分比
        
        self.segment_end_pose = None  # 世界坐标系下的段终点（仅 world 模式使用）
        self._world_start_pose = None  # 世界坐标系下的起点信息
        self._segment_is_world = False  # 当前段是否使用世界坐标系
        self._segment_start_wheel_yaw = None  # 直行段起点轮速航向（用于 cross-track 与 odom 位置同坐标系）
        self._map_origin_x = 2.50  # map→odom 变换参数（从 launch 传入）
        self._map_origin_y = 2.80
        self._map_origin_yaw = math.radians(90.0)
        self._command_lock = threading.Lock()
        self._last_command_at = time.monotonic()
        self._heartbeat_stale_reported = False
        heartbeat_hz = max(5.0, float(self.get_parameter('command_heartbeat_rate_hz').value))
        self._command_heartbeat_group = ReentrantCallbackGroup()
        self._command_heartbeat_timer = self.create_timer(
            1.0 / heartbeat_hz,
            self._command_heartbeat,
            callback_group=self._command_heartbeat_group,
        )

        self.get_logger().info(
            f'{self.test_feedback_prefix}导航节点已就绪，方向={self.direction_text()}，'
            f'模式={self.start_mode_text()}，'
            f'track_config={self._track_config_name}，'
            f'避障=边转边避 away={_avoid_cfg.avoid_turn_away_deg:.0f}deg back={_avoid_cfg.avoid_turn_back_deg:.0f}deg recover={_avoid_cfg.avoid_recover_deg:.0f}deg×'
            f'{_avoid_cfg.avoid_leg1_distance_m:.2f}m/{_avoid_cfg.avoid_leg2_distance_m:.2f}m '
            f'侧边阈值={_avoid_cfg.side_detour_threshold_m:.2f}m '
            f'闭环转弯 tol={_avoid_cfg.avoider_heading_tolerance_deg:.1f}deg，'
            f'可视化窗口=[{self._cluster_window_config["min_x"]:.2f}-{self._cluster_window_config["max_x"]:.2f}m, '
            f'±{self._cluster_window_config["half_y"]:.2f}m]'
        )
        self.get_logger().info(
            f'[AI_CAPTURE] enabled={self._stage2_ai_capture_enabled} '
            f'topic={self._stage2_ai_trigger_topic} lead={self._stage2_ai_capture_lead_m:.2f}m '
            f'trigger=top_long_before_right_side_arc'
        )
        self._log_session(
            'CONFIG',
            f'方向={self.direction_text()} 模式={self.start_mode_text()} '
            f'track_config={self._track_config_name} '
            f'避障 away={_avoid_cfg.avoid_turn_away_deg:.0f}deg back={_avoid_cfg.avoid_turn_back_deg:.0f}deg recover={_avoid_cfg.avoid_recover_deg:.0f}deg '
            f'L1={_avoid_cfg.avoid_leg1_distance_m:.2f}m L2={_avoid_cfg.avoid_leg2_distance_m:.2f}m '
            f'pose_source={self._navigation_pose_source} '
            f'wheel={self._wheel_odom_topic} ekf={self.odom_topic} '
            f'ring_v={self.ring_linear_speed:.2f} turn_v={self.turn_linear_speed:.2f} '
            f'turn_w={self.turn_angular_speed:.2f} head_kp={self.heading_kp:.2f} '
            f'dist_tol={self.distance_tolerance:.3f} '
            f'head_tol={math.degrees(self.heading_tolerance):.1f}deg '
            f'detour_d={self.detour_obstacle_distance:.2f}m '
            f'segment_timeout={self.segment_timeout:.1f}s',
        )
        self._log_session(
            'AI_CAPTURE_CONFIG',
            f'enabled={self._stage2_ai_capture_enabled} '
            f'topic={self._stage2_ai_trigger_topic} '
            f'lead={self._stage2_ai_capture_lead_m:.2f}m '
            f'trigger=top_long_before_right_side_arc',
        )

    def _setup_session_log(self) -> None:
        self.declare_parameter('session_log_subdir', 'direct_inertial_test')
        self.declare_parameter('session_log_filename', 'latest.log')
        self.declare_parameter('session_telemetry_interval_sec', 0.25)
        self.declare_parameter('control_gap_warn_sec', 0.35)
        subdir = (
            str(self.get_parameter('session_log_subdir').value).strip()
            or 'direct_inertial_test'
        )
        filename = (
            str(self.get_parameter('session_log_filename').value).strip() or 'latest.log'
        )
        self._telemetry_interval_sec = max(
            0.05, float(self.get_parameter('session_telemetry_interval_sec').value)
        )
        self._session_log = SessionFileLog(
            subdir,
            filename,
            session_title='direct inertial test session',
            workspace_root=os.path.expanduser('~/dev_ws'),
        )
        self._last_telemetry_sec = 0.0
        self._last_wait_log_sec = 0.0
        self._wheel_warmup_logged = False
        self._last_ekf_position = None
        self._wheel_twist = None
        self._ekf_twist = None
        self._last_cmd_linear = 0.0
        self._last_cmd_angular = 0.0
        self._last_control_loop_sec = None
        self._last_cmd_publish_sec = None
        self._control_gap_warn_sec = max(
            0.10, float(self.get_parameter('control_gap_warn_sec').value)
        )
        self.get_logger().info(
            f'{self.test_feedback_prefix}会话日志: {self._session_log.path}'
        )
        self._log_session('CONFIG', f'日志路径={self._session_log.path}')

    def destroy_node(self):
        self._set_vision_inference_active(False)
        self._set_stage2_http_active(False)
        vision_node = getattr(self, '_vision_node', None)
        if vision_node is not None and hasattr(vision_node, 'shutdown'):
            vision_node.shutdown()
        if getattr(self, '_session_log', None) is not None:
            self._session_log.close()
            self._session_log = None
        super().destroy_node()

    def _log_session(self, tag: str, message: str) -> None:
        if getattr(self, '_session_log', None) is None:
            return
        self._session_log.write(f'[{tag}] {message}')

    def publish_feedback(self, text: str) -> None:
        super().publish_feedback(text)
        self._log_session('FEEDBACK', text)

    def create_twist(self, linear_x=0.0, angular_z=0.0):
        with getattr(self, '_command_lock', threading.Lock()):
            self._last_cmd_linear = float(linear_x)
            self._last_cmd_angular = float(angular_z)
            self._last_command_at = time.monotonic()
            self._heartbeat_stale_reported = False
        return super().create_twist(linear_x, angular_z)

    def _command_heartbeat(self) -> None:
        """Keep command output continuous while track control is active."""
        if not getattr(self, '_track_mission_active', False):
            return
        with self._command_lock:
            age = time.monotonic() - self._last_command_at
            linear = self._last_cmd_linear
            angular = self._last_cmd_angular
        if age > self._control_gap_warn_sec and not self._heartbeat_stale_reported:
            self._heartbeat_stale_reported = True
            self._log_session(
                'CMD_HEARTBEAT_HOLD',
                f'control_age={age:.3f}s -> holding last command',
            )
        self.cmd_pub.publish(super().create_twist(linear, angular))

    def _log_control_timing(self, now_sec: float, tag: str, will_publish: bool = True) -> None:
        last_loop = getattr(self, '_last_control_loop_sec', None)
        last_pub = getattr(self, '_last_cmd_publish_sec', None)
        loop_gap = 0.0 if last_loop is None else now_sec - last_loop
        publish_gap = 0.0 if last_pub is None else now_sec - last_pub
        self._last_control_loop_sec = now_sec
        if will_publish:
            self._last_cmd_publish_sec = now_sec
        if loop_gap <= self._control_gap_warn_sec and publish_gap <= self._control_gap_warn_sec:
            return
        visual_age = float('nan')
        visual_valid = False
        if getattr(self, '_vision_node', None) is not None:
            try:
                line = self._get_vision_line_status()
                visual_age = float(line.get('age', 999.0) or 999.0)
                visual_valid = bool(line.get('valid', False))
            except Exception:
                visual_age = float('nan')
                visual_valid = False
        self._log_session(
            'CTRL_GAP',
            f'tag={tag} loop_gap={loop_gap:.3f}s publish_gap={publish_gap:.3f}s '
            f'will_publish={int(bool(will_publish))} '
            f'cmd=({self._last_cmd_linear:.3f},{self._last_cmd_angular:.3f}) '
            f'mission={int(bool(self.mission_active))} track={int(bool(self._track_mission_active))} '
            f'seg={(self.current_segment or {}).get("description", "none")} '
            f'vision_valid={int(visual_valid)} vision_age={visual_age:.2f}s '
            f'{self._full_telemetry()}',
        )

    def navigation_yaw(self):
        """统一位姿航向（current_yaw）；轮速模式下由 /odom 写入，IMU 仅诊断。"""
        return self.current_yaw

    def _wheel_pose_source_active(self) -> bool:
        return (
            getattr(self, '_navigation_pose_source', 'wheel') == 'wheel'
            and self._use_wheel_odom_for_distance
        )

    def _sync_unified_pose_from_wheel(self) -> None:
        if not self._wheel_pose_source_active():
            return
        if self.current_wheel_yaw is None:
            return
        # 位置用轮速里程计，但航向角始终用 IMU（轮速 yaw 不准）
        # self.current_yaw = self.current_wheel_yaw  # 注释掉，改用 IMU yaw

    def imu_callback(self, msg):
        self.imu_yaw = self.quaternion_to_yaw(msg.orientation)
        self.current_imu_yaw_rate = float(msg.angular_velocity.z)
        # 航向角始终使用 IMU，不管位置源是什么
        self.current_yaw = self.imu_yaw
        if self.waiting_for_phase2_start:
            self.try_start_mission()

    def _fmt_num(self, value, prec=3):
        if value is None:
            return 'nan'
        try:
            number = float(value)
        except (TypeError, ValueError):
            return 'nan'
        if not math.isfinite(number):
            return 'inf' if math.isinf(number) else 'nan'
        return f'{number:.{prec}f}'

    def _raw_projected_distance(self) -> float:
        if (
            self.segment_start_pose is None
            or self.current_position is None
            or self.segment_heading is None
        ):
            return 0.0
        dx = self.current_position[0] - self.segment_start_pose[0]
        dy = self.current_position[1] - self.segment_start_pose[1]
        return (
            dx * math.cos(self.segment_heading)
            + dy * math.sin(self.segment_heading)
        )

    def _cross_track_m(self) -> float:
        if (
            self.segment_start_pose is None
            or self.current_position is None
        ):
            return 0.0
        nav_pos = self._nav_position()
        if nav_pos is None:
            return 0.0
        # 用轮速航向（与 odom 位置同坐标系），fallback 到 IMU heading
        heading_for_cross = self._segment_start_wheel_yaw if self._segment_start_wheel_yaw is not None else self.segment_heading
        if heading_for_cross is None:
            return 0.0
        return cross_segment_m(
            self.segment_start_pose,
            heading_for_cross,
            nav_pos,
        )

    def _odom_to_map(self, odom_xy):
        """将 odom 帧坐标转换为 map 帧坐标。
        
        map→odom 静态变换: 平移 (2.50, 2.80)，旋转 90° (yaw)
        odom_x = -(map_y - 2.80)  
        odom_y = map_x - 2.50
        
        逆变换 → map_x = 2.50 + odom_y, map_y = 2.80 - odom_x
        """
        if odom_xy is None:
            return None
        ox, oy = odom_xy
        return (self._map_origin_x + oy, self._map_origin_y - ox)
    
    def _nav_position(self):
        """获取导航用位置：world 模式用 map 帧，否则用 odom 帧"""
        if self.current_position is None:
            return None
        if self._segment_is_world:
            return self._odom_to_map(self.current_position)
        return self.current_position

    def _full_telemetry(self) -> str:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        segment = self.current_segment or {}
        seg_type = str(segment.get('type', 'none'))
        seg_desc = str(segment.get('description', 'none'))

        wx = wy = 'nan'
        if self.current_position is not None:
            wx = self._fmt_num(self.current_position[0])
            wy = self._fmt_num(self.current_position[1])

        ekf_x = ekf_y = 'nan'
        if self._last_ekf_position is not None:
            ekf_x = self._fmt_num(self._last_ekf_position[0])
            ekf_y = self._fmt_num(self._last_ekf_position[1])

        sx = sy = 'nan'
        if self.segment_start_pose is not None:
            sx = self._fmt_num(self.segment_start_pose[0])
            sy = self._fmt_num(self.segment_start_pose[1])

        wvx = wvy = wwz = 'nan'
        if self._wheel_twist is not None:
            wvx = self._fmt_num(self._wheel_twist[0])
            wvy = self._fmt_num(self._wheel_twist[1])
            wwz = self._fmt_num(self._wheel_twist[2])

        evx = evy = ewz = 'nan'
        if self._ekf_twist is not None:
            evx = self._fmt_num(self._ekf_twist[0])
            evy = self._fmt_num(self._ekf_twist[1])
            ewz = self._fmt_num(self._ekf_twist[2])

        along = self.projected_distance() if self.current_position is not None else 0.0
        raw_along = self._raw_projected_distance() if self.current_position is not None else 0.0
        cross_cm = self._cross_track_m() * 100.0 if self.current_position is not None else 0.0

        target_m = seg_speed = 0.0
        if seg_type == 'move':
            target_m = float(segment.get('distance_m', 0.0))
            seg_speed = float(segment.get('speed', 0.0))

        nav_yaw = self.navigation_yaw()
        heading_err_deg = 'nan'
        if self.segment_heading is not None and nav_yaw is not None:
            heading_err_deg = self._fmt_num(
                math.degrees(self.angle_error(self.segment_heading, nav_yaw)),
                prec=1,
            )

        turn_err_deg = 'nan'
        if self.segment_target_yaw is not None and nav_yaw is not None:
            turn_err_deg = self._fmt_num(
                math.degrees(self.angle_error(self.segment_target_yaw, nav_yaw)),
                prec=1,
            )

        imu_wheel_err_deg = 'nan'
        if self.imu_yaw is not None and self.current_wheel_yaw is not None:
            imu_wheel_err_deg = self._fmt_num(
                math.degrees(self.angle_error(self.imu_yaw, self.current_wheel_yaw)),
                prec=1,
            )

        seg_elapsed = 'nan'
        if self.segment_started_at is not None:
            seg_elapsed = self._fmt_num(now_sec - self.segment_started_at, prec=2)

        parts = [
            (
                f't={now_sec:.3f} mission={int(self.mission_active)} '
                f'done={int(self.mission_finished)} '
                f'plan={self.plan_index}/{max(len(self.plan) - 1, 0)} '
                f'seg={seg_type}:{seg_desc} seg_t={seg_elapsed}s'
            ),
            f'wheel_xy=({wx},{wy}) ekf_xy=({ekf_x},{ekf_y}) anchor=({sx},{sy})',
            (
                f'yaw={self.format_yaw_deg(self.current_yaw)} '
                f'yaw_wheel={self.format_yaw_deg(self.current_wheel_yaw)} '
                f'yaw_imu={self.format_yaw_deg(self.imu_yaw)} '
                f'yaw_ekf={self.format_yaw_deg(self.current_odom_yaw)} '
                f'yaw_leg={self.format_yaw_deg(self.segment_heading)} '
                f'yaw_seg0={self.format_yaw_deg(self.segment_start_yaw)} '
                f'yaw_tgt={self.format_yaw_deg(self.segment_target_yaw)} '
                f'imu_off={imu_wheel_err_deg} '
                f'head_err={heading_err_deg} turn_err={turn_err_deg}'
            ),
            (
                f'wheel_v=({wvx},{wvy},{wwz}) ekf_v=({evx},{evy},{ewz}) '
                f'cmd_v=({self._fmt_num(self._last_cmd_linear)},{self._fmt_num(self._last_cmd_angular)}) '
                f'seg_v={self._fmt_num(seg_speed)}'
            ),
            (
                f'along={self._fmt_num(along)}/{self._fmt_num(target_m)}m '
                f'raw_along={self._fmt_num(raw_along)}m cross={cross_cm:+.1f}cm '
                f'dist_tol={self._fmt_num(self.distance_tolerance)}'
            ),
            (
                f'avoid={self._avoider.state_str} '
            ),
            (
                f'front={self.format_distance(self.front_obstacle_distance)}m '
                f'@ {self._fmt_num(self.front_obstacle_angle_deg, prec=1)}deg '
                f'left={self.format_distance(self.left_clearance_distance)}m '
                f'right={self.format_distance(self.right_clearance_distance)}m '
            ),
            (
                f'wheel_n={self._wheel_odom_msg_count} wheel_ready={int(self._wheel_odom_ready)} '
                f'frame={self.odom_frame_id}'
            ),
        ]
        return ' | '.join(parts)

    def _pose_diagnostic(self) -> str:
        return self._full_telemetry()

    def _maybe_log_telemetry(self, reason: str) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.mission_active:
            interval = self._telemetry_interval_sec
        elif self._wheel_odom_msg_count > 0:
            interval = 1.0
        else:
            return
        if now_sec - self._last_telemetry_sec < interval:
            return
        self._last_telemetry_sec = now_sec
        self._log_session('TELEM', f'{reason} | {self._full_telemetry()}')

    def _log_segment_enter(self, segment) -> None:
        if segment is None:
            return
        seg_type = segment.get('type', '?')
        desc = segment.get('description', '?')
        idx = self.plan_index
        parts = [f'#{idx} type={seg_type} desc={desc}']
        if seg_type == 'move':
            parts.append(f"L={float(segment.get('distance_m', 0.0)):.2f}m")
            parts.append(f"v={float(segment.get('speed', 0.0)):.2f}")
        elif seg_type == 'turn':
            parts.append(f"angle={float(segment.get('angle_deg', 0.0)):.0f}deg")
            if self.segment_target_yaw is not None:
                parts.append(
                    f"target_yaw={self.format_yaw_deg(self.segment_target_yaw)}deg"
                )
        elif seg_type == 'arc':
            parts.append(f"steering={float(segment.get('steering_angle_deg', 0.0)):+.1f}deg")
            parts.append(f"duration={float(segment.get('duration_sec', 0.0)):.2f}s")
        elif seg_type == 'pause':
            parts.append(f"duration={float(segment.get('duration', 0.0)):.2f}s")
        parts.append(self._pose_diagnostic())
        self._log_session('SEGMENT', ' '.join(parts))

    def _log_plan_summary(self, nav_succeeded: bool) -> None:
        lines = [f'nav_succeeded={nav_succeeded} 共{len(self.plan)}段:']
        for index, segment in enumerate(self.plan):
            seg_type = segment.get('type', '?')
            desc = segment.get('description', '?')
            if seg_type == 'move':
                lines.append(
                    f'  [{index}] move {desc} '
                    f'L={float(segment.get("distance_m", 0.0)):.2f}m '
                    f'v={float(segment.get("speed", 0.0)):.2f} '
                    f'detour={bool(segment.get("allow_detour", True))}'
                )
            elif seg_type == 'turn':
                lines.append(
                    f'  [{index}] turn {desc} '
                    f'{float(segment.get("angle_deg", 0.0)):.0f}deg'
                )
            elif seg_type == 'arc':
                lines.append(
                    f'  [{index}] arc {desc} '
                    f'steering={float(segment.get("steering_angle_deg", 0.0)):+.1f}deg '
                    f't={float(segment.get("duration_sec", 0.0)):.2f}s'
                )
            elif seg_type == 'pause':
                lines.append(
                    f'  [{index}] pause {desc} '
                    f'{float(segment.get("duration", 0.0)):.2f}s'
                )
            else:
                lines.append(f'  [{index}] {seg_type} {desc}')
        self._log_session('PLAN', '\n'.join(lines))

    def _setup_wheel_odom_position(self) -> None:
        """位姿/航向/计程/控制统一用轮速 /odom；EKF/IMU 仅诊断。"""
        self.declare_parameter('navigation_pose_source', 'wheel')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('wheel_odom_warmup_sec', 0.40)
        self.declare_parameter('wheel_odom_warmup_min_msgs', 5)
        self._navigation_pose_source = (
            str(self.get_parameter('navigation_pose_source').value).strip().lower() or 'wheel'
        )
        self._wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value).strip()
        self._wheel_odom_warmup_sec = max(
            0.0, float(self.get_parameter('wheel_odom_warmup_sec').value)
        )
        self._wheel_odom_warmup_min_msgs = max(
            1, int(self.get_parameter('wheel_odom_warmup_min_msgs').value)
        )
        self._wheel_odom_ready = False
        self._wheel_odom_msg_count = 0
        self._wheel_odom_first_rx_sec = None
        self.current_wheel_yaw = None
        self.imu_yaw = None
        self._use_wheel_odom_for_distance = bool(
            self._wheel_odom_topic and self._wheel_odom_topic != self.odom_topic
        )
        if self._use_wheel_odom_for_distance:
            self.create_subscription(
                Odometry, self._wheel_odom_topic, self._wheel_odom_callback, 10
            )
            self.get_logger().info(
                f'{self.test_feedback_prefix}统一位姿源={self._navigation_pose_source} '
                f'topic={self._wheel_odom_topic} '
                f'(xy+yaw+计程+控制同源; IMU/EKF {self.odom_topic} 仅日志; '
                f'warmup {self._wheel_odom_warmup_sec:.2f}s×'
                f'{self._wheel_odom_warmup_min_msgs}条)'
            )

    def _wheel_odom_warmed_up(self) -> bool:
        if not self._use_wheel_odom_for_distance:
            return self.current_position is not None
        if self._wheel_odom_first_rx_sec is None:
            return False
        if self._wheel_odom_msg_count < self._wheel_odom_warmup_min_msgs:
            return False
        now_sec = self.get_clock().now().nanoseconds / 1e9
        return (now_sec - self._wheel_odom_first_rx_sec) >= self._wheel_odom_warmup_sec

    def _wheel_odom_callback(self, msg: Odometry) -> None:
        self._last_wheel_odom_msg = msg  # 保存用于 marker 坐标转换
        position = msg.pose.pose.position
        twist = msg.twist.twist
        self.current_position = (float(position.x), float(position.y))
        self.current_wheel_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self._wheel_twist = (
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.angular.z),
        )
        self._sync_unified_pose_from_wheel()
        self._wheel_odom_msg_count += 1
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self._wheel_odom_first_rx_sec is None:
            self._wheel_odom_first_rx_sec = now_sec
            self._log_session(
                'ODOM_WHEEL',
                f'首条 {self._wheel_odom_topic} '
                f'pos=({position.x:.3f},{position.y:.3f}) '
                f'yaw={self.format_yaw_deg(self.current_wheel_yaw)}deg '
                f'v=({twist.linear.x:.3f},{twist.linear.y:.3f},{twist.angular.z:.3f})',
            )
        was_ready = self._wheel_odom_ready
        if self._wheel_odom_warmed_up():
            self._wheel_odom_ready = True
        if self._wheel_odom_ready and not was_ready and not self._wheel_warmup_logged:
            self._wheel_warmup_logged = True
            elapsed = now_sec - self._wheel_odom_first_rx_sec
            self._log_session(
                'ODOM_WHEEL',
                f'warmup 完成 msgs={self._wheel_odom_msg_count} '
                f'elapsed={elapsed:.2f}s | {self._full_telemetry()}',
            )
        self._maybe_log_telemetry('wheel_odom')
        if self.waiting_for_phase2_start:
            self.try_start_mission()

    def odom_callback(self, msg):
        ekf_pos = msg.pose.pose.position
        ekf_twist = msg.twist.twist
        self._last_ekf_position = (float(ekf_pos.x), float(ekf_pos.y))
        self._ekf_twist = (
            float(ekf_twist.linear.x),
            float(ekf_twist.linear.y),
            float(ekf_twist.angular.z),
        )
        self.current_odom_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_frame_id = msg.header.frame_id or self.odom_frame_id
        self._update_track_pose_from_odom_combined()
        
        if not self._wheel_pose_source_active() or not self._wheel_odom_ready:
            self.current_position = self._last_ekf_position
        if self.waiting_for_phase2_start:
            self.try_start_mission()

    def _update_track_pose_from_odom_combined(self):
        """Integrate EKF distance in the IMU heading frame for track control."""
        if (
            not self._track_mission_active
            or self._last_ekf_position is None
            or self.current_yaw is None
        ):
            return
        self._track_pose = self._track_pose_integrator.update(
            self._last_ekf_position, self.current_yaw
        )
        self._track_distance_m = self._track_pose_integrator.total_distance_m

    def projected_distance(self):
        """使用 odom 帧的欧氏距离判断直行完成，避免航向偏差导致投影错误。
        
        改用累计行驶距离（直线距离）代替方向投影：
        - 不依赖航向对齐
        - 简单鲁棒
        - 保留横偏修正保证轨迹直线性
        """
        if (
            self.segment_start_pose is None
            or self.current_position is None
        ):
            return 0.0
        
        # 直接用 odom 帧（current_position）计算欧氏距离
        dx = self.current_position[0] - self.segment_start_pose[0]
        dy = self.current_position[1] - self.segment_start_pose[1]
        distance = math.hypot(dx, dy)
        
        return max(0.0, distance)

    def _unify_segment_pose(self, segment) -> None:
        """段起点/航向/计程轴与 current_yaw(current_position) 完全对齐。
        
        如果 segment 包含 coordinate_system='world'，则使用 YAML 中的绝对坐标。
        """
        if not segment or self.current_yaw is None:
            return
        yaw = self.normalize_angle(self.current_yaw)
        seg_type = segment.get('type')
        is_world = (segment.get('coordinate_system') == 'world')

        if seg_type == 'turn':
            if 'force_start_yaw' in segment:
                self.segment_start_yaw = self.normalize_angle(
                    float(segment['force_start_yaw'])
                )
            else:
                self.segment_start_yaw = yaw
            
            # 统一使用增量角度 angle_deg，简单可靠
            angle_deg = float(segment.get('angle_deg', 0.0))
            # 应用系统性转角补偿（IMU/机械零点偏差）
            angle_compensation_deg = float(self.get_parameter('turn_angle_compensation_deg').value)
            compensated_angle_deg = angle_deg + angle_compensation_deg
            self.segment_target_yaw = self.normalize_angle(
                self.segment_start_yaw + math.radians(compensated_angle_deg)
            )
            self._last_turn_target_yaw = self.segment_target_yaw  # 存给下一个直行段做 heading 基准
            
            self.get_logger().info(
                f'[TURN] start={math.degrees(self.segment_start_yaw):.1f}deg '
                f'angle_deg={angle_deg:.1f}deg comp={angle_compensation_deg:+.1f}deg -> '
                f'target={math.degrees(self.segment_target_yaw):.1f}deg'
            )
            return

        if seg_type == 'arc':
            if self.current_position is not None:
                self.segment_start_pose = self.current_position
            self._last_turn_target_yaw = None
            self._arc_timer_last_log_sec = -1.0
            self._log_session(
                'ARC_SETUP',
                f'steering={float(segment.get("steering_angle_deg", 0.0)):+.1f}deg '
                f'duration={float(segment.get("duration_sec", 0.0)):.2f}s '
                f'linear={float(self.turn_linear_speed):.3f}m/s'
            )
            return

        if seg_type != 'move':
            return

        # 统一用 odom 相对坐标 + 欧氏距离完成直行
        # 航向从 YAML 的 heading_deg（如果有）或当前 yaw 继承
        self._segment_is_world = False
        if self.current_position is not None:
            self.segment_start_pose = self.current_position
        # 保存轮速航向（与 odom 位置同坐标系，用于 cross-track 计算）
        self._segment_start_wheel_yaw = self.current_wheel_yaw if self.current_wheel_yaw is not None else yaw
        
        # 优先用 YAML 的 heading_deg（正交矩形边方向）
        if 'heading_deg' in segment:
            heading = self.normalize_angle(math.radians(float(segment['heading_deg'])))
            self.segment_heading = heading
            self.segment_start_yaw = heading
        elif 'force_segment_heading' in segment:
            heading = self.normalize_angle(float(segment['force_segment_heading']))
            self.segment_heading = heading
            self.segment_start_yaw = heading
        else:
            # 优先用上一个转弯的目标 yaw，确保直行朝正确方向修正
            if self._last_turn_target_yaw is not None:
                heading = self.normalize_angle(self._last_turn_target_yaw)
            else:
                heading = yaw
            self.segment_heading = heading
            self.segment_start_yaw = heading

        if self.segment_start_pose is None:
            return
        x0, y0 = self.segment_start_pose
        anchor_line = (
            f'desc={segment.get("description", "?")} '
            f'start=({x0:.3f},{y0:.3f}) yaw={self.format_yaw_deg(yaw)}deg '
            f'yaw_imu={self.format_yaw_deg(self.imu_yaw)}deg '
            f'L={float(segment.get("distance_m", 0.0)):.2f}m'
        )
        self.get_logger().info(f'{self.test_feedback_prefix}里程锚点: {anchor_line}')
        self._log_session('ODOM_ANCHOR', anchor_line)

    def _missing_pose_inputs(self):
        missing = []
        if self._wheel_pose_source_active():
            if not self._wheel_odom_warmed_up():
                missing.append(
                    f'wheel_odom({self._wheel_odom_topic} '
                    f'{self._wheel_odom_msg_count}/{self._wheel_odom_warmup_min_msgs})'
                )
            elif self.current_yaw is None:
                missing.append('wheel_yaw')
        else:
            if self.current_position is None:
                missing.append(str(self.odom_topic))
            if self.current_yaw is None:
                missing.append('imu')
        return missing

    def resolve_test_direction(self, raw_value):
        normalized = str(raw_value).strip().lower()
        if normalized in ('clockwise', 'cw', '顺时针'):
            return 'clockwise'
        if normalized in ('counterclockwise', 'ccw', 'anticlockwise', 'anti-clockwise', '逆时针'):
            return 'counterclockwise'

        parsed = self.parse_direction(str(raw_value).strip())
        if parsed is not None:
            return parsed

        self.get_logger().warning(
            f'无法识别测试方向 "{raw_value}"，回退到顺时针'
        )
        return 'clockwise'

    def direction_text(self):
        direction = self.direction or self.test_direction or 'clockwise'
        return '顺时针' if direction == 'clockwise' else '逆时针'

    def nav_succeeded_for_test_start(self):
        if self.test_start_mode in ('after_corridor', 'nav_succeeded', 'corridor', 'true'):
            return True
        if self.test_start_mode in ('full_entry', 'pre_loop', 'nav_failed', 'false'):
            return False
        return bool(self.use_corridor_path)

    def _stage2_start_is_stationary(self):
        """Do not arm Stage2 while another velocity source moves the base."""
        twist = getattr(self, '_wheel_twist', None)
        if twist is None:
            # 官方 Stage2 不再强制等待 /odom；无轮速 twist 时由 Stage1 phase 切换
            # 和启动延迟保证交接，不阻塞 /odom_combined + IMU 的正常启动。
            return True
        speed = math.hypot(float(twist[0]), float(twist[1]))
        yaw_rate = abs(float(twist[2]))
        speed_limit = float(self.get_parameter('start_stationary_speed_mps').value)
        yaw_limit = float(self.get_parameter('start_stationary_yaw_rate_rps').value)
        now = self.get_clock().now().nanoseconds / 1e9
        if speed > speed_limit or yaw_rate > yaw_limit:
            self._stationary_since = None
            if now - self._last_stationary_log > 1.0:
                self._last_stationary_log = now
                self._log_session(
                    'START_BLOCKED',
                    f'底盘未静止 speed={speed:.3f} yaw_rate={yaw_rate:.3f}',
                )
            return False
        if self._stationary_since is None:
            self._stationary_since = now
            return float(self.get_parameter('start_stationary_hold_sec').value) <= 0.0
        return now - self._stationary_since >= float(
            self.get_parameter('start_stationary_hold_sec').value
        )

    def _start_track_mission(self):
        if self._last_ekf_position is None or self.current_yaw is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        self._track_pose_integrator.reset(self._last_ekf_position, self.current_yaw)
        self._track_pose = self._track_pose_integrator.pose
        self._track_distance_m = self._track_pose_integrator.total_distance_m
        self._track_controller.start(self.direction, self._track_pose,
                                     self.current_yaw, now,
                                     distance_m=self._track_distance_m)
        self._stage2_ai_capture_sent = False
        self._track_mission_active = True
        self.mission_active = True
        self.current_segment = {'type': 'track', 'description': 'rounded_track'}
        self.publish_state('running')
        self._log_session(
            'TRACK_START',
            f'direction={self.direction} local=(0.000,0.000) '
            f'odom_combined=({self._last_ekf_position[0]:.3f},'
            f'{self._last_ekf_position[1]:.3f}) '
            f'yaw_imu={math.degrees(self.current_yaw):.1f}',
        )
        self.publish_feedback(
            f'{self.test_feedback_prefix}圆角轨迹闭环启动，方向: {self.direction_text()}'
        )
        return True

    def _maybe_trigger_stage2_ai_capture(self) -> None:
        if not self._stage2_ai_capture_enabled or self._stage2_ai_capture_sent:
            return
        if self._track_controller.active_segment_name != 'top_long':
            return
        target = self._track_controller.active_segment_target_m
        progress = self._track_controller.active_segment_progress_m
        remaining = max(0.0, target - progress)
        if remaining > self._stage2_ai_capture_lead_m:
            return

        self._stage2_ai_capture_sent = True
        self._stage2_ai_trigger_pub.publish(Empty())
        message = (
            f'top_long progress={progress:.3f}/{target:.3f}m '
            f'remaining={remaining:.3f}m direction={self.direction}'
        )
        self.get_logger().info(f'[AI_CAPTURE] trigger published: {message}')
        self._log_session('AI_CAPTURE_TRIGGER', message)

    def _run_track_controller(self):
        now = self.get_clock().now().nanoseconds / 1e9
        visual = None
        if self._track_pose is None or self.current_yaw is None:
            stop_cmd = self.create_twist()
            self._log_control_timing(now, 'track_missing_pose')
            self.cmd_pub.publish(stop_cmd)
            return
        if self.front_obstacle_distance < float(
                self.get_parameter('turn_obstacle_stop_m').value):
            command = self._track_controller.safe_stop('front_obstacle')
        else:
            self._maybe_trigger_stage2_ai_capture()
            visual = self._get_vision_line_status() if getattr(
                self, '_vision_node', None) is not None else None
            track_map_xy = self._track_map_position()
            handoff_reached = (
                self._track_controller.active_segment_name == 'stage3_handoff_line'
                and track_map_xy is not None
                and track_map_xy[1] < self._track_stage3_handoff_map_y
            )
            command = self._track_controller.step(
                now, self._track_pose, self.current_yaw, visual,
                yaw_rate=getattr(self, 'current_imu_yaw_rate', 0.0),
                distance_m=self._track_distance_m,
                stage3_handoff_reached=handoff_reached,
            )
        cmd_msg = self.create_twist(command.linear, command.angular)
        self._log_control_timing(now, f'track:{command.segment or command.state}')
        self.cmd_pub.publish(cmd_msg)
        if visual and bool(visual.get('valid', False)):
            vision_log = (f'valid e={float(visual.get("error", 0.0) or 0.0):+.3f} '
                          f'c={float(visual.get("confidence", 0.0) or 0.0):.2f} '
                          f'age={float(visual.get("age", 999.0) or 999.0):.2f}')
        else:
            vision_log = 'invalid'
        self._log_session(
            'TRACK_CTRL',
            f'state={command.state} v={command.linear:.3f} w={command.angular:.3f} '
            f'progress={command.progress_m:.3f} cross={command.cross_track_m:.3f} '
            f'head={math.degrees(command.heading_error_rad):.1f} '
            f'target_v={command.target_speed:.3f} '
            f'segment={command.segment} '
            f'seg_s={command.segment_progress_m:.3f}/{command.segment_target_m:.3f} '
            f'turn={math.degrees(command.turn_progress_rad):.1f}/'
            f'{math.degrees(command.turn_target_rad):.1f} '
            f'entry_boundary={command.entry_boundary_trigger or "none"} '
            f'entry_guard={command.entry_boundary_window_min_m:.3f}/'
            f'{command.entry_boundary_window_max_m:.3f} '
            f'entry_top_y={command.entry_boundary_top_y_ratio:.3f} '
            f'entry_angle={command.entry_boundary_angle_deg:.1f} '
            f'entry_confirm={command.entry_boundary_confirm_frames} '
            f'imu_xy=({self._track_pose[0]:.3f},{self._track_pose[1]:.3f}) '
            f'ekf_s={self._track_distance_m:.3f} '
            f'imu_w={getattr(self, "current_imu_yaw_rate", 0.0):.3f} '
            f'vision={vision_log}',
        )
        if command.safe_stop:
            self._log_session('TRACK_SAFE_STOP', command.reason)
            self.publish_feedback(
                f'{self.test_feedback_prefix}轨迹控制停车: {command.reason}'
            )
            self._track_mission_active = False
            self.mission_active = False
        elif command.complete:
            if not self._publish_stage3_entry_anchor():
                return
            self._track_mission_active = False
            self.finish_mission()

    def _track_map_position(self):
        """Return the current map <- base_footprint translation, or None."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame_id,
                self._stage3_entry_anchor_base_frame,
                Time(),
            )
            translation = transform.transform.translation
            return float(translation.x), float(translation.y)
        except TransformException:
            return None

    def _publish_stage3_entry_anchor(self):
        map_xy = self._track_map_position()
        if map_xy is None:
            self.get_logger().warning(
                '[STAGE3_HANDOFF] waiting for map<-base_footprint TF before publishing entry anchor'
            )
            return False
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = map_xy[0]
        msg.point.y = map_xy[1]
        self._stage3_anchor_pub.publish(msg)
        self._log_session(
            'STAGE3_HANDOFF',
            f'map=({map_xy[0]:.3f},{map_xy[1]:.3f}) threshold_y='
            f'{self._track_stage3_handoff_map_y:.3f} source=tf_map',
        )
        self.get_logger().info(
            f'[STAGE3_HANDOFF] map anchor=({map_xy[0]:.2f},{map_xy[1]:.2f}), '
            f'y<{self._track_stage3_handoff_map_y:.2f}, source=tf_map'
        )
        return True

    def start_mode_text(self):
        if self.nav_succeeded_for_test_start():
            return '按比赛到达通道口后的惯导入口开始'
        return '按比赛未经过通道口时的完整入环动作开始'

    def format_distance(self, value):
        if not math.isfinite(value):
            return 'inf'
        return f'{value:.2f}'

    def format_yaw_deg(self, yaw):
        if yaw is None or not math.isfinite(yaw):
            return 'nan'
        return f'{math.degrees(self.normalize_angle(yaw)):.1f}'

    # ─── 段控制覆盖 ───────────────────────────────────────────────

    def begin_inertial_plan_after_nav(self, nav_succeeded):
        self._sync_unified_pose_from_wheel()
        super().begin_inertial_plan_after_nav(nav_succeeded)
        self._log_plan_summary(nav_succeeded)

    def reset_mission(self, clear_task):
        super().reset_mission(clear_task)

    def rectangle_segment_label(self, segment):
        description = str(segment.get('description', 'unknown'))

        if self.direction == 'clockwise':
            d = segment.get('distance_m', 0)
            labels = {
                'rect_enter_align': '通道后起点入口对齐',
                'rect_first_leg': f'底边向左 {d:.2f}m 段',
                'rect_corner_1': '左下拐角',
                'rect_side_1': f'左边向上 {d:.2f}m 段',
                'rect_corner_2': '左上拐角',
                'rect_top': f'顶边向右 {d:.2f}m 段',
                'rect_corner_3': '右上拐角',
                'rect_side_2': f'右边向下 {d:.2f}m 段',
                'rect_corner_4': '右下拐角',
                'rect_return_origin': f'底边回起点 {d:.2f}m 段',
            }
        else:
            d = segment.get('distance_m', 0)
            labels = {
                'rect_enter_align': '通道后起点入口对齐',
                'rect_first_leg': f'底边向右 {d:.2f}m 段',
                'rect_corner_1': '右下拐角',
                'rect_side_1': f'右边向上 {d:.2f}m 段',
                'rect_corner_2': '右上拐角',
                'rect_top': f'顶边向左 {d:.2f}m 段',
                'rect_corner_3': '左上拐角',
                'rect_side_2': f'左边向下 {d:.2f}m 段',
                'rect_corner_4': '左下拐角',
                'rect_return_origin': f'底边回起点 {d:.2f}m 段',
            }
        return labels.get(description, description)

    def start_segment(self, index):
        super().start_segment(index)
        self.last_progress_bucket = -1
        self.active_turn_heading_tolerance = self.heading_tolerance
        self._pure_linear_after_avoid = False  # 新段恢复惯导全控制
        self._avoider.reset()
        if hasattr(self, '_reset_vision_length_state'):
            self._reset_vision_length_state()

        if self.current_segment is None or self.plan_index != index:
            return

        segment = self.current_segment
        segment_type = segment.get('type')
        self._sync_unified_pose_from_wheel()
        if segment_type == 'turn' and 'heading_tolerance_rad' in segment:
            self.active_turn_heading_tolerance = max(
                1e-3, float(segment['heading_tolerance_rad'])
            )
        self._unify_segment_pose(segment)

        self._log_segment_enter(segment)

        label = self.rectangle_segment_label(segment)

        if segment_type == 'turn':
            # 计算实际转弯角度（基于 start_yaw 和 target_yaw）
            if hasattr(self, 'segment_start_yaw') and hasattr(self, 'segment_target_yaw') and \
               self.segment_start_yaw is not None and self.segment_target_yaw is not None:
                actual_turn_rad = self.angle_error(self.segment_target_yaw, self.segment_start_yaw)
                actual_turn_deg = math.degrees(actual_turn_rad)
                turn_text = '左转' if actual_turn_deg > 0.0 else '右转'
                self.publish_feedback(
                    f'{self.test_feedback_prefix}当前位置: {label}，开始{turn_text} {abs(actual_turn_deg):.0f} 度'
                )
            else:
                angle_deg = float(segment.get('angle_deg', 0.0))
                turn_text = '左转' if angle_deg > 0.0 else '右转'
                self.publish_feedback(
                    f'{self.test_feedback_prefix}当前位置: {label}，开始{turn_text} {abs(angle_deg):.0f} 度'
                )
            return

        if segment_type == 'move':
            distance_m = float(segment.get('distance_m', 0.0))
            self.publish_feedback(
                f'{self.test_feedback_prefix}当前位置: {label}，目标直行 {distance_m:.2f}m'
            )
            return

        if segment_type == 'arc':
            steering_angle_deg = float(segment.get('steering_angle_deg', 0.0))
            duration_sec = float(segment.get('duration_sec', 0.0))
            turn_text = '左打舵' if steering_angle_deg > 0.0 else '右打舵'
            self.publish_feedback(
                f'{self.test_feedback_prefix}当前位置: {label}，开始{turn_text} '
                f'{abs(steering_angle_deg):.0f}°，持续{duration_sec:.1f}s'
            )
            return

        if segment_type == 'pause':
            self.publish_feedback(f'{self.test_feedback_prefix}当前位置: {label}，短暂停稳')

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_frame_id = msg.header.frame_id
        data = self._scan_processor.process(msg)
        self.front_obstacle_distance = data.front_distance
        self.front_obstacle_angle_deg = data.front_angle_deg
        self.left_clearance_distance = data.left_clearance
        self.right_clearance_distance = data.right_clearance
        self._avoider.on_scan(
            data.front_distance, data.front_angle_deg,
            data.left_clearance, data.left_angle_deg,
            data.right_clearance, data.right_angle_deg,
        )

        # 新增：聚类可视化（前方 0.3-2.5m，左右 ±0.5m 窗口）
        try:
            self.all_clusters = self._scan_processor.cluster_obstacles_in_window(
                msg,
                min_x=self._cluster_window_config['min_x'],
                max_x=self._cluster_window_config['max_x'],
                half_y=self._cluster_window_config['half_y'],
                gap_tolerance=self._cluster_window_config['gap_tolerance']
            )
            n_clusters = len(self.all_clusters)
            total_points = sum(len(c) for c in self.all_clusters) if self.all_clusters else 0
            
            # # 打印到日志文件（用 _log_session 才能写入 RacingLogger 的文件）
            # self._log_session('CLUSTER_VIZ',
            #     f'clusters={n_clusters} points={total_points} '
            #     f'window=[{self._cluster_window_config["min_x"]:.2f},{self._cluster_window_config["max_x"]:.2f}]×±{self._cluster_window_config["half_y"]:.2f}m')
            
            # 每次扫描都发布，无聚类时自动清理旧 marker
            self.obstacle_markers.frame_id = msg.header.frame_id
            self.obstacle_markers.publish_from_clusters(
                self.all_clusters or [], color='red'
            )
            # self._log_session('MARKER_VIZ',
            #     f'markers={n_clusters} frame={msg.header.frame_id}')
        except Exception as e:
            self._log_session('CLUSTER_VIZ', f'ERROR: {e}')
            self.get_logger().warn(f'Obstacle visualization error: {e}', throttle_duration_sec=5.0)

    def _compute_move_lateral_angular(self) -> float:
        """计算直行段横向角速度（覆盖父类，使用视觉优先逻辑）"""
        return self._compute_move_lateral_angular_with_vision()

    def run_move_segment(self):
        if self.current_segment is not None and self.current_segment.get('type') == 'move':
            nominal_target = max(1e-6, float(self.current_segment.get('distance_m', 0.0)))
            progress_raw = self.projected_distance()
            if hasattr(self, '_vision_adjusted_move_target'):
                target_distance, vis_rem, free_ratio, vis_valid, vis_reason = (
                    self._vision_adjusted_move_target(nominal_target, progress_raw)
                )
            else:
                target_distance, vis_rem, free_ratio, vis_valid, vis_reason = (
                    nominal_target, None, 0.0, False, 'no_mixin'
                )
            progress = max(0.0, min(progress_raw, target_distance))
            ratio = progress / max(1e-6, target_distance)
            bucket = -1
            if ratio >= 0.75:
                bucket = 3
            elif ratio >= 0.50:
                bucket = 2
            elif ratio >= 0.25:
                bucket = 1

            if bucket > self.last_progress_bucket:
                self.last_progress_bucket = bucket
                if bucket >= 0:
                    label = self.rectangle_segment_label(self.current_segment)
                    rem_txt = f'{vis_rem:.2f}m' if vis_rem is not None else 'N/A'
                    progress_line = (
                        f'{label} 进度 {bucket * 25}% '
                        f'({progress:.2f}/{target_distance:.2f}m'
                        f' nom={nominal_target:.2f}m vis_rem={rem_txt})'
                    )
                    self.get_logger().info(
                        f'{self.test_feedback_prefix}当前位置: {progress_line}'
                    )
                    self._log_session(
                        'PROGRESS',
                        f'{progress_line} | valid={vis_valid} reason={vis_reason} '
                        f'free={free_ratio:.2f} | {self._pose_diagnostic()}',
                    )

        # 优先：段完成检查（放在避障之前，确保段不会被避障阻塞）
        if self.current_position is not None and self.segment_heading is not None:
            progress = self.projected_distance()
            if (self.current_segment is not None
                    and self.current_segment.get('type') == 'move'):
                nominal_target = max(1e-6, float(self.current_segment.get('distance_m', 0.0)))
                if hasattr(self, '_vision_move_should_finish'):
                    should_finish, target_distance, vis_rem, free_ratio, finish_reason = (
                        self._vision_move_should_finish(
                            nominal_target, progress, self.distance_tolerance
                        )
                    )
                else:
                    target_distance = nominal_target
                    should_finish = progress >= target_distance - self.distance_tolerance
                    vis_rem, free_ratio, finish_reason = None, 0.0, 'odom_only'

                if should_finish:
                    # 避障进行中 → 延迟切段，等避障完成
                    if self._avoider.is_active:
                        nav = NavState(
                            position=self.current_position,
                            yaw=self.navigation_yaw(),
                            segment_heading=self.segment_heading,
                            segment_start_pose=self.segment_start_pose,
                            current_segment=self.current_segment,
                            projected_distance=self.projected_distance(),
                        )
                        self._avoider.step(nav)
                        return
                    if self.last_progress_bucket < 4:
                        self.last_progress_bucket = 4
                        self.publish_feedback(
                            f'{self.test_feedback_prefix}当前位置: '
                            f'{self.rectangle_segment_label(self.current_segment)}，'
                            f'直行到位，准备切换到下一段'
                        )
                    rem_txt = f'{vis_rem:.2f}m' if vis_rem is not None else 'N/A'
                    done_desc = str(self.current_segment.get('description', '?'))
                    if done_desc == 'entry_short_straight':
                        done_msg = '短直道完成，准备进入半圆'
                    elif done_desc == 'rect_top':
                        done_msg = '长直道完成，准备进入下一段'
                    else:
                        done_msg = f'{done_desc} 直行完成'
                    self.get_logger().info(
                        f'[SEGMENT_DONE] {done_msg}: '
                        f'{progress:.2f}/{target_distance:.2f}m '
                        f'v={float(self.current_segment.get("speed", self.corridor_linear_speed)):.2f}m/s'
                    )
                    self._log_session(
                        'SEGMENT_DONE',
                        f'{self.current_segment.get("description", "?")} '
                        f'{progress:.3f}/{target_distance:.2f}m '
                        f'nom={nominal_target:.2f}m vis_rem={rem_txt} '
                        f'free={free_ratio:.2f} reason={finish_reason} | '
                        f'{self._pose_diagnostic()}',
                    )
                    self.cmd_pub.publish(self.create_twist())
                    self.start_segment(self.plan_index + 1)
                    return

        # ── 接近拐角检测：段末尾切换转弯障碍检测 ──
        if self.current_position is not None and self.segment_heading is not None and self.current_segment is not None:
            nominal_target = max(1e-6, float(self.current_segment.get('distance_m', 0.0)))
            progress = self.projected_distance()
            if hasattr(self, '_vision_adjusted_move_target'):
                target_distance, _, _, _, _ = self._vision_adjusted_move_target(
                    nominal_target, progress
                )
            else:
                target_distance = nominal_target
            remaining = target_distance - progress
            corner_approach = float(self.get_parameter('corner_approach_m').value)
            if remaining <= corner_approach:
                # 接近拐角：用 turn_obstacle_stop_m，避免雷达扫边误触发
                if math.isfinite(self.front_obstacle_distance) and self.front_obstacle_distance < float(self.get_parameter('turn_obstacle_stop_m').value):
                    angular = self._compute_move_lateral_angular()
                    self.cmd_pub.publish(self.create_twist(float(self.get_parameter('turn_obstacle_creep_speed').value), angular))
                    self._maybe_log_telemetry('corner_approach')
                    return
                # 前方空间够，不进避障，正常完成段
            else:
                # 正常避障：每帧 step 以支持触发；仅 active→idle 记一次完成
                was_avoiding = bool(self._avoider.is_active)
                nav = NavState(
                    position=self.current_position,
                    yaw=self.navigation_yaw(),
                    segment_heading=self.segment_heading,
                    segment_start_pose=self.segment_start_pose,
                    current_segment=self.current_segment,
                    projected_distance=self.projected_distance(),
                )
                if self._avoider.step(nav):
                    return
                if was_avoiding and not self._avoider.is_active:
                    self._pure_linear_after_avoid = True
                    self._log_session(
                        'AVOID',
                        f'避障完成，恢复段控制 | {self._pose_diagnostic()}'
                    )

        if self.current_position is None or self.segment_heading is None:
            self.cmd_pub.publish(self.create_twist())
            self._maybe_log_telemetry('move_no_pose')
            return

        # 避障完成后继续混合纠偏（IMU 主 + 视觉辅），不再锁死纯直线
        if self._pure_linear_after_avoid:
            self._pure_linear_after_avoid = False

        angular = self._compute_move_lateral_angular()
        linear = float(self.current_segment.get('speed', self.corridor_linear_speed))
        
        # === 加速渐变逻辑（转弯后平滑过渡）===
        if self._just_finished_turn:
            import time
            elapsed = time.time() - self._turn_finish_time
            
            if elapsed < self._accel_ramp_duration:
                # 渐变中：线性插值 turn_speed → target_speed
                ratio = elapsed / self._accel_ramp_duration
                turn_speed = float(self.get_parameter('turn_linear_speed').value)
                linear_ramped = linear * ratio + turn_speed * (1.0 - ratio)
                
                # 日志记录渐变过程（首次 + 25%/50%/75%）
                progress_pct = int(ratio * 100)
                if progress_pct >= self._last_ramp_pct + 25:
                    self._last_ramp_pct = (progress_pct // 25) * 25
                    self._log_session(
                        'ACCEL_RAMP',
                        f'{progress_pct}% | v={linear_ramped:.3f} '
                        f'(target={linear:.3f}) | elapsed={elapsed:.2f}s'
                    )
                
                linear = linear_ramped
            else:
                # 渐变完成，重置标志
                if self._just_finished_turn:  # 仅记录一次
                    self._log_session(
                        'ACCEL_RAMP',
                        f'100% 渐变完成 | v={linear:.3f} | elapsed={elapsed:.2f}s'
                    )
                self._just_finished_turn = False
                self._last_ramp_pct = -1
        
        self.cmd_pub.publish(self.create_twist(linear, angular))
        self._maybe_log_telemetry('move')

    def run_arc_segment(self):
        """Hold a fixed steering angle, then return the steering to center."""
        segment = self.current_segment or {}
        if self.segment_started_at is None:
            self.cmd_pub.publish(self.create_twist())
            self._maybe_log_telemetry('arc_no_pose')
            return

        steering_angle_deg = float(segment.get('steering_angle_deg', 0.0))
        duration_sec = max(0.01, float(segment.get('duration_sec', 0.0)))
        elapsed = max(0.0, self.get_clock().now().nanoseconds / 1e9 - self.segment_started_at)
        progress = min(1.0, elapsed / duration_sec)
        linear = float(segment.get('speed', segment.get('turn_linear_speed', self.turn_linear_speed)))
        # angular.z is reused here as the chassis steering-angle channel.
        # The production launch/direct relay passes this value to the chassis
        # unchanged, so the YAML degree value is intentionally sent as-is.
        # It is not a body yaw rate and must not be converted to rad/s/radians.
        steering_cmd = steering_angle_deg

        log_bucket = min(4, int(elapsed / max(duration_sec / 4.0, 0.25)))
        if log_bucket > getattr(self, '_arc_timer_last_log_sec', -1):
            self._arc_timer_last_log_sec = log_bucket
            self._log_session(
                'ARC_TIMER',
                f'{self.rectangle_segment_label(segment)} '
                f'elapsed={elapsed:.2f}/{duration_sec:.2f}s '
                f'progress={progress * 100:.0f}% steering={steering_angle_deg:+.1f}deg '
                f'steering_cmd={steering_cmd:+.1f}deg linear={linear:.3f}m/s'
            )

        if elapsed >= duration_sec:
            self._just_finished_turn = True
            self._turn_finish_time = self.get_clock().now().nanoseconds / 1e9
            self.cmd_pub.publish(self.create_twist(linear, 0.0))
            desc = str(segment.get('description', '?'))
            if desc == 'entry_45_arc':
                done_msg = '入口45°圆弧完成，准备进入短直道'
            elif desc == 'entry_semicircle':
                done_msg = '入口半圆完成，准备进入长直道'
            else:
                done_msg = f'{desc} 圆弧完成'
            self.get_logger().info(
                f'[SEGMENT_DONE] {done_msg}: '
                f'steering={steering_angle_deg:+.1f}deg '
                f't={duration_sec:.2f}s v={linear:.2f}m/s'
            )
            self._log_session(
                'ARC_COMPLETE',
                f'steering={steering_angle_deg:+.1f}deg duration={duration_sec:.2f}s '
                f'recenter=0.0deg linear={linear:.3f}m/s | {self._pose_diagnostic()}'
            )
            self.publish_feedback(
                f'{self.test_feedback_prefix}当前位置: '
                f'{self.rectangle_segment_label(segment)}，圆弧完成，进入下一段'
            )
            self.start_segment(self.plan_index + 1)
            return

        self.cmd_pub.publish(self.create_twist(linear, steering_cmd))
        self._maybe_log_telemetry(
            f'arc_timer elapsed={elapsed:.2f}/{duration_sec:.2f}s '
            f'steering={steering_angle_deg:+.1f}deg steering_cmd={steering_cmd:+.1f}deg '
            f'linear={linear:.3f}m/s'
        )

    def run_turn_segment(self):
        turn_tolerance = self.active_turn_heading_tolerance
        linear_speed = float(
            (self.current_segment or {}).get('turn_linear_speed', self.turn_linear_speed)
        )

        # 转角障碍检测：前方过近 → 蠕行转弯
        # 用 turn_obstacle_stop_m(0.25m) 替代 detour_obstacle_distance(0.48m)
        # 避免雷达扫到赛道边角误触发，同时防止电机停转
        if math.isfinite(self.front_obstacle_distance) and self.front_obstacle_distance < float(self.get_parameter('turn_obstacle_stop_m').value):
            linear_speed = float(self.get_parameter('turn_obstacle_creep_speed').value)

        nav_yaw = self.navigation_yaw()
        if nav_yaw is None or self.segment_target_yaw is None:
            self.cmd_pub.publish(self.create_twist())
            return

        error = self.angle_error(self.segment_target_yaw, nav_yaw)

        # 惯性补偿：提前若干度停止转弯
        inertia_comp_deg = float(self.get_parameter('turn_inertia_compensation_deg').value)
        effective_tolerance = turn_tolerance + math.radians(inertia_comp_deg)

        # 视觉辅助提前结束：必须先转完大部分名义角，且误差进入窗口，
        # 且中心竖带连续充满 hold_sec。绝不单独靠视觉结束转弯。
        vis_assist = False
        if hasattr(self, 'vision_turn_assist_ready') and self.segment_start_yaw is not None:
            total = abs(self.angle_error(self.segment_target_yaw, self.segment_start_yaw))
            done = abs(self.angle_error(nav_yaw, self.segment_start_yaw))
            progress_ratio = (done / total) if total > 1e-3 else 1.0
            vis_assist = bool(
                self.vision_turn_assist_ready(progress_ratio, abs(math.degrees(error)))
            )

        if abs(error) <= effective_tolerance or vis_assist:
            # 设置加速渐变标志
            self._just_finished_turn = True
            self._turn_finish_time = self.get_clock().now().nanoseconds / 1e9

            reason = 'vis_center_hold' if (vis_assist and abs(error) > effective_tolerance) else 'yaw_tol'
            self.publish_feedback(
                f'{self.test_feedback_prefix}当前位置: '
                f'{self.rectangle_segment_label(self.current_segment or {})}，'
                '转弯完成，进入下一段'
            )
            self.cmd_pub.publish(self.create_twist())

            self._log_session(
                'TURN_COMPLETE',
                f'转弯完成 err={math.degrees(error):.2f}° reason={reason} | '
                f'启动加速渐变 {self._accel_ramp_duration:.2f}s'
            )

            self.start_segment(self.plan_index + 1)
            return

        # 计算基础角速度（IMU 目标角主导）
        angular = self.clamp(self.turn_kp * error, self.turn_angular_speed)
        if abs(error) > turn_tolerance and abs(angular) < self.turn_min_angular_speed:
            angular = math.copysign(self.turn_min_angular_speed, error)

        # 转弯减速：剩余角度 < threshold 时线性衰减
        slowdown_threshold_deg = float(self.get_parameter('turn_slowdown_threshold_deg').value)
        min_speed_ratio = float(self.get_parameter('turn_min_speed_ratio').value)

        if abs(error) < math.radians(slowdown_threshold_deg):
            # 线性衰减，保留最低比例防止电机死区
            scale = abs(error) / math.radians(slowdown_threshold_deg)
            scale = max(min_speed_ratio, scale)
            angular *= scale

        self.cmd_pub.publish(self.create_twist(linear_speed, angular))
        self._maybe_log_telemetry(
            f'turn err={math.degrees(error):.1f}deg'
        )


    def finish_mission(self):
        self._log_session('MISSION', '完成 | ' + self._pose_diagnostic())
        super().finish_mission()
        self._set_vision_inference_active(False)
        self.get_logger().info('第二阶段完成')
        # 独立测试工具可注入 _request_stop；比赛 total 场景保持节点存活待命
        if hasattr(self, '_request_stop') and self._request_stop is not None:
            self._request_stop()


    def control_loop(self):
        if self.corridor_path_active:
            self.run_corridor_path_stage()
            return

        if self._track_mission_active:
            self._run_track_controller()
            return

        if not self.mission_active or self.current_segment is None:
            if not self.mission_active:
                self.cmd_pub.publish(self.create_twist())
            self._maybe_log_telemetry('idle')
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if (
            not self._avoider.is_active
            and self.segment_started_at is not None
            and now_sec - self.segment_started_at > self.segment_timeout
        ):
            desc = self.current_segment.get('description', 'unknown')
            self._log_session(
                'TIMEOUT',
                f'段超时 {desc} | {self._pose_diagnostic()}',
            )
            self.publish_feedback(
                f'{self.test_feedback_prefix}段超时: {desc}'
            )
            self._avoider.reset()
            self.start_segment(self.plan_index + 1)
            return

        segment_type = self.current_segment['type']
        if segment_type == 'track':
            self._run_track_controller()
            return
        if segment_type == 'turn':
            self.run_turn_segment()
        elif segment_type == 'arc':
            self.run_arc_segment()
        elif segment_type == 'move':
            self.run_move_segment()
        elif segment_type == 'pause':
            self.run_pause_segment(now_sec)
            self._maybe_log_telemetry('pause')
        else:
            self.start_segment(self.plan_index + 1)

    def _set_vision_inference_active(self, active: bool):
        node = getattr(self, '_vision_node', None)
        if node is not None and hasattr(node, 'set_inference_active'):
            node.set_inference_active(active)

    def _set_stage2_http_active(self, active: bool):
        node = getattr(self, '_vision_node', None)
        if node is None:
            return
        method_name = 'start_http_server' if active else 'stop_http_server'
        method = getattr(node, method_name, None)
        if method is not None:
            method()

    def phase_callback(self, msg):
        previous_phase = self.phase
        incoming = int(msg.data)
        self.get_logger().info(
            f'[PHASE] 收到 competition_phase={incoming} (之前={previous_phase}, initialized={self.phase_initialized})'
        )

        # 首次 latched 消息：phase=2 视为旧消息；phase=1 完成初始化
        if not self.phase_initialized:
            if incoming == 1:
                self.phase = 1
                self.phase_initialized = True
                self.waiting_for_phase2_start = False
                self.get_logger().info('[PHASE] ✓ Phase 初始化完成: phase=1，等待 Stage1 发布 phase=2')
                return
            if incoming == 2:
                if self.use_test_direction_fallback:
                    self.phase = 2
                    self.phase_initialized = True
                    self.waiting_for_phase2_start = True
                    self.start_after_time = None
                    self.reported_start_delay = False
                    self.reported_waiting_pose = False
                    self._set_stage2_http_active(True)
                    self._set_vision_inference_active(True)
                    self.get_logger().info('[PHASE] ✓ 测试模式接受初始 phase=2，准备启动 Stage2')
                    self.try_start_mission()
                    return
                self.phase = 1
                self.waiting_for_phase2_start = False
                self.get_logger().warn('[PHASE] ⚠ 忽略启动时的 phase=2（可能是旧消息），等待 phase=1')
                return
            self.phase = incoming
            return

        self.phase = incoming
        if previous_phase != self.phase and self.phase != 2:
            self.waiting_for_phase2_start = False
            self._set_vision_inference_active(False)
            self._set_stage2_http_active(False)
            if self.mission_active or previous_phase == 2:
                self.cmd_pub.publish(self.create_twist())
                self.mission_active = False
                self.publish_state('idle')
            return

        # 仅在真正切到 phase=2 时武装启动
        if previous_phase != 2 and self.phase == 2:
            self.waiting_for_phase2_start = True
            self.start_after_time = None
            self.reported_start_delay = False
            self.reported_waiting_pose = False
            self._set_stage2_http_active(True)
            self._set_vision_inference_active(True)
            self.get_logger().info('[MISSION] 收到 phase=2，准备启动 Stage2')
            self.try_start_mission()


    def task_callback(self, msg):
        raw = msg.data.strip()
        self.task_raw = raw
        parsed = self.parse_direction(raw)
        if parsed is None and raw:
            parsed = self.resolve_test_direction(raw)
        self.direction = parsed
        self.get_logger().info(f'[TASK] competition_qr_task="{raw}" → direction={self.direction}')
        # 扫码方向可提前缓存；只有 phase=2 武装后才真正启动
        if self.waiting_for_phase2_start:
            self.try_start_mission()


    def try_start_mission(self):
        if self.mission_active or self.mission_finished:
            return

        # 默认静默：只有 stage1 切到 phase=2 后才会 armed
        if not self.waiting_for_phase2_start:
            return

        if not self.phase_initialized or self.phase != 2:
            self.waiting_for_phase2_start = False
            return

        if self.direction is None:
            if self.use_test_direction_fallback and self.test_direction:
                self.direction = self.test_direction
                self.get_logger().info(
                    f'[MISSION] direction 未从扫码获得，使用 test_direction={self.direction}'
                )
            else:
                # 方向还没到，保持 armed，等 task_callback
                return

        missing_inputs = self._missing_pose_inputs()
        if missing_inputs:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self._last_wait_log_sec >= 3.0:
                self._last_wait_log_sec = now_sec
                if not self.reported_waiting_pose:
                    self.publish_feedback(
                        f'{self.test_feedback_prefix}等待输入就绪: '
                        + ', '.join(missing_inputs)
                    )
                    self.reported_waiting_pose = True
                self.get_logger().info(
                    f'{self.test_feedback_prefix}等待: {", ".join(missing_inputs)}'
                )
            return

        if not self._stage2_start_is_stationary():
            self.start_after_time = None
            self.reported_start_delay = False
            return

        if self.start_after_time is None:
            self.start_after_time = self.get_clock().now().nanoseconds / 1e9 + self.start_delay_sec
            if not self.reported_start_delay:
                self.publish_feedback(
                    f'{self.test_feedback_prefix}位姿已就绪，'
                    f'{self.start_delay_sec:.2f}s 后开始'
                )
                self.reported_start_delay = True
            self.get_logger().info(
                f'[MISSION] ⏱ phase=2 已确认，{self.start_delay_sec}秒后启动'
            )
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time < self.start_after_time:
            return

        self.get_logger().info(
            f'[MISSION] ✓ Stage2 任务启动: phase=2, direction={self.direction}'
        )
        self.waiting_for_phase2_start = False
        self.mission_active = True
        self.reported_start = True

        # 启用视觉推理
        if hasattr(self, '_set_vision_inference_active'):
            self._set_vision_inference_active(True)
        elif getattr(self, '_vision_node', None) is not None:
            self._vision_node.set_inference_active(True)

        self._start_track_mission()
        return
        self.begin_inertial_plan_after_nav(nav_succeeded=self.nav_succeeded_for_test_start())


def main(args=None):
    import threading
    import traceback
    from rclpy.executors import MultiThreadedExecutor

    from racing_stage2.cmd_vel_stop import (
        init_without_ros_signal_handler,
        install_stop_event,
        publish_stop,
    )

    init_without_ros_signal_handler(args)
    node = None
    executor = None
    stop_event = threading.Event()
    request_stop = None
    try:
        node = Stage2InertialNavigator()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        request_stop = install_stop_event(
            stop_event,
            lambda: publish_stop(node.cmd_pub),
            cli_topics=['/cmd_vel', '/stage2_cmd_vel'],
        )
        node._request_stop = request_stop
        threading.Thread(
            target=lambda: (stop_event.wait(), executor.shutdown()),
            daemon=True,
            name='Stage2ExecutorStop',
        ).start()
        executor.spin()
    except KeyboardInterrupt:
        if request_stop is not None:
            request_stop()
    except Exception as exc:
        tb = traceback.format_exc()
        try:
            if executor is not None:
                executor.shutdown()
        except Exception:
            pass
        try:
            if node is not None:
                node.get_logger().error(f'Stage2 crashed: {exc}\n{tb}')
        except Exception:
            pass
        print(f'[Stage2 FATAL] {exc}\n{tb}', flush=True)
        raise
    finally:
        if request_stop is not None:
            try:
                request_stop()
            except Exception:
                pass
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

import math
import heapq
import json
import numpy as np
import threading

import rclpy
import sys
import os

# 添加 voice_api 路径以支持 CN-TTS
voice_api_path = os.path.join(os.path.dirname(__file__), '../../../voice_driver')
if voice_api_path not in sys.path:
    sys.path.insert(0, voice_api_path)

try:
    from voice_api import CnTtsPlayer
    CN_TTS_AVAILABLE = True
except ImportError:
    CN_TTS_AVAILABLE = False

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformException, TransformListener

from racing_common.obstacle_marker_publisher import ObstacleMarkerPublisher
from racing_common.racing_logger import RacingLogger


class CompetitionController(Node):
    def __init__(self):
        super().__init__('competition_controller')

        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('stage2_cmd_topic', '/stage2_cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom_combined')  # map 坐标系
        self.declare_parameter('qr_result_topic', 'qr_scan_result')
        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('task_topic', 'competition_qr_task')
        self.declare_parameter('stage2_state_topic', 'stage2_state')
        self.declare_parameter('stage3_state_topic', 'stage3_state')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('blind_linear_speed', 0.2)
        self.declare_parameter('blind_angular_speed', 0.0)
        self.declare_parameter('avoid_linear_speed', 0.1)
        self.declare_parameter('avoid_angular_speed', 0.8)
        self.declare_parameter('avoid_min_duration_sec', 0.7)
        self.declare_parameter('avoid_clear_hold_sec', 0.25)
        self.declare_parameter('avoid_min_turn_angle_deg', 18.0)
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('clear_distance', 0.65)
        self.declare_parameter('scan_angle_deg', 45.0)
        self.declare_parameter('phase1_window_min_x', 0.18)
        self.declare_parameter('phase1_window_max_x', 0.85)
        self.declare_parameter('phase1_window_half_width', 0.22)
        self.declare_parameter('phase1_cluster_gap_tolerance', 0.12)
        self.declare_parameter('phase1_min_cluster_points', 3)
        self.declare_parameter('phase1_min_cluster_width', 0.06)
        self.declare_parameter('phase1_max_cluster_width', 0.55)
        self.declare_parameter('phase1_emergency_min_x', 0.08)
        self.declare_parameter('phase1_emergency_max_x', 0.45)
        self.declare_parameter('phase1_emergency_half_width', 0.12)
        self.declare_parameter('phase1_emergency_min_points', 2)
        self.declare_parameter('min_valid_range', 0.15)
        self.declare_parameter('recovery_linear_speed', 0.12)
        self.declare_parameter('recovery_turn_linear_speed', 0.08)
        self.declare_parameter('recovery_angular_speed', 0.75)
        self.declare_parameter('counter_steer_linear_speed', 0.10)
        self.declare_parameter('counter_steer_angular_speed', 0.95)
        self.declare_parameter('counter_steer_duration_scale', 1.35)
        self.declare_parameter('counter_steer_min_duration_sec', 0.45)
        self.declare_parameter('counter_steer_max_duration_sec', 1.20)
        self.declare_parameter('recovery_heading_kp', 2.4)
        self.declare_parameter('recovery_max_angular_speed', 1.1)
        self.declare_parameter('recovery_min_angular_speed', 0.5)
        self.declare_parameter('recovery_in_place_angle_deg', 8.0)
        self.declare_parameter('heading_tolerance_deg', 6.0)
        self.declare_parameter('recovery_timeout', 2.5)
        self.declare_parameter('recovery_duration_scale', 0.9)
        self.declare_parameter('stage2_cmd_timeout', 0.5)
        self.declare_parameter('transition_stop_duration', 0.0)
        self.declare_parameter('phase2_obstacle_override', False)
        self.declare_parameter('phase2_emergency_stop_distance', 0.22)
        self.declare_parameter('phase3_external_control', True)
        self.declare_parameter('phase3_emergency_stop_distance', 0.22)
        self.declare_parameter('enable_backing', True)
        self.declare_parameter('back_target_x', 2.0)
        self.declare_parameter('back_linear_speed', -0.15)
        self.declare_parameter('back_angular_kp', 1.8)
        self.declare_parameter('back_position_tolerance', 0.15)
        self.declare_parameter('back_path_sample_distance', 0.20)
        self.declare_parameter('back_timeout_sec', 10.0)
        self.declare_parameter('back_align_yaw_deg', 90.0)
        self.declare_parameter('back_align_tolerance_deg', 5.0)
        self.declare_parameter('back_align_timeout_sec', 5.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom_combined')
        self.declare_parameter('corridor_path_topic', '/stage1_corridor_path')
        self.declare_parameter('enable_corridor_navigation', True)
        self.declare_parameter('corridor_waypoints_json', '[{"x":2.80,"y":3.10}]')
        self.declare_parameter('corridor_waypoint_tolerance', 0.15)
        self.declare_parameter('corridor_goal_tolerance', 0.10)
        self.declare_parameter('corridor_goal_yaw_deg', 90.0)
        self.declare_parameter('corridor_goal_yaw_tolerance_deg', 2.0)
        # clone-main Stage2 corridor pure-pursuit 参数（Stage1 复用）
        self.declare_parameter('corridor_linear_speed', 0.14)
        self.declare_parameter('turn_linear_speed', 0.05)
        self.declare_parameter('turn_min_angular_speed', 0.20)
        self.declare_parameter('turn_angular_speed', 0.8)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('corridor_timeout_sec', 45.0)
        self.declare_parameter('corridor_rho_kp', 0.85)
        self.declare_parameter('corridor_alpha_kp', 2.0)
        self.declare_parameter('corridor_beta_kp', 0.80)
        self.declare_parameter('corridor_creep_speed', 0.04)
        self.declare_parameter('corridor_beta_blend_distance', 0.55)
        self.declare_parameter('corridor_reverse_enabled', False)
        self.declare_parameter('corridor_left_recover_x', 3.50)
        self.declare_parameter('corridor_left_recover_angular', 0.70)
        self.declare_parameter('corridor_left_recover_linear', 0.06)
        self.declare_parameter('corridor_lateral_kp', 1.6)
        self.declare_parameter('corridor_x_tolerance', 0.06)
        self.declare_parameter('phase1_avoid_startup_grace_sec', 1.5)
        # 兼容旧参数名（不再作为主控制）
        self.declare_parameter('corridor_capture_distance', 0.18)
        self.declare_parameter('corridor_capture_exit_distance', 0.30)
        self.declare_parameter('corridor_capture_speed', 0.05)
        self.declare_parameter('corridor_brake_distance', 0.40)
        self.declare_parameter('corridor_brake_kp', 0.28)
        self.declare_parameter('corridor_near_distance', 0.45)
        self.declare_parameter('pure_pursuit_lookahead_m', 0.45)
        self.declare_parameter('pure_pursuit_turn_kp', 1.8)
        self.declare_parameter('pure_pursuit_heading_stop_deg', 70.0)
        self.declare_parameter('use_corridor_planner', False)
        self.declare_parameter('planner_downsample', 4)
        self.declare_parameter('planner_occupied_threshold', 50)
        self.declare_parameter('planner_unknown_is_occupied', True)
        self.declare_parameter('planner_obstacle_inflation_m', 0.14)
        self.declare_parameter('planner_replan_period_sec', 0.5)

        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.stage2_cmd_topic = self.get_parameter('stage2_cmd_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.qr_result_topic = self.get_parameter('qr_result_topic').value
        self.phase_topic = self.get_parameter('phase_topic').value
        self.task_topic = self.get_parameter('task_topic').value
        self.stage2_state_topic = self.get_parameter('stage2_state_topic').value
        self.stage3_state_topic = self.get_parameter('stage3_state_topic').value
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.blind_linear_speed = float(self.get_parameter('blind_linear_speed').value)
        self.blind_angular_speed = float(self.get_parameter('blind_angular_speed').value)
        self.avoid_linear_speed = float(self.get_parameter('avoid_linear_speed').value)
        self.avoid_angular_speed = float(self.get_parameter('avoid_angular_speed').value)
        self.avoid_min_duration_sec = float(self.get_parameter('avoid_min_duration_sec').value)
        self.avoid_clear_hold_sec = float(self.get_parameter('avoid_clear_hold_sec').value)
        self.avoid_min_turn_angle_rad = math.radians(
            float(self.get_parameter('avoid_min_turn_angle_deg').value)
        )
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        self.scan_angle_deg = float(self.get_parameter('scan_angle_deg').value)
        self.phase1_window_min_x = float(self.get_parameter('phase1_window_min_x').value)
        self.phase1_window_max_x = float(self.get_parameter('phase1_window_max_x').value)
        self.phase1_window_half_width = float(self.get_parameter('phase1_window_half_width').value)
        self.phase1_cluster_gap_tolerance = float(self.get_parameter('phase1_cluster_gap_tolerance').value)
        self.phase1_min_cluster_points = int(self.get_parameter('phase1_min_cluster_points').value)
        self.phase1_min_cluster_width = float(self.get_parameter('phase1_min_cluster_width').value)
        self.phase1_max_cluster_width = float(self.get_parameter('phase1_max_cluster_width').value)
        self.phase1_emergency_min_x = float(self.get_parameter('phase1_emergency_min_x').value)
        self.phase1_emergency_max_x = float(self.get_parameter('phase1_emergency_max_x').value)
        self.phase1_emergency_half_width = float(self.get_parameter('phase1_emergency_half_width').value)
        self.phase1_emergency_min_points = int(self.get_parameter('phase1_emergency_min_points').value)
        self.min_valid_range = float(self.get_parameter('min_valid_range').value)
        self.recovery_linear_speed = float(self.get_parameter('recovery_linear_speed').value)
        self.recovery_turn_linear_speed = float(self.get_parameter('recovery_turn_linear_speed').value)
        self.recovery_angular_speed = float(self.get_parameter('recovery_angular_speed').value)
        self.counter_steer_linear_speed = float(self.get_parameter('counter_steer_linear_speed').value)
        self.counter_steer_angular_speed = float(self.get_parameter('counter_steer_angular_speed').value)
        self.counter_steer_duration_scale = float(self.get_parameter('counter_steer_duration_scale').value)
        self.counter_steer_min_duration_sec = float(self.get_parameter('counter_steer_min_duration_sec').value)
        self.counter_steer_max_duration_sec = float(self.get_parameter('counter_steer_max_duration_sec').value)
        self.recovery_heading_kp = float(self.get_parameter('recovery_heading_kp').value)
        self.recovery_max_angular_speed = float(self.get_parameter('recovery_max_angular_speed').value)
        self.recovery_min_angular_speed = float(self.get_parameter('recovery_min_angular_speed').value)
        self.recovery_in_place_angle_rad = math.radians(
            float(self.get_parameter('recovery_in_place_angle_deg').value)
        )
        self.heading_tolerance_rad = math.radians(float(self.get_parameter('heading_tolerance_deg').value))
        self.recovery_timeout = float(self.get_parameter('recovery_timeout').value)
        self.recovery_duration_scale = float(self.get_parameter('recovery_duration_scale').value)
        self.stage2_cmd_timeout = float(self.get_parameter('stage2_cmd_timeout').value)
        self.transition_stop_duration = float(self.get_parameter('transition_stop_duration').value)
        self.phase2_obstacle_override = bool(self.get_parameter('phase2_obstacle_override').value)
        self.phase2_emergency_stop_distance = float(self.get_parameter('phase2_emergency_stop_distance').value)
        self.phase3_external_control = bool(self.get_parameter('phase3_external_control').value)
        self.phase3_emergency_stop_distance = float(self.get_parameter('phase3_emergency_stop_distance').value)
        self.enable_backing = bool(self.get_parameter('enable_backing').value)
        self.back_target_x = float(self.get_parameter('back_target_x').value)
        self.back_linear_speed = float(self.get_parameter('back_linear_speed').value)
        self.back_angular_kp = float(self.get_parameter('back_angular_kp').value)
        self.back_position_tolerance = float(self.get_parameter('back_position_tolerance').value)
        self.back_path_sample_distance = float(self.get_parameter('back_path_sample_distance').value)
        self.back_timeout_sec = float(self.get_parameter('back_timeout_sec').value)
        self.back_align_yaw_rad = math.radians(float(self.get_parameter('back_align_yaw_deg').value))
        self.back_align_tolerance_rad = math.radians(float(self.get_parameter('back_align_tolerance_deg').value))
        self.back_align_timeout_sec = float(self.get_parameter('back_align_timeout_sec').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.corridor_path_topic = str(self.get_parameter('corridor_path_topic').value)
        self.enable_corridor_navigation = bool(self.get_parameter('enable_corridor_navigation').value)
        self.corridor_waypoints = self._parse_corridor_waypoints(
            str(self.get_parameter('corridor_waypoints_json').value)
        )
        self.corridor_waypoint_tolerance = float(self.get_parameter('corridor_waypoint_tolerance').value)
        self.corridor_goal_tolerance = float(self.get_parameter('corridor_goal_tolerance').value)
        self.corridor_goal_yaw = math.radians(float(self.get_parameter('corridor_goal_yaw_deg').value))
        self.corridor_goal_yaw_tolerance = math.radians(
            float(self.get_parameter('corridor_goal_yaw_tolerance_deg').value)
        )
        self.corridor_linear_speed = float(self.get_parameter('corridor_linear_speed').value)
        self.turn_linear_speed = float(self.get_parameter('turn_linear_speed').value)
        self.turn_min_angular_speed = float(self.get_parameter('turn_min_angular_speed').value)
        self.turn_angular_speed = float(self.get_parameter('turn_angular_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.corridor_timeout_sec = float(self.get_parameter('corridor_timeout_sec').value)
        self.corridor_rho_kp = float(self.get_parameter('corridor_rho_kp').value)
        self.corridor_alpha_kp = float(self.get_parameter('corridor_alpha_kp').value)
        self.corridor_beta_kp = float(self.get_parameter('corridor_beta_kp').value)
        self.corridor_creep_speed = float(self.get_parameter('corridor_creep_speed').value)
        self.corridor_beta_blend_distance = float(self.get_parameter('corridor_beta_blend_distance').value)
        self.corridor_reverse_enabled = bool(self.get_parameter('corridor_reverse_enabled').value)
        self.corridor_capture_distance = float(self.get_parameter('corridor_capture_distance').value)
        self.corridor_capture_exit_distance = float(self.get_parameter('corridor_capture_exit_distance').value)
        self.corridor_capture_speed = float(self.get_parameter('corridor_capture_speed').value)
        self.corridor_brake_distance = float(self.get_parameter('corridor_brake_distance').value)
        self.corridor_brake_kp = float(self.get_parameter('corridor_brake_kp').value)
        self.corridor_near_distance = float(self.get_parameter('corridor_near_distance').value)
        self.corridor_left_recover_x = float(self.get_parameter('corridor_left_recover_x').value)
        self.corridor_left_recover_angular = float(self.get_parameter('corridor_left_recover_angular').value)
        self.corridor_left_recover_linear = float(self.get_parameter('corridor_left_recover_linear').value)
        self.corridor_lateral_kp = float(self.get_parameter('corridor_lateral_kp').value)
        self.corridor_x_tolerance = float(self.get_parameter('corridor_x_tolerance').value)
        self.phase1_avoid_startup_grace_sec = float(self.get_parameter('phase1_avoid_startup_grace_sec').value)
        self.pure_pursuit_lookahead = float(self.get_parameter('pure_pursuit_lookahead_m').value)
        self.pure_pursuit_turn_kp = float(self.get_parameter('pure_pursuit_turn_kp').value)
        self.pure_pursuit_heading_stop = math.radians(
            float(self.get_parameter('pure_pursuit_heading_stop_deg').value)
        )
        self.use_corridor_planner = bool(self.get_parameter('use_corridor_planner').value)
        self.planner_downsample = max(1, int(self.get_parameter('planner_downsample').value))
        self.planner_occupied_threshold = int(self.get_parameter('planner_occupied_threshold').value)
        self.planner_unknown_is_occupied = bool(self.get_parameter('planner_unknown_is_occupied').value)
        self.planner_obstacle_inflation_m = float(
            self.get_parameter('planner_obstacle_inflation_m').value
        )
        self.planner_replan_period_sec = float(self.get_parameter('planner_replan_period_sec').value)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.phase_pub = self.create_publisher(Int32, self.phase_topic, latched_qos)
        self.task_pub = self.create_publisher(String, self.task_topic, latched_qos)

        self.create_subscription(LaserScan, self.scan_topic, self.lidar_callback, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.create_subscription(String, self.qr_result_topic, self.qr_callback, 10)
        self.create_subscription(Twist, self.stage2_cmd_topic, self.stage2_cmd_callback, 10)
        self.create_subscription(String, self.stage2_state_topic, self.stage2_state_callback, 10)
        self.create_subscription(String, self.stage3_state_topic, self.stage3_state_callback, 10)

        self.phase = 1
        self.mission_finished = False
        self.obstacle_found = False
        self.closest_obstacle_distance = float('inf')
        self.avoid_cmd = Twist()
        self.phase1_motion_state = 'forward'
        self.current_yaw = None
        self.desired_heading = None
        self.avoid_turn_direction = 0.0
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.recovery_uses_heading = False
        self.warned_missing_heading = False
        self.latest_stage2_cmd = Twist()
        self.latest_stage2_cmd_time = None
        self.transition_end_time = None
        self.qr_task = ''
        self.stage2_state = 'idle'
        self.stage3_state = 'idle'

        # 路径记录与后退状态
        self.current_odom = None
        self.path_record = []  # [(x, y, yaw), ...]
        self.last_recorded_position = None
        self.backing_started_time = None
        self.backing_path_index = -1
        self.aligning_started_time = None
        self.latest_map = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._map_pose_warned = False
        self.corridor_active = False
        self.corridor_nav_mode = 'idle'  # polar | capture | left_recover | idle
        self.corridor_capture_active = False
        self._node_start_time = self.get_clock().now()
        self.corridor_index = 0
        self.corridor_started_at = None
        self.corridor_path_points = []
        self.corridor_path_updated_at = 0.0
        self.corridor_resume_after_avoidance = False
        self.corridor_path_pub = self.create_publisher(Path, self.corridor_path_topic, 10)

        # RacingLogger：日志文件 ~/dev_ws/log/competition_stage1/latest.log
        self.log = RacingLogger(
            self, log_subdir='competition_stage1',
            log_filename='latest.log', session_title='Stage1 competition',
        )
        # CN-TTS 语音播报初始化
        self.tts_player = None
        if CN_TTS_AVAILABLE:
            try:
                self.tts_player = CnTtsPlayer(port='/dev/ttyS1', baudrate=9600, logger=self.get_logger())
                self.log.startup('CN-TTS 语音模块已初始化 (port=/dev/ttyS1, baud=9600)')
                self.get_logger().info('[VOICE] CN-TTS 模块已初始化')
            except Exception as e:
                self.log.warn('VOICE', f'CN-TTS 初始化失败: {e}')
                self.get_logger().warn(f'[VOICE] CN-TTS 初始化失败: {e}')
        else:
            self.log.warn('VOICE', 'CN-TTS 模块不可用（voice_api 未安装）')
            self.get_logger().warn('[VOICE] CN-TTS 模块不可用')


        # 障碍物可视化（rviz2 调试用）
        self.obstacle_markers = ObstacleMarkerPublisher(
            self, topic='/stage1_obstacle_markers', frame_id='base_link', radius=0.13
        )
        self._phase1_last_clusters = []  # 缓存上一帧的聚类结果

        # 初始化时立即发布 phase=1，覆盖可能存在的旧 TRANSIENT_LOCAL 消息
        self.publish_phase()
        self.log.startup(f'✓ 初始 phase={self.phase} 已发布到 {self.phase_topic}')
        self.create_timer(1.0 / max(control_rate_hz, 1.0), self.control_loop)

        self.log.startup('competition controller ready: phase1 blind drive, phase2 corridor, phase3 return-to-p')

    def quaternion_to_yaw(self, orientation):
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def angle_error(self, target_angle, current_angle):
        return self.normalize_angle(target_angle - current_angle)

    def create_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        return msg

    def publish_phase(self):
        self.phase_pub.publish(Int32(data=self.phase))

    def begin_phase_transition(self, target_phase, reason):
        if self.phase == target_phase:
            self.log.progress(f'Phase切换请求被忽略: 已经是 phase={target_phase}')
            return

        self.phase = target_phase
        self.publish_phase()
        self.log.mission(f'✓ Phase切换执行: {self.phase-1} → {target_phase}, 原因: {reason}')
        self.stop_robot()
        self.latest_stage2_cmd = Twist()
        self.latest_stage2_cmd_time = None
        if self.transition_stop_duration > 0.0:
            self.transition_end_time = self.get_clock().now() + Duration(seconds=self.transition_stop_duration)
        else:
            self.transition_end_time = None

    def clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def imu_callback(self, msg):
        self.current_yaw = self.quaternion_to_yaw(msg.orientation)
        if self.phase == 1 and self.desired_heading is None:
            self.desired_heading = self.current_yaw
            self.log.config(f'phase1 heading locked at {math.degrees(self.desired_heading):.1f} deg')

    def odom_callback(self, msg):
        """订阅 /odom_combined 用于路径记录（位置）"""
        self.current_odom = msg
        
        # Phase 1 前进时记录路径（只在 forward/avoiding/countersteering/recovering 时记录）
        # 位置用 odom (x, y)，角度用 IMU (self.current_yaw)
        if self.phase == 1 and self.enable_backing and self.phase1_motion_state in ('forward', 'avoiding', 'countersteering', 'recovering'):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            # 使用纯 IMU 角度，而不是 odom 的 orientation（避免融合后角度不一致）
            yaw = self.current_yaw if self.current_yaw is not None else 0.0
            
            # 采样：距离上次记录点 >= sample_distance 才记录
            if self.last_recorded_position is None:
                self.path_record.append((x, y, yaw))
                self.last_recorded_position = (x, y)
            else:
                dist = math.hypot(x - self.last_recorded_position[0], y - self.last_recorded_position[1])
                if dist >= self.back_path_sample_distance:
                    self.path_record.append((x, y, yaw))
                    self.last_recorded_position = (x, y)
                    # 限制最大路径点数防止内存占用
                    if len(self.path_record) > 1000:
                        self.path_record.pop(0)

    def map_callback(self, msg):
        self.latest_map = msg

    @staticmethod
    def _parse_corridor_waypoints(raw_json):
        try:
            raw_waypoints = json.loads(raw_json)
        except (TypeError, json.JSONDecodeError):
            return []
        if not isinstance(raw_waypoints, list):
            return []
        result = []
        for index, waypoint in enumerate(raw_waypoints):
            if not isinstance(waypoint, dict):
                continue
            result.append({
                'x': float(waypoint.get('x', 0.0)),
                'y': float(waypoint.get('y', 0.0)),
                'speed': float(waypoint.get('speed', 0.14)),
                'description': str(waypoint.get('description', f'corridor_wp_{index}')),
            })
        return result

    def corridor_goal_point(self):
        """返回通道导航最终目标点（来自 yaml corridor_waypoints_json 最后一点）。"""
        if self.corridor_waypoints:
            goal = self.corridor_waypoints[-1]
            return float(goal['x']), float(goal['y'])
        return None

    def _transform_xy(self, x, y, yaw, point_x, point_y):
        """把 source 坐标系点变换到 target 坐标系（2D）。"""
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            x + cos_yaw * point_x - sin_yaw * point_y,
            y + sin_yaw * point_x + cos_yaw * point_y,
        )

    def _lookup_map_xy_from_tf(self):
        """优先查 TF: map <- base_frame。"""
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
            t = transform.transform.translation
            return float(t.x), float(t.y)
        return None

    def get_map_position(self):
        """通道导航统一使用 map 坐标；失败时用 odom+静态 TF 兜底。"""
        map_xy = self._lookup_map_xy_from_tf()
        if map_xy is not None:
            return map_xy

        if self.current_odom is None:
            return None

        # 兜底：用 map->odom 静态变换把 odom 位姿转到 map
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, Time(), timeout=Duration(seconds=0.05)
            )
            t = transform.transform.translation
            q = transform.transform.rotation
            yaw = self.quaternion_to_yaw(q)
            ox = float(self.current_odom.pose.pose.position.x)
            oy = float(self.current_odom.pose.pose.position.y)
            return self._transform_xy(float(t.x), float(t.y), yaw, ox, oy)
        except TransformException:
            pass

        if not self._map_pose_warned:
            self._map_pose_warned = True
            self.log.warn(
                'POSE',
                f'无法获取 map 位姿，临时退回 {self.odom_topic} 原始坐标；'
                f'请检查 TF {self.map_frame}->{self.odom_frame}->{self.base_frame}'
            )
        pos = self.current_odom.pose.pose.position
        return float(pos.x), float(pos.y)

    def start_corridor_navigation(self, reason):
        if not self.enable_corridor_navigation or not self.corridor_waypoints:
            self.phase1_motion_state = 'forward'
            self.begin_phase_transition(2, reason)
            return
        self.corridor_active = True
        self.corridor_nav_mode = 'polar'
        self.corridor_capture_active = False
        self._corridor_timeout_logged = False
        self.corridor_index = 0
        self.corridor_started_at = self.get_clock().now().nanoseconds / 1e9
        self.corridor_path_points = []
        self.corridor_path_updated_at = 0.0
        self.corridor_planning_failures = 0
        self.phase1_motion_state = 'corridor'
        self.stop_robot()
        self.log.mission(f'后退完成，开始地图通道导航: {reason}')
        map_xy = self.get_map_position()
        goal_xy = self.corridor_goal_point()
        if map_xy is not None and goal_xy is not None:
            gx, gy = goal_xy
            odom_txt = ''
            if self.current_odom is not None:
                op = self.current_odom.pose.pose.position
                odom_txt = f', odom=({op.x:.2f},{op.y:.2f})'
            self.log.progress(
                f'通道导航起点(map): ({map_xy[0]:.2f}, {map_xy[1]:.2f}){odom_txt}, '
                f'目标(map): ({gx:.2f}, {gy:.2f}), '
                f'距离: {math.hypot(map_xy[0] - gx, map_xy[1] - gy):.2f}m'
            )

    def _map_world_to_grid(self, x, y, step):
        info = self.latest_map.info
        gx = int(math.floor((x - info.origin.position.x) / info.resolution))
        gy = int(math.floor((y - info.origin.position.y) / info.resolution))
        return gx // step, gy // step

    def _map_grid_to_world(self, gx, gy, step):
        info = self.latest_map.info
        return (
            info.origin.position.x + (gx * step + 0.5 * step) * info.resolution,
            info.origin.position.y + (gy * step + 0.5 * step) * info.resolution,
        )

    def _corridor_occupancy(self, step):
        info = self.latest_map.info
        width = max(1, info.width // step)
        height = max(1, info.height // step)
        source = np.asarray(self.latest_map.data, dtype=np.int16).reshape(info.height, info.width)
        occupied = np.zeros((height, width), dtype=bool)
        for gy in range(height):
            for gx in range(width):
                block = source[gy * step:min((gy + 1) * step, info.height), gx * step:min((gx + 1) * step, info.width)]
                if self.planner_unknown_is_occupied:
                    occupied[gy, gx] = bool(np.any((block < 0) | (block >= self.planner_occupied_threshold)))
                else:
                    occupied[gy, gx] = bool(np.any(block >= self.planner_occupied_threshold))
        radius = int(math.ceil(self.planner_obstacle_inflation_m / info.resolution / step))
        if radius <= 0:
            return occupied
        original = occupied.copy()
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue
                source_y0 = max(0, -dy)
                source_y1 = min(height, height - dy)
                source_x0 = max(0, -dx)
                source_x1 = min(width, width - dx)
                target_y0 = max(0, dy)
                target_y1 = min(height, height + dy)
                target_x0 = max(0, dx)
                target_x1 = min(width, width + dx)
                occupied[target_y0:target_y1, target_x0:target_x1] |= original[source_y0:source_y1, source_x0:source_x1]
        return occupied

    @staticmethod
    def _nearest_free(occupied, point):
        gx, gy = point
        height, width = occupied.shape
        if 0 <= gx < width and 0 <= gy < height and not occupied[gy, gx]:
            return point
        for radius in range(1, 20):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    x, y = gx + dx, gy + dy
                    if 0 <= x < width and 0 <= y < height and not occupied[y, x]:
                        return x, y
        return None

    def plan_corridor_path(self, start, goal):
        plan_start_time = self.get_clock().now().nanoseconds / 1e9
        self.log.progress(f'开始规划路径: 起点({start[0]:.2f}, {start[1]:.2f}) → 终点({goal[0]:.2f}, {goal[1]:.2f})')
        if self.latest_map is None:
            self.log.error('CORRIDOR', '路径规划失败: 地图数据未加载')
            return None
        step = max(1, self.planner_downsample)
        occupied = self._corridor_occupancy(step)
        start_cell = self._nearest_free(occupied, self._map_world_to_grid(start[0], start[1], step))
        goal_cell = self._nearest_free(occupied, self._map_world_to_grid(goal[0], goal[1], step))
        if start_cell is None or goal_cell is None:
            self.log.error('CORRIDOR', f'路径规划失败: 起点或终点不可通行 (start_cell={start_cell}, goal_cell={goal_cell})')
            return []
        frontier = [(0.0, start_cell)]
        came_from = {start_cell: None}
        cost = {start_cell: 0.0}
        neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal_cell:
                break
            for dx, dy in neighbors:
                nxt = current[0] + dx, current[1] + dy
                if not (0 <= nxt[0] < occupied.shape[1] and 0 <= nxt[1] < occupied.shape[0]) or occupied[nxt[1], nxt[0]]:
                    continue
                move_cost = 1.4142 if dx and dy else 1.0
                new_cost = cost[current] + move_cost
                if new_cost >= cost.get(nxt, float('inf')):
                    continue
                cost[nxt] = new_cost
                priority = new_cost + math.hypot(goal_cell[0] - nxt[0], goal_cell[1] - nxt[1])
                heapq.heappush(frontier, (priority, nxt))
                came_from[nxt] = current
        if goal_cell not in came_from:
            self.log.error('CORRIDOR', f'路径规划失败: A* 搜索未找到通路，已搜索 {len(came_from)} 个节点')
            return []
        cells = []
        current = goal_cell
        while current is not None:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        plan_end_time = self.get_clock().now().nanoseconds / 1e9
        self.log.progress(f'路径规划成功: {len(cells)} 个路点, 耗时 {(plan_end_time - plan_start_time)*1000:.1f}ms')
        return [self._map_grid_to_world(x, y, step) for x, y in cells]

    def _corridor_lookahead(self, path, current_xy):
        if not path:
            self.log.error('CORRIDOR', 'Lookahead: 路径为空！')
            return None
        if current_xy is None:
            return path[-1]
        traveled = 0.0
        previous = current_xy
        for point in path:
            traveled += math.hypot(point[0] - previous[0], point[1] - previous[1])
            if traveled >= self.pure_pursuit_lookahead:
                return point
            previous = point
        return path[-1]

    def maybe_advance_corridor_waypoint(self, pose_xy):
        """距当前路点足够近则推进到下一路点。"""
        while self.corridor_index < len(self.corridor_waypoints) - 1:
            waypoint = self.corridor_waypoints[self.corridor_index]
            distance = math.hypot(waypoint['x'] - pose_xy[0], waypoint['y'] - pose_xy[1])
            if distance > self.corridor_waypoint_tolerance:
                return
            self.corridor_index += 1
            self.log.progress(
                f'corridor waypoint advanced → index={self.corridor_index}/'
                f'{len(self.corridor_waypoints)-1}'
            )


    def _lookup_map_x(self):
        map_xy = self.get_map_position()
        if map_xy is None:
            return None
        return float(map_xy[0])

    def maybe_left_recover_cmd(self, reason_tag='left_recover'):
        """map_x 过大时向左旋回，返回 Twist 或 None。盲开/通道共用。"""
        map_x = self._lookup_map_x()
        if map_x is None:
            return None
        if map_x <= self.corridor_left_recover_x:
            return None
        linear = max(self.corridor_left_recover_linear, self.corridor_creep_speed, 0.05)
        angular = abs(self.corridor_left_recover_angular)
        now_ts = self.get_clock().now().nanoseconds / 1e9
        if now_ts - getattr(self, '_left_recover_log_time', 0.0) >= 1.0:
            self._left_recover_log_time = now_ts
            self.log.progress(
                f'{reason_tag}: map_x={map_x:.2f}>{self.corridor_left_recover_x:.2f} '
                f'v={linear:.2f} w={angular:.2f}'
            )
        return self.create_twist(linear, angular)

    def handle_corridor_navigation(self):
        """
        Stage1 通道导航：
          1) map_x>阈值：left_recover 向左旋回
          2) 中段：前进 ρ-α-β + 横向 x 修正
          3) 终点：capture 锁存，body-frame 微修正（过冲可短退）
          4) 位置+航向到位后 phase=2
        """
        now_ts = self.get_clock().now().nanoseconds / 1e9
        map_xy = self.get_map_position()
        if not hasattr(self, '_corridor_last_log_time'):
            self._corridor_last_log_time = 0.0

        if map_xy is None or self.current_yaw is None:
            self.stop_robot()
            return

        if self.corridor_started_at is not None and now_ts - self.corridor_started_at > self.corridor_timeout_sec:
            self.corridor_active = False
            self.corridor_nav_mode = 'idle'
            self.corridor_capture_active = False
            self.stop_robot()
            if not getattr(self, '_corridor_timeout_logged', False):
                self._corridor_timeout_logged = True
                self.log.error('CORRIDOR', '地图通道导航超时，保持 Stage1 停止，不提前进入 Stage2')
            return

        pose_xy = (float(map_xy[0]), float(map_xy[1]))
        yaw = float(self.current_yaw)
        self.maybe_advance_corridor_waypoint(pose_xy)

        waypoint = self.corridor_waypoints[self.corridor_index]
        goal_xy = (float(waypoint['x']), float(waypoint['y']))
        final_goal = self.corridor_waypoints[-1]
        final_goal_xy = (float(final_goal['x']), float(final_goal['y']))
        is_final = self.corridor_index >= len(self.corridor_waypoints) - 1

        dx = goal_xy[0] - pose_xy[0]
        dy = goal_xy[1] - pose_xy[1]
        rho = math.hypot(dx, dy)
        final_dx = final_goal_xy[0] - pose_xy[0]
        final_dy = final_goal_xy[1] - pose_xy[1]
        final_rho = math.hypot(final_dx, final_dy)
        yaw_error = self.angle_error(self.corridor_goal_yaw, yaw)
        los = math.atan2(dy, dx) if rho > 1e-6 else yaw
        alpha = self.angle_error(los, yaw)
        beta = self.angle_error(self.corridor_goal_yaw, los) if is_final else 0.0

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        target_x = cos_yaw * dx + sin_yaw * dy
        target_y = -sin_yaw * dx + cos_yaw * dy
        # map 横向误差（目标 x - 当前 x）：正值表示目标在右侧，需右修；负值在左
        map_x_err = final_dx if is_final else dx

        if now_ts - self._corridor_last_log_time >= 1.0:
            elapsed = now_ts - self.corridor_started_at if self.corridor_started_at else 0.0
            odom_txt = ''
            if self.current_odom is not None:
                op = self.current_odom.pose.pose.position
                odom_txt = f' odom=({op.x:.2f},{op.y:.2f})'
            self.log.segment(
                f'导航(map): ({pose_xy[0]:.2f},{pose_xy[1]:.2f})→'
                f'({final_goal_xy[0]:.2f},{final_goal_xy[1]:.2f}) {final_rho:.2f}m '
                f'xerr={map_x_err:+.2f} yaw={math.degrees(yaw):.0f}° '
                f'α={math.degrees(alpha):.0f}° err={math.degrees(yaw_error):.0f}° '
                f'mode={self.corridor_nav_mode} cap={int(self.corridor_capture_active)} '
                f'{elapsed:.0f}s{odom_txt}'
            )
            print(
                f"\r[Stage1导航] map({pose_xy[0]:.2f}, {pose_xy[1]:.2f}) → "
                f"目标({final_goal_xy[0]:.2f}, {final_goal_xy[1]:.2f}) | 距离{final_rho:.2f}m | "
                f"xerr{map_x_err:+.2f} | 航向{math.degrees(yaw):.0f}° | "
                f"mode={self.corridor_nav_mode} | 用时{elapsed:.0f}s",
                end="",
                flush=True,
            )
            self._corridor_last_log_time = now_ts

        # map_x 过大：强制向左旋回（通道导航内）
        left_cmd = self.maybe_left_recover_cmd('corridor_left_recover')
        if left_cmd is not None and final_rho > self.corridor_goal_tolerance:
            self.corridor_nav_mode = 'left_recover'
            self.corridor_capture_active = False
            self.cmd_pub.publish(left_cmd)
            return

        # capture 滞回
        if is_final:
            if (not self.corridor_capture_active) and final_rho <= self.corridor_capture_distance:
                self.corridor_capture_active = True
                self.log.progress(
                    f'capture enter: ρ={final_rho:.2f}m x={pose_xy[0]:.2f} xerr={map_x_err:+.2f}'
                )
            elif self.corridor_capture_active and final_rho >= self.corridor_capture_exit_distance:
                self.corridor_capture_active = False
                self.log.progress(
                    f'capture exit: ρ={final_rho:.2f}m x={pose_xy[0]:.2f}'
                )

        # 完成：距离 + x 精度 + 航向
        x_ok = abs(map_x_err) <= max(self.corridor_x_tolerance, self.corridor_goal_tolerance)
        if (
            is_final
            and final_rho <= self.corridor_goal_tolerance
            and x_ok
            and abs(yaw_error) <= self.corridor_goal_yaw_tolerance
        ):
            self.corridor_active = False
            self.corridor_nav_mode = 'idle'
            self.corridor_capture_active = False
            self.phase1_motion_state = 'forward'
            self.stop_robot()
            reason = (
                f'通道导航到达 map({pose_xy[0]:.2f}, {pose_xy[1]:.2f})≈'
                f'({final_goal_xy[0]:.2f}, {final_goal_xy[1]:.2f})，航向已对齐'
            )
            self.begin_phase_transition(2, reason)
            return

        # ===== capture：优先把 x 拉回，再拧航向 =====
        if is_final and self.corridor_capture_active:
            self.corridor_nav_mode = 'capture'
            # body x 拉近点；同时对 map_x 误差给额外横向角速度
            linear = self.clamp(1.6 * target_x, self.corridor_capture_speed)
            # 当 yaw≈90° 时，target_y 与 map_x_err 相关；再叠加显式 x 修正
            lateral_term = 3.0 * target_y + self.corridor_lateral_kp * map_x_err
            yaw_weight = max(0.20, min(1.0, 1.0 - (final_rho / max(self.corridor_capture_exit_distance, 1e-3))))
            # x 还没对齐时，先弱化终航向，避免边拧边漂
            if abs(map_x_err) > self.corridor_x_tolerance:
                yaw_weight *= 0.35
            angular = self.clamp(
                lateral_term + self.corridor_alpha_kp * yaw_weight * yaw_error,
                self.max_angular_speed,
            )

            if final_rho <= self.corridor_goal_tolerance and abs(map_x_err) <= self.corridor_x_tolerance:
                # 位置/ x 已够好：蠕行拧终航向
                angular = self.clamp(self.corridor_alpha_kp * yaw_error, self.turn_angular_speed)
                if abs(angular) < self.turn_min_angular_speed and abs(yaw_error) > self.corridor_goal_yaw_tolerance:
                    angular = math.copysign(self.turn_min_angular_speed, yaw_error if yaw_error != 0.0 else 1.0)
                if target_x < -0.02:
                    linear = -max(self.corridor_creep_speed, 0.03)
                else:
                    linear = max(self.corridor_creep_speed, 0.03)
            else:
                # x 偏差大时：用侧向修正主导，线速度限制更低
                if abs(map_x_err) > self.corridor_x_tolerance:
                    linear = self.clamp(linear, min(self.corridor_capture_speed, 0.04))
                if abs(angular) > 0.05 and abs(linear) < max(self.corridor_creep_speed, 0.03):
                    direction = -1.0 if target_x < 0.0 else 1.0
                    linear = direction * max(self.corridor_creep_speed, 0.03)

            if now_ts - getattr(self, '_approach_log_time', 0.0) >= 0.5:
                self._approach_log_time = now_ts
                self.log.progress(
                    f'capture: ρ={final_rho:.2f}m x={pose_xy[0]:.2f} xerr={map_x_err:+.2f} '
                    f'body=({target_x:.2f},{target_y:.2f}) yaw_err={math.degrees(yaw_error):.1f}° '
                    f'v={linear:.2f} w={angular:.2f}'
                )
            self.cmd_pub.publish(self.create_twist(linear, angular))
            return

        # ===== 中段前进：ρ-α-β + 显式 map_x 横向修正 =====
        self.corridor_nav_mode = 'polar'
        reverse = False
        control_alpha = alpha
        if self.corridor_reverse_enabled and abs(alpha) > (math.pi * 0.75):
            reverse = True
            control_alpha = self.normalize_angle(alpha + math.copysign(math.pi, alpha))

        if is_final and self.corridor_beta_blend_distance > 1e-6:
            beta_scale = max(0.0, min(1.0, 1.0 - (rho / self.corridor_beta_blend_distance)))
        else:
            beta_scale = 0.0 if not is_final else 1.0

        speed_cap = abs(float(waypoint.get('speed', self.corridor_linear_speed)))
        speed_cap = max(self.corridor_creep_speed, min(self.corridor_linear_speed, speed_cap))

        heading_scale = max(0.20, abs(math.cos(control_alpha)))
        linear = min(speed_cap, self.corridor_rho_kp * rho * heading_scale)

        if is_final and self.corridor_brake_distance > 1e-6 and rho < self.corridor_brake_distance:
            brake_cap = max(self.corridor_creep_speed, self.corridor_brake_kp * rho)
            if rho < 0.25:
                brake_cap = min(brake_cap, max(self.corridor_creep_speed, 0.05))
            if rho < 0.15:
                brake_cap = min(brake_cap, max(self.corridor_creep_speed, 0.03))
            linear = min(linear, brake_cap)

        if abs(control_alpha) > math.radians(45.0):
            linear = min(linear, max(self.corridor_creep_speed, self.turn_linear_speed))
        if abs(control_alpha) > math.radians(70.0):
            linear = min(linear, max(self.corridor_creep_speed, 0.03))

        if reverse:
            linear = -abs(linear)
        else:
            linear = abs(linear)

        # 横向：LOS α + 对 map_x 的额外修正，避免一路漂到 2.4x
        lateral_boost = 0.0
        if is_final:
            # 近场加强 x 回正；远场也给一点，防止只追 LOS 过冲到左侧
            lat_scale = 1.0 if rho < 1.2 else 0.55
            lateral_boost = self.corridor_lateral_kp * lat_scale * map_x_err

        angular = (
            self.corridor_alpha_kp * control_alpha
            + (self.corridor_beta_kp * beta_scale) * beta
            + lateral_boost
        )
        angular = self.clamp(angular, self.max_angular_speed)

        if abs(angular) > 0.05:
            min_lin = max(self.corridor_creep_speed, 0.03)
            if abs(linear) < min_lin:
                linear = math.copysign(min_lin, -1.0 if reverse else 1.0)

        if now_ts - getattr(self, '_approach_log_time', 0.0) >= 1.0:
            self._approach_log_time = now_ts
            self.log.progress(
                f'polar: ρ={rho:.2f}m x={pose_xy[0]:.2f} xerr={map_x_err:+.2f} '
                f'yaw={math.degrees(yaw):.1f}° α={math.degrees(control_alpha):.1f}° '
                f'lat={lateral_boost:.2f} rev={int(reverse)} v={linear:.2f} w={angular:.2f}'
            )

        self.cmd_pub.publish(self.create_twist(linear, angular))

    def begin_avoidance(self, danger_angle):
        if self.phase1_motion_state == 'corridor' or self.corridor_active:
            self.corridor_resume_after_avoidance = True
        self.phase1_motion_state = 'avoiding'
        self.avoid_turn_direction = -1.0 if danger_angle > 0.0 else 1.0
        self.avoid_started_time = self.get_clock().now()
        self.avoid_clear_since = None
        self.avoid_entry_yaw = self.current_yaw
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.recovery_uses_heading = False

        if self.desired_heading is None and self.current_yaw is not None:
            self.desired_heading = self.current_yaw

        self.log.feedback(
            f'avoid start dir={self.avoid_turn_direction:.0f} '
            f'danger_angle={danger_angle:.0f}°'
        )

    def begin_counter_steer(self):
        if self.phase1_motion_state != 'avoiding':
            return

        now = self.get_clock().now()
        avoid_duration = 0.0
        if self.avoid_started_time is not None:
            avoid_duration = (now - self.avoid_started_time).nanoseconds / 1e9
        self.last_avoid_duration = avoid_duration

        counter_duration = max(
            self.counter_steer_min_duration_sec,
            avoid_duration * self.counter_steer_duration_scale,
        )
        counter_duration = min(counter_duration, self.counter_steer_max_duration_sec)

        self.phase1_motion_state = 'countersteering'
        self.avoid_clear_since = None
        self.counter_steer_deadline = now + Duration(seconds=counter_duration)
        self.recovery_deadline = None
        self.recovery_uses_heading = False

        self.log.feedback(f'countersteer duration={counter_duration:.2f}s')

    def begin_recovery(self):
        if self.phase1_motion_state not in ('avoiding', 'countersteering'):
            return

        now = self.get_clock().now()
        avoid_duration = self.last_avoid_duration
        if avoid_duration <= 0.0 and self.avoid_started_time is not None:
            avoid_duration = (now - self.avoid_started_time).nanoseconds / 1e9

        self.phase1_motion_state = 'recovering'
        self.avoid_clear_since = None
        self.counter_steer_deadline = None
        self.recovery_uses_heading = self.current_yaw is not None and self.desired_heading is not None
        if self.recovery_uses_heading:
            heading_error = abs(self.angle_error(self.desired_heading, self.current_yaw))
            estimated_duration = max(
                0.6,
                heading_error / max(self.recovery_max_angular_speed, 0.1) * 1.6,
            )
            self.recovery_deadline = now + Duration(seconds=min(self.recovery_timeout, estimated_duration))
        else:
            recovery_duration = max(0.15, avoid_duration * self.recovery_duration_scale)
            recovery_duration = min(recovery_duration, self.recovery_timeout)
            self.recovery_deadline = now + Duration(seconds=recovery_duration)
            if not self.warned_missing_heading:
                self.warned_missing_heading = True
                self.log.warn('HEADING', 'imu heading unavailable, recovery falls back to timed reverse steering')

        deadline_sec = (self.recovery_deadline - now).nanoseconds / 1e9
        self.log.feedback(
            f'recovery start, uses_heading={self.recovery_uses_heading}, '
            f'deadline={deadline_sec:.2f}s'
        )

    def recovery_complete(self):
        now = self.get_clock().now()
        if self.recovery_uses_heading and self.current_yaw is not None and self.desired_heading is not None:
            if abs(self.angle_error(self.desired_heading, self.current_yaw)) <= self.heading_tolerance_rad:
                return True

        if self.recovery_deadline is not None and now >= self.recovery_deadline:
            return True

        return False

    def finish_recovery(self):
        if self.corridor_resume_after_avoidance and self.corridor_active:
            self.phase1_motion_state = 'corridor'
            self.corridor_nav_mode = 'corridor_path'
            self.corridor_resume_after_avoidance = False
        else:
            self.phase1_motion_state = 'forward'
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.recovery_uses_heading = False

        self.log.feedback('recovery complete, return to forward')

    def avoid_turn_reached(self):
        if self.current_yaw is None or self.avoid_entry_yaw is None:
            return True
        return abs(self.angle_error(self.current_yaw, self.avoid_entry_yaw)) >= self.avoid_min_turn_angle_rad

    def point_distance_xy(self, point_a, point_b):
        return math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])

    def collect_points_in_window(self, scan_msg, min_x, max_x, half_width):
        clusters = []
        current_cluster = []
        previous_point = None

        for index, distance in enumerate(scan_msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance < self.min_valid_range:
                if current_cluster:
                    clusters.append(current_cluster)
                    current_cluster = []
                previous_point = None
                continue

            angle = scan_msg.angle_min + index * scan_msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            if x < min_x or x > max_x or abs(y) > half_width:
                if current_cluster:
                    clusters.append(current_cluster)
                    current_cluster = []
                previous_point = None
                continue

            point = (x, y, distance)
            if previous_point is None or self.point_distance_xy(previous_point, point) <= self.phase1_cluster_gap_tolerance:
                current_cluster.append(point)
            else:
                if current_cluster:
                    clusters.append(current_cluster)
                current_cluster = [point]
            previous_point = point

        if current_cluster:
            clusters.append(current_cluster)

        return clusters

    def describe_cluster(self, cluster):
        nearest_distance = min(point[2] for point in cluster)
        center_x = sum(point[0] for point in cluster) / len(cluster)
        center_y = sum(point[1] for point in cluster) / len(cluster)
        span = self.point_distance_xy(cluster[0], cluster[-1])
        danger_angle_deg = math.degrees(math.atan2(center_y, max(center_x, 1e-6)))
        return {
            'distance': nearest_distance,
            'span': span,
            'danger_angle_deg': danger_angle_deg,
        }

    def find_phase1_forward_obstacle(self, scan_msg):
        clusters = self.collect_points_in_window(
            scan_msg,
            self.phase1_window_min_x,
            self.phase1_window_max_x,
            self.phase1_window_half_width,
        )

        # 发布所有聚类的可视化（rviz2 调试用）
        if clusters:
            self.obstacle_markers.publish_from_clusters(clusters, color='red')
            self._phase1_last_clusters = clusters
        else:
            self.obstacle_markers.clear()
            self._phase1_last_clusters = []

        nearest_obstacle = None
        for cluster in clusters:
            if len(cluster) < self.phase1_min_cluster_points:
                continue

            obstacle = self.describe_cluster(cluster)
            if obstacle['span'] < self.phase1_min_cluster_width:
                continue
            if obstacle['span'] > self.phase1_max_cluster_width:
                continue

            if nearest_obstacle is None or obstacle['distance'] < nearest_obstacle['distance']:
                nearest_obstacle = obstacle

        return nearest_obstacle

    def find_phase1_emergency_obstacle(self, scan_msg):
        clusters = self.collect_points_in_window(
            scan_msg,
            self.phase1_emergency_min_x,
            self.phase1_emergency_max_x,
            self.phase1_emergency_half_width,
        )

        nearest_obstacle = None
        for cluster in clusters:
            if len(cluster) < self.phase1_emergency_min_points:
                continue

            obstacle = self.describe_cluster(cluster)
            if nearest_obstacle is None or obstacle['distance'] < nearest_obstacle['distance']:
                nearest_obstacle = obstacle

        return nearest_obstacle

    def handle_phase1_lidar(self, scan_msg):
        # 启动宽限期：避免上电瞬间噪声/侧墙误触发
        if hasattr(self, '_node_start_time') and self.phase1_motion_state in ('forward', 'corridor'):
            grace = (self.get_clock().now() - self._node_start_time).nanoseconds / 1e9
            if grace < self.phase1_avoid_startup_grace_sec:
                self.obstacle_found = False
                self.closest_obstacle_distance = float('inf')
                if self.phase1_motion_state != 'recovering':
                    self.avoid_cmd = Twist()
                return

        # 避障优先级不变：通道中也会被障碍打断，恢复后继续 corridor
        if self.phase1_motion_state == 'avoiding':
            obstacle = self.find_phase1_emergency_obstacle(scan_msg)
        else:
            obstacle = self.find_phase1_forward_obstacle(scan_msg)

        if obstacle is not None:
            self.obstacle_found = True
            self.closest_obstacle_distance = obstacle['distance']

            if self.phase1_motion_state != 'avoiding':
                self.begin_avoidance(obstacle['danger_angle_deg'])
            else:
                self.avoid_clear_since = None

            if self.phase1_motion_state == 'avoiding':
                turn_direction = self.avoid_turn_direction
            else:
                turn_direction = -1.0 if obstacle['danger_angle_deg'] > 0.0 else 1.0

            self.avoid_cmd = self.create_twist(
                self.avoid_linear_speed,
                turn_direction * self.avoid_angular_speed,
            )
            return

        self.obstacle_found = False
        self.closest_obstacle_distance = float('inf')
        
        # 注：无障碍时清空 markers 已在 find_phase1_forward_obstacle() 中处理（line 388）
        # 这里不需要额外清空逻辑
        
        if self.phase1_motion_state == 'avoiding':
            now = self.get_clock().now()
            if self.avoid_clear_since is None:
                self.avoid_clear_since = now

            avoid_elapsed = 0.0
            if self.avoid_started_time is not None:
                avoid_elapsed = (now - self.avoid_started_time).nanoseconds / 1e9
            clear_elapsed = (now - self.avoid_clear_since).nanoseconds / 1e9

            if (
                avoid_elapsed >= self.avoid_min_duration_sec
                and clear_elapsed >= self.avoid_clear_hold_sec
                and self.avoid_turn_reached()
            ):
                self.begin_counter_steer()
            return

        if self.phase1_motion_state != 'recovering':
            self.avoid_cmd = Twist()

    def lidar_callback(self, msg):
        if self.phase == 1:
            was_corridor = self.phase1_motion_state == 'corridor'
            self.handle_phase1_lidar(msg)
            if was_corridor and self.phase1_motion_state == 'avoiding':
                self.corridor_resume_after_avoidance = True
            return

        min_dist = float('inf')
        danger_angle = 0.0
        found = False

        for index, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance < self.min_valid_range:
                continue

            angle_deg = math.degrees(msg.angle_min + index * msg.angle_increment)
            angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
            if abs(angle_deg) > self.scan_angle_deg:
                continue

            if distance < min_dist:
                min_dist = distance
                danger_angle = angle_deg
                found = distance < self.safe_distance

        if found:
            self.obstacle_found = True
            self.closest_obstacle_distance = min_dist
            if self.phase == 1 and self.phase1_motion_state != 'avoiding':
                self.begin_avoidance(danger_angle)
            elif self.phase == 1 and self.phase1_motion_state == 'avoiding':
                self.avoid_clear_since = None

            if self.phase == 1 and self.phase1_motion_state == 'avoiding':
                turn_direction = self.avoid_turn_direction
            else:
                turn_direction = -1.0 if danger_angle > 0.0 else 1.0

            self.avoid_cmd = self.create_twist(
                self.avoid_linear_speed,
                turn_direction * self.avoid_angular_speed,
            )
            return

        obstacle_cleared = min_dist > self.clear_distance or math.isinf(min_dist)
        self.obstacle_found = False
        self.closest_obstacle_distance = min_dist
        if self.phase == 1 and self.phase1_motion_state == 'avoiding':
            if not obstacle_cleared:
                self.avoid_clear_since = None
                return

            now = self.get_clock().now()
            if self.avoid_clear_since is None:
                self.avoid_clear_since = now

            avoid_elapsed = 0.0
            if self.avoid_started_time is not None:
                avoid_elapsed = (now - self.avoid_started_time).nanoseconds / 1e9
            clear_elapsed = (now - self.avoid_clear_since).nanoseconds / 1e9

            if (
                avoid_elapsed >= self.avoid_min_duration_sec
                and clear_elapsed >= self.avoid_clear_hold_sec
                and self.avoid_turn_reached()
            ):
                self.begin_counter_steer()
            return

        if self.phase != 1 or self.phase1_motion_state != 'recovering':
            self.avoid_cmd = Twist()

    def qr_callback(self, msg):
        if self.phase != 1:
            return

        if self.phase1_motion_state != 'forward':
            return

        task = msg.data.strip()
        if not task:
            return

        self.qr_task = task
        self.task_pub.publish(String(data=task))

        # 立即启动后退，不等待播报完成
        if self.enable_backing and len(self.path_record) > 0:
            self.phase1_motion_state = 'backing'
            self.backing_started_time = self.get_clock().now()
            self.backing_path_index = len(self.path_record) - 1
            self.log.mission(
                f'qr detected: {task}, backing mode, '
                f'{len(self.path_record)} waypoints recorded'
            )
        else:
            self.log.mission(f'qr detected: {task}, starting corridor navigation without backing')
            self.start_corridor_navigation(f'qr detected: {task}, no backing path')
        
        # 异步播报识别结果（后台线程执行，不阻塞后退）
        self._speak_qr_result_async(task)


    def stage2_state_callback(self, msg):
        self.stage2_state = msg.data.strip()
        if self.phase == 2 and self.stage2_state == 'complete':
            self.log.mission('stage2 complete, entering phase3')
            self.begin_phase_transition(3, 'stage2 complete, switched to phase3 return-to-p')

    def stage3_state_callback(self, msg):
        self.stage3_state = msg.data.strip()
        if self.phase == 3 and self.stage3_state == 'complete':
            self.mission_finished = True
            self.transition_end_time = None
            self.stop_robot()
            self.log.mission('stage3 complete, mission finished at p point')

    def stage2_cmd_callback(self, msg):
        self.latest_stage2_cmd = msg
        self.latest_stage2_cmd_time = self.get_clock().now()

    def stage2_cmd_is_fresh(self):
        if self.latest_stage2_cmd_time is None:
            return False

        age = self.get_clock().now() - self.latest_stage2_cmd_time
        return age.nanoseconds <= int(self.stage2_cmd_timeout * 1e9)

    def control_loop(self):
        if self.mission_finished:
            self.stop_robot()
            return

        if self.phase == 1:
            # backing 状态优先处理
            if self.phase1_motion_state == 'backing':
                self.handle_backing()
                return

            # 避障优先级最高（高于 corridor）
            if self.phase1_motion_state == 'avoiding':
                self.cmd_pub.publish(self.avoid_cmd)
                return

            if self.phase1_motion_state == 'countersteering':
                if self.counter_steer_deadline is not None and self.get_clock().now() >= self.counter_steer_deadline:
                    self.begin_recovery()
                    return

                self.cmd_pub.publish(
                    self.create_twist(
                        self.counter_steer_linear_speed,
                        -self.avoid_turn_direction * self.counter_steer_angular_speed,
                    )
                )
                return

            if self.phase1_motion_state == 'recovering':
                if self.recovery_complete():
                    self.finish_recovery()
                    self.cmd_pub.publish(self.create_twist(self.blind_linear_speed, self.blind_angular_speed))
                    return

                if self.recovery_uses_heading and self.current_yaw is not None and self.desired_heading is not None:
                    heading_error = self.angle_error(self.desired_heading, self.current_yaw)
                    angular_cmd = self.clamp(
                        self.recovery_heading_kp * heading_error,
                        self.recovery_max_angular_speed,
                    )
                    if abs(heading_error) > self.heading_tolerance_rad and abs(angular_cmd) < self.recovery_min_angular_speed:
                        angular_cmd = math.copysign(self.recovery_min_angular_speed, heading_error)

                    linear_cmd = self.recovery_turn_linear_speed
                    if abs(heading_error) <= self.recovery_in_place_angle_rad:
                        linear_cmd = self.recovery_linear_speed

                    self.cmd_pub.publish(self.create_twist(linear_cmd, angular_cmd))
                    return

                self.cmd_pub.publish(
                    self.create_twist(
                        self.recovery_linear_speed,
                        -self.avoid_turn_direction * self.recovery_angular_speed,
                    )
                )
                return

            if self.phase1_motion_state == 'corridor':
                self.handle_corridor_navigation()
                return

            # 盲开阶段 map_x 过大：向左旋回，避免贴右墙
            left_cmd = self.maybe_left_recover_cmd('blind_left_recover')
            if left_cmd is not None:
                self.cmd_pub.publish(left_cmd)
                return

            self.cmd_pub.publish(self.create_twist(self.blind_linear_speed, self.blind_angular_speed))
            return

        if self.phase2_obstacle_override and self.obstacle_found:
            self.cmd_pub.publish(self.avoid_cmd)
            return

        if self.obstacle_found and self.closest_obstacle_distance <= self.phase2_emergency_stop_distance:
            self.stop_robot()
            return

        if self.transition_end_time is not None and self.get_clock().now() < self.transition_end_time:
            self.stop_robot()
            return

        if self.phase == 3:
            if self.obstacle_found and self.closest_obstacle_distance <= self.phase3_emergency_stop_distance:
                self.stop_robot()
                return

            if self.phase3_external_control:
                return

            self.stop_robot()
            return

        if self.stage2_cmd_is_fresh():
            self.cmd_pub.publish(self.latest_stage2_cmd)
            return

        self.stop_robot()


    def handle_backing(self):
        """处理后退逻辑：沿记录路径反向跟踪"""
        if self.current_odom is None or self.current_yaw is None:
            self.stop_robot()
            return
        
        # 超时检查
        if self.backing_started_time is not None:
            elapsed = (self.get_clock().now() - self.backing_started_time).nanoseconds / 1e9
            if elapsed > self.back_timeout_sec:
                self.log.warn('BACKING', f'timeout after {elapsed:.1f}s, starting corridor navigation')
                self.start_corridor_navigation(f'qr task={self.qr_task}, backing timeout')
                return
        
        current_x = self.current_odom.pose.pose.position.x
        current_y = self.current_odom.pose.pose.position.y
        
        # 检查是否到达目标 x 位置（map 坐标系）
        if current_x <= self.back_target_x:
            self.log.segment(f'backing done at map_x={current_x:.2f}m, starting corridor navigation')
            self.start_corridor_navigation(f'qr task={self.qr_task}, backing complete')
            return
        
        # 检查路径是否倒序遍历完毕
        if self.backing_path_index < 0 or self.backing_path_index >= len(self.path_record):
            self.log.warn('BACKING', 'path exhausted, starting corridor navigation')
            self.start_corridor_navigation(f'qr task={self.qr_task}, backing path exhausted')
            return
        
        # 获取当前目标路点（包括来时的 yaw）
        target_x, target_y, target_yaw = self.path_record[self.backing_path_index]
        
        # 检查是否接近当前路点，若是则移动到上一个路点（倒序）
        dist_to_target = math.hypot(current_x - target_x, current_y - target_y)
        if dist_to_target < self.back_position_tolerance:
            self.backing_path_index -= 1
            self.log.progress(
                f'backing wp_index={self.backing_path_index}, '
                f'pos=({current_x:.2f}, {current_y:.2f}), '
                f'target_yaw={math.degrees(target_yaw):.1f}°'
            )
            if self.backing_path_index < 0:
                self.log.progress('backing reached start, starting corridor navigation')
                self.start_corridor_navigation(f'qr task={self.qr_task}, backing reached start')
                return
            target_x, target_y, target_yaw = self.path_record[self.backing_path_index]
        
        # 后退控制：车头朝向 = 路点记录的来时方向（精确复现轨迹）
        # 不使用实时几何方向 atan2(dy, dx)，而是直接用记录的 target_yaw
        heading_error = self.angle_error(target_yaw, self.current_yaw)
        
        angular_z = self.back_angular_kp * heading_error
        angular_z = self.clamp(angular_z, 1.0)
        
        # 倒车（负速度），车头保持来时方向
        self.cmd_pub.publish(self.create_twist(self.back_linear_speed, angular_z))
        
        self.log.progress(
            f'backing: wp={self.backing_path_index}, '
            f'current_x={current_x:.2f}m, '
            f'dist={dist_to_target:.2f}m, '
            f'target_yaw={math.degrees(target_yaw):.1f}°, '
            f'yaw_error={math.degrees(heading_error):.1f}°'
        )
    
    def handle_backing_align(self):
        """后退完成后对齐航向到指定角度，带超时"""
        if self.current_yaw is None:
            self.stop_robot()
            return

        # 超时检查
        if self.aligning_started_time is not None:
            elapsed = (self.get_clock().now() - self.aligning_started_time).nanoseconds / 1e9
            if elapsed > self.back_align_timeout_sec:
                self.log.warn('ALIGN', f'timeout after {elapsed:.1f}s, starting corridor navigation')
                self.start_corridor_navigation('backing align timeout')
                return

        heading_error = self.angle_error(self.back_align_yaw_rad, self.current_yaw)

        # 检查是否对齐完成
        if abs(heading_error) <= self.back_align_tolerance_rad:
            self.log.mission(
                f'backing align done at yaw={math.degrees(self.current_yaw):.1f}°, '
                f'switching to phase2'
            )
            self.start_corridor_navigation(f'qr task={self.qr_task}, backing+align complete')
            return

        # 对齐转向：大角度时原地转（linear_x=0），小角度时微速前进配合转向
        angular_z = self.clamp(self.recovery_heading_kp * heading_error, self.recovery_max_angular_speed)
        if abs(angular_z) < self.recovery_min_angular_speed:
            angular_z = math.copysign(self.recovery_min_angular_speed, heading_error)

        # 大角度（>30°）原地转，小角度（<8°）微速前进，中间角度慢速前进
        if abs(heading_error) > math.radians(30.0):
            linear_x = 0.0  # 原地转
        elif abs(heading_error) <= self.recovery_in_place_angle_rad:
            linear_x = self.recovery_linear_speed  # 0.12 m/s
        else:
            linear_x = self.recovery_turn_linear_speed  # 0.08 m/s

        self.log.feedback(
            f'aligning yaw={math.degrees(self.current_yaw):.1f}° '
            f'target=90° err={math.degrees(heading_error):.1f}° '
            f'cmd: linear={linear_x:.2f} angular={angular_z:.2f}'
        )
        self.cmd_pub.publish(self.create_twist(linear_x, angular_z))


    def _speak_qr_result(self, task):
        if self.tts_player is None:
            self.log.warn('VOICE', 'CN-TTS 模块未初始化，无法播报')
            return

        try:
            import re as re_local

            raw = str(task or '').strip()
            lowered = raw.lower()

            # 方向：优先文本关键词，其次按数字奇偶推断
            direction_text = ''
            if any(k in lowered for k in ('counterclockwise', 'anticlockwise', 'anti-clockwise', 'ccw')) or '逆时针' in raw:
                direction_text = '逆时针'
            elif any(k in lowered for k in ('clockwise', 'cw')) or '顺时针' in raw:
                direction_text = '顺时针'

            numbers = re_local.findall(r'\d+', raw)
            number_text = ''.join(numbers) if numbers else ''

            if not direction_text and number_text:
                # 无方向文本时，沿用赛事规则：奇=顺时针，偶=逆时针
                try:
                    numeric_value = int(number_text)
                    direction_text = '顺时针' if (numeric_value % 2 == 1) else '逆时针'
                except ValueError:
                    pass

            # 固定播报：数字 + 方向，例如 "1234 顺时针"
            # 注意：播报内容完全取决于二维码原文；原文无数字时只能播方向
            if number_text and direction_text:
                speak_text = f'{number_text} {direction_text}'
            elif number_text:
                speak_text = number_text
            elif direction_text:
                speak_text = direction_text
            else:
                speak_text = f'任务识别 {raw}'

            if not number_text:
                self.get_logger().warn(
                    f'[VOICE] 二维码原文无数字，无法播报编号。原文="{raw}"，仅播报方向/原文'
                )
            self.log.mission(f'QR播报开始: 原文="{raw}" → 播报="{speak_text}"')
            self.get_logger().info(f'[VOICE] 播报二维码: {speak_text}')

            self.tts_player.speak_text(speak_text)
            self.log.feedback(f'QR播报完成: "{speak_text}"')

        except Exception as e:
            self.log.error('VOICE', f'播报失败: {e}')
            self.get_logger().error(f'[VOICE] 播报异常: {e}')
    
    def _speak_qr_result_async(self, task):
        """异步播报二维码识别结果（后台线程执行）"""
        def speak_worker():
            self._speak_qr_result(task)
        
        thread = threading.Thread(target=speak_worker, daemon=True, name='TTS-QR-Broadcast')
        thread.start()
        self.log.progress(f'QR播报已启动后台线程，车辆开始后退')

    def destroy_node(self):
        self.log.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CompetitionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

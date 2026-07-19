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
from racing_common.yolo_bbox_detector import YoloBBoxDetector
from racing_stage1.stage1_vision_mixin import Stage1VisionMixin


class CompetitionController(Stage1VisionMixin, Node):
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
        self.declare_parameter('blind_left_search_x', 3.5)
        self.declare_parameter('blind_left_search_linear_speed', 0.12)
        self.declare_parameter('blind_left_search_angular_speed', 0.55)
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
        self.declare_parameter('back_turn_linear_speed', -0.12)
        self.declare_parameter('back_turn_slowdown_angle_deg', 18.0)
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
        self.declare_parameter('corridor_waypoints_json', '[{"x":2.50,"y":2.50}]')
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
        self.declare_parameter('corridor_entry_reorient_enabled', True)
        self.declare_parameter('corridor_entry_reorient_angle_deg', 50.0)
        self.declare_parameter('corridor_entry_reorient_done_deg', 25.0)
        self.declare_parameter('corridor_entry_reorient_timeout_sec', 4.0)
        self.declare_parameter('corridor_centerline_reorient_deg', 55.0)
        self.declare_parameter('corridor_avoid_while_reorient', False)
        self.declare_parameter('corridor_left_recover_x', 3.50)
        self.declare_parameter('corridor_left_recover_angular', 0.70)
        self.declare_parameter('corridor_left_recover_linear', 0.06)
        self.declare_parameter('corridor_lateral_kp', 1.8)
        self.declare_parameter('corridor_x_tolerance', 0.08)
        self.declare_parameter('corridor_heading_kp', 1.0)
        self.declare_parameter('corridor_stanley_k', 1.2)
        self.declare_parameter('corridor_align_linear_speed', 0.0)
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
        self.declare_parameter('use_corridor_planner', True)
        self.declare_parameter('planner_downsample', 4)
        self.declare_parameter('planner_occupied_threshold', 50)
        self.declare_parameter('planner_unknown_is_occupied', True)
        self.declare_parameter('planner_obstacle_inflation_m', 0.14)
        self.declare_parameter('planner_replan_period_sec', 2.5)
        # 区域进入：不要求精准到点/精准航向，进入入口区域即可切 Stage2
        self.declare_parameter('corridor_entry_region_radius_m', 0.35)
        self.declare_parameter('corridor_entry_yaw_tolerance_deg', 30.0)
        self.declare_parameter('corridor_require_yaw_for_release', True)
        self.declare_parameter('corridor_final_align_start_distance_m', 0.65)
        self.declare_parameter('corridor_final_align_min_speed', 0.16)
        self.declare_parameter('corridor_final_align_max_speed', 0.24)
        self.declare_parameter('corridor_final_align_heading_kp', 1.0)
        self.declare_parameter('corridor_final_align_lateral_kp', 0.15)
        self.declare_parameter('corridor_final_align_stable_sec', 0.12)
        self.declare_parameter('corridor_final_gate_x_tolerance_m', 0.45)
        self.declare_parameter('corridor_final_gate_y_before_m', 0.12)
        self.declare_parameter('corridor_final_gate_y_after_m', 0.16)
        self.declare_parameter('corridor_final_gate_yaw_tolerance_deg', 8.0)
        self.declare_parameter('corridor_final_overshoot_y_m', 0.22)
        self.declare_parameter('corridor_final_overshoot_yaw_tolerance_deg', 12.0)
        # 角速度死区 + 低通，抑制接近终点时频繁左右修角
        self.declare_parameter('corridor_angular_deadband', 0.06)
        self.declare_parameter('corridor_angular_filter_alpha', 0.30)
        self.declare_parameter('corridor_heading_hold_deg', 4.0)
        # 通道导航接近目标时禁用避障的距离阈值 (m)
        self.declare_parameter('corridor_disable_avoidance_distance_m', 0.60)
        self.declare_parameter('corridor_path_follow_mode', 'pure_pursuit')  # pure_pursuit | stanley
        self.declare_parameter('corridor_force_reorient_enabled', False)
        self.declare_parameter('corridor_pp_min_lookahead_m', 0.25)
        self.declare_parameter('corridor_pp_speed_scale', 1.0)
        self.declare_parameter('corridor_log_period_sec', 0.5)
        self.declare_parameter('corridor_replan_min_progress_m', 0.35)
        self.declare_parameter('corridor_replan_offpath_m', 0.28)
        self.declare_parameter('corridor_min_cruise_speed', 0.10)
        self.declare_parameter('corridor_max_turn_linear_speed', 0.08)
        # 通道 YOLO 中心对齐 + 阶段内 map 初始值重置（不发布/覆盖 /odom_combined）
        self.declare_parameter('channel_yolo_enabled', True)
        self.declare_parameter('channel_yolo_model_path', '/home/sunrise/dev_ws/best_rdk_tongdao.bin')
        self.declare_parameter('channel_yolo_camera_topic', '/aurora/rgb/image_raw')
        self.declare_parameter('channel_yolo_camera_info_topic', '/aurora/rgb/camera_info')
        self.declare_parameter('channel_yolo_camera_frame', 'camera')
        self.declare_parameter('channel_yolo_camera_frame_is_optical', True)
        self.declare_parameter('channel_yolo_bbox_anchor_v_ratio', 1.0)
        self.declare_parameter('channel_yolo_conf_thres', 0.25)
        self.declare_parameter('channel_yolo_iou_thres', 0.45)
        self.declare_parameter('channel_yolo_preview_path', '/tmp/stage1_channel_yolo.jpg')
        self.declare_parameter('channel_yolo_raw_path', '/tmp/stage1_channel_raw.jpg')
        self.declare_parameter('channel_yolo_http_port', 8081)
        self.declare_parameter('channel_yolo_trigger_y', 1.5)
        self.declare_parameter('channel_handoff_yaw_deg', 90.0)
        self.declare_parameter('channel_handoff_yaw_tolerance_deg', 5.0)
        self.declare_parameter('channel_reset_map_x', 2.5)
        self.declare_parameter('channel_reset_map_y', 2.5)
        self.declare_parameter('channel_reset_yaw_deg', 90.0)
        self.declare_parameter('channel_handoff_position_tolerance_m', 0.10)
        self.declare_parameter('channel_handoff_advance_m', 0.10)
        self.declare_parameter('channel_yolo_fallback_enabled', True)
        self.declare_parameter('channel_handoff_lateral_kp', 1.2)
        self.declare_parameter('channel_yolo_linear_speed', 0.18)
        self.declare_parameter('channel_yolo_chase_speed', 0.35)
        self.declare_parameter('channel_yolo_finish_speed', 0.08)
        self.declare_parameter('channel_yolo_max_angular', 0.35)
        self.declare_parameter('channel_yolo_yaw_correction_gain', 1.0)
        self.declare_parameter('channel_yolo_timeout_sec', 12.0)
        self.declare_parameter('channel_yolo_lost_timeout_sec', 0.30)
        self.declare_parameter('channel_yolo_finish_yaw_tolerance_deg', 3.0)
        # Visual handoff state machine: align to +Y, approach, then release at a map-Y gate.
        self.declare_parameter('channel_yolo_confirm_frames', 3)
        self.declare_parameter('channel_yolo_align_tolerance_deg', 8.0)
        self.declare_parameter('channel_yolo_align_speed', 0.08)
        self.declare_parameter('channel_yolo_lost_continue_sec', 6.0)
        self.declare_parameter('channel_handoff_release_y', 2.60)
        self.declare_parameter('channel_handoff_release_x', 2.50)
        self.declare_parameter('channel_handoff_release_x_tolerance_m', 0.35)
        self.declare_parameter('channel_handoff_release_yaw_tolerance_deg', 5.0)
        self.declare_parameter('channel_yolo_approach_max_distance_m', 2.40)

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
        self.blind_left_search_x = float(self.get_parameter('blind_left_search_x').value)
        self.blind_left_search_linear_speed = float(
            self.get_parameter('blind_left_search_linear_speed').value
        )
        self.blind_left_search_angular_speed = float(
            self.get_parameter('blind_left_search_angular_speed').value
        )
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
        self.back_turn_linear_speed = float(self.get_parameter('back_turn_linear_speed').value)
        self.back_turn_slowdown_angle_rad = math.radians(
            float(self.get_parameter('back_turn_slowdown_angle_deg').value)
        )
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
        self.corridor_entry_reorient_enabled = bool(self.get_parameter('corridor_entry_reorient_enabled').value)
        self.corridor_entry_reorient_angle = math.radians(
            float(self.get_parameter('corridor_entry_reorient_angle_deg').value)
        )
        self.corridor_entry_reorient_done = math.radians(
            float(self.get_parameter('corridor_entry_reorient_done_deg').value)
        )
        self.corridor_entry_reorient_timeout_sec = float(
            self.get_parameter('corridor_entry_reorient_timeout_sec').value
        )
        self.corridor_centerline_reorient = math.radians(
            float(self.get_parameter('corridor_centerline_reorient_deg').value)
        )
        self.corridor_avoid_while_reorient = bool(
            self.get_parameter('corridor_avoid_while_reorient').value
        )
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
        self.corridor_heading_kp = float(self.get_parameter('corridor_heading_kp').value)
        self.corridor_stanley_k = float(self.get_parameter('corridor_stanley_k').value)
        self.corridor_align_linear_speed = float(self.get_parameter('corridor_align_linear_speed').value)
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
        self.corridor_entry_region_radius = float(
            self.get_parameter('corridor_entry_region_radius_m').value
        )
        self.corridor_entry_yaw_tolerance = math.radians(
            float(self.get_parameter('corridor_entry_yaw_tolerance_deg').value)
        )
        self.corridor_require_yaw_for_release = bool(
            self.get_parameter('corridor_require_yaw_for_release').value
        )
        self.corridor_final_align_start_distance = float(
            self.get_parameter('corridor_final_align_start_distance_m').value
        )
        self.corridor_final_align_min_speed = float(
            self.get_parameter('corridor_final_align_min_speed').value
        )
        self.corridor_final_align_max_speed = float(
            self.get_parameter('corridor_final_align_max_speed').value
        )
        self.corridor_final_align_heading_kp = float(
            self.get_parameter('corridor_final_align_heading_kp').value
        )
        self.corridor_final_align_lateral_kp = float(
            self.get_parameter('corridor_final_align_lateral_kp').value
        )
        self.corridor_final_align_stable_sec = float(
            self.get_parameter('corridor_final_align_stable_sec').value
        )
        self.corridor_final_gate_x_tolerance = float(
            self.get_parameter('corridor_final_gate_x_tolerance_m').value
        )
        self.corridor_final_gate_y_before = float(
            self.get_parameter('corridor_final_gate_y_before_m').value
        )
        self.corridor_final_gate_y_after = float(
            self.get_parameter('corridor_final_gate_y_after_m').value
        )
        self.corridor_final_gate_yaw_tolerance = math.radians(
            float(self.get_parameter('corridor_final_gate_yaw_tolerance_deg').value)
        )
        self.corridor_final_overshoot_y = float(
            self.get_parameter('corridor_final_overshoot_y_m').value
        )
        self.corridor_final_overshoot_yaw_tolerance = math.radians(
            float(self.get_parameter('corridor_final_overshoot_yaw_tolerance_deg').value)
        )
        self.corridor_angular_deadband = float(
            self.get_parameter('corridor_angular_deadband').value
        )
        self.corridor_angular_filter_alpha = float(
            self.get_parameter('corridor_angular_filter_alpha').value
        )
        self.corridor_angular_filter_alpha = min(
            1.0, max(0.05, self.corridor_angular_filter_alpha)
        )
        self.corridor_heading_hold = math.radians(
            float(self.get_parameter('corridor_heading_hold_deg').value)
        )
        self.corridor_disable_avoidance_distance = float(
            self.get_parameter('corridor_disable_avoidance_distance_m').value
        )
        self.corridor_path_follow_mode = str(
            self.get_parameter('corridor_path_follow_mode').value
        ).strip().lower()
        self.corridor_force_reorient_enabled = bool(
            self.get_parameter('corridor_force_reorient_enabled').value
        )
        self.corridor_pp_min_lookahead = float(
            self.get_parameter('corridor_pp_min_lookahead_m').value
        )
        self.corridor_pp_speed_scale = float(
            self.get_parameter('corridor_pp_speed_scale').value
        )
        self.corridor_log_period_sec = float(
            self.get_parameter('corridor_log_period_sec').value
        )
        self.corridor_replan_min_progress_m = float(
            self.get_parameter('corridor_replan_min_progress_m').value
        )
        self.corridor_replan_offpath_m = float(
            self.get_parameter('corridor_replan_offpath_m').value
        )
        self.corridor_min_cruise_speed = float(
            self.get_parameter('corridor_min_cruise_speed').value
        )
        self.corridor_max_turn_linear_speed = float(
            self.get_parameter('corridor_max_turn_linear_speed').value
        )
        self.channel_yolo_enabled = bool(self.get_parameter('channel_yolo_enabled').value)
        self.channel_yolo_model_path = str(self.get_parameter('channel_yolo_model_path').value)
        self.channel_yolo_camera_topic = str(self.get_parameter('channel_yolo_camera_topic').value)
        self.channel_yolo_camera_info_topic = str(
            self.get_parameter('channel_yolo_camera_info_topic').value
        )
        self.channel_yolo_camera_frame = str(
            self.get_parameter('channel_yolo_camera_frame').value
        ).strip()
        self.channel_yolo_camera_frame_is_optical = bool(
            self.get_parameter('channel_yolo_camera_frame_is_optical').value
        )
        self.channel_yolo_bbox_anchor_v_ratio = float(
            self.get_parameter('channel_yolo_bbox_anchor_v_ratio').value
        )
        self.channel_yolo_conf_thres = float(self.get_parameter('channel_yolo_conf_thres').value)
        self.channel_yolo_iou_thres = float(self.get_parameter('channel_yolo_iou_thres').value)
        self.channel_yolo_preview_path = str(self.get_parameter('channel_yolo_preview_path').value)
        self.channel_yolo_raw_path = str(self.get_parameter('channel_yolo_raw_path').value)
        self.channel_yolo_http_port = int(self.get_parameter('channel_yolo_http_port').value)
        self.channel_yolo_trigger_y = float(self.get_parameter('channel_yolo_trigger_y').value)
        self.channel_handoff_yaw = math.radians(float(self.get_parameter('channel_handoff_yaw_deg').value))
        self.channel_handoff_yaw_tolerance = math.radians(
            float(self.get_parameter('channel_handoff_yaw_tolerance_deg').value)
        )
        self.channel_reset_map_x = float(self.get_parameter('channel_reset_map_x').value)
        self.channel_reset_map_y = float(self.get_parameter('channel_reset_map_y').value)
        self.channel_reset_yaw = math.radians(float(self.get_parameter('channel_reset_yaw_deg').value))
        self.channel_handoff_position_tolerance = float(
            self.get_parameter('channel_handoff_position_tolerance_m').value
        )
        self.channel_handoff_advance = float(
            self.get_parameter('channel_handoff_advance_m').value
        )
        self.channel_yolo_fallback_enabled = bool(
            self.get_parameter('channel_yolo_fallback_enabled').value
        )
        self.channel_handoff_lateral_kp = float(
            self.get_parameter('channel_handoff_lateral_kp').value
        )
        self.channel_yolo_linear_speed = float(self.get_parameter('channel_yolo_linear_speed').value)
        self.channel_yolo_chase_speed = float(self.get_parameter('channel_yolo_chase_speed').value)
        self.channel_yolo_finish_speed = float(self.get_parameter('channel_yolo_finish_speed').value)
        self.channel_yolo_max_angular = float(self.get_parameter('channel_yolo_max_angular').value)
        self.channel_yolo_yaw_correction_gain = float(
            self.get_parameter('channel_yolo_yaw_correction_gain').value
        )
        self.channel_yolo_timeout_sec = float(self.get_parameter('channel_yolo_timeout_sec').value)
        self.channel_yolo_lost_timeout_sec = float(
            self.get_parameter('channel_yolo_lost_timeout_sec').value
        )
        self.channel_yolo_finish_advance = self.channel_handoff_advance
        self.channel_yolo_finish_yaw_tolerance = math.radians(
            float(self.get_parameter('channel_yolo_finish_yaw_tolerance_deg').value)
        )
        self.channel_yolo_confirm_frames = max(
            1, int(self.get_parameter('channel_yolo_confirm_frames').value)
        )
        self.channel_yolo_align_tolerance = math.radians(
            float(self.get_parameter('channel_yolo_align_tolerance_deg').value)
        )
        self.channel_yolo_align_speed = float(
            self.get_parameter('channel_yolo_align_speed').value
        )
        self.channel_yolo_lost_continue_sec = float(
            self.get_parameter('channel_yolo_lost_continue_sec').value
        )
        self.channel_handoff_release_y = float(
            self.get_parameter('channel_handoff_release_y').value
        )
        self.channel_handoff_release_x = float(
            self.get_parameter('channel_handoff_release_x').value
        )
        self.channel_handoff_release_x_tolerance = float(
            self.get_parameter('channel_handoff_release_x_tolerance_m').value
        )
        self.channel_handoff_release_yaw_tolerance = math.radians(
            float(self.get_parameter('channel_handoff_release_yaw_tolerance_deg').value)
        )
        self.channel_yolo_approach_max_distance = float(
            self.get_parameter('channel_yolo_approach_max_distance_m').value
        )

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
        self._stage2_cmd_timeout_active = False
        self._last_stage2_timeout_log_sec = 0.0
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
        self.qr_processed = False  # 二维码去重标志：防止重复扫码播报
        self._map_pose_warned = False
        self.corridor_active = False
        self.corridor_nav_mode = 'idle'  # path_follow | left_recover | idle
        self.corridor_capture_active = False
        self.corridor_align_active = False
        self._node_start_time = self.get_clock().now()
        self.corridor_index = 0
        self.corridor_started_at = None
        self.corridor_path_points = []
        self.corridor_path_updated_at = 0.0
        self.corridor_resume_after_avoidance = False
        self.corridor_entry_reorient_active = False
        self.corridor_entry_reorient_started_at = None
        self.corridor_final_align_active = False
        self.corridor_final_align_since = None
        self.corridor_desired_heading = None
        self.corridor_planned_path = []
        self.corridor_path_cursor = 0
        self.corridor_last_plan_reason = ''
        self.corridor_planning_failures = 0
        self._corridor_timeout_logged = False
        self._corridor_last_log_time = 0.0
        self._corridor_last_detail_log_time = 0.0
        self._corridor_occ_cache_key = None
        self._corridor_occ_cache = None
        self._corridor_last_plan_pose = None
        self._corridor_plan_count = 0
        self._corridor_angular_cmd_filtered = 0.0
        self._channel_yolo_detector = None
        self._channel_handoff_step = 'idle'
        self._channel_handoff_started_at = None
        self._channel_map_target = None
        self._channel_visual_target_map = None
        self._channel_yolo_fallback_active = False
        self._channel_yolo_timeout_logged = False
        self._channel_yolo_lost_since = None
        self._channel_yolo_finish_start_pose = None
        self._channel_yolo_finish_aligned = False
        self._channel_yolo_chase_step = 'idle'
        self._channel_yolo_confirm_count = 0
        self._channel_yolo_confirm_timestamp = 0.0
        self._channel_yolo_last_detection_timestamp = 0.0
        self._map_pose_ready_logged = False
        self._channel_yolo_approach_start_odom = None
        self._channel_yolo_resume_state = None
        self._avoid_resume_state = None
        self._stage1_manual_map_active = False
        self._stage1_map_odom_x = 0.0
        self._stage1_map_odom_y = 0.0
        self._stage1_map_odom_yaw = 0.0
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.corridor_path_pub = self.create_publisher(Path, self.corridor_path_topic, path_qos)

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

        # 视觉通道导航初始化
        self._setup_vision_corridor()
        self._setup_channel_yolo_detector()

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

    def corridor_segment_heading(self, pose_xy=None, goal_xy=None):
        """通道当前段期望航向：优先 LOS，失败时回落 corridor_goal_yaw。"""
        if pose_xy is not None and goal_xy is not None:
            dx = float(goal_xy[0]) - float(pose_xy[0])
            dy = float(goal_xy[1]) - float(pose_xy[1])
            if math.hypot(dx, dy) > 1e-3:
                return math.atan2(dy, dx)
        if (
            self.corridor_waypoints
            and 0 <= self.corridor_index < len(self.corridor_waypoints)
        ):
            waypoint = self.corridor_waypoints[self.corridor_index]
            if 'yaw_deg' in waypoint:
                return math.radians(float(waypoint['yaw_deg']))
        return float(self.corridor_goal_yaw)

    def update_corridor_desired_heading(self, pose_xy=None, goal_xy=None):
        heading = self.corridor_segment_heading(pose_xy=pose_xy, goal_xy=goal_xy)
        self.corridor_desired_heading = heading
        # 通道内 recovery 必须跟通道航向，禁止回锁 phase1 盲开航向
        self.desired_heading = heading
        return heading

    def begin_corridor_entry_reorient(self, reason, pose_xy=None, goal_xy=None):
        heading = self.update_corridor_desired_heading(pose_xy=pose_xy, goal_xy=goal_xy)
        yaw = self.current_yaw if self.current_yaw is not None else 0.0
        # 倒车结束车头几乎反向时，直接以通道终航向 90° 为重定向目标，更稳
        if self.current_yaw is not None and abs(self.angle_error(heading, yaw)) >= math.radians(90.0):
            heading = float(self.corridor_goal_yaw)
            self.corridor_desired_heading = heading
            self.desired_heading = heading
        self.corridor_entry_reorient_active = True
        self.corridor_entry_reorient_started_at = self.get_clock().now().nanoseconds / 1e9
        self.corridor_nav_mode = 'reorient'
        err = self.angle_error(heading, yaw) if self.current_yaw is not None else 0.0
        self.log.progress(
            f'corridor reorient enter: {reason}, '
            f'target={math.degrees(heading):.1f}° '
            f'yaw={math.degrees(yaw):.1f}° '
            f'err={math.degrees(err):.1f}°'
        )
        return heading

    def finish_corridor_entry_reorient(self, reason):
        self.corridor_entry_reorient_active = False
        self.corridor_entry_reorient_started_at = None
        self.corridor_final_align_active = False
        self.corridor_final_align_since = None
        if self.corridor_nav_mode == 'reorient':
            self.corridor_nav_mode = 'centerline'
        yaw = self.current_yaw if self.current_yaw is not None else 0.0
        target = self.corridor_desired_heading if self.corridor_desired_heading is not None else self.corridor_goal_yaw
        err = self.angle_error(target, yaw) if self.current_yaw is not None else 0.0
        self.log.progress(
            f'corridor reorient done: {reason}, '
            f'yaw={math.degrees(yaw):.1f}° '
            f'target={math.degrees(target):.1f}° '
            f'err={math.degrees(err):.1f}°'
        )

    def handle_corridor_entry_reorient(self, pose_xy, goal_xy, now_ts):
        # 倒车结束车头几乎反向时，优先对准通道终航向 90°；否则跟当前段 LOS
        if abs(self.angle_error(self.corridor_goal_yaw, self.current_yaw)) >= math.radians(90.0):
            target_heading = float(self.corridor_goal_yaw)
            self.corridor_desired_heading = target_heading
            self.desired_heading = target_heading
        else:
            target_heading = self.update_corridor_desired_heading(pose_xy=pose_xy, goal_xy=goal_xy)

        yaw = float(self.current_yaw)
        # 始终走最短角；即使大角度也保持低速正向，不允许 v=0 时转向。
        heading_error = self.angle_error(target_heading, yaw)
        started = self.corridor_entry_reorient_started_at
        elapsed = (now_ts - started) if started is not None else 0.0
        abs_err = abs(heading_error)

        if abs_err <= self.corridor_entry_reorient_done:
            self.finish_corridor_entry_reorient('aligned')
            return False
        if elapsed >= self.corridor_entry_reorient_timeout_sec:
            self.finish_corridor_entry_reorient(f'timeout {elapsed:.1f}s')
            return False

        self.corridor_nav_mode = 'reorient'
        angular = self.clamp(self.corridor_alpha_kp * heading_error, self.turn_angular_speed)
        if abs_err > self.corridor_entry_reorient_done and abs(angular) < self.turn_min_angular_speed:
            angular = math.copysign(self.turn_min_angular_speed, heading_error if heading_error != 0.0 else 1.0)

        # 大角度用极低速圆弧修正，误差收敛后提高到蠕行速度。
        if abs_err < math.radians(45.0):
            linear = max(self.corridor_creep_speed, self.corridor_final_align_min_speed)
        else:
            linear = max(self.corridor_final_align_min_speed, min(self.corridor_creep_speed, 0.06))

        if now_ts - getattr(self, '_reorient_log_time', 0.0) >= 0.5:
            self._reorient_log_time = now_ts
            self.log.progress(
                f'reorient: target={math.degrees(target_heading):.1f}° '
                f'yaw={math.degrees(yaw):.1f}° err={math.degrees(heading_error):.1f}° '
                f'v={linear:.2f} w={angular:.2f} t={elapsed:.1f}s'
            )
        self.cmd_pub.publish(self.create_twist(linear, angular))
        return True

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

    def _smooth_corridor_angular(self, angular, heading_error=None):
        """死区 + 一阶低通，避免接近终点时角速度频繁左右抖动。"""
        cmd = float(angular)
        if heading_error is not None and abs(heading_error) <= self.corridor_heading_hold:
            cmd = 0.0
        if abs(cmd) < self.corridor_angular_deadband:
            cmd = 0.0
        alpha = self.corridor_angular_filter_alpha
        self._corridor_angular_cmd_filtered = (
            (1.0 - alpha) * self._corridor_angular_cmd_filtered + alpha * cmd
        )
        # 死区后滤波结果若仍极小，直接置 0，避免“微抖转向”
        if abs(self._corridor_angular_cmd_filtered) < 0.5 * self.corridor_angular_deadband:
            self._corridor_angular_cmd_filtered = 0.0
        return self._corridor_angular_cmd_filtered

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
        """接收里程消息；倒退路径只记录 map 位姿，IMU 只提供航向。"""
        self.current_odom = msg
        
        # Phase 1 前进时记录路径（只在 forward/avoiding/countersteering/recovering 时记录）
        # 位置统一为 map (x, y)，角度统一为 IMU (self.current_yaw)。
        if self.phase == 1 and self.enable_backing and self.phase1_motion_state in ('forward', 'avoiding', 'countersteering', 'recovering'):
            map_xy = self._get_strict_map_position()
            if map_xy is None:
                return
            x, y = map_xy
            if not self._map_pose_ready_logged:
                raw = self.current_odom.pose.pose.position
                self._map_pose_ready_logged = True
                self.log.startup(
                    f'map pose ready: odom=({float(raw.x):.2f},{float(raw.y):.2f}) '
                    f'-> map=({x:.2f},{y:.2f}); '
                    'Stage1 path recording uses map only'
                )
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

    def _manual_map_xy_from_odom(self):
        """阶段内 map 初始值覆盖：只影响本节点的 map xy 计算。"""
        if not self._stage1_manual_map_active or self.current_odom is None:
            return None
        pos = self.current_odom.pose.pose.position
        return self._transform_xy(
            self._stage1_map_odom_x,
            self._stage1_map_odom_y,
            self._stage1_map_odom_yaw,
            float(pos.x),
            float(pos.y),
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
        """Return map xy only; raw odometry must never be treated as map."""
        map_xy = self._get_strict_map_position()
        if map_xy is not None:
            return map_xy

        if not self._map_pose_warned:
            self._map_pose_warned = True
            self.log.warn(
                'POSE',
                f'map pose unavailable; holding map-dependent control until TF '
                f'{self.map_frame}->{self.odom_frame}->{self.base_frame} is ready. '
                f'Raw {self.odom_topic} coordinates are not used as map.',
            )
        return None

    def _get_strict_map_position(self):
        """Return map xy for backing; never fall back to raw odom coordinates."""
        manual_xy = self._manual_map_xy_from_odom()
        if manual_xy is not None:
            return manual_xy
        map_xy = self._lookup_map_xy_from_tf()
        if map_xy is not None:
            return map_xy
        if self.current_odom is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = self.quaternion_to_yaw(q)
        pos = self.current_odom.pose.pose.position
        return self._transform_xy(float(t.x), float(t.y), yaw, float(pos.x), float(pos.y))

    def _setup_channel_yolo_detector(self):
        if not self.channel_yolo_enabled:
            self.log.startup('Stage1 channel YOLO handoff disabled')
            return
        try:
            if not os.path.exists(self.channel_yolo_model_path):
                self.channel_yolo_enabled = False
                self.log.warn('CHANNEL_YOLO', f'model not found: {self.channel_yolo_model_path}')
                return
            self._channel_yolo_detector = YoloBBoxDetector(
                self,
                model_path=self.channel_yolo_model_path,
                camera_topic=self.channel_yolo_camera_topic,
                camera_info_topic=self.channel_yolo_camera_info_topic,
                target_name='stage1_channel',
                conf_thres=self.channel_yolo_conf_thres,
                iou_thres=self.channel_yolo_iou_thres,
                jpeg_output_path=self.channel_yolo_preview_path,
                raw_output_path=self.channel_yolo_raw_path,
                http_port=self.channel_yolo_http_port,
            )
            self._channel_yolo_detector.set_inference_active(False)
            self.log.startup(
                f'Stage1 channel YOLO enabled model={self.channel_yolo_model_path} '
                f'trigger_y={self.channel_yolo_trigger_y:.2f} reset='
                f'({self.channel_reset_map_x:.2f},{self.channel_reset_map_y:.2f})'
            )
        except Exception as e:
            self.channel_yolo_enabled = False
            self._channel_yolo_detector = None
            self.log.warn('CHANNEL_YOLO', f'init failed, disabled: {e}')

    def _set_channel_yolo_active(self, active):
        detector = getattr(self, '_channel_yolo_detector', None)
        if detector is not None:
            detector.set_inference_active(active)

    def _channel_yolo_has_fresh_detection(self, now_ts):
        detector = getattr(self, '_channel_yolo_detector', None)
        if detector is None:
            return False
        geometry = detector.get_detection_geometry()
        timestamp = float(geometry.get('timestamp') or 0.0)
        return bool(geometry.get('detected')) and timestamp > 0.0 and (
            now_ts - timestamp <= self.channel_yolo_lost_timeout_sec
        )

    def _channel_yolo_detection_confirmed(self, now_ts):
        """Require distinct positive inference frames before interrupting backing."""
        detector = getattr(self, '_channel_yolo_detector', None)
        if detector is None:
            return False
        geometry = detector.get_detection_geometry()
        timestamp = float(geometry.get('timestamp') or 0.0)
        if not geometry.get('detected') or timestamp <= 0.0 or (
            now_ts - timestamp > self.channel_yolo_lost_timeout_sec
        ):
            self._channel_yolo_confirm_count = 0
            return False
        if timestamp != self._channel_yolo_confirm_timestamp:
            self._channel_yolo_confirm_timestamp = timestamp
            self._channel_yolo_confirm_count += 1
        return self._channel_yolo_confirm_count >= self.channel_yolo_confirm_frames

    def begin_channel_yolo_chase(self, reason):
        if self.phase1_motion_state == 'channel_yolo_chase':
            return
        self.phase1_motion_state = 'channel_yolo_chase'
        self._channel_yolo_lost_since = None
        self._channel_yolo_chase_step = 'align_yaw'
        self._channel_yolo_approach_start_odom = None
        self._set_channel_yolo_active(True)
        self.log.mission(
            f'channel YOLO confirmed: {reason}, align to '
            f'{math.degrees(self.channel_handoff_yaw):.1f}deg before fast approach'
        )

    def _channel_forward_progress(self, pose_xy):
        if self.current_odom is None or self._channel_yolo_finish_start_pose is None:
            return 0.0
        pos = self.current_odom.pose.pose.position
        dx = float(pos.x) - self._channel_yolo_finish_start_pose[0]
        dy = float(pos.y) - self._channel_yolo_finish_start_pose[1]
        # /odom_combined 只用于位移计数；不使用其 orientation。
        return math.hypot(dx, dy)

    def _begin_channel_yolo_finish(self, now_ts):
        if self.phase1_motion_state == 'channel_yolo_finish':
            return
        self.phase1_motion_state = 'channel_yolo_finish'
        self._channel_yolo_lost_since = now_ts
        self._channel_yolo_finish_aligned = False
        if self.current_odom is not None:
            pos = self.current_odom.pose.pose.position
            self._channel_yolo_finish_start_pose = (float(pos.x), float(pos.y))
        else:
            self._channel_yolo_finish_start_pose = None
        self.log.mission(
            'channel YOLO lost: finish with forward motion, '
            f'advance={self.channel_yolo_finish_advance:.2f}m '
            f'yaw_tol={math.degrees(self.channel_yolo_finish_yaw_tolerance):.1f}deg'
        )

    def handle_channel_yolo_chase(self):
        now_ts = self.get_clock().now().nanoseconds / 1e9
        yaw = self.current_yaw
        if yaw is None:
            self.stop_robot()
            return
        pose_xy = self.get_map_position()
        if pose_xy is None:
            self.stop_robot()
            return

        yaw_error = self.angle_error(self.channel_handoff_yaw, yaw)
        x_error = pose_xy[0] - self.channel_handoff_release_x
        release_ready = (
            pose_xy[1] >= self.channel_handoff_release_y
            and abs(x_error) <= self.channel_handoff_release_x_tolerance
            and abs(yaw_error) <= self.channel_handoff_release_yaw_tolerance
        )
        if release_ready:
            self._set_channel_yolo_active(False)
            self.stop_robot()
            self._finish_corridor_release(
                pose_xy,
                (self.channel_handoff_release_x, self.channel_handoff_release_y),
                yaw,
                f'channel Y gate reached map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                f'x_err={x_error:+.2f}m yaw_err={math.degrees(yaw_error):+.1f}deg',
            )
            return

        if self._channel_yolo_chase_step == 'align_yaw':
            angular = self.clamp(
                self.channel_yolo_yaw_correction_gain * yaw_error,
                self.channel_yolo_max_angular,
            )
            if abs(yaw_error) <= self.channel_yolo_align_tolerance:
                self._channel_yolo_chase_step = 'approach'
                if self.current_odom is not None:
                    pos = self.current_odom.pose.pose.position
                    self._channel_yolo_approach_start_odom = (float(pos.x), float(pos.y))
                self.log.mission(
                    f'channel YOLO yaw aligned={math.degrees(yaw):.1f}deg; '
                    f'fast approach to map_y={self.channel_handoff_release_y:.2f}'
                )
            else:
                self.cmd_pub.publish(self.create_twist(self.channel_yolo_align_speed, angular))
                return

        approach_distance = None
        if self.current_odom is not None and self._channel_yolo_approach_start_odom is not None:
            pos = self.current_odom.pose.pose.position
            approach_distance = math.hypot(
                float(pos.x) - self._channel_yolo_approach_start_odom[0],
                float(pos.y) - self._channel_yolo_approach_start_odom[1],
            )
            if approach_distance >= self.channel_yolo_approach_max_distance:
                self.stop_robot()
                self._set_channel_yolo_active(False)
                self.log.warn(
                    'CHANNEL_YOLO',
                    f'fast approach distance limit {approach_distance:.2f}/'
                    f'{self.channel_yolo_approach_max_distance:.2f}m before Y gate; '
                    'falling back to corridor navigation',
                )
                self.start_corridor_navigation('channel YOLO approach distance limit')
                return

        detector = self._channel_yolo_detector
        geometry = detector.get_detection_geometry() if detector is not None else {}
        timestamp = float(geometry.get('timestamp') or 0.0)
        fresh = bool(geometry.get('detected')) and timestamp > 0.0 and (
            now_ts - timestamp <= self.channel_yolo_lost_timeout_sec
        )
        if not fresh:
            if self._channel_yolo_lost_since is None:
                self._channel_yolo_lost_since = now_ts
                self.log.warn('CHANNEL_YOLO', 'bbox lost during approach; holding IMU +Y')
            if now_ts - self._channel_yolo_lost_since >= self.channel_yolo_lost_continue_sec:
                self.log.warn(
                    'CHANNEL_YOLO',
                    f'bbox lost for {self.channel_yolo_lost_continue_sec:.1f}s before Y gate; '
                    'falling back to corridor navigation',
                )
                self._set_channel_yolo_active(False)
                self.start_corridor_navigation('channel YOLO lost before Y-gate release')
                return
            angular = self.clamp(
                self.channel_yolo_yaw_correction_gain * yaw_error,
                self.channel_yolo_max_angular,
            )
            self.cmd_pub.publish(self.create_twist(self.channel_yolo_chase_speed, angular))
            return

        self._channel_yolo_lost_since = None
        offset = float(getattr(detector, 'get_detection', lambda: (False, 0, None, 0, 0, 0))()[4])
        angular = (
            self.channel_yolo_yaw_correction_gain * yaw_error
            - self.channel_handoff_lateral_kp * offset
        )
        angular = self.clamp(angular, self.channel_yolo_max_angular)
        self.cmd_pub.publish(self.create_twist(self.channel_yolo_chase_speed, angular))
        if now_ts - getattr(self, '_channel_chase_log_time', 0.0) >= 0.5:
            self._channel_chase_log_time = now_ts
            self.log.segment(
                f'channel chase bbox_offset={offset:+.3f} '
                f'yaw_err={math.degrees(yaw_error):+.1f}deg '
                f'map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                f'y_gate={self.channel_handoff_release_y:.2f} '
                f'approach={(approach_distance if approach_distance is not None else float("nan")):.2f}m '
                f'v={self.channel_yolo_chase_speed:.2f} w={angular:+.2f}'
            )

    def handle_channel_yolo_finish(self):
        # Compatibility for an interrupted legacy state: finish now uses the
        # same bounded Y-gate controller rather than a blind distance segment.
        self.handle_channel_yolo_chase()

    def begin_channel_yolo_handoff(self, pose_xy, reason):
        if self.phase1_motion_state == 'channel_yolo_handoff':
            return
        self.phase1_motion_state = 'channel_yolo_handoff'
        self.corridor_active = False
        self.corridor_nav_mode = 'channel_yolo'
        self.corridor_final_align_active = False
        self.corridor_final_align_since = None
        self._channel_handoff_step = 'align_yaw'
        self._channel_handoff_started_at = self.get_clock().now().nanoseconds / 1e9
        self._channel_map_target = None
        self._channel_visual_target_map = None
        self._channel_yolo_fallback_active = False
        self._channel_yolo_timeout_logged = False
        self._set_channel_yolo_active(False)
        self.stop_robot()
        self.log.mission(
            f'channel YOLO handoff start: {reason}, map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
            f'target_yaw={math.degrees(self.channel_handoff_yaw):.1f}deg'
        )

    def _reset_stage1_local_map_origin(self):
        if self.current_odom is None:
            return False
        pos = self.current_odom.pose.pose.position
        raw_x = float(pos.x)
        raw_y = float(pos.y)
        # 只重置本节点的 map 初始值；不发布 /odom_combined，不改全局 TF。
        yaw_offset = self.normalize_angle(self.channel_reset_yaw - (self.current_yaw or self.channel_reset_yaw))
        cos_y = math.cos(yaw_offset)
        sin_y = math.sin(yaw_offset)
        self._stage1_map_odom_x = self.channel_reset_map_x - (cos_y * raw_x - sin_y * raw_y)
        self._stage1_map_odom_y = self.channel_reset_map_y - (sin_y * raw_x + cos_y * raw_y)
        self._stage1_map_odom_yaw = yaw_offset
        self._stage1_manual_map_active = True
        self.corridor_path_points = []
        self.corridor_planned_path = []
        self.corridor_path_cursor = 0
        self.corridor_path_updated_at = 0.0
        self.log.mission(
            f'Stage1 local map origin reset: odom=({raw_x:.3f},{raw_y:.3f}) '
            f'imu_yaw={(math.degrees(self.current_yaw) if self.current_yaw is not None else float("nan")):.1f}deg '
            f'-> map=({self.channel_reset_map_x:.2f},{self.channel_reset_map_y:.2f}) '
            f'yaw_offset={math.degrees(yaw_offset):.1f}deg'
        )
        return True

    @staticmethod
    def _rotate_vector_by_quaternion(vector, rotation):
        """Rotate a 3D vector by a geometry_msgs Quaternion."""
        qx = float(rotation.x)
        qy = float(rotation.y)
        qz = float(rotation.z)
        qw = float(rotation.w)
        vx, vy, vz = (float(value) for value in vector)
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return (
            vx + qw * tx + qy * tz - qz * ty,
            vy + qw * ty + qz * tx - qx * tz,
            vz + qw * tz + qx * ty - qy * tx,
        )

    def _channel_target_base_from_detection(self, geometry):
        """Project a configurable bbox vertical anchor onto the ground plane.

        The bbox center describes the object, not a point on the floor.  For a
        ground intersection the default anchor is the bottom-center of the
        detection; the anchor ratio remains configurable for camera/model
        calibration.
        """
        bbox = geometry.get('bbox')
        camera_info = geometry.get('camera_info')
        reported_frame = geometry.get('frame_id') or (
            camera_info or {}
        ).get('frame_id', '')
        if bbox is None or camera_info is None:
            self._channel_geometry_error = 'missing bbox or camera_info'
            return None
        candidates = []
        for frame in (reported_frame, self.channel_yolo_camera_frame):
            frame = str(frame).strip()
            if frame and frame not in candidates:
                candidates.append(frame)
        if not candidates:
            self._channel_geometry_error = 'camera frame_id is empty'
            return None
        fx = float(camera_info['fx'])
        fy = float(camera_info['fy'])
        cx = float(camera_info['cx'])
        cy = float(camera_info['cy'])
        if fx <= 0.0 or fy <= 0.0:
            return None

        x1, y1, x2, y2 = bbox
        pixel_u = 0.5 * (float(x1) + float(x2))
        anchor_ratio = self.clamp(self.channel_yolo_bbox_anchor_v_ratio, 0.0, 1.0)
        pixel_v = float(y1) + anchor_ratio * (float(y2) - float(y1))
        normalized_x = (pixel_u - cx) / fx
        normalized_y = (pixel_v - cy) / fy
        transform = None
        camera_frame = ''
        errors = []
        for candidate in candidates:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    candidate,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
                camera_frame = candidate
                break
            except TransformException as exc:
                errors.append(f'{candidate}: {exc}')
        if transform is None:
            self._channel_geometry_error = (
                f'TF {self.base_frame}<-{", ".join(candidates)} unavailable; '
                f'configured_frame={self.channel_yolo_camera_frame or "<empty>"}'
            )
            return None

        # URDF's `camera` link uses the robot convention (+X forward, +Y left,
        # +Z up), while CameraInfo rays use optical convention (+Z forward,
        # +X right, +Y down).  Optical frames need no conversion here.
        camera_is_optical = self.channel_yolo_camera_frame_is_optical
        if camera_frame in ('camera', 'camera_link', 'base_link'):
            camera_is_optical = False
        if camera_is_optical:
            ray_camera = (normalized_x, normalized_y, 1.0)
        else:
            ray_camera = (1.0, -normalized_x, -normalized_y)

        translation = transform.transform.translation
        direction = self._rotate_vector_by_quaternion(
            ray_camera, transform.transform.rotation
        )
        origin = (float(translation.x), float(translation.y), float(translation.z))
        if direction[2] >= -1e-4:
            self._channel_geometry_error = (
                f'ground ray points upward frame={camera_frame} '
                f'direction=({direction[0]:.3f},{direction[1]:.3f},{direction[2]:.3f})'
            )
            return None
        scale = -origin[2] / direction[2]
        if scale <= 0.0:
            self._channel_geometry_error = f'ground intersection is behind camera frame={camera_frame}'
            return None
        point = tuple(origin[index] + scale * direction[index] for index in range(3))
        self._channel_geometry_error = f'using camera frame={camera_frame}'
        return point[0], point[1]

    def _channel_target_map_from_base(self, target_base):
        """Transform a detected base-frame point into map using the full TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        point_map = self._rotate_vector_by_quaternion(
            (target_base[0], target_base[1], 0.0),
            transform.transform.rotation,
        )
        return (
            float(translation.x) + point_map[0],
            float(translation.y) + point_map[1],
        )

    def _drive_to_channel_map_target(self, pose_xy, yaw, now_ts):
        visual_x, visual_y = self._channel_visual_target_map
        target_x, target_y = self._channel_map_target
        target_dx = visual_x - pose_xy[0]
        target_dy = visual_y - pose_xy[1]
        heading = self.channel_handoff_yaw
        cos_heading = math.cos(heading)
        sin_heading = math.sin(heading)
        # advance_progress is traveled distance after the locked visual
        # center.  Keep the target-to-vehicle vector separate for lateral
        # steering so the release test cannot fire before the 10 cm advance.
        advance_progress = (
            (pose_xy[0] - visual_x) * cos_heading
            + (pose_xy[1] - visual_y) * sin_heading
        )
        lateral_error = -sin_heading * target_dx + cos_heading * target_dy
        yaw_error = self.angle_error(self.channel_handoff_yaw, yaw)
        if (
            advance_progress >= self.channel_handoff_advance
            and abs(lateral_error) <= self.channel_handoff_position_tolerance
            and abs(yaw_error) <= self.channel_handoff_yaw_tolerance
        ):
            self.stop_robot()
            self._set_channel_yolo_active(False)
            self._reset_stage1_local_map_origin()
            self._finish_corridor_release(
                pose_xy,
                (target_x, target_y),
                yaw,
                f'channel map target reached map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                f'target=({target_x:.2f},{target_y:.2f}) '
                f'advance={advance_progress:.2f}m lateral={lateral_error:.2f}m '
                f'yaw_err={math.degrees(yaw_error):.1f}deg',
            )
            return

        # Drive only toward the fixed map heading.  Once the visual target is
        # passed, longitudinal error no longer changes the steering command,
        # so the controller cannot turn around and orbit the target.
        angular = (
            self.channel_yolo_yaw_correction_gain * yaw_error
            + self.channel_handoff_lateral_kp * lateral_error
        )
        angular = self.clamp(angular, self.channel_yolo_max_angular)
        speed = self.channel_yolo_linear_speed
        self.cmd_pub.publish(self.create_twist(speed, angular))
        if now_ts - getattr(self, '_channel_yolo_log_time', 0.0) >= 0.5:
            self._channel_yolo_log_time = now_ts
            self.log.segment(
                f'channel_map_target map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                f'target=({target_x:.2f},{target_y:.2f}) '
                f'advance={advance_progress:.2f}/{self.channel_handoff_advance:.2f}m '
                f'lateral={lateral_error:.2f}m yaw_err={math.degrees(yaw_error):.1f}deg '
                f'v={speed:.2f} w={angular:.2f}'
            )

    def _start_channel_map_fallback(self, now_ts):
        """Use the configured map handoff when visual detection is unavailable."""
        heading = self.channel_handoff_yaw
        self._channel_visual_target_map = (
            self.channel_reset_map_x - self.channel_handoff_advance * math.cos(heading),
            self.channel_reset_map_y - self.channel_handoff_advance * math.sin(heading),
        )
        self._channel_map_target = (self.channel_reset_map_x, self.channel_reset_map_y)
        self._channel_handoff_step = 'forward_to_handoff'
        self._channel_yolo_fallback_active = True
        elapsed = now_ts - (self._channel_handoff_started_at or now_ts)
        self.log.warn(
            'CHANNEL_YOLO',
            f'no bbox after {elapsed:.1f}s; fallback to YAML map handoff '
            f'({self.channel_reset_map_x:.2f},{self.channel_reset_map_y:.2f})',
        )

    def handle_channel_yolo_handoff(self):
        now_ts = self.get_clock().now().nanoseconds / 1e9
        started = self._channel_handoff_started_at or now_ts
        timed_out = now_ts - started > self.channel_yolo_timeout_sec
        if timed_out:
            if not self._channel_yolo_timeout_logged:
                self._channel_yolo_timeout_logged = True
                self.log.warn('CHANNEL_YOLO', 'handoff timeout: detector has no valid bbox')

        yaw = self.current_yaw
        if yaw is None:
            self.stop_robot()
            return

        if (
            timed_out
            and self.channel_yolo_fallback_enabled
            and self._channel_map_target is None
            and not self._channel_yolo_fallback_active
        ):
            self._start_channel_map_fallback(now_ts)

        if self._channel_handoff_step == 'align_yaw':
            err = self.angle_error(self.channel_handoff_yaw, yaw)
            if abs(err) <= self.channel_handoff_yaw_tolerance:
                self._channel_handoff_step = 'detect_and_lock'
                self._set_channel_yolo_active(True)
                self.log.mission(
                    f'channel yaw aligned yaw={math.degrees(yaw):.1f}deg, enabling YOLO'
                )
                return
            angular = self.clamp(self.recovery_heading_kp * err, self.recovery_max_angular_speed)
            if abs(angular) < self.recovery_min_angular_speed:
                angular = math.copysign(self.recovery_min_angular_speed, err)
            linear = self.recovery_turn_linear_speed if abs(err) > self.recovery_in_place_angle_rad else self.recovery_linear_speed
            self.cmd_pub.publish(self.create_twist(linear, angular))
            if now_ts - getattr(self, '_channel_yaw_log_time', 0.0) >= 0.5:
                self._channel_yaw_log_time = now_ts
                self.log.progress(
                    f'channel_yaw_align yaw={math.degrees(yaw):.1f}deg '
                    f'target={math.degrees(self.channel_handoff_yaw):.1f}deg '
                    f'err={math.degrees(err):.1f}deg v={linear:.2f} w={angular:.2f}'
                )
            return

        pose_xy = self.get_map_position()
        if pose_xy is None:
            self.stop_robot()
            return
        if self._channel_map_target is not None:
            self._drive_to_channel_map_target(pose_xy, yaw, now_ts)
            return

        detector = self._channel_yolo_detector
        if detector is None:
            self.stop_robot()
            return
        geometry = detector.get_detection_geometry()
        age = now_ts - float(geometry.get('timestamp') or 0.0)
        if not geometry.get('detected') or geometry.get('bbox') is None or age > self.channel_yolo_timeout_sec:
            self.stop_robot()
            if now_ts - getattr(self, '_channel_yolo_log_time', 0.0) >= 0.5:
                self._channel_yolo_log_time = now_ts
                self.log.progress(
                    f'channel_yolo waiting bbox detected={geometry.get("detected")} '
                    f'age={age:.2f}s camera_info={geometry.get("camera_info") is not None}'
                )
            return

        target_base = self._channel_target_base_from_detection(geometry)
        if target_base is None:
            self.stop_robot()
            if now_ts - getattr(self, '_channel_geometry_log_time', 0.0) >= 0.5:
                self._channel_geometry_log_time = now_ts
                self.log.warn(
                    'CHANNEL_YOLO',
                    f'camera geometry unavailable, waiting: '
                    f'{getattr(self, "_channel_geometry_error", "unknown")}',
                )
            return
        target_map = self._channel_target_map_from_base(target_base)
        if target_map is None:
            self.stop_robot()
            return
        advance_x = self.channel_handoff_advance * math.cos(self.channel_handoff_yaw)
        advance_y = self.channel_handoff_advance * math.sin(self.channel_handoff_yaw)
        handoff_target = (target_map[0] + advance_x, target_map[1] + advance_y)
        self._channel_visual_target_map = target_map
        self._channel_map_target = handoff_target
        self._channel_handoff_step = 'forward_to_handoff'
        self.log.mission(
            f'channel geometry target base=({target_base[0]:.2f},{target_base[1]:.2f}) '
            f'visual_map=({target_map[0]:.2f},{target_map[1]:.2f}) '
            f'handoff_map=({handoff_target[0]:.2f},{handoff_target[1]:.2f}) '
            f'advance={self.channel_handoff_advance:.2f}m'
        )
        self._drive_to_channel_map_target(pose_xy, yaw, now_ts)

    def start_corridor_navigation(self, reason):
        # 只有倒退/YOLO追踪阶段使用通道检测；进入旧地图通道导航后释放BPU资源。
        self._set_channel_yolo_active(False)
        if not self.enable_corridor_navigation or not self.corridor_waypoints:
            self.phase1_motion_state = 'forward'
            self.begin_phase_transition(2, reason)
            return
        self.corridor_active = True
        self.corridor_nav_mode = 'path_follow'
        self.corridor_capture_active = False
        self.corridor_align_active = False
        self.corridor_entry_reorient_active = False
        self.corridor_entry_reorient_started_at = None
        self.corridor_final_align_active = False
        self.corridor_final_align_since = None
        self._corridor_timeout_logged = False
        self._corridor_angular_cmd_filtered = 0.0
        self.corridor_index = max(0, len(self.corridor_waypoints) - 1)
        self.corridor_started_at = self.get_clock().now().nanoseconds / 1e9
        self.corridor_path_points = []
        self.corridor_planned_path = []
        self.corridor_path_cursor = 0
        self.corridor_path_updated_at = 0.0
        self.corridor_planning_failures = 0
        self.corridor_last_plan_reason = ''
        self._corridor_last_plan_pose = None
        self._corridor_plan_count = 0
        self._corridor_vision_handoff_done = False
        self.phase1_motion_state = 'corridor'

        # 先使用地图/A* 导航到通道口，进入口子后直接放行 Stage2；
        # 视觉修正交给 Stage2 惯导融合，不在 Stage1 抢控制权。
        if hasattr(self, '_enable_vision_corridor'):
            try:
                self._enable_vision_corridor(False)
            except Exception as e:
                self.log.warn('VISION', f'关闭 Stage1 视觉导航失败，继续地图导航: {e}')
        self.log.mission(f'后退完成，先地图导航到通道口，随后切 Stage2 惯导+视觉修正: {reason}')

        self.stop_robot()
        map_xy = self.get_map_position()
        goal_xy = self.corridor_goal_point()
        if map_xy is not None and goal_xy is not None:
            target_heading = self.update_corridor_desired_heading(pose_xy=map_xy, goal_xy=goal_xy)
            gx, gy = goal_xy
            odom_txt = ''
            if self.current_odom is not None:
                op = self.current_odom.pose.pose.position
                odom_txt = f', odom=({op.x:.2f},{op.y:.2f})'
            yaw_txt = ''
            if self.current_yaw is not None:
                yaw_err = self.angle_error(self.corridor_goal_yaw, self.current_yaw)
                yaw_txt = (
                    f', yaw={math.degrees(self.current_yaw):.1f}° '
                    f'target={math.degrees(self.corridor_goal_yaw):.1f}° '
                    f'err={math.degrees(yaw_err):.1f}°'
                )
            dist = math.hypot(map_xy[0] - gx, map_xy[1] - gy)
            self.log.progress(
                f'区域进入起点(map): ({map_xy[0]:.2f}, {map_xy[1]:.2f}){odom_txt}, '
                f'目标区域中心(map): ({gx:.2f}, {gy:.2f}), 距离: {dist:.2f}m, '
                f'半径={self.corridor_entry_region_radius:.2f}m, '
                f'yaw_tol={math.degrees(self.corridor_entry_yaw_tolerance):.1f}°, '
                f'require_yaw={self.corridor_require_yaw_for_release}, '
                f'planner={self.use_corridor_planner}, mode={self.corridor_path_follow_mode}{yaw_txt}'
            )
            planned = self.refresh_corridor_planned_path(map_xy, goal_xy, reason='start')
            self.publish_corridor_path(map_xy)
            self.log.segment(
                f'corridor start plan_pts={len(planned or [])} '
                f'cursor=0 goal=({gx:.2f},{gy:.2f}) reason={reason}'
            )
        else:
            self.update_corridor_desired_heading()
            self.log.warn(
                'CORRIDOR',
                f'区域进入启动时位姿不足: map={map_xy is not None}, yaw={self.current_yaw is not None}'
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
        """Vectorized occupancy grid with map-stamp cache to avoid multi-second Python loops."""
        info = self.latest_map.info
        stamp = getattr(self.latest_map.header, 'stamp', None)
        stamp_key = (
            int(getattr(stamp, 'sec', 0) or 0),
            int(getattr(stamp, 'nanosec', 0) or 0),
            int(info.width),
            int(info.height),
            float(info.resolution),
            int(step),
            int(self.planner_occupied_threshold),
            bool(self.planner_unknown_is_occupied),
            float(self.planner_obstacle_inflation_m),
        )
        if self._corridor_occ_cache is not None and self._corridor_occ_cache_key == stamp_key:
            return self._corridor_occ_cache

        t0 = self.get_clock().now().nanoseconds / 1e9
        width = max(1, info.width // step)
        height = max(1, info.height // step)
        source = np.asarray(self.latest_map.data, dtype=np.int16).reshape(info.height, info.width)
        usable_h = height * step
        usable_w = width * step
        cropped = source[:usable_h, :usable_w]
        blocks = cropped.reshape(height, step, width, step)
        if self.planner_unknown_is_occupied:
            occupied = np.any((blocks < 0) | (blocks >= self.planner_occupied_threshold), axis=(1, 3))
        else:
            occupied = np.any(blocks >= self.planner_occupied_threshold, axis=(1, 3))

        radius = int(math.ceil(self.planner_obstacle_inflation_m / max(info.resolution, 1e-6) / step))
        if radius > 0:
            yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
            disk = (xx * xx + yy * yy) <= (radius * radius)
            inflated = occupied.copy()
            ys, xs = np.where(occupied)
            for y, x in zip(ys.tolist(), xs.tolist()):
                for dy in range(-radius, radius + 1):
                    ny = y + dy
                    if ny < 0 or ny >= height:
                        continue
                    for dx in range(-radius, radius + 1):
                        if not disk[dy + radius, dx + radius]:
                            continue
                        nx = x + dx
                        if 0 <= nx < width:
                            inflated[ny, nx] = True
            occupied = inflated

        self._corridor_occ_cache_key = stamp_key
        self._corridor_occ_cache = occupied
        dt_ms = (self.get_clock().now().nanoseconds / 1e9 - t0) * 1000.0
        if dt_ms > 80.0:
            self.log.progress(
                f'occupancy grid rebuild {width}x{height} step={step} '
                f'inflate={radius} took {dt_ms:.1f}ms'
            )
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

    def publish_corridor_path(self, start_xy=None):
        """发布 Stage1 通道导航路径到 RViz。"""
        points = []
        if self.corridor_planned_path:
            points = [(float(x), float(y)) for x, y in self.corridor_planned_path]
        elif start_xy is not None:
            points.append((float(start_xy[0]), float(start_xy[1])))
            goal_xy = self.corridor_goal_point()
            if goal_xy is not None:
                points.append((float(goal_xy[0]), float(goal_xy[1])))
        if not points:
            map_xy = self.get_map_position()
            if map_xy is not None:
                points.append((float(map_xy[0]), float(map_xy[1])))
            for waypoint in self.corridor_waypoints[self.corridor_index:]:
                points.append((float(waypoint['x']), float(waypoint['y'])))
        if len(points) < 2:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame
        for x, y in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.corridor_path_pub.publish(path_msg)
        self.corridor_path_points = points

    def _path_cross_track_m(self, path, pose_xy):
        """Approximate distance from pose to current planned polyline."""
        if not path or len(path) < 2:
            return 0.0
        best = float('inf')
        idx = max(0, min(self.corridor_path_cursor, len(path) - 2))
        # only check local segment neighborhood for speed
        for i in range(max(0, idx - 1), min(len(path) - 1, idx + 8)):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            dx = x2 - x1
            dy = y2 - y1
            seg_len2 = dx * dx + dy * dy
            if seg_len2 < 1e-9:
                d = math.hypot(pose_xy[0] - x1, pose_xy[1] - y1)
            else:
                t = ((pose_xy[0] - x1) * dx + (pose_xy[1] - y1) * dy) / seg_len2
                t = max(0.0, min(1.0, t))
                px = x1 + t * dx
                py = y1 + t * dy
                d = math.hypot(pose_xy[0] - px, pose_xy[1] - py)
            if d < best:
                best = d
        return 0.0 if not math.isfinite(best) else best

    def _should_refresh_corridor_path(self, pose_xy, now_ts, force=False, reason='periodic'):
        if force or not self.corridor_planned_path:
            return True, reason if force else 'empty'
        age = now_ts - float(self.corridor_path_updated_at or 0.0)
        if age < max(0.4, float(self.planner_replan_period_sec) * 0.35):
            return False, 'too_soon'
        progress = 0.0
        if self._corridor_last_plan_pose is not None:
            progress = math.hypot(
                pose_xy[0] - self._corridor_last_plan_pose[0],
                pose_xy[1] - self._corridor_last_plan_pose[1],
            )
        offpath = self._path_cross_track_m(self.corridor_planned_path, pose_xy)
        if offpath >= self.corridor_replan_offpath_m:
            return True, f'offpath_{offpath:.2f}m'
        if progress >= self.corridor_replan_min_progress_m and age >= self.planner_replan_period_sec:
            return True, f'progress_{progress:.2f}m'
        if age >= max(self.planner_replan_period_sec * 2.0, 4.0):
            return True, f'stale_{age:.1f}s'
        return False, f'hold age={age:.1f}s prog={progress:.2f}m off={offpath:.2f}m'

    def refresh_corridor_planned_path(self, start_xy, goal_xy, reason='periodic'):
        """Plan/refresh free-space path; fall back to straight line on failure."""
        now_ts = self.get_clock().now().nanoseconds / 1e9
        planned = None
        plan_mode = 'fallback_line'
        if self.use_corridor_planner:
            planned = self.plan_corridor_path(start_xy, goal_xy)
            if planned and len(planned) >= 2:
                plan_mode = 'astar'
            else:
                self.corridor_planning_failures += 1
                self.log.warn(
                    'CORRIDOR',
                    f'A* plan failed reason={reason} failures={self.corridor_planning_failures} '
                    f'start=({start_xy[0]:.2f},{start_xy[1]:.2f}) '
                    f'goal=({goal_xy[0]:.2f},{goal_xy[1]:.2f}) -> fallback line'
                )
                planned = None
        if planned is None:
            planned = [
                (float(start_xy[0]), float(start_xy[1])),
                (float(goal_xy[0]), float(goal_xy[1])),
            ]
        self.corridor_planned_path = planned
        self.corridor_path_cursor = 0
        self.corridor_path_updated_at = now_ts
        self.corridor_last_plan_reason = f'{reason}:{plan_mode}'
        self._corridor_last_plan_pose = (float(start_xy[0]), float(start_xy[1]))
        self._corridor_plan_count = int(getattr(self, '_corridor_plan_count', 0)) + 1
        path_len = 0.0
        for i in range(1, len(planned)):
            path_len += math.hypot(planned[i][0] - planned[i - 1][0], planned[i][1] - planned[i - 1][1])
        self.log.segment(
            f'corridor plan refresh reason={reason} mode={plan_mode} pts={len(planned)} '
            f'len={path_len:.2f}m start=({start_xy[0]:.2f},{start_xy[1]:.2f}) '
            f'goal=({goal_xy[0]:.2f},{goal_xy[1]:.2f}) count={self._corridor_plan_count}'
        )
        return planned

    def _advance_path_cursor(self, path, pose_xy):
        if not path:
            return 0
        idx = max(0, min(self.corridor_path_cursor, len(path) - 1))
        # 推进到最近前方点，避免 lookahead 回退
        while idx < len(path) - 1:
            d = math.hypot(path[idx][0] - pose_xy[0], path[idx][1] - pose_xy[1])
            if d > max(0.12, self.corridor_pp_min_lookahead * 0.5):
                break
            idx += 1
        self.corridor_path_cursor = idx
        return idx

    def _corridor_lookahead(self, path, current_xy):
        if not path:
            self.log.error('CORRIDOR', 'Lookahead: 路径为空！')
            return None
        if current_xy is None:
            return path[-1]
        idx = self._advance_path_cursor(path, current_xy)
        lookahead = max(self.pure_pursuit_lookahead, self.corridor_pp_min_lookahead)
        traveled = 0.0
        previous = current_xy
        for point in path[idx:]:
            traveled += math.hypot(point[0] - previous[0], point[1] - previous[1])
            if traveled >= lookahead:
                return point
            previous = point
        return path[-1]

    def maybe_advance_corridor_waypoint(self, pose_xy):
        """兼容旧接口：区域进入时固定盯最终入口目标。"""
        if self.corridor_waypoints:
            self.corridor_index = max(0, len(self.corridor_waypoints) - 1)

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

    def maybe_blind_left_search_cmd(self):
        """二维码未识别且 map_x 超过阈值时，低速向左搜索二维码。"""
        if self.qr_processed or self.phase1_motion_state != 'forward':
            return None

        map_x = self._lookup_map_x()
        if map_x is None or map_x <= self.blind_left_search_x:
            return None

        now_ts = self.get_clock().now().nanoseconds / 1e9
        if now_ts - getattr(self, '_blind_left_search_log_time', 0.0) >= 1.0:
            self._blind_left_search_log_time = now_ts
            self.log.progress(
                f'blind_left_search: qr未识别, map_x={map_x:.2f}>'
                f'{self.blind_left_search_x:.2f} '
                f'v={self.blind_left_search_linear_speed:.2f} '
                f'w={self.blind_left_search_angular_speed:.2f}'
            )
        return self.create_twist(
            self.blind_left_search_linear_speed,
            abs(self.blind_left_search_angular_speed),
        )

    def _corridor_region_release_ready(self, pose_xy, goal_xy, yaw):
        rho = math.hypot(goal_xy[0] - pose_xy[0], goal_xy[1] - pose_xy[1])
        yaw_error = self.angle_error(self.corridor_goal_yaw, yaw)
        pos_ok = rho <= self.corridor_entry_region_radius
        yaw_ok = abs(yaw_error) <= self.corridor_entry_yaw_tolerance
        if self.corridor_require_yaw_for_release:
            ready = pos_ok and yaw_ok
        else:
            ready = pos_ok
        return ready, rho, yaw_error, pos_ok, yaw_ok

    def _finish_corridor_release(self, pose_xy, goal_xy, yaw, reason):
        self.corridor_active = False
        self.corridor_nav_mode = 'idle'
        self.corridor_capture_active = False
        self.corridor_align_active = False
        self.corridor_entry_reorient_active = False
        self.corridor_entry_reorient_started_at = None
        self.corridor_final_align_active = False
        self.corridor_final_align_since = None
        self._corridor_angular_cmd_filtered = 0.0
        self.phase1_motion_state = 'forward'
        self.stop_robot()
        self.log.mission(reason)
        self.begin_phase_transition(2, reason)

    def _handle_corridor_final_align(self, pose_xy, goal_xy, yaw, rho, now_ts):
        """末端锁定到入口门线：对准 90° 正向穿线，不再追车后的目标点。"""
        x_error = goal_xy[0] - pose_xy[0]
        y_to_gate = goal_xy[1] - pose_xy[1]
        yaw_error = self.angle_error(self.corridor_goal_yaw, yaw)
        abs_yaw_error = abs(yaw_error)
        self.corridor_final_align_active = True

        # 小航向误差时优先直行，避免门口来回拧
        if abs_yaw_error <= self.corridor_heading_hold:
            angular = 0.0
            target_y = 0.0
        else:
            angular = self.corridor_final_align_heading_kp * yaw_error
            # 仅在航向已较正时再补横向，且横向贡献限幅
            if abs_yaw_error <= math.radians(12.0):
                target_y = -math.sin(yaw) * x_error + math.cos(yaw) * y_to_gate
                # 横向死区：5cm 内不修，减少噪声驱动转向
                if abs(target_y) < 0.05:
                    target_y = 0.0
                angular += self.corridor_final_align_lateral_kp * target_y
            else:
                target_y = 0.0
        angular = self.clamp(angular, self.max_angular_speed)
        angular = self._smooth_corridor_angular(angular, heading_error=yaw_error)

        # 接近终点提速：按到门线距离平滑降速，尽量保持冲门速度
        speed_ref_distance = max(y_to_gate, 0.10)
        linear = min(
            self.corridor_final_align_max_speed,
            max(self.corridor_final_align_min_speed, 0.75 * speed_ref_distance),
        )
        # 任何末端修角指令都保持非零正向速度。
        linear = max(linear, self.corridor_final_align_min_speed)
        # 大航向误差时略降速，但不要掉到爬行
        if abs_yaw_error > math.radians(25.0):
            linear = min(linear, max(self.corridor_final_align_min_speed, 0.16))
        self.corridor_nav_mode = 'final_align'

        x_ok = abs(x_error) <= self.corridor_final_gate_x_tolerance
        y_ok = -self.corridor_final_gate_y_after <= y_to_gate <= self.corridor_final_gate_y_before
        yaw_ok = abs_yaw_error <= self.corridor_final_gate_yaw_tolerance
        if x_ok and y_ok and yaw_ok:
            if self.corridor_final_align_since is None:
                self.corridor_final_align_since = now_ts
        else:
            self.corridor_final_align_since = None

        stable = (
            self.corridor_final_align_since is not None
            and now_ts - self.corridor_final_align_since >= self.corridor_final_align_stable_sec
        )
        if stable:
            reason = (
                f'final gate OK map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                f'xerr={x_error:+.2f}m ygate={y_to_gate:+.2f}m '
                f'yaw={math.degrees(yaw):.1f}deg err={math.degrees(yaw_error):.1f}deg '
                f'stable={self.corridor_final_align_stable_sec:.2f}s'
            )
            self._finish_corridor_release(pose_xy, goal_xy, yaw, reason)
            return True

        if (
            x_ok
            and y_to_gate < -self.corridor_final_overshoot_y
            and abs_yaw_error <= self.corridor_final_overshoot_yaw_tolerance
        ):
            reason = (
                f'final gate overshoot release map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                f'xerr={x_error:+.2f}m ygate={y_to_gate:+.2f}m '
                f'yaw={math.degrees(yaw):.1f}deg err={math.degrees(yaw_error):.1f}deg'
            )
            self._finish_corridor_release(pose_xy, goal_xy, yaw, reason)
            return True

        self.cmd_pub.publish(self.create_twist(linear, angular))
        if now_ts - self._corridor_last_log_time >= self.corridor_log_period_sec:
            self._corridor_last_log_time = now_ts
            stable_for = (
                now_ts - self.corridor_final_align_since
                if self.corridor_final_align_since is not None else 0.0
            )
            self.log.segment(
                f'final_align map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                f'xerr={x_error:+.2f}m ygate={y_to_gate:+.2f}m rho={rho:.2f}m '
                f'yaw={math.degrees(yaw):.1f}deg err={math.degrees(yaw_error):.1f}deg '
                f'gate(x={x_ok},y={y_ok},yaw={yaw_ok}) '
                f'lat={target_y:+.2f} v={linear:.2f} w={angular:.2f} stable={stable_for:.2f}s'
            )
        return False

    def handle_corridor_navigation(self):
        """
        Stage1 通道导航（区域进入）：
          1) A* 规划自由空间路径（失败则直线 fallback）
          2) Pure Pursuit 跟踪路径（允许斜穿）
          3) 进入末端区域后保持正向速度收敛到 90°，稳定后切 Stage2
          4) 超时策略放行
        """
        now_ts = self.get_clock().now().nanoseconds / 1e9
        map_xy = self.get_map_position()

        if map_xy is None or self.current_yaw is None:
            if now_ts - self._corridor_last_detail_log_time >= 1.0:
                self._corridor_last_detail_log_time = now_ts
                self.log.warn(
                    'CORRIDOR',
                    f'位姿不足，停车等待 map={map_xy is not None} yaw={self.current_yaw is not None}'
                )
            self.stop_robot()
            return

        if self.corridor_started_at is not None and now_ts - self.corridor_started_at > self.corridor_timeout_sec:
            if not self._corridor_timeout_logged:
                self._corridor_timeout_logged = True
                goal = self.corridor_goal_point()
                rho_txt = 'n/a'
                if goal is not None:
                    rho_txt = f'{math.hypot(map_xy[0]-goal[0], map_xy[1]-goal[1]):.2f}m'
                self.log.warn(
                    'CORRIDOR',
                    f'region entry TIMEOUT {self.corridor_timeout_sec:.1f}s, hold at stop | '
                    f'map=({map_xy[0]:.2f},{map_xy[1]:.2f}) yaw={math.degrees(self.current_yaw):.1f}deg '
                    f'rho={rho_txt} plans={getattr(self, "_corridor_plan_count", 0)} '
                    f'last_plan={self.corridor_last_plan_reason or "none"}'
                )
            self.stop_robot()
            return

        pose_xy = (float(map_xy[0]), float(map_xy[1]))
        yaw = float(self.current_yaw)
        goal_xy = self.corridor_goal_point()
        if goal_xy is None:
            self.log.error('CORRIDOR', '无入口目标点，停车等待 YAML 配置')
            self.stop_robot()
            return
        goal_xy = (float(goal_xy[0]), float(goal_xy[1]))
        self.corridor_index = max(0, len(self.corridor_waypoints) - 1)

        ready, rho, yaw_error, pos_ok, yaw_ok = self._corridor_region_release_ready(pose_xy, goal_xy, yaw)
        if self.corridor_final_align_active or rho <= self.corridor_final_align_start_distance:
            self._handle_corridor_final_align(pose_xy, goal_xy, yaw, rho, now_ts)
            return

        # 条件重规划：进度/偏航/过期才重算，避免每 0.5s 阻塞 1.7s
        need_replan, replan_why = self._should_refresh_corridor_path(
            pose_xy, now_ts, force=not self.corridor_planned_path, reason='empty'
        )
        if need_replan:
            self.refresh_corridor_planned_path(pose_xy, goal_xy, reason=replan_why)
            self.publish_corridor_path(pose_xy)
        elif now_ts - self._corridor_last_detail_log_time >= max(1.0, self.corridor_log_period_sec * 2.0):
            # 低频打印 hold 原因，方便场测排查
            pass

        path = self.corridor_planned_path
        if not path:
            path = [pose_xy, goal_xy]
            self.corridor_planned_path = path

        # map_x 过大：强制向左旋回
        left_cmd = self.maybe_left_recover_cmd('corridor_left_recover')
        if left_cmd is not None and rho > self.corridor_entry_region_radius:
            self.corridor_nav_mode = 'left_recover'
            self.cmd_pub.publish(left_cmd)
            if now_ts - self._corridor_last_detail_log_time >= self.corridor_log_period_sec:
                self._corridor_last_detail_log_time = now_ts
                self.log.progress(
                    f'left_recover during region entry ρ={rho:.2f}m '
                    f'map=({pose_xy[0]:.2f},{pose_xy[1]:.2f})'
                )
            return

        self.corridor_nav_mode = 'path_follow'
        look_pt = self._corridor_lookahead(path, pose_xy)
        if look_pt is None:
            look_pt = goal_xy
        dx = look_pt[0] - pose_xy[0]
        dy = look_pt[1] - pose_xy[1]
        los = math.atan2(dy, dx) if (abs(dx) + abs(dy)) > 1e-6 else yaw
        alpha = self.angle_error(los, yaw)
        self.corridor_desired_heading = los

        # Pure Pursuit 几何曲率
        ld = max(math.hypot(dx, dy), self.corridor_pp_min_lookahead)
        # body frame target
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        target_x = cos_yaw * dx + sin_yaw * dy
        target_y = -sin_yaw * dx + cos_yaw * dy

        speed_cap = max(self.corridor_min_cruise_speed, self.corridor_linear_speed * self.corridor_pp_speed_scale)
        heading_scale = max(0.55, abs(math.cos(alpha)))
        linear = min(speed_cap, max(self.corridor_min_cruise_speed, self.corridor_rho_kp * max(rho, 0.40) * heading_scale))

        if rho < self.corridor_brake_distance:
            brake_cap = max(self.corridor_creep_speed, self.corridor_brake_kp * max(rho, 0.22))
            linear = min(linear, max(self.corridor_creep_speed, brake_cap))
        if abs(alpha) > math.radians(45.0):
            linear = min(linear, max(self.corridor_creep_speed, self.corridor_max_turn_linear_speed))
        if abs(alpha) > math.radians(70.0):
            # 大角时仍保持一定前进速度，避免趴地拧
            linear = min(linear, max(self.corridor_creep_speed, 0.14))

        if self.corridor_path_follow_mode == 'stanley':
            v_ref = max(abs(linear), 0.12)
            angular = self.corridor_alpha_kp * alpha + math.atan2(self.corridor_stanley_k * target_y, v_ref)
        else:
            # pure pursuit: curvature = 2*y / ld^2
            curvature = 2.0 * target_y / max(ld * ld, 1e-3)
            angular = self.pure_pursuit_turn_kp * curvature * max(abs(linear), 0.12)
            # 大航向误差时叠加 LOS P 项，但增益更温和
            if abs(alpha) > math.radians(30.0):
                angular += 0.30 * self.corridor_alpha_kp * alpha

        angular = self.clamp(angular, self.max_angular_speed)
        angular = self._smooth_corridor_angular(angular, heading_error=alpha)
        if abs(angular) > 0.30:
            linear = min(linear, max(self.corridor_creep_speed, self.corridor_max_turn_linear_speed))
        elif abs(angular) > 0.18:
            linear = min(linear, max(self.corridor_min_cruise_speed * 0.90, 0.16))

        # 极近区域仍保持较高前进速度，交给 final_align 收门
        if rho < max(0.22, self.corridor_entry_region_radius * 0.75):
            linear = min(linear, max(self.corridor_creep_speed, 0.16))

        self.cmd_pub.publish(self.create_twist(linear, angular))

        if now_ts - self._corridor_last_log_time >= self.corridor_log_period_sec:
            self._corridor_last_log_time = now_ts
            elapsed = now_ts - self.corridor_started_at if self.corridor_started_at else 0.0
            cursor = self.corridor_path_cursor
            offpath = self._path_cross_track_m(path, pose_xy)
            self.log.segment(
                f'region_entry map=({pose_xy[0]:.2f},{pose_xy[1]:.2f})->'
                f'({goal_xy[0]:.2f},{goal_xy[1]:.2f}) rho={rho:.2f}m '
                f'yaw={math.degrees(yaw):.1f}deg yerr={math.degrees(yaw_error):.1f}deg '
                f'alpha={math.degrees(alpha):.1f}deg look=({look_pt[0]:.2f},{look_pt[1]:.2f}) '
                f'body=({target_x:.2f},{target_y:.2f}) ld={ld:.2f} offpath={offpath:.2f}m '
                f'v={linear:.2f} w={angular:.2f} mode={self.corridor_nav_mode} '
                f'plan={self.corridor_last_plan_reason} pts={len(path)} cursor={cursor} '
                f'plans={getattr(self, "_corridor_plan_count", 0)} '
                f'pos_ok={pos_ok} yaw_ok={yaw_ok} t={elapsed:.1f}s'
            )
            print(
                f"\r[Stage1区域进入] map({pose_xy[0]:.2f},{pose_xy[1]:.2f}) → "
                f"({goal_xy[0]:.2f},{goal_xy[1]:.2f}) | ρ{rho:.2f}m | "
                f"yaw{math.degrees(yaw):.0f}°/err{math.degrees(yaw_error):.0f}° | "
                f"v{linear:.2f} w{angular:.2f} | {elapsed:.0f}s",
                end="",
                flush=True,
            )
            self.publish_corridor_path(pose_xy)

    def begin_avoidance(self, danger_angle):
        if self.phase1_motion_state in ('backing', 'channel_yolo_chase', 'channel_yolo_finish'):
            self._avoid_resume_state = self.phase1_motion_state
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

        if self.corridor_active or self.corridor_resume_after_avoidance:
            # 通道避障后 recovery 必须回到通道航向，而不是 phase1 盲开锁向
            self.update_corridor_desired_heading()
        elif self.desired_heading is None and self.current_yaw is not None:
            self.desired_heading = self.current_yaw

        self.log.feedback(
            f'avoid start dir={self.avoid_turn_direction:.0f} '
            f'danger_angle={danger_angle:.0f}° '
            f'desired_yaw={(math.degrees(self.desired_heading) if self.desired_heading is not None else float("nan")):.1f}°'
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
        if self.corridor_active or self.corridor_resume_after_avoidance:
            self.update_corridor_desired_heading()
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
        if self._avoid_resume_state in ('backing', 'channel_yolo_chase', 'channel_yolo_finish'):
            resume_state = self._avoid_resume_state
            self._avoid_resume_state = None
            self.phase1_motion_state = resume_state
            self.log.feedback(f'recovery complete, return to {resume_state}')
        elif self.corridor_resume_after_avoidance and self.corridor_active:
            self.phase1_motion_state = 'corridor'
            self.corridor_resume_after_avoidance = False
            map_xy = self.get_map_position()
            if map_xy is not None and self.corridor_waypoints:
                goal_xy = (
                    float(self.corridor_waypoints[self.corridor_index]['x']),
                    float(self.corridor_waypoints[self.corridor_index]['y']),
                )
                target = self.update_corridor_desired_heading(pose_xy=map_xy, goal_xy=goal_xy)
                yaw_err = None
                if self.current_yaw is not None:
                    yaw_err = abs(self.angle_error(target, self.current_yaw))
                if (
                    self.corridor_entry_reorient_enabled
                    and yaw_err is not None
                    and yaw_err >= self.corridor_entry_reorient_done
                ):
                    self.begin_corridor_entry_reorient(
                        'post-avoid resume',
                        pose_xy=map_xy,
                        goal_xy=goal_xy,
                    )
                else:
                    self.corridor_nav_mode = 'centerline'
            else:
                self.update_corridor_desired_heading()
                self.corridor_nav_mode = 'centerline'
            self.log.feedback('recovery complete, return to corridor')
        else:
            self.phase1_motion_state = 'forward'
            self.log.feedback('recovery complete, return to forward')
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.recovery_uses_heading = False

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
        # 通道导航接近目标点时禁用避障，避免交接时被打断超出 Stage2 入口范围
        if self.phase1_motion_state == 'corridor':
            map_xy = self.get_map_position()
            if map_xy is not None and len(self.corridor_waypoints) > 0:
                goal_xy = self.corridor_waypoints[-1]
                dist_to_goal = math.hypot(goal_xy['x'] - map_xy[0], goal_xy['y'] - map_xy[1])
                if dist_to_goal <= self.corridor_disable_avoidance_distance:
                    self.obstacle_found = False
                    self.closest_obstacle_distance = float('inf')
                    self.avoid_cmd = Twist()
                    return

        # 通道入口大角度重定向时默认不避障，避免被侧墙二次打断并反向拉航向
        if (
            self.phase1_motion_state == 'corridor'
            and self.corridor_entry_reorient_active
            and not self.corridor_avoid_while_reorient
        ):
            self.obstacle_found = False
            self.closest_obstacle_distance = float('inf')
            self.avoid_cmd = Twist()
            return

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

        # 去重检查：防止重复扫码播报
        if self.qr_processed:
            self.log.progress('qr already processed, ignoring duplicate scan')
            return

        # 允许在前进、避障等状态下接收二维码(避免因临时避障错过扫码)
        if self.phase1_motion_state not in ('forward', 'avoiding', 'countersteering', 'recovering', 'corridor'):
            return

        task = msg.data.strip()
        if not task:
            return

        # 设置去重标志
        self.qr_processed = True
        self.qr_task = task
        self.task_pub.publish(String(data=task))

        # 立即启动后退，不等待播报完成
        if self.enable_backing and len(self.path_record) > 0:
            self.phase1_motion_state = 'backing'
            self._set_channel_yolo_active(True)
            now = self.get_clock().now()
            self.backing_started_time = now
            self.backing_path_index = len(self.path_record) - 1
            self.log.mission(
                f'qr detected: {task}, backing immediately, '
                f'{len(self.path_record)} waypoints recorded, YOLO armed'
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
        if self._stage2_cmd_timeout_active:
            self._stage2_cmd_timeout_active = False
            self.log.info(
                'STAGE2_CMD_RECOVER',
                f'received cmd after timeout: '
                f'v={msg.linear.x:.3f} w={msg.angular.z:.3f}',
            )

    def stage2_cmd_is_fresh(self):
        if self.latest_stage2_cmd_time is None:
            return False

        age = self.get_clock().now() - self.latest_stage2_cmd_time
        return age.nanoseconds <= int(self.stage2_cmd_timeout * 1e9)

    def stage2_cmd_age_sec(self):
        if self.latest_stage2_cmd_time is None:
            return None
        age = self.get_clock().now() - self.latest_stage2_cmd_time
        return age.nanoseconds / 1e9

    def log_stage2_cmd_timeout_if_needed(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self._stage2_cmd_timeout_active and now_sec - self._last_stage2_timeout_log_sec < 1.0:
            return
        self._stage2_cmd_timeout_active = True
        self._last_stage2_timeout_log_sec = now_sec
        age_sec = self.stage2_cmd_age_sec()
        age_text = 'none' if age_sec is None else f'{age_sec:.3f}s'
        self.log.warn(
            'STAGE2_CMD_TIMEOUT',
            f'phase=2 stop_robot: no fresh /stage2_cmd_vel '
            f'age={age_text} timeout={self.stage2_cmd_timeout:.3f}s '
            f'last_cmd=({self.latest_stage2_cmd.linear.x:.3f},'
            f'{self.latest_stage2_cmd.angular.z:.3f}) '
            f'stage2_state={self.stage2_state}',
        )

    def control_loop(self):
        if self.mission_finished:
            self.stop_robot()
            return

        if self.phase == 1:
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
                    if self.phase1_motion_state in ('backing', 'channel_yolo_chase', 'channel_yolo_finish'):
                        return
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
                # 视觉导航优先，但必须确保模块已成功初始化
                if (hasattr(self, '_vision_corridor_enabled') and
                    self._vision_corridor_enabled and
                    getattr(self, '_vision_corridor_active', False) and
                    hasattr(self, '_vision_corridor') and
                    self._vision_corridor is not None):
                    try:
                        cmd, vision_status = self._get_vision_corridor_control()
                        if cmd is not None:
                            self.cmd_pub.publish(cmd)
                        else:
                            self.stop_robot()
                        if vision_status.get('reached_entry', False):
                            self._enable_vision_corridor(False)
                            self.begin_phase_transition(2, '视觉检测到达通道入口')
                        return
                    except Exception as e:
                        self.log.error('VISION', f'视觉控制异常，降级到地图导航: {e}')
                        # 降级到地图导航，不要停止程序

                # 降级：使用原有地图 A* 导航
                self.handle_corridor_navigation()
                return

            if self.phase1_motion_state == 'channel_yolo_chase':
                self.handle_channel_yolo_chase()
                return

            if self.phase1_motion_state == 'channel_yolo_finish':
                self.handle_channel_yolo_finish()
                return

            if self.phase1_motion_state == 'channel_yolo_handoff':
                self.handle_channel_yolo_handoff()
                return

            # 二维码在左侧：盲开超过 map_x 阈值仍未识别时，低速向左搜索。
            left_search_cmd = self.maybe_blind_left_search_cmd()
            if left_search_cmd is not None:
                self.cmd_pub.publish(left_search_cmd)
                return

            # 兼容通道导航的旧左侧恢复逻辑。
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

        self.log_stage2_cmd_timeout_if_needed()
        self.stop_robot()

    def handle_backing(self):
        """处理后退逻辑：沿记录路径反向跟踪"""
        now_ts = self.get_clock().now().nanoseconds / 1e9
        if self._channel_yolo_detection_confirmed(now_ts):
            self.begin_channel_yolo_chase(
                f'{self.channel_yolo_confirm_frames} consecutive bbox frames while backing'
            )
            self.handle_channel_yolo_chase()
            return

        if self.current_odom is None or self.current_yaw is None:
            self.stop_robot()
            return
        
        # 超时检查
        if self.backing_started_time is not None:
            elapsed = (self.get_clock().now() - self.backing_started_time).nanoseconds / 1e9
            if elapsed > self.back_timeout_sec:
                self.log.warn('BACKING', f'timeout after {elapsed:.1f}s, starting corridor navigation')
                self._set_channel_yolo_active(False)
                self.start_corridor_navigation(f'qr task={self.qr_task}, backing timeout')
                return
        
        # 倒退路点在 map 中跟踪；结束线 X=back_target_x 属于 /odom_combined。
        # 不能把 map 的平移/旋转后的 X 当成赛程定义的 odom X，否则会多倒一段。
        map_xy = self._get_strict_map_position()
        if map_xy is None:
            self.log.warn('BACKING', 'map pose unavailable, holding position')
            self.stop_robot()
            return
        map_x, map_y = map_xy
        odom_x = float(self.current_odom.pose.pose.position.x)
        odom_y = float(self.current_odom.pose.pose.position.y)

        if odom_x <= self.back_target_x:
            self.log.segment(
                f'backing done at odom_x={odom_x:.2f}m '
                f'(target={self.back_target_x:.2f}m), '
                f'map=({map_x:.2f},{map_y:.2f}), odom_y={odom_y:.2f}m, '
                f'starting corridor navigation'
            )
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
        dist_to_target = math.hypot(map_x - target_x, map_y - target_y)
        if dist_to_target < self.back_position_tolerance:
            self.backing_path_index -= 1
            self.log.progress(
                f'backing wp_index={self.backing_path_index}, '
                f'map=({map_x:.2f}, {map_y:.2f}), '
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
        linear_speed = self.back_linear_speed
        if abs(heading_error) >= self.back_turn_slowdown_angle_rad:
            linear_speed = self.back_turn_linear_speed
        
        # 倒车（负速度），车头保持来时方向
        self.cmd_pub.publish(self.create_twist(linear_speed, angular_z))
        
        self.log.progress(
            f'backing: wp={self.backing_path_index}, '
            f'map=({map_x:.2f},{map_y:.2f}), '
            f'odom=({odom_x:.2f},{odom_y:.2f}), '
            f'dist={dist_to_target:.2f}m, '
            f'target_yaw={math.degrees(target_yaw):.1f}°, '
            f'yaw_error={math.degrees(heading_error):.1f}°, '
            f'linear={linear_speed:.2f}m/s'
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

        # 对齐转向始终保持正向速度，禁止 v=0 时输出角速度。
        angular_z = self.clamp(self.recovery_heading_kp * heading_error, self.recovery_max_angular_speed)
        if abs(angular_z) < self.recovery_min_angular_speed:
            angular_z = math.copysign(self.recovery_min_angular_speed, heading_error)

        if abs(heading_error) <= self.recovery_in_place_angle_rad:
            linear_x = self.recovery_linear_speed  # 0.12 m/s
        else:
            linear_x = max(self.recovery_turn_linear_speed, 0.06)

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

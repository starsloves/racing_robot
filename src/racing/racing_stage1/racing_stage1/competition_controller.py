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
from std_msgs.msg import Float64, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener

from racing_common.obstacle_marker_publisher import ObstacleMarkerPublisher
from racing_common.racing_logger import RacingLogger
from racing_stage1.stage1_vision_mixin import Stage1VisionMixin


class CompetitionController(Stage1VisionMixin, Node):
    def __init__(self):
        super().__init__('competition_controller')

        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('stage2_cmd_topic', '/stage2_cmd_vel')
        self.declare_parameter('stage3_cmd_topic', '/stage3_cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('imu_yaw_offset_deg', 0.0)
        self.declare_parameter('imu_initial_map_yaw_deg', 10.0)
        self.declare_parameter('imu_map_yaw_offset_topic', 'imu_map_yaw_offset')
        self.declare_parameter('reset_imu_yaw_on_phase2_handoff', True)
        self.declare_parameter('odom_topic', '/odom_combined')  # map 坐标系
        self.declare_parameter('qr_result_topic', 'qr_scan_result')
        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('task_topic', 'competition_qr_task')
        self.declare_parameter('stage2_state_topic', 'stage2_state')
        self.declare_parameter('stage3_state_topic', 'stage3_state')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('blind_linear_speed', 0.2)
        self.declare_parameter('blind_qr_slowdown_start_x_m', 1.5)
        self.declare_parameter('blind_qr_slowdown_linear_speed', 0.4)
        self.declare_parameter('blind_angular_speed', 0.0)
        self.declare_parameter('blind_scan_centerline_json', '[]')
        self.declare_parameter('blind_scan_corridor_half_width_m', 0.35)
        self.declare_parameter('blind_scan_capture_start_odom_x_m', 1.0)
        self.declare_parameter('blind_scan_guidance_start_odom_x_m', 3.0)
        self.declare_parameter('blind_scan_guidance_ramp_m', 0.80)
        self.declare_parameter('blind_scan_lateral_kp', 1.0)
        self.declare_parameter('blind_scan_max_heading_offset_deg', 20.0)
        self.declare_parameter('blind_scan_heading_kp', 1.4)
        self.declare_parameter('blind_scan_max_angular_speed', 0.35)
        self.declare_parameter('blind_scan_guidance_max_angular_speed', 0.18)
        self.declare_parameter('blind_scan_avoid_prediction_sec', 1.80)
        self.declare_parameter('blind_scan_avoid_prediction_step_sec', 0.05)
        self.declare_parameter('blind_scan_avoid_min_clearance_m', 0.28)
        self.declare_parameter('blind_scan_avoid_detection_max_x_m', 1.00)
        self.declare_parameter('blind_scan_escape_reverse_linear_speed_mps', -0.16)
        self.declare_parameter('blind_scan_escape_reverse_angular_speed', 0.55)
        self.declare_parameter('blind_scan_escape_reverse_duration_sec', 0.80)
        self.declare_parameter('blind_scan_escape_corridor_extra_m', 0.12)
        self.declare_parameter('blind_scan_escape_rear_min_x_m', -0.45)
        self.declare_parameter('blind_scan_escape_rear_max_x_m', -0.08)
        self.declare_parameter('blind_scan_escape_rear_half_width_m', 0.22)
        self.declare_parameter('avoid_linear_speed', 0.1)
        self.declare_parameter('avoid_angular_speed', 0.8)
        self.declare_parameter('avoid_min_duration_sec', 0.7)
        self.declare_parameter('avoid_clear_hold_sec', 0.25)
        self.declare_parameter('avoid_min_turn_angle_deg', 18.0)
        self.declare_parameter('avoid_right_turn_left_obstacle_angle_deg', 15.0)
        self.declare_parameter('corridor_avoid_goal_bias_enabled', True)
        self.declare_parameter('corridor_avoid_obstacle_side_penalty', 3.5)
        self.declare_parameter('corridor_obstacle_min_width_m', 0.12)
        # Corridor obstacles are handled by a short reverse steer, never by
        # the general forward detour that can leave the wall-defined field.
        self.declare_parameter('corridor_reverse_avoid_enabled', True)
        self.declare_parameter('corridor_reverse_avoid_linear_speed_mps', -0.14)
        self.declare_parameter('corridor_reverse_avoid_angular_speed', 0.18)
        self.declare_parameter('corridor_reverse_avoid_min_duration_sec', 0.65)
        self.declare_parameter('corridor_reverse_avoid_clear_hold_sec', 0.25)
        self.declare_parameter('corridor_reverse_avoid_entry_tolerance_m', 0.25)
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
        self.declare_parameter('stage3_cmd_timeout', 0.5)
        self.declare_parameter('transition_stop_duration', 0.0)
        self.declare_parameter('phase2_obstacle_override', False)
        self.declare_parameter('phase2_emergency_stop_distance', 0.22)
        self.declare_parameter('phase3_external_control', True)
        self.declare_parameter('phase3_emergency_stop_distance', 0.22)
        self.declare_parameter('enable_backing', True)
        self.declare_parameter('back_target_x', 2.0)
        self.declare_parameter('back_linear_speed', -0.15)
        self.declare_parameter('back_angular_kp', 1.8)
        self.declare_parameter('back_max_angular_speed', 0.60)
        self.declare_parameter('back_angular_slew_rate', 1.50)
        self.declare_parameter('back_position_tolerance', 0.15)
        self.declare_parameter('back_path_sample_distance', 0.20)
        self.declare_parameter('back_lookahead_m', 0.25)
        self.declare_parameter('back_timeout_sec', 10.0)
        self.declare_parameter('back_align_yaw_deg', 90.0)
        self.declare_parameter('back_align_tolerance_deg', 5.0)
        self.declare_parameter('back_align_timeout_sec', 5.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom_combined')
        # 静态 map->odom TF 的默认平移由 Stage1 YAML 统一保存，供 launch 读取。
        self.declare_parameter('map_to_odom_x', 0.30)
        self.declare_parameter('map_to_odom_y', 0.15)
        self.declare_parameter('corridor_path_topic', '/stage1_corridor_path')
        self.declare_parameter('enable_corridor_navigation', True)
        self.declare_parameter('corridor_waypoints_json', '[{"x":2.80,"y":3.10}]')
        self.declare_parameter('corridor_reference_path_enabled', False)
        self.declare_parameter('corridor_reference_path_json', '[]')
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
        self.declare_parameter('corridor_entry_reorient_enabled', True)
        self.declare_parameter('corridor_entry_reorient_angle_deg', 50.0)
        self.declare_parameter('corridor_entry_reorient_done_deg', 25.0)
        self.declare_parameter('corridor_entry_reorient_timeout_sec', 4.0)
        self.declare_parameter('corridor_centerline_reorient_deg', 55.0)
        self.declare_parameter('corridor_avoid_while_reorient', False)
        self.declare_parameter('corridor_left_recover_x', 3.50)
        self.declare_parameter('corridor_left_recover_angular', 0.70)
        self.declare_parameter('corridor_left_recover_linear', 0.06)
        self.declare_parameter('blind_right_turn_x', 3.5)
        self.declare_parameter('blind_right_turn_y', 1.5)
        self.declare_parameter('blind_right_turn_angular', 0.70)
        self.declare_parameter('blind_right_turn_linear', 0.08)
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
        # 区域进入：圆形区域命中后仍须越过最小 map Y 门线，防止入口前提前交权。
        self.declare_parameter('corridor_entry_region_radius_m', 0.35)
        self.declare_parameter('corridor_release_min_y_m', float('-inf'))
        self.declare_parameter('corridor_release_max_y_m', float('inf'))
        self.declare_parameter('corridor_reacquire_y_m', float('-inf'))
        self.declare_parameter('corridor_reacquire_reverse_speed', -0.12)
        self.declare_parameter('corridor_reacquire_max_reentry_heading_deg', 45.0)
        self.declare_parameter('corridor_reacquire_terminal_margin_m', 0.20)
        self.declare_parameter('corridor_terminal_entry_x_tolerance_m', 0.12)
        self.declare_parameter('corridor_entry_yaw_tolerance_deg', 30.0)
        self.declare_parameter('corridor_require_yaw_for_release', False)
        self.declare_parameter('corridor_terminal_enabled', True)
        self.declare_parameter('corridor_terminal_x_tolerance_m', 0.12)
        self.declare_parameter('corridor_terminal_x_exit_tolerance_m', 0.18)
        self.declare_parameter('corridor_terminal_yaw_tolerance_deg', 8.0)
        self.declare_parameter('corridor_terminal_release_yaw_tolerance_deg', 10.0)
        self.declare_parameter('corridor_terminal_linear_speed', 0.09)
        self.declare_parameter('corridor_terminal_micro_start_y_margin_m', 0.18)
        self.declare_parameter('corridor_terminal_lateral_gain', 1.2)
        self.declare_parameter('corridor_terminal_heading_kp', 1.4)
        self.declare_parameter('corridor_terminal_yaw_deadband_deg', 1.0)
        self.declare_parameter('corridor_terminal_max_angular_speed', 0.40)
        self.declare_parameter('corridor_terminal_micro_max_angular_speed', 0.18)
        self.declare_parameter('corridor_terminal_angular_slew_rate_rad_s2', 0.60)
        self.declare_parameter('corridor_terminal_x_filter_tau_sec', 0.20)
        self.declare_parameter('corridor_terminal_x_reverse_hysteresis_m', 0.015)
        self.declare_parameter('corridor_terminal_prealign_y_margin_m', 0.70)
        self.declare_parameter('corridor_terminal_prealign_heading_kp', 1.2)
        self.declare_parameter('corridor_terminal_prealign_max_angular_speed', 0.25)
        self.declare_parameter('terminal_reverse_align_linear_speed', -0.06)
        self.declare_parameter('terminal_reverse_align_heading_kp', 1.0)
        self.declare_parameter('terminal_reverse_align_max_angular_speed', 0.35)
        # 末端以两侧围墙建立局部通道坐标。激光只提供相对中心/平行误差，
        # 绝对航向仍严格使用 IMU，里程计角度绝不参与控制。
        self.declare_parameter('corridor_terminal_wall_lock_enabled', True)
        self.declare_parameter('corridor_terminal_wall_min_forward_m', 0.10)
        self.declare_parameter('corridor_terminal_wall_max_forward_m', 1.40)
        self.declare_parameter('corridor_terminal_wall_min_lateral_m', 0.16)
        self.declare_parameter('corridor_terminal_wall_max_lateral_m', 1.20)
        self.declare_parameter('corridor_terminal_wall_min_points', 8)
        self.declare_parameter('corridor_terminal_wall_min_span_m', 0.28)
        self.declare_parameter('corridor_terminal_wall_fit_residual_m', 0.035)
        self.declare_parameter('corridor_terminal_wall_axis_tolerance_deg', 25.0)
        # A wall is one spatially continuous scan cluster.  Sparse obstacle
        # returns are deliberately never merged into a candidate wall.
        self.declare_parameter('corridor_terminal_wall_cluster_gap_m', 0.30)
        self.declare_parameter('corridor_terminal_wall_cluster_min_span_m', 0.75)
        self.declare_parameter('corridor_terminal_wall_source_hold_sec', 0.60)
        self.declare_parameter('corridor_terminal_wall_source_heading_jump_deg', 12.0)
        self.declare_parameter('corridor_terminal_wall_source_distance_jump_m', 0.35)
        self.declare_parameter('corridor_terminal_wall_parallel_tolerance_deg', 5.0)
        self.declare_parameter('corridor_terminal_wall_width_min_m', 0.35)
        self.declare_parameter('corridor_terminal_wall_width_max_m', 1.60)
        self.declare_parameter('corridor_terminal_wall_filter_tau_sec', 0.18)
        self.declare_parameter('corridor_terminal_wall_max_age_sec', 0.25)
        # A fitted wall can momentarily disappear behind an entrance edge.
        # Keep it only as a steering reference for this bounded interval;
        # release/commit still require a fresh lock or a qualified latch.
        self.declare_parameter('corridor_terminal_wall_control_hold_sec', 0.75)
        self.declare_parameter('corridor_terminal_wall_latch_max_age_sec', 4.0)
        self.declare_parameter('corridor_terminal_wall_center_offset_m', 0.0)
        self.declare_parameter('corridor_terminal_wall_release_center_tolerance_m', 0.035)
        self.declare_parameter('corridor_terminal_wall_release_heading_deg', 2.5)
        self.declare_parameter('corridor_terminal_commit_y_margin_m', 0.35)
        self.declare_parameter('corridor_terminal_commit_speed_mps', 0.28)
        self.declare_parameter('corridor_terminal_commit_center_tolerance_m', 0.030)
        self.declare_parameter('corridor_terminal_commit_heading_deg', 5.0)
        self.declare_parameter('corridor_terminal_commit_abort_center_m', 0.060)
        self.declare_parameter('corridor_terminal_commit_abort_heading_deg', 12.0)
        self.declare_parameter('corridor_terminal_release_hold_sec', 0.20)
        # Start correcting from the walls well before the handoff gate.  A
        # 15cm initial lateral error needs real travel distance to converge.
        self.declare_parameter('corridor_terminal_capture_y_margin_m', 1.20)
        self.declare_parameter('corridor_terminal_capture_max_center_error_m', 0.12)
        # 二维码倒退结束后，通道全段直接由两侧围墙中线接管方向；
        # map 仅保留沿程交权门线与越界保护，不再参与转向。
        self.declare_parameter('corridor_wall_follow_enabled', True)
        self.declare_parameter('corridor_wall_follow_require_lock', True)
        self.declare_parameter('corridor_wall_follow_linear_speed_mps', 0.36)
        self.declare_parameter('corridor_wall_follow_correction_speed_mps', 0.22)
        self.declare_parameter('corridor_wall_follow_slow_center_error_m', 0.12)
        self.declare_parameter('corridor_wall_follow_lateral_gain', 1.0)
        self.declare_parameter('corridor_wall_follow_heading_kp', 1.4)
        self.declare_parameter('corridor_wall_follow_max_angular_speed', 0.45)
        self.declare_parameter('corridor_wall_follow_no_lock_log_sec', 1.0)
        self.declare_parameter('corridor_wall_acquire_speed_mps', 0.10)
        self.declare_parameter('corridor_wall_acquire_turn_linear_speed_mps', 0.12)
        self.declare_parameter('corridor_wall_acquire_heading_kp', 1.2)
        self.declare_parameter('corridor_wall_acquire_max_angular_speed', 0.45)
        self.declare_parameter('corridor_wall_acquire_turn_in_place_angle_deg', 20.0)
        self.declare_parameter('corridor_wall_follow_require_map_x_handoff', False)
        self.declare_parameter('corridor_wall_follow_single_wall_enabled', True)
        self.declare_parameter('corridor_wall_follow_single_wall_speed_mps', 0.20)
        self.declare_parameter('corridor_path_follow_mode', 'pure_pursuit')  # pure_pursuit | stanley
        self.declare_parameter('corridor_force_reorient_enabled', False)
        self.declare_parameter('corridor_pp_min_lookahead_m', 0.25)
        self.declare_parameter('corridor_pp_speed_scale', 1.0)
        self.declare_parameter('corridor_log_period_sec', 0.5)
        self.declare_parameter('corridor_replan_min_progress_m', 0.35)
        self.declare_parameter('corridor_replan_offpath_m', 0.28)
        self.declare_parameter('corridor_min_cruise_speed', 0.10)
        self.declare_parameter('corridor_max_turn_linear_speed', 0.08)

        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.stage2_cmd_topic = self.get_parameter('stage2_cmd_topic').value
        self.stage3_cmd_topic = self.get_parameter('stage3_cmd_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.imu_map_yaw_offset_topic = self.get_parameter('imu_map_yaw_offset_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.qr_result_topic = self.get_parameter('qr_result_topic').value
        self.phase_topic = self.get_parameter('phase_topic').value
        self.task_topic = self.get_parameter('task_topic').value
        self.stage2_state_topic = self.get_parameter('stage2_state_topic').value
        self.stage3_state_topic = self.get_parameter('stage3_state_topic').value
        self.imu_yaw_offset_rad = math.radians(
            float(self.get_parameter('imu_yaw_offset_deg').value)
        )
        self.imu_initial_map_yaw_rad = math.radians(
            float(self.get_parameter('imu_initial_map_yaw_deg').value)
        )
        self.reset_imu_yaw_on_phase2_handoff = bool(
            self.get_parameter('reset_imu_yaw_on_phase2_handoff').value
        )
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.blind_linear_speed = float(self.get_parameter('blind_linear_speed').value)
        self.blind_qr_slowdown_start_x_m = float(
            self.get_parameter('blind_qr_slowdown_start_x_m').value
        )
        self.blind_qr_slowdown_linear_speed = float(
            self.get_parameter('blind_qr_slowdown_linear_speed').value
        )
        self.blind_angular_speed = float(self.get_parameter('blind_angular_speed').value)
        self.blind_scan_centerline = self._parse_corridor_waypoints(
            str(self.get_parameter('blind_scan_centerline_json').value)
        )
        self.blind_scan_corridor_half_width = max(
            0.05, float(self.get_parameter('blind_scan_corridor_half_width_m').value)
        )
        self.blind_scan_capture_start_odom_x = float(
            self.get_parameter('blind_scan_capture_start_odom_x_m').value
        )
        self.blind_scan_guidance_start_odom_x = max(
            self.blind_scan_capture_start_odom_x,
            float(self.get_parameter('blind_scan_guidance_start_odom_x_m').value),
        )
        self.blind_scan_guidance_ramp = max(
            0.05, float(self.get_parameter('blind_scan_guidance_ramp_m').value)
        )
        self.blind_scan_lateral_kp = max(
            0.0, float(self.get_parameter('blind_scan_lateral_kp').value)
        )
        self.blind_scan_max_heading_offset = math.radians(
            float(self.get_parameter('blind_scan_max_heading_offset_deg').value)
        )
        self.blind_scan_heading_kp = max(
            0.0, float(self.get_parameter('blind_scan_heading_kp').value)
        )
        self.blind_scan_max_angular_speed = max(
            0.0, float(self.get_parameter('blind_scan_max_angular_speed').value)
        )
        self.blind_scan_guidance_max_angular_speed = min(
            self.blind_scan_max_angular_speed,
            max(
                0.0,
                float(self.get_parameter('blind_scan_guidance_max_angular_speed').value),
            ),
        )
        self.blind_scan_avoid_prediction_sec = max(
            0.10, float(self.get_parameter('blind_scan_avoid_prediction_sec').value)
        )
        self.blind_scan_avoid_prediction_step_sec = max(
            0.01, float(self.get_parameter('blind_scan_avoid_prediction_step_sec').value)
        )
        self.blind_scan_avoid_min_clearance = max(
            0.05, float(self.get_parameter('blind_scan_avoid_min_clearance_m').value)
        )
        self.blind_scan_avoid_detection_max_x = max(
            0.18,
            float(self.get_parameter('blind_scan_avoid_detection_max_x_m').value),
        )
        self.blind_scan_escape_reverse_linear_speed = min(
            -0.05, float(self.get_parameter('blind_scan_escape_reverse_linear_speed_mps').value)
        )
        self.blind_scan_escape_reverse_angular_speed = max(
            0.05, float(self.get_parameter('blind_scan_escape_reverse_angular_speed').value)
        )
        self.blind_scan_escape_reverse_duration = max(
            0.10, float(self.get_parameter('blind_scan_escape_reverse_duration_sec').value)
        )
        self.blind_scan_escape_corridor_extra = max(
            0.0, float(self.get_parameter('blind_scan_escape_corridor_extra_m').value)
        )
        self.blind_scan_escape_rear_min_x = float(
            self.get_parameter('blind_scan_escape_rear_min_x_m').value
        )
        self.blind_scan_escape_rear_max_x = float(
            self.get_parameter('blind_scan_escape_rear_max_x_m').value
        )
        self.blind_scan_escape_rear_half_width = max(
            0.05, float(self.get_parameter('blind_scan_escape_rear_half_width_m').value)
        )
        self.avoid_linear_speed = float(self.get_parameter('avoid_linear_speed').value)
        self.avoid_angular_speed = float(self.get_parameter('avoid_angular_speed').value)
        self.avoid_min_duration_sec = float(self.get_parameter('avoid_min_duration_sec').value)
        self.avoid_clear_hold_sec = float(self.get_parameter('avoid_clear_hold_sec').value)
        self.avoid_min_turn_angle_rad = math.radians(
            float(self.get_parameter('avoid_min_turn_angle_deg').value)
        )
        self.avoid_right_turn_left_obstacle_angle_rad = math.radians(
            float(self.get_parameter('avoid_right_turn_left_obstacle_angle_deg').value)
        )
        self.corridor_avoid_goal_bias_enabled = bool(
            self.get_parameter('corridor_avoid_goal_bias_enabled').value
        )
        self.corridor_avoid_obstacle_side_penalty = float(
            self.get_parameter('corridor_avoid_obstacle_side_penalty').value
        )
        self.corridor_obstacle_min_width = max(
            0.01,
            float(self.get_parameter('corridor_obstacle_min_width_m').value),
        )
        self.corridor_reverse_avoid_enabled = bool(
            self.get_parameter('corridor_reverse_avoid_enabled').value
        )
        self.corridor_reverse_avoid_linear_speed = min(
            -0.01, float(self.get_parameter('corridor_reverse_avoid_linear_speed_mps').value)
        )
        self.corridor_reverse_avoid_angular_speed = max(
            0.0, float(self.get_parameter('corridor_reverse_avoid_angular_speed').value)
        )
        self.corridor_reverse_avoid_min_duration = max(
            0.05, float(self.get_parameter('corridor_reverse_avoid_min_duration_sec').value)
        )
        self.corridor_reverse_avoid_clear_hold = max(
            0.0, float(self.get_parameter('corridor_reverse_avoid_clear_hold_sec').value)
        )
        self.corridor_reverse_avoid_entry_tolerance = max(
            0.05, float(self.get_parameter('corridor_reverse_avoid_entry_tolerance_m').value)
        )
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        self.scan_angle_deg = float(self.get_parameter('scan_angle_deg').value)
        self.phase1_window_min_x = float(self.get_parameter('phase1_window_min_x').value)
        self.phase1_window_max_x = float(self.get_parameter('phase1_window_max_x').value)
        self.blind_scan_avoid_detection_max_x = max(
            self.phase1_window_max_x, self.blind_scan_avoid_detection_max_x
        )
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
        self.stage3_cmd_timeout = float(self.get_parameter('stage3_cmd_timeout').value)
        self.transition_stop_duration = float(self.get_parameter('transition_stop_duration').value)
        self.phase2_obstacle_override = bool(self.get_parameter('phase2_obstacle_override').value)
        self.phase2_emergency_stop_distance = float(self.get_parameter('phase2_emergency_stop_distance').value)
        self.phase3_external_control = bool(self.get_parameter('phase3_external_control').value)
        self.phase3_emergency_stop_distance = float(self.get_parameter('phase3_emergency_stop_distance').value)
        self.enable_backing = bool(self.get_parameter('enable_backing').value)
        self.back_target_x = float(self.get_parameter('back_target_x').value)
        self.back_linear_speed = float(self.get_parameter('back_linear_speed').value)
        self.back_angular_kp = float(self.get_parameter('back_angular_kp').value)
        self.back_max_angular_speed = max(
            0.0, float(self.get_parameter('back_max_angular_speed').value)
        )
        self.back_angular_slew_rate = max(
            0.0, float(self.get_parameter('back_angular_slew_rate').value)
        )
        self.back_position_tolerance = float(self.get_parameter('back_position_tolerance').value)
        self.back_path_sample_distance = float(self.get_parameter('back_path_sample_distance').value)
        self.back_lookahead_m = max(
            0.0, float(self.get_parameter('back_lookahead_m').value)
        )
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
        self.corridor_reference_path_enabled = bool(
            self.get_parameter('corridor_reference_path_enabled').value
        )
        self.corridor_reference_path = self._parse_corridor_waypoints(
            str(self.get_parameter('corridor_reference_path_json').value)
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
        self.blind_right_turn_x = float(self.get_parameter('blind_right_turn_x').value)
        self.blind_right_turn_y = float(self.get_parameter('blind_right_turn_y').value)
        self.blind_right_turn_angular = float(self.get_parameter('blind_right_turn_angular').value)
        self.blind_right_turn_linear = float(self.get_parameter('blind_right_turn_linear').value)
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
        self.corridor_release_min_y = float(
            self.get_parameter('corridor_release_min_y_m').value
        )
        self.corridor_release_max_y = max(
            self.corridor_release_min_y,
            float(self.get_parameter('corridor_release_max_y_m').value),
        )
        self.corridor_reacquire_y = min(
            self.corridor_release_min_y,
            float(self.get_parameter('corridor_reacquire_y_m').value),
        )
        self.corridor_reacquire_reverse_speed = min(
            -0.01,
            float(self.get_parameter('corridor_reacquire_reverse_speed').value),
        )
        self.corridor_reacquire_max_reentry_heading = math.radians(max(
            5.0,
            min(80.0, float(self.get_parameter(
                'corridor_reacquire_max_reentry_heading_deg'
            ).value)),
        ))
        self.corridor_reacquire_terminal_margin = max(
            0.0,
            float(self.get_parameter('corridor_reacquire_terminal_margin_m').value),
        )
        self.corridor_entry_yaw_tolerance = math.radians(
            float(self.get_parameter('corridor_entry_yaw_tolerance_deg').value)
        )
        self.corridor_require_yaw_for_release = bool(
            self.get_parameter('corridor_require_yaw_for_release').value
        )
        self.corridor_terminal_enabled = bool(
            self.get_parameter('corridor_terminal_enabled').value
        )
        self.corridor_terminal_x_tolerance = max(
            0.0, float(self.get_parameter('corridor_terminal_x_tolerance_m').value)
        )
        self.corridor_terminal_x_exit_tolerance = max(
            self.corridor_terminal_x_tolerance,
            float(self.get_parameter('corridor_terminal_x_exit_tolerance_m').value),
        )
        self.corridor_terminal_entry_x_tolerance = min(
            self.corridor_terminal_x_exit_tolerance,
            max(
                self.corridor_terminal_x_tolerance,
                float(self.get_parameter('corridor_terminal_entry_x_tolerance_m').value),
            ),
        )
        self.corridor_terminal_yaw_tolerance = math.radians(float(
            self.get_parameter('corridor_terminal_yaw_tolerance_deg').value
        ))
        self.corridor_terminal_release_yaw_tolerance = math.radians(float(
            self.get_parameter('corridor_terminal_release_yaw_tolerance_deg').value
        ))
        self.corridor_terminal_linear_speed = max(
            0.01, float(self.get_parameter('corridor_terminal_linear_speed').value)
        )
        self.corridor_terminal_micro_start_y_margin = max(
            0.0, float(self.get_parameter('corridor_terminal_micro_start_y_margin_m').value)
        )
        self.corridor_terminal_lateral_gain = max(
            0.0, float(self.get_parameter('corridor_terminal_lateral_gain').value)
        )
        self.corridor_terminal_heading_kp = max(
            0.0, float(self.get_parameter('corridor_terminal_heading_kp').value)
        )
        self.corridor_terminal_yaw_deadband = math.radians(float(
            self.get_parameter('corridor_terminal_yaw_deadband_deg').value
        ))
        self.corridor_terminal_max_angular_speed = max(
            0.0, float(self.get_parameter('corridor_terminal_max_angular_speed').value)
        )
        self.corridor_terminal_micro_max_angular_speed = max(
            0.0, min(
                self.corridor_terminal_max_angular_speed,
                float(self.get_parameter('corridor_terminal_micro_max_angular_speed').value),
            )
        )
        self.corridor_terminal_angular_slew_rate = max(
            0.0, float(
                self.get_parameter('corridor_terminal_angular_slew_rate_rad_s2').value
            )
        )
        self.corridor_terminal_x_filter_tau = max(
            0.0, float(self.get_parameter('corridor_terminal_x_filter_tau_sec').value)
        )
        self.corridor_terminal_x_reverse_hysteresis = max(
            0.0, float(
                self.get_parameter('corridor_terminal_x_reverse_hysteresis_m').value
            )
        )
        self.corridor_terminal_prealign_y_margin = max(
            0.0, float(self.get_parameter('corridor_terminal_prealign_y_margin_m').value)
        )
        self.corridor_terminal_prealign_heading_kp = max(
            0.0, float(self.get_parameter('corridor_terminal_prealign_heading_kp').value)
        )
        self.corridor_terminal_prealign_max_angular_speed = max(
            0.0, float(
                self.get_parameter('corridor_terminal_prealign_max_angular_speed').value
            ),
        )
        self.terminal_reverse_align_linear_speed = min(
            -0.01, float(self.get_parameter('terminal_reverse_align_linear_speed').value)
        )
        self.terminal_reverse_align_heading_kp = max(
            0.0, float(self.get_parameter('terminal_reverse_align_heading_kp').value)
        )
        self.terminal_reverse_align_max_angular_speed = max(
            0.0, float(self.get_parameter('terminal_reverse_align_max_angular_speed').value)
        )
        self.corridor_terminal_wall_lock_enabled = bool(
            self.get_parameter('corridor_terminal_wall_lock_enabled').value
        )
        self.corridor_terminal_wall_min_forward = max(
            0.0, float(self.get_parameter('corridor_terminal_wall_min_forward_m').value)
        )
        self.corridor_terminal_wall_max_forward = max(
            self.corridor_terminal_wall_min_forward + 0.05,
            float(self.get_parameter('corridor_terminal_wall_max_forward_m').value),
        )
        self.corridor_terminal_wall_min_lateral = max(
            0.0, float(self.get_parameter('corridor_terminal_wall_min_lateral_m').value)
        )
        self.corridor_terminal_wall_max_lateral = max(
            self.corridor_terminal_wall_min_lateral + 0.05,
            float(self.get_parameter('corridor_terminal_wall_max_lateral_m').value),
        )
        self.corridor_terminal_wall_min_points = max(
            3, int(self.get_parameter('corridor_terminal_wall_min_points').value)
        )
        self.corridor_terminal_wall_min_span = max(
            0.05, float(self.get_parameter('corridor_terminal_wall_min_span_m').value)
        )
        self.corridor_terminal_wall_fit_residual = max(
            0.005, float(self.get_parameter('corridor_terminal_wall_fit_residual_m').value)
        )
        self.corridor_terminal_wall_axis_tolerance = math.radians(float(
            self.get_parameter('corridor_terminal_wall_axis_tolerance_deg').value
        ))
        self.corridor_terminal_wall_cluster_gap = max(
            0.05, float(self.get_parameter('corridor_terminal_wall_cluster_gap_m').value)
        )
        self.corridor_terminal_wall_cluster_min_span = max(
            self.corridor_terminal_wall_min_span,
            float(self.get_parameter('corridor_terminal_wall_cluster_min_span_m').value),
        )
        self.corridor_terminal_wall_source_hold = max(
            0.0, float(self.get_parameter('corridor_terminal_wall_source_hold_sec').value)
        )
        self.corridor_terminal_wall_source_heading_jump = math.radians(float(
            self.get_parameter('corridor_terminal_wall_source_heading_jump_deg').value
        ))
        self.corridor_terminal_wall_source_distance_jump = max(
            0.02, float(
                self.get_parameter('corridor_terminal_wall_source_distance_jump_m').value
            ),
        )
        self.corridor_terminal_wall_parallel_tolerance = math.radians(float(
            self.get_parameter('corridor_terminal_wall_parallel_tolerance_deg').value
        ))
        self.corridor_terminal_wall_width_min = max(
            0.05, float(self.get_parameter('corridor_terminal_wall_width_min_m').value)
        )
        self.corridor_terminal_wall_width_max = max(
            self.corridor_terminal_wall_width_min,
            float(self.get_parameter('corridor_terminal_wall_width_max_m').value),
        )
        self.corridor_terminal_wall_filter_tau = max(
            0.0, float(self.get_parameter('corridor_terminal_wall_filter_tau_sec').value)
        )
        self.corridor_terminal_wall_max_age = max(
            0.05, float(self.get_parameter('corridor_terminal_wall_max_age_sec').value)
        )
        self.corridor_terminal_wall_control_hold = max(
            self.corridor_terminal_wall_max_age,
            float(self.get_parameter('corridor_terminal_wall_control_hold_sec').value),
        )
        self.corridor_terminal_wall_latch_max_age = max(
            self.corridor_terminal_wall_max_age,
            float(self.get_parameter('corridor_terminal_wall_latch_max_age_sec').value),
        )
        self.corridor_terminal_wall_center_offset = float(
            self.get_parameter('corridor_terminal_wall_center_offset_m').value
        )
        self.corridor_terminal_wall_release_center_tolerance = max(
            0.0, float(
                self.get_parameter('corridor_terminal_wall_release_center_tolerance_m').value
            ),
        )
        self.corridor_terminal_wall_release_heading_tolerance = math.radians(float(
            self.get_parameter('corridor_terminal_wall_release_heading_deg').value
        ))
        self.corridor_terminal_commit_y_margin = max(
            0.0, float(self.get_parameter('corridor_terminal_commit_y_margin_m').value)
        )
        self.corridor_terminal_commit_speed = max(
            0.01, float(self.get_parameter('corridor_terminal_commit_speed_mps').value)
        )
        self.corridor_terminal_commit_center_tolerance = max(
            0.0, float(
                self.get_parameter('corridor_terminal_commit_center_tolerance_m').value
            ),
        )
        self.corridor_terminal_commit_heading_tolerance = math.radians(float(
            self.get_parameter('corridor_terminal_commit_heading_deg').value
        ))
        self.corridor_terminal_commit_abort_center = max(
            self.corridor_terminal_commit_center_tolerance,
            float(self.get_parameter('corridor_terminal_commit_abort_center_m').value),
        )
        self.corridor_terminal_commit_abort_heading = math.radians(float(
            self.get_parameter('corridor_terminal_commit_abort_heading_deg').value
        ))
        self.corridor_terminal_release_hold_sec = max(
            0.0, float(self.get_parameter('corridor_terminal_release_hold_sec').value)
        )
        self.corridor_terminal_capture_y_margin = max(
            self.corridor_terminal_commit_y_margin,
            float(self.get_parameter('corridor_terminal_capture_y_margin_m').value),
        )
        self.corridor_terminal_capture_max_center_error = max(
            self.corridor_terminal_commit_center_tolerance,
            float(self.get_parameter('corridor_terminal_capture_max_center_error_m').value),
        )
        self.corridor_wall_follow_enabled = bool(
            self.get_parameter('corridor_wall_follow_enabled').value
        )
        self.corridor_wall_follow_require_lock = bool(
            self.get_parameter('corridor_wall_follow_require_lock').value
        )
        self.corridor_wall_follow_linear_speed = max(
            0.01, float(self.get_parameter('corridor_wall_follow_linear_speed_mps').value)
        )
        self.corridor_wall_follow_correction_speed = min(
            self.corridor_wall_follow_linear_speed,
            max(
                0.01,
                float(self.get_parameter('corridor_wall_follow_correction_speed_mps').value),
            ),
        )
        self.corridor_wall_follow_slow_center_error = max(
            0.01,
            float(self.get_parameter('corridor_wall_follow_slow_center_error_m').value),
        )
        self.corridor_wall_follow_lateral_gain = max(
            0.0, float(self.get_parameter('corridor_wall_follow_lateral_gain').value)
        )
        self.corridor_wall_follow_heading_kp = max(
            0.0, float(self.get_parameter('corridor_wall_follow_heading_kp').value)
        )
        self.corridor_wall_follow_max_angular_speed = max(
            0.0, float(self.get_parameter('corridor_wall_follow_max_angular_speed').value)
        )
        self.corridor_wall_follow_no_lock_log_sec = max(
            0.1, float(self.get_parameter('corridor_wall_follow_no_lock_log_sec').value)
        )
        self.corridor_wall_acquire_speed = max(
            0.01, float(self.get_parameter('corridor_wall_acquire_speed_mps').value)
        )
        self.corridor_wall_acquire_turn_linear_speed = min(
            self.corridor_wall_acquire_speed,
            max(
                0.01,
                float(
                    self.get_parameter('corridor_wall_acquire_turn_linear_speed_mps').value
                ),
            ),
        )
        self.corridor_wall_acquire_heading_kp = max(
            0.0, float(self.get_parameter('corridor_wall_acquire_heading_kp').value)
        )
        self.corridor_wall_acquire_max_angular_speed = max(
            0.0, float(
                self.get_parameter('corridor_wall_acquire_max_angular_speed').value
            ),
        )
        self.corridor_wall_acquire_turn_in_place_angle = math.radians(float(
            self.get_parameter('corridor_wall_acquire_turn_in_place_angle_deg').value
        ))
        self.corridor_wall_follow_require_map_x_handoff = bool(
            self.get_parameter('corridor_wall_follow_require_map_x_handoff').value
        )
        self.corridor_wall_follow_single_wall_enabled = bool(
            self.get_parameter('corridor_wall_follow_single_wall_enabled').value
        )
        self.corridor_wall_follow_single_wall_speed = max(
            0.01, float(self.get_parameter('corridor_wall_follow_single_wall_speed_mps').value)
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

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.phase_pub = self.create_publisher(Int32, self.phase_topic, latched_qos)
        self.task_pub = self.create_publisher(String, self.task_topic, latched_qos)
        self.imu_map_yaw_offset_pub = self.create_publisher(
            Float64, self.imu_map_yaw_offset_topic, latched_qos
        )

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
        self.create_subscription(Twist, self.stage3_cmd_topic, self.stage3_cmd_callback, 10)
        self.create_subscription(String, self.stage2_state_topic, self.stage2_state_callback, 10)
        self.create_subscription(String, self.stage3_state_topic, self.stage3_state_callback, 10)

        self.phase = 1
        self.mission_finished = False
        self.obstacle_found = False
        self.closest_obstacle_distance = float('inf')
        self.avoid_cmd = Twist()
        self.phase1_motion_state = 'forward'
        self.current_yaw = None
        self.current_raw_imu_yaw = None
        self._imu_initial_raw_yaw = None
        self.desired_heading = None
        self.avoid_turn_direction = 0.0
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.blind_scan_escape_attempted = False
        self.blind_scan_escape_pending = False
        self.blind_scan_escape_deadline = None
        self.blind_scan_escape_direction = 0.0
        self.corridor_reverse_started_time = None
        self.corridor_reverse_clear_since = None
        self.corridor_reverse_direction = 0.0
        self.corridor_reverse_last_obstacle = None
        self.corridor_reverse_return_logged = False
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.recovery_uses_heading = False
        self.warned_missing_heading = False
        self.latest_stage2_cmd = Twist()
        self.latest_stage2_cmd_time = None
        self.latest_stage3_cmd = Twist()
        self.latest_stage3_cmd_time = None
        self.transition_end_time = None
        self.qr_task = ''
        self.stage2_state = 'idle'
        # A previous Stage2 completion is not evidence that this Phase2 run
        # has completed. Require an active state from the current run first.
        self.stage2_run_observed = False
        self.stage3_state = 'idle'

        # 路径记录与后退状态
        self.current_odom = None
        self.path_record = []  # [(x, y, yaw), ...]
        self.last_recorded_position = None
        self.backing_started_time = None
        self.backing_path_index = -1
        self.backing_last_angular_z = 0.0
        self.backing_last_command_time = None
        self.aligning_started_time = None
        self.latest_map = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.qr_processed = False  # 二维码去重标志：防止重复扫码播报
        self._map_pose_warned = False
        self.corridor_active = False
        self.corridor_nav_mode = 'idle'  # path_follow | left_recover | idle
        self.corridor_terminal_active = False
        self.corridor_terminal_reverse_align_active = False
        self.corridor_terminal_commit_active = False
        self._terminal_filtered_x_error = None
        self._terminal_x_filter_time = None
        self._terminal_lateral_direction = 0
        self._terminal_last_angular_z = 0.0
        self._terminal_last_angular_time = None
        self._terminal_wall_lock = None
        self._terminal_wall_geometry_latch = None
        self._terminal_wall_filter_time = None
        self._terminal_wall_last_quality = None
        self._corridor_single_wall_lock = None
        self._corridor_single_wall_reference = None
        self._terminal_wall_sources = {'left': None, 'right': None}
        self._corridor_wall_wait_log_time = 0.0
        self._terminal_release_ready_since = None
        self.corridor_reacquire_active = False
        self.corridor_reacquire_target_y = None
        self.corridor_reacquire_rejoin_y = None
        self.corridor_capture_active = False
        self.corridor_align_active = False
        self._node_start_time = self.get_clock().now()
        self.corridor_index = 0
        self.corridor_started_at = None
        self.corridor_entry_pose = None
        self.corridor_path_points = []
        self.corridor_path_updated_at = 0.0
        self.corridor_resume_after_avoidance = False
        self.corridor_entry_reorient_active = False
        self.corridor_entry_reorient_started_at = None
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
        # 始终最短角原地拧，禁止大角度边走边拧把车甩出通道
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

        # 大角度只原地转；误差 <45° 后再极慢前进
        linear = 0.0
        if abs_err < math.radians(45.0):
            linear = min(self.corridor_creep_speed, max(0.02, self.turn_linear_speed * 0.5))

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

        if target_phase == 2:
            self.stage2_state = 'idle'
            self.stage2_run_observed = False
            if (
                self.reset_imu_yaw_on_phase2_handoff
                and self.corridor_terminal_active
            ):
                self._rebase_imu_map_yaw(self.corridor_goal_yaw, 'phase1_to_phase2_handoff')

        if target_phase != 1 and hasattr(self, '_shutdown_vision_corridor'):
            self._shutdown_vision_corridor(f'phase_{target_phase}')

        self.phase = target_phase
        self.publish_phase()
        self.log.mission(f'✓ Phase切换执行: {self.phase-1} → {target_phase}, 原因: {reason}')
        # Preserve the final Stage2 command across the 2->3 phase update.
        # Stage3 takes over only after it has emitted its first command.
        if target_phase != 3:
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

    def _rebase_imu_map_yaw(self, target_yaw, reason):
        """Set the current physical IMU direction as a new shared map yaw."""
        if self.current_raw_imu_yaw is None:
            self.log.warn('IMU', f'IMU_REBASE skipped reason={reason}: no raw yaw')
            return False
        target_yaw = self.normalize_angle(float(target_yaw))
        self._imu_initial_raw_yaw = self.current_raw_imu_yaw
        self.imu_initial_map_yaw_rad = target_yaw
        self.current_yaw = target_yaw
        imu_map_yaw_offset = self.normalize_angle(target_yaw - self.current_raw_imu_yaw)
        self.imu_map_yaw_offset_pub.publish(Float64(data=imu_map_yaw_offset))
        self.log.mission(
            f'IMU_REBASE reason={reason} raw={math.degrees(self.current_raw_imu_yaw):.1f}deg '
            f'-> map={math.degrees(target_yaw):.1f}deg '
            f'offset={math.degrees(imu_map_yaw_offset):+.1f}deg published_to_stage2'
        )
        return True

    def imu_callback(self, msg):
        raw_yaw = self.quaternion_to_yaw(msg.orientation)
        self.current_raw_imu_yaw = raw_yaw
        if self._imu_initial_raw_yaw is None:
            self._imu_initial_raw_yaw = raw_yaw
            imu_map_yaw_offset = self.normalize_angle(
                self.imu_initial_map_yaw_rad - raw_yaw
            )
            self.imu_map_yaw_offset_pub.publish(Float64(data=imu_map_yaw_offset))
            self.log.config(
                f'IMU yaw initialized: raw_start={math.degrees(raw_yaw):.1f}° '
                f'-> map_start={math.degrees(self.imu_initial_map_yaw_rad):.1f}° '
                f'(map_offset={math.degrees(imu_map_yaw_offset):+.1f}°)'
            )

        # 只用 IMU 的相对转角，首帧显式对齐到赛场 map 航向。
        # 不读取 /odom 或 /odom_combined 的 orientation。
        raw_delta = self.normalize_angle(raw_yaw - self._imu_initial_raw_yaw)
        self.current_yaw = self.normalize_angle(
            self.imu_initial_map_yaw_rad + raw_delta
        )
        if self.phase == 1 and self.desired_heading is None:
            self.desired_heading = self.current_yaw
            self.log.config(
                f'phase1 heading locked at {math.degrees(self.desired_heading):.1f} deg '
                f'(initial map yaw={math.degrees(self.imu_initial_map_yaw_rad):.1f} deg)'
            )

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
        """返回通道导航当前目标点；路点按 YAML 顺序依次通过。"""
        if self.corridor_waypoints and 0 <= self.corridor_index < len(self.corridor_waypoints):
            goal = self.corridor_waypoints[self.corridor_index]
            return float(goal['x']), float(goal['y'])
        return None

    def corridor_on_final_waypoint(self):
        return bool(self.corridor_waypoints) and self.corridor_index >= len(self.corridor_waypoints) - 1

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
        self.corridor_nav_mode = 'path_follow'
        self.corridor_terminal_active = False
        self.corridor_terminal_reverse_align_active = False
        self._reset_terminal_lateral_control()
        self.corridor_reacquire_active = False
        self.corridor_reacquire_target_y = None
        self.corridor_reacquire_rejoin_y = None
        self.corridor_capture_active = False
        self.corridor_align_active = False
        self.corridor_entry_reorient_active = False
        self.corridor_entry_reorient_started_at = None
        self._corridor_timeout_logged = False
        # 围墙直跟时，方向不再依赖中继点；直接锁定最终交权门线。
        # 关闭该模式时保留原有中继点路径跟踪。
        self.corridor_index = (
            len(self.corridor_waypoints) - 1
            if self.corridor_wall_follow_enabled
            else 0
        )
        self.corridor_started_at = self.get_clock().now().nanoseconds / 1e9
        self.corridor_entry_pose = self.get_map_position()
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

        # 导航阶段启用 YOLO 视觉推理，辅助通道识别
        if hasattr(self, '_enable_vision_corridor'):
            try:
                self._enable_vision_corridor(True)
            except Exception as e:
                self.log.warn('VISION', f'启用 Stage1 YOLO 视觉推理失败，继续地图导航: {e}')
        mode_text = (
            '两侧围墙中心线直接接管方向，map 仅作交权门线'
            if self.corridor_wall_follow_enabled
            else '地图中心线路径跟踪'
        )
        self.log.mission(f'后退完成，{mode_text}: {reason}')

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
        idx = max(0, min(self.corridor_path_cursor - 1, len(path) - 2))
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
        if self.corridor_reference_path_enabled:
            return (not self.corridor_planned_path), 'reference_empty'
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

    def _reference_centerline_point_at_y(self, target_y):
        """Interpolate the surveyed centerline at a requested forward Y."""
        if not self.corridor_reference_path:
            return None
        points = [
            (float(point['x']), float(point['y']))
            for point in self.corridor_reference_path
        ]
        if target_y <= points[0][1]:
            return points[0]
        for first, second in zip(points, points[1:]):
            x1, y1 = first
            x2, y2 = second
            if y1 <= target_y <= y2:
                if abs(y2 - y1) < 1e-6:
                    return second
                ratio = (target_y - y1) / (y2 - y1)
                return (x1 + ratio * (x2 - x1), float(target_y))
        return points[-1]

    def _select_corridor_reacquire_plan(self, pose_xy):
        """Choose a staging line that leaves room to re-enter the centerline."""
        base_y = min(float(pose_xy[1]), self.corridor_reacquire_y)
        if not self.corridor_reference_path_enabled or not self.corridor_reference_path:
            return base_y, None

        max_rejoin_y = self.corridor_release_min_y - self.corridor_reacquire_terminal_margin
        tangent = math.tan(self.corridor_reacquire_max_reentry_heading)
        candidates = {base_y}
        candidates.update(float(point['y']) for point in self.corridor_reference_path)
        feasible = []
        for candidate_y in candidates:
            if candidate_y > base_y + 1e-6:
                continue
            rejoin_y = candidate_y
            lateral_error = 0.0
            # The centerline bends before the terminal. Iterate so the final
            # target, rather than only the staging point, obeys the angle cap.
            for _ in range(3):
                center_xy = self._reference_centerline_point_at_y(rejoin_y)
                if center_xy is None:
                    break
                lateral_error = abs(float(pose_xy[0]) - center_xy[0])
                rejoin_y = candidate_y + lateral_error / max(tangent, 1e-3)
            if rejoin_y <= max_rejoin_y + 1e-6:
                feasible.append((candidate_y, rejoin_y, lateral_error))

        if feasible:
            staging_y, rejoin_y, _ = max(feasible, key=lambda item: item[0])
            return staging_y, rejoin_y

        # The earliest surveyed point is the only remaining safe reference.
        earliest_y = min(float(point['y']) for point in self.corridor_reference_path)
        rejoin_y = earliest_y
        for _ in range(3):
            center_xy = self._reference_centerline_point_at_y(rejoin_y)
            lateral_error = abs(float(pose_xy[0]) - center_xy[0])
            rejoin_y = earliest_y + lateral_error / max(tangent, 1e-3)
        rejoin_y = min(max_rejoin_y, rejoin_y)
        return min(base_y, earliest_y), rejoin_y

    def _start_corridor_reacquire(self, pose_xy, reason):
        """Reset terminal capture and reverse to a centerline re-entry staging line."""
        staging_y, rejoin_y = self._select_corridor_reacquire_plan(pose_xy)
        self.corridor_reacquire_active = True
        self.corridor_reacquire_target_y = staging_y
        self.corridor_reacquire_rejoin_y = rejoin_y
        self.corridor_terminal_active = False
        self.corridor_terminal_reverse_align_active = False
        self._reset_terminal_lateral_control()
        rejoin_text = f'{rejoin_y:.2f}' if rejoin_y is not None else 'n/a'
        self.log.mission(
            f'{reason} map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}); reverse to '
            f'staging_y={staging_y:.2f}, rejoin_y={rejoin_text}'
        )

    def refresh_corridor_planned_path(self, start_xy, goal_xy, reason='periodic', rejoin_y=None):
        """Use the fast surveyed centerline when enabled, otherwise plan free space."""
        now_ts = self.get_clock().now().nanoseconds / 1e9
        planned = None
        plan_mode = 'fallback_line'
        if self.corridor_reference_path_enabled and self.corridor_reference_path:
            planned = [(float(start_xy[0]), float(start_xy[1]))]
            if rejoin_y is not None:
                rejoin_xy = self._reference_centerline_point_at_y(rejoin_y)
                if rejoin_xy is not None:
                    planned.append(rejoin_xy)
            for point in self.corridor_reference_path:
                point_xy = (float(point['x']), float(point['y']))
                min_y = rejoin_y if rejoin_y is not None else start_xy[1] - 0.05
                if point_xy[1] > min_y + 0.02 and point_xy[1] <= goal_xy[1] + 0.04:
                    planned.append(point_xy)
            if math.hypot(planned[-1][0] - goal_xy[0], planned[-1][1] - goal_xy[1]) > 0.02:
                planned.append((float(goal_xy[0]), float(goal_xy[1])))
            plan_mode = 'reference_reentry' if rejoin_y is not None else 'reference_centerline'
        elif self.use_corridor_planner:
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

    def _forward_path_projection(self, path, pose_xy):
        """Return the nearest projection without allowing progress to move backward."""
        if len(path) < 2:
            return None
        start_segment = max(0, min(self.corridor_path_cursor - 1, len(path) - 2))
        best = None
        for index in range(start_segment, len(path) - 1):
            x1, y1 = path[index]
            x2, y2 = path[index + 1]
            dx = x2 - x1
            dy = y2 - y1
            segment_length = math.hypot(dx, dy)
            if segment_length < 1e-9:
                continue
            ratio = ((pose_xy[0] - x1) * dx + (pose_xy[1] - y1) * dy) / (segment_length * segment_length)
            ratio = max(0.0, min(1.0, ratio))
            projection = (x1 + ratio * dx, y1 + ratio * dy)
            distance = math.hypot(pose_xy[0] - projection[0], pose_xy[1] - projection[1])
            if best is None or distance < best[0]:
                best = (distance, index, ratio, projection, segment_length)
        return best

    def _advance_path_cursor(self, path, pose_xy):
        if not path:
            return 0
        projection = self._forward_path_projection(path, pose_xy)
        if projection is None:
            return len(path) - 1
        _, segment_index, ratio, _, _ = projection
        if ratio > 1e-6:
            self.corridor_path_cursor = max(self.corridor_path_cursor, segment_index + 1)
        return self.corridor_path_cursor

    def _corridor_lookahead(self, path, current_xy):
        if not path:
            self.log.error('CORRIDOR', 'Lookahead: 路径为空！')
            return None
        if current_xy is None:
            return path[-1]
        self._advance_path_cursor(path, current_xy)
        projection = self._forward_path_projection(path, current_xy)
        if projection is None:
            return path[-1]
        _, segment_index, ratio, _, segment_length = projection
        lookahead = max(self.pure_pursuit_lookahead, self.corridor_pp_min_lookahead)
        remaining = lookahead
        index = segment_index
        segment_ratio = ratio
        while index < len(path) - 1:
            x1, y1 = path[index]
            x2, y2 = path[index + 1]
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            available = max(0.0, (1.0 - segment_ratio) * length)
            if remaining <= available and length > 1e-9:
                target_ratio = segment_ratio + remaining / length
                return (x1 + target_ratio * dx, y1 + target_ratio * dy)
            remaining -= available
            index += 1
            segment_ratio = 0.0
        return path[-1]

    def maybe_advance_corridor_waypoint(self, pose_xy):
        """到达中继点后推进到下一个约定点，最后一点由入口逻辑处理。"""
        if not self.corridor_waypoints or self.corridor_on_final_waypoint():
            return False
        target = self.corridor_waypoints[self.corridor_index]
        distance = math.hypot(
            float(target['x']) - pose_xy[0],
            float(target['y']) - pose_xy[1],
        )
        if distance > self.corridor_waypoint_tolerance:
            return False

        previous = self.corridor_index
        self.corridor_index += 1
        next_target = self.corridor_waypoints[self.corridor_index]
        self.corridor_planned_path = []
        self.corridor_path_cursor = 0
        self.corridor_path_updated_at = 0.0
        self.corridor_last_plan_reason = 'waypoint_advance'
        self.log.mission(
            f'corridor waypoint reached index={previous} '
            f'point=({float(target["x"]):.2f},{float(target["y"]):.2f}) '
            f'distance={distance:.2f}m -> index={self.corridor_index} '
            f'point=({float(next_target["x"]):.2f},{float(next_target["y"]):.2f})'
        )
        return True

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
            self.log.mission(
                f'{reason_tag}: map_x={map_x:.2f}>{self.corridor_left_recover_x:.2f} '
                f'v={linear:.2f} w={angular:.2f} 降速运行'
            )
        return self.create_twist(linear, angular)

    def maybe_blind_right_turn_cmd(self):
        """Stage1 盲开末段满足 map 坐标条件时向右旋回。"""
        map_xy = self.get_map_position()
        if map_xy is None:
            return None
        map_x, map_y = float(map_xy[0]), float(map_xy[1])
        if map_y <= self.blind_right_turn_y or map_x <= self.blind_right_turn_x:
            return None
        return self.create_twist(
            max(self.blind_right_turn_linear, 0.05),
            -abs(self.blind_right_turn_angular),
        )

    def blind_scan_is_active(self):
        """二维码尚未锁存时，判断扫码带约束是否已武装。"""
        if (
            self.phase != 1
            or self.qr_processed
            or self.phase1_motion_state in ('backing', 'corridor')
            or len(self.blind_scan_centerline) < 2
            or self.current_odom is None
        ):
            return False
        return (
            float(self.current_odom.pose.pose.position.x)
            >= self.blind_scan_capture_start_odom_x
        )

    def blind_scan_guidance_is_active(self):
        """仅在接近二维码观察位时允许中心线开始引导转向。"""
        return (
            self.blind_scan_is_active()
            and float(self.current_odom.pose.pose.position.x)
            >= self.blind_scan_guidance_start_odom_x
        )

    def blind_scan_guidance_scale(self):
        """在导向切入窗口内平滑增加转向，避免瞬时拉向二维码横带。"""
        if self.current_odom is None:
            return 0.0
        progress = (
            float(self.current_odom.pose.pose.position.x)
            - self.blind_scan_guidance_start_odom_x
        ) / self.blind_scan_guidance_ramp
        progress = self.clamp(progress, 1.0)
        return progress * progress * (3.0 - 2.0 * progress)

    def project_to_blind_scan_centerline(self, point_xy):
        """返回折线投影的有符号横向误差、切线航向和前瞻基点。"""
        if len(self.blind_scan_centerline) < 2:
            return None

        px, py = float(point_xy[0]), float(point_xy[1])
        best = None
        accumulated = 0.0
        for index in range(len(self.blind_scan_centerline) - 1):
            start = self.blind_scan_centerline[index]
            end = self.blind_scan_centerline[index + 1]
            sx, sy = float(start['x']), float(start['y'])
            ex, ey = float(end['x']), float(end['y'])
            dx, dy = ex - sx, ey - sy
            length = math.hypot(dx, dy)
            if length <= 1e-6:
                continue
            tangent_x, tangent_y = dx / length, dy / length
            ratio = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / (length * length)))
            proj_x, proj_y = sx + ratio * dx, sy + ratio * dy
            lateral = tangent_x * (py - proj_y) - tangent_y * (px - proj_x)
            distance_sq = (px - proj_x) ** 2 + (py - proj_y) ** 2
            candidate = {
                'distance_sq': distance_sq,
                'lateral': lateral,
                'heading': math.atan2(tangent_y, tangent_x),
                'segment': index,
                'ratio': ratio,
                'progress': accumulated + ratio * length,
            }
            if best is None or candidate['distance_sq'] < best['distance_sq']:
                best = candidate
            accumulated += length
        return best

    def blind_scan_heading(self, pose_xy):
        """将中心线横向误差换算为 IMU 航向目标，正误差向右收敛。"""
        projection = self.project_to_blind_scan_centerline(pose_xy)
        if projection is None:
            return None, None
        offset = math.atan(self.blind_scan_lateral_kp * projection['lateral'])
        offset = self.clamp(offset, self.blind_scan_max_heading_offset)
        return self.normalize_angle(projection['heading'] - offset), projection

    def blind_scan_guidance_cmd(self):
        """接近二维码观察位后，平滑并入扫码中心线。"""
        if not self.blind_scan_guidance_is_active() or self.current_yaw is None:
            return None
        pose_xy = self.get_map_position()
        if pose_xy is None:
            return None
        target_heading, projection = self.blind_scan_heading(pose_xy)
        if target_heading is None:
            return None
        self.desired_heading = target_heading
        angular = self.clamp(
            self.blind_scan_heading_kp * self.angle_error(target_heading, self.current_yaw),
            self.blind_scan_guidance_max_angular_speed,
        )
        return self.create_twist(
            self.blind_forward_speed(),
            self.blind_scan_guidance_scale() * angular,
        )

    def predict_blind_scan_avoidance(self, direction, obstacle, corridor_extra=0.0):
        """积分预测左/右避障和反舵，检查障碍净距及扫描带横向范围。"""
        pose_xy = self.get_map_position()
        if pose_xy is None or self.current_yaw is None:
            return None
        initial_projection = self.project_to_blind_scan_centerline(pose_xy)
        if initial_projection is None:
            return None

        obstacle_x = float(obstacle.get('center_x', 0.0))
        obstacle_y = float(obstacle.get('center_y', 0.0))
        cos_yaw, sin_yaw = math.cos(self.current_yaw), math.sin(self.current_yaw)
        obstacle_map_xy = (
            pose_xy[0] + cos_yaw * obstacle_x - sin_yaw * obstacle_y,
            pose_xy[1] + sin_yaw * obstacle_x + cos_yaw * obstacle_y,
        )
        minimum_turn_duration = self.avoid_min_turn_angle_rad / max(
            self.avoid_angular_speed, 1e-3
        )
        avoid_duration = max(self.avoid_min_duration_sec, minimum_turn_duration)
        counter_duration = min(
            self.counter_steer_max_duration_sec,
            max(
                self.counter_steer_min_duration_sec,
                avoid_duration * self.counter_steer_duration_scale,
            ),
        )
        phases = (
            (avoid_duration, self.avoid_linear_speed, direction * self.avoid_angular_speed),
            (
                counter_duration,
                self.counter_steer_linear_speed,
                -direction * self.counter_steer_angular_speed,
            ),
        )
        x, y, yaw = float(pose_xy[0]), float(pose_xy[1]), float(self.current_yaw)
        elapsed = 0.0
        max_lateral = abs(initial_projection['lateral'])
        min_obstacle_distance = math.hypot(x - obstacle_map_xy[0], y - obstacle_map_xy[1])
        for duration, linear, angular in phases:
            phase_elapsed = 0.0
            while phase_elapsed < duration:
                dt = min(self.blind_scan_avoid_prediction_step_sec, duration - phase_elapsed)
                x += linear * math.cos(yaw) * dt
                y += linear * math.sin(yaw) * dt
                yaw = self.normalize_angle(yaw + angular * dt)
                phase_elapsed += dt
                elapsed += dt
                projection = self.project_to_blind_scan_centerline((x, y))
                if projection is not None:
                    max_lateral = max(max_lateral, abs(projection['lateral']))
                min_obstacle_distance = min(
                    min_obstacle_distance, math.hypot(x - obstacle_map_xy[0], y - obstacle_map_xy[1])
                )

        # 对反舵后的剩余预测时间，按中心线恢复航向继续积分。
        while elapsed < self.blind_scan_avoid_prediction_sec:
            dt = min(self.blind_scan_avoid_prediction_step_sec, self.blind_scan_avoid_prediction_sec - elapsed)
            target_heading, projection = self.blind_scan_heading((x, y))
            if target_heading is None:
                break
            angular = self.clamp(
                self.recovery_heading_kp * self.angle_error(target_heading, yaw),
                self.recovery_max_angular_speed,
            )
            linear = self.recovery_turn_linear_speed
            x += linear * math.cos(yaw) * dt
            y += linear * math.sin(yaw) * dt
            yaw = self.normalize_angle(yaw + angular * dt)
            elapsed += dt
            max_lateral = max(max_lateral, abs(projection['lateral']))
            min_obstacle_distance = min(
                min_obstacle_distance, math.hypot(x - obstacle_map_xy[0], y - obstacle_map_xy[1])
            )

        final_projection = self.project_to_blind_scan_centerline((x, y))
        final_lateral = final_projection['lateral'] if final_projection is not None else float('inf')
        # 若车已经偏出扫描带，只接受不继续扩大偏差的候选，避免高位继续左绕。
        lateral_limit = (
            max(self.blind_scan_corridor_half_width, abs(initial_projection['lateral']))
            + max(0.0, corridor_extra)
        )
        corridor_ok = max_lateral <= lateral_limit + 1e-3
        obstacle_ok = min_obstacle_distance >= self.blind_scan_avoid_min_clearance
        score = abs(final_lateral) + 0.5 * max_lateral
        return {
            'direction': direction,
            'corridor_ok': corridor_ok,
            'obstacle_ok': obstacle_ok,
            'score': score,
            'max_lateral': max_lateral,
            'final_lateral': final_lateral,
            'min_obstacle_distance': min_obstacle_distance,
        }

    def _terminal_wall_lock_fresh(self, now_ts):
        lock = self._terminal_wall_lock
        if lock is None or now_ts - lock['stamp'] > self.corridor_terminal_wall_max_age:
            return None
        return lock

    def _terminal_wall_lock_for_control(self, now_ts):
        """Return a recent dual-wall lock for continuous steering only."""
        lock = self._terminal_wall_lock
        if (
            lock is None
            or now_ts - lock['stamp'] > self.corridor_terminal_wall_control_hold
        ):
            return None
        return lock

    def _terminal_wall_latch_fresh(self, now_ts):
        latch = self._terminal_wall_geometry_latch
        if latch is None or now_ts - latch['stamp'] > self.corridor_terminal_wall_latch_max_age:
            return None
        return latch

    def _fit_terminal_wall_line(self, points):
        """Fit one continuous wall cluster as y=m*x+b."""
        if len(points) < self.corridor_terminal_wall_min_points:
            return None
        values = np.asarray(points, dtype=float)
        x_values = values[:, 0]
        y_values = values[:, 1]
        if float(np.max(x_values) - np.min(x_values)) < self.corridor_terminal_wall_min_span:
            return None

        design = np.column_stack((x_values, np.ones_like(x_values)))
        slope, intercept = np.linalg.lstsq(design, y_values, rcond=None)[0]
        residuals = np.abs(y_values - (slope * x_values + intercept))
        inliers = residuals <= self.corridor_terminal_wall_fit_residual
        if int(np.count_nonzero(inliers)) < self.corridor_terminal_wall_min_points:
            return None

        x_values = x_values[inliers]
        y_values = y_values[inliers]
        if float(np.max(x_values) - np.min(x_values)) < self.corridor_terminal_wall_min_span:
            return None
        design = np.column_stack((x_values, np.ones_like(x_values)))
        slope, intercept = np.linalg.lstsq(design, y_values, rcond=None)[0]
        rms = float(np.sqrt(np.mean((y_values - (slope * x_values + intercept)) ** 2)))
        if rms > self.corridor_terminal_wall_fit_residual:
            return None
        span = float(np.max(x_values) - np.min(x_values))
        if span < self.corridor_terminal_wall_cluster_min_span:
            return None
        return float(slope), float(intercept), int(x_values.size), rms, span

    def _cluster_terminal_wall_points(self, scan_msg):
        """Split side-wall returns when adjacent Cartesian points have a gap."""
        clusters = {'left': [], 'right': []}
        active_side = None
        active_points = []
        previous_point = None
        max_forward = self.corridor_terminal_wall_max_forward
        if scan_msg.range_max > 0.0:
            max_forward = min(max_forward, float(scan_msg.range_max))

        def finish_active():
            nonlocal active_side, active_points, previous_point
            if active_side is not None and active_points:
                clusters[active_side].append(active_points)
            active_side = None
            active_points = []
            previous_point = None

        for index, distance in enumerate(scan_msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance < self.min_valid_range:
                finish_active()
                continue
            angle = scan_msg.angle_min + index * scan_msg.angle_increment
            point = (distance * math.cos(angle), distance * math.sin(angle))
            point_x, point_y = point
            if not self.corridor_terminal_wall_min_forward <= point_x <= max_forward:
                finish_active()
                continue
            if self.corridor_terminal_wall_min_lateral <= point_y <= self.corridor_terminal_wall_max_lateral:
                side = 'left'
            elif -self.corridor_terminal_wall_max_lateral <= point_y <= -self.corridor_terminal_wall_min_lateral:
                side = 'right'
            else:
                finish_active()
                continue

            separated = (
                side != active_side
                or previous_point is None
                or math.hypot(point_x - previous_point[0], point_y - previous_point[1])
                > self.corridor_terminal_wall_cluster_gap
            )
            if separated:
                finish_active()
                active_side = side
            active_points.append(point)
            previous_point = point
        finish_active()
        return clusters

    @staticmethod
    def _wall_cluster_rank(line):
        """Prefer a full long wall over a short local return."""
        return (line[4], line[2], -line[3])

    def _terminal_wall_axis_error(self, line):
        """Return the undirected wall-axis error against the corridor axis."""
        if self.current_yaw is None:
            return float('inf')
        return self._terminal_wall_heading_axis_error(math.atan(line[0]), self.current_yaw)

    def _terminal_wall_heading_axis_error(self, relative_heading, yaw):
        """Compare a local wall tangent with the fixed IMU exit axis."""
        wall_heading = self.normalize_angle(yaw + relative_heading)
        difference = abs(self.angle_error(self.corridor_goal_yaw, wall_heading))
        return min(difference, abs(math.pi - difference))

    def _select_terminal_wall_cluster(self, side, candidates, now_ts):
        """Keep a side attached to one compatible physical wall source."""
        if not candidates:
            return None
        candidates = sorted(candidates, key=self._wall_cluster_rank, reverse=True)
        source = self._terminal_wall_sources.get(side)
        source_was_fresh = (
            source is not None
            and now_ts - source['stamp'] <= self.corridor_terminal_wall_source_hold
        )
        if source is not None and now_ts - source['stamp'] <= self.corridor_terminal_wall_source_hold:
            compatible = [
                line for line in candidates
                if abs(math.atan(line[0]) - source['heading_error'])
                <= self.corridor_terminal_wall_source_heading_jump
                and abs(abs(line[1]) - source['distance'])
                <= self.corridor_terminal_wall_source_distance_jump
            ]
            if not compatible:
                return None
            selected = max(compatible, key=self._wall_cluster_rank)
        else:
            selected = candidates[0]
        self._terminal_wall_sources[side] = {
            'stamp': now_ts,
            'heading_error': math.atan(selected[0]),
            'distance': abs(selected[1]),
            'span': selected[4],
        }
        if not source_was_fresh:
            self.log.segment(
                f'wall_cluster_source_lock side={side} pts={selected[2]} '
                f'span={selected[4]:.2f}m rms={selected[3]:.3f}m '
                f'dist={abs(selected[1]):.2f}m '
                f'tangent={math.degrees(math.atan(selected[0])):.1f}deg '
                f'gap<={self.corridor_terminal_wall_cluster_gap:.2f}m'
            )
        return selected

    def _update_corridor_single_wall_lock(self, side, line, now_ts):
        """Keep a usable single side wall as a bootstrap direction source."""
        slope, intercept, point_count, rms, span = line
        heading_error = math.atan(slope)
        distance = abs(intercept)
        previous = self._corridor_single_wall_lock
        if previous is not None and previous['side'] == side:
            elapsed = max(0.0, min(0.25, now_ts - previous['stamp']))
            if self.corridor_terminal_wall_filter_tau > 1e-6:
                alpha = elapsed / (self.corridor_terminal_wall_filter_tau + elapsed)
                heading_error = previous['heading_error'] + alpha * (
                    heading_error - previous['heading_error']
                )
                distance = previous['distance'] + alpha * (distance - previous['distance'])
        self._corridor_single_wall_lock = {
            'stamp': now_ts,
            'side': side,
            'heading_error': heading_error,
            'distance': distance,
            'point_count': point_count,
            'rms': rms,
            'span': span,
        }

    def _update_terminal_wall_lock(self, scan_msg):
        """Build a local corridor frame from the two side walls ahead of the car."""
        if (
            not self.corridor_terminal_wall_lock_enabled
            or self.phase != 1
            or self.phase1_motion_state not in ('corridor', 'corridor_reverse_avoid')
        ):
            return

        now_ts = self.get_clock().now().nanoseconds / 1e9
        point_clusters = self._cluster_terminal_wall_points(scan_msg)
        left_points = sum(len(cluster) for cluster in point_clusters['left'])
        right_points = sum(len(cluster) for cluster in point_clusters['right'])
        left_geometry_candidates = [
            line for cluster in point_clusters['left']
            if (line := self._fit_terminal_wall_line(cluster)) is not None
        ]
        right_geometry_candidates = [
            line for cluster in point_clusters['right']
            if (line := self._fit_terminal_wall_line(cluster)) is not None
        ]
        left_candidates = [
            line for line in left_geometry_candidates
            if self._terminal_wall_axis_error(line) <= self.corridor_terminal_wall_axis_tolerance
        ]
        right_candidates = [
            line for line in right_geometry_candidates
            if self._terminal_wall_axis_error(line) <= self.corridor_terminal_wall_axis_tolerance
        ]
        left_line = self._select_terminal_wall_cluster('left', left_candidates, now_ts)
        right_line = self._select_terminal_wall_cluster('right', right_candidates, now_ts)
        if left_line is None or right_line is None:
            usable_side = None
            usable_line = None
            current_single = self._corridor_single_wall_lock
            if current_single is not None and now_ts - current_single['stamp'] <= self.corridor_terminal_wall_source_hold:
                selected = left_line if current_single['side'] == 'left' else right_line
                if selected is not None:
                    usable_side, usable_line = current_single['side'], selected
            if usable_line is None:
                available = [
                    (side, line) for side, line in (('left', left_line), ('right', right_line))
                    if line is not None
                ]
                if available:
                    usable_side, usable_line = max(available, key=lambda item: self._wall_cluster_rank(item[1]))
            if usable_line is not None and self.corridor_wall_follow_single_wall_enabled:
                self._update_corridor_single_wall_lock(usable_side, usable_line, now_ts)
            self._terminal_wall_last_quality = {
                'stamp': now_ts,
                'reason': (
                    'left_source' if left_candidates
                    else ('left_axis' if left_geometry_candidates else 'left_fit')
                ) if left_line is None else (
                    'right_source' if right_candidates
                    else ('right_axis' if right_geometry_candidates else 'right_fit')
                ),
                'left_candidates': left_points,
                'right_candidates': right_points,
                'left_clusters': len(point_clusters['left']),
                'right_clusters': len(point_clusters['right']),
                'left_wall_clusters': len(left_candidates),
                'right_wall_clusters': len(right_candidates),
                'left_geometry_clusters': len(left_geometry_candidates),
                'right_geometry_clusters': len(right_geometry_candidates),
            }
            return
        left_slope, left_intercept, left_count, left_rms, left_span = left_line
        right_slope, right_intercept, right_count, right_rms, right_span = right_line
        parallel_error = abs(math.atan(left_slope) - math.atan(right_slope))
        width = left_intercept - right_intercept
        if (
            parallel_error > self.corridor_terminal_wall_parallel_tolerance
            or not self.corridor_terminal_wall_width_min <= width <= self.corridor_terminal_wall_width_max
        ):
            if self.corridor_wall_follow_single_wall_enabled:
                current_single = self._corridor_single_wall_lock
                if current_single is not None and current_single['side'] == 'left':
                    self._update_corridor_single_wall_lock('left', left_line, now_ts)
                elif current_single is not None and current_single['side'] == 'right':
                    self._update_corridor_single_wall_lock('right', right_line, now_ts)
                elif left_rms <= right_rms:
                    self._update_corridor_single_wall_lock('left', left_line, now_ts)
                else:
                    self._update_corridor_single_wall_lock('right', right_line, now_ts)
            self._terminal_wall_last_quality = {
                'stamp': now_ts,
                'reason': 'parallel' if parallel_error > self.corridor_terminal_wall_parallel_tolerance else 'width',
                'left_candidates': left_points,
                'right_candidates': right_points,
                'left_clusters': len(point_clusters['left']),
                'right_clusters': len(point_clusters['right']),
                'left_wall_clusters': len(left_candidates),
                'right_wall_clusters': len(right_candidates),
                'width': width,
                'parallel_error': parallel_error,
            }
            return

        raw_center_error = 0.5 * (left_intercept + right_intercept)
        raw_center_error -= self.corridor_terminal_wall_center_offset
        raw_heading_error = math.atan(0.5 * (left_slope + right_slope))
        previous = self._terminal_wall_lock
        if previous is not None and self._terminal_wall_filter_time is not None:
            elapsed = max(0.0, min(0.25, now_ts - self._terminal_wall_filter_time))
            if self.corridor_terminal_wall_filter_tau > 1e-6:
                alpha = elapsed / (self.corridor_terminal_wall_filter_tau + elapsed)
                raw_center_error = previous['center_error'] + alpha * (
                    raw_center_error - previous['center_error']
                )
                raw_heading_error = previous['heading_error'] + alpha * (
                    raw_heading_error - previous['heading_error']
                )
        self._terminal_wall_lock = {
            'stamp': now_ts,
            'center_error': raw_center_error,
            'heading_error': raw_heading_error,
            'width': width,
            'left_count': left_count,
            'right_count': right_count,
            'left_span': left_span,
            'right_span': right_span,
            'rms': max(left_rms, right_rms),
        }
        self._terminal_wall_filter_time = now_ts
        wall_axis_error = float('inf')
        if self.current_yaw is not None:
            wall_axis_error = self._terminal_wall_heading_axis_error(
                raw_heading_error, self.current_yaw
            )
        if (
            abs(raw_center_error) <= self.corridor_terminal_wall_release_center_tolerance
            and wall_axis_error <= self.corridor_terminal_wall_axis_tolerance
        ):
            self._terminal_wall_geometry_latch = {
                'stamp': now_ts,
                'center_error': raw_center_error,
                'axis_error': wall_axis_error,
                'width': width,
            }
        self._corridor_single_wall_lock = None
        self._terminal_wall_last_quality = {
            'stamp': now_ts,
            'reason': 'ok',
            'left_candidates': left_points,
            'right_candidates': right_points,
            'left_clusters': len(point_clusters['left']),
            'right_clusters': len(point_clusters['right']),
            'left_wall_clusters': len(left_candidates),
            'right_wall_clusters': len(right_candidates),
            'width': width,
            'parallel_error': parallel_error,
        }

    def _follow_corridor_walls(self, pose_xy, yaw, now_ts):
        """Direct corridor steering from the two side walls after QR backing."""
        wall_lock = self._terminal_wall_lock_fresh(now_ts)
        if wall_lock is None:
            single_wall = self._corridor_single_wall_lock
            if (
                self.corridor_wall_follow_single_wall_enabled
                and single_wall is not None
                and now_ts - single_wall['stamp'] <= self.corridor_terminal_wall_max_age
            ):
                # One wall determines only its tangent.  Its absolute range
                # is not a centerline measurement and must never pull the
                # vehicle toward a guessed field-center distance.
                relative_target_error = single_wall['heading_error']
                desired_yaw = self.normalize_angle(yaw + relative_target_error)
                angular = self.clamp(
                    self.corridor_wall_follow_heading_kp * relative_target_error,
                    self.corridor_wall_follow_max_angular_speed,
                )
                angular = self._limit_terminal_angular_command(angular, now_ts)
                self.corridor_nav_mode = 'corridor_single_wall_follow'
                self.corridor_desired_heading = desired_yaw
                self.cmd_pub.publish(self.create_twist(
                    self.corridor_wall_follow_single_wall_speed, angular
                ))
                if now_ts - self._corridor_last_log_time >= self.corridor_log_period_sec:
                    self._corridor_last_log_time = now_ts
                    self.log.segment(
                        f'corridor_single_wall_follow side={single_wall["side"]} '
                        f'map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                        f'dist={single_wall["distance"]:.2f}m '
                        f'cluster_pts={single_wall["point_count"]} '
                        f'cluster_span={single_wall["span"]:.2f}m '
                        f'rms={single_wall["rms"]:.3f}m '
                        f'parallel={math.degrees(single_wall["heading_error"]):.1f}deg '
                        f'v={self.corridor_wall_follow_single_wall_speed:.2f} w={angular:.2f}'
                    )
                return True

            heading_error = self.angle_error(self.corridor_goal_yaw, yaw)
            angular = self.clamp(
                self.corridor_wall_acquire_heading_kp * heading_error,
                self.corridor_wall_acquire_max_angular_speed,
            )
            # Ackermann steering needs rolling motion.  Keep a minimum
            # forward speed during the large-angle acquisition turn.
            linear = (
                self.corridor_wall_acquire_turn_linear_speed
                if abs(heading_error) > self.corridor_wall_acquire_turn_in_place_angle
                else self.corridor_wall_acquire_speed
            )
            self.corridor_nav_mode = 'wall_acquire_align'
            self.corridor_desired_heading = self.corridor_goal_yaw
            self.cmd_pub.publish(self.create_twist(linear, angular))
            if now_ts - self._corridor_wall_wait_log_time >= self.corridor_wall_follow_no_lock_log_sec:
                self._corridor_wall_wait_log_time = now_ts
                quality = self._terminal_wall_last_quality
                if quality is None:
                    detail = 'no /scan wall sample received'
                else:
                    detail = (
                        f'reject={quality["reason"]} left_pts={quality["left_candidates"]} '
                        f'right_pts={quality["right_candidates"]}'
                    )
                    if 'width' in quality:
                        detail += f' width={quality["width"]:.2f}m'
                    detail += (
                        f' clusters=L{quality.get("left_clusters", 0)}'
                        f'/R{quality.get("right_clusters", 0)}'
                        f' geometry=L{quality.get("left_geometry_clusters", 0)}'
                        f'/R{quality.get("right_geometry_clusters", 0)}'
                        f' wall_clusters=L{quality.get("left_wall_clusters", 0)}'
                        f'/R{quality.get("right_wall_clusters", 0)}'
                    )
                self.log.warn(
                    'CORRIDOR',
                    f'wall_acquire_align map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                    f'{detail} desired={math.degrees(self.corridor_goal_yaw):.1f}deg '
                    f'err={math.degrees(heading_error):.1f}deg '
                    f'v={linear:.2f} w={angular:.2f}'
                )
            return self.corridor_wall_follow_require_lock

        relative_target_error = wall_lock['heading_error'] + math.atan(
            self.corridor_wall_follow_lateral_gain * wall_lock['center_error']
        )
        desired_yaw = self.normalize_angle(yaw + relative_target_error)
        angular = self.clamp(
            self.corridor_wall_follow_heading_kp * relative_target_error,
            self.corridor_wall_follow_max_angular_speed,
        )
        angular = self._limit_terminal_angular_command(angular, now_ts)
        center_abs = abs(wall_lock['center_error'])
        linear = (
            self.corridor_wall_follow_correction_speed
            if center_abs > self.corridor_wall_follow_slow_center_error
            else self.corridor_wall_follow_linear_speed
        )
        self.corridor_nav_mode = 'corridor_wall_follow'
        self.corridor_desired_heading = desired_yaw
        self.cmd_pub.publish(self.create_twist(linear, angular))

        if now_ts - self._corridor_last_log_time >= self.corridor_log_period_sec:
            self._corridor_last_log_time = now_ts
            self.log.segment(
                f'corridor_wall_follow map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                f'center={wall_lock["center_error"]:.3f}m '
                f'parallel={math.degrees(wall_lock["heading_error"]):.1f}deg '
                f'width={wall_lock["width"]:.2f}m '
                f'cluster_span=L{wall_lock["left_span"]:.2f}/R{wall_lock["right_span"]:.2f}m '
                f'v={linear:.2f} w={angular:.2f}'
            )
        return True

    def _corridor_region_release_ready(self, pose_xy, goal_xy, yaw, now_ts):
        rho = math.hypot(goal_xy[0] - pose_xy[0], goal_xy[1] - pose_xy[1])
        yaw_error = self.angle_error(self.corridor_goal_yaw, yaw)
        radius_ok = rho <= self.corridor_entry_region_radius
        # Before terminal commit, an upper Y bound prevents a failed approach
        # from handing over beyond the entry.  Once a qualified wall-center
        # latch has committed the final straight segment, map-Y drift must not
        # force a full reverse/retry cycle.
        y_gate_ok = (
            self.corridor_release_min_y <= pose_xy[1]
            and (
                pose_xy[1] <= self.corridor_release_max_y
                or self.corridor_terminal_commit_active
            )
        )
        x_error = goal_xy[0] - pose_xy[0]
        x_ok = abs(x_error) <= self.corridor_terminal_x_tolerance
        map_x_required = (
            not self.corridor_wall_follow_enabled
            or self.corridor_wall_follow_require_map_x_handoff
        )
        position_x_ok = x_ok or not map_x_required
        wall_lock = self._terminal_wall_lock_fresh(now_ts)
        wall_latch = self._terminal_wall_latch_fresh(now_ts)
        wall_ok = not self.corridor_terminal_wall_lock_enabled
        use_commit_latch = self.corridor_terminal_commit_active and wall_latch is not None
        if use_commit_latch:
            wall_ok = (
                abs(wall_latch['center_error'])
                <= self.corridor_terminal_wall_release_center_tolerance
                and wall_latch['axis_error'] <= self.corridor_terminal_wall_axis_tolerance
            )
        elif wall_lock is not None:
            wall_ok = (
                abs(wall_lock['center_error'])
                <= self.corridor_terminal_wall_release_center_tolerance
                and self._terminal_wall_heading_axis_error(
                    wall_lock['heading_error'], yaw
                ) <= self.corridor_terminal_wall_axis_tolerance
            )
        elif wall_latch is not None:
            wall_ok = (
                abs(wall_latch['center_error'])
                <= self.corridor_terminal_wall_release_center_tolerance
                and wall_latch['axis_error'] <= self.corridor_terminal_wall_axis_tolerance
            )
        if self.corridor_terminal_enabled:
            pos_ok = position_x_ok and y_gate_ok
            yaw_ok = abs(yaw_error) <= self.corridor_terminal_release_yaw_tolerance
            candidate = (
                self.corridor_terminal_active
                and pos_ok
                and yaw_ok
                and wall_ok
            )
        else:
            pos_ok = (radius_ok or not map_x_required) and y_gate_ok
            yaw_ok = abs(yaw_error) <= self.corridor_entry_yaw_tolerance
            candidate = pos_ok and (yaw_ok if self.corridor_require_yaw_for_release else True)

        if candidate:
            if self._terminal_release_ready_since is None:
                self._terminal_release_ready_since = now_ts
            ready = now_ts - self._terminal_release_ready_since >= self.corridor_terminal_release_hold_sec
        else:
            self._terminal_release_ready_since = None
            ready = False
        return (
            ready, rho, yaw_error, pos_ok, yaw_ok, radius_ok, y_gate_ok,
            x_error, x_ok, wall_ok, wall_lock,
        )

    def _reset_terminal_lateral_control(self):
        self._terminal_filtered_x_error = None
        self._terminal_x_filter_time = None
        self._terminal_lateral_direction = 0
        self._terminal_last_angular_z = 0.0
        self._terminal_last_angular_time = None
        self.corridor_terminal_commit_active = False
        self._terminal_release_ready_since = None
        self._terminal_wall_lock = None
        self._terminal_wall_geometry_latch = None
        self._terminal_wall_filter_time = None
        self._terminal_wall_last_quality = None
        self._terminal_wall_sources = {'left': None, 'right': None}
        self._corridor_single_wall_lock = None

    def _terminal_lateral_error_for_control(self, raw_x_error, now_ts):
        """Filter terminal X feedback and reject small direction reversals."""
        if self._terminal_filtered_x_error is None or self._terminal_x_filter_time is None:
            self._terminal_filtered_x_error = raw_x_error
        else:
            elapsed = max(0.0, min(0.25, now_ts - self._terminal_x_filter_time))
            if self.corridor_terminal_x_filter_tau <= 1e-6:
                self._terminal_filtered_x_error = raw_x_error
            else:
                alpha = elapsed / (self.corridor_terminal_x_filter_tau + elapsed)
                self._terminal_filtered_x_error += alpha * (
                    raw_x_error - self._terminal_filtered_x_error
                )
        self._terminal_x_filter_time = now_ts

        filtered_error = self._terminal_filtered_x_error
        hysteresis = self.corridor_terminal_x_reverse_hysteresis
        if abs(filtered_error) <= hysteresis:
            return 0.0

        direction = 1 if filtered_error > 0.0 else -1
        if (
            self._terminal_lateral_direction
            and direction != self._terminal_lateral_direction
            and abs(filtered_error) < 2.0 * hysteresis
        ):
            return 0.0
        self._terminal_lateral_direction = direction
        return filtered_error

    def _limit_terminal_angular_command(self, requested_angular, now_ts):
        """Limit terminal steering acceleration so IMU/TF noise cannot flip the wheels."""
        if self._terminal_last_angular_time is None or self.corridor_terminal_angular_slew_rate <= 0.0:
            angular = requested_angular
        else:
            elapsed = max(0.0, min(0.25, now_ts - self._terminal_last_angular_time))
            max_delta = self.corridor_terminal_angular_slew_rate * elapsed
            angular = max(
                self._terminal_last_angular_z - max_delta,
                min(self._terminal_last_angular_z + max_delta, requested_angular),
            )
        self._terminal_last_angular_z = angular
        self._terminal_last_angular_time = now_ts
        return angular

    def handle_corridor_navigation(self):
        """
        Stage1 通道导航（区域进入）：
          1) A* 规划自由空间路径（失败则直线 fallback）
          2) Pure Pursuit 跟踪路径（允许斜穿）
          3) 进入入口区域即切 Stage2；默认不强制精准航向
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
                    f'region entry TIMEOUT {self.corridor_timeout_sec:.1f}s, hold without Stage2 | '
                    f'map=({map_xy[0]:.2f},{map_xy[1]:.2f}) yaw={math.degrees(self.current_yaw):.1f}deg '
                    f'rho={rho_txt} plans={getattr(self, "_corridor_plan_count", 0)} '
                    f'last_plan={self.corridor_last_plan_reason or "none"}'
                )
            self.stop_robot()
            return

        pose_xy = (float(map_xy[0]), float(map_xy[1]))
        yaw = float(self.current_yaw)
        if self.maybe_advance_corridor_waypoint(pose_xy):
            # 切换中继点后本周期重新以新目标规划，避免沿旧路径继续走。
            goal_xy = self.corridor_goal_point()
            if goal_xy is not None:
                self.refresh_corridor_planned_path(pose_xy, goal_xy, reason='waypoint_advance')
                self.publish_corridor_path(pose_xy)
        goal_xy = self.corridor_goal_point()
        if goal_xy is None:
            self.log.error('CORRIDOR', '无入口目标点，直接放行 Stage2')
            self.begin_phase_transition(2, '无入口目标点，直接进入 Stage2')
            return
        goal_xy = (float(goal_xy[0]), float(goal_xy[1]))
        final_waypoint = self.corridor_on_final_waypoint()

        # A post-avoidance detour can carry the vehicle past the handoff gate.
        # Never hand off from that high-Y position: reverse to a centerline
        # staging line that leaves enough distance for a bounded-angle re-entry.
        if (
            final_waypoint
            and pose_xy[1] > self.corridor_release_max_y
            and not self.corridor_terminal_commit_active
        ):
            if not self.corridor_reacquire_active:
                self._start_corridor_reacquire(
                    pose_xy,
                    f'handoff Y overrun max_y={self.corridor_release_max_y:.2f}',
                )

        if self.corridor_reacquire_active:
            staging_y = self.corridor_reacquire_target_y
            if staging_y is None:
                staging_y = self.corridor_reacquire_y
            if pose_xy[1] <= staging_y:
                self.corridor_reacquire_active = False
                self.corridor_nav_mode = 'path_follow'
                self.refresh_corridor_planned_path(
                    pose_xy,
                    goal_xy,
                    reason='handoff_y_reacquire',
                    rejoin_y=self.corridor_reacquire_rejoin_y,
                )
                self.publish_corridor_path(pose_xy)
                self.log.mission(
                    f'handoff Y reacquire complete map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}); '
                    f'rejoin_y={self.corridor_reacquire_rejoin_y if self.corridor_reacquire_rejoin_y is not None else float("nan"):.2f}'
                )
                self.corridor_reacquire_target_y = None
                self.corridor_reacquire_rejoin_y = None
                return

            reverse_yaw_error = self.angle_error(self.corridor_goal_yaw, yaw)
            angular = self.clamp(
                self.terminal_reverse_align_heading_kp * reverse_yaw_error,
                self.terminal_reverse_align_max_angular_speed,
            )
            self.corridor_nav_mode = 'handoff_y_reacquire'
            self.corridor_desired_heading = self.corridor_goal_yaw
            self.cmd_pub.publish(self.create_twist(
                self.corridor_reacquire_reverse_speed, angular
            ))
            if now_ts - self._corridor_last_log_time >= self.corridor_log_period_sec:
                self._corridor_last_log_time = now_ts
                self.log.segment(
                    f'handoff_y_reacquire map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                    f'max_y={self.corridor_release_max_y:.2f} '
                    f'staging_y={staging_y:.2f} '
                    f'rejoin_y={self.corridor_reacquire_rejoin_y if self.corridor_reacquire_rejoin_y is not None else float("nan"):.2f} '
                    f'yaw={math.degrees(yaw):.1f}deg '
                    f'v={self.corridor_reacquire_reverse_speed:.2f} w={angular:.2f}'
                )
            return

        # 末端位置已经到位但航向未对齐时，才允许后退回正。通道中段始终正向跟踪。
        x_error = goal_xy[0] - pose_xy[0]
        yaw_to_goal_error = self.angle_error(self.corridor_goal_yaw, yaw)
        wall_capture_lock = self._terminal_wall_lock_fresh(now_ts)
        wall_capture_ok = (
            self.corridor_wall_follow_enabled
            and wall_capture_lock is not None
            and abs(wall_capture_lock['center_error'])
            <= self.corridor_terminal_capture_max_center_error
        )
        terminal_geometry_ready = (
            final_waypoint
            and self.corridor_terminal_enabled
            # Start the wall-based capture before the release gate so the
            # vehicle has real distance to converge rather than correcting at
            # the gate itself.
            and (
                self.corridor_release_min_y - self.corridor_terminal_capture_y_margin
                <= pose_xy[1] <= self.corridor_release_max_y
            )
            # Capture from the wider band; terminal control then converges to
            # the strict release band instead of leaving micro-correction dead.
            and (
                abs(x_error) <= self.corridor_terminal_entry_x_tolerance
                or wall_capture_ok
            )
        )
        terminal_entry_ok = (
            terminal_geometry_ready
            and abs(yaw_to_goal_error) <= self.corridor_terminal_yaw_tolerance
        )
        if terminal_entry_ok and not self.corridor_terminal_active:
            self.corridor_terminal_active = True
            self.corridor_terminal_reverse_align_active = False
            self.corridor_nav_mode = 'terminal_approach'
            self.log.mission(
                f'terminal approach latched map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                f'xerr={x_error:.3f}m capture_tol={self.corridor_terminal_entry_x_tolerance:.3f}m '
                f'yaw_err={math.degrees(yaw_to_goal_error):.1f}deg '
                f'release_y={self.corridor_release_min_y:.2f}'
            )
        elif (
            terminal_geometry_ready
            and pose_xy[1] >= self.corridor_release_min_y
            and not self.corridor_terminal_active
        ):
            if not self.corridor_terminal_reverse_align_active:
                self.corridor_terminal_reverse_align_active = True
                self.corridor_nav_mode = 'terminal_reverse_align'
                self.log.mission(
                    f'terminal reverse align enter map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                    f'xerr={x_error:.3f}m yaw={math.degrees(yaw):.1f}deg '
                    f'err={math.degrees(yaw_to_goal_error):.1f}deg '
                    f'target={math.degrees(self.corridor_goal_yaw):.1f}deg'
                )
        elif (
            self.corridor_terminal_active
            and pose_xy[1] < self.corridor_release_min_y
            and not self.corridor_wall_follow_enabled
            and abs(x_error) > self.corridor_terminal_x_exit_tolerance
        ):
            self.corridor_terminal_active = False
            self._reset_terminal_lateral_control()
            self.corridor_nav_mode = 'path_follow'
            self.log.warn(
                'CORRIDOR',
                f'terminal approach released: xerr={x_error:.3f}m exceeds '
                f'exit tolerance {self.corridor_terminal_x_exit_tolerance:.3f}m'
            )

        (
            ready, rho, yaw_error, pos_ok, yaw_ok, radius_ok, y_gate_ok,
            x_error, x_ok, wall_ok, wall_lock,
        ) = self._corridor_region_release_ready(
            pose_xy, goal_xy, yaw, now_ts
        )
        if ready:
            self.corridor_active = False
            self.corridor_nav_mode = 'idle'
            self.corridor_capture_active = False
            self.corridor_align_active = False
            self.corridor_entry_reorient_active = False
            self.corridor_entry_reorient_started_at = None
            self.phase1_motion_state = 'forward'
            self.stop_robot()
            reason = (
                f'region entry OK map({pose_xy[0]:.2f},{pose_xy[1]:.2f})~'
                f'({goal_xy[0]:.2f},{goal_xy[1]:.2f}) rho={rho:.2f}m '
                f'yaw={math.degrees(yaw):.1f}deg err={math.degrees(yaw_error):.1f}deg '
                f'pos_ok={pos_ok} radius_ok={radius_ok} y_gate_ok={y_gate_ok} '
                f'xerr={x_error:.3f}m x_ok={x_ok} min_y={self.corridor_release_min_y:.2f} yaw_ok={yaw_ok} '
                f'wall_ok={wall_ok} commit={self.corridor_terminal_commit_active} '
                f'require_yaw={self.corridor_require_yaw_for_release} '
                f'plans={getattr(self, "_corridor_plan_count", 0)} last_plan={self.corridor_last_plan_reason or "none"}'
            )
            self.log.mission(reason)
            self.begin_phase_transition(2, reason)
            return

        # After backing, side-wall geometry owns all normal corridor steering.
        # Do not let map Pure Pursuit or map-X recovery compete with it.
        if (
            self.corridor_wall_follow_enabled
            and not self.corridor_terminal_active
            and not self.corridor_terminal_reverse_align_active
        ):
            if self._follow_corridor_walls(pose_xy, yaw, now_ts):
                return

        # 条件重规划：进度/偏航/过期才重算，避免每 0.5s 阻塞 1.7s
        need_replan, replan_why = self._should_refresh_corridor_path(
            pose_xy, now_ts, force=not self.corridor_planned_path, reason='empty'
        )
        if need_replan and not (
            self.corridor_terminal_active
            or self.corridor_terminal_reverse_align_active
        ):
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
            # The recovery turn can move the vehicle far beyond the old path
            # origin. Re-anchor the remaining reference before Pure Pursuit
            # resumes so it never steers back toward a passed point.
            if (
                self.corridor_reference_path_enabled
                and path
                and math.hypot(path[0][0] - pose_xy[0], path[0][1] - pose_xy[1]) > 0.12
            ):
                path = self.refresh_corridor_planned_path(
                    pose_xy, goal_xy, reason='left_recover_reference_rejoin'
                )
                self.publish_corridor_path(pose_xy)
            self.corridor_nav_mode = 'left_recover'
            self.cmd_pub.publish(left_cmd)
            if now_ts - self._corridor_last_detail_log_time >= self.corridor_log_period_sec:
                self._corridor_last_detail_log_time = now_ts
                self.log.progress(
                    f'left_recover during region entry ρ={rho:.2f}m '
                    f'map=({pose_xy[0]:.2f},{pose_xy[1]:.2f})'
                )
            return

        if self.corridor_terminal_reverse_align_active:
            terminal_yaw_error = self.angle_error(self.corridor_goal_yaw, yaw)
            if abs(terminal_yaw_error) <= self.corridor_terminal_yaw_tolerance:
                self.corridor_terminal_reverse_align_active = False
                self.corridor_terminal_active = True
                self.corridor_nav_mode = 'terminal_approach'
                self.log.mission(
                    f'terminal reverse align complete map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                    f'yaw={math.degrees(yaw):.1f}deg '
                    f'err={math.degrees(terminal_yaw_error):.1f}deg'
                )
                return

            angular = self.clamp(
                self.terminal_reverse_align_heading_kp * terminal_yaw_error,
                self.terminal_reverse_align_max_angular_speed,
            )
            self.corridor_nav_mode = 'terminal_reverse_align'
            self.corridor_desired_heading = self.corridor_goal_yaw
            self.cmd_pub.publish(self.create_twist(
                self.terminal_reverse_align_linear_speed, angular
            ))
            if now_ts - self._corridor_last_log_time >= self.corridor_log_period_sec:
                self._corridor_last_log_time = now_ts
                elapsed = now_ts - self.corridor_started_at if self.corridor_started_at else 0.0
                self.log.segment(
                    f'terminal_reverse_align map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                    f'xerr={x_error:.3f}m yaw={math.degrees(yaw):.1f}deg '
                    f'desired={math.degrees(self.corridor_goal_yaw):.1f}deg '
                    f'err={math.degrees(terminal_yaw_error):.1f}deg '
                    f'v={self.terminal_reverse_align_linear_speed:.2f} w={angular:.2f} t={elapsed:.1f}s'
                )
            return

        if self.corridor_terminal_active:
            wall_lock = self._terminal_wall_lock_fresh(now_ts)
            wall_control_lock = self._terminal_wall_lock_for_control(now_ts)
            wall_latch = self._terminal_wall_latch_fresh(now_ts)
            wall_available = wall_lock is not None or wall_latch is not None
            wall_control_available = wall_control_lock is not None or wall_latch is not None
            wall_center_error = (
                wall_control_lock['center_error'] if wall_control_lock is not None
                else wall_latch['center_error'] if wall_latch is not None
                else 0.0
            )
            wall_axis_error = (
                self._terminal_wall_heading_axis_error(wall_lock['heading_error'], yaw)
                if wall_lock is not None
                else wall_latch['axis_error'] if wall_latch is not None
                else float('inf')
            )
            if self.corridor_terminal_commit_active:
                commit_latch = self._terminal_wall_latch_fresh(now_ts)
                commit_failed = (
                    (self.corridor_terminal_wall_lock_enabled and commit_latch is None)
                    or abs(self.angle_error(self.corridor_goal_yaw, yaw))
                    > self.corridor_terminal_commit_abort_heading
                )
                if commit_failed:
                    self._start_corridor_reacquire(pose_xy, 'terminal commit rejected')
                    self.corridor_nav_mode = 'terminal_commit_retry'
                    return
                # The final segment is intentionally straight. Once committed,
                # no fresh lateral estimate may reverse the steering direction.
                self.corridor_nav_mode = 'terminal_commit_straight'
                desired_yaw = self.corridor_goal_yaw
                angular_limit = self.corridor_terminal_max_angular_speed
                linear = self.corridor_terminal_commit_speed
                lateral_error = wall_center_error if wall_available else 0.0
            elif wall_control_available:
                # Before commit, steer toward the physical wall centerline.
                # At yaw=90deg, a positive body-Y center error is to the
                # vehicle's left, hence it requires a positive yaw offset.
                # Once centered, terminal commit below locks the fixed IMU
                # exit yaw and keeps the final segment straight.
                lateral_error = wall_center_error
                desired_yaw = self.normalize_angle(
                    self.corridor_goal_yaw + math.atan(
                        self.corridor_terminal_lateral_gain * lateral_error
                    )
                )
                self.corridor_nav_mode = 'terminal_wall_center_correct'
                angular_limit = self.corridor_terminal_max_angular_speed
                linear = self.corridor_terminal_linear_speed
            else:
                lateral_error = self._terminal_lateral_error_for_control(x_error, now_ts)
                micro_start_y = (
                    self.corridor_release_min_y - self.corridor_terminal_micro_start_y_margin
                )
                micro_correct_x = (
                    pose_xy[1] >= micro_start_y
                    and abs(x_error) > self.corridor_terminal_x_tolerance
                )
                if micro_correct_x:
                    self.corridor_nav_mode = 'terminal_micro_correct_fallback'
                    desired_yaw = self.normalize_angle(
                        self.corridor_goal_yaw + math.atan(
                            self.corridor_terminal_lateral_gain * lateral_error
                        )
                    )
                    angular_limit = self.corridor_terminal_micro_max_angular_speed
                else:
                    self.corridor_nav_mode = 'terminal_heading_settle_fallback'
                    desired_yaw = self.corridor_goal_yaw
                    angular_limit = self.corridor_terminal_max_angular_speed
                linear = self.corridor_terminal_linear_speed

            commit_wall_ok = False
            commit_map_ok = False
            commit_imu_ok = False
            commit_y_ok = pose_xy[1] >= (
                self.corridor_release_min_y - self.corridor_terminal_commit_y_margin
            )
            if not self.corridor_terminal_commit_active and commit_y_ok:
                commit_wall_ok = not self.corridor_terminal_wall_lock_enabled
                if wall_available:
                    commit_wall_ok = (
                        abs(wall_center_error)
                        <= self.corridor_terminal_commit_center_tolerance
                        and wall_axis_error <= self.corridor_terminal_wall_axis_tolerance
                    )
                commit_map_ok = (
                    abs(x_error) <= self.corridor_terminal_x_tolerance
                    or not self.corridor_wall_follow_enabled
                    or not self.corridor_wall_follow_require_map_x_handoff
                )
                # Commit starts the final straight segment; it is not the
                # release itself.  Requiring an already-perfect yaw here
                # prevents a well-centered car from ever reaching that
                # straightening segment before the Y overrun guard.
                commit_imu_ok = (
                    abs(self.angle_error(self.corridor_goal_yaw, yaw))
                    <= self.corridor_terminal_commit_heading_tolerance
                )
                if commit_wall_ok and commit_map_ok and commit_imu_ok:
                    self.corridor_terminal_commit_active = True
                    self._terminal_release_ready_since = None
                    self.log.mission(
                        f'terminal commit locked map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                        f'wall_center={wall_center_error if wall_available else 0.0:.3f}m '
                        f'wall_axis={math.degrees(wall_axis_error) if wall_available else 0.0:.1f}deg '
                        f'yaw={math.degrees(yaw):.1f}deg'
                    )
            terminal_yaw_error = self.angle_error(desired_yaw, yaw)
            if abs(terminal_yaw_error) <= self.corridor_terminal_yaw_deadband:
                angular = 0.0
            else:
                angular = self.clamp(
                    self.corridor_terminal_heading_kp * terminal_yaw_error,
                    angular_limit,
                )
            angular = self._limit_terminal_angular_command(angular, now_ts)
            self.corridor_desired_heading = desired_yaw
            self.cmd_pub.publish(self.create_twist(linear, angular))

            if now_ts - self._corridor_last_log_time >= self.corridor_log_period_sec:
                self._corridor_last_log_time = now_ts
                elapsed = now_ts - self.corridor_started_at if self.corridor_started_at else 0.0
                self.log.segment(
                    f'{self.corridor_nav_mode} map=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                    f'xerr={x_error:.3f}m lateral={lateral_error:.3f}m '
                    f'wall={wall_available} '
                    f'wall_heading={math.degrees(wall_control_lock["heading_error"]) if wall_control_lock is not None else 0.0:.1f}deg '
                    f'wall_axis={math.degrees(wall_axis_error) if wall_available else 0.0:.1f}deg '
                    f'wall_latched={wall_control_lock is None and wall_latch is not None} '
                    f'commit={self.corridor_terminal_commit_active} y_gate={y_gate_ok} '
                    f'commit_checks=y:{commit_y_ok} wall:{commit_wall_ok} '
                    f'map:{commit_map_ok} imu:{commit_imu_ok} '
                    f'yaw={math.degrees(yaw):.1f}deg desired={math.degrees(desired_yaw):.1f}deg '
                    f'err={math.degrees(terminal_yaw_error):.1f}deg '
                    f'v={linear:.2f} w={angular:.2f} t={elapsed:.1f}s'
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
        heading_scale = max(0.35, abs(math.cos(alpha)))
        linear = min(speed_cap, max(self.corridor_min_cruise_speed, self.corridor_rho_kp * max(rho, 0.35) * heading_scale))

        if rho < self.corridor_brake_distance:
            brake_cap = max(self.corridor_creep_speed, self.corridor_brake_kp * max(rho, 0.12))
            linear = min(linear, max(self.corridor_creep_speed, brake_cap))
        if abs(alpha) > math.radians(45.0):
            linear = min(linear, max(self.corridor_creep_speed, self.corridor_max_turn_linear_speed))
        if abs(alpha) > math.radians(70.0):
            # 大角时允许略走一点弧，不要趴在 0.04 蠕行
            linear = min(linear, max(self.corridor_creep_speed, 0.06))

        if self.corridor_path_follow_mode == 'stanley':
            v_ref = max(abs(linear), 0.08)
            angular = self.corridor_alpha_kp * alpha + math.atan2(self.corridor_stanley_k * target_y, v_ref)
        else:
            # pure pursuit: curvature = 2*y / ld^2
            curvature = 2.0 * target_y / max(ld * ld, 1e-3)
            angular = self.pure_pursuit_turn_kp * curvature * max(abs(linear), 0.08)
            # 大航向误差时叠加 LOS P 项，避免斜穿时拧不过来
            if abs(alpha) > math.radians(25.0):
                angular += 0.55 * self.corridor_alpha_kp * alpha

        angular = self.clamp(angular, self.max_angular_speed)

        # 在最终 Y 门线前预先收敛 IMU 航向。末段不能再让 Pure Pursuit 的近点几何
        # 转向反向拉走车头；横向误差通过 desired_terminal_yaw 继续闭环修正。
        terminal_prealign_active = (
            final_waypoint
            and self.corridor_terminal_enabled
            and pose_xy[1] >= (
                self.corridor_release_min_y - self.corridor_terminal_prealign_y_margin
            )
            and abs(x_error) <= self.corridor_terminal_x_exit_tolerance
        )
        terminal_prealign_yaw_error = None
        if terminal_prealign_active:
            lateral_x_error = self._terminal_lateral_error_for_control(x_error, now_ts)
            desired_terminal_yaw = self.normalize_angle(
                self.corridor_goal_yaw - math.atan(
                    self.corridor_terminal_lateral_gain * lateral_x_error
                )
            )
            terminal_prealign_yaw_error = self.angle_error(desired_terminal_yaw, yaw)
            prealign_angular = self.clamp(
                self.corridor_terminal_prealign_heading_kp * terminal_prealign_yaw_error,
                self.corridor_terminal_prealign_max_angular_speed,
            )
            angular = self._limit_terminal_angular_command(prealign_angular, now_ts)
            self.corridor_desired_heading = desired_terminal_yaw

        if abs(angular) > 0.25:
            linear = min(linear, max(self.corridor_creep_speed, self.corridor_max_turn_linear_speed))
        elif abs(angular) > 0.12:
            linear = min(linear, max(self.corridor_min_cruise_speed * 0.8, 0.08))

        # 极近区域只做缓慢收敛，不再 align_yaw
        if rho < max(0.22, self.corridor_entry_region_radius * 0.75):
            linear = min(linear, max(self.corridor_creep_speed, 0.08))

        self.cmd_pub.publish(self.create_twist(linear, angular))

        if now_ts - self._corridor_last_log_time >= self.corridor_log_period_sec:
            self._corridor_last_log_time = now_ts
            elapsed = now_ts - self.corridor_started_at if self.corridor_started_at else 0.0
            cursor = self.corridor_path_cursor
            offpath = self._path_cross_track_m(path, pose_xy)
            prealign_error_text = (
                f'{math.degrees(terminal_prealign_yaw_error):.1f}deg'
                if terminal_prealign_yaw_error is not None else 'n/a'
            )
            self.log.segment(
                f'region_entry map=({pose_xy[0]:.2f},{pose_xy[1]:.2f})->'
                f'({goal_xy[0]:.2f},{goal_xy[1]:.2f}) rho={rho:.2f}m '
                f'yaw={math.degrees(yaw):.1f}deg yerr={math.degrees(yaw_error):.1f}deg '
                f'alpha={math.degrees(alpha):.1f}deg look=({look_pt[0]:.2f},{look_pt[1]:.2f}) '
                f'body=({target_x:.2f},{target_y:.2f}) ld={ld:.2f} offpath={offpath:.2f}m '
                f'v={linear:.2f} w={angular:.2f} mode={self.corridor_nav_mode} '
                f'plan={self.corridor_last_plan_reason} pts={len(path)} cursor={cursor} '
                f'plans={getattr(self, "_corridor_plan_count", 0)} '
                f'pos_ok={pos_ok} radius_ok={radius_ok} y_gate_ok={y_gate_ok} '
                f'xerr={x_error:.3f}m x_ok={x_ok} terminal={self.corridor_terminal_active} '
                f'prealign={terminal_prealign_active} prealign_err={prealign_error_text} '
                f'min_y={self.corridor_release_min_y:.2f} yaw_ok={yaw_ok} t={elapsed:.1f}s'
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

    def choose_avoid_turn_direction(self, danger_angle, obstacle=None):
        """在通道导航中结合当前约定路点选绕行侧；局部障碍侧始终优先。"""
        fallback = (
            -1.0
            if math.radians(danger_angle) >= self.avoid_right_turn_left_obstacle_angle_rad
            else 1.0
        )
        if self.blind_scan_is_active() and obstacle is not None:
            corridor_extra = (
                self.blind_scan_escape_corridor_extra
                if self.blind_scan_escape_pending else 0.0
            )
            candidates = [
                self.predict_blind_scan_avoidance(direction, obstacle, corridor_extra)
                for direction in (-1.0, 1.0)
            ]
            candidates = [candidate for candidate in candidates if candidate is not None]
            feasible = [
                candidate for candidate in candidates
                if candidate['corridor_ok'] and candidate['obstacle_ok']
            ]
            if feasible:
                selected = min(feasible, key=lambda candidate: candidate['score'])
                detail = 'scan_prediction ' + ' '.join(
                    f"{'L' if candidate['direction'] > 0.0 else 'R'}="
                    f"safe={candidate['obstacle_ok']} lane={candidate['corridor_ok']} "
                    f"dmax={candidate['max_lateral']:.2f} "
                    f"dend={candidate['final_lateral']:.2f} "
                    f"clear={candidate['min_obstacle_distance']:.2f}"
                    for candidate in candidates
                )
                return selected['direction'], detail

            detail = 'scan_prediction_blocked ' + ' '.join(
                f"{'L' if candidate['direction'] > 0.0 else 'R'}="
                f"safe={candidate['obstacle_ok']} lane={candidate['corridor_ok']} "
                f"dmax={candidate['max_lateral']:.2f} "
                f"clear={candidate['min_obstacle_distance']:.2f}"
                for candidate in candidates
            )
            return 0.0, detail

        if (
            not self.corridor_avoid_goal_bias_enabled
            or not self.corridor_active
            or self.current_yaw is None
        ):
            return fallback, 'local_fallback'

        map_xy = self.get_map_position()
        goal_xy = self.corridor_goal_point()
        if map_xy is None or goal_xy is None:
            return fallback, 'local_no_map_goal'

        target_bearing = math.atan2(goal_xy[1] - map_xy[1], goal_xy[0] - map_xy[0])
        danger_angle_rad = math.radians(danger_angle)
        scores = {}
        for direction in (-1.0, 1.0):
            # 朝候选绕行侧转过最小避障角后，车头与当前路点连线的偏差。
            turned_yaw = self.normalize_angle(
                self.current_yaw + direction * self.avoid_min_turn_angle_rad
            )
            score = abs(self.angle_error(target_bearing, turned_yaw))

            # 明确侧向的近障是硬安全偏好，不能因为路点在同侧而朝障碍切入。
            if (
                abs(danger_angle_rad) >= self.avoid_right_turn_left_obstacle_angle_rad
                and math.copysign(1.0, danger_angle_rad) == direction
            ):
                score += self.corridor_avoid_obstacle_side_penalty
            scores[direction] = score

        direction = min(scores, key=scores.get)
        relative_goal = math.degrees(self.angle_error(target_bearing, self.current_yaw))
        detail = (
            f'goal_bias target=({goal_xy[0]:.2f},{goal_xy[1]:.2f}) '
            f'goal_angle={relative_goal:.1f}deg '
            f'score_right={scores[-1.0]:.2f} score_left={scores[1.0]:.2f}'
        )
        return direction, detail

    def begin_avoidance(self, danger_angle, obstacle=None):
        if self.phase1_motion_state == 'corridor' or self.corridor_active:
            self.corridor_resume_after_avoidance = True
            if self.corridor_reverse_avoid_enabled:
                self.begin_corridor_reverse_avoidance(danger_angle, obstacle)
                return
        # ROS base frame: +Y / +angle is left, angular.z < 0 is right.
        self.avoid_turn_direction, selection_detail = self.choose_avoid_turn_direction(
            danger_angle, obstacle=obstacle
        )
        if self.avoid_turn_direction == 0.0:
            if (
                self.blind_scan_is_active()
                and obstacle is not None
                and not self.blind_scan_escape_attempted
            ):
                self.begin_blind_scan_escape(danger_angle, obstacle, selection_detail)
                return
            self.phase1_motion_state = 'scan_blocked'
            self.avoid_cmd = Twist()
            self.log.mission(
                f'AVOID BLOCKED: danger_angle={danger_angle:.0f}° {selection_detail}'
            )
            return

        self.blind_scan_escape_pending = False

        self.phase1_motion_state = 'avoiding'
        self.avoid_started_time = self.get_clock().now()
        self.avoid_clear_since = None
        self.avoid_entry_yaw = self.current_yaw
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.recovery_uses_heading = False

        if self.corridor_active or self.corridor_resume_after_avoidance:
            # 通道避障后 recovery 必须回到通道航向，而不是 phase1 盲开锁向
            self.update_corridor_desired_heading()
        elif self.blind_scan_guidance_is_active():
            pose_xy = self.get_map_position()
            if pose_xy is not None:
                target_heading, _ = self.blind_scan_heading(pose_xy)
                if target_heading is not None:
                    self.desired_heading = target_heading
        elif self.desired_heading is None and self.current_yaw is not None:
            self.desired_heading = self.current_yaw

        turn_name = 'RIGHT' if self.avoid_turn_direction < 0.0 else 'LEFT'
        self.log.mission(
            f'AVOID {turn_name}: dir={self.avoid_turn_direction:.0f} '
            f'danger_angle={danger_angle:.0f}° '
            f'{selection_detail} '
            f'desired_yaw={(math.degrees(self.desired_heading) if self.desired_heading is not None else float("nan")):.1f}°'
        )

    def begin_blind_scan_escape(self, danger_angle, obstacle, selection_detail):
        """Create space before retrying a scan-lane-constrained forward detour."""
        # With negative linear velocity, a yaw command toward the obstacle's
        # side moves the vehicle laterally away from that obstacle.
        self.blind_scan_escape_direction = 1.0 if danger_angle >= 0.0 else -1.0
        now = self.get_clock().now()
        self.phase1_motion_state = 'scan_escape_reversing'
        self.blind_scan_escape_attempted = True
        self.blind_scan_escape_pending = True
        self.blind_scan_escape_deadline = now + Duration(
            seconds=self.blind_scan_escape_reverse_duration
        )
        self.avoid_cmd = self.create_twist(
            self.blind_scan_escape_reverse_linear_speed,
            self.blind_scan_escape_direction * self.blind_scan_escape_reverse_angular_speed,
        )
        turn_name = 'LEFT' if self.blind_scan_escape_direction > 0.0 else 'RIGHT'
        self.log.mission(
            f'SCAN_ESCAPE_REVERSE start obstacle=d={obstacle["distance"]:.2f}m '
            f'angle={danger_angle:.1f}deg reverse={self.blind_scan_escape_reverse_linear_speed:.2f}mps '
            f'turn={turn_name} duration={self.blind_scan_escape_reverse_duration:.2f}s '
            f'extra_lane={self.blind_scan_escape_corridor_extra:.2f}m {selection_detail}'
        )

    def begin_corridor_reverse_avoidance(self, danger_angle, obstacle):
        """Back away from a corridor obstacle without sacrificing wall lock."""
        # The front of the vehicle swings away from the obstacle: obstacle on
        # the right means a left yaw command, and vice versa.
        self.corridor_reverse_direction = 1.0 if danger_angle <= 0.0 else -1.0
        self.phase1_motion_state = 'corridor_reverse_avoid'
        self.corridor_reverse_started_time = self.get_clock().now()
        self.corridor_reverse_clear_since = None
        self.corridor_reverse_last_obstacle = obstacle
        self.corridor_reverse_return_logged = False
        self.avoid_cmd = self.create_twist(
            self.corridor_reverse_avoid_linear_speed,
            self.corridor_reverse_direction * self.corridor_reverse_avoid_angular_speed,
        )
        wall_lock = self._terminal_wall_lock_fresh(
            self.corridor_reverse_started_time.nanoseconds / 1e9
        )
        wall_detail = (
            f'wall_width={wall_lock["width"]:.2f}m '
            f'wall_center={wall_lock["center_error"]:.2f}m'
            if wall_lock is not None else 'wall=single_or_wait'
        )
        turn_name = 'LEFT' if self.corridor_reverse_direction > 0.0 else 'RIGHT'
        self.log.mission(
            f'CORRIDOR_REVERSE_AVOID start obstacle='
            f'd={obstacle["distance"]:.2f}m span={obstacle["span"]:.2f}m '
            f'pts={obstacle["point_count"]} angle={danger_angle:.1f}deg '
            f'reverse={self.corridor_reverse_avoid_linear_speed:.2f}mps '
            f'turn={turn_name} w={self.corridor_reverse_direction * self.corridor_reverse_avoid_angular_speed:.2f} '
            f'{wall_detail}'
        )

    def _corridor_reverse_entry_reached(self):
        if self.corridor_entry_pose is None:
            return True
        pose_xy = self.get_map_position()
        if pose_xy is None:
            return True
        return math.hypot(
            pose_xy[0] - self.corridor_entry_pose[0],
            pose_xy[1] - self.corridor_entry_pose[1],
        ) <= self.corridor_reverse_avoid_entry_tolerance

    def _corridor_reverse_return_cmd(self, now_ts):
        """Reverse along the observed boundary back toward the corridor mouth."""
        wall_lock = self._terminal_wall_lock_fresh(now_ts)
        if wall_lock is not None:
            # Reverse motion inverts the lateral effect of steering.  The
            # heading term remains unchanged, while the center correction is
            # the opposite of forward wall following.
            relative_target_error = wall_lock['heading_error'] - math.atan(
                self.corridor_wall_follow_lateral_gain * wall_lock['center_error']
            )
        else:
            single_wall = self._corridor_single_wall_lock
            if (
                single_wall is not None
                and now_ts - single_wall['stamp'] <= self.corridor_terminal_wall_max_age
            ):
                relative_target_error = single_wall['heading_error']
            else:
                relative_target_error = 0.0
        angular = self.clamp(
            self.corridor_wall_follow_heading_kp * relative_target_error,
            self.corridor_reverse_avoid_angular_speed,
        )
        return self.create_twist(self.corridor_reverse_avoid_linear_speed, angular)

    def finish_corridor_reverse_avoidance(self, reason):
        self.phase1_motion_state = 'corridor'
        self.corridor_resume_after_avoidance = False
        self.corridor_reverse_started_time = None
        self.corridor_reverse_clear_since = None
        self.corridor_reverse_direction = 0.0
        self.corridor_reverse_last_obstacle = None
        self.corridor_reverse_return_logged = False
        self.avoid_cmd = Twist()
        self.log.feedback(f'CORRIDOR_REVERSE_AVOID complete: {reason}; resume wall follow')

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
        elif self.blind_scan_guidance_is_active():
            pose_xy = self.get_map_position()
            if pose_xy is not None:
                target_heading, _ = self.blind_scan_heading(pose_xy)
                if target_heading is not None:
                    self.desired_heading = target_heading
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
            self.corridor_resume_after_avoidance = False
            map_xy = self.get_map_position()
            if map_xy is not None and self.corridor_waypoints:
                goal_xy = (
                    float(self.corridor_waypoints[self.corridor_index]['x']),
                    float(self.corridor_waypoints[self.corridor_index]['y']),
                )
                # An obstacle at the mouth can leave a large lateral error
                # with too little Y distance left to recover safely. Return
                # to the staging line before trying the terminal again.
                if (
                    self.corridor_on_final_waypoint()
                    and map_xy[1] >= self.corridor_reacquire_y
                ):
                    self._start_corridor_reacquire(map_xy, 'post-avoid terminal reset')
                    self.corridor_nav_mode = 'post_avoid_y_reacquire'
                    self.log.feedback('recovery complete, return to terminal staging')
                    self.avoid_started_time = None
                    self.avoid_clear_since = None
                    self.avoid_entry_yaw = None
                    self.last_avoid_duration = 0.0
                    self.counter_steer_deadline = None
                    self.recovery_deadline = None
                    self.recovery_uses_heading = False
                    return
                if self.corridor_reference_path_enabled:
                    self.refresh_corridor_planned_path(
                        map_xy, goal_xy, reason='post_avoid_reference_rejoin'
                    )
                    self.publish_corridor_path(map_xy)
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
            'point_count': len(cluster),
            'danger_angle_deg': danger_angle_deg,
            'center_x': center_x,
            'center_y': center_y,
        }

    def find_phase1_forward_obstacle(self, scan_msg, min_cluster_width=None):
        max_x = (
            self.blind_scan_avoid_detection_max_x
            if self.blind_scan_is_active()
            else self.phase1_window_max_x
        )
        clusters = self.collect_points_in_window(
            scan_msg,
            self.phase1_window_min_x,
            max_x,
            self.phase1_window_half_width,
        )

        # 发布所有聚类的可视化（rviz2 调试用）
        if clusters:
            self.obstacle_markers.publish_from_clusters(clusters, color='red')
            self._phase1_last_clusters = clusters
        else:
            self.obstacle_markers.clear()
            self._phase1_last_clusters = []

        min_width = (
            self.phase1_min_cluster_width
            if min_cluster_width is None else max(0.0, float(min_cluster_width))
        )
        nearest_obstacle = None
        for cluster in clusters:
            if len(cluster) < self.phase1_min_cluster_points:
                continue

            obstacle = self.describe_cluster(cluster)
            if obstacle['span'] < min_width:
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

    def find_blind_scan_escape_rear_obstacle(self, scan_msg):
        clusters = self.collect_points_in_window(
            scan_msg,
            self.blind_scan_escape_rear_min_x,
            self.blind_scan_escape_rear_max_x,
            self.blind_scan_escape_rear_half_width,
        )
        for cluster in clusters:
            if len(cluster) >= self.phase1_emergency_min_points:
                return self.describe_cluster(cluster)
        return None

    def handle_phase1_lidar(self, scan_msg):
        # 后退阶段完全不处理激光避障，由后退逻辑全权控制
        if self.phase1_motion_state == 'backing':
            return

        if self.phase1_motion_state == 'scan_escape_reversing':
            rear_obstacle = self.find_blind_scan_escape_rear_obstacle(scan_msg)
            if rear_obstacle is not None:
                self.phase1_motion_state = 'scan_blocked'
                self.avoid_cmd = Twist()
                self.log.mission(
                    f'SCAN_ESCAPE_REVERSE BLOCKED rear=d={rear_obstacle["distance"]:.2f}m '
                    f'angle={rear_obstacle["danger_angle_deg"]:.1f}deg'
                )
                return

            if (
                self.blind_scan_escape_deadline is not None
                and self.get_clock().now() >= self.blind_scan_escape_deadline
            ):
                self.phase1_motion_state = 'forward'
                self.blind_scan_escape_deadline = None
                self.avoid_cmd = Twist()
                self.log.feedback('SCAN_ESCAPE_REVERSE complete; retry forward avoidance')
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
        if self.phase1_motion_state == 'corridor_reverse_avoid':
            obstacle = self.find_phase1_forward_obstacle(
                scan_msg, self.corridor_obstacle_min_width
            )
            now = self.get_clock().now()
            if obstacle is not None:
                self.obstacle_found = True
                self.closest_obstacle_distance = obstacle['distance']
                self.corridor_reverse_last_obstacle = obstacle
                self.corridor_reverse_clear_since = None
                self.corridor_reverse_return_logged = False
                return

            self.obstacle_found = False
            self.closest_obstacle_distance = float('inf')
            if self.corridor_reverse_clear_since is None:
                self.corridor_reverse_clear_since = now
            clear_elapsed = (now - self.corridor_reverse_clear_since).nanoseconds / 1e9
            reverse_elapsed = 0.0
            if self.corridor_reverse_started_time is not None:
                reverse_elapsed = (now - self.corridor_reverse_started_time).nanoseconds / 1e9
            now_ts = now.nanoseconds / 1e9
            entry_reached = self._corridor_reverse_entry_reached()
            if not entry_reached:
                self.avoid_cmd = self._corridor_reverse_return_cmd(now_ts)
                if not self.corridor_reverse_return_logged:
                    pose_xy = self.get_map_position()
                    self.corridor_reverse_return_logged = True
                    self.log.mission(
                        f'CORRIDOR_REVERSE_AVOID obstacle clear; return corridor entry '
                        f'entry=({self.corridor_entry_pose[0]:.2f},{self.corridor_entry_pose[1]:.2f}) '
                        f'current=({pose_xy[0]:.2f},{pose_xy[1]:.2f}) '
                        f'tol={self.corridor_reverse_avoid_entry_tolerance:.2f}m'
                    )
                return
            if reverse_elapsed >= self.corridor_reverse_avoid_min_duration and clear_elapsed >= self.corridor_reverse_avoid_clear_hold:
                self.finish_corridor_reverse_avoidance(
                    f'return_entry reverse={reverse_elapsed:.2f}s clear={clear_elapsed:.2f}s'
                )
            return

        if self.phase1_motion_state == 'avoiding':
            obstacle = self.find_phase1_emergency_obstacle(scan_msg)
        else:
            obstacle = self.find_phase1_forward_obstacle(
                scan_msg,
                self.corridor_obstacle_min_width
                if self.corridor_active or self.phase1_motion_state == 'corridor'
                else None,
            )

        if obstacle is not None:
            self.obstacle_found = True
            self.closest_obstacle_distance = obstacle['distance']

            if self.phase1_motion_state != 'avoiding':
                self.begin_avoidance(obstacle['danger_angle_deg'], obstacle=obstacle)
            else:
                self.avoid_clear_since = None

            if self.phase1_motion_state == 'scan_blocked':
                self.avoid_cmd = Twist()
                return

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

        if not self.blind_scan_escape_pending:
            self.blind_scan_escape_attempted = False

        if self.phase1_motion_state == 'scan_blocked':
            self.phase1_motion_state = 'forward'
            self.blind_scan_escape_attempted = False
            self.blind_scan_escape_pending = False
            self.avoid_cmd = Twist()
            self.log.feedback('scan avoidance obstruction cleared, resume centerline')
            return
        
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
            # Side-wall fitting is independent from the forward obstacle detector.
            # It never commands the chassis directly; the 20Hz terminal loop
            # consumes only fresh, quality-checked geometry.
            self._update_terminal_wall_lock(msg)
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
        if self.phase1_motion_state not in (
            'forward', 'avoiding', 'scan_escape_reversing', 'countersteering', 'recovering', 'corridor'
        ):
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
            # 后退期间禁用 YOLO 推理，避免模型占用 BPU
            if hasattr(self, '_enable_vision_corridor'):
                try:
                    self._enable_vision_corridor(False)
                except Exception as e:
                    self.log.warn('VISION', f'关闭 YOLO 推理失败: {e}')
            self.phase1_motion_state = 'backing'
            self.backing_started_time = self.get_clock().now()
            self.backing_last_angular_z = 0.0
            self.backing_last_command_time = self.backing_started_time
            # 最后一个采样点通常就在扫码位置；从它的前一个点开始，
            # 避免倒车控制先追逐当前位置而错过真正的回程轨迹。
            self.backing_path_index = max(0, len(self.path_record) - 2)
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
        if self.phase != 2:
            return

        if self.stage2_state == 'complete':
            if not self.stage2_run_observed:
                self.log.warn(
                    'PHASE',
                    'ignored Stage2 complete before this phase2 run reported an active state',
                )
                return
            self.log.mission('stage2 complete, entering phase3')
            self.begin_phase_transition(3, 'stage2 complete, switched to phase3 return-to-p')
            return

        if self.stage2_state not in ('', 'idle', 'failed'):
            if not self.stage2_run_observed:
                self.stage2_run_observed = True
                self.log.mission(
                    f'Stage2 active state confirmed for this run: {self.stage2_state}'
                )

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

    def stage3_cmd_callback(self, msg):
        self.latest_stage3_cmd = msg
        self.latest_stage3_cmd_time = self.get_clock().now()

    def stage2_cmd_is_fresh(self):
        if self.latest_stage2_cmd_time is None:
            return False

        age = self.get_clock().now() - self.latest_stage2_cmd_time
        return age.nanoseconds <= int(self.stage2_cmd_timeout * 1e9)

    def stage3_cmd_is_fresh(self):
        if self.latest_stage3_cmd_time is None:
            return False
        age = self.get_clock().now() - self.latest_stage3_cmd_time
        return age.nanoseconds <= int(self.stage3_cmd_timeout * 1e9)

    def blind_forward_speed(self):
        if self.current_odom is not None:
            odom_x = float(self.current_odom.pose.pose.position.x)
            if odom_x > self.blind_qr_slowdown_start_x_m:
                return self.blind_qr_slowdown_linear_speed
        return self.blind_linear_speed

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

            if self.phase1_motion_state == 'scan_escape_reversing':
                self.cmd_pub.publish(self.avoid_cmd)
                return

            if self.phase1_motion_state == 'corridor_reverse_avoid':
                self.cmd_pub.publish(self.avoid_cmd)
                return

            if self.phase1_motion_state == 'scan_blocked':
                self.stop_robot()
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
                    scan_cmd = self.blind_scan_guidance_cmd()
                    self.cmd_pub.publish(
                        scan_cmd
                        if scan_cmd is not None
                        else self.create_twist(self.blind_forward_speed(), self.blind_angular_speed)
                    )
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
                if (not self.corridor_reference_path_enabled and
                    hasattr(self, '_vision_corridor_enabled') and
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

            scan_cmd = self.blind_scan_guidance_cmd()
            if scan_cmd is not None:
                self.cmd_pub.publish(scan_cmd)
                return

            # 未配置扫描中心线时，保留旧版盲开区域转向作为兼容回退。
            right_cmd = self.maybe_blind_right_turn_cmd()
            if right_cmd is not None:
                self.cmd_pub.publish(right_cmd)
                return

            # 盲开阶段 map_x 过大：向左旋回，避免贴右墙
            left_cmd = self.maybe_left_recover_cmd('blind_left_recover')
            if left_cmd is not None:
                self.cmd_pub.publish(left_cmd)
                return

            self.cmd_pub.publish(
                self.create_twist(self.blind_forward_speed(), self.blind_angular_speed)
            )
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
            # Phase3 near-obstacle recovery is owned by Stage3.  Do not
            # overwrite its bounded reverse command with a stale hard stop.
            if self.stage3_cmd_is_fresh():
                self.cmd_pub.publish(self.latest_stage3_cmd)
                return

            if self.stage2_cmd_is_fresh():
                self.cmd_pub.publish(self.latest_stage2_cmd)
                return

            if self.phase3_external_control:
                self.stop_robot()
                return

            self.stop_robot()
            return

        if self.stage2_cmd_is_fresh():
            self.cmd_pub.publish(self.latest_stage2_cmd)
            return

        self.stop_robot()


    def handle_backing(self):
        """处理后退逻辑：沿记录路径反向跟踪"""
        # 后退期间确保 YOLO 推理已禁用，释放 BPU 资源
        if hasattr(self, '_enable_vision_corridor'):
            try:
                self._enable_vision_corridor(False)
            except Exception:
                pass

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
        
        # 路径跟踪用 odom（与 path_record 同系）；结束判定用 map（与 back_target_x 同系）
        odom_x = float(self.current_odom.pose.pose.position.x)
        odom_y = float(self.current_odom.pose.pose.position.y)
        map_xy = self.get_map_position()
        map_x = float(map_xy[0]) if map_xy is not None else None
        map_y = float(map_xy[1]) if map_xy is not None else None
        
        # back_target_x 是 map 坐标；禁止再用 odom_x 比较
        if map_x is not None and map_x <= self.back_target_x:
            self.log.segment(
                f'backing done at map_x={map_x:.2f}m '
                f'(target={self.back_target_x:.2f}m, odom_x={odom_x:.2f}m), '
                f'starting corridor navigation'
            )
            self.start_corridor_navigation(f'qr task={self.qr_task}, backing complete')
            return
        
        # 检查路径是否倒序遍历完毕
        if self.backing_path_index < 0 or self.backing_path_index >= len(self.path_record):
            self.log.warn('BACKING', 'path exhausted, starting corridor navigation')
            self.start_corridor_navigation(f'qr task={self.qr_task}, backing path exhausted')
            return
        
        # 先消化已经到达或已经越过的倒序路点。仅按圆形到点容差会在高速
        # 倒车时漏过采样点：车已跨过路点后距离又变大，旧前瞻点随即落到
        # 车头前方，导致反向航向目标翻转。投影判定保证索引只沿倒序路径
        # 单调推进，不依赖恰好命中路点附近的小圆。
        while self.backing_path_index >= 0:
            waypoint_x, waypoint_y, _ = self.path_record[self.backing_path_index]
            reached_waypoint = (
                math.hypot(odom_x - waypoint_x, odom_y - waypoint_y)
                < self.back_position_tolerance
            )
            passed_waypoint = False
            if self.backing_path_index > 0:
                previous_x, previous_y, _ = self.path_record[self.backing_path_index - 1]
                segment_x = previous_x - waypoint_x
                segment_y = previous_y - waypoint_y
                segment_length_sq = segment_x * segment_x + segment_y * segment_y
                if segment_length_sq > 1e-8:
                    # >= 0 表示当前位置已越过该路点所在的法线平面，进入了
                    # 倒序路径的下一段；无需再回头追逐这个旧点。
                    passed_waypoint = (
                        (odom_x - waypoint_x) * segment_x
                        + (odom_y - waypoint_y) * segment_y
                    ) >= 0.0
            if not reached_waypoint and not passed_waypoint:
                break
            self.backing_path_index -= 1

        if self.backing_path_index < 0:
            self.log.progress('backing reached start, starting corridor navigation')
            self.start_corridor_navigation(f'qr task={self.qr_task}, backing reached start')
            return

        # 倒序累积前瞻距离，目标点始终位于车辆已经走过的来时轨迹上。
        # 路点保存的 yaw 只作历史记录；几何回放不能用它直接锁车头，
        # 否则 IMU 漂移会在倒车时被放大为持续转弯。
        target_index = self.backing_path_index
        target_x, target_y, _ = self.path_record[target_index]
        accumulated = 0.0
        while target_index > 0 and accumulated < self.back_lookahead_m:
            next_x, next_y, _ = self.path_record[target_index - 1]
            accumulated += math.hypot(next_x - target_x, next_y - target_y)
            target_index -= 1
            target_x, target_y, _ = self.path_record[target_index]

        dist_to_target = math.hypot(odom_x - target_x, odom_y - target_y)
        travel_yaw = math.atan2(target_y - odom_y, target_x - odom_x)
        # linear.x 为负，车尾才是实际行进方向，因此车头期望朝向与轨迹反向。
        target_yaw = self.normalize_angle(travel_yaw + math.pi)
        heading_error = self.angle_error(target_yaw, self.current_yaw)
        
        requested_angular_z = self.clamp(
            self.back_angular_kp * heading_error,
            self.back_max_angular_speed,
        )
        now = self.get_clock().now()
        if self.backing_last_command_time is not None:
            elapsed = max(
                0.0,
                (now - self.backing_last_command_time).nanoseconds / 1e9,
            )
            max_delta = self.back_angular_slew_rate * elapsed
            angular_z = max(
                self.backing_last_angular_z - max_delta,
                min(self.backing_last_angular_z + max_delta, requested_angular_z),
            )
        else:
            angular_z = requested_angular_z
        self.backing_last_angular_z = angular_z
        self.backing_last_command_time = now

        # 倒车始终沿记录轨迹后退，航向误差仅通过角速度闭环修正。
        linear_x = self.back_linear_speed
        self.cmd_pub.publish(self.create_twist(linear_x, angular_z))
        
        self.log.progress(
            f'backing: wp={self.backing_path_index}, lookahead_wp={target_index}, '
            f'map_x={map_x if map_x is not None else float("nan"):.2f}m, '
            f'odom_x={odom_x:.2f}m, '
            f'dist={dist_to_target:.2f}m, '
            f'target_yaw={math.degrees(target_yaw):.1f}°, '
            f'yaw_error={math.degrees(heading_error):.1f}°, '
            f'mode=reverse, '
            f'cmd=({linear_x:.2f},{angular_z:.2f})'
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
        if hasattr(self, '_shutdown_vision_corridor'):
            self._shutdown_vision_corridor('node_shutdown')
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

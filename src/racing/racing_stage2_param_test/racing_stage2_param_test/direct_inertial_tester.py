"""Main controller for the standalone stage2 inertial parameter tester.

Navigation: world odom + waypoint targets (see docs/NAVIGATION.md, world_plan_nav.py).

Submodules:
- `direct_inertial_tester_navigation.py`: MISSION vs AVOID handoff.
- `direct_inertial_tester_avoidance.py`: goal_direct waypoint detour.
- `direct_inertial_tester_obstacle.py`: lidar perception, markers, trigger logging.
"""

import math
import os

import rclpy
from visualization_msgs.msg import MarkerArray

from racing_stage2.stage2_inertial_navigator import Stage2InertialNavigator

from .direct_inertial_tester_avoidance import DirectInertialTesterAvoidanceMixin
from .direct_inertial_tester_cmd_safety import DirectInertialTesterCmdSafetyMixin
from .direct_inertial_tester_debug_log import DirectInertialTesterDebugLogMixin
from .direct_inertial_tester_navigation import DirectInertialTesterNavigationMixin
from .direct_inertial_tester_obstacle import DirectInertialTesterObstacleMixin
from .world_plan_nav import DirectInertialTesterWorldPlanMixin
from .ring_track import RING_CHANNEL_ENTRY_YAW_RAD, nominal_mission_finish_pose, segment_endpoints_world
from . import world_segment

# Run E: 末段名义终点与世界距离一致；勿再用 0.40m 误 finish（见 STAGE2_GOAL_DIRECT_FIX_LOG §8）
FINISH_WORLD_DIST_M = 0.15
FINISH_SKIP_TRIM_LAT_M = 0.10
FINISH_SKIP_TRIM_HEAD_RAD = math.radians(10.0)
from .test_log_paths import debug_log_path as default_debug_log_path
from .vehicle_param_sync import sync_stage2_runtime_parameters, sync_tester_runtime_parameters


class DirectInertialTester(
    DirectInertialTesterNavigationMixin,
    DirectInertialTesterAvoidanceMixin,
    DirectInertialTesterWorldPlanMixin,
    DirectInertialTesterObstacleMixin,
    DirectInertialTesterDebugLogMixin,
    DirectInertialTesterCmdSafetyMixin,
    Stage2InertialNavigator,
):
    """Standalone parameter-tuning node for the stage2 inertial loop."""

    def __init__(self):
        super().__init__()
        self.declare_parameter(
            'debug_log_path',
            os.path.join(os.getcwd(), 'log', 'direct_inertial_tester_debug.log'),
        )
        self.declare_parameter('detour_debug_log_period_sec', 0.50)

        self.debug_log_path = (
            str(self.get_parameter('debug_log_path').value).strip() or default_debug_log_path()
        )
        self.reset_debug_log()

        # Test-mission parameters.
        self.declare_parameter('test_direction', 'clockwise')
        self.declare_parameter('test_start_mode', 'auto')
        self.declare_parameter('test_feedback_prefix', '惯导参数测试')
        self.declare_parameter('rectangle_first_leg_m', 1.20)
        self.declare_parameter('rectangle_side_leg_m', 0.60)
        self.declare_parameter('rectangle_top_leg_m', 2.80)

        # Obstacle-circle perception parameters.
        self.declare_parameter('obstacle_circle_topic', 'detected_obstacle_circles')
        self.declare_parameter('obstacle_circle_cluster_distance_threshold', 0.18)
        self.declare_parameter('obstacle_circle_min_cluster_points', 3)
        self.declare_parameter('obstacle_circle_min_range_m', 0.10)
        self.declare_parameter('obstacle_circle_max_range_m', 2.50)
        self.declare_parameter('obstacle_circle_padding_m', 0.04)
        self.declare_parameter('obstacle_circle_min_radius_m', 0.10)
        self.declare_parameter('obstacle_circle_max_radius_m', 0.50)
        self.declare_parameter('obstacle_circle_max_cluster_span_m', 0.90)
        self.declare_parameter('obstacle_circle_marker_height_m', 0.05)
        self.declare_parameter('obstacle_circle_path_half_width_m', 0.18)
        self.declare_parameter('obstacle_corridor_body_half_width_m', 0.12)
        self.declare_parameter('obstacle_side_fence_center_y_m', 0.30)
        self.declare_parameter('obstacle_opposite_wall_min_center_x_m', 0.85)
        self.declare_parameter('detour_turn_max_trigger_distance_m', 0.55)
        self.declare_parameter('obstacle_opposite_front_min_distance_m', 0.70)
        self.declare_parameter('obstacle_circle_planning_margin_m', 0.03)
        self.declare_parameter('obstacle_circle_forward_margin_m', 0.12)
        # Early detect starts replan; clear distance (hysteresis) allows rejoin to main route.
        self.declare_parameter('detour_obstacle_detect_distance', 1.00)
        self.declare_parameter('detour_obstacle_clear_distance', 0.65)
        self.declare_parameter('detour_follow_min_linear_m', 0.14)

        # goal_direct avoidance (move segments only).
        self.declare_parameter('avoid_watch_distance_m', 0.45)
        self.declare_parameter('avoid_commit_distance_m', 0.30)
        self.declare_parameter('avoid_bias_yaw_deg', 20.0)
        self.declare_parameter('avoid_bias_yaw_max_deg', 24.0)
        self.declare_parameter('avoid_pass_clearance_m', 0.10)
        self.declare_parameter('avoid_bypass_max_lateral_m', 0.34)
        self.declare_parameter('avoid_parallel_front_margin_m', 0.18)
        self.declare_parameter('avoid_rejoin_heading_tol_deg', 6.0)
        self.declare_parameter('avoid_rejoin_lateral_tol_m', 0.06)
        self.declare_parameter('avoid_corner_zone_before_m', 0.55)
        self.declare_parameter('avoid_corner_zone_after_m', 0.45)
        self.declare_parameter('avoid_corner_apex_box_m', 0.35)
        self.declare_parameter('avoid_corner_prefer_inside', True)
        self.declare_parameter('avoid_corner_outside_margin_m', 0.12)
        self.declare_parameter('avoid_speed_out_mps', 0.08)
        self.declare_parameter('avoid_speed_pass_mps', 0.10)
        self.declare_parameter('avoid_speed_rejoin_mps', 0.08)
        self.declare_parameter('avoid_corner_speed_mps', 0.09)
        self.declare_parameter('avoid_corner_speed_slow_mps', 0.07)
        self.declare_parameter('avoid_max_angular_speed', 0.42)
        self.declare_parameter('avoid_perception_loss_hold_sec', 0.40)
        self.declare_parameter('avoid_approach_creep_speed_mps', 0.10)
        self.declare_parameter('avoid_approach_speed_ratio', 0.35)
        # goal_direct avoidance (Pure Pursuit style waypoints).
        self.declare_parameter('avoid_goal_bypass_offset_m', 0.0)
        self.declare_parameter('avoid_goal_pass_margin_m', 0.12)
        self.declare_parameter('avoid_goal_cut_segment_len_m', 0.68)
        self.declare_parameter('avoid_goal_reach_tol_m', 0.07)
        self.declare_parameter('avoid_goal_heading_kp', 0.0)
        self.declare_parameter('avoid_goal_exit_inward_margin_m', 0.17)

        self._sync_stage2_runtime_parameters()
        self._sync_tester_runtime_parameters()

        # Main runtime state.
        self.phase = 2
        self.task_raw = self.test_direction_raw
        self.direction = self.test_direction
        self.reported_waiting_pose = False
        self.reported_start_delay = False
        self.last_progress_bucket = -1
        self.detour_front_test_angle_deg = min(self.detour_front_angle_deg, 35.0)
        self.detour_side_test_window_deg = min(self.detour_side_window_deg, 16.0)
        self.detour_strategy = 'goal_direct'
        self.detour_debug_log_period_sec = max(
            0.10,
            float(self.get_parameter('detour_debug_log_period_sec').value),
        )
        self.init_detour_debug_log_state()
        self.init_cmd_vel_safety()
        self.active_turn_heading_tolerance = self.heading_tolerance
        self.front_obstacle_angle_deg = 0.0
        self.detected_obstacle_circles = []
        self.active_obstacle_circle = None
        self.last_obstacle_circle_marker_count = 0
        self.avoid_bypass_max_lateral_m = 0.34
        self.segment_integrated_distance_m = 0.0
        self._segment_integrated_prev_xy = None
        self.ring_origin_world = None
        self._corner_shortcut_move_progress_reset = False
        self.reset_avoidance_runtime()
        self.reset_segment_world_plan()
        self._last_search_log_at = 0.0

        self.obstacle_circle_pub = self.create_publisher(MarkerArray, self.obstacle_circle_topic, 10)
        self.log_parameter_snapshot()

        self.log_debug_info(
            f'{self.test_feedback_prefix}节点已就绪，方向={self.direction_text()}，'
            f'模式={self.start_mode_text()}，'
            f'矩形参数=({self.rectangle_first_leg_m:.2f}, '
            f'{self.rectangle_side_leg_m:.2f}, {self.rectangle_top_leg_m:.2f})m，'
            f'避障算法={self.detour_strategy}（目标点斜切绕障），'
            f'触发距离={self.detour_obstacle_detect_distance:.2f}m，'
            f'清除距离={self.detour_obstacle_clear_distance:.2f}m，'
            f'前向检测角±{self.detour_front_test_angle_deg:.0f}度，'
            f'debug_log={self.debug_log_path}'
        )

    def _sync_stage2_runtime_parameters(self):
        sync_stage2_runtime_parameters(self)

    def _sync_tester_runtime_parameters(self):
        sync_tester_runtime_parameters(self)

    def apply_vehicle_parameters_from_ros(self):
        """set_parameters(yaml+launch) 后重读，离线仿真与实车 launch 对齐。"""
        self._sync_stage2_runtime_parameters()
        self._sync_tester_runtime_parameters()
        self.active_turn_heading_tolerance = self.heading_tolerance

    def resolve_test_direction(self, raw_value):
        normalized = str(raw_value).strip().lower()
        if normalized in ('clockwise', 'cw', '顺时针'):
            return 'clockwise'
        if normalized in ('counterclockwise', 'ccw', 'anticlockwise', 'anti-clockwise', '逆时针'):
            return 'counterclockwise'

        parsed = self.parse_direction(str(raw_value).strip())
        if parsed is not None:
            return parsed

        self.log_debug_warning(f'无法识别测试方向 "{raw_value}"，回退到顺时针')
        return 'clockwise'

    def direction_text(self):
        return '顺时针' if self.test_direction == 'clockwise' else '逆时针'

    def nav_succeeded_for_test_start(self):
        if self.test_start_mode in ('after_corridor', 'nav_succeeded', 'corridor', 'true'):
            return True
        if self.test_start_mode in ('full_entry', 'pre_loop', 'nav_failed', 'false'):
            return False
        return bool(self.use_corridor_path)

    def start_mode_text(self):
        if self.nav_succeeded_for_test_start():
            return '按比赛到达通道口后的惯导入口开始'
        return '按比赛未经过通道口时的完整入环动作开始'

    def log_parameter_snapshot(self):
        sections = {
            'topics': {
                'phase_topic': self.phase_topic,
                'task_topic': self.task_topic,
                'odom_topic': self.odom_topic,
                'imu_topic': self.imu_topic,
                'scan_topic': self.scan_topic,
                'cmd_topic': self.cmd_topic,
                'feedback_topic': self.feedback_topic,
                'state_topic': self.state_topic,
                'obstacle_circle_topic': self.obstacle_circle_topic,
            },
            'test': {
                'test_direction_raw': self.test_direction_raw,
                'resolved_direction': self.test_direction,
                'test_start_mode': self.test_start_mode,
                'nav_start_mode': self.start_mode_text(),
                'test_feedback_prefix': self.test_feedback_prefix,
                'rectangle_first_leg_m': f'{self.rectangle_first_leg_m:.3f}',
                'rectangle_side_leg_m': f'{self.rectangle_side_leg_m:.3f}',
                'rectangle_top_leg_m': f'{self.rectangle_top_leg_m:.3f}',
            },
            'motion': {
                'control_rate_hz': f'{self.control_rate_hz:.3f}',
                'start_delay_sec': f'{self.start_delay_sec:.3f}',
                'corridor_linear_speed': f'{self.corridor_linear_speed:.3f}',
                'ring_linear_speed': f'{self.ring_linear_speed:.3f}',
                'turn_linear_speed': f'{self.turn_linear_speed:.3f}',
                'turn_angular_speed': f'{self.turn_angular_speed:.3f}',
                'turn_min_angular_speed': f'{self.turn_min_angular_speed:.3f}',
                'turn_kp': f'{self.turn_kp:.3f}',
                'heading_kp': f'{self.heading_kp:.3f}',
                'max_angular_speed': f'{self.max_angular_speed:.3f}',
                'distance_tolerance': f'{self.distance_tolerance:.3f}',
                'heading_tolerance_deg': f'{math.degrees(self.heading_tolerance):.3f}',
                'segment_timeout': f'{self.segment_timeout:.3f}',
                'pure_pursuit_lookahead_m': f'{self.pure_pursuit_lookahead_m:.3f}',
                'pure_pursuit_heading_stop_deg': f'{math.degrees(self.pure_pursuit_heading_stop):.3f}',
                'pure_pursuit_turn_kp': f'{self.pure_pursuit_turn_kp:.3f}',
            },
            'debug': {
                'debug_log_path': self.debug_log_path,
                'detour_debug_log_period_sec': f'{self.detour_debug_log_period_sec:.3f}',
            },
            'obstacle': {
                'detour_enabled': self.detour_enabled,
                'detour_strategy': self.detour_strategy,
                'detour_obstacle_distance': f'{self.detour_obstacle_distance:.3f}',
                'detour_obstacle_detect_distance': f'{self.detour_obstacle_detect_distance:.3f}',
                'detour_obstacle_clear_distance': f'{self.detour_obstacle_clear_distance:.3f}',
                'detour_front_angle_deg': f'{self.detour_front_angle_deg:.3f}',
                'detour_front_test_angle_deg': f'{self.detour_front_test_angle_deg:.3f}',
                'detour_side_center_deg': f'{self.detour_side_center_deg:.3f}',
                'detour_side_window_deg': f'{self.detour_side_window_deg:.3f}',
                'detour_side_test_window_deg': f'{self.detour_side_test_window_deg:.3f}',
                'detour_min_side_clearance': f'{self.detour_min_side_clearance:.3f}',
                'detour_lateral_distance_m': f'{self.detour_lateral_distance_m:.3f}',
                'detour_forward_distance_m': f'{self.detour_forward_distance_m:.3f}',
                'detour_cooldown_sec': f'{self.detour_cooldown_sec:.3f}',
                'obstacle_circle_cluster_distance_threshold': f'{self.obstacle_circle_cluster_distance_threshold:.3f}',
                'obstacle_circle_min_cluster_points': self.obstacle_circle_min_cluster_points,
                'obstacle_circle_min_range_m': f'{self.obstacle_circle_min_range_m:.3f}',
                'obstacle_circle_max_range_m': f'{self.obstacle_circle_max_range_m:.3f}',
                'obstacle_circle_padding_m': f'{self.obstacle_circle_padding_m:.3f}',
                'obstacle_circle_min_radius_m': f'{self.obstacle_circle_min_radius_m:.3f}',
                'obstacle_circle_max_radius_m': f'{self.obstacle_circle_max_radius_m:.3f}',
                'obstacle_circle_max_cluster_span_m': f'{self.obstacle_circle_max_cluster_span_m:.3f}',
                'obstacle_circle_marker_height_m': f'{self.obstacle_circle_marker_height_m:.3f}',
                'obstacle_circle_path_half_width_m': f'{self.obstacle_circle_path_half_width_m:.3f}',
                'obstacle_corridor_body_half_width_m': f'{self.obstacle_corridor_body_half_width_m:.3f}',
                'obstacle_side_fence_center_y_m': f'{self.obstacle_side_fence_center_y_m:.3f}',
                'obstacle_opposite_wall_min_center_x_m': f'{self.obstacle_opposite_wall_min_center_x_m:.3f}',
                'detour_turn_max_trigger_distance_m': f'{self.detour_turn_max_trigger_distance_m:.3f}',
                'obstacle_opposite_front_min_distance_m': f'{self.obstacle_opposite_front_min_distance_m:.3f}',
                'obstacle_circle_planning_margin_m': f'{self.obstacle_circle_planning_margin_m:.3f}',
                'obstacle_circle_forward_margin_m': f'{self.obstacle_circle_forward_margin_m:.3f}',
            },
            'corridor_avoid': {
                'avoid_watch_distance_m': f'{self.avoid_watch_distance_m:.3f}',
                'avoid_commit_distance_m': f'{self.avoid_commit_distance_m:.3f}',
                'avoid_bias_yaw_deg': f'{self.avoid_bias_yaw_deg:.3f}',
                'avoid_bias_yaw_max_deg': f'{self.avoid_bias_yaw_max_deg:.3f}',
                'avoid_pass_clearance_m': f'{self.avoid_pass_clearance_m:.3f}',
                'avoid_parallel_front_margin_m': f'{self.avoid_parallel_front_margin_default_m:.3f}',
                'avoid_rejoin_heading_tol_deg': f'{math.degrees(self.avoid_rejoin_heading_tol):.3f}',
                'avoid_rejoin_lateral_tol_m': f'{self.avoid_rejoin_lateral_tol_m:.3f}',
                'avoid_corner_zone_before_m': f'{self.avoid_corner_zone_before_m:.3f}',
                'avoid_corner_zone_after_m': f'{self.avoid_corner_zone_after_m:.3f}',
                'avoid_corner_apex_box_m': f'{self.avoid_corner_apex_box_m:.3f}',
                'avoid_corner_prefer_inside': self.avoid_corner_prefer_inside,
                'avoid_corner_outside_margin_m': f'{self.avoid_corner_outside_margin_m:.3f}',
                'avoid_speed_out_mps': f'{self.avoid_speed_out_mps:.3f}',
                'avoid_speed_pass_mps': f'{self.avoid_speed_pass_mps:.3f}',
                'avoid_speed_rejoin_mps': f'{self.avoid_speed_rejoin_mps:.3f}',
                'avoid_corner_speed_mps': f'{self.avoid_corner_speed_mps:.3f}',
                'avoid_max_angular_speed': f'{self.avoid_max_angular_speed:.3f}',
                'avoid_approach_creep_speed_mps': f'{self.avoid_approach_creep_speed_mps:.3f}',
                'avoid_approach_speed_ratio': f'{self.avoid_approach_speed_ratio:.3f}',
            },
        }
        self.write_debug_log('CONFIG', 'startup_parameter_snapshot_begin')
        for section_name, items in sections.items():
            summary = '，'.join(f'{key}={value}' for key, value in items.items())
            self.write_debug_log('CONFIG', f'{section_name}: {summary}')
        self.write_debug_log('CONFIG', 'startup_parameter_snapshot_end')

    def log_segment_debug_snapshot(self, index, segment, label):
        segment_type = segment.get('type', 'unknown')
        extra = ''
        if segment_type == 'move':
            extra = f'distance_m={float(segment.get("distance_m", 0.0)):.3f}'
        elif segment_type == 'turn':
            extra = f'angle_deg={float(segment.get("angle_deg", 0.0)):.3f}'
        elif segment_type == 'pause':
            extra = f'duration={float(segment.get("duration", 0.0)):.3f}'
        self.write_debug_log(
            'SEGMENT',
            (
                f'index={index}，label={label}，description={segment.get("description", "unknown")}，'
                f'type={segment_type}，{extra}，'
                f'start_yaw={self.format_yaw_deg(self.segment_start_yaw)}deg，'
                f'target_yaw={self.format_yaw_deg(self.segment_target_yaw)}deg，'
                f'heading={self.format_yaw_deg(self.segment_heading)}deg，'
                f'pose={self.format_position_xy()}'
            ),
        )

    def reset_mission(self, clear_task):
        self.reset_avoidance_runtime()
        super().reset_mission(clear_task)

    def rectangle_segment_label(self, segment):
        description = str(segment.get('description', 'unknown'))
        if self.direction == 'clockwise':
            labels = {
                'rect_enter_align': '通道后起点入口对齐',
                'rect_first_leg': f'底边向左 {self.rectangle_first_leg_m:.2f}m 段',
                'rect_corner_1': '左下拐角',
                'rect_side_1': f'左边向上 {self.rectangle_side_leg_m:.2f}m 段',
                'rect_corner_2': '左上拐角',
                'rect_top': f'顶边向右 {self.rectangle_top_leg_m:.2f}m 段',
                'rect_corner_3': '右上拐角',
                'rect_side_2': f'右边向下 {self.rectangle_side_leg_m:.2f}m 段',
                'rect_corner_4': '右下拐角',
                'rect_return_origin': f'底边回起点 {self.rectangle_first_leg_m:.2f}m 段',
            }
        else:
            labels = {
                'rect_enter_align': '通道后起点入口对齐',
                'rect_first_leg': f'底边向右 {self.rectangle_first_leg_m:.2f}m 段',
                'rect_corner_1': '右下拐角',
                'rect_side_1': f'右边向上 {self.rectangle_side_leg_m:.2f}m 段',
                'rect_corner_2': '右上拐角',
                'rect_top': f'顶边向左 {self.rectangle_top_leg_m:.2f}m 段',
                'rect_corner_3': '左上拐角',
                'rect_side_2': f'左边向下 {self.rectangle_side_leg_m:.2f}m 段',
                'rect_corner_4': '左下拐角',
                'rect_return_origin': f'底边回起点 {self.rectangle_first_leg_m:.2f}m 段',
            }
        return labels.get(description, description)

    def build_inertial_plan(self, nav_succeeded):
        """与实车一致：通道到位后仍执行 rect_enter_align，再 rect_first_leg…"""
        pre_loop_plan = self.parse_pre_loop_plan()
        if nav_succeeded and self.corridor_path_skip_pre_loop_plan:
            pre_loop_plan = self.parse_post_corridor_path_plan()
        return pre_loop_plan + self.build_ring_plan()

    def _ring_track_geometry_kwargs(self):
        origin = getattr(self, 'ring_origin_world', None) or (0.0, 0.0, RING_CHANNEL_ENTRY_YAW_RAD)
        return {
            'direction': getattr(self, 'test_direction', 'clockwise'),
            'first_leg_m': self.rectangle_first_leg_m,
            'side_leg_m': self.rectangle_side_leg_m,
            'top_leg_m': self.rectangle_top_leg_m,
            'origin_xy': (float(origin[0]), float(origin[1])),
            'origin_yaw': float(origin[2]),
        }

    def reset_segment_integrated_distance(self):
        self.segment_integrated_distance_m = 0.0
        self._segment_integrated_prev_xy = None

    def odom_callback(self, msg):
        prev = self.current_position
        super().odom_callback(msg)
        if (
            self.current_segment is not None
            and self.current_segment.get('type') == 'move'
            and prev is not None
            and self.current_position is not None
        ):
            step = math.hypot(
                self.current_position[0] - prev[0],
                self.current_position[1] - prev[1],
            )
            if step > 1e-6:
                self.segment_integrated_distance_m += step

    def nominal_segment_geometry(self):
        return segment_endpoints_world(
            self.test_direction,
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
            **{
                k: v
                for k, v in self._ring_track_geometry_kwargs().items()
                if k in ('origin_xy', 'origin_yaw')
            },
        )

    def begin_inertial_plan_after_nav(self, nav_succeeded):
        self.reset_corridor_path_state()
        self.pending_segment_start_pose = self.current_position
        self.pending_segment_start_yaw = self.current_yaw
        if self.current_position is not None and self.current_yaw is not None:
            self.ring_origin_world = (
                float(self.current_position[0]),
                float(self.current_position[1]),
                float(self.current_yaw),
            )
        else:
            self.ring_origin_world = (0.0, 0.0, float(RING_CHANNEL_ENTRY_YAW_RAD))

        self.plan = self.build_inertial_plan(nav_succeeded)
        if not self.plan:
            self.publish_feedback('第二阶段没有可执行段，直接结束')
            self.finish_mission()
            return

        self.start_segment(0)

    def control_now_sec(self):
        if hasattr(self, '_offline_sim_time'):
            return float(self._offline_sim_time)
        return self.get_clock().now().nanoseconds / 1e9

    def offline_sim_advance(self, dt):
        self._offline_sim_time = float(getattr(self, '_offline_sim_time', 0.0)) + float(dt)

    def start_segment(self, index):
        self.reset_avoidance_runtime()
        self.reset_segment_integrated_distance()
        super().start_segment(index)
        # 离线仿真用 control_now_sec；实车与 wall clock 一致。
        self.segment_started_at = self.control_now_sec()
        self.last_progress_bucket = -1
        self.active_turn_heading_tolerance = self.heading_tolerance
        self.move_heading_settle_m = 0.0
        if index > 0 and self.plan[index - 1].get('type') == 'turn':
            if self.current_segment and self.current_segment.get('type') == 'move':
                self.move_heading_settle_m = 0.40
        if self.current_segment and self.current_segment.get('type') == 'move':
            if getattr(self, '_corner_shortcut_move_progress_reset', False):
                self._corner_shortcut_move_progress_reset = False

        if self.current_segment is None or self.plan_index != index:
            return

        segment = self.current_segment
        segment_type = segment.get('type')
        if segment_type == 'turn' and getattr(self, '_corner_shortcut_turn_target', None) is not None:
            if self.segment_heading is not None:
                self.segment_start_yaw = self.normalize_angle(float(self.segment_heading))
            elif self.current_yaw is not None:
                self.segment_start_yaw = self.normalize_angle(float(self.current_yaw))
            self.segment_target_yaw = self.normalize_angle(float(self._corner_shortcut_turn_target))
            if hasattr(self, 'sim') and self.segment_heading is not None:
                self.sim.yaw = float(self.segment_heading)
                self.current_yaw = self.sim.yaw
            self._corner_shortcut_turn_target = None
        if segment_type == 'turn' and 'force_start_yaw' in segment:
            self.segment_start_yaw = self.normalize_angle(float(segment['force_start_yaw']))
            self.segment_target_yaw = self.normalize_angle(
                self.segment_start_yaw + math.radians(float(segment.get('angle_deg', 0.0)))
            )
        if segment_type == 'turn' and 'force_target_yaw' in segment:
            self.segment_target_yaw = self.normalize_angle(float(segment['force_target_yaw']))
        if segment_type == 'turn' and 'heading_tolerance_rad' in segment:
            self.active_turn_heading_tolerance = max(1e-3, float(segment['heading_tolerance_rad']))
        if segment_type == 'move' and 'force_segment_heading' in segment:
            forced_heading = self.normalize_angle(float(segment['force_segment_heading']))
            self.segment_start_yaw = forced_heading
            self.segment_heading = forced_heading

        if segment_type == 'move':
            self.load_move_segment_world_plan()
        else:
            self.reset_segment_world_plan()

        label = self.rectangle_segment_label(segment)
        self.log_segment_debug_snapshot(index, segment, label)
        if segment_type == 'turn':
            angle_deg = float(segment.get('angle_deg', 0.0))
            turn_text = '左转' if angle_deg > 0.0 else '右转'
            self.publish_feedback(
                f'{self.test_feedback_prefix}当前位置: {label}，开始{turn_text} {abs(angle_deg):.0f} 度'
            )
            return

        if segment_type == 'move':
            distance_m = float(segment.get('distance_m', 0.0))
            self.log_mission_move_segment_begin(index)
            self.publish_feedback(
                f'{self.test_feedback_prefix}当前位置: {label}，目标直行 {distance_m:.2f}m'
            )
            return

        if segment_type == 'pause':
            self.publish_feedback(f'{self.test_feedback_prefix}当前位置: {label}，短暂停稳')

    def control_loop(self):
        if self.corridor_path_active:
            self.run_corridor_path_stage()
            return

        if not self.mission_active or self.current_segment is None:
            self.publish_cmd_vel()
            return

        now_sec = self.control_now_sec()
        last_move_await_finish = (
            self.plan_index >= len(self.plan) - 1
            and (self.current_segment or {}).get('type') == 'move'
            and not self.mission_last_segment_finish_pose_ready()
        )
        if (
            self.segment_started_at is not None
            and not getattr(self, 'avoidance_active', False)
            and now_sec - self.segment_started_at > self.segment_timeout
            and not last_move_await_finish
        ):
            self.publish_feedback(f'段超时，强制切换: {self.current_segment.get("description", "unknown")}')
            self.start_segment(self.plan_index + 1)
            return

        if self.navigation_step(now_sec):
            return

        # Hold mission move while a trigger is pending (avoidance enters next tick).
        if (
            self.current_segment.get('type') == 'move'
            and self.avoidance_should_enter()
        ):
            self.write_debug_log(
                'TRIGGER',
                (
                    f'进入避障区 corridor_watch nearest='
                    f'{self.format_distance(self.template_blocker_distance_m())}m | '
                    f'{self.format_scan_ranges_compact()}'
                ),
            )
            self.publish_cmd_vel(0.0, 0.0)
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

    def _mission_move_heading_error_rad(self):
        if self.segment_heading is None or self.current_yaw is None:
            return 0.0
        return self.angle_error(self.segment_heading, self.current_yaw)

    def _mission_move_pd_terms(self):
        lateral_error = self.segment_lateral_offset_m()
        heading_error = self._mission_move_heading_error_rad()
        lat_gain = 2.4
        head_deg = abs(math.degrees(heading_error))
        lat_eff = lat_gain
        if head_deg > 8.0:
            lat_eff = lat_gain * max(0.20, 1.0 - (head_deg - 8.0) / 28.0)
        lat_term = lat_eff * (-lateral_error)
        head_term = self.heading_kp * heading_error
        return lat_term, head_term, lateral_error, heading_error

    def nominal_finish_xy_world(self):
        return nominal_mission_finish_pose(
            self.test_direction,
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )

    def mission_return_finish_along_ok(self):
        """E2: 回程段沿名义弦线已足够前进，防 exit 绕障中途误 finish。"""
        seg = self.current_segment or {}
        if str(seg.get('description', '')) != 'rect_return_origin':
            return True
        if self.current_position is None or self.segment_heading is None:
            return False
        ep = segment_endpoints_world(
            self.test_direction,
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )
        start_xy, _end_xy = ep['rect_return_origin']
        along = world_segment.along_m(self.current_position, start_xy, self.segment_heading)
        target = float(seg.get('distance_m', 0.0))
        seg_tol = float(self.distance_tolerance)
        end_thresh = self.segment_end_progress_threshold_m(target, seg_tol)
        return along + seg_tol >= end_thresh

    def _finish_approach_cmd(self):
        """末段：沿当前 move 段弦线收敛（横偏+航向），勿横插名义终点。

        竖边绕障外鼓时，应靠 exit 段 lat 收回；末段只沿回程弦线小步前进。
        """
        if self.segment_heading is None or self.current_yaw is None:
            return 0.0, 0.0
        lateral_error = self.segment_lateral_offset_m()
        heading_error = self._mission_move_heading_error_rad()
        head_deg = abs(math.degrees(heading_error))
        lat_abs = abs(lateral_error)
        lat_term = 2.8 * (-lateral_error)
        head_term = self.heading_kp * heading_error
        if head_deg > 45.0:
            lat_term = 0.0
            head_term = math.copysign(
                min(self.max_angular_speed, abs(self.heading_kp * heading_error)),
                heading_error,
            )
        elif head_deg > 12.0:
            lat_term = 0.0
        angular = self.clamp(lat_term + head_term, self.max_angular_speed)
        if head_deg > 20.0 or lat_abs > FINISH_SKIP_TRIM_LAT_M:
            linear = 0.0
        else:
            fx, fy = self.nominal_finish_xy_world()
            dist_finish = math.hypot(
                self.current_position[0] - fx,
                self.current_position[1] - fy,
            )
            linear = min(0.08, 0.03 + 0.25 * max(0.0, dist_finish - FINISH_WORLD_DIST_M))
            linear *= max(0.2, math.cos(heading_error))
        return linear, angular

    def mission_last_segment_finish_pose_ready(self):
        """末 move 且里程达标、世界距终点近、沿回程 enough（E1+E2）。"""
        if self.plan_index < len(self.plan) - 1:
            return False
        if (self.current_segment or {}).get('type') != 'move':
            return False
        if self.current_position is None:
            return False
        target_distance = float((self.current_segment or {}).get('distance_m', 0.0))
        progress = self.projected_distance()
        tol = float(self.distance_tolerance)
        if progress < target_distance - tol:
            return False
        fx, fy = self.nominal_finish_xy_world()
        dist_finish = math.hypot(self.current_position[0] - fx, self.current_position[1] - fy)
        if dist_finish > FINISH_WORLD_DIST_M:
            return False
        return self.mission_return_finish_along_ok()

    def _log_mission_move_tick(
        self,
        phase,
        linear,
        angular,
        lat_term=None,
        head_term=None,
    ):
        segment = self.current_segment or {}
        self.maybe_log_mission_move_control(
            self.control_now_sec(),
            phase,
            float(linear),
            float(angular),
            self.segment_lateral_offset_m(),
            self._mission_move_heading_error_rad(),
            self.projected_distance(),
            float(segment.get('distance_m', 0.0)),
            lat_term=lat_term,
            head_term=head_term,
        )

    def run_move_segment(self):
        if self.current_segment is not None and self.current_segment.get('type') == 'move':
            target_distance = max(1e-6, float(self.current_segment.get('distance_m', 0.0)))
            progress = max(0.0, min(self.projected_distance(), target_distance))
            ratio = progress / target_distance
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
                    self.log_debug_info(
                        f'{self.test_feedback_prefix}当前位置: '
                        f'{self.rectangle_segment_label(self.current_segment)}，'
                        f'进度 {bucket * 25}% '
                        f'({progress:.2f}/{target_distance:.2f}m)'
                    )

            if progress >= target_distance - self.distance_tolerance and self.last_progress_bucket < 4:
                self.last_progress_bucket = 4
                self.publish_feedback(
                    f'{self.test_feedback_prefix}当前位置: '
                    f'{self.rectangle_segment_label(self.current_segment)}，'
                    f'直行到位，准备切换到下一段'
                )

        if self.current_position is None or self.segment_heading is None:
            self.publish_cmd_vel()
            return

        progress = self.projected_distance()
        target_distance = float(self.current_segment['distance_m'])
        tol = float(self.distance_tolerance)
        if progress >= target_distance - tol or self.segment_move_complete_on_plan(
            along_tol=tol,
            lat_tol=FINISH_SKIP_TRIM_LAT_M,
        ):
            last_move = self.plan_index >= len(self.plan) - 1
            if self.mission_last_segment_finish_pose_ready():
                lat = abs(self.segment_lateral_offset_m())
                head_err = (
                    abs(self.angle_error(self.segment_heading, self.current_yaw))
                    if self.current_yaw is not None and self.segment_heading is not None
                    else float('inf')
                )
                if lat <= FINISH_SKIP_TRIM_LAT_M and head_err <= FINISH_SKIP_TRIM_HEAD_RAD:
                    self.publish_cmd_vel()
                    self.start_segment(self.plan_index + 1)
                    return
                # E1: 世界距达标即完成；段内横偏/航向仅影响是否跳过 trim
                self.publish_cmd_vel()
                self.start_segment(self.plan_index + 1)
                return
            lat = abs(self.segment_lateral_offset_m())
            head_err = (
                abs(self.angle_error(self.segment_heading, self.current_yaw))
                if self.current_yaw is not None
                else 0.0
            )
            need_trim = lat > FINISH_SKIP_TRIM_LAT_M or head_err > FINISH_SKIP_TRIM_HEAD_RAD
            if last_move and not self.mission_last_segment_finish_pose_ready():
                linear, angular = self._finish_approach_cmd()
                lateral_error = self.segment_lateral_offset_m()
                heading_error = self._mission_move_heading_error_rad()
                self._log_mission_move_tick(
                    'finish_approach',
                    linear,
                    angular,
                    lat_term=2.8 * (-lateral_error),
                    head_term=self.heading_kp * heading_error,
                )
                self.publish_cmd_vel(linear, angular)
                return
            if need_trim:
                lateral_error = self.segment_lateral_offset_m()
                heading_error = self._mission_move_heading_error_rad()
                head_deg = abs(math.degrees(heading_error))
                if head_deg > 45.0:
                    lat_term = 0.0
                    head_term = self.heading_kp * heading_error
                    angular = self.clamp(head_term, self.max_angular_speed)
                elif head_deg > 12.0:
                    lat_term = 0.0
                    head_term = self.heading_kp * heading_error
                    angular = self.clamp(head_term, self.max_angular_speed)
                else:
                    lat_term = 2.8 * (-lateral_error)
                    head_term = self.heading_kp * heading_error
                    combined = lat_term + head_term
                    if abs(combined) < 0.04 and lat > FINISH_SKIP_TRIM_LAT_M:
                        lat_term = 2.8 * (-lateral_error)
                        head_term = 0.0
                        combined = lat_term
                    angular = self.clamp(combined, self.max_angular_speed)
                self._log_mission_move_tick(
                    'segment_end_trim',
                    0.0,
                    angular,
                    lat_term=lat_term,
                    head_term=head_term,
                )
                self.publish_cmd_vel(0.0, angular)
                return
            self.publish_cmd_vel()
            self.start_segment(self.plan_index + 1)
            return

        linear = float(self.current_segment.get('speed', self.corridor_linear_speed))
        approach_cap = self.mission_obstacle_linear_cap_mps()
        if approach_cap is not None:
            linear = min(linear, approach_cap)
            self.maybe_log_template_approach_cap(self.control_now_sec(), approach_cap, linear)
        nearest = self.detour_nearest_obstacle_distance_m()
        if math.isfinite(nearest) and nearest < 0.70:
            linear = min(linear, max(self.detour_follow_min_linear_m, nearest * 0.18))
        lat_term, head_term, lateral_error, heading_error = self._mission_move_pd_terms()
        linear, angular = self.mission_nominal_move_cmd(linear)
        settle_m = getattr(self, 'move_heading_settle_m', 0.0)
        if (
            settle_m > 0.0
            and progress < settle_m
            and self.current_yaw is not None
            and self.segment_heading is not None
        ):
            head_err = heading_error
            lat_abs = abs(self.segment_lateral_offset_m())
            if abs(head_err) > math.radians(5.0) or lat_abs > 0.12:
                lat_eff = 2.4 * max(0.20, 1.0 - abs(math.degrees(head_err)) / 35.0)
                lat_term = lat_eff * (-self.segment_lateral_offset_m())
                head_term = 2.4 * self.heading_kp * head_err
                angular = self.clamp(lat_term + head_term, 0.32)
                linear = min(linear, 0.18) * max(0.45, math.cos(head_err))
                linear, angular = self.mission_passed_static_obstacle_adjustment(linear, angular)
                self._log_mission_move_tick(
                    'post_turn_settle',
                    linear,
                    angular,
                    lat_term=lat_term,
                    head_term=head_term,
                )
                self.publish_cmd_vel(linear, angular)
                return
        linear, angular = self.mission_passed_static_obstacle_adjustment(linear, angular)
        self._log_mission_move_tick(
            'nominal_track',
            linear,
            angular,
            lat_term=lat_term,
            head_term=head_term,
        )
        self.publish_cmd_vel(linear, angular)

    def run_pause_segment(self, now_sec):
        self.publish_cmd_vel()
        duration = float(self.current_segment.get('duration', 0.0))
        if self.segment_started_at is not None and now_sec - self.segment_started_at >= duration:
            self.start_segment(self.plan_index + 1)

    def run_turn_segment(self):
        trim_cmd = self.consume_pending_lateral_trim_cmd()
        if trim_cmd is not None:
            self.publish_cmd_vel(trim_cmd[0], trim_cmd[1])
            return

        turn_tolerance = self.active_turn_heading_tolerance
        linear_speed = float(
            (self.current_segment or {}).get('turn_linear_speed', self.turn_linear_speed)
        )

        if self.current_yaw is None or self.segment_target_yaw is None:
            self.publish_cmd_vel()
            return

        error = self.angle_error(self.segment_target_yaw, self.current_yaw)
        if abs(error) <= turn_tolerance:
            self.publish_feedback(
                f'{self.test_feedback_prefix}当前位置: '
                f'{self.rectangle_segment_label(self.current_segment or {})}，'
                '转弯完成，进入下一段'
            )
            residual_deg = 0.0
            if self.segment_target_yaw is not None and self.current_yaw is not None:
                residual_deg = math.degrees(
                    self.angle_error(self.segment_target_yaw, self.current_yaw)
                )
            next_index = self.plan_index + 1
            if (
                next_index < len(self.plan)
                and self.plan[next_index].get('type') == 'move'
            ):
                self.write_debug_log(
                    'MOVE',
                    (
                        f'转弯结束→下一段 {self.plan[next_index].get("description", "?")} | '
                        f'弯末yaw={self.format_yaw_deg(self.current_yaw)}deg '
                        f'弯目标={self.format_yaw_deg(self.segment_target_yaw)}deg '
                        f'残差={residual_deg:+.1f}deg | 残差>5°则下一段前0.4m边转边走'
                    ),
                )
            self.publish_cmd_vel()
            self.start_segment(next_index)
            return

        angular = self.clamp(self.turn_kp * error, self.turn_angular_speed)
        if abs(error) > turn_tolerance and abs(angular) < self.turn_min_angular_speed:
            angular = math.copysign(self.turn_min_angular_speed, error)

        self.publish_cmd_vel(linear_speed, angular)

    def build_ring_plan(self):
        entry_turn = 90.0 if self.direction == 'clockwise' else -90.0
        corner_turn = -entry_turn
        return [
            {
                'type': 'turn',
                'angle_deg': entry_turn,
                'description': 'rect_enter_align',
                'allow_detour': False,
                'turn_linear_speed': self.turn_linear_speed,
            },
            {
                'type': 'move',
                'distance_m': self.rectangle_first_leg_m,
                'speed': self.ring_linear_speed,
                'description': 'rect_first_leg',
                'allow_detour': True,
            },
            {
                'type': 'turn',
                'angle_deg': corner_turn,
                'description': 'rect_corner_1',
                'allow_detour': False,
            },
            {
                'type': 'move',
                'distance_m': self.rectangle_side_leg_m,
                'speed': self.ring_linear_speed,
                'description': 'rect_side_1',
                'allow_detour': True,
            },
            {
                'type': 'turn',
                'angle_deg': corner_turn,
                'description': 'rect_corner_2',
                'allow_detour': False,
            },
            {
                'type': 'move',
                'distance_m': self.rectangle_top_leg_m,
                'speed': self.ring_linear_speed,
                'description': 'rect_top',
                'allow_detour': True,
            },
            {
                'type': 'turn',
                'angle_deg': corner_turn,
                'description': 'rect_corner_3',
                'allow_detour': False,
            },
            {
                'type': 'move',
                'distance_m': self.rectangle_side_leg_m,
                'speed': self.ring_linear_speed,
                'description': 'rect_side_2',
                'allow_detour': True,
            },
            {
                'type': 'turn',
                'angle_deg': corner_turn,
                'description': 'rect_corner_4',
                'allow_detour': False,
            },
            {
                'type': 'move',
                'distance_m': self.rectangle_first_leg_m,
                'speed': self.ring_linear_speed,
                'description': 'rect_return_origin',
                'allow_detour': True,
            },
        ]

    def phase_callback(self, msg):
        self.phase = 2

    def task_callback(self, msg):
        self.task_raw = self.test_direction_raw
        self.direction = self.test_direction

    def try_start_mission(self):
        if self.mission_active or self.mission_finished:
            return

        self.phase = 2
        self.direction = self.test_direction

        missing_inputs = []
        if self.current_position is None:
            missing_inputs.append('odom')
        if self.current_yaw is None:
            missing_inputs.append('imu')

        if missing_inputs:
            if not self.reported_waiting_pose:
                self.publish_feedback(
                    f'{self.test_feedback_prefix}等待输入就绪: {", ".join(missing_inputs)}'
                )
                self.reported_waiting_pose = True
            return

        current_time = self.control_now_sec()
        if self.start_after_time is None:
            self.start_after_time = current_time + self.start_delay_sec
            if not self.reported_start_delay:
                self.publish_feedback(
                    f'{self.test_feedback_prefix}位姿已就绪，{self.start_delay_sec:.2f}s 后开始'
                )
                self.reported_start_delay = True
            return

        if current_time < self.start_after_time:
            return

        self.mission_active = True
        self.reported_start = True
        self.publish_feedback(
            f'{self.test_feedback_prefix}开始执行，方向: {self.direction_text()}，'
            f'模式: {self.start_mode_text()}，'
            f'矩形圈: 左/右横边{self.rectangle_first_leg_m:.2f}m，'
            f'竖边{self.rectangle_side_leg_m:.2f}m，'
            f'顶部横边{self.rectangle_top_leg_m:.2f}m'
        )
        self.begin_inertial_plan_after_nav(nav_succeeded=self.nav_succeeded_for_test_start())


def main(args=None):
    rclpy.init(args=args)
    node = DirectInertialTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.publish_emergency_stop('main_finally')
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

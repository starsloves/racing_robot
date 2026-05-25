"""Main controller for the standalone stage2 inertial parameter tester.

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
from .ring_track import nominal_move_heading, progress_on_driven_segment_m, segment_endpoints_nominal
from .test_log_paths import debug_log_path as default_debug_log_path
from .vehicle_param_sync import sync_stage2_runtime_parameters, sync_tester_runtime_parameters


class DirectInertialTester(
    DirectInertialTesterNavigationMixin,
    DirectInertialTesterAvoidanceMixin,
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
        self._move_progress_along_entry_by_segment = {}
        self._pending_lateral_trim_chord = None
        self._corner_shortcut_move_progress_reset = False
        self.reset_avoidance_runtime()
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

    def nominal_segment_progress_m(self, world_xy=None):
        """Along-segment mileage: project world XY onto the nominal move chord (lateral bypass excluded)."""
        segment = self.current_segment
        if segment is None or segment.get('type') != 'move':
            return None
        if world_xy is None:
            if self.current_position is None:
                return None
            world_xy = self.current_position
        name = segment.get('description', '')
        endpoints = self.nominal_segment_geometry()
        if name not in endpoints:
            return None
        start, end = endpoints[name]
        sx, sy = float(start[0]), float(start[1])
        ex, ey = float(end[0]), float(end[1])
        px, py = float(world_xy[0]), float(world_xy[1])
        vx, vy = ex - sx, ey - sy
        length_sq = vx * vx + vy * vy
        if length_sq < 1e-9:
            return 0.0
        t = ((px - sx) * vx + (py - sy) * vy) / length_sq
        t = max(0.0, min(1.0, t))
        driven_progress = t * math.sqrt(length_sq)
        return self.plan_progress_from_driven_progress_m(driven_progress, name)

    def plan_progress_from_driven_progress_m(self, driven_progress_m, segment_name=None):
        """实测弦长与计划段长不一致时，把沿程里程缩放到计划 distance_m 坐标系。"""
        if driven_progress_m is None:
            return None
        segment = self.current_segment or {}
        name = segment_name or segment.get('description', '')
        plan_len = float(segment.get('distance_m', 0.0))
        from racing_stage2_param_test.ring_track import driven_segment_length_m

        driven_len = driven_segment_length_m(
            name,
            getattr(self, 'test_direction', 'clockwise'),
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )
        if driven_len and plan_len > 1e-3 and driven_len > 1e-3:
            return float(driven_progress_m) * (plan_len / driven_len)
        return float(driven_progress_m)

    def move_segment_entry_along_m(self, segment_name=None):
        name = segment_name or (self.current_segment or {}).get('description', '')
        if not name:
            return None
        return self._move_progress_along_entry_by_segment.get(name)

    def set_move_progress_entry_from_pose(self, segment_name=None):
        """转弯/避障 handoff 后：本段沿程里程从当前在弦线上的投影算起。"""
        name = segment_name or (self.current_segment or {}).get('description', '')
        if not name or self.current_position is None:
            return
        along = progress_on_driven_segment_m(
            self.current_position,
            name,
            getattr(self, 'test_direction', 'clockwise'),
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )
        if along is not None:
            self._move_progress_along_entry_by_segment[name] = float(along)

    def _driven_plan_progress_m(self, segment_name=None):
        if self.current_position is None:
            return None
        segment = self.current_segment or {}
        if segment.get('type') != 'move':
            return None
        name = segment_name or segment.get('description', '')
        if not name:
            return None
        along = progress_on_driven_segment_m(
            self.current_position,
            name,
            getattr(self, 'test_direction', 'clockwise'),
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )
        if along is None:
            return None
        entry = self.move_segment_entry_along_m(name)
        if entry is not None:
            along = max(0.0, float(along) - float(entry))
        return self.plan_progress_from_driven_progress_m(along, name)

    def projected_distance(self):
        progress = self._driven_plan_progress_m()
        if progress is not None:
            return progress
        progress = self.nominal_segment_progress_m()
        if progress is not None:
            return progress
        return super().projected_distance()

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
        from racing_stage2_param_test.ring_track import DRIVEN_CW_SEGMENT_ENDPOINTS

        if self.test_direction == 'clockwise' and DRIVEN_CW_SEGMENT_ENDPOINTS:
            return dict(DRIVEN_CW_SEGMENT_ENDPOINTS)
        return segment_endpoints_nominal(
            self.test_direction,
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )

    def anchor_move_segment_to_nominal(self):
        """只锚定段航向；起点保留弯后实车位姿（与实车 start_segment 一致）。"""
        segment = self.current_segment
        if segment is None or segment.get('type') != 'move':
            return
        name = segment.get('description', '')
        heading = nominal_move_heading(
            name,
            self.test_direction,
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )
        if heading is None:
            return
        self.segment_start_yaw = self.normalize_angle(heading)
        self.segment_heading = self.segment_start_yaw

    def begin_inertial_plan_after_nav(self, nav_succeeded):
        self.reset_corridor_path_state()
        self.pending_segment_start_pose = self.current_position
        self.pending_segment_start_yaw = self.current_yaw

        self.plan = self.build_inertial_plan(nav_succeeded)
        if not self.plan:
            self.publish_feedback('第二阶段没有可执行段，直接结束')
            self.finish_mission()
            return

        self.start_segment(0)
        if self.current_segment and self.current_segment.get('type') == 'move':
            self.anchor_move_segment_to_nominal()

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
                self.set_move_progress_entry_from_pose()
                self._corner_shortcut_move_progress_reset = False
            self.anchor_move_segment_to_nominal()

        if self.current_segment is None or self.plan_index != index:
            return

        segment = self.current_segment
        segment_type = segment.get('type')
        if segment_type == 'turn' and getattr(self, '_corner_shortcut_turn_target', None) is not None:
            self.segment_target_yaw = self.normalize_angle(float(self._corner_shortcut_turn_target))
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
        if (
            self.segment_started_at is not None
            and not getattr(self, 'avoidance_active', False)
            and now_sec - self.segment_started_at > self.segment_timeout
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
        if progress >= target_distance - self.distance_tolerance:
            lat = abs(self.segment_lateral_offset_m())
            head_err = (
                abs(self.angle_error(self.segment_heading, self.current_yaw))
                if self.current_yaw is not None
                else 0.0
            )
            if lat > 0.10 or head_err > math.radians(10.0):
                lateral_error = self.segment_lateral_offset_m()
                heading_error = (
                    self.angle_error(self.segment_heading, self.current_yaw)
                    if self.current_yaw is not None
                    else 0.0
                )
                angular = self.clamp(
                    2.8 * (-lateral_error) + self.heading_kp * heading_error,
                    self.max_angular_speed,
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
        linear, angular = self.mission_nominal_move_cmd(linear)
        settle_m = getattr(self, 'move_heading_settle_m', 0.0)
        if (
            settle_m > 0.0
            and progress < settle_m
            and self.current_yaw is not None
            and self.segment_heading is not None
        ):
            head_err = self.angle_error(self.segment_heading, self.current_yaw)
            if abs(head_err) > math.radians(5.0):
                angular = self.clamp(2.4 * self.heading_kp * head_err, 0.32)
                linear = min(linear, 0.18) * max(0.45, math.cos(head_err))
                linear, angular = self.mission_passed_static_obstacle_adjustment(linear, angular)
                self.publish_cmd_vel(linear, angular)
                return
        linear, angular = self.mission_passed_static_obstacle_adjustment(linear, angular)
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
            self.publish_cmd_vel()
            self.start_segment(self.plan_index + 1)
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

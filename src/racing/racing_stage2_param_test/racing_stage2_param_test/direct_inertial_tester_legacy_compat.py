"""Debug-log and FLOW/POSE hooks for restored v1 (odom progress) direct_inertial_tester."""

import math

from .test_log_paths import debug_log_path as default_debug_log_path


class DirectInertialTesterLegacyLogCompatMixin:
    """Keeps direct_inertial_tester_debug_log working without world-plan navigation."""

    def init_legacy_debug_log(self):
        self.declare_parameter('debug_log_path', '')
        self.declare_parameter('debug_log_verbose', False)
        self.declare_parameter('pose_log_period_sec', 0.5)
        self.declare_parameter('detour_debug_log_period_sec', 0.5)

        raw_path = str(self.get_parameter('debug_log_path').value).strip()
        self.debug_log_path = raw_path or default_debug_log_path()
        self.debug_log_verbose = bool(self.get_parameter('debug_log_verbose').value)
        self.detour_debug_log_period_sec = max(
            0.1,
            float(self.get_parameter('detour_debug_log_period_sec').value),
        )
        setattr(
            self,
            'pose_log_period_sec',
            max(0.25, float(self.get_parameter('pose_log_period_sec').value)),
        )
        self._flow_obstacle_watch_logged = False
        self._last_turn_debug_log_at = -1.0
        self._last_mission_move_log_at = -1.0

    @property
    def detour_strategy(self):
        return 'stage1_style'

    def resolved_field_track_config_path(self):
        return (
            f'odom/rectangle legs '
            f'({self.rectangle_first_leg_m:.2f}, '
            f'{self.rectangle_side_leg_m:.2f}, '
            f'{self.rectangle_top_leg_m:.2f}) m'
        )

    def format_nav_simple_line(self):
        pos = self.current_position
        if pos is None:
            xy = 'odom=nan'
        else:
            xy = f'odom=({float(pos[0]):.2f},{float(pos[1]):.2f})'
        if self.current_yaw is None:
            yaw = 'ψ=nan'
        else:
            yaw = f'ψ={self.format_yaw_deg(self.current_yaw)}°'
        segment = self.current_segment or {}
        if segment.get('type') == 'move':
            target = max(1e-6, float(segment.get('distance_m', 0.0)))
            progress = max(0.0, min(self.projected_distance(), target))
            return f'{xy} {yaw} prog={progress:.2f}/{target:.2f}m'
        return f'{xy} {yaw}'

    def begin_inertial_plan_after_nav(self, nav_succeeded):
        super().begin_inertial_plan_after_nav(nav_succeeded)
        if self.plan:
            self.log_flow_mission_ready()

    def legacy_log_turn_complete(self, error_rad):
        next_name = '?'
        next_index = self.plan_index + 1
        if 0 <= next_index < len(self.plan):
            next_name = str(self.plan[next_index].get('description', '?'))
        self.log_flow_turn_done(
            next_name,
            self.current_yaw,
            self.segment_target_yaw,
            error_rad,
        )

    def legacy_log_move_tick(self, now_sec, linear, angular, heading_error_rad):
        segment = self.current_segment or {}
        if segment.get('type') != 'move':
            return
        progress = self.projected_distance()
        target = float(segment.get('distance_m', 0.0))
        self.maybe_log_mission_move_control(
            now_sec,
            'odom',
            linear,
            angular,
            0.0,
            heading_error_rad,
            progress,
            target,
        )

    def legacy_log_turn_tick(self, now_sec, error_rad, angular, linear_speed):
        self.log_turn_tick_snapshot(
            now_sec,
            None,
            self.current_yaw,
            self.segment_target_yaw,
            error_rad,
            angular,
            linear_speed,
        )

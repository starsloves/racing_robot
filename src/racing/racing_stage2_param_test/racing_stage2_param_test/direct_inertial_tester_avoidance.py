"""Goal-direct avoidance — pose + world waypoint targets.

On enter: plan bypass → pass → (rejoin | exit) as odom (x,y) goals on nominal plan S→E.
Segment along/lat use world_plan_nav (fixed plan chord, not per-entry P0).
"""

import math

from . import world_segment
from .ring_track import (
    apply_turn_arc_world,
    preferred_bypass_side_for_segment,
    segment_end_goal_world,
    segment_endpoints_world,
)


class DirectInertialTesterAvoidanceMixin:
    """Unified goal_direct detour for straight move segments."""

    GOAL_PHASES = ('bypass', 'pass', 'rejoin', 'exit', 'next_leg')

    def reset_avoidance_runtime(self):
        passed_world = list(getattr(self, 'avoidance_passed_obstacles_world', []))
        cooldown_until = float(getattr(self, 'detour_cooldown_until', 0.0))
        self.avoidance_active = False
        self.avoidance_started_at = None
        self.avoidance_entry_progress = 0.0
        self.avoidance_clear_streak = 0
        self.avoidance_obstacle_passed = False
        self.locked_bypass_side = 0
        self.locked_bypass_side_at_enter = 0
        self.locked_obstacle_circle = None
        self.locked_obstacle_along_s = None
        self.locked_obstacle_world_xy = None
        self.locked_obstacle_lost_at = None
        self.avoid_target_offset_m = 0.28
        self.avoid_parallel_front_margin_m = getattr(
            self, 'avoid_parallel_front_margin_default_m', 0.18
        )
        self.avoid_template_feasible = True
        self.avoid_forbidden_linear_block = False
        self._last_avoid_cmd_log_at = -1.0
        self._last_goal_detour_log_at = -1.0
        self.scan_obstacle_points_robot = []
        self.avoidance_passed_obstacles_world = passed_world
        self.detour_cooldown_until = cooldown_until
        self.goal_direct_phase = 'follow'
        self.goal_direct_sequence = []
        self.goal_direct_index = 0
        self.goal_bypass_xy = None
        self.goal_pass_xy = None
        self.goal_rejoin_xy = None
        self.goal_exit_xy = None
        self.goal_next_leg_xy = None
        self.goal_pass_along_m = None
        self.goal_pass_lateral_start_m = 0.0
        self.goal_rejoin_along_m = None
        self.need_direct_cut = False
        self.goal_direct_phase_started_at = None

    @property
    def avoidance_phase(self):
        if not getattr(self, 'avoidance_active', False):
            return 'follow'
        phase = getattr(self, 'goal_direct_phase', 'follow')
        if phase in self.GOAL_PHASES:
            return phase
        return 'handoff'

    def phase_elapsed_sec(self, now_sec):
        if getattr(self, 'avoidance_started_at', None) is None:
            return 0.0
        return max(0.0, now_sec - self.avoidance_started_at)

    def goal_direct_phase_elapsed_sec(self, now_sec):
        started = getattr(self, 'goal_direct_phase_started_at', None)
        if started is None:
            return self.phase_elapsed_sec(now_sec)
        return max(0.0, now_sec - float(started))

    def _mark_goal_direct_phase_started(self, now_sec=None):
        if now_sec is None:
            now_sec = self.control_now_sec() if hasattr(self, 'control_now_sec') else 0.0
        self.goal_direct_phase_started_at = float(now_sec)

    def locked_obstacle_world_xy_for_bypass(self):
        world = getattr(self, 'locked_obstacle_world_xy', None)
        if world is not None:
            return float(world[0]), float(world[1])
        circle = self.active_obstacle_circle
        if circle is not None:
            return self.obstacle_center_world_xy(circle)
        return None

    def bypass_side_from_segment_obstacle(self, required_lat):
        """锥桶相对当前段 P0+ψ 的左右。"""
        world = self.locked_obstacle_world_xy_for_bypass()
        if world is None or self.segment_start_pose is None or self.segment_heading is None:
            return None, False
        cross = world_segment.lateral_m(world, self.segment_start_pose, self.segment_heading)
        if abs(cross) < 0.03:
            return None, False
        bypass_side = 1 if cross < 0.0 else -1
        shift = self.effective_side_clearance_m(float(-bypass_side))
        return bypass_side, shift >= required_lat * 0.65

    def effective_side_clearance_m(self, lateral_sign):
        if lateral_sign > 0.0:
            base = self.left_clearance_distance
        else:
            base = self.right_clearance_distance
        if not math.isfinite(base):
            base = 0.0
        body = self.obstacle_corridor_body_half_width_m
        circle = self.active_obstacle_circle
        if circle is not None:
            center_y = float(circle.get('center_y', 0.0))
            radius = self.obstacle_circle_planning_radius(circle) or float(circle['radius'])
            if lateral_sign > 0.0 and center_y < 0.0:
                base -= radius + self.obstacle_circle_planning_margin_m
            if lateral_sign < 0.0 and center_y > 0.0:
                base -= radius + self.obstacle_circle_planning_margin_m
        return max(0.0, base - body)

    def compute_bypass_offset_m(self):
        circle = self.active_obstacle_circle
        radius = 0.12
        if circle is not None:
            planning_r = self.obstacle_circle_planning_radius(circle)
            radius = planning_r if planning_r is not None else float(circle['radius'])
        tracking_slack_m = 0.04
        offset = (
            radius
            + self.obstacle_circle_planning_margin_m
            + self.obstacle_corridor_body_half_width_m
            + self.avoid_pass_clearance_m
            + tracking_slack_m
        )
        seg_len = float((self.current_segment or {}).get('distance_m', 99.0))
        if seg_len < float(getattr(self, 'avoid_goal_cut_segment_len_m', 0.68)):
            tracking_slack_m = 0.05
        if circle is not None and hasattr(self, 'circle_offset_in_segment_path_frame'):
            frame = self.circle_offset_in_segment_path_frame(circle)
            if frame is not None:
                _along, obs_lat = frame
                offset = max(
                    offset,
                    abs(obs_lat)
                    + radius
                    + self.obstacle_corridor_body_half_width_m
                    + self.avoid_pass_clearance_m
                    + tracking_slack_m,
                )
        cap = float(getattr(self, 'avoid_goal_bypass_offset_m', getattr(self, 'avoid_bypass_max_lateral_m', 0.34)))
        offset = min(cap, max(0.28, offset))
        if seg_len < float(getattr(self, 'avoid_goal_cut_segment_len_m', 0.68)):
            offset = min(0.32, cap, max(0.28, offset))
        return offset

    def select_bypass_side(self, required_lat):
        """一次锁定绕边：内绕 preferred 优先；横偏明显走对侧；正中 preferred；否则比 clearance。"""
        segment_name = (self.current_segment or {}).get('description', '')
        preferred = preferred_bypass_side_for_segment(
            segment_name,
            getattr(self, 'test_direction', 'clockwise'),
        )

        if preferred in (-1, 1):
            pref_shift = self.effective_side_clearance_m(float(-preferred))
            seg_len = float((self.current_segment or {}).get('distance_m', 99.0))
            cut_len = float(getattr(self, 'avoid_goal_cut_segment_len_m', 0.68))
            if pref_shift >= required_lat * 0.65 and seg_len < cut_len:
                return preferred, True

        world = None
        static = getattr(self, 'scenario_static_obstacles_world', [])
        if static:
            world = (float(static[0][0]), float(static[0][1]))
        elif self.active_obstacle_circle is not None:
            world = self.obstacle_center_world_xy(self.active_obstacle_circle)
        cross = None
        if segment_name and world is not None and self.segment_start_pose is not None and self.segment_heading is not None:
            cross = world_segment.lateral_m(world, self.segment_start_pose, self.segment_heading)
        if cross is not None and abs(cross) < 0.03 and preferred in (-1, 1):
            shift = self.effective_side_clearance_m(float(-preferred))
            if shift >= required_lat * 0.60:
                return preferred, True

        driven_side, driven_ok = self.bypass_side_from_segment_obstacle(required_lat)
        if driven_ok:
            return driven_side, True

        left_shift = self.effective_side_clearance_m(1.0)
        right_shift = self.effective_side_clearance_m(-1.0)
        if left_shift >= required_lat and right_shift >= required_lat:
            if preferred in (-1, 1):
                pref_shift = self.effective_side_clearance_m(float(-preferred))
                if pref_shift >= required_lat * 0.65:
                    return preferred, True
            if left_shift > right_shift + 0.02:
                return 1, True
            if right_shift > left_shift + 0.02:
                return -1, True
            if preferred in (-1, 1):
                return preferred, True
            return (1 if left_shift >= right_shift else -1), True
        if left_shift >= required_lat:
            return 1, True
        if right_shift >= required_lat:
            return -1, True
        if preferred in (-1, 1):
            pref_shift = self.effective_side_clearance_m(float(-preferred))
            opp = -preferred
            opp_shift = self.effective_side_clearance_m(float(-opp))
            if pref_shift >= opp_shift:
                return preferred, pref_shift >= required_lat * 0.82
            return opp, opp_shift >= required_lat * 0.82
        if left_shift >= right_shift:
            return 1, left_shift >= required_lat * 0.82 or right_shift >= required_lat * 0.82
        return -1, right_shift >= required_lat * 0.82 or left_shift >= required_lat * 0.82

    def resolve_locked_obstacle_along_s_m(self, circle):
        """Obstacle along-segment position in plan frame (for triggers / pass checks)."""
        world = self.locked_obstacle_world_xy_for_bypass()
        if world is not None:
            along = self.progress_along_segment_m(world)
            if along is not None:
                seg_len = float((self.current_segment or {}).get('distance_m', 99.0))
                if -0.05 <= along <= seg_len + 0.20:
                    return max(0.0, float(along))
        static = getattr(self, 'scenario_static_obstacles_world', [])
        for sx, sy, _sr in static:
            along = self.progress_along_segment_m((sx, sy))
            if along is not None and -0.05 <= along <= float(
                (self.current_segment or {}).get('distance_m', 99.0)
            ) + 0.20:
                return max(0.0, float(along))
        if hasattr(self, 'circle_offset_in_segment_path_frame'):
            frame = self.circle_offset_in_segment_path_frame(circle)
            if frame is not None:
                along_rel, _lat = frame
                return max(0.0, float(self.projected_distance()) + float(along_rel))
        world = self.obstacle_center_world_xy(circle)
        if world is not None:
            along = self.progress_along_segment_m(world)
            if along is not None:
                return max(0.0, float(along))
        return None

    def upcoming_move_segment(self):
        next_index = self.plan_index + 1
        plan = getattr(self, 'plan', [])
        if next_index + 1 >= len(plan):
            return None
        if plan[next_index].get('type') != 'turn':
            return None
        upcoming = plan[next_index + 1]
        if upcoming.get('type') != 'move':
            return None
        return upcoming

    def upcoming_move_entry_world(self):
        """下一段 move 入口（名义链式折线或 turn 弧后近似）。"""
        upcoming = self.upcoming_move_segment()
        if upcoming is None:
            return None
        name = str(upcoming.get('description', ''))
        geo = self._ring_track_geometry_kwargs()
        endpoints = segment_endpoints_world(
            geo['direction'],
            geo['first_leg_m'],
            geo['side_leg_m'],
            geo['top_leg_m'],
            origin_xy=geo.get('origin_xy', (0.0, 0.0)),
            origin_yaw=geo.get('origin_yaw', 0.0),
        )
        if name in endpoints:
            return endpoints[name][0]
        if (
            self.current_position is not None
            and self.segment_heading is not None
            and self.next_plan_segment_is_turn()
        ):
            next_turn = self.plan[self.plan_index + 1]
            x, y, next_heading = apply_turn_arc_world(
                float(self.current_position[0]),
                float(self.current_position[1]),
                float(self.segment_heading),
                float(next_turn.get('angle_deg', 0.0)),
            )
            return (
                x + math.cos(next_heading) * 0.15,
                y + math.sin(next_heading) * 0.15,
            )
        return None

    def plan_avoidance_goals(self):
        """入障时算 bypass / pass / rejoin|next_leg 世界坐标与 need_direct_cut。"""
        segment_name = (self.current_segment or {}).get('description', '')
        world = self.locked_obstacle_world_xy_for_bypass()
        side = int(self.locked_bypass_side or 1)
        offset = float(self.avoid_target_offset_m)
        tol = float(self.distance_tolerance)
        seg_len_plan = float((self.current_segment or {}).get('distance_m', 0.0))
        s_end = max(0.0, seg_len_plan - tol)

        s_obs_plan = getattr(self, 'locked_obstacle_along_s', None)
        if s_obs_plan is None:
            s_obs_plan = seg_len_plan * 0.5
        s_obs_plan = float(s_obs_plan)

        if world is not None:
            s_along = self.progress_along_segment_m(world)
            if s_along is not None:
                s_obs_plan = max(0.0, float(s_along))

        radius = 0.12
        static = getattr(self, 'scenario_static_obstacles_world', [])
        if static:
            radius = float(static[0][2])
        elif self.locked_obstacle_circle is not None:
            radius = float(self.locked_obstacle_circle.get('radius', 0.12))

        half_span = max(0.18, offset * 0.9)
        cut_len = float(getattr(self, 'avoid_goal_cut_segment_len_m', 0.68))
        need_direct_cut = (
            (s_obs_plan + half_span > s_end - tol)
            or (seg_len_plan < cut_len)
        )
        self.need_direct_cut = need_direct_cut

        lateral = float(side) * offset
        bypass = self.segment_progress_to_world(s_obs_plan, lateral)
        if bypass is None and world is not None:
            bypass = (float(world[0]), float(world[1]))

        pass_margin = float(getattr(self, 'avoid_goal_pass_margin_m', 0.12))
        pass_along = min(seg_len_plan, s_obs_plan + radius + pass_margin)
        self.goal_pass_along_m = float(pass_along)
        self.goal_pass_lateral_start_m = float(lateral) * 0.85
        pass_goal = self.segment_progress_to_world(pass_along, self.goal_pass_lateral_start_m)

        inward = float(getattr(self, 'avoid_goal_exit_inward_margin_m', 0.17))
        inward += float(self.obstacle_corridor_body_half_width_m) * 0.5
        geo = self._ring_track_geometry_kwargs()
        exit_goal = segment_end_goal_world(
            segment_name,
            geo['direction'],
            inward,
            geo['first_leg_m'],
            geo['side_leg_m'],
            geo['top_leg_m'],
            origin_xy=geo.get('origin_xy', (0.0, 0.0)),
            origin_yaw=geo.get('origin_yaw', 0.0),
        )
        if exit_goal is None:
            exit_goal = self.segment_progress_to_world(seg_len_plan, 0.0)

        next_leg_goal = None
        if not need_direct_cut and not self.next_plan_segment_is_turn():
            next_leg_goal = self.upcoming_move_entry_world()

        rejoin_goal = None
        self.goal_rejoin_along_m = None
        if not need_direct_cut:
            s_mirror_plan = min(s_obs_plan + half_span, s_end)
            ahead = float(getattr(self, 'avoid_goal_rejoin_ahead_m', 0.22))
            progress_plan = self.projected_distance()
            s_rejoin = min(max(progress_plan + ahead, s_mirror_plan), s_end)
            self.goal_rejoin_along_m = float(s_rejoin)
            rejoin_goal = self.segment_progress_to_world(s_rejoin, 0.0)

        self.goal_bypass_xy = bypass
        self.goal_pass_xy = pass_goal
        self.goal_rejoin_xy = rejoin_goal
        self.goal_exit_xy = exit_goal
        self.goal_next_leg_xy = next_leg_goal

        if need_direct_cut:
            self.goal_direct_sequence = ['bypass', 'pass', 'exit']
        elif next_leg_goal is not None:
            self.goal_direct_sequence = ['bypass', 'pass', 'next_leg']
        else:
            self.goal_direct_sequence = ['bypass', 'pass', 'rejoin']
        self.goal_direct_index = 0
        self.goal_direct_phase = self.goal_direct_sequence[0]
        self._mark_goal_direct_phase_started()

    def lock_avoidance_geometry(self):
        self.avoid_target_offset_m = self.compute_bypass_offset_m()
        self.locked_bypass_side, self.avoid_template_feasible = self.select_bypass_side(
            self.avoid_target_offset_m
        )
        circle = self.active_obstacle_circle
        self.locked_obstacle_along_s = None
        self.locked_obstacle_world_xy = None
        if circle is not None:
            self.locked_obstacle_circle = dict(circle)
            static = getattr(self, 'scenario_static_obstacles_world', [])
            for sx, sy, _sr in static:
                along = self.progress_along_segment_m((sx, sy))
                seg_len = float((self.current_segment or {}).get('distance_m', 99.0))
                if along is not None and -0.05 <= along <= seg_len + 0.20:
                    self.locked_obstacle_world_xy = (float(sx), float(sy))
                    break
            if self.locked_obstacle_world_xy is None:
                world = self.obstacle_center_world_xy(circle)
                if world is None and self.segment_start_pose is not None:
                    along = self.obstacle_along_segment_m(circle)
                    if along is not None:
                        world = (
                            float(self.segment_start_pose[0])
                            + math.cos(self.segment_heading) * float(along),
                            float(self.segment_start_pose[1])
                            + math.sin(self.segment_heading) * float(along),
                        )
                self.locked_obstacle_world_xy = world
            self.locked_obstacle_along_s = self.resolve_locked_obstacle_along_s_m(circle)
        self.locked_obstacle_lost_at = None
        self.avoid_parallel_front_margin_m = self.avoid_parallel_front_margin_default_m
        self.plan_avoidance_goals()

    def obstacle_along_segment_m(self, circle):
        world = self.obstacle_center_world_xy(circle)
        if world is None:
            return None
        return self.progress_along_segment_m(world)

    def refresh_locked_obstacle_circle(self):
        if self.locked_obstacle_circle is None:
            return
        locked = self.locked_obstacle_circle
        for circle in self.planning_obstacle_circles():
            dist = math.hypot(
                float(circle['center_x']) - float(locked['center_x']),
                float(circle['center_y']) - float(locked['center_y']),
            )
            if dist <= float(locked['radius']) + 0.15:
                self.locked_obstacle_circle = dict(circle)
                return

    def next_plan_segment_is_turn(self):
        next_index = self.plan_index + 1
        if next_index >= len(getattr(self, 'plan', [])):
            return False
        return self.plan[next_index].get('type') == 'turn'

    def upcoming_corner_target_yaw(self):
        if not self.next_plan_segment_is_turn():
            return None
        if self.segment_heading is None:
            return None
        next_seg = self.plan[self.plan_index + 1]
        angle_deg = float(next_seg.get('angle_deg', 0.0))
        return self.normalize_angle(self.segment_heading + math.radians(angle_deg))

    def direct_cut_lateral_handoff_limit_m(self):
        return max(
            self.avoid_rejoin_lateral_tol_m * 2.0,
            float(self.avoid_target_offset_m) * 0.45,
        )

    def exit_handoff_lat_limit_m(self):
        """E3: exit handoff 横偏上限（与 goal_direct_handoff_ready 一致）。"""
        return min(self.direct_cut_lateral_handoff_limit_m(), 0.10)

    def segment_along_past_locked_obstacle(self, margin_m=0.08):
        s_obs = getattr(self, 'locked_obstacle_along_s', None)
        if s_obs is None:
            return True
        radius = float((self.locked_obstacle_circle or {}).get('radius', 0.12))
        progress = self.projected_distance()
        return progress + 0.04 >= float(s_obs) + radius + float(margin_m)

    def segment_end_progress_threshold_m(self, target_m, seg_tol=None):
        tol = float(seg_tol if seg_tol is not None else self.distance_tolerance)
        return max(float(target_m) - tol, float(target_m) * 0.88)

    def direct_cut_at_segment_end(self, seg_tol=None):
        """段末：名义沿程达标或已靠近 plan 终点 E。"""
        tol = float(seg_tol if seg_tol is not None else self.distance_tolerance)
        target = float((self.current_segment or {}).get('distance_m', 0.0))
        progress = self.projected_distance()
        along_ok = progress + tol >= self.segment_end_progress_threshold_m(target, tol)
        end_ok = self.distance_to_segment_plan_end_m() <= self.SEGMENT_END_REACH_M
        return along_ok or end_ok

    def dynamic_rejoin_goal_xy(self):
        """Rejoin target stays ahead of progress (never behind the robot)."""
        if getattr(self, 'goal_direct_phase', '') != 'rejoin':
            return getattr(self, 'goal_rejoin_xy', None)
        s_obs = getattr(self, 'locked_obstacle_along_s', None)
        if s_obs is None:
            return getattr(self, 'goal_rejoin_xy', None)
        offset = float(self.avoid_target_offset_m)
        half_span = max(0.18, offset * 0.9)
        ahead = float(getattr(self, 'avoid_goal_rejoin_ahead_m', 0.22))
        seg_len = float((self.current_segment or {}).get('distance_m', 0.0))
        tol = float(self.distance_tolerance)
        s_end = max(0.0, seg_len - tol)
        progress = self.projected_distance()
        s_mirror = min(float(s_obs) + half_span, s_end)
        along = min(max(progress + ahead, s_mirror), s_end)
        self.goal_rejoin_along_m = float(along)
        goal = self.segment_progress_to_world(along, 0.0)
        if goal is not None:
            self.goal_rejoin_xy = goal
        return goal

    def prepare_direct_cut_corner_handoff(self):
        """direct_cut handoff：下一段 move 从 turn 结束位姿起算。"""
        self._corner_shortcut_move_progress_reset = True

    def consume_pending_lateral_trim_cmd(self):
        return None

    def active_avoidance_goal_xy(self):
        phase = getattr(self, 'goal_direct_phase', '')
        if phase == 'bypass':
            rolling = self.rolling_bypass_goal_xy()
            if rolling is not None:
                return rolling
            return getattr(self, 'goal_bypass_xy', None)
        if phase == 'pass':
            dynamic = self.dynamic_pass_goal_xy()
            if dynamic is not None:
                return dynamic
            return getattr(self, 'goal_pass_xy', None)
        if phase == 'rejoin':
            return self.dynamic_rejoin_goal_xy()
        if phase == 'exit':
            end_xy = self.segment_plan_end_xy
            if end_xy is not None and self.distance_to_segment_plan_end_m() > self.SEGMENT_END_REACH_M:
                return end_xy
            return getattr(self, 'goal_exit_xy', None) or end_xy
        if phase == 'next_leg':
            return getattr(self, 'goal_next_leg_xy', None)
        return None

    def dynamic_pass_goal_xy(self):
        """Pass phase: segment goal with decaying lateral (direct_cut → exit)."""
        if getattr(self, 'goal_direct_phase', '') != 'pass':
            return None
        pass_along = getattr(self, 'goal_pass_along_m', None)
        if pass_along is None:
            return None
        lateral = self.current_pass_lateral_m()
        return self.segment_progress_to_world(pass_along, lateral)

    def current_pass_lateral_m(self):
        side = float(self.locked_bypass_side or 1)
        start_lat = float(getattr(self, 'goal_pass_lateral_start_m', 0.0))
        if not getattr(self, 'need_direct_cut', False):
            return start_lat
        progress = self.projected_distance()
        pass_along = getattr(self, 'goal_pass_along_m', None)
        seg_len = float((self.current_segment or {}).get('distance_m', 0.0))
        if pass_along is None:
            return start_lat
        span = max(0.12, seg_len - float(pass_along))
        t = self._clamp01((float(progress) - float(pass_along)) / span)
        return side * abs(start_lat) * (1.0 - 0.72 * t)

    def current_exit_lateral_m(self):
        """Exit phase: decay bypass lateral toward 0 along remaining segment."""
        if getattr(self, 'goal_direct_phase', '') != 'exit':
            return 0.0
        if not getattr(self, 'need_direct_cut', False):
            return 0.0
        side = float(self.locked_bypass_side or 1)
        start_lat = float(getattr(self, 'goal_pass_lateral_start_m', 0.0))
        pass_along = getattr(self, 'goal_pass_along_m', None)
        progress = self.projected_distance()
        seg_len = float((self.current_segment or {}).get('distance_m', 0.0))
        if pass_along is None:
            return side * abs(start_lat) * 0.35
        span = max(0.10, seg_len - float(pass_along))
        t = self._clamp01((float(progress) - float(pass_along)) / span)
        return side * abs(start_lat) * (1.0 - 0.85 * t)

    def goal_direct_exit_segment_cmd(self):
        """direct_cut exit: track segment ψ with decaying lateral target (no world PP)."""
        if self.segment_heading is None or self.current_yaw is None:
            return 0.0, 0.0
        progress = self.projected_distance()
        target = float((self.current_segment or {}).get('distance_m', 0.0))
        seg_tol = float(self.distance_tolerance)
        end_thresh = self.segment_end_progress_threshold_m(target, seg_tol)
        remaining = max(0.0, end_thresh - progress)
        target_lat = self.current_exit_lateral_m()
        lateral_error = self.segment_lateral_offset_m() - target_lat
        heading_error = self.angle_error(self.segment_heading, self.current_yaw)
        head_deg = abs(math.degrees(heading_error))
        lat_gain = 2.4
        if progress >= end_thresh and abs(lateral_error) > 0.06 and head_deg <= 20.0:
            lat_gain_eff = 3.2
            head_kp_eff = 0.0
        else:
            lat_gain_eff = lat_gain
            if head_deg > 8.0:
                lat_gain_eff = lat_gain * max(0.35, 1.0 - (head_deg - 8.0) / 32.0)
            head_kp_eff = self.heading_kp * (0.45 if head_deg > 15.0 else 0.70)
        angular = self.clamp(
            lat_gain_eff * (-lateral_error) + head_kp_eff * heading_error,
            self.avoid_max_angular_speed,
        )
        v_max = self.goal_direct_base_speed_mps()
        clear = self.goal_clearance_gate()
        pass_along = getattr(self, 'goal_pass_along_m', progress)
        span = max(0.12, end_thresh - float(pass_along))
        linear = v_max * self._clamp01(remaining / span) * clear
        linear = max(self.avoid_approach_creep_speed_mps * 0.45, linear)
        if abs(lateral_error) > 0.12:
            linear = max(linear, v_max * 0.38)
        if remaining <= 0.0 and abs(lateral_error) > self.exit_handoff_lat_limit_m():
            linear = max(linear, self.avoid_approach_creep_speed_mps * 0.55)
        if end_thresh - progress < seg_tol * 1.5:
            linear = min(linear, v_max * 0.55)
        if not self.avoid_template_feasible:
            linear = 0.0
            angular = self.clamp(
                lat_gain * 0.6 * (-lateral_error) + self.heading_kp * 0.6 * heading_error,
                self.avoid_max_angular_speed * 0.6,
            )
        if not self.goal_motion_allows_cmd(linear, angular):
            linear = min(linear, self.avoid_approach_creep_speed_mps * 0.40)
            self.avoid_forbidden_linear_block = linear < 0.02
        else:
            self.avoid_forbidden_linear_block = False
        return max(0.0, linear), angular

    def bypass_lateral_build_fraction(self):
        target_lat = float(self.locked_bypass_side or 1) * float(self.avoid_target_offset_m)
        return self._clamp01(abs(self.segment_lateral_offset_m()) / max(abs(target_lat), 0.08))

    def rolling_bypass_goal_xy(self):
        """过桶前：沿程只比当前略前 + 全横偏，先侧移不顶锥桶（短边/斜切）。"""
        s_obs = getattr(self, 'locked_obstacle_along_s', None)
        if s_obs is None:
            return None
        seg_len_plan = float((self.current_segment or {}).get('distance_m', 0.0))
        cut_len = float(getattr(self, 'avoid_goal_cut_segment_len_m', 0.68))
        if not getattr(self, 'need_direct_cut', False) and seg_len_plan >= cut_len:
            return None
        progress_plan = self.projected_distance()
        if progress_plan + 0.04 >= float(s_obs):
            return None
        radius = float((self.locked_obstacle_circle or {}).get('radius', 0.12))
        lat_build = self.bypass_lateral_build_fraction()
        if getattr(self, 'need_direct_cut', False):
            ahead = max(0.05, min(0.12, 0.05 + 0.10 * lat_build))
        else:
            ahead = max(0.03, min(0.07, 0.03 + 0.06 * lat_build))
        anchor = min(float(s_obs) - max(0.03, radius * 0.5), progress_plan + ahead)
        lateral = float(self.locked_bypass_side or 1) * float(self.avoid_target_offset_m)
        return self.segment_progress_to_world(max(0.0, anchor), lateral)

    def distance_to_active_goal_m(self):
        goal = self.active_avoidance_goal_xy()
        if goal is None or self.current_position is None:
            return float('inf')
        return math.hypot(goal[0] - self.current_position[0], goal[1] - self.current_position[1])

    def goal_reach_tolerance_m(self):
        return float(getattr(self, 'avoid_goal_reach_tol_m', 0.07))

    def advance_goal_direct_phase(self):
        seq = getattr(self, 'goal_direct_sequence', [])
        idx = getattr(self, 'goal_direct_index', 0)
        if idx + 1 >= len(seq):
            return False
        self.goal_direct_index = idx + 1
        self.goal_direct_phase = seq[self.goal_direct_index]
        self._mark_goal_direct_phase_started()
        return True

    def update_goal_direct_phase(self):
        tol = self.goal_reach_tolerance_m()
        dist = self.distance_to_active_goal_m()
        phase = getattr(self, 'goal_direct_phase', 'bypass')

        if phase == 'bypass':
            if dist <= tol or self.robot_passed_locked_obstacle():
                self.advance_goal_direct_phase()
            return
        if phase == 'pass':
            progress = self.projected_distance()
            target = float((self.current_segment or {}).get('distance_m', 0.0))
            if (
                self.robot_passed_locked_obstacle()
                or dist <= tol
                or progress >= target - float(self.distance_tolerance) * 1.5
                or self.distance_to_segment_plan_end_m() <= self.SEGMENT_END_REACH_M * 1.8
            ):
                self.advance_goal_direct_phase()
            return
        if phase in ('rejoin', 'exit', 'next_leg') and dist <= tol:
            return
        if phase == 'next_leg':
            return

    @staticmethod
    def _clamp01(value):
        return min(1.0, max(0.0, float(value)))

    def goal_clearance_gate(self):
        if self.obstacle_passed_for_handoff():
            return 1.0
        nearest = self.template_blocker_distance_m()
        if not math.isfinite(nearest):
            return 1.0
        watch = float(self.avoid_watch_distance_m)
        commit = float(self.avoid_commit_distance_m)
        if nearest >= watch:
            return 1.0
        span = max(0.05, watch - commit)
        gate = self._clamp01((nearest - commit) / span)
        if not getattr(self, 'avoidance_active', False):
            return gate
        if nearest < 0.14:
            return max(0.10, gate)
        return max(gate, 0.35)

    # ------------------------------------------------------------------ triggers
    def avoidance_can_run(self):
        if not self.detour_enabled or self.current_segment is None:
            return False
        if self.current_segment.get('type') != 'move':
            return False
        return bool(self.current_segment.get('allow_detour', True))

    def template_path_blocker_imminent(self):
        return self.dwa_path_blocker_imminent()

    def template_blocker_distance_m(self):
        scenario_circle = self.scenario_static_obstacle_circle()
        if scenario_circle is not None:
            return float(
                scenario_circle.get('nearest_distance', scenario_circle.get('closest_x', 99.0))
            )
        nearest = self.detour_nearest_obstacle_distance_m()
        circle = self.active_obstacle_circle
        if circle is not None:
            return float(circle.get('nearest_distance', circle.get('closest_x', nearest)))
        return nearest

    def scenario_obstacle_applies_to_current_segment(self):
        target_segment = getattr(self, 'scenario_obstacle_segment', '')
        if not target_segment:
            return True
        segment = self.current_segment or {}
        if segment.get('type') != 'move':
            return False
        if segment.get('description', '') != target_segment:
            return False
        static = getattr(self, 'scenario_static_obstacles_world', [])
        if not static:
            return True
        sx, sy, sr = static[0]
        return self.obstacle_on_active_segment_path(sx, sy, sr)

    def avoidance_should_enter(self):
        if not self.avoidance_can_run():
            return False
        if not self.scenario_obstacle_applies_to_current_segment():
            return False
        if hasattr(self, 'approaching_turn_segment_end') and self.approaching_turn_segment_end():
            circle = self.active_obstacle_circle
            if circle is None or not self.circle_blocks_segment_path(circle):
                return False
        now_sec = self.control_now_sec() if hasattr(self, 'control_now_sec') else 0.0
        if now_sec < getattr(self, 'detour_cooldown_until', 0.0):
            return False
        passed = getattr(self, 'avoidance_passed_obstacles_world', [])
        static = getattr(self, 'scenario_static_obstacles_world', [])
        if passed and static and len(passed) >= len(static):
            return False
        if self.obstacle_already_passed_in_mission():
            return False
        if not self.dwa_path_blocker_imminent():
            return False
        nearest = self.template_blocker_distance_m()
        return math.isfinite(nearest) and nearest <= self.avoid_watch_distance_m

    def avoidance_in_approach_envelope(self):
        if self.avoidance_active or not self.avoidance_can_run():
            return False
        if not self.dwa_path_blocker_imminent():
            return False
        nearest = self.template_blocker_distance_m()
        if not math.isfinite(nearest):
            return False
        return (
            nearest > self.avoid_watch_distance_m
            and nearest <= self.detour_obstacle_detect_distance
        )

    def mission_obstacle_linear_cap_mps(self):
        if not self.avoidance_in_approach_envelope():
            return None
        nearest = self.template_blocker_distance_m()
        if not math.isfinite(nearest):
            return self.avoid_approach_creep_speed_mps
        span = max(
            0.08,
            self.detour_obstacle_detect_distance - self.avoid_watch_distance_m,
        )
        blend = (nearest - self.avoid_watch_distance_m) / span
        blend = min(1.0, max(0.0, blend))
        segment_speed = float((self.current_segment or {}).get('speed', self.ring_linear_speed))
        cap = self.avoid_approach_creep_speed_mps + blend * (
            segment_speed * self.avoid_approach_speed_ratio - self.avoid_approach_creep_speed_mps
        )
        return max(self.avoid_approach_creep_speed_mps, cap)

    # ------------------------------------------------------------------ pass / handoff
    def tracking_obstacle_robot_frame(self):
        circle = self.active_obstacle_circle
        if circle is not None and self.locked_obstacle_circle is not None:
            locked = self.locked_obstacle_circle
            dist = math.hypot(
                float(circle['center_x']) - float(locked['center_x']),
                float(circle['center_y']) - float(locked['center_y']),
            )
            if dist <= float(locked['radius']) + 0.18:
                return circle
        if circle is not None:
            return circle
        if self.locked_obstacle_circle is not None:
            return self.locked_obstacle_circle
        return None

    def refresh_locked_obstacle_along_s(self):
        if getattr(self, 'locked_obstacle_along_s', None) is not None:
            return
        circle = self.locked_obstacle_circle
        if circle is None:
            return
        along = self.resolve_locked_obstacle_along_s_m(circle)
        if along is not None:
            self.locked_obstacle_along_s = along

    def obstacle_center_world_xy(self, circle):
        if circle is None:
            return None
        if self.current_position is None or self.current_yaw is None:
            return None
        cx = float(circle.get('center_x', 0.0))
        cy = float(circle.get('center_y', 0.0))
        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)
        wx = self.current_position[0] + cos_y * cx - sin_y * cy
        wy = self.current_position[1] + sin_y * cx + cos_y * cy
        return wx, wy

    def locked_obstacle_physically_passed(self):
        track = self.tracking_obstacle_robot_frame()
        if track is None:
            return False
        farthest_x = float(track.get('farthest_x', track.get('closest_x', 0.0)))
        return farthest_x < -self.avoid_parallel_front_margin_m

    def robot_passed_locked_obstacle(self, margin_m=0.08):
        if self.locked_obstacle_physically_passed():
            self.avoidance_obstacle_passed = True
            return True
        phase = getattr(self, 'goal_direct_phase', '')
        if phase in ('pass', 'exit', 'next_leg') and getattr(self, 'need_direct_cut', False):
            track = self.tracking_obstacle_robot_frame()
            if track is not None and float(track.get('farthest_x', 0.0)) < -margin_m:
                self.avoidance_obstacle_passed = True
                return True
        s_obs = getattr(self, 'locked_obstacle_along_s', None)
        if s_obs is None:
            return False
        radius = float((self.locked_obstacle_circle or {}).get('radius', 0.12))
        progress = self.projected_distance()
        along_pass = progress + 0.04 >= float(s_obs) + radius + margin_m
        if not along_pass:
            self.avoidance_obstacle_passed = False
            return False
        offset = float(self.avoid_target_offset_m)
        lat_ok = abs(self.segment_lateral_offset_m()) >= offset * 0.32
        passed = lat_ok or getattr(self, 'need_direct_cut', False)
        self.avoidance_obstacle_passed = passed
        return passed

    def obstacle_passed_for_handoff(self, margin_m=0.10):
        del margin_m
        return self.robot_passed_locked_obstacle()

    def recover_track_complete(self):
        if self.segment_heading is None or self.current_yaw is None:
            return False
        heading_err = abs(self.angle_error(self.segment_heading, self.current_yaw))
        lateral_err = abs(self.segment_lateral_offset_m())
        return (
            heading_err <= self.avoid_rejoin_heading_tol
            and lateral_err <= self.avoid_rejoin_lateral_tol_m
        )

    def goal_direct_handoff_ready(self):
        if not self.avoidance_active:
            return False
        phase = getattr(self, 'goal_direct_phase', '')
        if phase not in ('rejoin', 'exit', 'next_leg'):
            return False
        dist = self.distance_to_active_goal_m()
        tol = self.goal_reach_tolerance_m()
        seg_tol = float(self.distance_tolerance)
        target = float((self.current_segment or {}).get('distance_m', 0.0))
        progress = self.projected_distance()
        relaxed_lat = max(
            self.avoid_rejoin_lateral_tol_m * 2.0,
            float(self.avoid_target_offset_m) * 0.35,
        )
        lat_ok = abs(self.segment_lateral_offset_m()) <= relaxed_lat
        passed_ok = (
            self.obstacle_passed_for_handoff(margin_m=0.08)
            or self.segment_along_past_locked_obstacle(margin_m=0.08)
        )

        if phase == 'exit' and getattr(self, 'need_direct_cut', False):
            if not passed_ok:
                return False
            end_thresh = self.segment_end_progress_threshold_m(target, seg_tol)
            if progress + seg_tol < end_thresh and self.distance_to_segment_plan_end_m() > self.SEGMENT_END_REACH_M:
                return False
            handoff_lat = self.exit_handoff_lat_limit_m()
            if abs(self.segment_lateral_offset_m()) > handoff_lat:
                return False
            if self.distance_to_segment_plan_end_m() > self.SEGMENT_END_REACH_M * 1.15:
                return False
            return self.avoidance_clear_streak >= 1

        if not passed_ok:
            return False

        if phase == 'rejoin':
            self.dynamic_rejoin_goal_xy()
            rejoin_along = getattr(self, 'goal_rejoin_along_m', None)
            if rejoin_along is not None and progress + seg_tol >= float(rejoin_along) and lat_ok:
                return self.avoidance_clear_streak >= 1

        if dist > tol * 1.35 and phase != 'next_leg':
            return False
        if self.recover_track_complete():
            return self.avoidance_clear_streak >= 1
        if lat_ok:
            return self.avoidance_clear_streak >= 1
        return dist <= tol * 0.85

    def segment_avoidance_handoff_ready(self):
        return self.goal_direct_handoff_ready()

    # ------------------------------------------------------------------ debug / legacy hooks
    @property
    def local_replan_active(self):
        return False

    @property
    def local_replan_points(self):
        return []

    def local_replan_status_metrics(self):
        progress = self.projected_distance()
        entry = getattr(self, 'avoidance_entry_progress', 0.0)
        heading_err = 0.0
        if self.segment_heading is not None and self.current_yaw is not None:
            heading_err = self.angle_error(self.segment_heading, self.current_yaw)
        segment = self.current_segment or {}
        target = float(segment.get('distance_m', 0.0))
        return {
            'travel_distance': max(0.0, progress - entry),
            'locked_path_progress': max(0.0, progress - entry),
            'goal_distance': max(0.0, target - progress),
            'min_travel_before_finish': self.min_detour_travel_before_finish(),
            'heading_error': heading_err,
            'obstacle_passed': self.obstacle_passed_for_handoff(),
            'locked_path_ready': self.recover_track_complete(),
        }

    def locked_path_progress_state(self):
        entry = getattr(self, 'avoidance_entry_progress', 0.0)
        progress = max(0.0, self.projected_distance() - entry)
        target = float((self.current_segment or {}).get('distance_m', 0.0))
        return 0, progress, target

    def min_detour_travel_before_finish(self):
        return max(0.20, float(getattr(self, 'avoid_target_offset_m', 0.28)) * 0.45)

    def min_locked_path_progress_before_finish(self):
        return 0.0

    def format_local_replan_metrics(self, metrics):
        goal = self.active_avoidance_goal_xy()
        goal_text = 'none'
        if goal is not None:
            goal_text = f'({goal[0]:.2f},{goal[1]:.2f})'
        end_d = self.distance_to_segment_plan_end_m()
        return (
            f'mode=goal_direct phase={self.avoidance_phase} goal={goal_text} '
            f'travel={metrics.get("travel_distance", 0.0):.2f}m '
            f'along={self.projected_distance():.2f}m dist_E={end_d:.2f}m '
            f'passed={metrics.get("obstacle_passed", False)} '
            f'track_ok={metrics.get("locked_path_ready", False)}'
        )

    def detour_ready_for_rejoin_follow(self, metrics=None):
        return self.recover_track_complete() and self.obstacle_passed_for_handoff()

    # ------------------------------------------------------------------ tracking / safety
    def locked_circle_still_visible(self):
        if self.locked_obstacle_circle is None:
            return False
        locked = self.locked_obstacle_circle
        for circle in self.planning_obstacle_circles():
            dist = math.hypot(
                float(circle['center_x']) - float(locked['center_x']),
                float(circle['center_y']) - float(locked['center_y']),
            )
            if dist <= float(locked['radius']) + 0.12:
                return True
        return False

    def update_locked_obstacle_tracking(self, now_sec):
        if self.locked_obstacle_circle is None:
            return True
        if self.locked_circle_still_visible():
            self.locked_obstacle_lost_at = None
            self.refresh_locked_obstacle_circle()
            self.refresh_locked_obstacle_along_s()
            return True
        if self.locked_obstacle_lost_at is None:
            self.locked_obstacle_lost_at = now_sec
        if (now_sec - self.locked_obstacle_lost_at) < self.avoid_perception_loss_hold_sec:
            return True
        if (
            getattr(self, 'locked_obstacle_world_xy', None) is not None
            and getattr(self, 'locked_obstacle_along_s', None) is not None
        ):
            return True
        if self.locked_obstacle_physically_passed():
            return True
        if self.obstacle_passed_for_handoff(margin_m=0.08):
            return True
        return False

    def robot_body_radius_m(self):
        return max(self.obstacle_corridor_body_half_width_m + 0.06, 0.16)

    def dwa_clearance_along_motion(
        self,
        linear_v,
        angular_w,
        horizon_sec=0.55,
        steps=9,
        ignore_locked_obstacle=False,
    ):
        if steps <= 0:
            return float('inf')
        dt = horizon_sec / steps
        x = 0.0
        y = 0.0
        yaw = 0.0
        body_r = self.robot_body_radius_m()
        min_clear = float('inf')
        hits = self.scan_obstacle_points_robot
        circles = []
        locked = self.locked_obstacle_circle if ignore_locked_obstacle else None
        for circle in self.planning_obstacle_circles():
            if locked is not None:
                dist = math.hypot(
                    float(circle['center_x']) - float(locked['center_x']),
                    float(circle['center_y']) - float(locked['center_y']),
                )
                if dist <= float(locked['radius']) + 0.20:
                    continue
            planning_radius = self.obstacle_circle_planning_radius(circle)
            if planning_radius is None:
                continue
            circles.append((
                float(circle['center_x']),
                float(circle['center_y']),
                planning_radius + body_r,
            ))
        locked_center = None
        locked_skip_r = 0.0
        if ignore_locked_obstacle and locked is not None:
            locked_center = (
                float(locked.get('center_x', 0.0)),
                float(locked.get('center_y', 0.0)),
            )
            locked_skip_r = float(locked.get('radius', 0.12)) + body_r + 0.14

        for _ in range(steps):
            for hit_x, hit_y in hits:
                if locked_center is not None:
                    if (
                        math.hypot(hit_x - locked_center[0], hit_y - locked_center[1])
                        <= locked_skip_r
                    ):
                        continue
                min_clear = min(
                    min_clear,
                    math.hypot(hit_x - x, hit_y - y) - body_r,
                )
            for center_x, center_y, forbidden_radius in circles:
                min_clear = min(
                    min_clear,
                    math.hypot(center_x - x, center_y - y) - forbidden_radius,
                )
            yaw += angular_w * dt
            x += linear_v * math.cos(yaw) * dt
            y += linear_v * math.sin(yaw) * dt
        return min_clear

    def motion_safe(self, linear_v, angular_w):
        if abs(linear_v) < 1e-4 and abs(angular_w) < 1e-4:
            return True
        clearance = self.dwa_clearance_along_motion(
            linear_v,
            angular_w,
            horizon_sec=0.40,
            steps=7,
            ignore_locked_obstacle=True,
        )
        return clearance >= self.robot_body_radius_m() * 0.55

    def goal_motion_allows_cmd(self, linear_v, angular_w):
        return self.motion_safe(linear_v, angular_w)

    # ------------------------------------------------------------------ control
    def goal_direct_base_speed_mps(self):
        segment = self.current_segment or {}
        phase = getattr(self, 'goal_direct_phase', 'bypass')
        if phase == 'bypass':
            base = max(self.avoid_speed_out_mps, self.avoid_speed_pass_mps)
        elif phase == 'pass':
            base = self.avoid_speed_pass_mps
        elif phase in ('exit', 'next_leg') and getattr(self, 'need_direct_cut', False):
            base = min(
                float(segment.get('speed', self.ring_linear_speed)) * 0.75,
                self.avoid_corner_speed_mps,
            )
        else:
            base = self.avoid_speed_rejoin_mps
        return min(
            float(segment.get('speed', self.ring_linear_speed)),
            max(base, self.avoid_approach_creep_speed_mps),
        )

    def goal_direct_cmd(self, now_sec):
        del now_sec
        if self.current_position is None or self.current_yaw is None:
            return 0.0, 0.0

        self.update_goal_direct_phase()

        phase = getattr(self, 'goal_direct_phase', 'bypass')
        if (
            phase == 'exit'
            and getattr(self, 'need_direct_cut', False)
            and self.distance_to_segment_plan_end_m() <= self.SEGMENT_END_REACH_M
        ):
            return self.goal_direct_exit_segment_cmd()

        goal = self.active_avoidance_goal_xy()
        if goal is None:
            return 0.0, 0.0

        gx, gy = float(goal[0]), float(goal[1])
        x, y = float(self.current_position[0]), float(self.current_position[1])
        dist = math.hypot(gx - x, gy - y)
        heading_cmd = math.atan2(gy - y, gx - x)
        heading_err = self.angle_error(heading_cmd, self.current_yaw)

        kp = float(getattr(self, 'avoid_goal_heading_kp', self.heading_kp))
        if phase == 'bypass':
            kp *= 1.25
        omega = self.clamp(kp * heading_err, self.avoid_max_angular_speed)

        v_max = self.goal_direct_base_speed_mps()
        reach = max(self.goal_reach_tolerance_m(), 0.05)
        dist_scale = self._clamp01(dist / max(reach * 2.2, 0.14))
        curve_scale = self._clamp01(1.0 - abs(heading_err) / math.radians(50.0))
        clear = self.goal_clearance_gate()
        linear = v_max * max(0.30, dist_scale) * max(0.40, curve_scale) * clear

        s_obs = getattr(self, 'locked_obstacle_along_s', None)
        progress = self.projected_distance()
        before_abreast = s_obs is not None and progress + 0.04 < float(s_obs)
        if phase == 'bypass' and before_abreast:
            linear = min(linear, v_max * 0.48)
            nearest = self.template_blocker_distance_m()
            if math.isfinite(nearest) and nearest < 0.18:
                linear = min(linear, self.avoid_approach_creep_speed_mps * 0.70)
        if phase == 'pass':
            nearest = self.template_blocker_distance_m()
            if math.isfinite(nearest) and nearest < 0.15:
                creep = self.avoid_approach_creep_speed_mps
                scale = 0.42 if nearest < 0.11 else 0.58
                linear = min(linear, creep * scale)

        if dist <= reach * 1.2:
            linear = min(linear, v_max * 0.65)
        if getattr(self, 'goal_direct_phase', '') == 'exit' and getattr(self, 'need_direct_cut', False):
            linear = min(linear, v_max * 0.72)

        if not self.avoid_template_feasible:
            linear = 0.0
            omega = self.clamp(kp * 0.6 * heading_err, self.avoid_max_angular_speed * 0.6)

        if not self.goal_motion_allows_cmd(linear, omega):
            linear = min(linear, self.avoid_approach_creep_speed_mps * 0.40)
            self.avoid_forbidden_linear_block = linear < 0.02
        else:
            self.avoid_forbidden_linear_block = False

        return max(0.0, linear), omega

    def publish_goal_direct_cmd(self, linear, angular, now_sec):
        phase = self.avoidance_phase
        state = 'avoid_active'
        if not self.avoid_template_feasible or self.avoid_forbidden_linear_block:
            state = 'avoid_hold'
        self.publish_state(state)
        self.publish_cmd_vel(max(0.0, linear), angular)
        if now_sec - self._last_avoid_cmd_log_at >= 0.20:
            self._last_avoid_cmd_log_at = now_sec
            if hasattr(self, 'log_template_cmd'):
                self.log_template_cmd(phase, linear, angular, now_sec)

    def maybe_log_goal_direct_snapshot(self, now_sec):
        if not self.avoidance_active:
            return
        if now_sec - self._last_goal_detour_log_at < self.detour_debug_log_period_sec:
            return
        self._last_goal_detour_log_at = now_sec
        if hasattr(self, 'maybe_log_template_detour_snapshot'):
            self.maybe_log_template_detour_snapshot(now_sec)

    def avoidance_stuck_handoff_ready(self, now_sec):
        if not self.avoidance_active:
            return False
        if not self.obstacle_passed_for_handoff(margin_m=0.08):
            return False
        phase = getattr(self, 'goal_direct_phase', '')
        if phase not in ('rejoin', 'exit', 'next_leg'):
            return False
        phase_elapsed = self.goal_direct_phase_elapsed_sec(now_sec)
        if phase == 'exit' and getattr(self, 'need_direct_cut', False):
            if phase_elapsed >= 8.0 and self.distance_to_segment_plan_end_m() <= self.SEGMENT_END_REACH_M * 1.5:
                handoff_lat = self.exit_handoff_lat_limit_m()
                if abs(self.segment_lateral_offset_m()) <= handoff_lat * 1.25:
                    return True
            if self.direct_cut_at_segment_end() and phase_elapsed >= 6.0:
                handoff_lat = self.exit_handoff_lat_limit_m()
                if abs(self.segment_lateral_offset_m()) <= handoff_lat:
                    return True
                return False
        if phase_elapsed >= 10.0 and self.distance_to_active_goal_m() <= self.goal_reach_tolerance_m() * 2.5:
            return True
        segment = self.current_segment or {}
        target = float(segment.get('distance_m', 0.0))
        progress = self.projected_distance()
        tol = float(self.distance_tolerance)
        if (
            progress + tol >= target
            and phase_elapsed >= 3.0
            and phase in ('exit', 'next_leg')
            and getattr(self, 'need_direct_cut', False)
        ):
            end_thresh = self.segment_end_progress_threshold_m(target, tol)
            if progress + tol < end_thresh:
                return False
            return abs(self.segment_lateral_offset_m()) <= self.exit_handoff_lat_limit_m()
        return False

    def format_goal_compact(self):
        def fmt(pt):
            if pt is None:
                return 'none'
            return f'({pt[0]:.2f},{pt[1]:.2f})'

        return (
            f'phase={self.goal_direct_phase} cut={self.need_direct_cut} '
            f'bypass={fmt(self.goal_bypass_xy)} pass={fmt(self.goal_pass_xy)} '
            f'rejoin={fmt(self.goal_rejoin_xy)} exit={fmt(self.goal_exit_xy)} '
            f'next_leg={fmt(self.goal_next_leg_xy)}'
        )

    # ------------------------------------------------------------------ mission helpers
    def mission_nominal_move_cmd(self, linear_mps):
        if self.segment_heading is None or self.current_yaw is None:
            return float(linear_mps), 0.0
        lateral_error = self.segment_lateral_offset_m()
        heading_error = self.angle_error(self.segment_heading, self.current_yaw)
        lat_gain = 2.4
        head_deg = abs(math.degrees(heading_error))
        # 横偏大时仍保留航向项；否则只拧横向会把 yaw 拧飞（见 MOVE 日志 STEER_FLIP）。
        lat_eff = lat_gain
        if head_deg > 8.0:
            lat_eff = lat_gain * max(0.20, 1.0 - (head_deg - 8.0) / 28.0)
        angular = lat_eff * (-lateral_error) + self.heading_kp * heading_error
        if head_deg > 20.0:
            angular = self.clamp(angular, min(self.max_angular_speed, 0.45))
        angular = self.clamp(angular, self.max_angular_speed)
        linear = float(linear_mps) * max(0.55, math.cos(heading_error))
        return linear, angular

    def obstacle_on_active_segment_path(
        self,
        world_x,
        world_y,
        obstacle_radius,
        along_margin=0.20,
        lateral_extra=0.12,
    ):
        if self.current_segment is None or self.current_segment.get('type') != 'move':
            return False
        if not self._segment_plan_frame_ready():
            return False
        along = self.progress_along_segment_m((world_x, world_y))
        if along is None:
            return False
        length = self.segment_plan_length_m
        if along < -along_margin or along > length + along_margin:
            return False
        lateral = world_segment.lateral_m(
            (float(world_x), float(world_y)),
            self.segment_plan_start_xy,
            self.segment_plan_heading_rad,
        )
        half_width = self.driving_corridor_half_width_m() + float(obstacle_radius) + lateral_extra
        return abs(lateral) <= half_width

    def mission_passed_static_obstacle_adjustment(self, linear, angular):
        if self.avoidance_active or self.current_position is None:
            return linear, angular
        static = getattr(self, 'scenario_static_obstacles_world', [])
        passed = getattr(self, 'avoidance_passed_obstacles_world', [])
        if not static or not passed:
            return linear, angular
        x, y = self.current_position
        body = self.robot_body_radius_m()
        best_cap = None
        for sx, sy, sr in static:
            matched = any(
                math.hypot(px - sx, py - sy) <= sr + pr + 0.30
                for px, py, pr in passed
            )
            if not matched or not self.obstacle_on_active_segment_path(sx, sy, sr):
                continue
            dist = math.hypot(x - sx, y - sy)
            need = sr + body + 0.06
            if dist > need + 0.50:
                continue
            cap = self.avoid_approach_creep_speed_mps * (
                0.32 if dist <= need + 0.10 else 0.32 + 0.55 * min(1.0, max(0.0, 1.0 - (dist - need - 0.10) / 0.40))
            )
            best_cap = cap if best_cap is None else min(best_cap, cap)
        if best_cap is None:
            return linear, angular
        return min(linear, best_cap), angular

    def obstacle_already_passed_in_mission(self, circle=None):
        circle = circle if circle is not None else self.active_obstacle_circle
        if circle is None:
            return False
        world = self.obstacle_center_world_xy(circle)
        if world is None:
            return False
        radius = float(circle.get('radius', 0.12))
        for px, py, pr in getattr(self, 'avoidance_passed_obstacles_world', []):
            if math.hypot(world[0] - px, world[1] - py) <= radius + pr + 0.20:
                return True
        static = getattr(self, 'scenario_static_obstacles_world', [])
        for sx, sy, sr in static:
            if math.hypot(world[0] - sx, world[1] - sy) > radius + sr + 0.24:
                continue
            for px, py, pr in getattr(self, 'avoidance_passed_obstacles_world', []):
                if math.hypot(px - sx, py - sy) <= sr + pr + 0.30:
                    return True
        return False

    def record_passed_obstacle_world(self):
        world = getattr(self, 'locked_obstacle_world_xy', None)
        if world is None:
            circle = self.locked_obstacle_circle
            if circle is None:
                return
            world = self.obstacle_center_world_xy(circle)
        if world is None:
            return
        radius = float((self.locked_obstacle_circle or {}).get('radius', 0.12))
        self.avoidance_passed_obstacles_world.append(
            (float(world[0]), float(world[1]), radius)
        )

    def update_scan_obstacle_cloud_robot(self, scan_msg):
        points = []
        for index, distance in enumerate(scan_msg.ranges):
            if (
                math.isnan(distance)
                or math.isinf(distance)
                or distance < self.obstacle_circle_min_range_m
                or distance > self.obstacle_circle_max_range_m
            ):
                continue
            angle_deg = math.degrees(scan_msg.angle_min + index * scan_msg.angle_increment)
            angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
            if abs(angle_deg) > 85.0:
                continue
            angle_rad = math.radians(angle_deg)
            points.append((distance * math.cos(angle_rad), distance * math.sin(angle_rad)))
        self.scan_obstacle_points_robot = points

    def maybe_log_template_approach_cap(self, now_sec, cap_mps, segment_speed):
        if hasattr(self, 'log_template_mission_cap'):
            self.log_template_mission_cap(cap_mps, segment_speed)

    # ------------------------------------------------------------------ FSM API
    def avoidance_enter(self, now_sec):
        self.avoidance_active = True
        self.avoidance_started_at = now_sec
        self.avoidance_entry_progress = self.projected_distance()
        nearest = self.template_blocker_distance_m()
        self.avoidance_entry_nearest_m = (
            float(nearest) if math.isfinite(nearest) else 99.0
        )
        self.avoidance_clear_streak = 0
        self.avoidance_obstacle_passed = False
        self.lock_avoidance_geometry()
        self.locked_bypass_side_at_enter = self.locked_bypass_side
        self._last_avoid_cmd_log_at = -1.0
        self.write_debug_log(
            'DECISION',
            (
                f'AVOID_ENTER segment={self.current_segment.get("description", "?")} '
                f'algorithm=goal_direct nearest={self.format_distance(nearest)}m '
                f'side={self.locked_bypass_side:+d} offset={self.avoid_target_offset_m:.2f}m '
                f'| {self.format_goal_compact()} '
                f'| {self.format_template_avoid_compact()} '
                f'| {self.format_scan_ranges_compact()}'
            ),
        )

    def avoidance_exit(self, reason):
        direct_cut = getattr(self, 'need_direct_cut', False)
        if reason in ('segment_complete', 'recovered', 'direct_cut', 'corner_shortcut') and getattr(
            self, 'locked_obstacle_world_xy', None
        ) is not None:
            self.record_passed_obstacle_world()
            now_sec = self.control_now_sec() if hasattr(self, 'control_now_sec') else 0.0
            self.detour_cooldown_until = now_sec + float(getattr(self, 'detour_cooldown_sec', 2.0))
            if hasattr(self, 'segment_started_at') and self.segment_started_at is not None:
                self.segment_started_at = now_sec
        self.write_debug_log(
            'DECISION',
            (
                f'AVOID_EXIT reason={reason} mode={self.avoidance_phase} '
                f'side={self.locked_bypass_side:+d} progress={self.projected_distance():.2f}m '
                f'lat={self.segment_lateral_offset_m():.2f}m cut={direct_cut} '
                f'| {self.format_goal_compact()} '
                f'| {self.format_template_avoid_compact()} '
                f'| {self.format_scan_ranges_compact()}'
            ),
        )
        self.reset_avoidance_runtime()

    def avoidance_step(self, now_sec):
        if not self.avoidance_can_run():
            if self.avoidance_active:
                self.avoidance_exit('segment_no_avoid')
            return False

        if not self.avoidance_active:
            if not self.avoidance_should_enter():
                return False
            self.avoidance_enter(now_sec)

        if not self.update_locked_obstacle_tracking(now_sec):
            self.publish_state('avoid_hold')
            self.publish_cmd_vel(0.0, 0.0)
            self.write_debug_log('CMD', f'GOAL_HOLD perception_lost | {self.format_template_avoid_compact()}')
            return True

        if self.recover_track_complete() or self.obstacle_passed_for_handoff(margin_m=0.08):
            self.avoidance_clear_streak += 1
        else:
            self.avoidance_clear_streak = 0

        if self.goal_direct_handoff_ready():
            direct_cut = getattr(self, 'need_direct_cut', False)
            if direct_cut:
                self.prepare_direct_cut_corner_handoff()
            if direct_cut and self.next_plan_segment_is_turn():
                self._corner_shortcut_turn_target = self.upcoming_corner_target_yaw()
            exit_reason = 'direct_cut' if direct_cut else 'segment_complete'
            self.avoidance_exit(exit_reason)
            if direct_cut and self.next_plan_segment_is_turn():
                self.start_segment(self.plan_index + 1)
            return True

        if self.avoidance_stuck_handoff_ready(now_sec):
            direct_cut = getattr(self, 'need_direct_cut', False)
            if direct_cut:
                self.prepare_direct_cut_corner_handoff()
            if direct_cut and self.next_plan_segment_is_turn():
                self._corner_shortcut_turn_target = self.upcoming_corner_target_yaw()
            self.avoidance_exit('segment_complete')
            if direct_cut and self.next_plan_segment_is_turn():
                self.start_segment(self.plan_index + 1)
            return True

        self.maybe_log_goal_direct_snapshot(now_sec)
        linear, angular = self.goal_direct_cmd(now_sec)
        self.publish_goal_direct_cmd(linear, angular, now_sec)
        return True

    def dwa_path_blocker_imminent(self):
        if not self.scenario_obstacle_applies_to_current_segment():
            return False
        scenario_circle = self.scenario_static_obstacle_circle()
        if scenario_circle is not None:
            nearest = float(
                scenario_circle.get('nearest_distance', scenario_circle.get('closest_x', 99.0))
            )
            if nearest <= self.detour_obstacle_detect_distance:
                return True
        circle = self.active_obstacle_circle
        if circle is None:
            return False
        if not self.circle_counts_for_detour_trigger(circle):
            return False
        nearest = float(circle.get('nearest_distance', circle.get('closest_x', float('inf'))))
        return math.isfinite(nearest) and nearest <= self.detour_obstacle_detect_distance

    def corridor_blocker_imminent(self):
        """Debug-log compatibility alias."""
        return self.dwa_path_blocker_imminent()

    def corridor_handoff_ready(self):
        """Debug-log compatibility alias."""
        return self.goal_direct_handoff_ready()

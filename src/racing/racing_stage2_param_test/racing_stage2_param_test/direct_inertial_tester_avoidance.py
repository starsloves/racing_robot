"""Goal-direct avoidance — few waypoints + Pure Pursuit style go-to-point.

On enter: plan bypass → pass → (rejoin | exit) world goals on the driven chord.
Control steers toward the active goal with regulated speed (near-goal / clearance).
Short segments or insufficient room to rejoin → direct cut, then corner_align on driven chord.
"""

import math

from .ring_track import (
    driven_segment_endpoints,
    driven_segment_length_m,
    point_on_driven_segment,
    preferred_bypass_side_for_segment,
    progress_on_driven_segment_m,
    segment_end_goal_world,
    signed_cross_track_on_driven_segment,
)


class DirectInertialTesterAvoidanceMixin:
    """Unified goal_direct detour for straight move segments."""

    GOAL_PHASES = ('bypass', 'pass', 'rejoin', 'exit', 'corner_align')

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
        self.goal_corner_align_xy = None
        self.need_direct_cut = False

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

    # ------------------------------------------------------------------ geometry
    def segment_lateral_offset_m(self):
        if self.current_position is None:
            return 0.0
        segment_name = (self.current_segment or {}).get('description', '')
        if segment_name:
            cross = signed_cross_track_on_driven_segment(
                self.current_position,
                segment_name,
                getattr(self, 'test_direction', 'clockwise'),
                self.rectangle_first_leg_m,
                self.rectangle_side_leg_m,
                self.rectangle_top_leg_m,
            )
            if cross is not None:
                return cross
        if self.segment_start_pose is None or self.segment_heading is None:
            return 0.0
        dx = self.current_position[0] - self.segment_start_pose[0]
        dy = self.current_position[1] - self.segment_start_pose[1]
        heading = self.segment_heading
        return -math.sin(heading) * dx + math.cos(heading) * dy

    def locked_obstacle_world_xy_for_bypass(self):
        world = getattr(self, 'locked_obstacle_world_xy', None)
        if world is not None:
            return float(world[0]), float(world[1])
        circle = self.active_obstacle_circle
        if circle is not None:
            return self.obstacle_center_world_xy(circle)
        return None

    def bypass_side_from_driven_line_obstacle(self, required_lat):
        """锥桶相对实测直行线的左右：与 segment 名无关的通用规则。"""
        segment_name = (self.current_segment or {}).get('description', '')
        world = self.locked_obstacle_world_xy_for_bypass()
        if not segment_name or world is None:
            return None, False
        cross = signed_cross_track_on_driven_segment(
            world,
            segment_name,
            getattr(self, 'test_direction', 'clockwise'),
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )
        if cross is None:
            return None, False
        if abs(cross) < 0.03:
            return None, False
        bypass_side = 1 if cross < 0.0 else -1
        shift = self.effective_side_clearance_m(float(-bypass_side))
        return bypass_side, shift >= required_lat * 0.65

    def progress_along_segment_m(self, world_xy):
        if hasattr(self, 'nominal_segment_progress_m'):
            progress = self.nominal_segment_progress_m(world_xy)
            if progress is not None:
                return progress
        if self.segment_start_pose is None or self.segment_heading is None:
            return None
        dx = float(world_xy[0]) - float(self.segment_start_pose[0])
        dy = float(world_xy[1]) - float(self.segment_start_pose[1])
        heading = self.segment_heading
        return math.cos(heading) * dx + math.sin(heading) * dy

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
            offset = min(cap, max(offset, 0.33))
        return offset

    def select_bypass_side(self, required_lat):
        """一次锁定绕边：横偏明显走对侧；正中优先 preferred；否则比 clearance。"""
        segment_name = (self.current_segment or {}).get('description', '')
        preferred = preferred_bypass_side_for_segment(
            segment_name,
            getattr(self, 'test_direction', 'clockwise'),
        )

        world = None
        static = getattr(self, 'scenario_static_obstacles_world', [])
        if static:
            world = (float(static[0][0]), float(static[0][1]))
        elif self.active_obstacle_circle is not None:
            world = self.obstacle_center_world_xy(self.active_obstacle_circle)
        cross = None
        if segment_name and world is not None:
            cross = signed_cross_track_on_driven_segment(
                world,
                segment_name,
                getattr(self, 'test_direction', 'clockwise'),
                self.rectangle_first_leg_m,
                self.rectangle_side_leg_m,
                self.rectangle_top_leg_m,
            )
        if cross is not None and abs(cross) < 0.03 and preferred in (-1, 1):
            shift = self.effective_side_clearance_m(float(-preferred))
            if shift >= required_lat * 0.60:
                return preferred, True

        driven_side, driven_ok = self.bypass_side_from_driven_line_obstacle(required_lat)
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
        segment_name = (self.current_segment or {}).get('description', '')
        world = self.locked_obstacle_world_xy_for_bypass()
        if segment_name and world is not None:
            along = progress_on_driven_segment_m(
                world,
                segment_name,
                getattr(self, 'test_direction', 'clockwise'),
                self.rectangle_first_leg_m,
                self.rectangle_side_leg_m,
                self.rectangle_top_leg_m,
            )
            if along is not None:
                if hasattr(self, 'plan_progress_from_driven_progress_m'):
                    scaled = self.plan_progress_from_driven_progress_m(along, segment_name)
                    if scaled is not None:
                        along = scaled
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

    def driven_progress_to_world(self, along_m, lateral_m=0.0):
        segment_name = (self.current_segment or {}).get('description', '')
        if not segment_name:
            return None
        return point_on_driven_segment(
            segment_name,
            along_m,
            lateral_m,
            getattr(self, 'test_direction', 'clockwise'),
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )

    def plan_avoidance_goals(self):
        """入障时算 bypass / pass / rejoin|exit 世界坐标与 need_direct_cut。"""
        segment_name = (self.current_segment or {}).get('description', '')
        world = self.locked_obstacle_world_xy_for_bypass()
        side = int(self.locked_bypass_side or 1)
        offset = float(self.avoid_target_offset_m)
        tol = float(self.distance_tolerance)
        seg_len_plan = float((self.current_segment or {}).get('distance_m', 0.0))
        s_end = max(0.0, seg_len_plan - tol)

        driven_len = driven_segment_length_m(
            segment_name,
            getattr(self, 'test_direction', 'clockwise'),
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )
        if driven_len is None or driven_len < 1e-3:
            driven_len = seg_len_plan

        s_obs_plan = getattr(self, 'locked_obstacle_along_s', None)
        if s_obs_plan is None:
            s_obs_plan = seg_len_plan * 0.5
        s_obs_plan = float(s_obs_plan)

        if world is not None and segment_name:
            driven_s = progress_on_driven_segment_m(
                world,
                segment_name,
                getattr(self, 'test_direction', 'clockwise'),
                self.rectangle_first_leg_m,
                self.rectangle_side_leg_m,
                self.rectangle_top_leg_m,
            )
        else:
            driven_s = s_obs_plan * (driven_len / max(seg_len_plan, 1e-3))

        if driven_s is None:
            driven_s = s_obs_plan * (driven_len / max(seg_len_plan, 1e-3))

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
        bypass = self.driven_progress_to_world(driven_s, lateral)
        if bypass is None and world is not None:
            bypass = (float(world[0]), float(world[1]))

        pass_margin = float(getattr(self, 'avoid_goal_pass_margin_m', 0.12))
        pass_driven = min(driven_len, driven_s + radius + pass_margin)
        pass_goal = self.driven_progress_to_world(pass_driven, lateral * 0.85)

        inward = float(getattr(self, 'avoid_goal_exit_inward_margin_m', 0.17))
        inward += float(self.obstacle_corridor_body_half_width_m) * 0.5
        exit_goal = segment_end_goal_world(
            segment_name,
            getattr(self, 'test_direction', 'clockwise'),
            inward,
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )
        if exit_goal is None:
            exit_goal = self.driven_progress_to_world(driven_len, 0.0)

        corner_align_goal = self.driven_progress_to_world(driven_len, 0.0)
        if corner_align_goal is None:
            corner_align_goal = exit_goal

        rejoin_goal = None
        if not need_direct_cut:
            s_mirror_plan = min(s_obs_plan + half_span, s_end)
            s_mirror_driven = s_mirror_plan * (driven_len / max(seg_len_plan, 1e-3))
            rejoin_goal = self.driven_progress_to_world(s_mirror_driven, 0.0)

        self.goal_bypass_xy = bypass
        self.goal_pass_xy = pass_goal
        self.goal_rejoin_xy = rejoin_goal
        self.goal_exit_xy = exit_goal
        self.goal_corner_align_xy = corner_align_goal

        if need_direct_cut:
            self.goal_direct_sequence = ['bypass', 'pass', 'corner_align']
        else:
            self.goal_direct_sequence = ['bypass', 'pass', 'rejoin']
        self.goal_direct_index = 0
        self.goal_direct_phase = self.goal_direct_sequence[0]

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

    def direct_cut_progress_on_chord_m(self):
        """沿实测弦线的里程（与 handoff / corner_align 一致，不受弯后锚点干扰）。"""
        seg_name = (self.current_segment or {}).get('description', '')
        if not seg_name or self.current_position is None:
            return None, None
        along = progress_on_driven_segment_m(
            self.current_position,
            seg_name,
            getattr(self, 'test_direction', 'clockwise'),
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )
        driven_len = driven_segment_length_m(
            seg_name,
            getattr(self, 'test_direction', 'clockwise'),
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )
        return along, driven_len

    def prepare_direct_cut_corner_handoff(self):
        """direct_cut 进拐角：记下需收束的弦线，下一段 move 里程从当前投影零点起算。"""
        seg_name = (self.current_segment or {}).get('description', '')
        if seg_name and abs(self.segment_lateral_offset_m()) > self.direct_cut_lateral_handoff_limit_m():
            self._pending_lateral_trim_chord = seg_name
        else:
            self._pending_lateral_trim_chord = None
        self._corner_shortcut_move_progress_reset = True

    def consume_pending_lateral_trim_cmd(self):
        """转弯段内先把上一直行弦线的横偏收到位，再对准 target_yaw。"""
        chord = getattr(self, '_pending_lateral_trim_chord', None)
        if not chord or self.current_position is None:
            return None
        cross = signed_cross_track_on_driven_segment(
            self.current_position,
            chord,
            getattr(self, 'test_direction', 'clockwise'),
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )
        if cross is None:
            self._pending_lateral_trim_chord = None
            return None
        limit = max(0.07, self.avoid_rejoin_lateral_tol_m * 1.25)
        if abs(cross) <= limit:
            self._pending_lateral_trim_chord = None
            return None
        omega = self.clamp(3.5 * (-cross), self.max_angular_speed)
        return 0.0, omega

    def active_avoidance_goal_xy(self):
        phase = getattr(self, 'goal_direct_phase', '')
        if phase == 'bypass':
            rolling = self.rolling_bypass_goal_xy()
            if rolling is not None:
                return rolling
            return getattr(self, 'goal_bypass_xy', None)
        if phase == 'pass':
            return getattr(self, 'goal_pass_xy', None)
        if phase == 'rejoin':
            return getattr(self, 'goal_rejoin_xy', None)
        if phase == 'exit':
            return getattr(self, 'goal_exit_xy', None)
        if phase == 'corner_align':
            return getattr(self, 'goal_corner_align_xy', None)
        return None

    def bypass_lateral_build_fraction(self):
        target_lat = float(self.locked_bypass_side or 1) * float(self.avoid_target_offset_m)
        return self._clamp01(abs(self.segment_lateral_offset_m()) / max(abs(target_lat), 0.08))

    def rolling_bypass_goal_xy(self):
        """过桶前：沿程只比当前略前 + 全横偏，先侧移不顶锥桶（短边/斜切）。"""
        segment_name = (self.current_segment or {}).get('description', '')
        s_obs = getattr(self, 'locked_obstacle_along_s', None)
        if not segment_name or s_obs is None:
            return None
        seg_len_plan = float((self.current_segment or {}).get('distance_m', 0.0))
        cut_len = float(getattr(self, 'avoid_goal_cut_segment_len_m', 0.68))
        if not getattr(self, 'need_direct_cut', False) and seg_len_plan >= cut_len:
            return None
        progress = self.projected_distance()
        if progress + 0.04 >= float(s_obs):
            return None
        seg_len_plan = float((self.current_segment or {}).get('distance_m', 0.0))
        driven_len = driven_segment_length_m(
            segment_name,
            getattr(self, 'test_direction', 'clockwise'),
            self.rectangle_first_leg_m,
            self.rectangle_side_leg_m,
            self.rectangle_top_leg_m,
        )
        if driven_len is None or seg_len_plan < 1e-3:
            return None
        scale = driven_len / seg_len_plan
        world = self.locked_obstacle_world_xy_for_bypass()
        driven_s = (
            progress_on_driven_segment_m(
                world,
                segment_name,
                getattr(self, 'test_direction', 'clockwise'),
                self.rectangle_first_leg_m,
                self.rectangle_side_leg_m,
                self.rectangle_top_leg_m,
            )
            if world is not None
            else float(s_obs) * scale
        )
        if driven_s is None:
            driven_s = float(s_obs) * scale
        radius = float((self.locked_obstacle_circle or {}).get('radius', 0.12))
        progress_driven = progress * scale
        lat_build = self.bypass_lateral_build_fraction()
        ahead = max(0.03, min(0.07, 0.03 + 0.06 * lat_build))
        anchor = min(float(driven_s) - max(0.03, radius * 0.5), progress_driven + ahead)
        lateral = float(self.locked_bypass_side or 1) * float(self.avoid_target_offset_m)
        return self.driven_progress_to_world(max(0.0, anchor), lateral)

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
            ):
                self.advance_goal_direct_phase()
            return
        if phase in ('rejoin', 'exit') and dist <= tol:
            return
        if phase == 'corner_align':
            return

    def goal_direct_corner_align_cmd(self):
        """段末/拐角：先朝弦线末点 corner 目标收束，横偏足够小再对准下一段转弯。"""
        if self.segment_heading is None or self.current_yaw is None or self.current_position is None:
            return 0.0, 0.0

        lat_err = self.segment_lateral_offset_m()
        cross = abs(lat_err)
        lat_tol = self.avoid_rejoin_lateral_tol_m
        along, driven_len = self.direct_cut_progress_on_chord_m()
        target = float((self.current_segment or {}).get('distance_m', 0.0))
        if along is not None and driven_len and driven_len > 1e-3:
            remain = max(0.0, float(driven_len) - float(along))
            near_end = (
                float(along) >= float(driven_len) - 0.12
                and cross <= max(lat_tol * 1.5, self.direct_cut_lateral_handoff_limit_m())
            )
        else:
            progress = self.projected_distance()
            remain = max(0.0, target - progress)
            near_end = remain <= max(0.14, target * 0.22) and cross <= lat_tol * 1.5

        turn_yaw = self.upcoming_corner_target_yaw()
        goal = getattr(self, 'goal_corner_align_xy', None)
        x, y = float(self.current_position[0]), float(self.current_position[1])
        if goal is not None and cross > lat_tol * 0.85:
            gx, gy = float(goal[0]), float(goal[1])
            heading_cmd = math.atan2(gy - y, gx - x)
        elif turn_yaw is not None and near_end:
            heading_cmd = turn_yaw
        else:
            heading_cmd = self.segment_heading
        heading_err = self.angle_error(heading_cmd, self.current_yaw)

        kp = float(getattr(self, 'avoid_goal_heading_kp', self.heading_kp)) or self.heading_kp
        lat_gain = 4.0 if cross > lat_tol else 2.8
        omega = self.clamp(
            lat_gain * (-lat_err) + 1.1 * kp * heading_err,
            self.avoid_max_angular_speed,
        )

        v_max = min(
            float(self.avoid_corner_speed_mps),
            float((self.current_segment or {}).get('speed', self.ring_linear_speed)) * 0.85,
        )
        along_scale = self._clamp01(remain / max(0.10, (driven_len or target) * 0.25))
        curve_scale = self._clamp01(1.0 - abs(heading_err) / math.radians(42.0))
        clear = self.goal_clearance_gate()
        if goal is not None and cross > lat_tol:
            dist_goal = math.hypot(float(goal[0]) - x, float(goal[1]) - y)
            creep = max(self.avoid_approach_creep_speed_mps * 0.65, v_max * 0.28)
            linear = min(creep, v_max * 0.42) * clear * max(0.45, curve_scale)
        else:
            linear = v_max * max(0.32, along_scale) * max(0.42, curve_scale) * clear
            if cross > lat_tol:
                linear = min(linear, v_max * 0.45)

        if not self.avoid_template_feasible:
            linear = 0.0
            omega = self.clamp(kp * 0.5 * heading_err, self.avoid_max_angular_speed * 0.5)
        elif not self.goal_motion_allows_cmd(linear, omega):
            linear = min(linear, self.avoid_approach_creep_speed_mps * 0.55)
            self.avoid_forbidden_linear_block = linear < 0.02
        else:
            self.avoid_forbidden_linear_block = False

        return max(0.0, linear), omega

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
        return max(gate, 0.35) if getattr(self, 'avoidance_active', False) else gate

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
        if not self.obstacle_passed_for_handoff(margin_m=0.08):
            return False
        phase = getattr(self, 'goal_direct_phase', '')
        if phase not in ('rejoin', 'exit', 'corner_align'):
            return False
        dist = self.distance_to_active_goal_m()
        tol = self.goal_reach_tolerance_m()
        if dist > tol * 1.35:
            return False
        if getattr(self, 'need_direct_cut', False):
            seg_tol = float(self.distance_tolerance)
            lat_lim = self.direct_cut_lateral_handoff_limit_m()
            along, driven_len = self.direct_cut_progress_on_chord_m()
            if along is not None and driven_len and driven_len > 1e-3:
                if float(along) + seg_tol < float(driven_len) * 0.88:
                    return False
                chord_end = float(along) + seg_tol >= float(driven_len)
            else:
                progress = self.projected_distance()
                target = float((self.current_segment or {}).get('distance_m', 0.0))
                if progress + seg_tol < target * 0.88:
                    return False
                chord_end = progress + seg_tol >= target
            if not chord_end:
                return False
            if abs(self.segment_lateral_offset_m()) > lat_lim:
                return False
            turn_yaw = self.upcoming_corner_target_yaw()
            if turn_yaw is not None and self.current_yaw is not None:
                heading_err = abs(self.angle_error(turn_yaw, self.current_yaw))
                if heading_err <= max(self.avoid_rejoin_heading_tol * 2.0, math.radians(12.0)):
                    return True
            return self.obstacle_passed_for_handoff()
        if self.recover_track_complete():
            return self.avoidance_clear_streak >= 1
        relaxed_lat = max(
            self.avoid_rejoin_lateral_tol_m * 2.0,
            float(self.avoid_target_offset_m) * 0.35,
        )
        if abs(self.segment_lateral_offset_m()) <= relaxed_lat:
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
        return (
            f'mode=goal_direct phase={self.avoidance_phase} goal={goal_text} '
            f'travel={metrics.get("travel_distance", 0.0):.2f}m '
            f'progress={self.projected_distance():.2f}m '
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
        elif phase in ('exit', 'corner_align') and getattr(self, 'need_direct_cut', False):
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
        if getattr(self, 'goal_direct_phase', '') == 'corner_align':
            return self.goal_direct_corner_align_cmd()

        goal = self.active_avoidance_goal_xy()
        if goal is None:
            return 0.0, 0.0

        gx, gy = float(goal[0]), float(goal[1])
        x, y = float(self.current_position[0]), float(self.current_position[1])
        dist = math.hypot(gx - x, gy - y)
        heading_cmd = math.atan2(gy - y, gx - x)
        heading_err = self.angle_error(heading_cmd, self.current_yaw)

        kp = float(getattr(self, 'avoid_goal_heading_kp', self.heading_kp))
        phase = getattr(self, 'goal_direct_phase', 'bypass')
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
        if phase not in ('rejoin', 'exit', 'corner_align'):
            return False
        elapsed = self.phase_elapsed_sec(now_sec)
        if elapsed >= 10.0 and self.distance_to_active_goal_m() <= self.goal_reach_tolerance_m() * 2.5:
            return True
        segment = self.current_segment or {}
        target = float(segment.get('distance_m', 0.0))
        progress = self.projected_distance()
        tol = float(self.distance_tolerance)
        if progress + tol >= target and elapsed >= 3.0 and phase in ('exit', 'corner_align'):
            lat_lim = self.direct_cut_lateral_handoff_limit_m()
            if abs(self.segment_lateral_offset_m()) <= lat_lim:
                return True
            return elapsed >= 22.0
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
            f'corner={fmt(self.goal_corner_align_xy)}'
        )

    # ------------------------------------------------------------------ mission helpers
    def mission_nominal_move_cmd(self, linear_mps):
        if self.segment_heading is None or self.current_yaw is None:
            return float(linear_mps), 0.0
        lateral_error = self.segment_lateral_offset_m()
        heading_error = self.angle_error(self.segment_heading, self.current_yaw)
        lat_gain = 2.4
        lat_tol = max(0.05, self.avoid_rejoin_lateral_tol_m * 1.2)
        abs_lat = abs(lateral_error)
        if abs_lat > lat_tol:
            angular = lat_gain * (-lateral_error)
        else:
            angular = lat_gain * (-lateral_error) + self.heading_kp * heading_error
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
        if self.segment_start_pose is None or self.segment_heading is None:
            return False
        along = self.progress_along_segment_m((world_x, world_y))
        if along is None:
            return False
        length = float(self.current_segment.get('distance_m', 0.0))
        if along < -along_margin or along > length + along_margin:
            return False
        heading = self.segment_heading
        sx, sy = self.segment_start_pose
        lateral = -math.sin(heading) * (float(world_x) - sx) + math.cos(heading) * (float(world_y) - sy)
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

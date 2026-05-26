"""Structured detour/path debug logging for direct_inertial_tester."""

import math


class DirectInertialTesterDebugLogMixin:
    """Compact, decision-focused debug log for obstacle detours."""

    def write_debug_log(self, level, message):
        try:
            stamp = self.debug_log_timestamp()
            with open(self.debug_log_path, 'a', encoding='utf-8') as log_file:
                log_file.write(f'[{level}][{stamp:.3f}] {message}\n')
                log_file.flush()
        except OSError as exc:
            self.get_logger().warning(f'调试日志文件写入失败: {exc}')

    def init_detour_debug_log_state(self):
        self.last_detour_follow_log_time = -1.0
        self.last_obstacle_trigger_log_time = -1.0
        self.last_obstacle_suppress_log_time = -1.0
        self.last_plan_fail_log_at = -1.0
        self.last_plan_fail_reason = ''
        self._obstacle_was_active = False
        self._last_follow_mode = None
        self._last_finish_gate_signature = None
        self._last_detour_cmd_log_at = -1.0
        self._last_template_phase = None
        self._last_template_detour_log_at = -1.0
        self._last_template_approach_log_at = -1.0
        self._in_approach_envelope_logged = False
        self._last_template_gate_signature = None
        self._last_template_abort_log_at = -1.0
        self._last_mission_move_log_at = -1.0
        self._last_mission_move_phase = None
        self._last_mission_move_angular = None

    def maybe_log_detour_cmd_throttled(
        self,
        now_sec,
        follow_mode,
        linear,
        angular,
        target_point,
        target_local,
        metrics,
    ):
        if now_sec - getattr(self, '_last_detour_cmd_log_at', -1.0) < 0.25:
            return
        self._last_detour_cmd_log_at = now_sec
        target_text = 'none'
        if target_local is not None:
            target_text = f'({target_local[0]:.2f},{target_local[1]:.2f})'
        self.write_debug_log(
            'CMD',
            (
                f'follow={follow_mode} cmd linear={linear:.3f} angular={angular:.3f} '
                f'target_local={target_text} nearest='
                f'{self.format_distance(metrics.get("nearest_obstacle_m"))}m | '
                f'travel={metrics.get("travel_distance", 0.0):.2f}/'
                f'{metrics.get("min_travel_before_finish", 0.0):.2f}m '
                f'path_progress={metrics.get("locked_path_progress", 0.0):.2f}'
            ),
        )

    def debug_log_timestamp(self):
        if hasattr(self, 'control_now_sec'):
            return self.control_now_sec()
        return self.get_clock().now().nanoseconds / 1e9

    def format_xy_point(self, point, prefix=''):
        if point is None:
            return f'{prefix}(nan,nan)'
        return f'{prefix}({float(point[0]):.2f},{float(point[1]):.2f})'

    def circle_center_in_world(self, circle):
        if circle is None:
            return None
        return self.robot_local_to_world_point((float(circle['center_x']), float(circle['center_y'])))

    def format_xy_polyline(self, points, max_points=12):
        if not points:
            return '[]'
        shown = points[:max_points]
        segments = [f'({float(p[0]):.2f},{float(p[1]):.2f})' for p in shown]
        text = ' -> '.join(segments)
        if len(points) > max_points:
            text = f'{text} -> ...+{len(points) - max_points}'
        return f'[{text}]'

    def format_active_circle_compact(self, circle=None):
        circle = self.active_obstacle_circle if circle is None else circle
        if circle is None:
            return 'active_circle=none'
        block = self.blocking_circle_robot_metrics()
        block_text = 'block=none'
        if block is not None:
            block_text = (
                f"block_robot farthest_x={block['farthest_x']:.2f}m "
                f"closest_x={block['closest_x']:.2f}m"
            )
        return (
            f"active_circle robot=({circle['center_x']:.2f},{circle['center_y']:.2f}) "
            f"closest_x={circle['closest_x']:.2f}m farthest_x={circle.get('farthest_x', 0.0):.2f}m "
            f"r={circle['radius']:.2f}m | {block_text}"
        )

    def format_scan_ranges_compact(self):
        return (
            f"front={self.format_distance(self.front_obstacle_distance)}m"
            f"@{self.front_obstacle_angle_deg:.1f}deg "
            f"left={self.format_distance(self.left_clearance_distance)}m "
            f"right={self.format_distance(self.right_clearance_distance)}m "
            f"nearest={self.format_distance(self.detour_nearest_obstacle_distance_m())}m"
        )

    def format_detour_state_compact(self):
        phase = getattr(self, 'avoidance_phase', 'follow')
        return (
            f'avoid_active={getattr(self, "avoidance_active", False)} '
            f'phase={phase} '
            f'locked_side={getattr(self, "locked_bypass_side", 0):+d} '
            f'front={self.format_distance(self.front_obstacle_distance)}m '
            f'clear_streak={getattr(self, "avoidance_clear_streak", 0)} '
            f'progress={self.projected_distance():.2f}m'
        )

    def format_template_avoid_compact(self):
        circle = (
            self.tracking_obstacle_robot_frame()
            if hasattr(self, 'tracking_obstacle_robot_frame')
            else self.active_obstacle_circle
        )
        track_farthest = 'nan'
        if circle is not None:
            track_farthest = f"{float(circle.get('farthest_x', circle.get('closest_x', 0.0))):.2f}"
        locked_side = getattr(self, 'locked_bypass_side', 0)
        lat_target = (
            self.corridor_lateral_target_m()
            if hasattr(self, 'corridor_lateral_target_m')
            else 0.0
        )
        zone = (
            self.robot_corner_zone_reason()
            if hasattr(self, 'robot_corner_zone_reason')
            else getattr(self, 'avoid_corner_zone_reason', '')
        )
        return (
            f'{self.format_detour_state_compact()} '
            f'zone={zone or "none"} '
            f'lat_target={lat_target:.2f}m '
            f'offset={getattr(self, "avoid_target_offset_m", 0.0):.2f}m '
            f'lat={self.segment_lateral_offset_m():.2f}m '
            f'signed_lat={(locked_side or 1) * self.segment_lateral_offset_m():.2f}m '
            f'track_farthest_x={track_farthest}m '
            f'pass_margin={getattr(self, "avoid_parallel_front_margin_m", 0.18):.2f}m '
            f'passed={self.obstacle_passed_for_handoff() if hasattr(self, "obstacle_passed_for_handoff") else False} '
            f'recover={self.recover_track_complete() if hasattr(self, "recover_track_complete") else False} '
            f'phase_elapsed={self.phase_elapsed_sec(self.debug_log_timestamp()) if hasattr(self, "phase_elapsed_sec") else 0.0:.1f}s'
        )

    def format_template_side_clearances(self):
        required = getattr(self, 'avoid_target_offset_m', 0.28)
        left_clear = self.effective_side_clearance_m(1.0) if hasattr(self, 'effective_side_clearance_m') else 0.0
        right_clear = self.effective_side_clearance_m(-1.0) if hasattr(self, 'effective_side_clearance_m') else 0.0
        return (
            f'clear_left={left_clear:.2f}m clear_right={right_clear:.2f}m req={required:.2f}m'
        )

    def evaluate_template_phase_gates(self):
        segment = self.current_segment or {}
        target = float(segment.get('distance_m', 0.0))
        progress = self.projected_distance()
        tol = float(getattr(self, 'distance_tolerance', 0.05))
        overshoot = (
            self.corridor_progress_overshoot_cap_m()
            if hasattr(self, 'corridor_progress_overshoot_cap_m')
            else 0.12
        )
        heading_err = 0.0
        if self.segment_heading is not None and self.current_yaw is not None:
            heading_err = abs(self.angle_error(self.segment_heading, self.current_yaw))
        lat = self.segment_lateral_offset_m()
        nearest = (
            self.template_blocker_distance_m()
            if hasattr(self, 'template_blocker_distance_m')
            else float('inf')
        )
        gates = {
            'blocker_imminent': (
                self.corridor_blocker_imminent()
                if hasattr(self, 'corridor_blocker_imminent')
                else False
            ),
            'within_watch': math.isfinite(nearest) and nearest <= getattr(self, 'avoid_watch_distance_m', 0.45),
            'obstacle_passed': (
                self.obstacle_passed_for_handoff()
                if hasattr(self, 'obstacle_passed_for_handoff')
                else False
            ),
            'recover_heading': heading_err <= getattr(self, 'avoid_rejoin_heading_tol', math.radians(6.0)),
            'recover_lateral': abs(lat) <= getattr(self, 'avoid_rejoin_lateral_tol_m', 0.06),
            'progress_ok': progress >= target - tol,
            'progress_not_overshoot': progress <= target + overshoot,
            'clear_streak': getattr(self, 'avoidance_clear_streak', 0) >= 2,
            'handoff_ready': (
                self.corridor_handoff_ready()
                if hasattr(self, 'corridor_handoff_ready')
                else False
            ),
            'perception_ok': (
                self.locked_obstacle_circle is None
                or self.locked_circle_still_visible()
                if hasattr(self, 'locked_circle_still_visible')
                else True
            ),
        }
        return gates

    def format_template_phase_gate_report(self):
        gates = self.evaluate_template_phase_gates()
        return '; '.join(f'{name}={"OK" if ok else "NO"}' for name, ok in gates.items())

    def log_template_lock_snapshot(self, tag='LOCK'):
        self.write_debug_log(
            'DECISION',
            (
                f'TEMPLATE_{tag} side={getattr(self, "locked_bypass_side", 0):+d} '
                f'corner={getattr(self, "avoid_use_corner_template", False)} '
                f'zone={getattr(self, "avoid_corner_zone_reason", "") or "none"} '
                f'choice={getattr(self, "avoid_corner_choice", "") or "straight"} '
                f'offset={getattr(self, "avoid_target_offset_m", 0.0):.2f}m '
                f'wide={getattr(self, "avoid_template_wide", False)} '
                f'feasible={getattr(self, "avoid_template_feasible", True)} | '
                f'{self.format_template_side_clearances()} | '
                f'{self.format_active_circle_compact()}'
            ),
        )

    def log_template_phase_transition(self, old_phase, new_phase, reason, now_sec=None):
        if old_phase == new_phase:
            return
        self._last_template_phase = new_phase
        elapsed = 0.0
        if hasattr(self, 'phase_elapsed_sec') and now_sec is not None:
            elapsed = self.phase_elapsed_sec(now_sec)
        self.write_debug_log(
            'DECISION',
            (
                f'TEMPLATE_PHASE {old_phase}->{new_phase} reason={reason} '
                f'elapsed={elapsed:.1f}s | {self.format_template_avoid_compact()}'
            ),
        )
        gate_sig = (new_phase, self.format_template_phase_gate_report())
        if gate_sig != self._last_template_gate_signature:
            self._last_template_gate_signature = gate_sig
            self.write_debug_log('DECISION', f'  gates {self.format_template_phase_gate_report()}')

    def log_template_abort_eval(self, now_sec):
        if now_sec - getattr(self, '_last_template_abort_log_at', -1.0) < 0.40:
            return
        self._last_template_abort_log_at = now_sec
        self.write_debug_log(
            'DECISION',
            (
                f'TEMPLATE_ABORT_EVAL streak={getattr(self, "avoidance_abort_clear_streak", 0)}/'
                f'{getattr(self, "avoid_abort_clear_streak_required", 5)} '
                f'clear_dist={getattr(self, "avoid_abort_clear_distance_m", 0.65):.2f}m | '
                f'{self.format_scan_ranges_compact()} | {self.format_template_phase_gate_report()}'
            ),
        )

    def log_template_hold(self, reason, extra=''):
        msg = f'TEMPLATE_HOLD reason={reason}'
        if extra:
            msg = f'{msg} {extra}'
        msg = f'{msg} | {self.format_template_avoid_compact()}'
        self.write_debug_log('CMD', msg)

    def log_template_mission_cap(self, cap_mps, segment_speed):
        self.write_debug_log(
            'DETOUR',
            (
                f'APPROACH_CREEP cap={cap_mps:.3f}m/s segment_speed={segment_speed:.3f}m/s '
                f'watch={getattr(self, "avoid_watch_distance_m", 0.45):.2f}m '
                f'detect={getattr(self, "detour_obstacle_detect_distance", 1.0):.2f}m | '
                f'{self.format_scan_ranges_compact()}'
            ),
        )

    def maybe_log_template_approach_envelope(self, now_sec):
        in_envelope = (
            hasattr(self, 'avoidance_in_approach_envelope')
            and self.avoidance_in_approach_envelope()
        )
        if in_envelope:
            if not self._in_approach_envelope_logged or now_sec - self._last_template_approach_log_at >= 0.8:
                self._in_approach_envelope_logged = True
                self._last_template_approach_log_at = now_sec
                cap = self.mission_obstacle_linear_cap_mps() if hasattr(self, 'mission_obstacle_linear_cap_mps') else None
                cap_text = f'{cap:.3f}' if cap is not None else 'nan'
                self.write_debug_log(
                    'TRIGGER',
                    (
                        f'进入接近限速区 template_approach cap={cap_text}m/s | '
                        f'{self.format_scan_ranges_compact()} | '
                        f'{self.format_active_circle_compact()}'
                    ),
                )
        elif self._in_approach_envelope_logged:
            self._in_approach_envelope_logged = False
            self.write_debug_log(
                'TRIGGER',
                f'离开接近限速区 | {self.format_scan_ranges_compact()}',
            )

    def maybe_log_template_detour_snapshot(self, now_sec):
        if not getattr(self, 'avoidance_active', False):
            return
        if now_sec - self._last_template_detour_log_at < self.detour_debug_log_period_sec:
            return
        self._last_template_detour_log_at = now_sec
        self.write_debug_log(
            'DETOUR',
            (
                f'{self.format_template_avoid_compact()} | '
                f'{self.format_scan_ranges_compact()} | '
                f'{self.format_template_side_clearances()}'
            ),
        )

    def log_template_cmd(self, phase, linear, angular, now_sec):
        if now_sec - getattr(self, '_last_avoid_cmd_log_at', -1.0) < 0.20:
            return
        self._last_avoid_cmd_log_at = now_sec
        nearest = self.template_blocker_distance_m() if hasattr(self, 'template_blocker_distance_m') else float('inf')
        entry_side = getattr(self, 'locked_bypass_side_at_enter', 0)
        locked_side = getattr(self, 'locked_bypass_side', 0)
        self.write_debug_log(
            'CMD',
            (
                f'TEMPLATE phase={phase} linear={linear:.3f} angular={angular:.3f} '
                f'side={locked_side:+d} side_at_enter={entry_side:+d} '
                f'side_unchanged={locked_side == entry_side} '
                f'corner={getattr(self, "avoid_use_corner_template", False)} '
                f'zone={getattr(self, "avoid_corner_zone_reason", "") or "none"} '
                f'feasible={getattr(self, "avoid_template_feasible", True)} '
                f'wide={getattr(self, "avoid_template_wide", False)} '
                f'forbidden_fwd={getattr(self, "avoid_forbidden_linear_block", False)} '
                f'offset={getattr(self, "avoid_target_offset_m", 0.0):.2f}m '
                f'nearest={self.format_distance(nearest)}m '
                f'lat={self.segment_lateral_offset_m():.2f}m '
                f'choice={getattr(self, "avoid_corner_choice", "") or "straight"} | '
                f'gates={self.format_template_phase_gate_report()}'
            ),
        )

    def evaluate_finish_gates(self, metrics=None):
        metrics = metrics or self.local_replan_status_metrics()
        segment = self.current_segment or {}
        target = float(segment.get('distance_m', 0.0))
        progress = self.projected_distance()
        tol = float(getattr(self, 'distance_tolerance', 0.05))
        overshoot = (
            self.corridor_progress_overshoot_cap_m()
            if hasattr(self, 'corridor_progress_overshoot_cap_m')
            else 0.12
        )
        heading_err = abs(metrics.get('heading_error') or 0.0)
        lat = self.segment_lateral_offset_m()

        gates = {
            'obstacle_passed': self.obstacle_has_been_passed(),
            'recover_heading': heading_err <= getattr(self, 'avoid_rejoin_heading_tol', math.radians(6.0)),
            'recover_lateral': abs(lat) <= getattr(self, 'avoid_rejoin_lateral_tol_m', 0.06),
            'progress_ok': progress >= target - tol,
            'progress_not_overshoot': progress <= target + overshoot,
            'clear_streak': getattr(self, 'avoidance_clear_streak', 0) >= 2,
            'not_in_trigger_zone': not self.obstacle_is_active(),
            'handoff_ready': (
                self.corridor_handoff_ready()
                if hasattr(self, 'corridor_handoff_ready')
                else False
            ),
        }
        return gates

    def format_finish_gate_report(self, metrics=None):
        gates = self.evaluate_finish_gates(metrics)
        parts = [f"{name}={'OK' if ok else 'NO'}" for name, ok in gates.items()]
        metrics = metrics or self.local_replan_status_metrics()
        return (
            f"{'; '.join(parts)} | "
            f"travel={metrics.get('travel_distance', 0.0):.2f}m "
            f"progress={self.projected_distance():.2f}m "
            f"heading_err={math.degrees(metrics.get('heading_error') or 0.0):.1f}deg"
        )

    def log_finish_decision(self, reason, metrics=None):
        metrics = metrics or self.local_replan_status_metrics()
        signature = (reason, self.format_finish_gate_report(metrics))
        if signature == self._last_finish_gate_signature:
            return
        self._last_finish_gate_signature = signature
        self.write_debug_log(
            'DECISION',
            (
                f'END_DETOUR reason={reason} | {self.format_detour_state_compact()} | '
                f'{self.format_scan_ranges_compact()} | {self.format_active_circle_compact()}'
            ),
        )
        self.write_debug_log('DECISION', f'  gates {self.format_finish_gate_report(metrics)}')

    def log_follow_mode_decision(self, follow_mode, metrics, target_point, remaining_count):
        if follow_mode == self._last_follow_mode:
            return
        previous = self._last_follow_mode
        self._last_follow_mode = follow_mode
        rejoin_ready = self.detour_ready_for_rejoin_follow(metrics)
        target_local = self.world_to_robot_local_point(target_point)
        target_local_text = 'none'
        if target_local is not None:
            target_local_text = f'({target_local[0]:.2f},{target_local[1]:.2f})'
        why = f'rejoin_ready={rejoin_ready}'
        if follow_mode == 'rejoin':
            why = f'rejoin_ready=True switched_from={previous}'
        elif not rejoin_ready:
            why = (
                f'rejoin_ready=False passed={metrics.get("obstacle_passed", False)} '
                f'path_ready={metrics.get("locked_path_ready", False)} '
                f'path_progress={metrics.get("locked_path_progress", 0.0):.2f}'
            )
        self.write_debug_log(
            'DECISION',
            (
                f'FOLLOW_MODE {previous or "none"}->{follow_mode} {why} | '
                f'{self.format_detour_state_compact()}'
            ),
        )
        self.write_debug_log(
            'DECISION',
            (
                f'  target_odom={self.format_xy_point(target_point)} '
                f'target_robot_local={target_local_text} '
                f'remaining_pts={remaining_count} '
                f'{self.format_local_replan_metrics(metrics)}'
            ),
        )

    def maybe_log_obstacle_trigger_edge(self, now_sec):
        active = self.obstacle_is_active()
        if active and not self._obstacle_was_active:
            if now_sec - self.last_obstacle_trigger_log_time >= 0.2:
                self.last_obstacle_trigger_log_time = now_sec
                mode = 'template_active'
                if not getattr(self, 'avoidance_active', False):
                    if hasattr(self, 'avoidance_should_enter') and self.avoidance_should_enter():
                        mode = 'template_watch'
                    elif hasattr(self, 'avoidance_in_approach_envelope') and self.avoidance_in_approach_envelope():
                        mode = 'template_approach'
                    else:
                        mode = 'blocker_detect'
                self.write_debug_log(
                    'TRIGGER',
                    (
                        f'进入避障触发区 mode={mode} detect<='
                        f'{self.segment_detour_trigger_distance_m():.2f}m '
                        f'watch={getattr(self, "avoid_watch_distance_m", 0.45):.2f}m | '
                        f'{self.format_scan_ranges_compact()} | '
                        f'{self.format_active_circle_compact()} '
                        f'front_only_limit={self.front_only_detour_trigger_distance_m():.2f}m'
                    ),
                )
        elif not active and self._obstacle_was_active:
            self.write_debug_log(
                'TRIGGER',
                (
                    f'离开触发区(锁定路径可能仍在执行) | {self.format_detour_state_compact()} | '
                    f'{self.format_scan_ranges_compact()}'
                ),
            )
        elif not active:
            suppress_reason = self.obstacle_trigger_suppressed_reason()
            if (
                suppress_reason is not None
                and now_sec - getattr(self, 'last_obstacle_suppress_log_time', 0.0) >= 0.8
            ):
                self.last_obstacle_suppress_log_time = now_sec
                raw_nearest = self.raw_detour_nearest_obstacle_distance_m()
                path_note = ''
                if suppress_reason == 'not_on_segment_path':
                    circle = self.active_obstacle_circle
                    if circle is None and self.detected_obstacle_circles:
                        for candidate in self.detected_obstacle_circles:
                            offset = self.circle_offset_in_segment_path_frame(candidate)
                            if offset is not None:
                                circle = candidate
                                break
                    offset = None if circle is None else self.circle_offset_in_segment_path_frame(circle)
                    if offset is not None:
                        path_note = (
                            f' path_frame_along={offset[0]:.2f}m lateral={offset[1]:.2f}m '
                            f'seg_heading={self.format_yaw_deg(self.segment_path_heading_rad())}deg'
                        )
                self.write_debug_log(
                    'TRIGGER',
                    (
                        f'抑制避障触发 reason={suppress_reason} raw_nearest='
                        f'{raw_nearest:.2f}m filtered_active=False | '
                        f'{self.format_scan_ranges_compact()} | '
                        f'{self.format_active_circle_compact()}{path_note}'
                    ),
                )
        self._obstacle_was_active = active
        if hasattr(self, 'maybe_log_template_approach_envelope'):
            self.maybe_log_template_approach_envelope(now_sec)

    def maybe_log_template_approach_cap(self, now_sec, cap_mps, segment_speed):
        if now_sec - getattr(self, '_last_template_approach_log_at', -1.0) < 0.6:
            return
        self._last_template_approach_log_at = now_sec
        self.log_template_mission_cap(cap_mps, segment_speed)

    def log_local_plan_attempt(self, plan_reason, reference_points, anchor_point, path_points, plan_meta=None):
        plan_meta = plan_meta or {}
        if not path_points:
            now = self.debug_log_timestamp()
            if (
                plan_reason == self.last_plan_fail_reason
                and now - self.last_plan_fail_log_at < 0.45
            ):
                return
            self.last_plan_fail_reason = plan_reason
            self.last_plan_fail_log_at = now
            self.write_debug_log(
                'PLAN',
                (
                    f'PLAN_FAIL reason={plan_reason} | {self.format_detour_state_compact()} | '
                    f'{self.format_active_circle_compact()} | anchor='
                    f'{self.format_xy_point(anchor_point)} continue_main_route=1'
                ),
            )
            return

        segment_mode = 'unknown'
        if self.current_segment is not None:
            segment_mode = str(self.current_segment.get('type', 'unknown'))
        path_len = self.polyline_length(path_points)
        clearance_line = self.format_path_vs_circle_clearance(path_points, self.active_obstacle_circle)
        rejoin_along = plan_meta.get('rejoin_along_m')
        rejoin_text = f'{rejoin_along:.2f}m' if rejoin_along is not None else 'nan'
        self.write_debug_log(
            'PLAN',
            (
                f'PLAN_OK segment={segment_mode} reason={plan_reason} planner={plan_meta.get("planner", "?")} '
                f'points={len(path_points)} len={path_len:.2f}m rejoin_along={rejoin_text} '
                f'pass_side={plan_meta.get("pass_side", "?")} | {self.format_detour_state_compact()}'
            ),
        )
        self.write_debug_log(
            'PLAN',
            (
                f'  path_odom={self.format_xy_polyline(path_points)} anchor='
                f'{self.format_xy_point(anchor_point)} | {clearance_line}'
            ),
        )
        self.write_debug_log('PLAN', f'  {self.format_active_circle_compact()}')

    def format_path_vs_circle_clearance(self, path_points, circle):
        if circle is None or not path_points:
            return 'path_vs_circle=none'
        odom_center = self.circle_center_in_world(circle)
        if odom_center is None:
            return 'path_vs_circle=odom_unavailable'
        planning_r = float(circle['radius']) + self.obstacle_circle_planning_margin_m
        min_center_dist = min(
            math.hypot(point[0] - odom_center[0], point[1] - odom_center[1])
            for point in path_points
        )
        net_clearance = min_center_dist - planning_r
        return (
            f'path_vs_circle net_clearance={net_clearance:.2f}m '
            f'({"可绕" if net_clearance > 0.05 else "贴边/穿障"})'
        )

    def log_local_plan_follow(self, follow_mode, target_point, metrics, remaining_count):
        target_local = self.world_to_robot_local_point(target_point)
        target_local_text = 'none'
        if target_local is not None:
            target_local_text = f'({target_local[0]:.2f},{target_local[1]:.2f})'
        heading_error = metrics.get('heading_error')
        heading_text = 'nan' if heading_error is None else f'{math.degrees(heading_error):.1f}deg'
        self.log_follow_mode_decision(follow_mode, metrics, target_point, remaining_count)
        self.write_debug_log(
            'FOLLOW',
            (
                f'mode={follow_mode} target_robot_local={target_local_text} '
                f'heading_err={heading_text} {self.format_local_replan_metrics(metrics)}'
            ),
        )

    def maybe_log_local_plan_follow_throttled(self, now_sec, follow_mode, target_point, metrics, remaining_count):
        if not self.local_replan_active or not self.local_replan_points:
            return
        if now_sec - self.last_detour_follow_log_time < self.detour_debug_log_period_sec:
            return
        self.last_detour_follow_log_time = now_sec
        self.log_local_plan_follow(follow_mode, target_point, metrics, remaining_count)

    def _mission_move_phase_label(self, phase):
        labels = {
            'post_turn_settle': '弯后航向收敛(段内前0.4m,航向>5°则边转边走)',
            'nominal_track': '直行贴实测弦线(PD:横偏+航向)',
            'segment_end_trim': '段末停车对齐(里程够但横偏>10cm或航向>10°)',
        }
        return labels.get(phase, phase)

    def _mission_move_steer_explain(self, lateral_m, heading_err_rad, angular):
        parts = []
        if abs(lateral_m) >= 0.04:
            if lateral_m > 0.0:
                parts.append(f'横偏+{lateral_m:.2f}m在弦线左侧→控制应右转压回')
            else:
                parts.append(f'横偏{lateral_m:.2f}m在弦线右侧→控制应左转压回')
        if heading_err_rad is not None and abs(heading_err_rad) >= math.radians(2.5):
            parts.append(f'航向误差{math.degrees(heading_err_rad):+.1f}°')
        if abs(angular) < 0.02:
            parts.append('ω≈0')
        elif angular > 0.0:
            parts.append(f'实际下发左转ω={angular:+.3f}')
        else:
            parts.append(f'实际下发右转ω={angular:+.3f}')
        if not parts:
            return '横偏航向均小'
        return '；'.join(parts)

    def log_mission_move_segment_begin(self, plan_index):
        """弯后进入 move 段时必打一条，解释参考线/横偏从哪来。"""
        segment = self.current_segment or {}
        if segment.get('type') != 'move':
            return
        name = segment.get('description', '')
        prev_desc = 'none'
        if plan_index > 0 and hasattr(self, 'plan'):
            prev_desc = self.plan[plan_index - 1].get('description', 'none')
        pose_xy = self.format_position_xy()
        along_m = 0.0
        lateral_m = 0.0
        if hasattr(self, 'progress_along_segment_m') and self.current_position is not None:
            along_val = self.progress_along_segment_m(self.current_position)
            if along_val is not None:
                along_m = float(along_val)
        if hasattr(self, 'segment_lateral_offset_m'):
            lateral_m = float(self.segment_lateral_offset_m())
        world_text = f'plan沿程={along_m:.2f}m 横偏={lateral_m:+.2f}m(左正)'
        plan_s = getattr(self, 'segment_plan_start_xy', None)
        plan_e = getattr(self, 'segment_plan_end_xy', None)
        if plan_s is not None and plan_e is not None:
            plan_line = (
                f'plan S=({plan_s[0]:.3f},{plan_s[1]:.3f}) '
                f'E=({plan_e[0]:.3f},{plan_e[1]:.3f})'
            )
        else:
            plan_line = 'plan S/E=unset'
        target = self.navigation_target_xy() if hasattr(self, 'navigation_target_xy') else None
        if target is not None:
            plan_line += f' target=({target[0]:.3f},{target[1]:.3f})'
        seg_h_rad = self.segment_heading
        psi_text = self.format_yaw_deg(seg_h_rad) if seg_h_rad is not None else 'nan'
        settle = getattr(self, 'move_heading_settle_m', 0.0)
        yaw = self.format_yaw_deg(self.current_yaw) if self.current_yaw is not None else 'nan'
        seg_h = self.format_yaw_deg(self.segment_heading) if self.segment_heading is not None else 'nan'
        head_err = 0.0
        if self.segment_heading is not None and self.current_yaw is not None:
            head_err = math.degrees(self.angle_error(self.segment_heading, self.current_yaw))
        self.write_debug_log(
            'MOVE',
            (
                f'段开始 {name} | 上一段={prev_desc} | odom={pose_xy} '
                f'yaw={yaw}deg 段目标航向={seg_h}deg 航向误差={head_err:+.1f}deg | '
                f'{world_text} | {plan_line} ψ={psi_text}deg | '
                f'弯后收敛距离={settle:.2f}m | '
                f'说明:沿程/横偏相对名义plan弦线;无障target=段末E'
            ),
        )

    def log_turn_finished_enter_move(self):
        if self.current_segment is None or self.current_segment.get('type') != 'move':
            return
        err_deg = 0.0
        if self.segment_target_yaw is not None and self.current_yaw is not None:
            err_deg = math.degrees(self.angle_error(self.segment_target_yaw, self.current_yaw))
        self.write_debug_log(
            'MOVE',
            (
                f'转弯结束→进入直行 {self.current_segment.get("description", "?")} | '
                f'弯末yaw={self.format_yaw_deg(self.current_yaw)}deg '
                f'弯目标={self.format_yaw_deg(self.segment_target_yaw)}deg '
                f'残差={err_deg:+.1f}deg | 若残差大,接下来0.4m会边转边走'
            ),
        )

    def maybe_log_mission_move_control(
        self,
        now_sec,
        phase,
        linear,
        angular,
        lateral_m,
        heading_err_rad,
        progress_m,
        target_m,
        lat_term=None,
        head_term=None,
    ):
        period = max(0.10, float(getattr(self, 'detour_debug_log_period_sec', 0.5)))
        phase_changed = phase != getattr(self, '_last_mission_move_phase', None)
        if not phase_changed and now_sec - getattr(self, '_last_mission_move_log_at', -1.0) < period:
            return
        self._last_mission_move_log_at = now_sec
        self._last_mission_move_phase = phase

        prev_angular = getattr(self, '_last_mission_move_angular', None)
        if (
            prev_angular is not None
            and abs(prev_angular) > 0.04
            and abs(angular) > 0.04
            and prev_angular * angular < 0.0
        ):
            self.write_debug_log(
                'MOVE',
                (
                    f'STEER_FLIP 转向反转(内外来回拧) | 上次ω={prev_angular:+.3f} '
                    f'本次ω={angular:+.3f} | 横偏={lateral_m:+.2f}m '
                    f'航向误差={math.degrees(heading_err_rad):+.1f}deg | '
                    f'常见原因:横偏过大+增益高,或航向与横偏控制打架'
                ),
            )
        self._last_mission_move_angular = float(angular)

        term_text = ''
        if lat_term is not None and head_term is not None:
            term_text = f' | 分项 lat项={lat_term:+.3f} head项={head_term:+.3f}'
        self.write_debug_log(
            'MOVE',
            (
                f'phase={phase} {self._mission_move_phase_label(phase)} | '
                f'段={self.current_segment.get("description", "?")} '
                f'进度={progress_m:.2f}/{target_m:.2f}m | '
                f'odom={self.format_position_xy()} yaw={self.format_yaw_deg(self.current_yaw)}deg '
                f'段航向={self.format_yaw_deg(self.segment_heading)}deg | '
                f'横偏={lateral_m:+.2f}m 航向误差={math.degrees(heading_err_rad):+.1f}deg | '
                f'cmd v={linear:.3f} ω={angular:+.3f}{term_text} | '
                f'{self._mission_move_steer_explain(lateral_m, heading_err_rad, angular)}'
            ),
        )

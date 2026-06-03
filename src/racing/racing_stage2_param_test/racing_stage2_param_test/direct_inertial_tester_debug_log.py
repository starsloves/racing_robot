"""Structured detour/path debug logging for direct_inertial_tester."""

import math

# 默认写 FLOW（关键节点）+ POSE（中途位置/航向）；verbose 时另写 CONFIG/MOVE/DETOUR 等。
_DEFAULT_LOG_LEVELS = frozenset({'FLOW', 'POSE', 'WARN', 'ERROR'})


class DirectInertialTesterDebugLogMixin:
    """Compact, decision-focused debug log for obstacle detours."""

    _TRIANGLE_PHASE_LABELS = {
        'tri_out_turn': '出弯转向',
        'tri_out_run': '出弯直行',
        'tri_return_turn': '回正转向',
        'tri_return_run': '回正直行',
        'tri_rejoin_turn': '对齐段ψ',
    }

    def debug_log_is_verbose(self):
        return bool(getattr(self, 'debug_log_verbose', False))

    def write_debug_log(self, level, message):
        if not self.debug_log_is_verbose() and level not in _DEFAULT_LOG_LEVELS:
            return
        try:
            stamp = self.debug_log_timestamp()
            with open(self.debug_log_path, 'a', encoding='utf-8') as log_file:
                log_file.write(f'[{level}][{stamp:.3f}] {message}\n')
                log_file.flush()
        except OSError as exc:
            self.get_logger().warning(f'调试日志文件写入失败: {exc}')

    def log_flow_mission_ready(self):
        self.write_debug_log(
            'FLOW',
            (
                f'【就绪】方向={self.direction_text()} '
                f'路点={self.resolved_field_track_config_path()} '
                f'避障={getattr(self, "detour_strategy", "?")}'
            ),
        )

    def log_flow_segment_start(self, index, segment, label):
        segment_type = segment.get('type', 'unknown')
        desc = segment.get('description', '?')
        core = self.format_nav_simple_line()
        if segment_type == 'turn':
            angle_deg = float(segment.get('angle_deg', 0.0))
            turn_text = '左转' if angle_deg > 0.0 else '右转'
            self.write_debug_log(
                'FLOW',
                f'【段{index}】{label} | {turn_text}{abs(angle_deg):.0f}° | {core}',
            )
            return
        if segment_type == 'move':
            dist_m = float(segment.get('distance_m', 0.0))
            self.write_debug_log(
                'FLOW',
                f'【段{index}】{label} | 直行 {dist_m:.2f}m | {core}',
            )
            return
        if segment_type == 'pause':
            duration = float(segment.get('duration', 0.0))
            self.write_debug_log(
                'FLOW',
                f'【段{index}】{label} | 停稳 {duration:.2f}s | {core}',
            )
            return
        self.write_debug_log('FLOW', f'【段{index}】{label} | {core}')

    def log_flow_turn_done(self, next_name, plan_yaw, plan_target, plan_err):
        core = self.format_nav_simple_line()
        target_deg = (
            math.degrees(plan_target)
            if plan_target is not None
            else float('nan')
        )
        self.write_debug_log(
            'FLOW',
            (
                f'【转弯完成】→ {next_name} | {core} | '
                f'弯末={self.format_yaw_deg(plan_yaw)}° '
                f'下段ψ={target_deg:.0f}° 残差={math.degrees(plan_err):+.1f}°'
            ),
        )

    def log_flow_move_done(self, via_trim=False):
        segment = self.current_segment or {}
        name = segment.get('description', '?')
        core = self.format_nav_simple_line()
        note = '段末对齐完成' if via_trim else '到达段末'
        self.write_debug_log('FLOW', f'【直行完成】{name} | {core} | {note}')

    def log_flow_obstacle_watch(self):
        if getattr(self, '_flow_obstacle_watch_logged', False):
            return
        self._flow_obstacle_watch_logged = True
        nearest = (
            self.template_blocker_distance_m()
            if hasattr(self, 'template_blocker_distance_m')
            else float('inf')
        )
        self.write_debug_log(
            'FLOW',
            (
                f'【接近障碍】| {self.format_nav_simple_line()} | '
                f'前方障碍={self.format_distance(nearest)}m → 减速待命'
            ),
        )

    def log_flow_avoid_start(self):
        segment = (self.current_segment or {}).get('description', '?')
        nearest = (
            self.template_blocker_distance_m()
            if hasattr(self, 'template_blocker_distance_m')
            else float('inf')
        )
        side = int(getattr(self, 'locked_bypass_side', 0) or 0)
        side_text = '左' if side > 0 else ('右' if side < 0 else '自动')
        bias = float(getattr(self, 'avoid_triangle_bias_deg', 30.0))
        leg = float(getattr(self, 'avoid_triangle_leg_m', 0.80))
        seg_psi = '?'
        out_psi = '?'
        ret_psi = '?'
        if hasattr(self, 'tri_seg_heading_yaw') and self.tri_seg_heading_yaw is not None:
            seg_psi = self.format_yaw_deg(self.tri_seg_heading_yaw)
        if hasattr(self, 'tri_out_target_yaw') and self.tri_out_target_yaw is not None:
            out_psi = self.format_yaw_deg(self.tri_out_target_yaw)
        if hasattr(self, 'tri_return_target_yaw') and self.tri_return_target_yaw is not None:
            ret_psi = self.format_yaw_deg(self.tri_return_target_yaw)
        self.write_debug_log(
            'FLOW',
            (
                f'【开始避障】段={segment} | {self.format_nav_simple_line()} | '
                f'障碍={self.format_distance(nearest)}m 绕{side_text} '
                f'bias={bias:.0f}° 单腿={leg:.2f}m | '
                f'计划 出弯→{out_psi}° 回弯→{ret_psi}° 对齐→{seg_psi}°'
            ),
        )

    def log_flow_avoid_phase(self, old_phase, new_phase):
        old_label = self._TRIANGLE_PHASE_LABELS.get(old_phase, old_phase)
        new_label = self._TRIANGLE_PHASE_LABELS.get(new_phase, new_phase)
        lat = self.segment_lateral_offset_m()
        head_err = 0.0
        if hasattr(self, '_mission_move_heading_error_rad'):
            head_err = math.degrees(self._mission_move_heading_error_rad())
        self.write_debug_log(
            'FLOW',
            (
                f'【避障阶段】{old_label}→{new_label} | {self.format_nav_simple_line()} | '
                f'横偏={lat:+.2f}m 航向残差={head_err:+.1f}°'
            ),
        )

    def log_flow_avoid_rejoin(self, reason):
        segment = (self.current_segment or {}).get('description', '?')
        lat = self.segment_lateral_offset_m()
        head_err = math.degrees(self.avoidance_mission_heading_err_rad())
        reason_map = {
            'segment_complete': '绕障完成',
            'recovered': '回正完成',
            'direct_cut': '切回段内',
            'corner_shortcut': '弯角捷径',
            'segment_no_avoid': '段不支持避障',
        }
        reason_text = reason_map.get(reason, reason)
        self.write_debug_log(
            'FLOW',
            (
                f'【回正】段={segment} | {self.format_nav_simple_line()} | '
                f'横偏={lat:+.2f}m 航向残差={head_err:+.1f}° | {reason_text}→恢复直行'
            ),
        )

    def pose_log_period_sec(self):
        return max(0.25, float(getattr(self, 'detour_debug_log_period_sec', 0.5)))

    def log_pose_tick(self, now_sec, last_at_attr, mode_label, segment_name=None, extra=''):
        """默认开启：周期性记录当前位置与航向。"""
        last_at = float(getattr(self, last_at_attr, -1.0))
        if now_sec - last_at < self.pose_log_period_sec():
            return
        setattr(self, last_at_attr, now_sec)
        name = segment_name or (self.current_segment or {}).get('description', '?')
        core = self.format_nav_simple_line()
        msg = f'{mode_label} {name} | {core}'
        if extra:
            msg = f'{msg} | {extra}'
        self.write_debug_log('POSE', msg)

    def log_flow_avoid_pose_tick(self, now_sec):
        phase = getattr(self, 'goal_direct_phase', '?')
        phase_label = self._TRIANGLE_PHASE_LABELS.get(phase, phase)
        lat = self.segment_lateral_offset_m()
        segment = (self.current_segment or {}).get('description', '?')
        self.log_pose_tick(
            now_sec,
            '_last_avoid_pose_log_at',
            f'避障 {phase_label}',
            segment,
            f'横偏={lat:+.2f}m',
        )

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
        self._last_turn_debug_log_at = -1.0
        self._last_avoid_pose_log_at = -1.0

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

    def format_where_am_i(self, ref_xy=None, ref_label=''):
        """一行说清：现在 map 在哪、odom 在哪、相对参考点差多少。"""
        world_xy = (
            self.world_navigation_xy()
            if hasattr(self, 'world_navigation_xy')
            else None
        )
        raw = self.current_position
        parts = []
        if world_xy is not None:
            parts.append(f'map=({world_xy[0]:.2f},{world_xy[1]:.2f})')
        else:
            parts.append('map=nan')
        if raw is not None:
            parts.append(f'odom=({raw[0]:.2f},{raw[1]:.2f})')
        else:
            parts.append('odom=nan')
        if ref_xy is not None and world_xy is not None:
            dx = float(world_xy[0]) - float(ref_xy[0])
            dy = float(world_xy[1]) - float(ref_xy[1])
            label = ref_label or '参考'
            parts.append(f'Δ{label}=({dx:+.2f},{dy:+.2f})')
        return ' '.join(parts)

    def format_yaw_offset_note(self):
        offset = float(getattr(self, 'world_yaw_offset_rad', 0.0))
        return f'offset={math.degrees(offset):+.1f}°(plan=raw-offset)'

    def log_turn_tick_snapshot(
        self,
        now_sec,
        plan_yaw,
        raw_yaw,
        phys_target_rad,
        error_rad,
        angular,
        linear_speed,
    ):
        segment = self.current_segment or {}
        desc = segment.get('description', '?')
        if self.debug_log_is_verbose():
            del plan_yaw, raw_yaw, phys_target_rad, error_rad, angular, linear_speed
            if now_sec - getattr(self, '_last_turn_debug_log_at', -1.0) < self.pose_log_period_sec():
                return
            self._last_turn_debug_log_at = now_sec
            core = (
                self.format_nav_simple_line()
                if hasattr(self, 'format_nav_simple_line')
                else self.format_nav_core_line()
            )
            self.write_debug_log('TURN', f'{desc} | {core}')
            return
        self.log_pose_tick(now_sec, '_last_turn_debug_log_at', '转弯', desc)

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
                if hasattr(self, 'log_obstacle_perception_snapshot'):
                    self.log_obstacle_perception_snapshot(f'TRIGGER_ENTER mode={mode}')
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
                if hasattr(self, 'log_obstacle_perception_snapshot'):
                    self.log_obstacle_perception_snapshot(
                        f'TRIGGER_SUPPRESS reason={suppress_reason}'
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
            'nominal_track': '直行贴世界弦线(PD:横偏+航向, target=段末E)',
            'segment_end_trim': '段末停车对齐(沿程/距E达标但横偏>10cm或航向>10°)',
            'finish_approach': '末段回程贴弦线收敛',
        }
        return labels.get(phase, phase)

    def format_mission_world_nav_snapshot(self):
        """兼容旧调用；主日志用 format_nav_core_line。"""
        if hasattr(self, 'format_nav_core_line'):
            return self.format_nav_core_line()
        return 'nav=nan'

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
        del plan_index
        if self.debug_log_is_verbose():
            segment = self.current_segment or {}
            if segment.get('type') != 'move':
                return
            name = segment.get('description', '')
            self.write_debug_log(
                'MOVE',
                (
                    f'段开始 {name} | 起点=当前位姿 | '
                    f'{self.format_nav_simple_line()} | '
                    f'计划沿程={float(getattr(self, "segment_plan_length_m", 0.0)):.2f}m'
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

        seg_name = (self.current_segment or {}).get('description', '?')
        if not self.debug_log_is_verbose():
            head_deg = math.degrees(float(heading_err_rad))
            self.write_debug_log(
                'POSE',
                (
                    f'直行 {seg_name} | {self.format_nav_simple_line()} | '
                    f'航向误差={head_deg:+.1f}° cmd_ω={float(angular):+.3f}'
                ),
            )
            self._last_mission_move_angular = float(angular)
            return

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

        world_snap = self.format_nav_simple_line() if hasattr(self, 'format_nav_simple_line') else self.format_mission_world_nav_snapshot()
        seg_name = (self.current_segment or {}).get('description', '?')
        cmd_note = (
            f' cmd_ω={angular:+.3f}'
            if phase in ('straight_open_loop', 'heading_hold_gentle')
            else ''
        )
        self.write_debug_log(
            'MOVE',
            f'{seg_name} | {world_snap}{cmd_note}',
        )

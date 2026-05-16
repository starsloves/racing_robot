import math

import rclpy
from rclpy.duration import Duration

from racing_stage2.stage2_inertial_navigator import Stage2InertialNavigator


class DirectInertialTester(Stage2InertialNavigator):
    def __init__(self):
        super().__init__()

        self.declare_parameter('test_direction', 'clockwise')
        self.declare_parameter('test_start_mode', 'auto')
        self.declare_parameter('test_feedback_prefix', '惯导参数测试')
        self.declare_parameter('rectangle_first_leg_m', 1.20)
        self.declare_parameter('rectangle_side_leg_m', 0.60)
        self.declare_parameter('rectangle_top_leg_m', 2.80)

        self.test_direction_raw = str(self.get_parameter('test_direction').value).strip()
        self.test_direction = self.resolve_test_direction(self.test_direction_raw)
        self.test_start_mode = str(self.get_parameter('test_start_mode').value).strip().lower() or 'auto'
        self.test_feedback_prefix = str(self.get_parameter('test_feedback_prefix').value).strip() or '惯导参数测试'
        self.rectangle_first_leg_m = max(
            0.0,
            float(self.get_parameter('rectangle_first_leg_m').value),
        )
        self.rectangle_side_leg_m = max(
            0.0,
            float(self.get_parameter('rectangle_side_leg_m').value),
        )
        self.rectangle_top_leg_m = max(
            0.0,
            float(self.get_parameter('rectangle_top_leg_m').value),
        )

        self.phase = 2
        self.task_raw = self.test_direction_raw
        self.direction = self.test_direction

        self.reported_waiting_pose = False
        self.reported_start_delay = False
        self.last_progress_bucket = -1
        self.detour_front_confirm_count = 0
        self.detour_front_test_angle_deg = min(self.detour_front_angle_deg, 35.0)
        self.detour_side_test_window_deg = min(self.detour_side_window_deg, 16.0)
        self.detour_heading_gate_rad = math.radians(12.0)
        self.detour_confirm_required = 3
        self.detour_turn_settle_sec = 0.30
        self.detour_realign_pause_sec = 2.0
        self.detour_turn_heading_tolerance = min(self.heading_tolerance, math.radians(1.5))
        self.detour_turn_linear_speed = self.turn_linear_speed
        self.detour_lane_change_angle_deg = 60.0
        self.active_turn_heading_tolerance = self.heading_tolerance
        self.last_detour_turn_log_time = 0.0
        self.detour_detection_locked = False
        self.detour_resume_yaw = None
        self.front_obstacle_angle_deg = 0.0

        # Stage1-style obstacle handling state machine.
        self.stage1_avoid_linear_speed = 0.10
        self.stage1_avoid_angular_speed = 0.8
        self.stage1_avoid_min_duration_sec = 0.7
        self.stage1_avoid_clear_hold_sec = 0.25
        self.stage1_avoid_min_turn_angle_rad = math.radians(18.0)
        self.stage1_clear_distance = 0.65
        self.stage1_counter_steer_linear_speed = 0.10
        self.stage1_counter_steer_angular_speed = 0.95
        self.stage1_counter_steer_duration_scale = 1.35
        self.stage1_counter_steer_min_duration_sec = 0.45
        self.stage1_counter_steer_max_duration_sec = 1.20
        self.stage1_recovery_linear_speed = 0.12
        self.stage1_recovery_turn_linear_speed = 0.08
        self.stage1_recovery_angular_speed = 0.75
        self.stage1_recovery_heading_kp = 2.4
        self.stage1_recovery_max_angular_speed = 1.1
        self.stage1_recovery_min_angular_speed = 0.5
        self.stage1_recovery_in_place_angle_rad = math.radians(8.0)
        self.stage1_recovery_timeout = 2.5
        self.stage1_recovery_duration_scale = 0.9
        self.stage1_obstacle_state = 'forward'
        self.stage1_avoid_turn_direction = 0.0
        self.stage1_avoid_started_time = None
        self.stage1_avoid_clear_since = None
        self.stage1_avoid_entry_yaw = None
        self.stage1_last_avoid_duration = 0.0
        self.stage1_counter_steer_deadline = None
        self.stage1_recovery_deadline = None
        self.stage1_recovery_uses_heading = False
        self.stage1_desired_heading = None

        self.get_logger().info(
            f'{self.test_feedback_prefix}节点已就绪，方向={self.direction_text()}，'
            f'模式={self.start_mode_text()}，'
            f'矩形参数=({self.rectangle_first_leg_m:.2f}, '
            f'{self.rectangle_side_leg_m:.2f}, {self.rectangle_top_leg_m:.2f})m，'
            f'避障方向=左右择优，且回到避障前yaw角，前向检测角±{self.detour_front_test_angle_deg:.0f}度'
        )

    def resolve_test_direction(self, raw_value):
        normalized = str(raw_value).strip().lower()
        if normalized in ('clockwise', 'cw', '顺时针'):
            return 'clockwise'
        if normalized in ('counterclockwise', 'ccw', 'anticlockwise', 'anti-clockwise', '逆时针'):
            return 'counterclockwise'

        parsed = self.parse_direction(str(raw_value).strip())
        if parsed is not None:
            return parsed

        self.get_logger().warning(
            f'无法识别测试方向 "{raw_value}"，回退到顺时针'
        )
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

    def format_distance(self, value):
        if not math.isfinite(value):
            return 'inf'
        return f'{value:.2f}'

    def format_yaw_deg(self, yaw):
        if yaw is None or not math.isfinite(yaw):
            return 'nan'
        return f'{math.degrees(self.normalize_angle(yaw)):.1f}'

    def sector_closest_obstacle(self, scan_msg, min_angle_deg, max_angle_deg):
        min_distance = float('inf')
        min_angle = 0.0
        for index, distance in enumerate(scan_msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance <= 0.0:
                continue

            angle_deg = math.degrees(scan_msg.angle_min + index * scan_msg.angle_increment)
            angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
            if angle_deg < min_angle_deg or angle_deg > max_angle_deg:
                continue

            if distance < min_distance:
                min_distance = distance
                min_angle = angle_deg

        return min_distance, min_angle

    def is_detour_segment(self, segment):
        description = str((segment or {}).get('description', ''))
        return description.startswith('detour_') or bool((segment or {}).get('is_detour', False))

    def log_detour(self, message):
        self.get_logger().info(f'{self.test_feedback_prefix}避障: {message}')

    def reset_stage1_obstacle_state(self):
        self.stage1_obstacle_state = 'forward'
        self.stage1_avoid_turn_direction = 0.0
        self.stage1_avoid_started_time = None
        self.stage1_avoid_clear_since = None
        self.stage1_avoid_entry_yaw = None
        self.stage1_last_avoid_duration = 0.0
        self.stage1_counter_steer_deadline = None
        self.stage1_recovery_deadline = None
        self.stage1_recovery_uses_heading = False
        self.stage1_desired_heading = None

    def begin_stage1_avoidance(self, danger_angle_deg):
        self.stage1_obstacle_state = 'avoiding'
        self.stage1_avoid_turn_direction = -1.0 if danger_angle_deg > 0.0 else 1.0
        self.stage1_avoid_started_time = self.get_clock().now()
        self.stage1_avoid_clear_since = None
        self.stage1_avoid_entry_yaw = self.current_yaw
        self.stage1_counter_steer_deadline = None
        self.stage1_recovery_deadline = None
        self.stage1_recovery_uses_heading = False
        self.stage1_desired_heading = self.segment_heading if self.segment_heading is not None else self.current_yaw
        turn_text = '左' if self.stage1_avoid_turn_direction > 0.0 else '右'
        self.log_detour(
            f'参考第一阶段开始避障，danger_angle={danger_angle_deg:.1f}deg，'
            f'转向={turn_text}，desired_yaw={self.format_yaw_deg(self.stage1_desired_heading)}deg'
        )

    def begin_stage1_counter_steer(self):
        if self.stage1_obstacle_state != 'avoiding':
            return

        now = self.get_clock().now()
        avoid_duration = 0.0
        if self.stage1_avoid_started_time is not None:
            avoid_duration = (now - self.stage1_avoid_started_time).nanoseconds / 1e9
        self.stage1_last_avoid_duration = avoid_duration

        counter_duration = max(
            self.stage1_counter_steer_min_duration_sec,
            avoid_duration * self.stage1_counter_steer_duration_scale,
        )
        counter_duration = min(counter_duration, self.stage1_counter_steer_max_duration_sec)

        self.stage1_obstacle_state = 'countersteering'
        self.stage1_avoid_clear_since = None
        self.stage1_counter_steer_deadline = now + Duration(seconds=counter_duration)
        self.stage1_recovery_deadline = None
        self.stage1_recovery_uses_heading = False
        self.log_detour(f'进入反打阶段，duration={counter_duration:.2f}s')

    def begin_stage1_recovery(self):
        if self.stage1_obstacle_state not in ('avoiding', 'countersteering'):
            return

        now = self.get_clock().now()
        avoid_duration = self.stage1_last_avoid_duration
        if avoid_duration <= 0.0 and self.stage1_avoid_started_time is not None:
            avoid_duration = (now - self.stage1_avoid_started_time).nanoseconds / 1e9

        self.stage1_obstacle_state = 'recovering'
        self.stage1_avoid_clear_since = None
        self.stage1_counter_steer_deadline = None
        self.stage1_recovery_uses_heading = (
            self.current_yaw is not None and self.stage1_desired_heading is not None
        )
        if self.stage1_recovery_uses_heading:
            heading_error = abs(self.angle_error(self.stage1_desired_heading, self.current_yaw))
            estimated_duration = max(
                0.6,
                heading_error / max(self.stage1_recovery_max_angular_speed, 0.1) * 1.6,
            )
            timeout_sec = min(self.stage1_recovery_timeout, estimated_duration)
            self.stage1_recovery_deadline = now + Duration(seconds=timeout_sec)
            self.log_detour(
                f'进入回正阶段，desired_yaw={self.format_yaw_deg(self.stage1_desired_heading)}deg，'
                f'timeout={timeout_sec:.2f}s'
            )
            return

        recovery_duration = max(0.15, avoid_duration * self.stage1_recovery_duration_scale)
        recovery_duration = min(recovery_duration, self.stage1_recovery_timeout)
        self.stage1_recovery_deadline = now + Duration(seconds=recovery_duration)
        self.log_detour(f'进入定时回正阶段，duration={recovery_duration:.2f}s')

    def stage1_recovery_complete(self):
        now = self.get_clock().now()
        if self.stage1_recovery_uses_heading and self.current_yaw is not None and self.stage1_desired_heading is not None:
            if abs(self.angle_error(self.stage1_desired_heading, self.current_yaw)) <= self.heading_tolerance:
                return True

        if self.stage1_recovery_deadline is not None and now >= self.stage1_recovery_deadline:
            return True

        return False

    def finish_stage1_recovery(self):
        self.log_detour(
            f'参考第一阶段避障完成，恢复原始航向 {self.format_yaw_deg(self.stage1_desired_heading)}deg'
        )
        self.reset_stage1_obstacle_state()

    def stage1_avoid_turn_reached(self):
        if self.current_yaw is None or self.stage1_avoid_entry_yaw is None:
            return True
        return abs(self.angle_error(self.current_yaw, self.stage1_avoid_entry_yaw)) >= self.stage1_avoid_min_turn_angle_rad

    def current_segment_allows_stage1_avoidance(self):
        if not self.detour_enabled or self.current_segment is None:
            return False
        if self.current_segment.get('type') != 'move':
            return False
        if not bool(self.current_segment.get('allow_detour', True)):
            return False
        return True

    def run_stage1_style_obstacle_avoidance(self):
        if self.stage1_obstacle_state == 'forward':
            return False

        if self.stage1_obstacle_state == 'avoiding':
            obstacle_present = math.isfinite(self.front_obstacle_distance) and self.front_obstacle_distance <= self.detour_obstacle_distance
            cmd = self.create_twist(
                self.stage1_avoid_linear_speed,
                self.stage1_avoid_turn_direction * self.stage1_avoid_angular_speed,
            )
            if obstacle_present:
                self.stage1_avoid_clear_since = None
                self.cmd_pub.publish(cmd)
                return True

            now = self.get_clock().now()
            if self.stage1_avoid_clear_since is None:
                self.stage1_avoid_clear_since = now
                self.log_detour('前方已清空，开始保持清空计时')

            avoid_elapsed = 0.0
            if self.stage1_avoid_started_time is not None:
                avoid_elapsed = (now - self.stage1_avoid_started_time).nanoseconds / 1e9
            clear_elapsed = (now - self.stage1_avoid_clear_since).nanoseconds / 1e9

            if (
                avoid_elapsed >= self.stage1_avoid_min_duration_sec
                and clear_elapsed >= self.stage1_avoid_clear_hold_sec
                and self.stage1_avoid_turn_reached()
            ):
                self.begin_stage1_counter_steer()
                return self.run_stage1_style_obstacle_avoidance()

            self.cmd_pub.publish(cmd)
            return True

        if self.stage1_obstacle_state == 'countersteering':
            if self.stage1_counter_steer_deadline is not None and self.get_clock().now() >= self.stage1_counter_steer_deadline:
                self.begin_stage1_recovery()
                return self.run_stage1_style_obstacle_avoidance()

            self.cmd_pub.publish(
                self.create_twist(
                    self.stage1_counter_steer_linear_speed,
                    -self.stage1_avoid_turn_direction * self.stage1_counter_steer_angular_speed,
                )
            )
            return True

        if self.stage1_obstacle_state == 'recovering':
            if self.stage1_recovery_complete():
                self.finish_stage1_recovery()
                return False

            if self.stage1_recovery_uses_heading and self.current_yaw is not None and self.stage1_desired_heading is not None:
                heading_error = self.angle_error(self.stage1_desired_heading, self.current_yaw)
                angular_cmd = self.clamp(
                    self.stage1_recovery_heading_kp * heading_error,
                    self.stage1_recovery_max_angular_speed,
                )
                if abs(heading_error) > self.heading_tolerance and abs(angular_cmd) < self.stage1_recovery_min_angular_speed:
                    angular_cmd = math.copysign(self.stage1_recovery_min_angular_speed, heading_error)

                linear_cmd = self.stage1_recovery_turn_linear_speed
                if abs(heading_error) <= self.stage1_recovery_in_place_angle_rad:
                    linear_cmd = self.stage1_recovery_linear_speed

                self.cmd_pub.publish(self.create_twist(linear_cmd, angular_cmd))
                return True

            self.cmd_pub.publish(
                self.create_twist(
                    self.stage1_recovery_linear_speed,
                    -self.stage1_avoid_turn_direction * self.stage1_recovery_angular_speed,
                )
            )
            return True

        return False

    def reset_mission(self, clear_task):
        self.detour_detection_locked = False
        self.detour_resume_yaw = None
        self.reset_stage1_obstacle_state()
        super().reset_mission(clear_task)

    def rectangle_segment_label(self, segment):
        description = str(segment.get('description', 'unknown'))

        detour_labels = {
            'detour_right_shift_out_turn': '右侧避障外摆转向',
            'detour_right_shift_out_move': '右侧避障侧移离开原路线',
            'detour_right_forward_align': '右侧避障回正到原始航向',
            'detour_right_forward_align_wait': '右侧避障回正前等待',
            'detour_right_pass_obstacle': '右侧避障沿原始航向通过障碍',
            'detour_right_return_turn': '右侧避障转向准备回到原路线',
            'detour_right_return_move': '右侧避障侧移回到原路线',
            'detour_right_resume_align': '右侧避障最终回正到原始航向',
            'detour_right_resume_align_wait': '右侧避障最终回正前等待',
            'detour_right_settle_before_turn': '右侧避障结束停稳',
            'detour_left_shift_out_turn': '左侧避障外摆转向',
            'detour_left_shift_out_move': '左侧避障侧移离开原路线',
            'detour_left_forward_align': '左侧避障回正到原始航向',
            'detour_left_forward_align_wait': '左侧避障回正前等待',
            'detour_left_pass_obstacle': '左侧避障沿原始航向通过障碍',
            'detour_left_return_turn': '左侧避障转向准备回到原路线',
            'detour_left_return_move': '左侧避障侧移回到原路线',
            'detour_left_resume_align': '左侧避障最终回正到原始航向',
            'detour_left_resume_align_wait': '左侧避障最终回正前等待',
            'detour_left_settle_before_turn': '左侧避障结束停稳',
        }
        if description in detour_labels:
            return detour_labels[description]
        if description.endswith('_resume'):
            return '避障后回到原路线'

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

    def start_segment(self, index):
        super().start_segment(index)
        self.last_progress_bucket = -1
        self.detour_front_confirm_count = 0
        self.active_turn_heading_tolerance = self.heading_tolerance
        self.last_detour_turn_log_time = 0.0

        if self.current_segment is None or self.plan_index != index:
            return

        segment = self.current_segment
        segment_type = segment.get('type')
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

        if self.is_detour_segment(segment):
            if segment_type == 'turn':
                self.log_detour(
                    f'进入 {segment.get("description", "detour_turn")}，'
                    f'current_yaw={self.format_yaw_deg(self.current_yaw)}deg，'
                    f'start_yaw={self.format_yaw_deg(self.segment_start_yaw)}deg，'
                    f'target_yaw={self.format_yaw_deg(self.segment_target_yaw)}deg，'
                    f'tol={math.degrees(self.active_turn_heading_tolerance):.1f}deg'
                )
            elif segment_type == 'move':
                self.log_detour(
                    f'进入 {segment.get("description", "detour_move")}，'
                    f'distance={float(segment.get("distance_m", 0.0)):.2f}m，'
                    f'heading={self.format_yaw_deg(self.segment_heading)}deg，'
                    f'current_yaw={self.format_yaw_deg(self.current_yaw)}deg'
                )
            elif segment_type == 'pause':
                self.log_detour(
                    f'进入 {segment.get("description", "detour_pause")}，'
                    f'duration={float(segment.get("duration", 0.0)):.2f}s，'
                    f'current_yaw={self.format_yaw_deg(self.current_yaw)}deg'
                )

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

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_frame_id = msg.header.frame_id
        self.front_obstacle_distance, self.front_obstacle_angle_deg = self.sector_closest_obstacle(
            msg,
            -self.detour_front_test_angle_deg,
            self.detour_front_test_angle_deg,
        )
        half_window = self.detour_side_test_window_deg / 2.0
        self.left_clearance_distance = self.sector_min_distance(
            msg,
            self.detour_side_center_deg - half_window,
            self.detour_side_center_deg + half_window,
        )
        self.right_clearance_distance = self.sector_min_distance(
            msg,
            -self.detour_side_center_deg - half_window,
            -self.detour_side_center_deg + half_window,
        )

    def detour_side_text(self, side):
        return '左侧' if side == 'left' else '右侧'

    def side_clearance_metric(self, clearance):
        if math.isnan(clearance):
            return float('-inf')
        return clearance

    def side_clearance_ok(self, clearance):
        if math.isnan(clearance):
            return False
        if math.isinf(clearance):
            return True
        return clearance >= self.detour_min_side_clearance

    def select_detour_side(self):
        left_clear = self.left_clearance_distance
        right_clear = self.right_clearance_distance
        left_ok = self.side_clearance_ok(left_clear)
        right_ok = self.side_clearance_ok(right_clear)

        if left_ok and right_ok:
            return 'left' if self.side_clearance_metric(left_clear) >= self.side_clearance_metric(right_clear) else 'right'
        if left_ok:
            return 'left'
        if right_ok:
            return 'right'
        return None

    def maybe_inject_detour(self):
        if self.detour_detection_locked:
            self.detour_front_confirm_count = 0
            return False

        if not self.current_segment_allows_detour():
            self.detour_front_confirm_count = 0
            return False

        if not math.isfinite(self.front_obstacle_distance) or self.front_obstacle_distance > self.detour_obstacle_distance:
            self.detour_front_confirm_count = 0
            return False

        if self.segment_heading is not None and self.current_yaw is not None:
            heading_error = self.angle_error(self.segment_heading, self.current_yaw)
            if abs(heading_error) > self.detour_heading_gate_rad:
                self.detour_front_confirm_count = 0
                return False

        self.detour_front_confirm_count = min(
            self.detour_front_confirm_count + 1,
            self.detour_confirm_required,
        )
        if self.detour_front_confirm_count < self.detour_confirm_required:
            return False

        side = self.select_detour_side()
        if side is None:
            self.log_detour(
                f'等待，front={self.format_distance(self.front_obstacle_distance)}m，'
                f'left={self.format_distance(self.left_clearance_distance)}m，'
                f'right={self.format_distance(self.right_clearance_distance)}m，'
                f'min_clear={self.detour_min_side_clearance:.2f}m，'
                '未找到可安全绕行侧'
            )
            self.publish_state('detour_waiting')
            self.cmd_pub.publish(self.create_twist())
            return True

        progress = self.projected_distance()
        target_distance = float(self.current_segment['distance_m'])
        remaining_distance = max(0.0, target_distance - progress)
        if remaining_distance <= self.distance_tolerance:
            self.detour_front_confirm_count = 0
            return False

        forward_distance = min(self.detour_forward_distance_m, remaining_distance)
        resume_distance = max(0.0, remaining_distance - forward_distance)
        detour_segments = self.build_detour_segments(side, forward_distance, resume_distance)
        entry_yaw = self.segment_heading if self.segment_heading is not None else self.segment_start_yaw
        self.detour_detection_locked = True
        self.detour_resume_yaw = self.normalize_angle(entry_yaw) if entry_yaw is not None else None
        self.log_detour(
            f'触发，front={self.format_distance(self.front_obstacle_distance)}m，'
            f'left={self.format_distance(self.left_clearance_distance)}m，'
            f'right={self.format_distance(self.right_clearance_distance)}m，'
            f'选侧={self.detour_side_text(side)}，'
            f'entry_yaw={self.format_yaw_deg(entry_yaw)}deg，'
            f'progress={progress:.2f}/{target_distance:.2f}m，'
            f'forward={forward_distance:.2f}m，resume={resume_distance:.2f}m，'
            f'锁定检测直到回到 {self.format_yaw_deg(self.detour_resume_yaw)}deg'
        )
        self.plan = self.plan[:self.plan_index] + detour_segments + self.plan[self.plan_index + 1:]
        self.detour_cooldown_until = self.get_clock().now().nanoseconds / 1e9 + self.detour_cooldown_sec
        self.detour_front_confirm_count = 0
        self.publish_feedback(
            f'检测到前方障碍，选择更通畅的{self.detour_side_text(side)}避障，随后回归原线路并回到避障前yaw角'
        )
        self.start_segment(self.plan_index)
        return True

    def build_detour_segments(self, side, forward_distance, resume_distance):
        detour_entry_yaw = self.segment_heading if self.segment_heading is not None else self.segment_start_yaw
        if detour_entry_yaw is None:
            return []

        detour_entry_yaw = self.normalize_angle(detour_entry_yaw)
        side_sign = 1.0 if side == 'left' else -1.0
        lane_change_angle_rad = math.radians(self.detour_lane_change_angle_deg)
        shift_heading = self.normalize_angle(detour_entry_yaw + side_sign * lane_change_angle_rad)
        return_heading = self.normalize_angle(detour_entry_yaw - side_sign * lane_change_angle_rad)

        total_remaining_distance = max(0.0, forward_distance + resume_distance)
        max_lateral_distance = (total_remaining_distance * math.tan(lane_change_angle_rad)) / 2.0
        effective_lateral_distance = min(self.detour_lateral_distance_m, max(0.0, max_lateral_distance))

        if effective_lateral_distance <= self.distance_tolerance:
            self.log_detour(
                f'剩余距离不足以执行绕障，remaining={total_remaining_distance:.2f}m，'
                f'angle={self.detour_lane_change_angle_deg:.0f}deg'
            )
            return []

        lane_change_move_distance = effective_lateral_distance / math.sin(lane_change_angle_rad)
        lane_change_forward_progress = lane_change_move_distance * math.cos(lane_change_angle_rad) * 2.0
        remaining_after_lane_change = max(0.0, total_remaining_distance - lane_change_forward_progress)
        pass_distance = min(forward_distance, remaining_after_lane_change)
        resume_distance_after_detour = max(0.0, remaining_after_lane_change - pass_distance)

        detour_segments = [
            {
                'type': 'turn',
                'angle_deg': side_sign * self.detour_lane_change_angle_deg,
                'description': f'detour_{side}_shift_out_turn',
                'force_start_yaw': detour_entry_yaw,
                'force_target_yaw': shift_heading,
                'heading_tolerance_rad': self.detour_turn_heading_tolerance,
                'turn_linear_speed': self.detour_turn_linear_speed,
            },
            {
                'type': 'move',
                'distance_m': lane_change_move_distance,
                'speed': self.corridor_linear_speed,
                'description': f'detour_{side}_shift_out_move',
                'allow_detour': False,
                'is_detour': True,
                'force_segment_heading': shift_heading,
            },
            {
                'type': 'pause',
                'duration': self.detour_realign_pause_sec,
                'description': f'detour_{side}_forward_align_wait',
            },
            {
                'type': 'turn',
                'angle_deg': -side_sign * self.detour_lane_change_angle_deg,
                'description': f'detour_{side}_forward_align',
                'force_start_yaw': shift_heading,
                'force_target_yaw': detour_entry_yaw,
                'heading_tolerance_rad': self.detour_turn_heading_tolerance,
                'turn_linear_speed': self.detour_turn_linear_speed,
            },
            {
                'type': 'move',
                'distance_m': pass_distance,
                'speed': self.corridor_linear_speed,
                'description': f'detour_{side}_pass_obstacle',
                'allow_detour': False,
                'is_detour': True,
                'force_segment_heading': detour_entry_yaw,
            },
            {
                'type': 'turn',
                'angle_deg': -side_sign * self.detour_lane_change_angle_deg,
                'description': f'detour_{side}_return_turn',
                'force_start_yaw': detour_entry_yaw,
                'force_target_yaw': return_heading,
                'heading_tolerance_rad': self.detour_turn_heading_tolerance,
                'turn_linear_speed': self.detour_turn_linear_speed,
            },
            {
                'type': 'move',
                'distance_m': lane_change_move_distance,
                'speed': self.corridor_linear_speed,
                'description': f'detour_{side}_return_move',
                'allow_detour': False,
                'is_detour': True,
                'force_segment_heading': return_heading,
            },
            {
                'type': 'pause',
                'duration': self.detour_realign_pause_sec,
                'description': f'detour_{side}_resume_align_wait',
            },
            {
                'type': 'turn',
                'angle_deg': side_sign * self.detour_lane_change_angle_deg,
                'description': f'detour_{side}_resume_align',
                'force_start_yaw': return_heading,
                'force_target_yaw': detour_entry_yaw,
                'heading_tolerance_rad': self.detour_turn_heading_tolerance,
                'turn_linear_speed': self.detour_turn_linear_speed,
            },
        ]

        if pass_distance <= self.distance_tolerance:
            detour_segments = [
                segment for segment in detour_segments
                if segment.get('description') != f'detour_{side}_pass_obstacle'
            ]

        if resume_distance_after_detour > self.distance_tolerance:
            detour_segments.append({
                'type': 'move',
                'distance_m': resume_distance_after_detour,
                'speed': float(self.current_segment.get('speed', self.corridor_linear_speed)),
                'description': f'{self.current_segment.get("description", "segment")}_resume',
                'allow_detour': False,
                'force_segment_heading': detour_entry_yaw,
            })

        self.log_detour(
            f'绕障几何，angle={self.detour_lane_change_angle_deg:.0f}deg，'
            f'lateral={effective_lateral_distance:.2f}m，'
            f'lane_change_move={lane_change_move_distance:.2f}m，'
            f'forward_after_lane_change={remaining_after_lane_change:.2f}m，'
            f'pass={pass_distance:.2f}m，resume={resume_distance_after_detour:.2f}m'
        )

        next_segment_index = self.plan_index + 1
        next_segment = self.plan[next_segment_index] if next_segment_index < len(self.plan) else None

        if (
            detour_entry_yaw is not None
            and next_segment is not None
            and next_segment.get('type') == 'turn'
        ):
            next_segment['force_start_yaw'] = detour_entry_yaw
            next_segment['force_target_yaw'] = self.normalize_angle(
                detour_entry_yaw + math.radians(float(next_segment.get('angle_deg', 0.0)))
            )
            self.log_detour(
                f'原始转弯锚定，segment={next_segment.get("description", "turn")}，'
                f'start_yaw={self.format_yaw_deg(detour_entry_yaw)}deg，'
                f'target_yaw={self.format_yaw_deg(next_segment["force_target_yaw"])}deg'
            )

        if next_segment is not None and next_segment.get('type') == 'turn':
            detour_segments.append({
                'type': 'pause',
                'duration': self.detour_turn_settle_sec,
                'description': f'detour_{side}_settle_before_turn',
            })

        return detour_segments

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
                    self.get_logger().info(
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

        if self.run_stage1_style_obstacle_avoidance():
            return

        if (
            self.current_segment_allows_stage1_avoidance()
            and math.isfinite(self.front_obstacle_distance)
            and self.front_obstacle_distance <= self.detour_obstacle_distance
        ):
            self.begin_stage1_avoidance(self.front_obstacle_angle_deg)
            self.run_stage1_style_obstacle_avoidance()
            return

        if self.current_position is None or self.segment_heading is None:
            self.cmd_pub.publish(self.create_twist())
            return

        progress = self.projected_distance()
        target_distance = float(self.current_segment['distance_m'])
        if progress >= target_distance - self.distance_tolerance:
            self.cmd_pub.publish(self.create_twist())
            self.start_segment(self.plan_index + 1)
            return

        heading_error = 0.0 if self.current_yaw is None else self.angle_error(self.segment_heading, self.current_yaw)
        angular = self.clamp(self.heading_kp * heading_error, self.max_angular_speed)
        linear = float(self.current_segment.get('speed', self.corridor_linear_speed))
        self.cmd_pub.publish(self.create_twist(linear, angular))

    def run_turn_segment(self):
        turn_tolerance = self.active_turn_heading_tolerance
        linear_speed = float(
            (self.current_segment or {}).get('turn_linear_speed', self.turn_linear_speed)
        )

        if self.current_yaw is None or self.segment_target_yaw is None:
            self.cmd_pub.publish(self.create_twist())
            return

        error = self.angle_error(self.segment_target_yaw, self.current_yaw)
        if abs(error) <= turn_tolerance:
            if self.is_detour_segment(self.current_segment):
                description = str((self.current_segment or {}).get('description', ''))
                self.log_detour(
                    f'完成 {self.current_segment.get("description", "detour_turn")}，'
                    f'current_yaw={self.format_yaw_deg(self.current_yaw)}deg，'
                    f'target_yaw={self.format_yaw_deg(self.segment_target_yaw)}deg，'
                    f'error={math.degrees(error):.2f}deg'
                )
                if description.endswith('_resume_align'):
                    self.detour_detection_locked = False
                    self.log_detour(
                        f'已回到避障前yaw，恢复障碍检测，resume_yaw={self.format_yaw_deg(self.detour_resume_yaw)}deg，'
                        f'current_yaw={self.format_yaw_deg(self.current_yaw)}deg'
                    )
                    self.detour_resume_yaw = None
            self.publish_feedback(
                f'{self.test_feedback_prefix}当前位置: '
                f'{self.rectangle_segment_label(self.current_segment or {})}，'
                '转弯完成，进入下一段'
            )
            self.cmd_pub.publish(self.create_twist())
            self.start_segment(self.plan_index + 1)
            return

        angular = self.clamp(self.turn_kp * error, self.turn_angular_speed)
        if abs(error) > turn_tolerance and abs(angular) < self.turn_min_angular_speed:
            angular = math.copysign(self.turn_min_angular_speed, error)

        if self.is_detour_segment(self.current_segment):
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self.last_detour_turn_log_time >= 0.5:
                self.last_detour_turn_log_time = now_sec
                self.log_detour(
                    f'执行 {self.current_segment.get("description", "detour_turn")}，'
                    f'current_yaw={self.format_yaw_deg(self.current_yaw)}deg，'
                    f'target_yaw={self.format_yaw_deg(self.segment_target_yaw)}deg，'
                    f'error={math.degrees(error):.2f}deg，'
                    f'angular={angular:.2f}rad/s，'
                    f'linear={linear_speed:.2f}m/s'
                )

        self.cmd_pub.publish(self.create_twist(linear_speed, angular))

    def build_ring_plan(self):
        entry_turn = 90.0 if self.direction == 'clockwise' else -90.0
        corner_turn = -entry_turn

        return [
            {
                'type': 'turn',
                'angle_deg': entry_turn,
                'description': 'rect_enter_align',
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

        current_time = self.get_clock().now().nanoseconds / 1e9
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
            node.cmd_pub.publish(node.create_twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
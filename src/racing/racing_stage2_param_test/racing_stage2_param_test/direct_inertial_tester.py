import math

import rclpy
from nav_msgs.msg import Odometry

from racing_stage2.stage2_inertial_navigator import Stage2InertialNavigator
from racing_stage2_param_test.s1_geometry import (
    build_s1_plan,
    cross_segment_m,
    obstacle_is_left,
)
from racing_stage2_param_test.session_file_log import SessionFileLog


class DirectInertialTester(Stage2InertialNavigator):
    def __init__(self):
        super().__init__()

        self.declare_parameter('test_direction', 'clockwise')
        self.declare_parameter('test_start_mode', 'auto')
        self.declare_parameter('test_feedback_prefix', '惯导参数测试')
        self.declare_parameter('rectangle_first_leg_m', 1.20)
        self.declare_parameter('rectangle_side_leg_m', 0.60)
        self.declare_parameter('rectangle_top_leg_m', 2.80)
        # 第一个转角（rect_enter_align）的目标角度，默认 95°（替代原 90°）
        # sign 由方向决定：clockwise→+，counterclockwise→-
        # 仅影响第 1 段；后 4 个 corner 仍是 ∓90°
        self.declare_parameter('rect_enter_align_deg', 95.0)

        # 第二个转角（rect_corner_1，第 1 个 corner）的目标角度，默认 80°
        self.declare_parameter('rect_corner_1_deg', 80.0)
        self.declare_parameter('rect_corner_2_deg', 85.0)
        self.declare_parameter('rect_corner_3_deg', 85.0)
        self.declare_parameter('rect_corner_4_deg', 85.0)


        # Stage1 边转边避参数（子类可重新声明覆盖默认值）
        self.declare_parameter('avoid_leg_heading_offset_deg', 30.0)
        self.declare_parameter('avoid_leg1_distance_m', 0.30)
        self.declare_parameter('avoid_leg2_distance_m', 0.40)
        self.declare_parameter('avoid_leg_linear_speed', 0.10)
        self.declare_parameter('avoid_turn_linear_speed', 0.08)
        self.declare_parameter('avoid_leg_distance_tol_m', 0.04)
        self.declare_parameter('avoid_turn_angular_speed', 0.40)

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

        self.rect_enter_align_deg = float(self.get_parameter('rect_enter_align_deg').value)

        self.rect_corner_1_deg = float(self.get_parameter('rect_corner_1_deg').value)
        self.rect_corner_2_deg = float(self.get_parameter('rect_corner_2_deg').value)
        self.rect_corner_3_deg = float(self.get_parameter('rect_corner_3_deg').value)
        self.rect_corner_4_deg = float(self.get_parameter('rect_corner_4_deg').value)

        # Stage1 边转边避几何与执行参数
        self._avoid_offset_deg = float(self.get_parameter('avoid_leg_heading_offset_deg').value)
        self._avoid_offset_rad = math.radians(self._avoid_offset_deg)
        self._avoid_leg1_distance_m = max(0.05, float(self.get_parameter('avoid_leg1_distance_m').value))
        self._avoid_leg2_distance_m = max(0.05, float(self.get_parameter('avoid_leg2_distance_m').value))
        self._avoid_leg_linear_speed = max(0.02, float(self.get_parameter('avoid_leg_linear_speed').value))
        self._avoid_turn_linear_speed = max(0.02, float(self.get_parameter('avoid_turn_linear_speed').value))
        self._avoid_distance_tol_m = max(0.0, float(self.get_parameter('avoid_leg_distance_tol_m').value))
        self._avoid_turn_angular_speed = max(0.1, float(self.get_parameter('avoid_turn_angular_speed').value))

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

        # Stage1 风格边转边避状态机
        self._avoid_state = 'idle'
        self._avoid_plan = None
        self._avoid_leg_start_xy = None
        self._avoid_turn_accum_rad = 0.0
        self._avoid_turn_required_rad = 0.0
        self._avoid_turn_sign = 1.0
        self._avoid_prev_yaw = None
        self._avoid_turn_target_yaw = None
        self._avoid_cooldown_until = 0.0

        self._setup_wheel_odom_position()
        self._setup_session_log()

        self.get_logger().info(
            f'{self.test_feedback_prefix}节点已就绪，方向={self.direction_text()}，'
            f'模式={self.start_mode_text()}，'
            f'矩形参数=({self.rectangle_first_leg_m:.2f}, '
            f'{self.rectangle_side_leg_m:.2f}, {self.rectangle_top_leg_m:.2f})m，'
            f'避障=Stage1边转边避 ±{self._avoid_offset_deg:.0f}deg×'
            f'{self._avoid_leg1_distance_m:.2f}m/{self._avoid_leg2_distance_m:.2f}m，'
            f'前向检测角±{self.detour_front_test_angle_deg:.0f}度'
        )
        self._log_session(
            'CONFIG',
            f'方向={self.direction_text()} 模式={self.start_mode_text()} '
            f'矩形=({self.rectangle_first_leg_m:.2f},{self.rectangle_side_leg_m:.2f},'
            f'{self.rectangle_top_leg_m:.2f})m '
            f'避障±{self._avoid_offset_deg:.0f}deg '
            f'L1={self._avoid_leg1_distance_m:.2f}m L2={self._avoid_leg2_distance_m:.2f}m '
            f'pose_source={self._navigation_pose_source} '
            f'wheel={self._wheel_odom_topic} ekf={self.odom_topic} '
            f'ring_v={self.ring_linear_speed:.2f} turn_v={self.turn_linear_speed:.2f} '
            f'turn_w={self.turn_angular_speed:.2f} head_kp={self.heading_kp:.2f} '
            f'dist_tol={self.distance_tolerance:.3f} '
            f'head_tol={math.degrees(self.heading_tolerance):.1f}deg '
            f'detour_d={self.detour_obstacle_distance:.2f}m '
            f'segment_timeout={self.segment_timeout:.1f}s',
        )

    def _setup_session_log(self) -> None:
        self.declare_parameter('session_log_subdir', 'direct_inertial_test')
        self.declare_parameter('session_log_filename', 'latest.log')
        self.declare_parameter('session_telemetry_interval_sec', 0.25)
        subdir = (
            str(self.get_parameter('session_log_subdir').value).strip()
            or 'direct_inertial_test'
        )
        filename = (
            str(self.get_parameter('session_log_filename').value).strip() or 'latest.log'
        )
        self._telemetry_interval_sec = max(
            0.05, float(self.get_parameter('session_telemetry_interval_sec').value)
        )
        self._session_log = SessionFileLog(
            subdir,
            filename,
            session_title='direct inertial test session',
        )
        self._last_telemetry_sec = 0.0
        self._last_wait_log_sec = 0.0
        self._wheel_warmup_logged = False
        self._last_avoid_state_logged = 'idle'
        self._last_ekf_position = None
        self._wheel_twist = None
        self._ekf_twist = None
        self._last_cmd_linear = 0.0
        self._last_cmd_angular = 0.0
        self.get_logger().info(
            f'{self.test_feedback_prefix}会话日志: {self._session_log.path}'
        )
        self._log_session('CONFIG', f'日志路径={self._session_log.path}')

    def destroy_node(self):
        if getattr(self, '_session_log', None) is not None:
            self._session_log.close()
            self._session_log = None
        super().destroy_node()

    def _log_session(self, tag: str, message: str) -> None:
        if getattr(self, '_session_log', None) is None:
            return
        self._session_log.write(f'[{tag}] {message}')

    def publish_feedback(self, text: str) -> None:
        super().publish_feedback(text)
        self._log_session('FEEDBACK', text)

    def create_twist(self, linear_x=0.0, angular_z=0.0):
        self._last_cmd_linear = float(linear_x)
        self._last_cmd_angular = float(angular_z)
        return super().create_twist(linear_x, angular_z)

    def navigation_yaw(self):
        """统一位姿航向（current_yaw）；轮速模式下由 /odom 写入，IMU 仅诊断。"""
        return self.current_yaw

    def _wheel_pose_source_active(self) -> bool:
        return (
            getattr(self, '_navigation_pose_source', 'wheel') == 'wheel'
            and self._use_wheel_odom_for_distance
        )

    def _sync_unified_pose_from_wheel(self) -> None:
        if not self._wheel_pose_source_active():
            return
        if self.current_wheel_yaw is None:
            return
        self.current_yaw = self.current_wheel_yaw

    def imu_callback(self, msg):
        self.imu_yaw = self.quaternion_to_yaw(msg.orientation)
        if not self._wheel_pose_source_active():
            self.current_yaw = self.imu_yaw
        self.try_start_mission()

    def _fmt_num(self, value, prec=3):
        if value is None:
            return 'nan'
        try:
            number = float(value)
        except (TypeError, ValueError):
            return 'nan'
        if not math.isfinite(number):
            return 'inf' if math.isinf(number) else 'nan'
        return f'{number:.{prec}f}'

    def _raw_projected_distance(self) -> float:
        if (
            self.segment_start_pose is None
            or self.current_position is None
            or self.segment_heading is None
        ):
            return 0.0
        dx = self.current_position[0] - self.segment_start_pose[0]
        dy = self.current_position[1] - self.segment_start_pose[1]
        return (
            dx * math.cos(self.segment_heading)
            + dy * math.sin(self.segment_heading)
        )

    def _cross_track_m(self) -> float:
        if (
            self.segment_start_pose is None
            or self.current_position is None
            or self.segment_heading is None
        ):
            return 0.0
        return cross_segment_m(
            self.segment_start_pose,
            self.segment_heading,
            self.current_position,
        )

    def _full_telemetry(self) -> str:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        segment = self.current_segment or {}
        seg_type = str(segment.get('type', 'none'))
        seg_desc = str(segment.get('description', 'none'))

        wx = wy = 'nan'
        if self.current_position is not None:
            wx = self._fmt_num(self.current_position[0])
            wy = self._fmt_num(self.current_position[1])

        ekf_x = ekf_y = 'nan'
        if self._last_ekf_position is not None:
            ekf_x = self._fmt_num(self._last_ekf_position[0])
            ekf_y = self._fmt_num(self._last_ekf_position[1])

        sx = sy = 'nan'
        if self.segment_start_pose is not None:
            sx = self._fmt_num(self.segment_start_pose[0])
            sy = self._fmt_num(self.segment_start_pose[1])

        wvx = wvy = wwz = 'nan'
        if self._wheel_twist is not None:
            wvx = self._fmt_num(self._wheel_twist[0])
            wvy = self._fmt_num(self._wheel_twist[1])
            wwz = self._fmt_num(self._wheel_twist[2])

        evx = evy = ewz = 'nan'
        if self._ekf_twist is not None:
            evx = self._fmt_num(self._ekf_twist[0])
            evy = self._fmt_num(self._ekf_twist[1])
            ewz = self._fmt_num(self._ekf_twist[2])

        along = self.projected_distance() if self.current_position is not None else 0.0
        raw_along = self._raw_projected_distance() if self.current_position is not None else 0.0
        cross_cm = self._cross_track_m() * 100.0 if self.current_position is not None else 0.0

        target_m = seg_speed = 0.0
        if seg_type == 'move':
            target_m = float(segment.get('distance_m', 0.0))
            seg_speed = float(segment.get('speed', 0.0))

        avoid_leg_m = 0.0
        if self._avoid_leg_start_xy is not None and self.current_position is not None:
            avoid_leg_m = self._avoid_leg_traveled_m(
                self.current_position[0], self.current_position[1]
            )

        nav_yaw = self.navigation_yaw()
        heading_err_deg = 'nan'
        if self.segment_heading is not None and nav_yaw is not None:
            heading_err_deg = self._fmt_num(
                math.degrees(self.angle_error(self.segment_heading, nav_yaw)),
                prec=1,
            )

        turn_err_deg = 'nan'
        if self.segment_target_yaw is not None and nav_yaw is not None:
            turn_err_deg = self._fmt_num(
                math.degrees(self.angle_error(self.segment_target_yaw, nav_yaw)),
                prec=1,
            )

        imu_wheel_err_deg = 'nan'
        if self.imu_yaw is not None and self.current_wheel_yaw is not None:
            imu_wheel_err_deg = self._fmt_num(
                math.degrees(self.angle_error(self.imu_yaw, self.current_wheel_yaw)),
                prec=1,
            )

        seg_elapsed = 'nan'
        if self.segment_started_at is not None:
            seg_elapsed = self._fmt_num(now_sec - self.segment_started_at, prec=2)

        parts = [
            (
                f't={now_sec:.3f} mission={int(self.mission_active)} '
                f'done={int(self.mission_finished)} '
                f'plan={self.plan_index}/{max(len(self.plan) - 1, 0)} '
                f'seg={seg_type}:{seg_desc} seg_t={seg_elapsed}s'
            ),
            f'wheel_xy=({wx},{wy}) ekf_xy=({ekf_x},{ekf_y}) anchor=({sx},{sy})',
            (
                f'yaw={self.format_yaw_deg(self.current_yaw)} '
                f'yaw_wheel={self.format_yaw_deg(self.current_wheel_yaw)} '
                f'yaw_imu={self.format_yaw_deg(self.imu_yaw)} '
                f'yaw_ekf={self.format_yaw_deg(self.current_odom_yaw)} '
                f'yaw_leg={self.format_yaw_deg(self.segment_heading)} '
                f'yaw_seg0={self.format_yaw_deg(self.segment_start_yaw)} '
                f'yaw_tgt={self.format_yaw_deg(self.segment_target_yaw)} '
                f'imu_off={imu_wheel_err_deg} '
                f'head_err={heading_err_deg} turn_err={turn_err_deg}'
            ),
            (
                f'wheel_v=({wvx},{wvy},{wwz}) ekf_v=({evx},{evy},{ewz}) '
                f'cmd_v=({self._fmt_num(self._last_cmd_linear)},{self._fmt_num(self._last_cmd_angular)}) '
                f'seg_v={self._fmt_num(seg_speed)}'
            ),
            (
                f'along={self._fmt_num(along)}/{self._fmt_num(target_m)}m '
                f'raw_along={self._fmt_num(raw_along)}m cross={cross_cm:+.1f}cm '
                f'dist_tol={self._fmt_num(self.distance_tolerance)}'
            ),
            (
                f'avoid={self._avoid_state} avoid_leg={self._fmt_num(avoid_leg_m)}m '
                f'avoid_turn={self._fmt_num(math.degrees(self._avoid_turn_accum_rad), prec=1)}/'
                f'{self._fmt_num(math.degrees(self._avoid_turn_required_rad), prec=1)}deg '
                f'detour_lock={int(self.detour_detection_locked)}'
            ),
            (
                f'front={self.format_distance(self.front_obstacle_distance)}m '
                f'@ {self._fmt_num(self.front_obstacle_angle_deg, prec=1)}deg '
                f'left={self.format_distance(self.left_clearance_distance)}m '
                f'right={self.format_distance(self.right_clearance_distance)}m '
                f'detour_trig={self._fmt_num(self.detour_obstacle_distance)}m'
            ),
            (
                f'wheel_n={self._wheel_odom_msg_count} wheel_ready={int(self._wheel_odom_ready)} '
                f'frame={self.odom_frame_id}'
            ),
        ]
        return ' | '.join(parts)

    def _pose_diagnostic(self) -> str:
        return self._full_telemetry()

    def _maybe_log_telemetry(self, reason: str) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.mission_active:
            interval = self._telemetry_interval_sec
        elif self._wheel_odom_msg_count > 0:
            interval = 1.0
        else:
            return
        if now_sec - self._last_telemetry_sec < interval:
            return
        self._last_telemetry_sec = now_sec
        self._log_session('TELEM', f'{reason} | {self._full_telemetry()}')

    def _log_segment_enter(self, segment) -> None:
        if segment is None:
            return
        seg_type = segment.get('type', '?')
        desc = segment.get('description', '?')
        idx = self.plan_index
        parts = [f'#{idx} type={seg_type} desc={desc}']
        if seg_type == 'move':
            parts.append(f"L={float(segment.get('distance_m', 0.0)):.2f}m")
            parts.append(f"v={float(segment.get('speed', 0.0)):.2f}")
        elif seg_type == 'turn':
            parts.append(f"angle={float(segment.get('angle_deg', 0.0)):.0f}deg")
            if self.segment_target_yaw is not None:
                parts.append(
                    f"target_yaw={self.format_yaw_deg(self.segment_target_yaw)}deg"
                )
        elif seg_type == 'pause':
            parts.append(f"duration={float(segment.get('duration', 0.0)):.2f}s")
        parts.append(self._pose_diagnostic())
        self._log_session('SEGMENT', ' '.join(parts))

    def _log_plan_summary(self, nav_succeeded: bool) -> None:
        lines = [f'nav_succeeded={nav_succeeded} 共{len(self.plan)}段:']
        for index, segment in enumerate(self.plan):
            seg_type = segment.get('type', '?')
            desc = segment.get('description', '?')
            if seg_type == 'move':
                lines.append(
                    f'  [{index}] move {desc} '
                    f'L={float(segment.get("distance_m", 0.0)):.2f}m '
                    f'v={float(segment.get("speed", 0.0)):.2f} '
                    f'detour={bool(segment.get("allow_detour", True))}'
                )
            elif seg_type == 'turn':
                lines.append(
                    f'  [{index}] turn {desc} '
                    f'{float(segment.get("angle_deg", 0.0)):.0f}deg'
                )
            elif seg_type == 'pause':
                lines.append(
                    f'  [{index}] pause {desc} '
                    f'{float(segment.get("duration", 0.0)):.2f}s'
                )
            else:
                lines.append(f'  [{index}] {seg_type} {desc}')
        self._log_session('PLAN', '\n'.join(lines))

    def _setup_wheel_odom_position(self) -> None:
        """位姿/航向/计程/控制统一用轮速 /odom；EKF/IMU 仅诊断。"""
        self.declare_parameter('navigation_pose_source', 'wheel')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('wheel_odom_warmup_sec', 0.40)
        self.declare_parameter('wheel_odom_warmup_min_msgs', 5)
        self._navigation_pose_source = (
            str(self.get_parameter('navigation_pose_source').value).strip().lower() or 'wheel'
        )
        self._wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value).strip()
        self._wheel_odom_warmup_sec = max(
            0.0, float(self.get_parameter('wheel_odom_warmup_sec').value)
        )
        self._wheel_odom_warmup_min_msgs = max(
            1, int(self.get_parameter('wheel_odom_warmup_min_msgs').value)
        )
        self._wheel_odom_ready = False
        self._wheel_odom_msg_count = 0
        self._wheel_odom_first_rx_sec = None
        self.current_wheel_yaw = None
        self.imu_yaw = None
        self._use_wheel_odom_for_distance = bool(
            self._wheel_odom_topic and self._wheel_odom_topic != self.odom_topic
        )
        if self._use_wheel_odom_for_distance:
            self.create_subscription(
                Odometry, self._wheel_odom_topic, self._wheel_odom_callback, 10
            )
            self.get_logger().info(
                f'{self.test_feedback_prefix}统一位姿源={self._navigation_pose_source} '
                f'topic={self._wheel_odom_topic} '
                f'(xy+yaw+计程+控制同源; IMU/EKF {self.odom_topic} 仅日志; '
                f'warmup {self._wheel_odom_warmup_sec:.2f}s×'
                f'{self._wheel_odom_warmup_min_msgs}条)'
            )

    def _wheel_odom_warmed_up(self) -> bool:
        if not self._use_wheel_odom_for_distance:
            return self.current_position is not None
        if self._wheel_odom_first_rx_sec is None:
            return False
        if self._wheel_odom_msg_count < self._wheel_odom_warmup_min_msgs:
            return False
        now_sec = self.get_clock().now().nanoseconds / 1e9
        return (now_sec - self._wheel_odom_first_rx_sec) >= self._wheel_odom_warmup_sec

    def _wheel_odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        twist = msg.twist.twist
        self.current_position = (float(position.x), float(position.y))
        self.current_wheel_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self._wheel_twist = (
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.angular.z),
        )
        self._sync_unified_pose_from_wheel()
        self._wheel_odom_msg_count += 1
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self._wheel_odom_first_rx_sec is None:
            self._wheel_odom_first_rx_sec = now_sec
            self._log_session(
                'ODOM_WHEEL',
                f'首条 {self._wheel_odom_topic} '
                f'pos=({position.x:.3f},{position.y:.3f}) '
                f'yaw={self.format_yaw_deg(self.current_wheel_yaw)}deg '
                f'v=({twist.linear.x:.3f},{twist.linear.y:.3f},{twist.angular.z:.3f})',
            )
        was_ready = self._wheel_odom_ready
        if self._wheel_odom_warmed_up():
            self._wheel_odom_ready = True
        if self._wheel_odom_ready and not was_ready and not self._wheel_warmup_logged:
            self._wheel_warmup_logged = True
            elapsed = now_sec - self._wheel_odom_first_rx_sec
            self._log_session(
                'ODOM_WHEEL',
                f'warmup 完成 msgs={self._wheel_odom_msg_count} '
                f'elapsed={elapsed:.2f}s | {self._full_telemetry()}',
            )
        self._maybe_log_telemetry('wheel_odom')
        self.try_start_mission()

    def odom_callback(self, msg):
        ekf_pos = msg.pose.pose.position
        ekf_twist = msg.twist.twist
        self._last_ekf_position = (float(ekf_pos.x), float(ekf_pos.y))
        self._ekf_twist = (
            float(ekf_twist.linear.x),
            float(ekf_twist.linear.y),
            float(ekf_twist.angular.z),
        )
        self.current_odom_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_frame_id = msg.header.frame_id or self.odom_frame_id
        if not self._wheel_pose_source_active() or not self._wheel_odom_ready:
            self.current_position = self._last_ekf_position
            if not self._wheel_pose_source_active():
                self.current_yaw = self.current_odom_yaw
        self.try_start_mission()

    def projected_distance(self):
        if (
            self.segment_start_pose is None
            or self.current_position is None
            or self.segment_heading is None
        ):
            return 0.0
        dx = self.current_position[0] - self.segment_start_pose[0]
        dy = self.current_position[1] - self.segment_start_pose[1]
        along = (
            dx * math.cos(self.segment_heading)
            + dy * math.sin(self.segment_heading)
        )
        return max(0.0, along)

    def _unify_segment_pose(self, segment) -> None:
        """段起点/航向/计程轴与 current_yaw(current_position) 完全对齐。"""
        if not segment or self.current_yaw is None:
            return
        yaw = self.normalize_angle(self.current_yaw)
        seg_type = segment.get('type')

        if seg_type == 'turn':
            if 'force_start_yaw' in segment:
                self.segment_start_yaw = self.normalize_angle(
                    float(segment['force_start_yaw'])
                )
            else:
                self.segment_start_yaw = yaw
            if 'force_target_yaw' in segment:
                self.segment_target_yaw = self.normalize_angle(
                    float(segment['force_target_yaw'])
                )
            else:
                self.segment_target_yaw = self.normalize_angle(
                    self.segment_start_yaw
                    + math.radians(float(segment.get('angle_deg', 0.0)))
                )
            return

        if seg_type != 'move':
            return

        if self.current_position is not None:
            self.segment_start_pose = self.current_position
        if 'force_segment_heading' in segment:
            heading = self.normalize_angle(float(segment['force_segment_heading']))
            self.segment_heading = heading
            self.segment_start_yaw = heading
        else:
            self.segment_heading = yaw
            self.segment_start_yaw = yaw

        if self.segment_start_pose is None:
            return
        x0, y0 = self.segment_start_pose
        anchor_line = (
            f'desc={segment.get("description", "?")} '
            f'start=({x0:.3f},{y0:.3f}) yaw={self.format_yaw_deg(yaw)}deg '
            f'yaw_imu={self.format_yaw_deg(self.imu_yaw)}deg '
            f'L={float(segment.get("distance_m", 0.0)):.2f}m'
        )
        self.get_logger().info(f'{self.test_feedback_prefix}里程锚点: {anchor_line}')
        self._log_session('ODOM_ANCHOR', anchor_line)

    def _missing_pose_inputs(self):
        missing = []
        if self._wheel_pose_source_active():
            if not self._wheel_odom_warmed_up():
                missing.append(
                    f'wheel_odom({self._wheel_odom_topic} '
                    f'{self._wheel_odom_msg_count}/{self._wheel_odom_warmup_min_msgs})'
                )
            elif self.current_yaw is None:
                missing.append('wheel_yaw')
        else:
            if self.current_position is None:
                missing.append(str(self.odom_topic))
            if self.current_yaw is None:
                missing.append('imu')
        return missing

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
        line = f'{self.test_feedback_prefix}避障: {message}'
        self.get_logger().info(line)
        self._log_session('DETOUR', f'{message} | {self._pose_diagnostic()}')

    def current_segment_allows_stage1_avoidance(self):
        if not self.detour_enabled or self.current_segment is None:
            return False
        if self.current_segment.get('type') != 'move':
            return False
        if not bool(self.current_segment.get('allow_detour', True)):
            return False
        return True

    # ─── Stage1 风格边转边避状态机 ───────────────────────────────

    @property
    def _avoid_active(self) -> bool:
        return self._avoid_state != 'idle'

    def _reset_avoid(self) -> None:
        prev_state = self._avoid_state
        self._avoid_state = 'idle'
        self._avoid_plan = None
        self._avoid_leg_start_xy = None
        self._avoid_turn_accum_rad = 0.0
        self._avoid_turn_required_rad = 0.0
        self._avoid_turn_sign = 1.0
        self._avoid_prev_yaw = None
        self._avoid_turn_target_yaw = None
        self._avoid_cooldown_until = 0.0
        if prev_state != 'idle' and prev_state != self._last_avoid_state_logged:
            self._log_session(
                'AVOID',
                f'状态 {prev_state} → idle | {self._pose_diagnostic()}',
            )
            self._last_avoid_state_logged = 'idle'

    # ── 开环转角积分（过滤 IMU 跳变 >15°） ──

    def _avoid_begin_turn(self, target_yaw: float, yaw: float) -> None:
        err = self.angle_error(target_yaw, yaw)
        self._avoid_turn_required_rad = abs(err)
        self._avoid_turn_sign = math.copysign(1.0, err) if abs(err) > 1e-6 else 1.0
        self._avoid_turn_accum_rad = 0.0
        self._avoid_prev_yaw = yaw
        self._avoid_turn_target_yaw = target_yaw

    def _avoid_accumulate_turn(self, yaw: float) -> None:
        if self._avoid_prev_yaw is None:
            self._avoid_prev_yaw = yaw
            return
        # normalize(current - prev)：正值=左转，负值=右转
        step = self.angle_error(yaw, self._avoid_prev_yaw)
        self._avoid_prev_yaw = yaw
        # 单帧 >15° 视为 IMU 跳变，不计入开环转角
        if abs(step) > math.radians(15.0):
            return
        # step 与 turn_sign 同号才累计（机器人确实在朝目标方向转）
        if self._avoid_turn_sign * step > 0.0:
            self._avoid_turn_accum_rad += abs(step)

    def _avoid_turn_done(self) -> bool:
        need = max(0.0, self._avoid_turn_required_rad - math.radians(2.0))
        return self._avoid_turn_accum_rad >= need

    # ── 直行距离计数 ──

    def _avoid_mark_leg_start(self, x: float, y: float) -> None:
        self._avoid_leg_start_xy = (x, y)

    def _avoid_leg_traveled_m(self, x: float, y: float) -> float:
        if self._avoid_leg_start_xy is None:
            return 0.0
        dx = x - self._avoid_leg_start_xy[0]
        dy = y - self._avoid_leg_start_xy[1]
        return math.hypot(dx, dy)

    def _avoid_leg_done(self, x: float, y: float, target_m: float) -> bool:
        return self._avoid_leg_traveled_m(x, y) >= target_m - self._avoid_distance_tol_m

    # ── 触发判断 ──

    def _estimate_avoid_projection_m(self) -> float:
        """估算完整避障沿段航向 vx 方向消耗的投影里程（m）。
        用于在触发前预判剩余距离是否够用。
        """
        offset = self._avoid_offset_rad
        vt = self._avoid_turn_linear_speed
        w = self._avoid_turn_angular_speed
        leg1 = self._avoid_leg1_distance_m
        leg2 = self._avoid_leg2_distance_m

        if w < 1e-6:
            return leg1 + leg2 + 0.5

        # 转向阶段精确投影 = (vt/w) * ∫cos(θ)dθ
        ta_proj = (vt / w) * math.sin(offset)                     # TURN_AWAY：0 → +offset
        leg1_proj = leg1 * math.cos(offset)                        # LEG1
        tb_proj = (vt / w) * (math.sin(offset) - math.sin(-offset))  # TURN_BACK：+offset → -offset
        leg2_proj = leg2 * math.cos(offset)                        # LEG2
        tr_proj = (vt / w) * (0.0 - math.sin(-offset))             # TURN_RECOVER：-offset → 0
        fine_proj = 0.02                                           # FINE_ALIGN 小修正
        return ta_proj + leg1_proj + tb_proj + leg2_proj + tr_proj + fine_proj

    def _should_trigger_avoid(self) -> bool:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec < self._avoid_cooldown_until:
            return False
        if not self.current_segment_allows_stage1_avoidance():
            return False
        if not math.isfinite(self.front_obstacle_distance):
            return False
        if self.front_obstacle_distance > self.detour_obstacle_distance:
            return False
        nav_yaw = self.navigation_yaw()
        if self.segment_heading is not None and nav_yaw is not None:
            if abs(self.angle_error(self.segment_heading, nav_yaw)) > self.detour_heading_gate_rad:
                return False

        # 预计算剩余里程是否够完成完整避障 zigzag
        if self.current_segment is not None and self.current_segment.get('type') == 'move':
            remaining = float(self.current_segment['distance_m']) - self.projected_distance()
            needed = self._estimate_avoid_projection_m()
            if remaining < needed - 0.05:  # 宽容 5cm（估算 vs 实际差异）
                self.log_detour(
                    f'剩余里程 {remaining:.2f}m 不足（避障需约 {needed:.2f}m），跳过触发'
                )
                return False
        return True

    # ── 启动避障 ──

    def _start_avoid(self, x: float, y: float, yaw: float) -> None:
        if self.segment_heading is None or self.segment_start_pose is None:
            self.log_detour('避障未启动：缺少段航向或段起点')
            self._avoid_state = 'idle'
            return

        obstacle_left = obstacle_is_left(self.front_obstacle_angle_deg)
        plan = build_s1_plan(
            psi0_rad=self.segment_heading,
            leg1_distance_m=self._avoid_leg1_distance_m,
            leg2_distance_m=self._avoid_leg2_distance_m,
            offset_rad=self._avoid_offset_rad,
            obstacle_left=obstacle_left,
        )
        self._avoid_plan = plan
        self._avoid_state = 'turn_away'
        self._avoid_begin_turn(plan.psi1, yaw)
        dir_text = '左' if obstacle_left else '右'
        self.log_detour(
            f'Stage1 边转边避启动 obstacle={dir_text} '
            f'ψ₀={math.degrees(plan.psi0):.1f}° '
            f'→ ψ₁={math.degrees(plan.psi1):.1f}° '
            f'→ ψ₂={math.degrees(plan.psi2):.1f}° '
            f'L₁={plan.leg1_distance_m:.2f}m L₂={plan.leg2_distance_m:.2f}m'
        )

    # ── 主步进 ──

    def _try_avoid_step(self) -> bool:
        """Stage1 风格边转边避状态机。

        Returns:
            True  — 避障接管了本帧控制（调用方应 return 不执行正常直行）
            False — 避障未激活，正常控制继续
        """
        nav_yaw = self.navigation_yaw()
        if self.current_position is None or nav_yaw is None:
            if self._avoid_active:
                self.cmd_pub.publish(self.create_twist())
            return self._avoid_active

        x, y = self.current_position
        yaw = nav_yaw

        # ── IDLE：检查触发 ──
        if self._avoid_state == 'idle':
            if not self._should_trigger_avoid():
                return False
            if self.segment_heading is None or self.segment_start_pose is None:
                self.log_detour('避障未启动：缺少段航向或段起点')
                return False
            self._start_avoid(x, y, yaw)
            if self._avoid_state != self._last_avoid_state_logged:
                self._last_avoid_state_logged = self._avoid_state
            # fall through 执行首帧

        plan = self._avoid_plan
        if plan is None:
            self._reset_avoid()
            return False

        # ── 里程超限：避障中已超过段终点 → 终止避障，段自然过渡 ──
        if self._avoid_state not in ('idle', 'fine_align'):
            if self.current_segment is not None and self.current_segment.get('type') == 'move':
                target_m = float(self.current_segment['distance_m'])
                if self.projected_distance() >= target_m - self.distance_tolerance:
                    progress = self.projected_distance()
                    self.log_detour(
                        f'里程超限 {progress:.2f}/{target_m:.2f}m → '
                        f'终止避障，段自然过渡到下段'
                    )
                    # 预置下段起始偏航角 = 段航向（避免当前偏航角污染转弯目标）
                    if hasattr(self, 'pending_segment_start_yaw'):
                        self.pending_segment_start_yaw = self.segment_heading
                    self._reset_avoid()
                    return False

        # ── TURN_AWAY：转向 offset 角，边减速前进 ──
        if self._avoid_state == 'turn_away':
            if self._avoid_turn_required_rad <= 0.0:
                self._avoid_begin_turn(plan.psi1, yaw)
            self._avoid_accumulate_turn(yaw)
            omega = self._avoid_turn_sign * self._avoid_turn_angular_speed
            self.cmd_pub.publish(self.create_twist(self._avoid_turn_linear_speed, omega))
            if self._avoid_turn_done():
                self.log_detour(
                    f'TURN_AWAY 完成 yaw={math.degrees(yaw):.1f}° '
                    f'(目标 ψ₁={math.degrees(plan.psi1):.1f}°) → LEG1 {plan.leg1_distance_m:.2f}m'
                )
                self._avoid_state = 'leg1'
                self._avoid_mark_leg_start(x, y)
            return True

        # ── LEG1：直行 ω=0，不纠航 ──
        if self._avoid_state == 'leg1':
            self.cmd_pub.publish(self.create_twist(self._avoid_leg_linear_speed, 0.0))
            if self._avoid_leg_done(x, y, plan.leg1_distance_m):
                self.log_detour(
                    f'LEG1 完成 {self._avoid_leg_traveled_m(x, y):.2f}m → TURN_BACK ψ₂={math.degrees(plan.psi2):.1f}°'
                )
                self._avoid_state = 'turn_back'
                self._avoid_begin_turn(plan.psi2, yaw)
            return True

        # ── TURN_BACK：反向转 2*offset，减速前进 ──
        if self._avoid_state == 'turn_back':
            if self._avoid_turn_required_rad <= 0.0:
                self._avoid_begin_turn(plan.psi2, yaw)
            self._avoid_accumulate_turn(yaw)
            omega = self._avoid_turn_sign * self._avoid_turn_angular_speed
            self.cmd_pub.publish(self.create_twist(self._avoid_turn_linear_speed, omega))
            if self._avoid_turn_done():
                self.log_detour(
                    f'TURN_BACK 完成 yaw={math.degrees(yaw):.1f}° '
                    f'(目标 ψ₂={math.degrees(plan.psi2):.1f}°) → LEG2 {plan.leg2_distance_m:.2f}m'
                )
                self._avoid_state = 'leg2'
                self._avoid_mark_leg_start(x, y)
            return True

        # ── LEG2：直行 ω=0 ──
        if self._avoid_state == 'leg2':
            self.cmd_pub.publish(self.create_twist(self._avoid_leg_linear_speed, 0.0))
            if self._avoid_leg_done(x, y, plan.leg2_distance_m):
                self.log_detour(
                    f'LEG2 完成 {self._avoid_leg_traveled_m(x, y):.2f}m → TURN_RECOVER ψ₀={math.degrees(plan.psi0):.1f}°'
                )
                self._avoid_state = 'turn_recover'
                self._avoid_begin_turn(plan.psi0, yaw)
            return True

        # ── TURN_RECOVER：开环回身 offset，减速前进 ──
        if self._avoid_state == 'turn_recover':
            if self._avoid_turn_required_rad <= 0.0:
                self._avoid_begin_turn(plan.psi0, yaw)
            self._avoid_accumulate_turn(yaw)
            omega = self._avoid_turn_sign * self._avoid_turn_angular_speed
            self.cmd_pub.publish(self.create_twist(self._avoid_turn_linear_speed, omega))
            if self._avoid_turn_done():
                err_deg = math.degrees(abs(self.angle_error(plan.psi0, yaw)))
                self.log_detour(
                    f'TURN_RECOVER 开环完成 yaw={math.degrees(yaw):.1f}° '
                    f'(ψ₀={math.degrees(plan.psi0):.1f}° err={err_deg:.1f}°) → FINE_ALIGN 闭环修正'
                )
                self._avoid_state = 'fine_align'
            return True

        # ── FINE_ALIGN：闭环 KP 修正残余航向偏差 ──
        if self._avoid_state == 'fine_align':
            err = self.angle_error(plan.psi0, yaw)
            if abs(err) <= self.heading_tolerance:
                self._avoid_cooldown_until = (
                    self.get_clock().now().nanoseconds / 1e9 + self.detour_cooldown_sec
                )
                self.log_detour(
                    f'FINE_ALIGN 完成 yaw={math.degrees(yaw):.1f}° '
                    f'(ψ₀={math.degrees(plan.psi0):.1f}° err={math.degrees(err):.1f}°) → 避障结束 '
                    f'(冷却 {self.detour_cooldown_sec:.0f}s)'
                )
                self._avoid_state = 'idle'
                self._avoid_plan = None
                self.cmd_pub.publish(self.create_twist())
            else:
                omega = self.clamp(self.heading_kp * err, self._avoid_turn_angular_speed)
                if abs(omega) < 0.1:
                    omega = math.copysign(0.1, err)
                self.cmd_pub.publish(self.create_twist(self._avoid_turn_linear_speed, omega))
            return True

        return self._avoid_active

    # ─── 段控制覆盖 ───────────────────────────────────────────────

    def begin_inertial_plan_after_nav(self, nav_succeeded):
        self._reset_avoid()
        self._sync_unified_pose_from_wheel()
        super().begin_inertial_plan_after_nav(nav_succeeded)
        self._log_plan_summary(nav_succeeded)

    def reset_mission(self, clear_task):
        self.detour_detection_locked = False
        self.detour_resume_yaw = None
        self._reset_avoid()
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
        self._reset_avoid()

        if self.current_segment is None or self.plan_index != index:
            return

        segment = self.current_segment
        segment_type = segment.get('type')
        self._sync_unified_pose_from_wheel()
        if segment_type == 'turn' and 'heading_tolerance_rad' in segment:
            self.active_turn_heading_tolerance = max(
                1e-3, float(segment['heading_tolerance_rad'])
            )
        self._unify_segment_pose(segment)

        self._log_segment_enter(segment)
        if self._avoid_state != self._last_avoid_state_logged:
            self._last_avoid_state_logged = self._avoid_state

        label = self.rectangle_segment_label(segment)

        if self.is_detour_segment(segment):
            if segment_type == 'turn':
                self.log_detour(
                    f'进入 {segment.get("description", "detour_turn")}，'
                    f'yaw={self.format_yaw_deg(self.current_yaw)}deg，'
                    f'start_yaw={self.format_yaw_deg(self.segment_start_yaw)}deg，'
                    f'target_yaw={self.format_yaw_deg(self.segment_target_yaw)}deg，'
                    f'tol={math.degrees(self.active_turn_heading_tolerance):.1f}deg'
                )
            elif segment_type == 'move':
                self.log_detour(
                    f'进入 {segment.get("description", "detour_move")}，'
                    f'distance={float(segment.get("distance_m", 0.0)):.2f}m，'
                    f'heading={self.format_yaw_deg(self.segment_heading)}deg，'
                    f'yaw={self.format_yaw_deg(self.current_yaw)}deg'
                )
            elif segment_type == 'pause':
                self.log_detour(
                    f'进入 {segment.get("description", "detour_pause")}，'
                    f'duration={float(segment.get("duration", 0.0)):.2f}s，'
                    f'yaw={self.format_yaw_deg(self.current_yaw)}deg'
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

        nav_yaw = self.navigation_yaw()
        if self.segment_heading is not None and nav_yaw is not None:
            heading_error = self.angle_error(self.segment_heading, nav_yaw)
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


    def _compute_move_lateral_angular(self) -> float:
        """计算直行段横向角速度。子类可覆盖以替换视觉居中。"""
        if self.current_position is None or self.segment_heading is None:
            return 0.0
        nav_yaw = self.navigation_yaw()
        if nav_yaw is None:
            return 0.0
        heading_error = self.angle_error(self.segment_heading, nav_yaw)
        return self.clamp(self.heading_kp * heading_error, self.max_angular_speed)

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
                    label = self.rectangle_segment_label(self.current_segment)
                    progress_line = (
                        f'{label} 进度 {bucket * 25}% '
                        f'({progress:.2f}/{target_distance:.2f}m)'
                    )
                    self.get_logger().info(
                        f'{self.test_feedback_prefix}当前位置: {progress_line}'
                    )
                    self._log_session(
                        'PROGRESS',
                        f'{progress_line} | {self._pose_diagnostic()}',
                    )

            if progress >= target_distance - self.distance_tolerance and self.last_progress_bucket < 4:
                self.last_progress_bucket = 4
                self.publish_feedback(
                    f'{self.test_feedback_prefix}当前位置: '
                    f'{self.rectangle_segment_label(self.current_segment)}，'
                    f'直行到位，准备切换到下一段'
                )

        # Stage1 风格边转边避（替代旧的 run_stage1_style_obstacle_avoidance）
        if self._try_avoid_step():
            if self._avoid_state != self._last_avoid_state_logged:
                self._log_session(
                    'AVOID',
                    f'状态 → {self._avoid_state} | {self._pose_diagnostic()}',
                )
                self._last_avoid_state_logged = self._avoid_state
            self._maybe_log_telemetry('avoid_active')
            return

        if self.current_position is None or self.segment_heading is None:
            self.cmd_pub.publish(self.create_twist())
            self._maybe_log_telemetry('move_no_pose')
            return

        progress = self.projected_distance()
        target_distance = float(self.current_segment['distance_m'])
        if progress >= target_distance - self.distance_tolerance:
            self._log_session(
                'SEGMENT_DONE',
                f'{self.current_segment.get("description", "?")} '
                f'{progress:.3f}/{target_distance:.2f}m | {self._pose_diagnostic()}',
            )
            self.cmd_pub.publish(self.create_twist())
            self.start_segment(self.plan_index + 1)
            return

        angular = self._compute_move_lateral_angular()
        linear = float(self.current_segment.get('speed', self.corridor_linear_speed))
        self.cmd_pub.publish(self.create_twist(linear, angular))
        self._maybe_log_telemetry('move')

    def run_turn_segment(self):
        turn_tolerance = self.active_turn_heading_tolerance
        linear_speed = float(
            (self.current_segment or {}).get('turn_linear_speed', self.turn_linear_speed)
        )

        # 转角障碍检测：前方障碍过近 → 原地转向不前移
        if (math.isfinite(self.front_obstacle_distance)
                and self.front_obstacle_distance < self.detour_obstacle_distance * 0.5):
            linear_speed = 0.0

        nav_yaw = self.navigation_yaw()
        if nav_yaw is None or self.segment_target_yaw is None:
            self.cmd_pub.publish(self.create_twist())
            return

        error = self.angle_error(self.segment_target_yaw, nav_yaw)
        if abs(error) <= turn_tolerance:
            if self.is_detour_segment(self.current_segment):
                description = str((self.current_segment or {}).get('description', ''))
                self.log_detour(
                    f'完成 {self.current_segment.get("description", "detour_turn")}，'
                    f'nav_yaw={self.format_yaw_deg(nav_yaw)}deg，'
                    f'target_yaw={self.format_yaw_deg(self.segment_target_yaw)}deg，'
                    f'error={math.degrees(error):.2f}deg'
                )
                if description.endswith('_resume_align'):
                    self.detour_detection_locked = False
                    self.log_detour(
                        f'已回到避障前yaw，恢复障碍检测，resume_yaw={self.format_yaw_deg(self.detour_resume_yaw)}deg，'
                        f'nav_yaw={self.format_yaw_deg(nav_yaw)}deg'
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
                    f'nav_yaw={self.format_yaw_deg(nav_yaw)}deg，'
                    f'target_yaw={self.format_yaw_deg(self.segment_target_yaw)}deg，'
                    f'error={math.degrees(error):.2f}deg，'
                    f'angular={angular:.2f}rad/s，'
                    f'linear={linear_speed:.2f}m/s'
                )

        self.cmd_pub.publish(self.create_twist(linear_speed, angular))
        self._maybe_log_telemetry(
            f'turn err={math.degrees(error):.1f}deg'
        )

    def finish_mission(self):
        self._log_session('MISSION', f'完成 | {self._pose_diagnostic()}')
        super().finish_mission()

    def control_loop(self):
        if self.corridor_path_active:
            self.run_corridor_path_stage()
            return

        if not self.mission_active or self.current_segment is None:
            if not self.mission_active:
                self.cmd_pub.publish(self.create_twist())
            self._maybe_log_telemetry('idle')
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if (
            not self._avoid_active
            and self.segment_started_at is not None
            and now_sec - self.segment_started_at > self.segment_timeout
        ):
            desc = self.current_segment.get('description', 'unknown')
            self._log_session(
                'TIMEOUT',
                f'段超时 {desc} | {self._pose_diagnostic()}',
            )
            self.publish_feedback(
                f'{self.test_feedback_prefix}段超时: {desc}'
            )
            self._reset_avoid()
            self.start_segment(self.plan_index + 1)
            return

        segment_type = self.current_segment['type']
        if segment_type == 'turn':
            self.run_turn_segment()
        elif segment_type == 'move':
            self.run_move_segment()
        elif segment_type == 'pause':
            self.run_pause_segment(now_sec)
            self._maybe_log_telemetry('pause')
        else:
            self.start_segment(self.plan_index + 1)

    def build_ring_plan(self):
        # 5 段转角独立 yaml 可调：rect_enter_align_deg / rect_corner_{1,2,3,4}_deg
        sign = 1.0 if self.direction == 'clockwise' else -1.0
        entry_turn = sign * self.rect_enter_align_deg
        corner1_turn = -sign * self.rect_corner_1_deg
        corner2_turn = -sign * self.rect_corner_2_deg
        corner3_turn = -sign * self.rect_corner_3_deg
        corner4_turn = -sign * self.rect_corner_4_deg

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
                'angle_deg': corner1_turn,
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
                'angle_deg': corner2_turn,
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
                'angle_deg': corner3_turn,
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
                'angle_deg': corner4_turn,
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

        missing_inputs = self._missing_pose_inputs()

        if missing_inputs:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self._last_wait_log_sec >= 3.0:
                self._last_wait_log_sec = now_sec
                wait_line = f'等待: {", ".join(missing_inputs)}'
                if not self.reported_waiting_pose:
                    self.publish_feedback(
                        f'{self.test_feedback_prefix}等待输入就绪: '
                        f'{", ".join(missing_inputs)}'
                    )
                    self.reported_waiting_pose = True
                self._log_session('STARTUP', f'{wait_line} | {self._full_telemetry()}')
            self._maybe_log_telemetry('startup_wait')
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.start_after_time is None:
            self.start_after_time = current_time + self.start_delay_sec
            if not self.reported_start_delay:
                ready_line = (
                    f'位姿就绪 delay={self.start_delay_sec:.2f}s | '
                    f'{self._pose_diagnostic()}'
                )
                self.publish_feedback(
                    f'{self.test_feedback_prefix}位姿已就绪，'
                    f'{self.start_delay_sec:.2f}s 后开始'
                )
                self._log_session('STARTUP', ready_line)
                self.reported_start_delay = True
            return

        if current_time < self.start_after_time:
            return

        self.mission_active = True
        self.reported_start = True
        self._log_session(
            'MISSION',
            f'任务开始 方向={self.direction_text()} | {self._pose_diagnostic()}',
        )
        self.publish_feedback(
            f'{self.test_feedback_prefix}开始执行，方向: {self.direction_text()}，'
            f'模式: {self.start_mode_text()}，'
            f'矩形圈: 左/右横边{self.rectangle_first_leg_m:.2f}m，'
            f'竖边{self.rectangle_side_leg_m:.2f}m，'
            f'顶部横边{self.rectangle_top_leg_m:.2f}m'
        )
        self.begin_inertial_plan_after_nav(nav_succeeded=self.nav_succeeded_for_test_start())


def main(args=None):
    import threading

    from racing_stage2_param_test.cmd_vel_stop import (
        init_without_ros_signal_handler,
        install_stop_event,
        publish_stop,
        spin_until_stop,
    )

    init_without_ros_signal_handler(args)
    node = DirectInertialTester()
    stop_event = threading.Event()

    request_stop = install_stop_event(
        stop_event,
        lambda: publish_stop(node.cmd_pub),
        cli_topics=['/cmd_vel', '/stage2_cmd_vel'],
    )

    try:
        spin_until_stop(node, stop_event)
    except KeyboardInterrupt:
        request_stop()
    finally:
        request_stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

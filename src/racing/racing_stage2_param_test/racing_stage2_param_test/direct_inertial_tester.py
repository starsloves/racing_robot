import math

import rclpy
from nav_msgs.msg import Odometry

from ament_index_python import get_package_share_directory
from racing_stage2_param_test import field_track
from racing_stage2.stage2_inertial_navigator import Stage2InertialNavigator
from racing_stage2_param_test.scan_processor import ScanProcessor
from racing_common.racing_logger import RacingLogger
from racing_stage2_param_test.spiral_avoider import SpiralAvoider, SpiralConfig


class DirectInertialTester(Stage2InertialNavigator):
    def __init__(self):
        super().__init__()

        self.declare_parameter('test_direction', 'clockwise')
        self.declare_parameter('test_start_mode', 'auto')
        # field_track yaml 路径。空=根据 direction 自动选择 config/field_track_{direction}.yaml
        self.declare_parameter('field_track_yaml', '')
        
        # ═══ 新避障模块参数 ═══════════════════════════════
        self.declare_parameter('avoidance_mode', 'constant_curvature')  # 'spiral' 或 'constant_curvature'
        self.declare_parameter('side_detour_threshold_m', 0.25)  # 侧边触发阈值
        # 螺旋方案参数
        self.declare_parameter('spiral_linear_speed', 0.15)
        self.declare_parameter('spiral_angular_initial', 2.0)
        self.declare_parameter('spiral_total_time', 2.0)
        # 恒定曲率方案参数
        self.declare_parameter('constant_angular_speed', 0.70)
        self.declare_parameter('constant_delta_theta_deg', 30.0)
        self.declare_parameter('constant_heading_tolerance_deg', 2.0)
        # Phase2 提前退出参数
        self.declare_parameter('phase2_exit_overshoot_deg', 8.0)
        # 固定转角方案参数
        self.declare_parameter('avoidance_turn_angle_deg', 30.0)
        self.declare_parameter('avoidance_phase1_duration_s', 0.8)
        self.declare_parameter('avoidance_phase2_turn_angle_deg', 30.0)
        self.declare_parameter('avoidance_phase2_duration_s', 0.8)
        self.declare_parameter('enable_phase3', False)
        self.declare_parameter('avoidance_phase3_turn_angle_deg', 10.0)
        self.declare_parameter('avoidance_phase3_duration_s', 0.5)
        self.declare_parameter('avoidance_heading_kp', 2.0)
        self.declare_parameter('avoidance_cooldown_sec', 2.0)
        self.declare_parameter('avoidance_trigger_distance_m', 0.55)
        # 调试停车参数
        self.declare_parameter('stage_pause_enabled', False)
        self.declare_parameter('stage_pause_duration_sec', 5.0)
        self.declare_parameter('stage_pause_on_trigger', False)
        self.declare_parameter('stage_pause_on_phase1_end', False)
        self.declare_parameter('stage_pause_on_phase2_end', False)
        self.declare_parameter('stage_pause_on_phase3_end', False)

        self.test_direction_raw = str(self.get_parameter('test_direction').value).strip()
        self.test_direction = self.resolve_test_direction(self.test_direction_raw)
        self.test_start_mode = str(self.get_parameter('test_start_mode').value).strip().lower() or 'auto'
        ft_custom = str(self.get_parameter('field_track_yaml').value)
        if ft_custom:
            self._field_track_yaml = ft_custom
        else:
            pkg_dir = get_package_share_directory('racing_stage2_param_test')
            self._field_track_yaml = field_track.resolve_yaml_path(pkg_dir, self.test_direction, '')

        self.phase = 2
        self.task_raw = self.test_direction_raw
        self.direction = self.test_direction

        self.reported_waiting_pose = False
        self.reported_start_delay = False
        self.last_progress_bucket = -1
        self.active_turn_heading_tolerance = self.heading_tolerance

        # 雷达处理模块
        self._scan_processor = ScanProcessor(
            front_angle_deg=self.detour_front_angle_deg,
            side_window_deg=self.detour_side_window_deg,
            side_center_deg=self.detour_side_center_deg,
        )
        self.front_obstacle_distance = float('inf')
        self.front_obstacle_angle_deg = 0.0
        self.left_clearance_distance = float('inf')
        self.right_clearance_distance = float('inf')

        # ═══ 新避障模块（默认启用）═══════════════════════
        _spiral_cfg = SpiralConfig(
            mode=str(self.get_parameter('avoidance_mode').value),
            linear_speed=float(self.get_parameter('spiral_linear_speed').value),
            angular_initial=float(self.get_parameter('spiral_angular_initial').value),
            total_time=float(self.get_parameter('spiral_total_time').value),
            constant_angular=float(self.get_parameter('constant_angular_speed').value),
            constant_delta_theta_deg=float(self.get_parameter('constant_delta_theta_deg').value),
            heading_tolerance_deg=float(self.get_parameter('constant_heading_tolerance_deg').value),
            phase2_exit_overshoot_deg=float(self.get_parameter('phase2_exit_overshoot_deg').value),
            # 固定转角方案参数
            phase1_turn_angle_deg=float(self.get_parameter('avoidance_turn_angle_deg').value),
            phase1_duration_s=float(self.get_parameter('avoidance_phase1_duration_s').value),
            phase2_turn_angle_deg=float(self.get_parameter('avoidance_phase2_turn_angle_deg').value),
            phase2_duration_s=float(self.get_parameter('avoidance_phase2_duration_s').value),
            enable_phase3=bool(self.get_parameter('enable_phase3').value),
            phase3_turn_angle_deg=float(self.get_parameter('avoidance_phase3_turn_angle_deg').value),
            phase3_duration_s=float(self.get_parameter('avoidance_phase3_duration_s').value),
            heading_control_kp=float(self.get_parameter('avoidance_heading_kp').value),
            side_obstacle_threshold_m=float(self.get_parameter('side_detour_threshold_m').value),
            # 调试停车参数
            stage_pause_enabled=bool(self.get_parameter('stage_pause_enabled').value),
            stage_pause_duration_sec=float(self.get_parameter('stage_pause_duration_sec').value),
            stage_pause_on_trigger=bool(self.get_parameter('stage_pause_on_trigger').value),
            stage_pause_on_phase1_end=bool(self.get_parameter('stage_pause_on_phase1_end').value),
            stage_pause_on_phase2_end=bool(self.get_parameter('stage_pause_on_phase2_end').value),
            stage_pause_on_phase3_end=bool(self.get_parameter('stage_pause_on_phase3_end').value),
            verbose=True,
        )
        # 冷却时间（avoider 外控制，用节点自己的属性）
        self._avoidance_cooldown_sec = float(self.get_parameter('avoidance_cooldown_sec').value)
        self._avoidance_trigger_dist = float(self.get_parameter('avoidance_trigger_distance_m').value)
        self._avoidance_cooldown_until = 0.0
        # 日志模块必须在避障模块之前创建，为避障提供 RacingLogger
        self._spiral_cfg_for_log = _spiral_cfg
        self._setup_logger()
        self._setup_wheel_odom_position()

        self._spiral_avoider = SpiralAvoider(
            self.cmd_pub,
            self.logger,
            self.get_clock(),
            _spiral_cfg,
            cmd_observer=self._observe_avoid_cmd,
        )

        self.logger.config(
            f'方向={self.direction_text()} 模式={self.start_mode_text()} '
            f'field_track={self._field_track_yaml} '
            f'pose_source={self._navigation_pose_source} '
            f'wheel={self._wheel_odom_topic} ekf={self.odom_topic} '
            f'ring_v={self.ring_linear_speed:.2f} turn_v={self.turn_linear_speed:.2f} '
            f'turn_w={self.turn_angular_speed:.2f} head_kp={self.heading_kp:.2f} '
            f'dist_tol={self.distance_tolerance:.3f} '
            f'head_tol={math.degrees(self.heading_tolerance):.1f}deg '
            f'segment_timeout={self.segment_timeout:.1f}s',
        )

    def _setup_logger(self) -> None:
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
        self.logger = RacingLogger(self, subdir, filename,
                                   session_title='direct inertial test session')
        self._last_telemetry_sec = 0.0
        self._last_wait_log_sec = 0.0
        self._wheel_warmup_logged = False
        self._last_ekf_position = None
        self._wheel_twist = None
        self._ekf_twist = None
        self._last_cmd_linear = 0.0
        self._last_cmd_angular = 0.0
        self.logger.info('LOGGER', f'日志路径: {self.logger.path}')
        
        # 输出避障模块配置（延迟到此处）
        _cfg = self._spiral_cfg_for_log
        mode_text = _cfg.mode
        if mode_text == 'fixed_timing':
            self.logger.info('AVOID',
                f'新避障模块 '
                f'模式=固定转角 '
                f'Phase1转角={_cfg.phase1_turn_angle_deg:.0f}° '
                f'Phase1时长={_cfg.phase1_duration_s:.1f}s '
                f'Phase2转角={_cfg.phase2_turn_angle_deg:.0f}° '
                f'Phase2时长={_cfg.phase2_duration_s:.1f}s '
                f'v={_cfg.linear_speed:.2f}m/s '
                f'冷却={self._avoidance_cooldown_sec:.1f}s'
            )
        else:
            self.logger.info('AVOID',
                f'新避障模块 '
                f'模式={_cfg.mode} '
                f'v={_cfg.linear_speed:.2f}m/s '
                + (f'ω_init={_cfg.angular_initial:.2f}rad/s T={_cfg.total_time:.1f}s'
                   if _cfg.mode == 'spiral'
                   else f'ω={_cfg.constant_angular:.2f}rad/s Δθ={_cfg.constant_delta_theta_deg:.0f}°')
            )

    def destroy_node(self):
        if getattr(self, 'logger', None) is not None:
            self.logger.close()
            self.logger = None
        super().destroy_node()

    def publish_feedback(self, text: str) -> None:
        super().publish_feedback(text)

    def create_twist(self, linear_x=0.0, angular_z=0.0):
        self._last_cmd_linear = float(linear_x)
        self._last_cmd_angular = float(angular_z)
        return super().create_twist(linear_x, angular_z)

    def _observe_avoid_cmd(self, linear_x, angular_z):
        """避障模块真实命令同步到日志"""
        self._last_cmd_linear = float(linear_x)
        self._last_cmd_angular = float(angular_z)

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
        """计算横向偏离（垂直于段方向的距离）"""
        if (
            self.segment_start_pose is None
            or self.current_position is None
            or self.segment_heading is None
        ):
            return 0.0
        # 内联 cross_segment_m 计算
        dx = self.current_position[0] - self.segment_start_pose[0]
        dy = self.current_position[1] - self.segment_start_pose[1]
        return -dx * math.sin(self.segment_heading) + dy * math.cos(self.segment_heading)

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
                f'front={self.format_distance(self.front_obstacle_distance)}m '
                f'@ {self._fmt_num(self.front_obstacle_angle_deg, prec=1)}deg '
                f'left={self.format_distance(self.left_clearance_distance)}m '
                f'right={self.format_distance(self.right_clearance_distance)}m '
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
        self.logger.telemetry(reason, self._full_telemetry())

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
        is_turn = seg_type == 'turn'
        self.logger.info('SEGMENT', ' '.join(parts), file_only=True)
        if is_turn:
            desc = segment.get('description', '?')
            angle = float(segment.get('angle_deg', 0.0))
            self.logger.segment(f'#{idx} turn {desc} {angle:.0f}deg')

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
        self.logger.plan('\n'.join(lines))

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
            self.logger.info('POSE',
                f'统一位姿源={self._navigation_pose_source} '
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
            self.logger.odom_wheel(
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
            self.logger.odom_wheel(
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
        self.logger.info('ANCHOR', anchor_line)
        self.logger.odom_anchor(anchor_line)

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

        self.logger.warn('DIRECTION',
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

    # ─── 段控制覆盖 ───────────────────────────────────────────────

    def begin_inertial_plan_after_nav(self, nav_succeeded):
        self._sync_unified_pose_from_wheel()
        super().begin_inertial_plan_after_nav(nav_succeeded)
        self._log_plan_summary(nav_succeeded)

    def reset_mission(self, clear_task):
        super().reset_mission(clear_task)

    def rectangle_segment_label(self, segment):
        description = str(segment.get('description', 'unknown'))

        if self.direction == 'clockwise':
            d = segment.get('distance_m', 0)
            labels = {
                'rect_enter_align': '通道后起点入口对齐',
                'rect_first_leg': f'底边向左 {d:.2f}m 段',
                'rect_corner_1': '左下拐角',
                'rect_side_1': f'左边向上 {d:.2f}m 段',
                'rect_corner_2': '左上拐角',
                'rect_top': f'顶边向右 {d:.2f}m 段',
                'rect_corner_3': '右上拐角',
                'rect_side_2': f'右边向下 {d:.2f}m 段',
                'rect_corner_4': '右下拐角',
                'rect_return_origin': f'底边回起点 {d:.2f}m 段',
            }
        else:
            d = segment.get('distance_m', 0)
            labels = {
                'rect_enter_align': '通道后起点入口对齐',
                'rect_first_leg': f'底边向右 {d:.2f}m 段',
                'rect_corner_1': '右下拐角',
                'rect_side_1': f'右边向上 {d:.2f}m 段',
                'rect_corner_2': '右上拐角',
                'rect_top': f'顶边向左 {d:.2f}m 段',
                'rect_corner_3': '左上拐角',
                'rect_side_2': f'左边向下 {d:.2f}m 段',
                'rect_corner_4': '左下拐角',
                'rect_return_origin': f'底边回起点 {d:.2f}m 段',
            }
        return labels.get(description, description)

    def start_segment(self, index):
        super().start_segment(index)
        self.last_progress_bucket = -1
        self.active_turn_heading_tolerance = self.heading_tolerance
        
        # 重置避障模块
        self._spiral_avoider.reset()
        
        # 新段新锚点，重置避障冷却
        self._avoidance_cooldown_until = 0.0

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

        label = self.rectangle_segment_label(segment)

        if segment_type == 'turn':
            angle_deg = float(segment.get('angle_deg', 0.0))
            turn_text = '左转' if angle_deg > 0.0 else '右转'
            self.publish_feedback(
                f'当前位置: {label}，开始{turn_text} {abs(angle_deg):.0f} 度'
            )
            return

        if segment_type == 'move':
            distance_m = float(segment.get('distance_m', 0.0))
            self.publish_feedback(
                f'当前位置: {label}，目标直行 {distance_m:.2f}m'
            )
            return

        if segment_type == 'pause':
            self.publish_feedback(f'当前位置: {label}，短暂停稳')

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_frame_id = msg.header.frame_id
        data = self._scan_processor.process(msg)
        self.front_obstacle_distance = data.front_distance
        self.front_obstacle_angle_deg = data.front_angle_deg
        self.left_clearance_distance = data.left_clearance
        self.right_clearance_distance = data.right_clearance
        
        # 更新避障模块的传感器数据
        self._spiral_avoider.on_scan(
            data.front_distance, data.front_angle_deg,
            data.left_clearance, data.right_clearance,
        )

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
        # ★ 更新避障模块的横偏数据（用于 Phase2 斜向回归计算）
        if self.current_position is not None:
            cross_m = self._cross_track_m()
            self._spiral_avoider.on_cross_error(cross_m)
        
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
                    self.logger.progress(f'{progress_line} | {self._pose_diagnostic()}')

            if progress >= target_distance - self.distance_tolerance and self.last_progress_bucket < 4:
                self.last_progress_bucket = 4
                self.publish_feedback(
                    f'当前位置: '
                    f'{self.rectangle_segment_label(self.current_segment)}，'
                    f'直行到位，准备切换到下一段'
                )

        # ═══════════════════════════════════════════════
        # 新避障模块接管控制权
        # ═══════════════════════════════════════════════
        # 提前取段速度，惯导和避障共用
        linear = float(self.current_segment.get('speed', self.corridor_linear_speed)) \
            if self.current_segment else self.corridor_linear_speed

        if self._spiral_avoider.is_active:
            if self._spiral_avoider.step(self.navigation_yaw(), self.current_position):
                return  # 避障中，跳过后续惯导控制
            else:
                # 避障完成，恢复惯导控制
                now_sec = self.get_clock().now().nanoseconds / 1e9
                self._avoidance_cooldown_until = now_sec + self._avoidance_cooldown_sec
                self.logger.info('AVOID',
                    f'避障完成，恢复惯导控制 '
                    f'冷却至 t={self._avoidance_cooldown_until:.1f}s'
                )
        else:
            # 检测是否需要触发避障（带冷却检查）
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec >= self._avoidance_cooldown_until:
                if self._spiral_avoider.should_trigger(
                        trigger_distance=self._avoidance_trigger_dist):
                    if self._spiral_avoider.start(
                        self.navigation_yaw(),
                        linear_speed=linear,
                        track_direction=self.segment_heading,  # 传递轨道方向
                        robot_pos=self.current_position,       # 传递机器人位置
                    ):
                        self.logger.info('AVOID', '检测到障碍，避障模块接管')
                        return

        # ── 段完成检查 ────────────────────────────────────
        if self.current_position is not None and self.segment_heading is not None and self.current_segment is not None:
            target_distance = max(1e-6, float(self.current_segment.get('distance_m', 0.0)))
            progress = self.projected_distance()
            
            if progress >= target_distance - self.distance_tolerance:
                self.logger.segment(
                    f'{self.current_segment.get("description", "?")} '
                    f'{progress:.3f}/{target_distance:.2f}m 完成')
                self.cmd_pub.publish(self.create_twist())
                self.start_segment(self.plan_index + 1)
                return

        # ── 正常惯导控制 ──────────────────────────────────

        if self.current_position is None or self.segment_heading is None:
            self.cmd_pub.publish(self.create_twist())
            self._maybe_log_telemetry('move_no_pose')
            return

        angular = self._compute_move_lateral_angular()
        self.cmd_pub.publish(self.create_twist(linear, angular))
        self._maybe_log_telemetry('move')

    def run_turn_segment(self):
        turn_tolerance = self.active_turn_heading_tolerance
        linear_speed = float(
            (self.current_segment or {}).get('turn_linear_speed', self.turn_linear_speed)
        )

        nav_yaw = self.navigation_yaw()
        if nav_yaw is None or self.segment_target_yaw is None:
            self.cmd_pub.publish(self.create_twist())
            return

        error = self.angle_error(self.segment_target_yaw, nav_yaw)
        if abs(error) <= turn_tolerance:
            self.publish_feedback(
                f'当前位置: '
                f'{self.rectangle_segment_label(self.current_segment or {})}，'
                '转弯完成，进入下一段'
            )
            self.cmd_pub.publish(self.create_twist())
            self.start_segment(self.plan_index + 1)
            return

        angular = self.clamp(self.turn_kp * error, self.turn_angular_speed)
        if abs(error) > turn_tolerance and abs(angular) < self.turn_min_angular_speed:
            angular = math.copysign(self.turn_min_angular_speed, error)

        self.cmd_pub.publish(self.create_twist(linear_speed, angular))
        self._maybe_log_telemetry(
            f'turn err={math.degrees(error):.1f}deg'
        )

    def finish_mission(self):
        self.logger.info('MISSION', '完成 | ' + self._pose_diagnostic(), file_only=True)
        super().finish_mission()
        self.logger.info('MISSION', '第二阶段测试完成，自动退出', file_only=True)
        if hasattr(self, '_request_stop') and self._request_stop is not None:
            self._request_stop()

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
            not self._spiral_avoider.is_active
            and self.segment_started_at is not None
            and now_sec - self.segment_started_at > self.segment_timeout
        ):
            desc = self.current_segment.get('description', 'unknown')
            self.logger.timeout(f'段超时 {desc}')
            self.publish_feedback(f'段超时: {desc}')
            self._spiral_avoider.reset()
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
        pkg_dir = get_package_share_directory('racing_stage2_param_test')
        yaml_path = field_track.resolve_yaml_path(pkg_dir, self.direction, '')
        return field_track.load_plan(
            yaml_path,
            self.direction,
            ring_linear_speed=self.ring_linear_speed,
            allow_detour=True,
        )

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
                        f'等待输入就绪: '
                        f'{", ".join(missing_inputs)}'
                    )
                    self.reported_waiting_pose = True
                self.logger.startup(f'{wait_line} | {self._full_telemetry()}')
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
                    f'位姿已就绪，'
                    f'{self.start_delay_sec:.2f}s 后开始'
                )
                self.logger.startup(ready_line)
                self.reported_start_delay = True
            return

        if current_time < self.start_after_time:
            return

        self.mission_active = True
        self.reported_start = True
        self.logger.info('MISSION',
            f'任务开始 方向={self.direction_text()} | {self._pose_diagnostic()}',
            file_only=True,
        )
        self.publish_feedback(
            f'开始执行，方向: {self.direction_text()}，'
            f'模式: {self.start_mode_text()}，'
            f'field_track: {self._field_track_yaml}'
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

    node._request_stop = request_stop
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

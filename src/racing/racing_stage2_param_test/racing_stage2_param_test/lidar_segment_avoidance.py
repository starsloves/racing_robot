import math
from dataclasses import dataclass
from typing import Optional
from geometry_msgs.msg import Twist


@dataclass
class AvoidanceScanSnapshot:
    front_distance: float = float('inf')
    front_angle_deg: float = 0.0
    left_clearance: float = float('inf')
    right_clearance: float = float('inf')


@dataclass
class LidarSegmentAvoidanceConfig:
    trigger_distance_m: float = 0.72
    confirm_frames: int = 2
    cooldown_sec: float = 2.0
    avoidance_linear_speed: float = 0.10
    min_linear_speed: float = 0.06
    merge_linear_speed: float = 0.06
    omega_speed_scale: float = 0.85
    max_omega: float = 0.75
    max_omega_rate: float = 2.4
    lookahead_m: float = 0.16
    heading_kp: float = 2.6
    cross_kp: float = 3.8
    bypass_cross_kp: float = 5.2
    bypass_heading_kp: float = 3.2
    merge_heading_kp: float = 3.0
    merge_cross_kp: float = 5.2
    bypass_heading_deg: float = 16.0
    base_clearance_m: float = 0.17
    obstacle_inflate_m: float = 0.10
    max_clearance_m: float = 0.28
    shift_distance_m: float = 0.30
    pass_margin_m: float = 0.14
    merge_distance_m: float = 0.36
    max_projection_m: float = 1.0
    finish_heading_tol_deg: float = 5.0
    finish_cross_tol_m: float = 0.05
    side_blocked_threshold_m: float = 0.30
    side_clear_threshold_m: float = 0.42
    lateral_confirm_frames: int = 2
    planner_samples_per_meter: float = 28.0
    planner_min_samples: int = 24
    front_angle_bias_deg: float = 6.0
    min_front_clearance_m: float = 0.18
    front_escape_distance_m: float = 0.28
    early_shift_margin_m: float = 0.08
    early_shift_ratio: float = 0.72
    merge_front_clear_distance_m: float = 0.85
    merge_min_offset_ratio: float = 0.55
    merge_min_progress_ratio: float = 0.40
    merge_spin_heading_deg: float = 18.0
    merge_creep_heading_deg: float = 10.0
    merge_creep_speed: float = 0.03
    merge_hold_zero_cross_m: float = 0.12
    merge_release_heading_deg: float = 8.0
    timeout_sec: float = 7.0
    verbose: bool = True


@dataclass
class AvoidanceCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass
class PlannedPath:
    detour_sign: float
    obstacle_s: float
    y_clear: float
    shift_end_s: float
    pass_end_s: float
    merge_end_s: float
    points: list


class AvoidanceDetector:
    def __init__(self, cfg: LidarSegmentAvoidanceConfig):
        self.cfg = cfg
        self._snapshot = AvoidanceScanSnapshot()
        self._trigger_streak = 0

    def reset(self):
        self._trigger_streak = 0

    def update_scan(self, snapshot: AvoidanceScanSnapshot):
        self._snapshot = snapshot
        trigger = math.isfinite(snapshot.front_distance) and snapshot.front_distance <= self.cfg.trigger_distance_m
        if trigger:
            self._trigger_streak += 1
        else:
            self._trigger_streak = 0

    @property
    def snapshot(self) -> AvoidanceScanSnapshot:
        return self._snapshot

    def should_trigger(self) -> bool:
        return self._trigger_streak >= max(1, int(self.cfg.confirm_frames))

    def select_detour_sign(self) -> Optional[float]:
        snapshot = self._snapshot
        if snapshot.front_angle_deg > self.cfg.front_angle_bias_deg:
            if snapshot.right_clearance > self.cfg.side_blocked_threshold_m:
                return -1.0
        elif snapshot.front_angle_deg < -self.cfg.front_angle_bias_deg:
            if snapshot.left_clearance > self.cfg.side_blocked_threshold_m:
                return +1.0

        if snapshot.left_clearance > snapshot.right_clearance:
            if snapshot.left_clearance > self.cfg.side_blocked_threshold_m:
                return +1.0
        else:
            if snapshot.right_clearance > self.cfg.side_blocked_threshold_m:
                return -1.0
        return None


class AvoidancePlanner:
    def __init__(self, cfg: LidarSegmentAvoidanceConfig):
        self.cfg = cfg

    def build_path(self, snapshot: AvoidanceScanSnapshot, remaining_distance_m: float, detour_sign: float):
        if not math.isfinite(snapshot.front_distance):
            return None, 'front_distance_invalid'

        front_angle_rad = math.radians(snapshot.front_angle_deg)
        obstacle_s = max(0.08, snapshot.front_distance * math.cos(front_angle_rad))
        obstacle_y = snapshot.front_distance * math.sin(front_angle_rad)
        lateral_bias = max(0.0, abs(obstacle_y))
        y_clear = min(
            self.cfg.max_clearance_m,
            self.cfg.base_clearance_m + 0.35 * lateral_bias + self.cfg.obstacle_inflate_m,
        )

        shift_end_s = min(self.cfg.shift_distance_m, max(0.22, obstacle_s - 0.12))
        pass_end_s = obstacle_s + self.cfg.pass_margin_m
        merge_end_s = pass_end_s + self.cfg.merge_distance_m

        max_projection = min(self.cfg.max_projection_m, max(0.25, remaining_distance_m - 0.03))
        if merge_end_s > max_projection:
            overflow = merge_end_s - max_projection
            merge_end_s -= overflow
            pass_end_s = min(pass_end_s, merge_end_s - 0.12)
            shift_end_s = min(shift_end_s, pass_end_s - 0.10)

        shift_end_s = max(0.20, shift_end_s)
        pass_end_s = max(shift_end_s + 0.10, pass_end_s)
        merge_end_s = max(pass_end_s + 0.14, merge_end_s)

        if merge_end_s > max_projection + 1e-6:
            return None, f'projection_budget_insufficient({merge_end_s:.2f}>{max_projection:.2f})'

        points = self._sample_path(shift_end_s, pass_end_s, merge_end_s, detour_sign * y_clear)
        return PlannedPath(
            detour_sign=detour_sign,
            obstacle_s=obstacle_s,
            y_clear=detour_sign * y_clear,
            shift_end_s=shift_end_s,
            pass_end_s=pass_end_s,
            merge_end_s=merge_end_s,
            points=points,
        ), None

    def _sample_path(self, shift_end_s: float, pass_end_s: float, merge_end_s: float, y_clear: float):
        samples = max(
            self.cfg.planner_min_samples,
            int(math.ceil(merge_end_s * self.cfg.planner_samples_per_meter)),
        )
        points = []
        for idx in range(samples + 1):
            s = merge_end_s * idx / samples
            if s <= shift_end_s:
                y = y_clear * self._quintic_ratio(s / max(shift_end_s, 1e-6))
            elif s <= pass_end_s:
                y = y_clear
            else:
                denom = max(merge_end_s - pass_end_s, 1e-6)
                ratio = (s - pass_end_s) / denom
                y = y_clear * (1.0 - self._quintic_ratio(ratio))
            points.append((s, y))
        return points

    @staticmethod
    def _quintic_ratio(ratio: float) -> float:
        ratio = max(0.0, min(1.0, ratio))
        return 10.0 * ratio ** 3 - 15.0 * ratio ** 4 + 6.0 * ratio ** 5


class AvoidanceTracker:
    def __init__(self, cfg: LidarSegmentAvoidanceConfig):
        self.cfg = cfg
        self._last_omega = 0.0

    def reset(self):
        self._last_omega = 0.0

    def force_shift(self, detour_sign: float, robot_y: float, heading_error: float, nominal_speed: float, target_y: float):
        desired_heading = detour_sign * math.radians(self.cfg.bypass_heading_deg)
        heading_error_to_target = desired_heading - heading_error
        cross_error = target_y - robot_y
        omega = self._limit_omega(
            self.cfg.bypass_heading_kp * heading_error_to_target
            + self.cfg.bypass_cross_kp * cross_error
        )
        linear = max(self.cfg.min_linear_speed, min(nominal_speed * 0.85, nominal_speed))
        return AvoidanceCommand(linear_x=linear, angular_z=omega)

    def follow_path(self, path: PlannedPath, robot_s: float, robot_y: float, heading_error: float, nominal_speed: float):
        target_s = min(path.merge_end_s, robot_s + self.cfg.lookahead_m)
        target_y = self._interpolate_path(path.points, target_s)
        alpha = math.atan2(target_y - robot_y, max(0.02, target_s - robot_s)) - heading_error
        cross_error = target_y - robot_y
        omega = self._limit_omega(self.cfg.heading_kp * alpha + self.cfg.bypass_cross_kp * cross_error)
        return self._build_command(nominal_speed, omega)

    def merge_align(self, robot_y: float, heading_error: float, nominal_speed: float):
        omega = self._limit_omega(
            -self.cfg.merge_heading_kp * heading_error
            -self.cfg.merge_cross_kp * robot_y
        )
        heading_deg = abs(math.degrees(heading_error))
        cross_abs = abs(robot_y)
        if heading_deg >= self.cfg.merge_spin_heading_deg and cross_abs >= self.cfg.merge_hold_zero_cross_m:
            linear = self.cfg.merge_creep_speed
        elif heading_deg >= self.cfg.merge_creep_heading_deg:
            linear = self.cfg.merge_creep_speed
        else:
            linear = self.cfg.merge_linear_speed
        if (
            abs(robot_y) <= self.cfg.finish_cross_tol_m * 2.0
            and abs(math.degrees(heading_error)) <= self.cfg.merge_release_heading_deg
        ):
            linear = nominal_speed
        return AvoidanceCommand(linear_x=linear, angular_z=omega)

    def _build_command(self, nominal_speed: float, omega: float):
        scaled_speed = nominal_speed * (1.0 - self.cfg.omega_speed_scale * min(1.0, abs(omega) / max(self.cfg.max_omega, 1e-6)))
        linear = max(self.cfg.min_linear_speed, scaled_speed)
        return AvoidanceCommand(linear_x=linear, angular_z=omega)

    def _limit_omega(self, omega: float):
        omega = max(-self.cfg.max_omega, min(self.cfg.max_omega, omega))
        if self.cfg.max_omega_rate <= 0.0:
            self._last_omega = omega
            return omega
        max_delta = self.cfg.max_omega_rate / 30.0
        delta = omega - self._last_omega
        if abs(delta) > max_delta:
            omega = self._last_omega + math.copysign(max_delta, delta)
        self._last_omega = omega
        return omega

    @staticmethod
    def _interpolate_path(points, target_s: float):
        if not points:
            return 0.0
        if target_s <= points[0][0]:
            return points[0][1]
        if target_s >= points[-1][0]:
            return points[-1][1]
        for index in range(len(points) - 1):
            s0, y0 = points[index]
            s1, y1 = points[index + 1]
            if s0 <= target_s <= s1:
                ratio = (target_s - s0) / max(s1 - s0, 1e-6)
                return y0 + ratio * (y1 - y0)
        return points[-1][1]


class LidarSegmentAvoidanceManager:
    def __init__(self, cmd_pub, logger, clock, cfg: LidarSegmentAvoidanceConfig, cmd_observer=None):
        self.cmd_pub = cmd_pub
        self._log = logger
        self._clock = clock
        self.cfg = cfg
        self._cmd_observer = cmd_observer
        self._detector = AvoidanceDetector(cfg)
        self._planner = AvoidancePlanner(cfg)
        self._tracker = AvoidanceTracker(cfg)
        self.reset(full=True)

    def reset(self, full=False):
        self._state = 'idle'
        self._detector.reset()
        self._tracker.reset()
        self._plan = None
        self._scan = AvoidanceScanSnapshot()
        self._start_time = None
        self._start_yaw = None
        self._start_position = None
        self._nominal_speed = self.cfg.avoidance_linear_speed
        if full or not hasattr(self, '_cooldown_until'):
            self._cooldown_until = 0.0
        self._side_blocked_frames = 0
        self._side_clear_frames = 0
        self._last_detour_sign = -1.0
        self._obstacle_side_sign = 1.0

    @property
    def is_active(self) -> bool:
        return self._state != 'idle'

    def on_scan(self, front_distance, front_angle_deg, left_clearance, right_clearance):
        self._scan = AvoidanceScanSnapshot(
            front_distance=float(front_distance),
            front_angle_deg=float(front_angle_deg),
            left_clearance=float(left_clearance),
            right_clearance=float(right_clearance),
        )
        self._detector.update_scan(self._scan)
        self._update_lateral_status()

    def should_trigger(self) -> bool:
        return self._now_sec() >= self._cooldown_until and self._detector.should_trigger()

    def start(self, current_yaw, robot_pos, remaining_distance_m, nominal_speed):
        if self.is_active:
            return False
        detour_sign = self._detector.select_detour_sign()
        if detour_sign is None:
            self._log.warn('AVOID', '触发避障但两侧空间不足，放弃接管')
            return False
        plan, reason = self._planner.build_path(self._scan, remaining_distance_m, detour_sign)
        if plan is None:
            self._log.warn('AVOID', f'局部规划失败: {reason}')
            return False

        self._state = 'bypass'
        self._plan = plan
        self._start_time = self._now_sec()
        self._start_yaw = current_yaw
        self._start_position = robot_pos
        self._nominal_speed = min(float(nominal_speed), self.cfg.avoidance_linear_speed)
        self._side_blocked_frames = 0
        self._side_clear_frames = 0
        self._last_detour_sign = detour_sign
        self._obstacle_side_sign = -detour_sign
        self._log.info(
            'AVOID',
            '启动 detour=%s obs_s=%.2fm clear=%.0fcm shift=%.2fm pass=%.2fm merge=%.2fm front=%.2fm@%.1fdeg'
            % (
                'left' if detour_sign > 0.0 else 'right',
                plan.obstacle_s,
                abs(plan.y_clear) * 100.0,
                plan.shift_end_s,
                plan.pass_end_s,
                plan.merge_end_s,
                self._scan.front_distance,
                self._scan.front_angle_deg,
            ),
        )
        return True

    def step(self, current_yaw, robot_pos):
        if not self.is_active:
            return False
        now = self._now_sec()
        if self._start_time is None or self._start_position is None or self._start_yaw is None:
            self._abort('start_pose_missing')
            return False
        if now - self._start_time > self.cfg.timeout_sec:
            self._abort('timeout')
            return False
        if math.isfinite(self._scan.front_distance) and self._scan.front_distance < self.cfg.min_front_clearance_m:
            self._publish_cmd(0.0, 0.0)
            self._log.warn('AVOID', f'前向急停 front={self._scan.front_distance:.2f}m')
            return True

        robot_s, robot_y = self._track_position(robot_pos)
        heading_error = self._normalize_angle(current_yaw - self._start_yaw)

        obstacle_passed = robot_s >= (self._plan.obstacle_s + self.cfg.pass_margin_m)
        lateral_cleared = self._side_clear_frames >= max(1, self.cfg.lateral_confirm_frames)
        enough_offset = abs(robot_y) >= abs(self._plan.y_clear) * self.cfg.merge_min_offset_ratio
        enough_progress = robot_s >= self._plan.shift_end_s * self.cfg.merge_min_progress_ratio
        front_open = (
            math.isfinite(self._scan.front_distance)
            and self._scan.front_distance >= self.cfg.merge_front_clear_distance_m
        )
        heading_ready = abs(math.degrees(heading_error)) >= self.cfg.bypass_heading_deg * 0.85
        merge_ready = lateral_cleared or (front_open and enough_progress and enough_offset and heading_ready)
        if self._state == 'bypass' and (robot_s >= self._plan.pass_end_s or merge_ready):
            self._state = 'merge'
            self._log.info(
                'AVOID',
                f'进入回归 robot_s={robot_s:.2f}m y={robot_y*100:.1f}cm '
                f'passed={int(obstacle_passed)} lateral_clear={int(lateral_cleared)} '
                f'front_open={int(front_open)} offset_ok={int(enough_offset)} '
                f'progress_ok={int(enough_progress)} heading_ok={int(heading_ready)}',
            )

        if self._state == 'bypass':
            shift_target = abs(self._plan.y_clear) * self.cfg.early_shift_ratio
            if (
                (robot_s <= self._plan.shift_end_s or abs(robot_y) < shift_target)
                and not front_open
            ):
                command = self._tracker.force_shift(
                    self._plan.detour_sign,
                    robot_y,
                    heading_error,
                    self._nominal_speed,
                    self._plan.y_clear,
                )
            else:
                command = self._tracker.follow_path(
                    self._plan,
                    robot_s,
                    robot_y,
                    heading_error,
                    self._nominal_speed,
                )
            if (
                math.isfinite(self._scan.front_distance)
                and self._scan.front_distance <= self.cfg.front_escape_distance_m
                and abs(robot_y) < (abs(self._plan.y_clear) - self.cfg.early_shift_margin_m)
            ):
                command.angular_z = self._plan.detour_sign * self.cfg.max_omega
                command.linear_x = self.cfg.min_linear_speed
        else:
            command = self._tracker.merge_align(robot_y, heading_error, self._nominal_speed)

        self._publish_cmd(command.linear_x, command.angular_z)

        heading_ok = abs(math.degrees(heading_error)) <= self.cfg.finish_heading_tol_deg
        cross_ok = abs(robot_y) <= self.cfg.finish_cross_tol_m
        if self._state == 'merge' and robot_s >= self._plan.merge_end_s and heading_ok and cross_ok:
            self._log.info(
                'AVOID',
                f'完成 robot_s={robot_s:.2f}m y={robot_y*100:.1f}cm head={math.degrees(heading_error):.1f}deg',
            )
            self._cooldown_until = now + self.cfg.cooldown_sec
            self._state = 'idle'
            self._tracker.reset()
            self._publish_cmd(0.0, 0.0)
            return False
        return True

    def force_reset(self, reason='reset'):
        if self.is_active:
            self._log.warn('AVOID', f'强制重置: {reason}')
        self._publish_cmd(0.0, 0.0)
        self.reset()

    def _abort(self, reason):
        self._log.warn('AVOID', f'中止: {reason}')
        self._cooldown_until = self._now_sec() + self.cfg.cooldown_sec
        self._publish_cmd(0.0, 0.0)
        self.reset()

    def _track_position(self, robot_pos):
        dx = robot_pos[0] - self._start_position[0]
        dy = robot_pos[1] - self._start_position[1]
        robot_s = dx * math.cos(self._start_yaw) + dy * math.sin(self._start_yaw)
        robot_y = -dx * math.sin(self._start_yaw) + dy * math.cos(self._start_yaw)
        return max(0.0, robot_s), robot_y

    def _update_lateral_status(self):
        # 通过判据必须盯住“障碍物所在侧”，而不是绕行侧。
        if self._obstacle_side_sign > 0.0:
            lateral_clearance = self._scan.left_clearance
        else:
            lateral_clearance = self._scan.right_clearance
        if math.isfinite(lateral_clearance) and lateral_clearance <= self.cfg.side_blocked_threshold_m:
            self._side_blocked_frames += 1
            self._side_clear_frames = 0
        elif math.isfinite(lateral_clearance) and lateral_clearance >= self.cfg.side_clear_threshold_m and self._side_blocked_frames > 0:
            self._side_clear_frames += 1

    def _publish_cmd(self, linear_x=0.0, angular_z=0.0):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        if self._cmd_observer is not None:
            self._cmd_observer(float(linear_x), float(angular_z))
        self.cmd_pub.publish(cmd)

    def _now_sec(self):
        return self._clock.now().nanoseconds / 1e9

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

# DEPRECATED: 此文件未被任何模块 import，实际避障逻辑已在 direct_inertial_tester._try_avoid_step() 中内联实现。保留仅作参考。
"""S1：原地慢转够角 → 停稳 → 直行 L → … → 回 ψ₀ 停稳。

不用边走边转、不在直行时纠航、不用 IMU 闭环 bang-bang（场测 yaw 乱跳会来回拧）。
转向按「还需转多少角」开环积分，转够再停。
"""

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional, Tuple

from racing_stage2_param_test.s1_geometry import S1Plan


class S1Phase(Enum):
    IDLE = auto()
    TURN_LEG1 = auto()
    SETTLE_LEG1 = auto()
    DRIVE_LEG1 = auto()
    TURN_LEG2 = auto()
    SETTLE_LEG2 = auto()
    DRIVE_LEG2 = auto()
    TURN_RECOVER = auto()
    SETTLE_RECOVER = auto()


@dataclass(frozen=True)
class S1Config:
    leg_offset_deg: float = 30.0
    leg1_distance_m: float = 0.30
    leg2_distance_m: float = 0.50
    leg_linear_speed: float = 0.10
    distance_tol_m: float = 0.04
    turn_angular_speed: float = 0.40
    turn_settle_sec: float = 0.35
    turn_angle_slack_rad: float = math.radians(2.0)
    telemetry_interval_sec: float = 0.20

    @property
    def leg_offset_rad(self) -> float:
        return math.radians(self.leg_offset_deg)

    def leg_threshold_m(self, leg_distance_m: float) -> float:
        return max(0.0, leg_distance_m - self.distance_tol_m)


@dataclass(frozen=True)
class DriveCommand:
    linear_x: float
    angular_z: float


class S1Executor:
    def __init__(
        self,
        config: S1Config,
        log: Optional[Callable[[str], None]] = None,
        telemetry: Optional[Callable[[str], None]] = None,
    ):
        self.config = config
        self._log = log or (lambda _message: None)
        self._telemetry = telemetry or (lambda _message: None)
        self.phase = S1Phase.IDLE
        self.plan: Optional[S1Plan] = None
        self._leg_start_xy: Optional[Tuple[float, float]] = None
        self._settle_until_sec: float = 0.0
        self._turn_required_rad: float = 0.0
        self._turn_sign: float = 1.0
        self._turn_accum_rad: float = 0.0
        self._prev_yaw_rad: Optional[float] = None
        self._last_telemetry_sec = 0.0

    @property
    def active(self) -> bool:
        return self.phase != S1Phase.IDLE

    def reset(self) -> None:
        self.phase = S1Phase.IDLE
        self.plan = None
        self._leg_start_xy = None
        self._settle_until_sec = 0.0
        self._turn_required_rad = 0.0
        self._turn_sign = 1.0
        self._turn_accum_rad = 0.0
        self._prev_yaw_rad = None
        self._last_telemetry_sec = 0.0

    @staticmethod
    def should_trigger(
        segment_allows_detour: bool,
        front_obstacle_distance: float,
        trigger_distance_m: float,
        segment_heading_rad: Optional[float],
        current_yaw_rad: Optional[float],
        heading_gate_rad: float,
        angle_error_fn,
    ) -> bool:
        if not segment_allows_detour:
            return False
        if not math.isfinite(front_obstacle_distance):
            return False
        if front_obstacle_distance > trigger_distance_m:
            return False
        if segment_heading_rad is not None and current_yaw_rad is not None:
            if abs(angle_error_fn(segment_heading_rad, current_yaw_rad)) > heading_gate_rad:
                return False
        return True

    def start(self, plan: S1Plan, danger_angle_deg: float, start_xy: Tuple[float, float]) -> None:
        del start_xy
        self.plan = plan
        self.phase = S1Phase.TURN_LEG1
        self._leg_start_xy = None
        # start() 在首帧 step 前调用，yaw 由首帧 step 传入时再 begin_turn
        self._turn_required_rad = 0.0
        self._log(
            f'S1 开始 ψ₀={math.degrees(plan.psi0):.1f}deg '
            f'脚1={math.degrees(plan.psi1):.1f}deg×{plan.leg1_distance_m:.2f}m '
            f'脚2={math.degrees(plan.psi2):.1f}deg×{plan.leg2_distance_m:.2f}m '
            f'(danger={danger_angle_deg:.1f}deg)'
        )

    def finish(self, cross_track_m: float, yaw_deg: float) -> None:
        psi0_deg = (
            math.degrees(self.plan.psi0) if self.plan is not None else float('nan')
        )
        self._log(
            f'S1 结束 cross_track={cross_track_m * 100:.1f}cm yaw={yaw_deg:.1f}deg '
            f'ψ₀={psi0_deg:.1f}deg'
        )
        self.reset()

    def step(
        self,
        x: float,
        y: float,
        yaw: float,
        now_sec: float,
        angle_error_fn,
        clamp_fn,
    ) -> Optional[DriveCommand]:
        del clamp_fn
        if not self.active or self.plan is None:
            return None

        plan = self.plan
        cmd = DriveCommand(0.0, 0.0)

        if self.phase == S1Phase.TURN_LEG1:
            if self._turn_required_rad <= 0.0:
                self._begin_open_loop_turn(plan.psi1, yaw, angle_error_fn)
            cmd = self._step_open_loop_turn(plan.psi1, yaw, angle_error_fn, now_sec)
            if self._open_loop_turn_done():
                self._log(
                    f'脚1 转到位 {math.degrees(yaw):.1f}deg '
                    f'(目标{math.degrees(plan.psi1):.1f}deg) → 停稳'
                )
                self._enter_settle(S1Phase.SETTLE_LEG1, now_sec)

        elif self.phase == S1Phase.SETTLE_LEG1:
            cmd = self._step_settle(now_sec)
            if self._settle_done(now_sec):
                self._log(f'脚1 停稳 → 直行 {plan.leg1_distance_m:.2f}m ω=0')
                self._mark_leg_start((x, y))
                self.phase = S1Phase.DRIVE_LEG1

        elif self.phase == S1Phase.DRIVE_LEG1:
            cmd = DriveCommand(self.config.leg_linear_speed, 0.0)
            if self._leg_travel_done(x, y):
                self._log(f'脚1 走完 {plan.leg1_distance_m:.2f}m')
                self.phase = S1Phase.TURN_LEG2
                self._begin_open_loop_turn(plan.psi2, yaw, angle_error_fn)

        elif self.phase == S1Phase.TURN_LEG2:
            if self._turn_required_rad <= 0.0:
                self._begin_open_loop_turn(plan.psi2, yaw, angle_error_fn)
            cmd = self._step_open_loop_turn(plan.psi2, yaw, angle_error_fn, now_sec)
            if self._open_loop_turn_done():
                self._log(
                    f'脚2 转到位 {math.degrees(yaw):.1f}deg '
                    f'(目标{math.degrees(plan.psi2):.1f}deg) → 停稳'
                )
                self._enter_settle(S1Phase.SETTLE_LEG2, now_sec)

        elif self.phase == S1Phase.SETTLE_LEG2:
            cmd = self._step_settle(now_sec)
            if self._settle_done(now_sec):
                self._log(f'脚2 停稳 → 直行 {plan.leg2_distance_m:.2f}m ω=0')
                self._mark_leg_start((x, y))
                self.phase = S1Phase.DRIVE_LEG2

        elif self.phase == S1Phase.DRIVE_LEG2:
            cmd = DriveCommand(self.config.leg_linear_speed, 0.0)
            if self._leg_travel_done(x, y):
                self._log(f'脚2 走完 {plan.leg2_distance_m:.2f}m → 回身 ψ₀')
                self.phase = S1Phase.TURN_RECOVER
                self._begin_open_loop_turn(plan.psi0, yaw, angle_error_fn)

        elif self.phase == S1Phase.TURN_RECOVER:
            if self._turn_required_rad <= 0.0:
                self._begin_open_loop_turn(plan.psi0, yaw, angle_error_fn)
            cmd = self._step_open_loop_turn(plan.psi0, yaw, angle_error_fn, now_sec)
            if self._open_loop_turn_done():
                self._log(
                    f'回身转到位 {math.degrees(yaw):.1f}deg '
                    f'(ψ₀={math.degrees(plan.psi0):.1f}deg) → 停稳'
                )
                self._enter_settle(S1Phase.SETTLE_RECOVER, now_sec)

        elif self.phase == S1Phase.SETTLE_RECOVER:
            cmd = self._step_settle(now_sec)
            if self._settle_done(now_sec):
                self._log(f'回正完成 ψ₀={math.degrees(plan.psi0):.1f}deg')
                self.phase = S1Phase.IDLE
                cmd = DriveCommand(0.0, 0.0)

        self._maybe_telemetry(now_sec, x, y, yaw, cmd, angle_error_fn)
        return cmd

    def _enter_settle(self, settle_phase: S1Phase, now_sec: float) -> None:
        self.phase = settle_phase
        self._settle_until_sec = now_sec + self.config.turn_settle_sec
        self._turn_required_rad = 0.0
        self._turn_accum_rad = 0.0
        self._prev_yaw_rad = None

    def _step_settle(self, now_sec: float) -> DriveCommand:
        del now_sec
        return DriveCommand(0.0, 0.0)

    def _settle_done(self, now_sec: float) -> bool:
        return now_sec >= self._settle_until_sec

    def _begin_open_loop_turn(
        self,
        target_yaw: float,
        yaw: float,
        angle_error_fn,
    ) -> None:
        err = angle_error_fn(target_yaw, yaw)
        self._turn_required_rad = abs(err)
        self._turn_sign = math.copysign(1.0, err) if abs(err) > 1e-6 else 1.0
        self._turn_accum_rad = 0.0
        self._prev_yaw_rad = yaw

    def _step_open_loop_turn(
        self,
        target_yaw: float,
        yaw: float,
        angle_error_fn,
        now_sec: float,
    ) -> DriveCommand:
        del target_yaw, now_sec
        self._accumulate_turn(yaw, angle_error_fn)
        omega = self._turn_sign * self.config.turn_angular_speed
        return DriveCommand(0.0, omega)

    def _accumulate_turn(self, yaw: float, angle_error_fn) -> None:
        if self._prev_yaw_rad is None:
            self._prev_yaw_rad = yaw
            return
        step = angle_error_fn(self._prev_yaw_rad, yaw)
        self._prev_yaw_rad = yaw
        # 单帧 >15° 视为 IMU 跳变，不计入开环转角
        if abs(step) > math.radians(15.0):
            return
        if self._turn_sign * step > 0.0:
            self._turn_accum_rad += abs(step)

    def _open_loop_turn_done(self) -> bool:
        need = max(0.0, self._turn_required_rad - self.config.turn_angle_slack_rad)
        return self._turn_accum_rad >= need

    def _mark_leg_start(self, xy: Tuple[float, float]) -> None:
        self._leg_start_xy = (float(xy[0]), float(xy[1]))

    def _leg_travel_done(self, x: float, y: float) -> bool:
        if self._leg_start_xy is None or self.plan is None:
            return False
        dx = x - self._leg_start_xy[0]
        dy = y - self._leg_start_xy[1]
        traveled = math.hypot(dx, dy)
        leg_m = (
            self.plan.leg1_distance_m
            if self.phase == S1Phase.DRIVE_LEG1
            else self.plan.leg2_distance_m
        )
        return traveled >= self.config.leg_threshold_m(leg_m)

    def _maybe_telemetry(
        self,
        now_sec: float,
        x: float,
        y: float,
        yaw: float,
        cmd: DriveCommand,
        angle_error_fn,
    ) -> None:
        if now_sec - self._last_telemetry_sec < self.config.telemetry_interval_sec:
            return
        self._last_telemetry_sec = now_sec
        plan = self.plan
        if plan is None:
            return

        target_yaw = plan.psi0
        leg_target_m = 0.0
        if self.phase in (S1Phase.TURN_LEG1, S1Phase.SETTLE_LEG1, S1Phase.DRIVE_LEG1):
            target_yaw = plan.psi1
            leg_target_m = plan.leg1_distance_m
        elif self.phase in (S1Phase.TURN_LEG2, S1Phase.SETTLE_LEG2, S1Phase.DRIVE_LEG2):
            target_yaw = plan.psi2
            leg_target_m = plan.leg2_distance_m

        traveled = 0.0
        if self.phase in (S1Phase.DRIVE_LEG1, S1Phase.DRIVE_LEG2) and self._leg_start_xy:
            traveled = math.hypot(
                x - self._leg_start_xy[0],
                y - self._leg_start_xy[1],
            )

        err_deg = math.degrees(angle_error_fn(target_yaw, yaw))
        turn_pct = 100.0
        if self._turn_required_rad > 1e-6:
            turn_pct = min(
                100.0, 100.0 * self._turn_accum_rad / self._turn_required_rad
            )

        self._telemetry(
            f'phase={self.phase.name} yaw={math.degrees(yaw):.1f} '
            f'tgt={math.degrees(target_yaw):.1f} err={err_deg:+.1f}deg '
            f'turn={turn_pct:.0f}% v={cmd.linear_x:.2f} w={cmd.angular_z:+.2f} '
            f'travel={traveled:.2f}/{leg_target_m:.2f}m'
        )

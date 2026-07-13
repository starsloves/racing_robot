"""avoid_controller.py — 边转边避独立模块。

闭环航向控制 + 侧边距离触发 + 绕行方向选择。

用法：
    config = AvoidConfig(...)
    avoider = AvoidController(cmd_pub, logger, clock, config)
    avoider.on_scan(front, angle, left, right)
    if avoider.step(NavState(...)):
        return  # 避障已发布 cmd_vel，导航跳过本帧
"""

import math
from dataclasses import dataclass
from geometry_msgs.msg import Twist

from racing_stage2_param_test.avoid_geometry import (
    build_avoid_plan,
    normalize_angle,
    obstacle_is_left,
)


@dataclass
class NavState:
    position: tuple
    yaw: float
    segment_heading: float
    segment_start_pose: tuple
    current_segment: dict
    projected_distance: float


class AvoidConfig:
    def __init__(
        self,
        detour_obstacle_distance=0.48,
        detour_heading_gate_deg=12.0,
        detour_confirm_required=3,
        detour_cooldown_sec=3.0,
        avoid_leg1_distance_m=0.05,
        avoid_leg2_distance_m=0.10,
        avoid_leg_linear_speed=0.10,
        avoid_turn_linear_speed=0.08,
        avoid_leg_distance_tol_m=0.04,
        avoid_turn_angular_speed=0.40,
        avoid_turn_away_deg=30.0,
        avoid_turn_back_deg=40.0,
        avoid_recover_deg=40.0,
        avoider_heading_tolerance_deg=1.5,
        distance_tolerance=0.05,
        heading_kp=1.6,
        side_detour_threshold_m=0.18,
        side_detour_enabled=True,
        turn_min_angular_speed=0.10,
    ):
        self.detour_obstacle_distance = detour_obstacle_distance
        self.detour_heading_gate_rad = math.radians(detour_heading_gate_deg)
        self.detour_confirm_required = detour_confirm_required
        self.detour_cooldown_sec = detour_cooldown_sec
        self.avoid_leg1_distance_m = avoid_leg1_distance_m
        self.avoid_leg2_distance_m = avoid_leg2_distance_m
        self.avoid_leg_linear_speed = avoid_leg_linear_speed
        self.avoid_turn_linear_speed = avoid_turn_linear_speed
        self.avoid_leg_distance_tol_m = avoid_leg_distance_tol_m
        self.avoid_turn_angular_speed = avoid_turn_angular_speed
        self.avoid_turn_away_deg = float(avoid_turn_away_deg)
        self.avoid_turn_back_deg = float(avoid_turn_back_deg)
        self.avoid_recover_deg = float(avoid_recover_deg)
        self.avoid_turn_away_rad = math.radians(self.avoid_turn_away_deg)
        self.avoid_turn_back_rad = math.radians(self.avoid_turn_back_deg)
        self.avoid_recover_rad = math.radians(self.avoid_recover_deg)
        self.avoider_heading_tolerance_deg = float(avoider_heading_tolerance_deg)
        self.heading_tolerance = math.radians(self.avoider_heading_tolerance_deg)
        self.distance_tolerance = distance_tolerance
        self.heading_kp = heading_kp
        self.side_detour_threshold_m = side_detour_threshold_m
        self.side_detour_enabled = side_detour_enabled
        self.turn_min_angular_speed = turn_min_angular_speed


class AvoidController:
    def __init__(self, cmd_pub, logger, clock, config=None, vision_callback=None):
        self.cmd_pub = cmd_pub
        self._log = logger
        self._clock = clock
        self.cfg = config or AvoidConfig()
        self._vision_callback = vision_callback  # 新增：视觉修正回调（用于 leg2）

        self._state = 'idle'
        self._plan = None
        self._leg_start_xy = None
        self._turn_target_yaw = None
        self._cooldown_until = 0.0
        self._last_state_logged = 'idle'

        self.front_distance = float('inf')
        self.front_angle_deg = 0.0
        self.left_clearance = float('inf')
        self.left_angle_deg = 0.0
        self.right_clearance = float('inf')
        self.right_angle_deg = 0.0

    # ── 外部接口 ───────────────────────────────────────────

    def on_scan(self, front_distance, front_angle_deg, left_clearance, left_angle_deg, right_clearance, right_angle_deg):
        self.front_distance = front_distance
        self.front_angle_deg = front_angle_deg
        self.left_clearance = left_clearance
        self.left_angle_deg = left_angle_deg
        self.right_clearance = right_clearance
        self.right_angle_deg = right_angle_deg

    def reset(self):
        prev = self._state
        self._state = 'idle'
        self._plan = None
        self._leg_start_xy = None
        self._turn_target_yaw = None
        self._cooldown_until = 0.0
        if prev != 'idle' and prev != self._last_state_logged:
            self._log.info(f'[AVOID] 状态 {prev} → idle')
            self._last_state_logged = 'idle'

    @property
    def is_active(self) -> bool:
        return self._state != 'idle'

    @property
    def state_str(self) -> str:
        return self._state

    def has_obstacle(self) -> bool:
        return (math.isfinite(self.front_distance)
                and self._effective_front_m() < self.cfg.detour_obstacle_distance)

    # ── 角度换算 ───────────────────────────────────────────

    def _effective_front_m(self) -> float:
        """有效前向距离：雷达测距 × cos(水平偏角)，统一到车头方向"""
        if not math.isfinite(self.front_distance):
            return float('inf')
        return self.front_distance * abs(math.cos(math.radians(self.front_angle_deg)))

    def _effective_left_m(self) -> float:
        """有效左侧横向距离：雷达测距 × sin(射线角度)，统一到垂直车身方向"""
        if not math.isfinite(self.left_clearance):
            return float('inf')
        return self.left_clearance * abs(math.sin(math.radians(self.left_angle_deg)))

    def _effective_right_m(self) -> float:
        """有效右侧横向距离"""
        if not math.isfinite(self.right_clearance):
            return float('inf')
        return self.right_clearance * abs(math.sin(math.radians(self.right_angle_deg)))

    # ── 工具 ───────────────────────────────────────────────

    @staticmethod
    def _angle_error(target, actual):
        return normalize_angle(target - actual)

    @staticmethod
    def _clamp(value, bound):
        return max(-bound, min(bound, value))

    def _now_sec(self):
        return self._clock.now().nanoseconds / 1e9

    def _log_detour(self, msg):
        self._log.info(f'[DETOUR] {msg}')

    def _format_yaw_deg(self, yaw):
        if yaw is None or not math.isfinite(yaw):
            return 'nan'
        return f'{math.degrees(normalize_angle(yaw)):.1f}'

    @staticmethod
    def _zero_twist():
        return Twist()

    @staticmethod
    def _make_twist(vx=0.0, wz=0.0):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        return t

    # ── 闭环转弯 ───────────────────────────────────────────

    def _turn_toward(self, target_yaw, yaw, linear_speed=None) -> bool:
        """闭环转到目标航向。返回 True 表示已到位。"""
        if linear_speed is None:
            linear_speed = self.cfg.avoid_turn_linear_speed
        error = self._angle_error(target_yaw, yaw)
        if abs(error) <= self.cfg.heading_tolerance:
            return True
        omega = self._clamp(self.cfg.heading_kp * error, self.cfg.avoid_turn_angular_speed)
        if abs(omega) < self.cfg.turn_min_angular_speed:
            omega = math.copysign(self.cfg.turn_min_angular_speed, error)
        self.cmd_pub.publish(self._make_twist(linear_speed, omega))
        return False

    # ── 直行距离 ───────────────────────────────────────────

    def _mark_leg_start(self, x, y):
        self._leg_start_xy = (x, y)

    def _leg_traveled_m(self, x, y):
        if self._leg_start_xy is None:
            return 0.0
        dx = x - self._leg_start_xy[0]
        dy = y - self._leg_start_xy[1]
        return math.hypot(dx, dy)

    def _leg_done(self, x, y, target_m):
        return self._leg_traveled_m(x, y) >= target_m - self.cfg.avoid_leg_distance_tol_m

    # ── 预估投影 ───────────────────────────────────────────

    def _estimate_projection_m(self):
        offset = self.cfg.avoid_turn_away_rad
        vt = self.cfg.avoid_turn_linear_speed
        w = self.cfg.avoid_turn_angular_speed
        leg1 = self.cfg.avoid_leg1_distance_m
        leg2 = self.cfg.avoid_leg2_distance_m

        if w < 1e-6:
            return leg1 + leg2 + 0.5

        ta_proj = (vt / w) * math.sin(offset)
        leg1_proj = leg1 * math.cos(offset)
        tb_proj = (vt / w) * (math.sin(offset) - math.sin(-offset))
        leg2_proj = leg2 * math.cos(offset)
        tr_proj = (vt / w) * (0.0 - math.sin(-offset))
        fine_proj = 0.02
        return ta_proj + leg1_proj + tb_proj + leg2_proj + tr_proj + fine_proj

    # ── 绕行方向选择 ──────────────────────────────────────

    def select_detour_side(self):
        """根据侧边空间选择绕行方向。返回 'left' 或 'right'，两侧都不够则 None。"""
        left_eff = self._effective_left_m()
        right_eff = self._effective_right_m()
        left_ok = math.isfinite(left_eff) and left_eff >= self.cfg.side_detour_threshold_m
        right_ok = math.isfinite(right_eff) and right_eff >= self.cfg.side_detour_threshold_m

        if left_ok and right_ok:
            return 'left' if left_eff >= right_eff else 'right'
        if left_ok:
            return 'left'
        if right_ok:
            return 'right'
        return None

    # ── 触发判断 ───────────────────────────────────────────

    def _should_trigger(self, nav: NavState) -> bool:
        now = self._now_sec()
        if now < self._cooldown_until:
            return False
        seg = nav.current_segment
        if seg is None or seg.get('type') != 'move':
            return False
        if not bool(seg.get('allow_detour', True)):
            return False
        if nav.segment_heading is not None and nav.yaw is not None:
            if abs(self._angle_error(nav.segment_heading, nav.yaw)) > self.cfg.detour_heading_gate_rad:
                return False

        # 触发条件：前方障碍 OR 侧边空间不足（用角度换算后的有效距离）
        front_blocked = math.isfinite(self.front_distance) and self._effective_front_m() < self.cfg.detour_obstacle_distance
        side_cramped = False
        if self.cfg.side_detour_enabled:
            side_cramped = (
                (math.isfinite(self.left_clearance)
                 and self._effective_left_m() < self.cfg.side_detour_threshold_m)
                or (math.isfinite(self.right_clearance)
                    and self._effective_right_m() < self.cfg.side_detour_threshold_m)
            )

        if not front_blocked and not side_cramped:
            return False
        return True

    # ── 启动 ───────────────────────────────────────────────

    def _start(self, nav: NavState):
        if nav.segment_heading is None or nav.segment_start_pose is None:
            self._log_detour('避障未启动：缺少段航向或段起点')
            self._state = 'idle'
            return

        # 确定障碍在哪一侧
        front_blocked = math.isfinite(self.front_distance) and self._effective_front_m() < self.cfg.detour_obstacle_distance
        side_cramped = self.cfg.side_detour_enabled and (
            (math.isfinite(self.left_clearance)
             and self._effective_left_m() < self.cfg.side_detour_threshold_m)
            or (math.isfinite(self.right_clearance)
                and self._effective_right_m() < self.cfg.side_detour_threshold_m)
        )

        if front_blocked and not side_cramped:
            # 仅前方障碍触发：用 front_angle 判断障碍方位
            obstacle_left = obstacle_is_left(self.front_angle_deg)
        elif side_cramped:
            # 侧边触发：选空间大的一侧绕行
            side = self.select_detour_side()
            if side is None:
                # 两侧都不够，回退到 front_angle
                obstacle_left = obstacle_is_left(self.front_angle_deg)
            else:
                # 往右绕 → 障碍在左
                obstacle_left = (side == 'right')
        else:
            obstacle_left = obstacle_is_left(self.front_angle_deg)

        plan = build_avoid_plan(
            psi0_rad=nav.segment_heading,
            leg1_distance_m=self.cfg.avoid_leg1_distance_m,
            leg2_distance_m=self.cfg.avoid_leg2_distance_m,
            offset_away_rad=self.cfg.avoid_turn_away_rad,
            offset_back_rad=self.cfg.avoid_turn_back_rad,
            offset_recover_rad=self.cfg.avoid_recover_rad,
            obstacle_left=obstacle_left,
        )
        self._plan = plan
        self._state = 'turn_away'
        self._turn_target_yaw = plan.psi1
        dir_text = '左' if obstacle_left else '右'
        self._log_detour(
            f'避障启动 obstacle={dir_text} '
            f'ψ₀={math.degrees(plan.psi0):.1f}° '
            f'→ ψ₁={math.degrees(plan.psi1):.1f}° '
            f'→ ψ₂={math.degrees(plan.psi2):.1f}° '
            f'→ ψ₃={math.degrees(plan.psi3):.1f}° '
            f'L₁={plan.leg1_distance_m:.2f}m L₂={plan.leg2_distance_m:.2f}m '
            f'eff_front={self._effective_front_m():.2f}m '
            f'eff_left={self._effective_left_m():.2f}m '
            f'eff_right={self._effective_right_m():.2f}m'
        )

    @staticmethod
    def _fmt_clr(v):
        if not math.isfinite(v):
            return 'inf'
        return f'{v:.2f}m'

    # ── 主步进 ─────────────────────────────────────────────

    def step(self, nav: NavState) -> bool:
        if nav.position is None or nav.yaw is None:
            if self.is_active:
                self.cmd_pub.publish(self._zero_twist())
            return self.is_active

        x, y = nav.position
        yaw = nav.yaw

        if self._state == 'idle':
            if not self._should_trigger(nav):
                return False
            if nav.segment_heading is None or nav.segment_start_pose is None:
                self._log_detour('避障未启动：缺少段航向或段起点')
                return False
            self._start(nav)
            if self._state != self._last_state_logged:
                self._last_state_logged = self._state

        plan = self._plan
        if plan is None:
            self.reset()
            return False

        if self._state not in ('idle', 'fine_align'):
            pass

        if self._state == 'turn_away':
            if self._turn_toward(plan.psi1, yaw):
                self._log_detour(
                    f'TURN_AWAY 完成 yaw={self._format_yaw_deg(yaw)}° '
                    f'(目标 ψ₁={self._format_yaw_deg(plan.psi1)}°) → LEG1 {plan.leg1_distance_m:.2f}m'
                )
                self._state = 'leg1'
                self._mark_leg_start(x, y)
            return True

        if self._state == 'leg1':
            self.cmd_pub.publish(self._make_twist(self.cfg.avoid_leg_linear_speed, 0.0))
            if self._leg_done(x, y, plan.leg1_distance_m):
                self._log_detour(
                    f'LEG1 完成 {self._leg_traveled_m(x, y):.2f}m → TURN_BACK ψ₂={self._format_yaw_deg(plan.psi2)}°'
                )
                self._state = 'turn_back'
                self._turn_target_yaw = plan.psi2
            return True

        if self._state == 'turn_back':
            if self._turn_toward(plan.psi2, yaw):
                self._log_detour(
                    f'TURN_BACK 完成 yaw={self._format_yaw_deg(yaw)}° '
                    f'(目标 ψ₂={self._format_yaw_deg(plan.psi2)}°) → LEG2 {plan.leg2_distance_m:.2f}m'
                )
                self._state = 'leg2'
                self._mark_leg_start(x, y)
            return True

        if self._state == 'leg2':
            # 基础线速度
            linear = self.cfg.avoid_leg_linear_speed
            angular = 0.0
            
            # 尝试视觉修正（全程启用，只要有检测就修正）
            if self._vision_callback is not None:
                vision_angular = self._vision_callback()  # 返回 float 或 None
                if vision_angular is not None:
                    angular = vision_angular
            
            self.cmd_pub.publish(self._make_twist(linear, angular))
            
            if self._leg_done(x, y, plan.leg2_distance_m):
                self._log_detour(
                    f'LEG2 完成 {self._leg_traveled_m(x, y):.2f}m → TURN_RECOVER ψ₃={self._format_yaw_deg(plan.psi3)}°'
                )
                self._state = 'turn_recover'
                self._turn_target_yaw = plan.psi3
            return True

        if self._state == 'turn_recover':
            if self._turn_toward(plan.psi3, yaw):
                err_deg = math.degrees(abs(self._angle_error(plan.psi3, yaw)))
                self._log_detour(
                    f'TURN_RECOVER 完成 yaw={self._format_yaw_deg(yaw)}° '
                    f'(ψ₃={self._format_yaw_deg(plan.psi3)}° err={err_deg:.1f}°) → FINE_ALIGN'
                )
                self._state = 'fine_align'
            return True

        if self._state == 'fine_align':
            err = self._angle_error(plan.psi3, yaw)
            if abs(err) <= self.cfg.heading_tolerance:
                self._cooldown_until = self._now_sec() + self.cfg.detour_cooldown_sec
                self._log_detour(
                    f'FINE_ALIGN 完成 yaw={self._format_yaw_deg(yaw)}° '
                    f'(ψ₃={self._format_yaw_deg(plan.psi3)}° err={math.degrees(err):.1f}°) → 避障结束 '
                    f'(冷却 {self.cfg.detour_cooldown_sec:.0f}s)'
                )
                self._state = 'idle'
                self._plan = None
                self.cmd_pub.publish(self._zero_twist())
            else:
                omega = self._clamp(self.cfg.heading_kp * err, self.cfg.avoid_turn_angular_speed)
                if abs(omega) < self.cfg.turn_min_angular_speed:
                    omega = math.copysign(self.cfg.turn_min_angular_speed, err)
                self.cmd_pub.publish(self._make_twist(self.cfg.avoid_turn_linear_speed, omega))
            return True

        return self.is_active
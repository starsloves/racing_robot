"""
反应式避障模块 - 基于雷达实时距离反馈

核心思路：
- 触发时选择方向并锁定
- SHIFT_OUT: 快速切出建立侧移
- MAINTAIN: 保持安全距离通过障碍（死区+分段控制）
- MERGE_BACK: 反向贴墙回归轨道（利用侧向雷达距离反馈）

关键特点：
- 全程雷达反馈，不依赖横偏里程计累积
- 防抖动设计（死区、分段控制、角加速度限制、移动平均滤波）
- 天然支持连续障碍物
"""

from enum import IntEnum
from dataclasses import dataclass
from typing import Optional, Tuple, List
import math

from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from racing_common.racing_logger import RacingLogger


class AvoidanceState(IntEnum):
    """避障状态"""
    IDLE = 0           # 空闲，正常导航
    SHIFT_OUT = 1      # 快速切出建立侧移
    MAINTAIN = 2       # 保持距离通过障碍
    MERGE_BACK = 3     # 反向贴墙回归轨道


class DetourDirection(IntEnum):
    """绕行方向"""
    LEFT = 1           # 往左绕
    RIGHT = -1         # 往右绕


@dataclass
class ReactiveAvoidanceConfig:
    """反应式避障配置"""
    # 触发与方向选择
    trigger_distance_m: float = 0.55
    trigger_confirm_frames: int = 2
    direction_angle_threshold_deg: float = 5.0
    direction_clearance_margin_m: float = 0.10
    
    # 雷达扇区
    front_sector_angle_deg: float = 18.0
    side_sector_center_deg: float = 65.0
    side_sector_window_deg: float = 15.0
    
    # SHIFT_OUT 阶段
    shift_linear_speed: float = 0.12
    shift_omega_emergency: float = 0.65
    shift_omega_strong: float = 0.50
    shift_omega_side_near: float = 0.40
    shift_omega_normal: float = 0.35
    shift_cross_threshold_m: float = 0.20
    shift_side_threshold_m: float = 0.28
    shift_front_safe_m: float = 0.50
    shift_projection_threshold_m: float = 0.40
    
    # MAINTAIN 阶段
    maintain_linear_speed: float = 0.15
    maintain_target_side_distance_m: float = 0.32
    maintain_deadband_m: float = 0.05
    maintain_omega_very_near: float = -0.55
    maintain_omega_near: float = -0.30
    maintain_omega_far: float = 0.35
    maintain_omega_mid_far: float = 0.20
    maintain_front_protect_dist_m: float = 0.35
    maintain_front_protect_omega: float = 0.50
    maintain_front_protect_speed: float = 0.10
    maintain_to_merge_side_threshold_m: float = 0.70
    maintain_to_merge_front_threshold_m: float = 1.00
    maintain_to_merge_angle_threshold_deg: float = 90.0
    maintain_to_merge_confirm_frames: int = 3
    
    # MERGE_BACK 阶段
    merge_linear_speed_high_error: float = 0.08
    merge_linear_speed_low_error: float = 0.12
    merge_heading_threshold_deg: float = 15.0
    merge_obstacle_visible_dist_m: float = 1.50
    merge_obstacle_visible_angle_min_deg: float = 90.0
    merge_obstacle_visible_angle_max_deg: float = 150.0
    merge_omega_far: float = 0.30
    merge_omega_mid_far: float = 0.18
    merge_omega_near: float = -0.15
    merge_side_target_min_m: float = 0.28
    merge_side_target_max_m: float = 0.38
    merge_side_far_threshold_m: float = 0.50
    merge_heading_kp_with_obs: float = 2.0
    merge_heading_kp_no_obs: float = 2.5
    merge_finish_heading_tol_deg: float = 5.0
    merge_finish_confirm_frames: int = 5
    
    # 全局限制
    max_omega_rate: float = 2.0  # rad/s²
    max_projection_distance_m: float = 1.00
    emergency_merge_threshold_m: float = 0.85
    distance_filter_window: int = 3
    avoidance_timeout_sec: float = 8.0
    cooldown_sec: float = 2.0
    
    # 动态角度跟踪
    dynamic_angle_window_deg: float = 30.0  # 动态角度搜索窗口（半宽15°）


class MovingAverageFilter:
    """移动平均滤波器"""
    def __init__(self, window: int = 3):
        self.window = window
        self.buffer = []
    
    def update(self, value: float) -> float:
        self.buffer.append(value)
        if len(self.buffer) > self.window:
            self.buffer.pop(0)
        return sum(self.buffer) / len(self.buffer) if self.buffer else value
    
    def reset(self):
        self.buffer.clear()


class OmegaSmoother:
    """角速度平滑器（角加速度限制）"""
    def __init__(self, max_rate: float = 2.0):
        self.max_rate = max_rate  # rad/s²
        self.last_omega = 0.0
        self.last_time = None
    
    def smooth(self, target_omega: float, current_time) -> float:
        if self.last_time is None:
            self.last_time = current_time
            self.last_omega = target_omega
            return target_omega
        
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        if dt <= 0:
            return self.last_omega
        
        max_delta = self.max_rate * dt
        delta = target_omega - self.last_omega
        
        if delta > max_delta:
            output = self.last_omega + max_delta
        elif delta < -max_delta:
            output = self.last_omega - max_delta
        else:
            output = target_omega
        
        self.last_omega = output
        self.last_time = current_time
        return output
    
    def reset(self):
        self.last_omega = 0.0
        self.last_time = None


class ReactiveAvoidanceManager:
    """反应式避障管理器"""
    
    def __init__(self, cmd_pub, logger: RacingLogger, clock: Clock, cfg: ReactiveAvoidanceConfig):
        self._cmd_pub = cmd_pub
        self._log = logger
        self._clock = clock
        self._cfg = cfg
        
        # 状态
        self._state = AvoidanceState.IDLE
        self._detour_direction: Optional[int] = None  # DetourDirection
        
        # 初始状态记录
        self._start_position: Optional[Tuple[float, float]] = None
        self._start_yaw: Optional[float] = None
        self._last_position: Optional[Tuple[float, float]] = None
        self._current_yaw: Optional[float] = None
        self._accumulated_projection = 0.0
        self._obstacle_initial_angle: Optional[float] = None  # 触发时障碍物相对车体的角度（度）
        
        # 雷达数据（滤波后）
        self._front_distance = 999.0
        self._front_angle = 0.0
        self._left_clearance = 999.0
        self._right_clearance = 999.0
        self._side_angle = 0.0  # 绕行侧障碍物角度
        
        # 原始雷达数据（用于记录）
        self._front_distance_raw = 999.0
        self._left_clearance_raw = 999.0
        self._right_clearance_raw = 999.0
        
        # 原始扫描数据（用于动态角度查询）
        self._scan_ranges: List[float] = []
        self._scan_angle_min = 0.0
        self._scan_angle_increment = 0.0
        
        # 滤波器
        self._front_filter = MovingAverageFilter(window=cfg.distance_filter_window)
        self._side_filter = MovingAverageFilter(window=cfg.distance_filter_window)
        
        # 平滑器
        self._omega_smoother = OmegaSmoother(max_rate=cfg.max_omega_rate)
        
        # 状态切换确认计数
        self._transition_confirm_count = 0
        self._trigger_confirm_count = 0
        
        # 完成确认计数
        self._finish_confirm_count = 0
        
        # 超时保护
        self._start_time = None
        
        # 冷却时间
        self._last_finish_time = None
    
    def on_scan(self, front_dist: float, front_angle: float, 
                left_clear: float, right_clear: float, side_angle: float = 0.0,
                scan_msg: Optional[LaserScan] = None):
        """雷达回调
        
        Args:
            front_dist: 前方扇区最近障碍距离（m）
            front_angle: 前方障碍角度（度）
            left_clear: 左侧扇区净空（m）
            right_clear: 右侧扇区净空（m）
            side_angle: 侧向障碍物角度（度，可选）
            scan_msg: 原始激光扫描消息（用于动态角度查询）
        """
        # 保存原始数据
        self._front_distance_raw = front_dist
        self._left_clearance_raw = left_clear
        self._right_clearance_raw = right_clear
        
        # 保存原始扫描数据
        if scan_msg is not None:
            self._scan_ranges = list(scan_msg.ranges)
            self._scan_angle_min = scan_msg.angle_min
            self._scan_angle_increment = scan_msg.angle_increment
        
        # 更新滤波数据
        self._front_distance = self._front_filter.update(front_dist)
        self._front_angle = front_angle
        self._left_clearance = left_clear
        self._right_clearance = right_clear
        self._side_angle = side_angle
    
    def _dynamic_side_angle_deg(self) -> float:
        """计算动态侧向角度
        
        根据当前航向变化量，计算障碍物应该在的方向。
        往左绕 → 障碍物在右边 → 角度为负
        往右绕 → 障碍物在左边 → 角度为正
        
        Returns:
            动态侧向角度（度）
        """
        if self._start_yaw is None:
            return 0.0
        
        yaw_change = self._current_yaw - self._start_yaw
        yaw_change_deg = math.degrees(yaw_change)
        
        # 往左绕：障碍物在右侧，角度 = -|yaw_change|
        # 往右绕：障碍物在左侧，角度 = +|yaw_change|
        dynamic_angle = -self._detour_direction * abs(yaw_change_deg)
        
        # 限制最小值（避免角度太小时看不到）
        if abs(dynamic_angle) < 15.0:
            dynamic_angle = -self._detour_direction * 15.0
        
        return dynamic_angle
    
    def _distance_at_angle(self, center_angle_deg: float, window_deg: float = 30.0) -> float:
        """在原始扫描数据中，查询指定角度附近的最近障碍距离
        
        Args:
            center_angle_deg: 查询中心角度（度）
            window_deg: 搜索窗口大小（度）
        
        Returns:
            最近障碍距离（m），无数据则返回 inf
        """
        if not self._scan_ranges:
            self._log.warn('REACT_DYN', "原始扫描数据为空")
            return float('inf')
        
        half = window_deg / 2.0
        min_angle = center_angle_deg - half
        max_angle = center_angle_deg + half
        
        min_dist = float('inf')
        valid_count = 0
        distances_in_window = []  # 调试：记录窗口内的所有距离
        
        for i, dist in enumerate(self._scan_ranges):
            if math.isinf(dist) or math.isnan(dist) or dist <= 0.0:
                continue
            angle = math.degrees(self._scan_angle_min + i * self._scan_angle_increment)
            angle = (angle + 180.0) % 360.0 - 180.0
            if angle < min_angle or angle > max_angle:
                continue
            valid_count += 1
            distances_in_window.append(dist)
            if dist < min_dist:
                min_dist = dist
        
        if valid_count == 0:
            self._log.warn('REACT_DYN', f"角度 {center_angle_deg:.1f}° 窗口 {window_deg:.0f}° 范围内无有效点")
        elif min_dist > 1.0:  # 如果最小距离 > 1.0m，输出详细信息
            sorted_dists = sorted(distances_in_window)[:5]  # 前5个最小值
            self._log.warn('REACT_DYN', f"角度 {center_angle_deg:.1f}° 窗口内最小5个距离: {sorted_dists}, 点数={valid_count}")
        
        return min_dist
    
    def should_trigger(self) -> bool:
        """检查是否应触发避障"""
        if self._state != AvoidanceState.IDLE:
            return False
        
        # 冷却时间检查
        if self._last_finish_time is not None:
            elapsed = (self._clock.now().nanoseconds - self._last_finish_time) / 1e9
            if elapsed < self._cfg.cooldown_sec:
                return False
        
        # 触发条件：前方距离 <= 阈值
        if self._front_distance <= self._cfg.trigger_distance_m:
            self._trigger_confirm_count += 1
            if self._trigger_confirm_count >= self._cfg.trigger_confirm_frames:
                return True
        else:
            self._trigger_confirm_count = 0
        
        return False
    
    def start(self, yaw: float, position: Tuple[float, float]):
        """启动避障
        
        Args:
            yaw: 当前航向（弧度）
            position: 当前位置 (x, y)
        """
        if self._state != AvoidanceState.IDLE:
            self._log.warn('AVOID', "避障已在运行，忽略启动请求")
            return
        
        # 选择绕行方向
        direction = self._select_direction()
        
        # 记录初始状态
        self._start_position = position
        self._last_position = position
        self._start_yaw = yaw
        self._detour_direction = direction
        self._accumulated_projection = 0.0
        self._start_time = self._clock.now().nanoseconds
        self._obstacle_initial_angle = self._front_angle  # 记录障碍物初始角度
        
        # 重置滤波器和平滑器
        self._side_filter.reset()
        self._omega_smoother.reset()
        
        # 重置计数器
        self._transition_confirm_count = 0
        self._finish_confirm_count = 0
        
        # 切换到 SHIFT_OUT
        self._state = AvoidanceState.SHIFT_OUT
        
        direction_str = "LEFT" if direction == DetourDirection.LEFT else "RIGHT"
        self._log.info(
            'AVOID',
            f"避障启动: 方向={direction_str}, "
            f"前方={self._front_distance:.2f}m, 角度={self._front_angle:.1f}°, "
            f"左侧={self._left_clearance:.2f}m, 右侧={self._right_clearance:.2f}m"
        )
    
    def _select_direction(self) -> int:
        """选择绕行方向
        
        Returns:
            DetourDirection.LEFT (+1) 或 DetourDirection.RIGHT (-1)
        """
        # 方案1：根据障碍物角度
        if self._front_angle < -self._cfg.direction_angle_threshold_deg:
            # 障碍物偏右 → 往左绕
            return DetourDirection.LEFT
        elif self._front_angle > self._cfg.direction_angle_threshold_deg:
            # 障碍物偏左 → 往右绕
            return DetourDirection.RIGHT
        
        # 方案2：比较左右净空
        clearance_diff = self._left_clearance - self._right_clearance
        if clearance_diff > self._cfg.direction_clearance_margin_m:
            return DetourDirection.LEFT
        else:
            return DetourDirection.RIGHT
    
    def step(self, yaw: float, position: Tuple[float, float]) -> bool:
        """主控制循环
        
        Args:
            yaw: 当前航向（弧度）
            position: 当前位置 (x, y)
        
        Returns:
            True = 继续避障, False = 完成
        """
        if self._state == AvoidanceState.IDLE:
            return False
        
        # 跟踪当前航向
        self._current_yaw = yaw
        
        # 超时保护
        elapsed = (self._clock.now().nanoseconds - self._start_time) / 1e9
        if elapsed > self._cfg.avoidance_timeout_sec:
            self._log.warn('AVOID', f"避障超时 ({elapsed:.1f}s)，强制完成")
            self._finish()
            return False
        
        # 更新累积投影距离
        self._update_projection(position)
        
        # 根据状态执行
        if self._state == AvoidanceState.SHIFT_OUT:
            return self._step_shift_out(yaw, position)
        elif self._state == AvoidanceState.MAINTAIN:
            return self._step_maintain(yaw, position)
        elif self._state == AvoidanceState.MERGE_BACK:
            return self._step_merge_back(yaw, position)
        
        return False
    
    def _update_projection(self, position: Tuple[float, float]):
        """更新投影距离"""
        if self._last_position is None:
            self._last_position = position
            return
        
        dx = position[0] - self._last_position[0]
        dy = position[1] - self._last_position[1]
        ds = dx * math.cos(self._start_yaw) + dy * math.sin(self._start_yaw)
        self._accumulated_projection += ds
        self._last_position = position
    
    def _step_shift_out(self, yaw: float, position: Tuple[float, float]) -> bool:
        """SHIFT_OUT 阶段控制"""
        # 更新侧向雷达（根据绕行方向）
        # 往左绕 → 障碍物在右边 → 看右侧距离
        # 往右绕 → 障碍物在左边 → 看左侧距离
        if self._detour_direction == DetourDirection.LEFT:
            side_distance = self._right_clearance
        else:
            side_distance = self._left_clearance
        
        side_distance_filtered = self._side_filter.update(side_distance)
        
        # 估算横偏（粗略）
        dx = position[0] - self._start_position[0]
        dy = position[1] - self._start_position[1]
        cross_offset = abs(-dx * math.sin(self._start_yaw) + dy * math.cos(self._start_yaw))
        
        # 分段控制角速度
        if self._front_distance < 0.35:
            target_omega = self._cfg.shift_omega_emergency
        elif self._front_distance < 0.45:
            target_omega = self._cfg.shift_omega_strong
        elif side_distance_filtered < 0.20:
            target_omega = self._cfg.shift_omega_side_near
        else:
            target_omega = self._cfg.shift_omega_normal
        
        # 应用方向
        target_omega *= self._detour_direction
        
        # 平滑输出
        omega = self._omega_smoother.smooth(target_omega, self._clock.now())
        linear = self._cfg.shift_linear_speed
        
        # 发布指令
        self._publish_cmd(linear, omega)
        
        # 日志
        self._log.info(
            'REACT_SHIFT',
            f"front={self._front_distance:.2f}m @ {self._front_angle:.1f}°, side={side_distance_filtered:.2f}m, "
            f"cross={cross_offset:.3f}m, proj={self._accumulated_projection:.2f}m, ω={omega:.2f}"
        )
        
        # 切换条件检查
        yaw_change = yaw - self._start_yaw
        yaw_change_deg = math.degrees(yaw_change)
        
        cond1 = cross_offset >= self._cfg.shift_cross_threshold_m  # 横偏足够
        cond3 = self._accumulated_projection >= self._cfg.shift_projection_threshold_m  # 前进距离足够
        cond4 = abs(yaw_change_deg) >= 20.0  # 已经转了至少20°
        
        switch_reason = None
        if cond1:
            switch_reason = f"cross={cross_offset:.3f}m"
        elif cond4:
            switch_reason = f"yaw={yaw_change_deg:.1f}°"
        elif cond3:
            switch_reason = f"proj={self._accumulated_projection:.2f}m"
        
        if switch_reason:
            self._state = AvoidanceState.MAINTAIN
            self._transition_confirm_count = 0
            self._log.info(
                'AVOID',
                f"切换到 MAINTAIN: {switch_reason}"
            )
        
        return True
    
    def _step_maintain(self, yaw: float, position: Tuple[float, float]) -> bool:
        """MAINTAIN 阶段控制
        
        锁定障碍物：计算障碍物当前相对车体的角度，查询该方向的距离
        """
        # 计算航向变化
        yaw_change = yaw - self._start_yaw
        yaw_change_deg = math.degrees(yaw_change)
        
        # 障碍物当前相对车体的角度 = 初始角度 - 航向变化
        # 例：初始 -15°，左转 +30° → 障碍物相对角 = -15° - 30° = -45°（更偏右）
        obstacle_relative_angle = self._obstacle_initial_angle - yaw_change_deg
        
        # 在该角度附近查询距离（窗口 ±15°）
        obs_distance_raw = self._distance_at_angle(obstacle_relative_angle, 30.0)
        
        # 如果查询失败或距离过远（说明查到远处物体），回退到前方雷达
        if math.isinf(obs_distance_raw) or obs_distance_raw > 1.5:
            obs_distance = self._front_distance
            self._log.warn('REACT_MAINTAIN', f"动态查询失败或过远 ({obs_distance_raw:.2f}m)，回退到前方雷达")
        else:
            obs_distance = obs_distance_raw
        
        obs_distance_filtered = self._side_filter.update(obs_distance)
        
        # 距离误差控制
        distance_error = obs_distance_filtered - self._cfg.maintain_target_side_distance_m
        
        if distance_error < -0.12:  # 很近（< 0.20m）→ 远离
            target_omega = self._cfg.maintain_omega_very_near
        elif distance_error < -self._cfg.maintain_deadband_m:  # 稍近（0.20-0.27m）
            target_omega = self._cfg.maintain_omega_near
        elif distance_error > 0.12:  # 很远（> 0.44m）→ 靠近
            target_omega = self._cfg.maintain_omega_far
        elif distance_error > self._cfg.maintain_deadband_m:  # 稍远（0.37-0.44m）
            target_omega = self._cfg.maintain_omega_mid_far
        else:  # 死区（0.27-0.37m），不调整
            target_omega = 0.0
        
        # 应用方向
        target_omega *= self._detour_direction
        
        # 前方保护
        linear = self._cfg.maintain_linear_speed
        if self._front_distance < 0.30:
            target_omega = self._detour_direction * self._cfg.maintain_front_protect_omega
            linear = self._cfg.maintain_front_protect_speed
        
        # 平滑输出
        omega = self._omega_smoother.smooth(target_omega, self._clock.now())
        
        # 发布指令
        self._publish_cmd(linear, omega)
        
        # 日志
        self._log.info(
            'REACT_MAINTAIN',
            f"obs_angle={obstacle_relative_angle:.1f}°, obs_dist={obs_distance_filtered:.2f}m, "
            f"yaw_chg={yaw_change_deg:.1f}°, front={self._front_distance:.2f}m, ω={omega:.2f}"
        )
        
        # 切换条件：障碍物移到侧后方（相对角 > 100°）且前方开阔
        obs_passed = abs(obstacle_relative_angle) > 100.0  # 障碍物在侧后方
        front_clear = self._front_distance > 0.80  # 前方净空 > 0.8m
        
        if obs_passed and front_clear:
            self._transition_confirm_count += 1
            if self._transition_confirm_count >= 2:  # 确认 2 帧
                self._state = AvoidanceState.MERGE_BACK
                self._finish_confirm_count = 0
                self._log.info(
                    'AVOID',
                    f"切换到 MERGE_BACK: obs_angle={obstacle_relative_angle:.1f}°, front={self._front_distance:.2f}m"
                )
        else:
            self._transition_confirm_count = 0
        
        # 投影距离保护
        if self._accumulated_projection > self._cfg.emergency_merge_threshold_m:
            self._log.warn('AVOID', f"投影距离超限 ({self._accumulated_projection:.2f}m)，强制切换到 MERGE_BACK")
            self._state = AvoidanceState.MERGE_BACK
            self._finish_confirm_count = 0
        
        return True
    
    def _step_merge_back(self, yaw: float, position: Tuple[float, float]) -> bool:
        """MERGE_BACK 阶段控制（回归轨道）
        
        利用动态角度跟踪障碍物距离，回归到初始航向
        """
        # 航向误差
        heading_error = yaw - self._start_yaw
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        heading_error_deg = math.degrees(heading_error)
        
        # 用动态角度查询障碍物距离
        dynamic_angle = self._dynamic_side_angle_deg()
        obs_distance = self._distance_at_angle(dynamic_angle, self._cfg.dynamic_angle_window_deg)
        
        if math.isinf(obs_distance):
            # 保底使用固定侧向雷达
            if self._detour_direction == DetourDirection.LEFT:
                obs_distance = self._right_clearance
            else:
                obs_distance = self._left_clearance
        
        obs_distance_filtered = self._side_filter.update(obs_distance)
        
        # 障碍物是否可见（距离 < 1.5m）
        obstacle_visible = obs_distance_filtered < self._cfg.merge_obstacle_visible_dist_m
        
        if obstacle_visible:
            # 利用距离反馈回归
            if obs_distance_filtered > self._cfg.merge_side_far_threshold_m:
                target_omega = self._detour_direction * self._cfg.merge_omega_far
            elif obs_distance_filtered > self._cfg.merge_side_target_max_m:
                target_omega = self._detour_direction * self._cfg.merge_omega_mid_far
            elif obs_distance_filtered < self._cfg.merge_side_target_min_m:
                target_omega = self._detour_direction * self._cfg.merge_omega_near
            else:
                target_omega = -self._cfg.merge_heading_kp_with_obs * heading_error
        else:
            # 纯航向回正
            target_omega = -self._cfg.merge_heading_kp_no_obs * heading_error
        
        # 速度策略
        if abs(heading_error_deg) > self._cfg.merge_heading_threshold_deg:
            linear = self._cfg.merge_linear_speed_high_error
        else:
            linear = self._cfg.merge_linear_speed_low_error
        
        # 平滑输出
        omega = self._omega_smoother.smooth(target_omega, self._clock.now())
        
        # 发布指令
        self._publish_cmd(linear, omega)
        
        # 日志
        self._log.info(
            'REACT_MERGE',
            f"heading_err={heading_error_deg:.1f}°, obs={obs_distance_filtered:.2f}m, "
            f"obs_visible={obstacle_visible}, ω={omega:.2f}"
        )
        
        # 完成条件：航向回正
        heading_ok = abs(heading_error_deg) <= self._cfg.merge_finish_heading_tol_deg
        
        if heading_ok:
            self._finish_confirm_count += 1
            if self._finish_confirm_count >= self._cfg.merge_finish_confirm_frames:
                self._log.info(
                    'AVOID',
                    f"避障完成: heading={heading_error_deg:.1f}°, obs={obs_distance_filtered:.2f}m, "
                    f"proj={self._accumulated_projection:.2f}m"
                )
                self._finish()
                return False
        else:
            self._finish_confirm_count = 0
        
        # 投影距离强制完成
        if self._accumulated_projection > self._cfg.max_projection_distance_m:
            if abs(heading_error_deg) <= 10.0:
                self._log.warn(
                    'AVOID',
                    f"投影距离超限 ({self._accumulated_projection:.2f}m)，航向基本回正，强制完成"
                )
                self._finish()
                return False
        
        return True
    
    def _publish_cmd(self, linear: float, omega: float):
        """发布速度指令"""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = omega
        self._cmd_pub.publish(cmd)
    
    def _finish(self):
        """完成避障"""
        self._state = AvoidanceState.IDLE
        self._detour_direction = None
        self._start_position = None
        self._start_yaw = None
        self._last_position = None
        self._accumulated_projection = 0.0
        self._start_time = None
        self._last_finish_time = self._clock.now().nanoseconds
        
        # 重置计数器
        self._trigger_confirm_count = 0
        self._transition_confirm_count = 0
        self._finish_confirm_count = 0
        
        # 停车
        self._publish_cmd(0.0, 0.0)
    
    @property
    def is_active(self) -> bool:
        """是否正在避障"""
        return self._state != AvoidanceState.IDLE
    
    @property
    def current_state(self) -> AvoidanceState:
        """当前状态"""
        return self._state
    
    @property
    def projection_distance(self) -> float:
        """累积投影距离"""
        return self._accumulated_projection

"""
spiral_avoider.py — 独立避障模块（完全解耦设计）

设计原则：
- 完全独立于惯导逻辑
- 惯导检测到障碍 → 交权给避障
- 避障完成 → 交权回惯导
- 避障期间完全接管 /cmd_vel 发布

支持方案：
- 渐进式螺旋避障（平滑无抖动）
- 恒定曲率避障（简单快速）
- 可通过配置切换

使用方法：
    avoider = SpiralAvoider(...)
    
    # 主循环
    while True:
        if avoider.is_active:
            # 避障中，避障模块接管
            if not avoider.step(current_yaw):
                # 避障完成，恢复惯导控制
                pass
        else:
            # 检测障碍
            if detect_obstacle():
                avoider.start(obstacle_left)
            else:
                # 正常惯导控制
                publish_navigation_cmd()
"""

import math
from dataclasses import dataclass
from geometry_msgs.msg import Twist


@dataclass
class SpiralConfig:
    """螺旋避障配置"""
    # 模式选择
    mode: str = 'spiral'                     # 'spiral' 或 'constant_curvature'
    
    # ── 螺旋方案参数 ────────────────────────────────
    linear_speed: float = 0.15               # 线速度（m/s）
    angular_initial: float = 2.0             # 初始角速度（rad/s）
    total_time: float = 2.0                  # 单阶段时长（s）
    control_rate_hz: float = 20.0            # 控制频率（Hz）
    min_angular_threshold: float = 0.05      # 停止阈值（rad/s）
    
    # ── 恒定曲率方案参数 ────────────────────────────
    constant_angular: float = 0.70           # 固定角速度（rad/s）
    constant_delta_theta_deg: float = 30.0   # 偏转角度（度）
    heading_tolerance_deg: float = 2.0       # 转弯容差（度）
    
    # ── Phase2 提前退出参数 ─────────────────────────
    phase2_exit_overshoot_deg: float = 8.0   # Phase2 过冲角度，退出时车头朝向中线（度）
    
    # ── 固定转角方案参数 ────────────────────────────
    phase1_turn_angle_deg: float = 30.0      # Phase1 目标航向偏移（度）
    phase1_duration_s: float = 0.8           # Phase1 保持时长（s）
    phase2_turn_angle_deg: float = 30.0      # Phase2 目标航向偏移（度，相对 Phase1 结束航向）
    phase2_duration_s: float = 0.8           # Phase2 保持时长（s）
    phase3_turn_angle_deg: float = 10.0      # Phase3 航向微调偏移（度，自动决定方向）
    phase3_duration_s: float = 0.5           # Phase3 保持时长（s）
    enable_phase3: bool = False              # 是否启用 Phase3 航向微调
    heading_control_kp: float = 2.0          # 航向控制增益（rad/s per rad error）
    
    # ── 方向选择 ────────────────────────────────────
    side_obstacle_threshold_m: float = 0.25  # 侧边障碍判定阈值
    front_angle_threshold_deg: float = 5.0   # 障碍物偏向判定角度（原15°太大，改5°）
    
    # ── 安全检查 ────────────────────────────────────
    timeout_sec: float = 5.0                 # 总超时（Phase1+2 约 3s，5s 足够）
    min_front_clearance_m: float = 0.20      # 紧急停车距离（m）
    
    # ── 日志 ────────────────────────────────────────
    verbose: bool = True                     # 详细日志
    
    # ── 调试停车 ────────────────────────────────────
    stage_pause_enabled: bool = False        # 阶段停车总开关
    stage_pause_duration_sec: float = 5.0    # 停车观察时长（秒）
    stage_pause_on_trigger: bool = False     # Stage 0: 检测到障碍触发避障时
    stage_pause_on_phase1_end: bool = False  # Stage 1: Phase1 完成（右转绕障结束）
    stage_pause_on_phase2_end: bool = False  # Stage 2: Phase2 完成（回正结束）
    stage_pause_on_phase3_end: bool = False  # Stage 3: Phase3 完成（航向微调结束）


class SpiralAvoider:
    """独立避障模块（完全解耦）"""
    
    def __init__(self, cmd_pub, logger, clock, cfg=None, cmd_observer=None):
        self.cmd_pub = cmd_pub
        self._log = logger
        self._clock = clock
        self.cfg = cfg or SpiralConfig()
        self._cmd_observer = cmd_observer
        
        # 状态变量（两阶段：phase1_away / phase2_back）
        self._state = 'idle'             # 'idle', 'phase1_away', 'phase2_back', 'phase3_fine_tune'
        self._start_yaw = None           # 避障开始时的航向
        self._start_time = None          # 避障开始时间
        self._phase1_start_time = None   # 阶段1开始时间
        self._phase2_start_time = None   # 阶段2开始时间
        self._phase3_start_time = None   # 阶段3开始时间
        self._omega_sign = 1.0           # 转向符号（+1左转, -1右转）
        self._target_yaw = None          # 当前目标航向（rad）
        self._phase1_end_yaw = None      # Phase1 结束时的航向（用于 Phase2 计算偏移）
        self._phase2_end_yaw = None      # Phase2 结束时的航向（用于 Phase3 计算偏移）
        
        # 动态速度与段参数（由主控传入）
        self._active_linear_speed = self.cfg.linear_speed  # 当前段速度
        
        # 传感器数据（由外部更新）
        self.front_distance = float('inf')
        self.front_angle_deg = 0.0
        self.left_clearance = float('inf')
        self.right_clearance = float('inf')
        
        # 统计
        self._total_avoidances = 0
        self._last_direction = None
        
        # 调试停车状态
        self._pause_state = None         # 'trigger', 'phase1_end', 'phase2_end', None
        self._pause_start_time = None    # 停车开始时间
        self._pending_detour_right = None  # 触发时保存的绕行方向
        self._pending_linear_speed = None  # 触发时保存的速度
    
    # ══════════════════════════════════════════════════
    # 外部接口
    # ══════════════════════════════════════════════════
    
    def reset(self):
        """重置避障器"""
        prev = self._state
        self._state = 'idle'
        self._start_yaw = None
        self._start_time = None
        self._phase1_start_time = None
        self._phase2_start_time = None
        self._phase3_start_time = None
        self._target_yaw = None
        self._phase1_end_yaw = None
        self._phase2_end_yaw = None
        self._active_linear_speed = self.cfg.linear_speed
        self._pause_state = None
        self._pause_start_time = None
        self._pending_detour_right = None
        self._pending_linear_speed = None
        
        if prev != 'idle' and self.cfg.verbose:
            self._log.info('AVOID', f'状态 {prev} → idle')
    
    @property
    def is_active(self) -> bool:
        """避障是否正在运行"""
        return self._state != 'idle'
    
    def on_scan(self, front_distance, front_angle_deg, 
                left_clearance, right_clearance):
        """更新传感器数据（由主控调用）"""
        self.front_distance = front_distance
        self.front_angle_deg = front_angle_deg
        self.left_clearance = left_clearance
        self.right_clearance = right_clearance
    
    def should_trigger(self, trigger_distance=0.55) -> bool:
        """判断是否应该触发避障（由主控调用）"""
        if self._state != 'idle':
            return False
        return (math.isfinite(self.front_distance) and 
                self.front_distance < trigger_distance)
    
    def start(self, current_yaw, detour_right=None, linear_speed=None):
        """启动避障（由主控调用）
        
        Args:
            current_yaw: 当前航向（rad）
            detour_right: 是否往右侧绕行（True=右绕，False=左绕，None=自动判断）
            linear_speed: 当前直行段速度（m/s），None=用配置默认值
        
        Returns:
            bool: 是否成功启动
        """
        if self._state != 'idle':
            self._log.warn('AVOID', '已在运行中，忽略启动请求')
            return False
        
        # 确定绕行方向
        if detour_right is None:
            direction = self._select_detour_direction()
            if direction is None:
                self._log.warn('AVOID', '无法选择绕行方向，两侧空间不足')
                return False
            detour_right = (direction == 'right')
        
        # ★ 触发停车检查
        if self.cfg.stage_pause_enabled and self.cfg.stage_pause_on_trigger:
            self._state = 'pause_trigger'
            self._pause_state = 'trigger'
            self._pause_start_time = self._now_sec()
            self._start_yaw = current_yaw
            self._omega_sign = -1.0 if detour_right else +1.0
            self._pending_detour_right = detour_right
            self._pending_linear_speed = linear_speed if linear_speed is not None else self.cfg.linear_speed
            
            if self.cfg.verbose:
                self._log.info('STAGE_PAUSE', '═' * 70)
                self._log.info('STAGE_PAUSE', '🛑 Stage 0: 检测到障碍，触发避障')
                self._log.info('STAGE_PAUSE', '═' * 70)
                self._log.info('STAGE_PAUSE', f'前方距离: {self.front_distance:.2f}m @ {self.front_angle_deg:.1f}°')
                self._log.info('STAGE_PAUSE', f'左侧间隙: {self.left_clearance:.2f}m')
                self._log.info('STAGE_PAUSE', f'右侧间隙: {self.right_clearance:.2f}m')
                self._log.info('STAGE_PAUSE', f'选择方向: {"右绕" if detour_right else "左绕"}')
                self._log.info('STAGE_PAUSE', f'当前航向: {math.degrees(current_yaw):.1f}°')
                self._log.info('STAGE_PAUSE', '═' * 70)
                self._log.info('STAGE_PAUSE', f'⏸️  停车观察 {self.cfg.stage_pause_duration_sec:.0f} 秒...')
                self._log.info('STAGE_PAUSE', '═' * 70)
            
            return True
        
        # 正常启动避障（不停车）
        self._state = 'phase1_away'
        self._start_yaw = current_yaw
        self._start_time = self._now_sec()
        self._phase1_start_time = self._start_time
        self._omega_sign = -1.0 if detour_right else +1.0  # 右绕=右转(ω负), 左绕=左转(ω正)
        self._total_avoidances += 1
        self._last_direction = 'right' if detour_right else 'left'
        
        # 记录段速度
        self._active_linear_speed = linear_speed if linear_speed is not None else self.cfg.linear_speed
        
        if self.cfg.verbose:
            mode_text = '螺旋' if self.cfg.mode == 'spiral' else '恒定曲率'
            self._log.info('AVOID',
                f'启动 #{self._total_avoidances} '
                f'模式={mode_text} '
                f'往{"右" if detour_right else "左"}侧绕行 '
                f'v={self._active_linear_speed:.2f}m/s'
            )
        
        return True
    
    def step(self, current_yaw) -> bool:
        """避障主循环（由主控在每个控制周期调用）
        
        Args:
            current_yaw: 当前航向（rad）
        
        Returns:
            bool: True=避障中（继续调用），False=避障完成（恢复惯导）
        """
        if self._state == 'idle':
            return False
        
        now = self._now_sec()
        
        # ★ 处理停车状态
        if self._state in ['pause_trigger', 'pause_phase1_end', 'pause_phase2_end', 'pause_phase3_end']:
            elapsed = now - self._pause_start_time
            
            # 发布停车指令
            self._publish_cmd(0.0, 0.0)
            
            # 检查是否停车时间到
            if elapsed >= self.cfg.stage_pause_duration_sec:
                if self.cfg.verbose:
                    self._log.info('STAGE_RESUME', f'停车观察结束，继续执行')
                
                # 根据停车原因恢复对应状态
                if self._state == 'pause_trigger':
                    # 触发停车结束 → 开始 Phase1
                    self._state = 'phase1_away'
                    self._start_time = now
                    self._phase1_start_time = now
                    self._total_avoidances += 1
                    self._last_direction = 'right' if self._pending_detour_right else 'left'
                    self._active_linear_speed = self._pending_linear_speed
                    self._target_yaw = None
                    
                    if self.cfg.verbose:
                        mode_text = '固定转角'
                        self._log.info('AVOID',
                            f'启动 #{self._total_avoidances} '
                            f'模式={mode_text} '
                            f'往{"右" if self._pending_detour_right else "左"}侧绕行 '
                            f'v={self._active_linear_speed:.2f}m/s'
                        )
                
                elif self._state == 'pause_phase1_end':
                    # Phase1 停车结束 → 进入 Phase2
                    self._state = 'phase2_back'
                    self._phase2_start_time = now
                    self._target_yaw = None
                
                elif self._state == 'pause_phase2_end':
                    # Phase2 停车结束 → 检查是否进入 Phase3
                    if self.cfg.enable_phase3:
                        self._state = 'phase3_fine_tune'
                        self._phase3_start_time = now
                        self._target_yaw = None
                        if self.cfg.verbose:
                            self._log.info('AVOID', '进入 Phase3 航向微调')
                    else:
                        # 不启用 Phase3，直接结束
                        self.reset()
                        self._publish_cmd()
                        return False
                
                elif self._state == 'pause_phase3_end':
                    # Phase3 停车结束 → 避障完成
                    self.reset()
                    self._publish_cmd()
                    return False
                
                # 清除停车状态
                self._pause_state = None
                self._pause_start_time = None
                self._pending_detour_right = None
                self._pending_linear_speed = None
            
            return True  # 继续停车或刚恢复
        
        # 超时保护
        if self._start_time and now - self._start_time > self.cfg.timeout_sec:
            self._log.warn('AVOID', f'超时（{now - self._start_time:.1f}s），强制退出')
            self.reset()
            self._publish_cmd()
            return False
        
        # 紧急停车检查
        if (math.isfinite(self.front_distance) and 
            self.front_distance < self.cfg.min_front_clearance_m):
            self._log.warn('AVOID', f'紧急停车！前方 {self.front_distance:.2f}m')
            self._publish_cmd()
            return True  # 保持避障状态，等待障碍物移开
        
        # 根据模式选择执行函数
        if self.cfg.mode == 'spiral':
            return self._step_spiral(current_yaw)
        elif self.cfg.mode == 'fixed_timing':
            return self._step_fixed_timing(current_yaw)
        else:
            return self._step_constant_curvature(current_yaw)
    
    # ══════════════════════════════════════════════════
    # 内部辅助
    # ══════════════════════════════════════════════════
    
    def _publish_cmd(self, linear_x=0.0, angular_z=0.0):
        """统一发布命令并同步日志"""
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        if self._cmd_observer is not None:
            self._cmd_observer(float(linear_x), float(angular_z))
        self.cmd_pub.publish(cmd)
    
    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))
    
    # ══════════════════════════════════════════════════
    # 螺旋方案实现
    # ══════════════════════════════════════════════════
    
    def _step_spiral(self, current_yaw) -> bool:
        """螺旋避障步进"""
        now = self._now_sec()
        
        # ── 阶段1：偏离 ─────────────────────────────
        if self._state == 'phase1_away':
            elapsed = now - self._phase1_start_time
            total_time = self.cfg.total_time
            
            # 计算当前角速度（线性衰减）
            omega = self.cfg.angular_initial * (1.0 - elapsed / total_time)
            omega = max(0.0, omega)
            
            self._publish_cmd(
                self._active_linear_speed,
                self._omega_sign * omega,
            )
            
            # 阶段1完成判断
            if omega < self.cfg.min_angular_threshold or elapsed >= total_time:
                if (math.isfinite(self.front_distance) and 
                    self.front_distance < self.cfg.continuous_check_distance):
                    self._is_continuous = True
                    if self.cfg.verbose:
                        self._log.info('AVOID', '检测到连续障碍，延长阶段2')
                
                self._state = 'phase2_back'
                self._phase2_start_time = now
                if self.cfg.verbose:
                    self._log.info('AVOID', f'阶段1完成 耗时={elapsed:.2f}s')
            
            return True
        
        # ── 阶段2：回正 ─────────────────────────────
        elif self._state == 'phase2_back':
            elapsed = now - self._phase2_start_time
            total_time = self.cfg.total_time
            if self._is_continuous:
                total_time *= self.cfg.continuous_time_multiplier
            
            # 计算当前角速度（线性衰减，符号反向）
            omega = self.cfg.angular_initial * (1.0 - elapsed / total_time)
            omega = max(0.0, omega)
            
            self._publish_cmd(
                self._active_linear_speed,
                -self._omega_sign * omega,
            )
            
            # 阶段2完成判断
            if omega < self.cfg.min_angular_threshold or elapsed >= total_time:
                total_elapsed = now - self._start_time
                if self.cfg.verbose:
                    self._log.info('AVOID',
                        f'完成 #{self._total_avoidances} '
                        f'总耗时={total_elapsed:.2f}s '
                        f'{"(连续障碍)" if self._is_continuous else ""}'
                    )
                self.reset()
                self._publish_cmd()
                return False
            
            return True
        
        return False
    
    # ══════════════════════════════════════════════════
    # 恒定曲率方案实现
    # ══════════════════════════════════════════════════
    
    def _step_constant_curvature(self, current_yaw) -> bool:
        """恒定曲率避障步进（双阶段版本）
        
        Phase1: 偏离 - 固定角速度转弯至目标角度 Δθ
        Phase2: 回正过冲 - 反向转弯，略过起始航向，保留朝向中线的小角度
                 退出后交给惯导，航向控制器平滑回正，顺便带回横偏
        
        关键：使用未归一化的角度差，避免 ±180° 跳变
        """
        
        # ── 阶段1：偏离 ─────────────────────────────
        if self._state == 'phase1_away':
            target_delta = math.radians(self.cfg.constant_delta_theta_deg)
            # 不归一化，允许超过 ±180°
            yaw_delta = current_yaw - self._start_yaw
            
            self._publish_cmd(
                self._active_linear_speed,
                self._omega_sign * self.cfg.constant_angular,
            )
            
            tolerance = math.radians(self.cfg.heading_tolerance_deg)
            # Phase1 停止条件：累计转角达到目标
            if abs(yaw_delta) >= target_delta - tolerance:
                self._state = 'phase2_back'
                self._phase2_start_time = self._now_sec()
                if self.cfg.verbose:
                    now = self._now_sec()
                    self._log.info('AVOID',
                        f'Phase1完成 '
                        f'实际转角={math.degrees(yaw_delta):.1f}° '
                        f'耗时={now - self._phase1_start_time:.2f}s'
                    )
            
            return True
        
        # ── 阶段2：回正过冲 ─────────────────────────
        elif self._state == 'phase2_back':
            # 不归一化，允许超过 ±180°
            yaw_delta = current_yaw - self._start_yaw
            
            self._publish_cmd(
                self._active_linear_speed,
                -self._omega_sign * self.cfg.constant_angular,
            )
            
            # Phase2 不对称退出：略过 0°，保留朝向路径中线的小角度
            # omega_sign=+1(左绕)→Phase2右转→yaw_delta从+30°递减
            #   → 退出条件: yaw_delta < -overshoot_deg（车头朝右，指向中线）
            # omega_sign=-1(右绕)→Phase2左转→yaw_delta从-30°递增
            #   → 退出条件: yaw_delta > +overshoot_deg（车头朝左，指向中线）
            overshoot = math.radians(self.cfg.phase2_exit_overshoot_deg)
            if (self._omega_sign > 0 and yaw_delta < -overshoot) or \
               (self._omega_sign < 0 and yaw_delta > overshoot):
                now = self._now_sec()
                total_elapsed = now - self._start_time
                if self.cfg.verbose:
                    self._log.info('AVOID',
                        f'Phase2完成 #{self._total_avoidances} '
                        f'退出航向={math.degrees(yaw_delta):.1f}° '
                        f'（过冲{math.degrees(overshoot):.1f}°）'
                        f'总耗时={total_elapsed:.2f}s'
                    )
                self.reset()
                self._publish_cmd()
                return False
            
            return True
        
        return False
    
    # ══════════════════════════════════════════════════
    # 固定转角方案实现
    # ══════════════════════════════════════════════════

    def _step_fixed_timing(self, current_yaw) -> bool:
        """固定转角避障步进（目标航向保持）

        Phase1: 设定目标航向 = 起始航向 + turn_angle，保持 duration 时间
        Phase2: 设定目标航向 = Phase1目标 - turn_angle，保持 duration 时间
        使用比例控制器跟踪目标航向

        Args:
            current_yaw: 当前航向（rad）

        Returns:
            bool: True=避障中，False=完成
        """
        now = self._now_sec()

        # ── 阶段1：偏离 ─────────────────────────────
        if self._state == 'phase1_away':
            elapsed = now - self._phase1_start_time
            duration = self.cfg.phase1_duration_s
            
            # 首次进入：设定目标航向
            if self._target_yaw is None:
                phase1_offset = self._omega_sign * math.radians(self.cfg.phase1_turn_angle_deg)
                self._target_yaw = self._normalize_angle(self._start_yaw + phase1_offset)
                if self.cfg.verbose:
                    self._log.info('AVOID',
                        f'Phase1启动 目标航向偏移={self.cfg.phase1_turn_angle_deg:.0f}° '
                        f'起始yaw={math.degrees(self._start_yaw):.1f}° '
                        f'目标yaw={math.degrees(self._target_yaw):.1f}°'
                    )
            
            # 计算航向误差与控制输出
            heading_error = self._normalize_angle(self._target_yaw - current_yaw)
            omega = self.cfg.heading_control_kp * heading_error
            
            self._publish_cmd(
                self._active_linear_speed,
                omega,
            )

            if elapsed >= duration:
                # 记录 Phase1 结束时的航向
                self._phase1_end_yaw = current_yaw
                
                if self.cfg.verbose:
                    self._log.info('AVOID',
                        f'Phase1完成 耗时={elapsed:.2f}s '
                        f'目标偏移={self.cfg.phase1_turn_angle_deg:.0f}° '
                        f'实际yaw={math.degrees(current_yaw):.1f}°'
                    )
                
                # ★ Phase1 结束停车检查
                if self.cfg.stage_pause_enabled and self.cfg.stage_pause_on_phase1_end:
                    self._state = 'pause_phase1_end'
                    self._pause_state = 'phase1_end'
                    self._pause_start_time = now
                    
                    if self.cfg.verbose:
                        self._log.info('STAGE_PAUSE', '═' * 70)
                        self._log.info('STAGE_PAUSE', '🛑 Stage 1: Phase1 完成（右转绕障结束）')
                        self._log.info('STAGE_PAUSE', '═' * 70)
                        self._log.info('STAGE_PAUSE', f'Phase1 末航向: {math.degrees(current_yaw):.1f}°')
                        start_yaw_deg = math.degrees(self._start_yaw)
                        turn_deg = math.degrees(self._normalize_angle(current_yaw - self._start_yaw))
                        target_turn_deg = self._omega_sign * self.cfg.phase1_turn_angle_deg
                        self._log.info('STAGE_PAUSE', f'起始航向: {start_yaw_deg:.1f}°')
                        self._log.info('STAGE_PAUSE', f'实际转角: {turn_deg:.1f}° (目标 {target_turn_deg:.0f}°)')
                        self._log.info('STAGE_PAUSE', '═' * 70)
                        self._log.info('STAGE_PAUSE', f'⏸️  停车观察 {self.cfg.stage_pause_duration_sec:.0f} 秒...')
                        self._log.info('STAGE_PAUSE', '═' * 70)
                    
                    return True
                
                # 正常切换到 Phase2（不停车）
                self._state = 'phase2_back'
                self._phase2_start_time = now
                self._target_yaw = None

            return True

        # ── 阶段2：回正 ─────────────────────────────
        elif self._state == 'phase2_back':
            elapsed = now - self._phase2_start_time
            duration = self.cfg.phase2_duration_s
            
            # 首次进入：设定目标航向
            if self._target_yaw is None:
                # Phase2 向内切回航道：起始航向 + 反向偏移
                # 例如：右避障 Phase1=-33°，Phase2=+33°（相对起始航向 0°）
                phase2_offset = -self._omega_sign * math.radians(self.cfg.phase2_turn_angle_deg)
                self._target_yaw = self._normalize_angle(self._start_yaw + phase2_offset)
                if self.cfg.verbose:
                    self._log.info('AVOID',
                        f'Phase2启动 向内切回航道 目标偏移={-self._omega_sign * self.cfg.phase2_turn_angle_deg:.0f}° '
                        f'起始yaw={math.degrees(self._start_yaw):.1f}° '
                        f'Phase1末yaw={math.degrees(self._phase1_end_yaw):.1f}° '
                        f'Phase2目标yaw={math.degrees(self._target_yaw):.1f}°'
                    )
            
            # 计算航向误差与控制输出
            heading_error = self._normalize_angle(self._target_yaw - current_yaw)
            omega = self.cfg.heading_control_kp * heading_error

            self._publish_cmd(
                self._active_linear_speed,
                omega,
            )

            if elapsed >= duration:
                now = self._now_sec()
                total_elapsed = now - self._start_time
                
                # 记录 Phase2 结束航向
                self._phase2_end_yaw = current_yaw
                
                if self.cfg.verbose:
                    self._log.info('AVOID',
                        f'Phase2完成 #{self._total_avoidances} '
                        f'总耗时={total_elapsed:.2f}s '
                        f'最终yaw={math.degrees(current_yaw):.1f}°'
                    )
                
                # ★ Phase2 结束停车检查
                if self.cfg.stage_pause_enabled and self.cfg.stage_pause_on_phase2_end:
                    self._state = 'pause_phase2_end'
                    self._pause_state = 'phase2_end'
                    self._pause_start_time = now
                    
                    if self.cfg.verbose:
                        self._log.info('STAGE_PAUSE', '═' * 70)
                        self._log.info('STAGE_PAUSE', '🛑 Stage 2: Phase2 完成（避障回正结束）')
                        self._log.info('STAGE_PAUSE', '═' * 70)
                        self._log.info('STAGE_PAUSE', f'最终航向: {math.degrees(current_yaw):.1f}°')
                        self._log.info('STAGE_PAUSE', f'总耗时: {total_elapsed:.2f}s')
                        self._log.info('STAGE_PAUSE', f'避障次数: #{self._total_avoidances}')
                        self._log.info('STAGE_PAUSE', '═' * 70)
                        self._log.info('STAGE_PAUSE', f'⏸️  停车观察 {self.cfg.stage_pause_duration_sec:.0f} 秒...')
                        self._log.info('STAGE_PAUSE', '═' * 70)
                    
                    return True
                
                # ★ 检查是否进入 Phase3
                if self.cfg.enable_phase3:
                    self._state = 'phase3_fine_tune'
                    self._phase3_start_time = now
                    self._target_yaw = None  # 重置，Phase3 重新计算
                    
                    if self.cfg.verbose:
                        self._log.info('AVOID',
                            f'Phase2完成 yaw={math.degrees(current_yaw):.1f}° → 进入Phase3航向微调')
                    
                    return True
                
                # 正常结束避障（不停车，不启用 Phase3）
                self.reset()
                self._publish_cmd()
                return False

            return True

        # ── 阶段3：航向微调 ─────────────────────────────
        elif self._state == 'phase3_fine_tune':
            elapsed = now - self._phase3_start_time
            duration = self.cfg.phase3_duration_s
            
            # 首次进入：计算目标航向
            if self._target_yaw is None:
                # 根据 Phase2 结束航向相对起始航向的偏差，决定转向方向
                heading_error = self._normalize_angle(self._start_yaw - self._phase2_end_yaw)
                
                # 根据偏差方向决定 Phase3 转角符号
                if heading_error > 0:  # 当前偏右，需要左转
                    phase3_offset = math.radians(self.cfg.phase3_turn_angle_deg)
                else:  # 当前偏左，需要右转
                    phase3_offset = -math.radians(self.cfg.phase3_turn_angle_deg)
                
                self._target_yaw = self._normalize_angle(self._phase2_end_yaw + phase3_offset)
                
                if self.cfg.verbose:
                    self._log.info('AVOID',
                        f'Phase3启动 Phase2末yaw={math.degrees(self._phase2_end_yaw):.1f}° '
                        f'起始yaw={math.degrees(self._start_yaw):.1f}° '
                        f'目标yaw={math.degrees(self._target_yaw):.1f}°'
                    )
            
            # 计算航向误差与控制输出
            heading_error = self._normalize_angle(self._target_yaw - current_yaw)
            omega = self.cfg.heading_control_kp * heading_error
            
            # 使用避障时的速度（不改变）
            self._publish_cmd(
                self._active_linear_speed,
                omega
            )
            
            if elapsed >= duration:
                now = self._now_sec()
                total_elapsed = now - self._start_time
                
                if self.cfg.verbose:
                    self._log.info('AVOID',
                        f'Phase3完成 耗时={elapsed:.2f}s '
                        f'最终yaw={math.degrees(current_yaw):.1f}° '
                        f'总耗时={total_elapsed:.2f}s'
                    )
                
                # ★ Phase3 结束停车检查
                if self.cfg.stage_pause_enabled and self.cfg.stage_pause_on_phase3_end:
                    self._state = 'pause_phase3_end'
                    self._pause_state = 'phase3_end'
                    self._pause_start_time = now
                    
                    if self.cfg.verbose:
                        self._log.info('STAGE_PAUSE', '═' * 70)
                        self._log.info('STAGE_PAUSE', '🛑 Stage 3: Phase3 完成（航向微调结束）')
                        self._log.info('STAGE_PAUSE', '═' * 70)
                        self._log.info('STAGE_PAUSE', f'最终航向: {math.degrees(current_yaw):.1f}°')
                        self._log.info('STAGE_PAUSE', f'起始航向: {math.degrees(self._start_yaw):.1f}°')
                        heading_diff = math.degrees(self._normalize_angle(current_yaw - self._start_yaw))
                        self._log.info('STAGE_PAUSE', f'航向偏差: {heading_diff:.1f}°')
                        self._log.info('STAGE_PAUSE', f'总耗时: {total_elapsed:.2f}s')
                        self._log.info('STAGE_PAUSE', '═' * 70)
                        self._log.info('STAGE_PAUSE', f'⏸️  停车观察 {self.cfg.stage_pause_duration_sec:.0f} 秒...')
                        self._log.info('STAGE_PAUSE', '═' * 70)
                    
                    return True
                
                # Phase3 完成，结束避障
                self.reset()
                self._publish_cmd()
                return False
            
            return True

        return False

    # ══════════════════════════════════════════════════
    # 辅助函数
    # ══════════════════════════════════════════════════
    
    def _select_detour_direction(self) -> str:
        """选择绕行方向（'left' 或 'right' 或 None）
        
        规则（简单直接）：
        1. 障碍物偏向哪侧，就往另一侧绕
        2. 仅看侧向空间，哪侧空间大就往哪侧绕
        3. 以上都不行，用上次方向或默认右绕
        """
        
        # 1. 障碍物偏向 → 往反方向绕
        if abs(self.front_angle_deg) > self.cfg.front_angle_threshold_deg:
            if self.front_angle_deg > 0:  # 障碍在左 → 往右绕
                if self.right_clearance > self.cfg.side_obstacle_threshold_m:
                    return 'right'
            else:  # 障碍在右 → 往左绕
                if self.left_clearance > self.cfg.side_obstacle_threshold_m:
                    return 'left'
        
        # 2. 直接比较侧向空间，哪侧大往哪侧绕
        if self.left_clearance > self.right_clearance:
            if self.left_clearance > self.cfg.side_obstacle_threshold_m:
                return 'left'
        elif self.right_clearance > self.left_clearance:
            if self.right_clearance > self.cfg.side_obstacle_threshold_m:
                return 'right'
        
        # 3. 空间相当，保持上次方向或默认右绕
        if self._last_direction:
            return self._last_direction
        return 'right'
    
    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def _now_sec(self):
        return self._clock.now().nanoseconds / 1e9


# ────────────────────────────────────────────────
# 使用示例
# ────────────────────────────────────────────────

if __name__ == '__main__':
    import rclpy
    from rclpy.node import Node
    
    class TestNode(Node):
        def __init__(self):
            super().__init__('spiral_test')
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            
            cfg = SpiralConfig(
                linear_speed=0.15,
                angular_initial=1.30,
                total_time=2.0,
            )
            self.avoider = SpiralAvoider(
                self.cmd_pub,
                self.get_logger(),
                self.get_clock(),
                cfg
            )
            
            self.avoider.on_scan(
                front_distance=0.45,
                front_angle_deg=10.0,
                left_clearance=0.30,
                right_clearance=0.40,
            )
            
            if self.avoider.start(current_yaw=0.0):
                self.create_timer(0.05, self.control_loop)
        
        def control_loop(self):
            current_yaw = 0.0
            
            if not self.avoider.step(current_yaw):
                self.get_logger().info('避障完成')
                self.destroy_timer(self.timer)
    
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)
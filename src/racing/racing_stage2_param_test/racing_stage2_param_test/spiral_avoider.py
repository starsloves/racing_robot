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
    mode: str = 'spiral'                     # 'spiral' 或 'constant_curvature' 或 'pure_pursuit'
    
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
    
    # ── Pure Pursuit 方案参数 ────────────────────────
    pp_lookahead_distance_m: float = 0.8     # 预瞄距离（m）
    pp_kp: float = 1.5                       # Pure Pursuit 比例增益
    pp_phase2_duration_s: float = 1.0        # Phase2 持续时间（s）
    pp_phase2_speed: float = 0.19            # Phase2 速度（m/s）
    pp_ramp_duration_s: float = 0.3          # 渐变启动时间（s）
    pp_max_angular: float = 0.5              # 最大角速度限制（rad/s）
    
    # ── 方向选择 ────────────────────────────────────
    side_obstacle_threshold_m: float = 0.25  # 侧边障碍判定阈值
    front_angle_threshold_deg: float = 5.0   # 障碍物偏向判定角度（原15°太大，改5°）
    
    # ── 安全检查 ────────────────────────────────────
    timeout_sec: float = 5.0                 # 总超时（Phase1+2 约 3s，5s 足够）
    min_front_clearance_m: float = 0.20      # 紧急停车距离（m）
    
    # ── 日志 ────────────────────────────────────────
    verbose: bool = True                     # 详细日志
    
    # ── Lane Change Feedback 方案参数（新）────────────
    avoid_target_offset_m: float = 0.22          # 目标横偏（m），安全侧移量
    avoid_shift_heading_deg: float = 20.0        # SHIFT_OUT 阶段目标航向偏移（度）
    avoid_pass_margin_m: float = 0.20            # 通过障碍物后的纵向余量（m）
    avoid_merge_heading_tolerance_deg: float = 5.0   # MERGE_BACK 结束航向容差（度）
    avoid_merge_cross_tolerance_m: float = 0.05      # MERGE_BACK 结束横偏容差（m）
    avoid_heading_kp: float = 1.8                # 航向误差增益（rad/s per rad）
    avoid_cross_kp: float = 2.5                  # 横偏误差增益（rad/s per m）
    avoid_omega_limit: float = 0.45              # 角速度限幅（rad/s）
    avoid_omega_rate_limit: float = 0.8          # 角速度变化率限幅（rad/s²）
    avoid_min_phase_hold_s: float = 0.35         # 最短阶段保持时间（s），避免频繁切换
    avoid_shift_cross_threshold: float = 0.90    # SHIFT_OUT→BYPASS_HOLD 横偏达成比例（0.90=90%）
    avoid_deadzone_heading_deg: float = 1.0      # 航向误差死区（度）
    avoid_deadzone_cross_m: float = 0.01         # 横偏误差死区（m）
    
    # ── Local Path Pure Pursuit 方案参数（推荐新方案）───────
    lpp_s1: float = 0.35                     # P1 点纵向距离（m）
    lpp_y_clear: float = 0.20                # 旁路横向偏移（m）
    lpp_s_pass: float = 0.20                 # P2 点超过障碍物的纵向余量（m）
    lpp_s3_margin: float = 0.55              # P3 点在障碍物之后的纵向距离（m）
    lpp_lookahead: float = 0.30              # Pure Pursuit 预瞄距离（m）
    lpp_heading_kp: float = 1.5              # 航向控制增益
    lpp_max_omega: float = 0.40              # 最大角速度（rad/s）
    lpp_finish_heading_tol_deg: float = 5.0  # 结束航向容差（度）
    lpp_finish_cross_tol_m: float = 0.05     # 结束横偏容差（m）
    lpp_obstacle_pass_check_s: float = 0.15  # 障碍物通过判定：robot_s > obs_s + margin
    
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
                                         # lane_change_feedback: 'shift_out', 'bypass_hold', 'merge_back'
        self._start_yaw = None           # 避障开始时的航向
        self._start_time = None          # 避障开始时间
        self._phase1_start_time = None   # 阶段1开始时间
        self._phase2_start_time = None   # 阶段2开始时间
        self._phase3_start_time = None   # 阶段3开始时间
        self._omega_sign = 1.0           # 转向符号（+1左转, -1右转）
        self._target_yaw = None          # 当前目标航向（rad）
        self._phase1_end_yaw = None      # Phase1 结束时的航向（用于 Phase2 计算偏移）
        self._phase2_end_yaw = None      # Phase2 结束时的航向（用于 Phase3 计算偏移）
        self._track_direction = None     # 轨道方向（Pure Pursuit 使用）
        self._robot_pos = None           # 当前机器人位置（Pure Pursuit 使用）
        
        # 动态速度与段参数（由主控传入）
        self._active_linear_speed = self.cfg.linear_speed  # 当前段速度
        
        # 横偏记录（用于 Phase2 斜向回归计算）
        self._cross_error_at_phase1_start = 0.0  # Phase1 启动时的横偏（m）
        self._phase1_end_cross = 0.0             # Phase1 结束时的横偏（m）
        
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
        
        # 外部传入的横偏数据（由主控提供）
        self._last_cross_error = 0.0     # 最新的横偏数据（m）
        
        # ── lane_change_feedback 专用状态 ─────────────────
        self._obstacle_s = None          # 障碍物纵向里程（m）
        self._obstacle_pos = None        # 障碍物世界坐标位置 (x, y)
        self._current_progress = 0.0     # 当前段内里程（m）
        self._reference_heading = None   # 原轨道参考航向（rad）
        self._last_omega_cmd = 0.0       # 上一帧角速度指令（用于斜率限制）
        self._phase_enter_time = 0.0     # 当前阶段进入时间（用于最短保持时间）
        
        # ── local_path_pure_pursuit 专用状态 ─────────────────
        self._local_path_waypoints = []  # 局部路径点列表 [(x, y), ...]
        self._path_index = 0             # 当前追踪的路径点索引
        self._start_position = None      # 触发避障时的机器人位置 (x, y)（用于计算实时 robot_s）
    
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
        self._track_direction = None
        self._robot_pos = None
        self._active_linear_speed = self.cfg.linear_speed
        self._pause_state = None
        self._pause_start_time = None
        self._pending_detour_right = None
        self._pending_linear_speed = None
        self._cross_error_at_phase1_start = 0.0
        self._phase1_end_cross = 0.0
        self._last_cross_error = 0.0
        self._obstacle_s = None
        self._obstacle_pos = None
        self._current_progress = 0.0
        self._reference_heading = None
        self._last_omega_cmd = 0.0
        self._phase_enter_time = 0.0
        self._local_path_waypoints = []
        self._path_index = 0
        self._start_position = None
        
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
    
    def on_cross_error(self, cross_error_m):
        """更新横偏数据（由主控调用）
        
        Args:
            cross_error_m: 当前横偏（m），正值=右偏，负值=左偏
        """
        self._last_cross_error = cross_error_m
    
    def on_progress(self, current_progress_m, reference_heading_rad):
        """更新段内里程和参考航向（lane_change_feedback 需要）
        
        Args:
            current_progress_m: 当前段内里程（m）
            reference_heading_rad: 原轨道参考航向（rad）
        """
        self._current_progress = current_progress_m
        self._reference_heading = reference_heading_rad
    
    def should_trigger(self, trigger_distance=0.55) -> bool:
        """判断是否应该触发避障（由主控调用）"""
        if self._state != 'idle':
            return False
        return (math.isfinite(self.front_distance) and 
                self.front_distance < trigger_distance)
    
    def start(self, current_yaw, detour_right=None, linear_speed=None, track_direction=None, robot_pos=None):
        """启动避障（由主控调用）
        
        Args:
            current_yaw: 当前航向（rad）
            detour_right: 是否往右侧绕行（True=右绕，False=左绕，None=自动判断）
            linear_speed: 当前直行段速度（m/s），None=用配置默认值
            track_direction: 轨道方向（rad），Pure Pursuit 使用
            robot_pos: 当前机器人位置 (x, y)，Pure Pursuit 使用
        
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
        # 根据模式选择初始状态
        if self.cfg.mode == 'lane_change_feedback':
            self._state = 'shift_out'
        elif self.cfg.mode == 'local_path_pure_pursuit':
            self._state = 'following_path'
        else:
            self._state = 'phase1_away'
        
        self._start_yaw = current_yaw
        self._start_time = self._now_sec()
        self._phase1_start_time = self._start_time
        self._phase_enter_time = self._start_time
        self._omega_sign = -1.0 if detour_right else +1.0  # 右绕=右转(ω负), 左绕=左转(ω正)
        self._total_avoidances += 1
        self._last_direction = 'right' if detour_right else 'left'
        
        # 记录段速度
        self._active_linear_speed = linear_speed if linear_speed is not None else self.cfg.linear_speed
        
        # ★ 记录 Phase1 启动时的横偏（用于 Phase2 斜向回归计算）
        self._cross_error_at_phase1_start = self._last_cross_error
        
        # 保存 Pure Pursuit 需要的参数
        self._track_direction = track_direction
        self._robot_pos = robot_pos
        
        # ★ local_path_pure_pursuit 专用：记录起点位置（用于计算实时 robot_s）
        if self.cfg.mode == 'local_path_pure_pursuit':
            self._start_position = robot_pos if robot_pos is not None else (0.0, 0.0)
        
        # ★ lane_change_feedback 专用：记录障碍物位置
        if self.cfg.mode == 'lane_change_feedback':
            # 计算障碍物世界坐标位置
            if self._robot_pos is not None and math.isfinite(self.front_distance):
                front_angle_rad = math.radians(self.front_angle_deg)
                obstacle_distance_along_track = self.front_distance * math.cos(front_angle_rad)
                self._obstacle_s = self._current_progress + obstacle_distance_along_track
                
                # 障碍物在车体坐标系中的位置
                obs_local_x = self.front_distance * math.cos(front_angle_rad)
                obs_local_y = self.front_distance * math.sin(front_angle_rad)
                
                # 转换到世界坐标系
                cos_yaw = math.cos(current_yaw)
                sin_yaw = math.sin(current_yaw)
                obs_world_x = self._robot_pos[0] + obs_local_x * cos_yaw - obs_local_y * sin_yaw
                obs_world_y = self._robot_pos[1] + obs_local_x * sin_yaw + obs_local_y * cos_yaw
                self._obstacle_pos = (obs_world_x, obs_world_y)
            else:
                self._obstacle_s = None
                self._obstacle_pos = None
            
            self._last_omega_cmd = 0.0
        
        # ★ local_path_pure_pursuit 专用：构造局部路径
        if self.cfg.mode == 'local_path_pure_pursuit':
            self._build_local_path()
        
        if self.cfg.verbose:
            mode_text = {
                'spiral': '螺旋', 
                'constant_curvature': '恒定曲率', 
                'fixed_timing': '固定转角', 
                'pure_pursuit': 'Pure Pursuit',
                'lane_change_feedback': '换道反馈',
                'local_path_pure_pursuit': '局部路径纯追踪'
            }.get(self.cfg.mode, self.cfg.mode)
            
            log_msg = (
                f'启动 #{self._total_avoidances} '
                f'模式={mode_text} '
                f'往{"右" if detour_right else "左"}侧绕行 '
                f'v={self._active_linear_speed:.2f}m/s'
            )
            
            if self.cfg.mode == 'lane_change_feedback':
                obs_info = ''
                if self._obstacle_pos:
                    obs_info = (
                        f' obs_world=({self._obstacle_pos[0]:.2f},{self._obstacle_pos[1]:.2f}) '
                        f'robot=({self._robot_pos[0]:.2f},{self._robot_pos[1]:.2f})'
                    )
                log_msg += (
                    f' front_dist={self.front_distance:.2f}m '
                    f'front_ang={self.front_angle_deg:.1f}° '
                    f'cross={self._last_cross_error*100:.1f}cm'
                    + obs_info
                )
            
            self._log.info('AVOID', log_msg)
        
        return True
    
    def step(self, current_yaw, robot_pos=None) -> bool:
        """避障主循环（由主控在每个控制周期调用）
        
        Args:
            current_yaw: 当前航向（rad）
            robot_pos: 当前机器人位置 (x, y)，Pure Pursuit 使用
        
        Returns:
            bool: True=避障中（继续调用），False=避障完成（恢复惯导）
        """
        if self._state == 'idle':
            return False
        
        # 更新机器人位置（Pure Pursuit 需要）
        if robot_pos is not None:
            self._robot_pos = robot_pos
        
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
        elif self.cfg.mode == 'lane_change_feedback':
            return self._step_lane_change_feedback(current_yaw)
        elif self.cfg.mode == 'constant_curvature':
            return self._step_constant_curvature(current_yaw)
        elif self.cfg.mode == 'pure_pursuit':
            return self._step_pure_pursuit(current_yaw)
        elif self.cfg.mode == 'local_path_pure_pursuit':
            return self._step_local_path_pure_pursuit(current_yaw)
        else:
            self._log.warn('AVOID', f'未知模式: {self.cfg.mode}')
            self.reset()
            return False
    
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
                        f'Phase1启动 目标偏移={self.cfg.phase1_turn_angle_deg:.0f}° '
                        f'起始yaw={math.degrees(self._start_yaw):.1f}° '
                        f'目标yaw={math.degrees(self._target_yaw):.1f}°'
                    )
            
            # 比例控制跟踪目标航向
            heading_error = self._normalize_angle(self._target_yaw - current_yaw)
            omega = self.cfg.heading_control_kp * heading_error
            omega = self._clamp(omega, 0.8)  # 限幅 0.8 rad/s
            
            # 使用配置的避障速度（而非段速度）
            self._publish_cmd(
                self.cfg.linear_speed,
                omega,
            )

            if elapsed >= duration:
                # 记录 Phase1 结束时的航向和横偏
                self._phase1_end_yaw = current_yaw
                self._phase1_end_cross = self._last_cross_error
                
                if self.cfg.verbose:
                    actual_turn = math.degrees(self._normalize_angle(current_yaw - self._start_yaw))
                    self._log.info('AVOID',
                        f'Phase1完成 耗时={elapsed:.2f}s '
                        f'目标转角={self._omega_sign * self.cfg.phase1_turn_angle_deg:.0f}° '
                        f'实际转角={actual_turn:.1f}° '
                        f'横偏={self._phase1_end_cross*100:.1f}cm'
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

        # ── 阶段2：对称回正 + 过冲修正 ─────────────────────
        elif self._state == 'phase2_back':
            elapsed = now - self._phase2_start_time
            duration = self.cfg.phase2_duration_s
            
            # 首次进入：计算回正目标航向
            if self._target_yaw is None:
                # ★ Phase2 过冲回正策略：
                # 
                # 目标：回到起始航向 + 固定过冲角度（用于消除横偏）
                # 
                # 原理：车从 Phase1 末航向（如 -30°）转到目标航向需要时间
                # 如果目标 = 起始航向 + 小角度，等转到目标时 Phase2 快结束了
                # 没有足够时间横向移动
                # 
                # 方案：使用固定过冲角度（如 25°），确保有足够侧向移动
                # 即使车最后航向偏右，惯导接手后会自然修正
                
                # 固定过冲角度（相对于起始航向，与绕行方向相反）
                # 右绕(omega_sign=-1) → Phase2 目标 = 起始航向 + 35°
                # 左绕(omega_sign=+1) → Phase2 目标 = 起始航向 - 35°
                # 对称于 Phase1 转角
                overshoot_angle_deg = 35.0
                overshoot_angle_rad = -self._omega_sign * math.radians(overshoot_angle_deg)
                self._target_yaw = self._normalize_angle(self._start_yaw + overshoot_angle_rad)
                
                if self.cfg.verbose:
                    current_heading_offset = math.degrees(self._normalize_angle(self._phase1_end_yaw - self._start_yaw))
                    target_offset = math.degrees(self._normalize_angle(self._target_yaw - self._start_yaw))
                    self._log.info('AVOID',
                        f'Phase2启动 过冲回正 '
                        f'Phase1末航向偏移={current_heading_offset:.1f}° '
                        f'Phase1末横偏={self._phase1_end_cross*100:.1f}cm '
                        f'过冲角度={overshoot_angle_deg:.0f}° '
                        f'起始yaw={math.degrees(self._start_yaw):.1f}° '
                        f'目标yaw={math.degrees(self._target_yaw):.1f}° '
                        f'目标偏移={target_offset:.1f}°'
                    )
            
            # 比例控制跟踪目标航向
            heading_error = self._normalize_angle(self._target_yaw - current_yaw)
            omega = self.cfg.heading_control_kp * heading_error
            omega = self._clamp(omega, 0.8)
            
            # 使用配置的避障速度（而非段速度）
            self._publish_cmd(
                self.cfg.linear_speed,
                omega,
            )

            if elapsed >= duration:
                now = self._now_sec()
                total_elapsed = now - self._start_time
                
                # 记录 Phase2 结束航向
                self._phase2_end_yaw = current_yaw
                
                if self.cfg.verbose:
                    heading_deviation = math.degrees(self._normalize_angle(current_yaw - self._start_yaw))
                    self._log.info('AVOID',
                        f'Phase2完成 #{self._total_avoidances} '
                        f'总耗时={total_elapsed:.2f}s '
                        f'最终航向相对起始={heading_deviation:.1f}°'
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
    
    # ══════════════════════════════════════════════════
    # Pure Pursuit 方案实现
    # ══════════════════════════════════════════════════
    
    def _step_pure_pursuit(self, current_yaw) -> bool:
        """Pure Pursuit 避障步进
        
        Phase1: 固定转角偏离（与 fixed_timing 相同）
        Phase2: Pure Pursuit 引导回归（目标=前方轨道方向）
        
        Args:
            current_yaw: 当前航向（rad）
        
        Returns:
            bool: True=避障中，False=完成
        """
        now = self._now_sec()
        
        # ── 阶段1：偏离（固定角速度）─────────────────────
        if self._state == 'phase1_away':
            elapsed = now - self._phase1_start_time
            duration = self.cfg.phase1_duration_s
            
            # 计算固定角速度（角度 / 时间）
            omega = self._omega_sign * math.radians(self.cfg.phase1_turn_angle_deg) / duration
            
            self._publish_cmd(self._active_linear_speed, omega)
            
            # Phase1 完成条件：时间到
            if elapsed >= duration:
                self._phase1_end_yaw = current_yaw
                self._state = 'phase2_back'
                self._phase2_start_time = now
                self._target_yaw = None  # 重置目标航向
                
                if self.cfg.verbose:
                    self._log.info('AVOID',
                        f'Phase1完成 耗时={elapsed:.2f}s '
                        f'转角={self.cfg.phase1_turn_angle_deg:.0f}° '
                        f'ω={math.degrees(omega):.1f}°/s'
                    )
            
            return True
        
        # ── 阶段2：Pure Pursuit 回归（暂时简化为固定反向转角）─────────
        elif self._state == 'phase2_back':
            elapsed = now - self._phase2_start_time
            duration = self.cfg.pp_phase2_duration_s
            
            # 简化方案：Phase2 用固定角速度回正（与 Phase1 对称）
            # 原因：当前没有可靠的横偏测量，Pure Pursuit 需要准确的轨道中线位置
            # 未来改进：集成轨道中线跟踪后再使用真正的 Pure Pursuit
            omega = -self._omega_sign * math.radians(self.cfg.phase1_turn_angle_deg) / duration
            
            self._publish_cmd(self.cfg.pp_phase2_speed, omega)
            
            # Phase2 完成条件：时间到
            if elapsed >= duration:
                total_elapsed = now - self._start_time
                heading_diff = math.degrees(self._normalize_angle(current_yaw - self._start_yaw))
                
                if self.cfg.verbose:
                    self._log.info('AVOID',
                        f'Phase2完成 #{self._total_avoidances} '
                        f'总耗时={total_elapsed:.2f}s '
                        f'航向偏差={heading_diff:.1f}° '
                        f'(相对起始航向)'
                    )
                
                self.reset()
                self._publish_cmd()
                return False
            
            return True
        
        return False

    # ══════════════════════════════════════════════════
    # Lane Change Feedback 方案实现（三阶段几何闭环）
    # ══════════════════════════════════════════════════

    def _step_lane_change_feedback(self, current_yaw) -> bool:
        """Lane Change Feedback 避障步进（三阶段几何闭环）
        
        SHIFT_OUT: 建立安全横偏 → 达到目标横偏
        BYPASS_HOLD: 保持横偏通过障碍 → 纵向超过障碍物
        MERGE_BACK: 平滑回归原轨道 → 横偏和航向回到容差内
        
        控制律：
          omega = sat(k_psi * e_psi + k_y * e_y, omega_max)
        
        Args:
            current_yaw: 当前航向（rad）
        
        Returns:
            bool: True=避障中，False=完成
        """
        now = self._now_sec()
        
        # 确保参考航向已设置
        if self._reference_heading is None:
            self._reference_heading = self._start_yaw
        
        # ── 阶段1：SHIFT_OUT（建立安全横偏）─────────────
        if self._state == 'shift_out':
            # 目标横偏（带符号）
            target_cross = self._omega_sign * self.cfg.avoid_target_offset_m
            # 目标航向偏移（帮助快速建立横偏）
            target_heading_offset = self._omega_sign * math.radians(self.cfg.avoid_shift_heading_deg)
            target_heading = self._normalize_angle(self._reference_heading + target_heading_offset)
            
            # 计算误差
            cross_error = target_cross - self._last_cross_error
            heading_error = self._normalize_angle(target_heading - current_yaw)
            
            # 统一控制律
            omega_cmd = self._compute_omega_cmd(heading_error, cross_error)
            
            # ★ 调试日志（前3次）
            if self.cfg.verbose and (now - self._phase_enter_time) < 0.5:
                self._log.info('SHIFT_DEBUG',
                    f't={now-self._phase_enter_time:.2f}s '
                    f'tgt_cross={target_cross*100:.1f}cm '
                    f'cur_cross={self._last_cross_error*100:.1f}cm '
                    f'cross_err={cross_error*100:.1f}cm '
                    f'tgt_head={math.degrees(target_heading):.1f}° '
                    f'cur_head={math.degrees(current_yaw):.1f}° '
                    f'head_err={math.degrees(heading_error):.1f}° '
                    f'omega_cmd={omega_cmd:.3f}'
                )
            
            # 发布指令
            self._publish_cmd(self._active_linear_speed, omega_cmd)
            
            # 阶段完成判断：横偏达到目标的 90%
            cross_achieved = abs(self._last_cross_error) >= abs(target_cross) * self.cfg.avoid_shift_cross_threshold
            min_time_ok = (now - self._phase_enter_time) >= self.cfg.avoid_min_phase_hold_s
            
            if cross_achieved and min_time_ok:
                if self.cfg.verbose:
                    elapsed = now - self._phase_enter_time
                    self._log.info('AVOID',
                        f'SHIFT_OUT完成 耗时={elapsed:.2f}s '
                        f'横偏={self._last_cross_error*100:.1f}cm '
                        f'(目标{target_cross*100:.0f}cm) '
                        f'航向={math.degrees(current_yaw):.1f}° '
                        f'(目标{math.degrees(target_heading):.1f}°)'
                    )
                
                self._state = 'bypass_hold'
                self._phase_enter_time = now
            
            return True
        
        # ── 阶段2：BYPASS_HOLD（保持横偏通过障碍）─────────
        elif self._state == 'bypass_hold':
            # 目标：保持横偏，航向逐渐回到参考方向
            target_cross = self._omega_sign * self.cfg.avoid_target_offset_m
            target_heading = self._reference_heading  # 航向目标=原轨道方向
            
            # 计算误差
            cross_error = target_cross - self._last_cross_error
            heading_error = self._normalize_angle(target_heading - current_yaw)
            
            # 统一控制律
            omega_cmd = self._compute_omega_cmd(heading_error, cross_error)
            
            # 发布指令
            self._publish_cmd(self._active_linear_speed, omega_cmd)
            
            # 阶段完成判断：真实距离已超过障碍物
            if self._obstacle_pos is not None and self._robot_pos is not None:
                # 计算车与障碍物的欧几里得距离
                dx = self._robot_pos[0] - self._obstacle_pos[0]
                dy = self._robot_pos[1] - self._obstacle_pos[1]
                distance_to_obstacle = math.hypot(dx, dy)
                passed_obstacle = distance_to_obstacle >= self.cfg.avoid_pass_margin_m
            else:
                # 没有障碍物位置记录，按最短保持时间退出
                passed_obstacle = False
            
            min_time_ok = (now - self._phase_enter_time) >= self.cfg.avoid_min_phase_hold_s
            
            if passed_obstacle and min_time_ok:
                if self.cfg.verbose:
                    elapsed = now - self._phase_enter_time
                    dist_to_obs = math.hypot(dx, dy) if (self._obstacle_pos and self._robot_pos) else float('nan')
                    self._log.info('AVOID',
                        f'BYPASS_HOLD完成 耗时={elapsed:.2f}s '
                        f'dist_to_obs={dist_to_obs:.2f}m '
                        f'pass_margin={self.cfg.avoid_pass_margin_m:.2f}m '
                        f'横偏={self._last_cross_error*100:.1f}cm '
                        f'航向={math.degrees(current_yaw):.1f}°'
                    )
                
                self._state = 'merge_back'
                self._phase_enter_time = now
            
            return True
        
        # ── 阶段3：MERGE_BACK（平滑回归原轨道）─────────────
        elif self._state == 'merge_back':
            # 目标：横偏→0，航向→参考方向
            target_cross = 0.0
            target_heading = self._reference_heading
            
            # 计算误差
            cross_error = target_cross - self._last_cross_error
            heading_error = self._normalize_angle(target_heading - current_yaw)
            
            # 统一控制律
            omega_cmd = self._compute_omega_cmd(heading_error, cross_error)
            
            # 发布指令
            self._publish_cmd(self._active_linear_speed, omega_cmd)
            
            # 阶段完成判断：航向和横偏都回到容差内
            heading_ok = abs(heading_error) <= math.radians(self.cfg.avoid_merge_heading_tolerance_deg)
            cross_ok = abs(self._last_cross_error) <= self.cfg.avoid_merge_cross_tolerance_m
            min_time_ok = (now - self._phase_enter_time) >= self.cfg.avoid_min_phase_hold_s
            
            if heading_ok and cross_ok and min_time_ok:
                total_elapsed = now - self._start_time
                if self.cfg.verbose:
                    self._log.info('AVOID',
                        f'MERGE_BACK完成 #{self._total_avoidances} '
                        f'总耗时={total_elapsed:.2f}s '
                        f'最终横偏={self._last_cross_error*100:.1f}cm '
                        f'最终航向偏差={math.degrees(heading_error):.1f}°'
                    )
                
                self.reset()
                self._publish_cmd()
                return False
            
            return True
        
        return False
    
    def _compute_omega_cmd(self, heading_error, cross_error):
        """统一的角速度计算（带死区和限幅）
        
        Args:
            heading_error: 航向误差（rad）
            cross_error: 横偏误差（m）
        
        Returns:
            float: 角速度指令（rad/s）
        """
        # 死区处理
        heading_deadzone = math.radians(self.cfg.avoid_deadzone_heading_deg)
        if abs(heading_error) < heading_deadzone:
            heading_error = 0.0
        
        cross_deadzone = self.cfg.avoid_deadzone_cross_m
        if abs(cross_error) < cross_deadzone:
            cross_error = 0.0
        
        # 双误差综合控制律
        # 注意：横偏误差不直接用于产生角速度，应该转化为航向修正
        # cross_error > 0 表示需要往左，但不直接乘增益，而是通过航向偏移实现
        # 这里简化为：直接用横偏误差产生侧向控制（类似 P 控制）
        omega_raw = (
            self.cfg.avoid_heading_kp * heading_error +
            self.cfg.avoid_cross_kp * cross_error
        )
        
        # 限幅
        omega_limited = SpiralAvoider._clamp(omega_raw, self.cfg.avoid_omega_limit)
        
        # 斜率限制（避免频繁反向）
        if self.cfg.avoid_omega_rate_limit > 0:
            dt = 1.0 / self.cfg.control_rate_hz
            max_delta = self.cfg.avoid_omega_rate_limit * dt
            omega_delta = omega_limited - self._last_omega_cmd
            if abs(omega_delta) > max_delta:
                omega_limited = self._last_omega_cmd + math.copysign(max_delta, omega_delta)
        
        self._last_omega_cmd = omega_limited
        return omega_limited
    
    # ══════════════════════════════════════════════════
    # Local Path Pure Pursuit 方案实现（推荐新方案）
    # ══════════════════════════════════════════════════
    
    def _build_local_path(self):
        """构造三点局部避障路径（在轨道坐标系中）
        
        轨道坐标系定义：
        - x轴：原轨道方向（起始航向）
        - y轴：垂直于轨道（左正右负）
        - 原点：触发避障时的车辆位置
        
        路径点：
        P0 = (0, 0)                          起点
        P1 = (s1, y_clear * sign)            侧移点
        P2 = (s_obs + s_pass, y_clear * sign) 旁路点
        P3 = (s_obs + s3_margin, 0)           回归点
        
        sign: 右绕 = -1, 左绕 = +1
        """
        # 计算障碍物在轨道坐标系中的纵向位置
        # front_distance * cos(angle) 是沿轨道方向的投影
        front_angle_rad = math.radians(self.front_angle_deg)
        s_obs = self.front_distance * math.cos(front_angle_rad)
        
        # 参数
        s1 = self.cfg.lpp_s1
        y_clear = self.cfg.lpp_y_clear * self._omega_sign  # 带符号
        s_pass = self.cfg.lpp_s_pass
        s3_margin = self.cfg.lpp_s3_margin
        
        # 构造路径点（轨道坐标系）
        path_track = [
            (0.0, 0.0),                       # P0 起点
            (s1, y_clear),                    # P1 侧移点
            (s_obs + s_pass, y_clear),        # P2 旁路点
            (s_obs + s3_margin, 0.0),         # P3 回归点
        ]
        
        # 转换到世界坐标系（假设车起始位置为世界系原点，起始航向为轨道方向）
        # 实际上我们用相对坐标即可，不需要真正的世界坐标
        # 保存轨道系路径，后续跟踪时实时转换
        self._local_path_waypoints = path_track
        self._path_index = 0
        
        # 记录障碍物纵向位置（用于判断通过）
        self._obstacle_s = s_obs
        
        if self.cfg.verbose:
            total_projection = s_obs + s3_margin
            self._log.info('LOCAL_PATH',
                f'构造局部路径 s_obs={s_obs:.2f}m '
                f'y_clear={y_clear*100:.0f}cm '
                f'投影距离={total_projection:.2f}m'
            )
            for i, (s, y) in enumerate(path_track):
                self._log.info('LOCAL_PATH', f'  P{i} = ({s:.2f}, {y*100:.0f}cm)')
    
    def _step_local_path_pure_pursuit(self, current_yaw) -> bool:
        """Local Path Pure Pursuit 避障步进
        
        状态：following_path
        
        控制：Pure Pursuit 跟踪局部路径
        完成判据：
        1. 车已超过 P3 点（轨道投影）
        2. 横偏 <= 容差
        3. 航向偏差 <= 容差
        
        Args:
            current_yaw: 当前航向（rad）
        
        Returns:
            bool: True=避障中，False=完成
        """
        if self._state != 'following_path':
            return False
        
        # ── 坐标转换（用轮速里程计实时位置） ──────────────────
        # ★ robot_s：沿轨道方向从触发点开始的累计里程
        # 计算当前位置相对起点的位移向量
        if self._robot_pos is None or self._start_position is None:
            self._log.warn('PATH_TRACK', '机器人位置未知，无法跟踪路径')
            return False
        
        dx = self._robot_pos[0] - self._start_position[0]
        dy = self._robot_pos[1] - self._start_position[1]
        
        # 投影到轨道方向（_start_yaw 是触发时航向）
        robot_s = dx * math.cos(self._start_yaw) + dy * math.sin(self._start_yaw)
        robot_s = max(0.0, robot_s)  # 不能为负
        
        # 横偏：垂直于轨道方向（左正右负）
        robot_y = -dx * math.sin(self._start_yaw) + dy * math.cos(self._start_yaw)
        
        # ★ 调试：首次调用时打印位置信息
        if self.cfg.verbose and (self._now_sec() - self._start_time) < 0.1:
            self._log.info('DEBUG_POS',
                f'start_pos=({self._start_position[0]:.3f},{self._start_position[1]:.3f}) '
                f'robot_pos=({self._robot_pos[0]:.3f},{self._robot_pos[1]:.3f}) '
                f'dx={dx:.3f} dy={dy:.3f} '
                f'start_yaw={math.degrees(self._start_yaw):.1f}° '
                f'→ robot_s={robot_s:.3f}m robot_y={robot_y*100:.1f}cm'
            )
        
        # 当前航向相对轨道的偏差
        heading_error = self._normalize_angle(current_yaw - self._start_yaw)
        
        # ── 完成判据检查（修正：不能只看纵向位置） ──────────
        # ★ 问题：原判据只看 robot_s >= p3_s，车可能在 y≠0 时就满足条件
        # ★ 修正：必须先通过障碍物，且横偏和航向都接近目标才算完成
        
        # 条件1：车已超过障碍物（用较小的余量，避免过早触发）
        obstacle_passed = robot_s >= (self._obstacle_s + self.cfg.lpp_obstacle_pass_check_s)
        
        # 条件2：横偏小于容差
        cross_ok = abs(robot_y) <= self.cfg.lpp_finish_cross_tol_m
        
        # 条件3：航向偏差小于容差
        heading_tol = math.radians(self.cfg.lpp_finish_heading_tol_deg)
        heading_ok = abs(heading_error) <= heading_tol
        
        # ★ 每秒打印一次完成判据检查（用于调试）
        now = self._now_sec()
        if self.cfg.verbose and (int(now * 2) != int((now - 0.03) * 2)):  # 每0.5秒
            self._log.info('FINISH_CHECK',
                f't={now-self._start_time:.1f}s '
                f'robot_s={robot_s:.2f}m robot_y={robot_y*100:.1f}cm heading_err={math.degrees(heading_error):.1f}° | '
                f'条件1_通过障碍={obstacle_passed}({robot_s:.2f}>={self._obstacle_s+self.cfg.lpp_obstacle_pass_check_s:.2f}) '
                f'条件2_横偏={cross_ok}({abs(robot_y)*100:.1f}<={self.cfg.lpp_finish_cross_tol_m*100:.0f}cm) '
                f'条件3_航向={heading_ok}({abs(math.degrees(heading_error)):.1f}<={self.cfg.lpp_finish_heading_tol_deg:.0f}°)'
            )
        
        # ★ 必须三个条件同时满足：通过障碍 + 横偏小 + 航向正
        if obstacle_passed and cross_ok and heading_ok:
            total_elapsed = self._now_sec() - self._start_time
            if self.cfg.verbose:
                self._log.info('AVOID',
                    f'局部路径完成 #{self._total_avoidances} '
                    f'总耗时={total_elapsed:.2f}s '
                    f'最终robot_s={robot_s:.2f}m '
                    f'横偏={robot_y*100:.1f}cm '
                    f'航向偏差={math.degrees(heading_error):.1f}°'
                )
            
            self.reset()
            self._publish_cmd()
            return False
        
        # ── 控制策略选择 ──────────────────────
        # ★ 超过P3后，切换到直接航向+横偏反馈控制，避免Pure Pursuit过冲
        p3_s = self._local_path_waypoints[-1][0]
        
        if robot_s > p3_s:
            # ═══ 回归阶段：航向+横偏双反馈 ═══
            # 目标：航向回正 + 横偏归零
            omega = -2.0 * heading_error - 3.0 * robot_y
            omega = self._clamp(omega, self.cfg.lpp_max_omega)
            
            self._publish_cmd(self._active_linear_speed, omega)
            
            if self.cfg.verbose and (self._now_sec() - self._phase_enter_time) < 0.5:
                self._log.info('ALIGN_TRACK',
                    f't={self._now_sec()-self._start_time:.2f}s '
                    f'robot_s={robot_s:.2f}m y={robot_y*100:.0f}cm heading={math.degrees(heading_error):.1f}° '
                    f'omega={omega:.3f} [双反馈回归]'
                )
        else:
            # ═══ 绕障阶段：Pure Pursuit 路径跟踪 ═══
            # 在路径上找预瞄点
            lookahead = self.cfg.lpp_lookahead
            target_s = robot_s + lookahead
            
            # 在 waypoints 中找到预瞄点（简化：线性插值）
            target_track_x, target_track_y = self._interpolate_path(target_s)
            
            # 计算车到预瞄点的向量（轨道坐标系）
            dx = target_track_x - robot_s
            dy = target_track_y - robot_y
            
            # 预瞄点相对车头的角度
            # 车头方向 = current_yaw
            # 轨道方向 = _start_yaw
            # 预瞄向量在轨道系中的角度 = atan2(dy, dx)
            # 转到车头系：需要减去 (current_yaw - _start_yaw)
            target_angle_track = math.atan2(dy, dx)
            alpha = self._normalize_angle(target_angle_track - heading_error)
            
            # Pure Pursuit 控制律
            # omega = (2 * v * sin(alpha)) / lookahead
            # 简化为比例控制
            omega = self.cfg.lpp_heading_kp * alpha
            omega = self._clamp(omega, self.cfg.lpp_max_omega)
            
            # 发布控制指令
            self._publish_cmd(self._active_linear_speed, omega)
            
            # ── 调试日志（每秒1次） ──────────────────────
            now = self._now_sec()
            if self.cfg.verbose and (now - self._phase_enter_time) < 0.5:
                self._log.info('PATH_TRACK',
                    f't={now-self._start_time:.2f}s '
                    f'robot_s={robot_s:.2f}m y={robot_y*100:.0f}cm '
                    f'target=({target_track_x:.2f},{target_track_y*100:.0f}cm) '
                    f'alpha={math.degrees(alpha):.1f}° '
                    f'omega={omega:.3f}'
                )
        
        return True
    
    def _interpolate_path(self, target_s):
        """在局部路径上插值找到纵向位置为 target_s 的点
        
        Args:
            target_s: 目标纵向位置（轨道坐标系）
        
        Returns:
            (x, y): 插值后的点坐标（轨道坐标系）
        """
        waypoints = self._local_path_waypoints
        
        # ★ 如果超过最后一个点（P3），沿原轨道方向延伸（y=0）
        # 避免车掉头追踪后方的P3点
        if target_s >= waypoints[-1][0]:
            return (target_s, 0.0)  # 延伸到目标纵向位置，横偏保持0
        
        # 如果小于第一个点，返回第一个点
        if target_s <= waypoints[0][0]:
            return waypoints[0]
        
        # 在路径点中找到包含 target_s 的段
        for i in range(len(waypoints) - 1):
            s0, y0 = waypoints[i]
            s1, y1 = waypoints[i + 1]
            
            if s0 <= target_s <= s1:
                # 线性插值
                if abs(s1 - s0) < 1e-6:
                    return (s0, y0)
                
                ratio = (target_s - s0) / (s1 - s0)
                y_interp = y0 + ratio * (y1 - y0)
                return (target_s, y_interp)
        
        # 理论上不会到这里
        return waypoints[-1]


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
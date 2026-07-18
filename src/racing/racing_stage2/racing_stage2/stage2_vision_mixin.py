#!/usr/bin/env python3
"""
direct_inertial_tester_vision.py — 视觉修正 mixin

为 Stage2InertialNavigator 提供视觉车道居中功能：
1. 初始化视觉模块（VisionLaneCentering）
2. 提供视觉修正逻辑（move 段横向修正）
3. 视觉数据通过 ROS Image 话题 /vision_debug 发布（供 RViz2 显示）

要求父类提供：
    - self.get_parameter(name)
    - self._log_session(category, message)
    - self.clamp(value, limit)
    - self.angle_error(target, current)
    - self.current_position, self.segment_heading
    - self.navigation_yaw()
    - self.heading_kp, self.max_angular_speed
"""

import math
import os
import time


class Stage2VisionMixin:
    """
    视觉修正 mixin，供 Stage2InertialNavigator 混入。
    """
    
    def _setup_vision_centering(self):
        """初始化视觉居中模块"""
        from racing_stage2.vision_lane_centering import VisionLaneCentering
        
        # 参数声明
        self.declare_parameter('vision_offset_correction_enabled', True)
        self.declare_parameter('imu_heading_correction_enabled', True)
        self.declare_parameter('fusion_mode_enabled', True)
        self.declare_parameter('vision_model_path', 
            '/home/sunrise/dev_ws/src/racing/racing_stage2/models/bset.bin')
        self.declare_parameter('vision_conf_thres', 0.25)
        self.declare_parameter('vision_iou_thres', 0.45)
        self.declare_parameter('vision_crop_ratio', 0.4)
        self.declare_parameter('vision_crop_side_ratio', 0.20)  # 左右各裁比例，保留中间 60%
        self.declare_parameter('vision_offset_kp', 0.4)
        self.declare_parameter('vision_max_angular', 0.80)
        self.declare_parameter('vision_http_port', 8082)
        # mask 质心纠偏 / 中心竖带到位
        self.declare_parameter('vision_center_band', 0.18)
        self.declare_parameter('vision_center_occ_thresh', 0.45)
        self.declare_parameter('vision_centroid_bottom_ratio', 0.70)
        self.declare_parameter('vision_center_hold_sec', 0.20)  # 兼容旧：居中持续后可降权
        # 多行中线跟随（来自 seg_line_follower）
        self.declare_parameter('vision_centerline_mode_enabled', True)
        self.declare_parameter('vision_sample_rows', 9)
        self.declare_parameter('vision_lookahead_ratio', 0.62)
        self.declare_parameter('vision_min_mask_pixels_per_row', 12)
        self.declare_parameter('vision_min_valid_rows', 4)
        self.declare_parameter('vision_mask_threshold', 0.50)
        self.declare_parameter('vision_error_filter_alpha', 0.35)

        # 纯 SEG 主控（替换 Stage2 段式惯导链路）
        self.declare_parameter('vision_pure_mode_enabled', True)
        self.declare_parameter('vision_cruise_speed', 0.34)
        self.declare_parameter('vision_corner_speed', 0.30)
        self.declare_parameter('vision_min_speed', 0.12)
        self.declare_parameter('vision_search_angular', 0.28)
        self.declare_parameter('vision_lost_timeout_sec', 0.40)
        self.declare_parameter('vision_mission_distance_scale', 1.00)
        self.declare_parameter('vision_mission_timeout_sec', 90.0)
        # 纯SEG结束：路径里程 + 回到通道口区域 (map x≈2.5)
        self.declare_parameter('vision_finish_min_path_m', 5.0)
        self.declare_parameter('vision_finish_x_m', 2.50)
        self.declare_parameter('vision_finish_x_tol_m', 0.35)
        self.declare_parameter('vision_finish_y_min_m', 1.60)
        self.declare_parameter('vision_finish_y_max_m', 3.20)
        self.declare_parameter('vision_finish_require_x_rise', True)
        self.declare_parameter('vision_curve_speed_thresh', 0.22)
        self.declare_parameter('vision_error_speed_thresh', 0.30)
        # 混合弯道：环向前馈 + 视觉修正
        self.declare_parameter('vision_turn_bias_enabled', True)
        self.declare_parameter('vision_turn_bias_angular', 0.55)      # 弯道前馈角速度
        self.declare_parameter('vision_turn_bias_enter_rem_m', 1.05)  # rem 低于此开始掺入前馈
        self.declare_parameter('vision_turn_bias_full_rem_m', 0.70)   # rem 低于此全量前馈
        self.declare_parameter('vision_turn_slowdown_err', 0.12)      # |e| 小于此开始减速转
        self.declare_parameter('vision_turn_min_omega_ratio', 0.45)   # 弯道末端最小角速度比例
        self.declare_parameter('vision_overshoot_reverse_gain', 0.65) # 过冲反向增益
        self.declare_parameter('vision_exit_accel_ramp_sec', 0.45)    # 出弯加速斜坡
        # 入口/弯道转角主控（不依赖不可靠 rem）
        self.declare_parameter('vision_entry_yaw_full_deg', 82.0)     # 入口目标转角
        self.declare_parameter('vision_entry_yaw_taper_deg', 40.0)    # 开始衰减前馈
        self.declare_parameter('vision_entry_yaw_exit_deg', 70.0)     # 允许切 EXIT_ALIGN
        self.declare_parameter('vision_corner_yaw_full_deg', 78.0)    # 普通弯目标转角
        self.declare_parameter('vision_corner_yaw_taper_deg', 45.0)   # 普通弯衰减
        self.declare_parameter('vision_entry_max_angular', 0.80)      # 入口角速度上限
        self.declare_parameter('vision_entry_min_speed', 0.08)        # 入口最低速

        # Pure Pursuit 路径跟踪控制（基于引导线的几何跟踪）
        self.declare_parameter('vision_use_pure_pursuit', True)
        self.declare_parameter('vision_lookahead_distance_m', 0.35)
        self.declare_parameter('vision_wheelbase_m', 0.15)
        self.declare_parameter('vision_pursuit_kp', 1.8)

        # 旧 PD 控制参数（兼容回退）
        self.declare_parameter('vision_angular_kp', 1.25)
        self.declare_parameter('vision_angular_kd', 0.20)
        self.declare_parameter('vision_curvature_kp', 0.45)
        self.declare_parameter('vision_deadband', 0.035)

        self.declare_parameter('vision_primary_control', True)
        self.declare_parameter('vision_budget_disable_enabled', False)
        self.declare_parameter('vision_min_confidence', 0.28)
        self.declare_parameter('vision_primary_max_head_err_deg', 45.0)
        
        # 融合策略参数
        self.declare_parameter('imu_heading_deadzone_deg', 1.0)
        self.declare_parameter('imu_max_yaw_rate_deg_s', 600.0)
        self.declare_parameter('vision_timeout_sec', 0.5)
        # 旧预算参数保留；默认不启用永久关闭
        self.declare_parameter('vision_offset_max_sec', 3.0)
        self.declare_parameter('fusion_weight_imu', 0.25)
        self.declare_parameter('fusion_weight_vision', 0.75)

        # 视觉纵向定长/修正参数（odom 主，视觉辅助）
        self.declare_parameter('vision_length_correction_enabled', True)
        self.declare_parameter('vision_length_min_progress_ratio', 0.55)
        self.declare_parameter('vision_length_min_progress_m', 0.18)
        self.declare_parameter('vision_length_stop_remaining_m', 0.22)
        self.declare_parameter('vision_length_confirm_frames', 4)
        self.declare_parameter('vision_length_max_shorten_ratio', 0.25)
        self.declare_parameter('vision_length_max_extend_ratio', 0.12)
        self.declare_parameter('vision_length_max_extend_m', 0.18)
        self.declare_parameter('vision_range_near_m', 0.15)
        self.declare_parameter('vision_range_far_m', 2.50)
        self.declare_parameter('vision_range_center_band', 0.30)
        self.declare_parameter('vision_range_occ_thresh', 0.12)

        # 拐弯视觉辅助：仅在角度接近目标时，中心竖带连续充满可提前结束
        self.declare_parameter('vision_turn_assist_enabled', True)
        self.declare_parameter('vision_turn_assist_hold_sec', 0.50)
        self.declare_parameter('vision_turn_assist_min_progress_ratio', 0.70)
        # 转弯最短硬打时间 + 最小转角（第1弯 vs 后续弯）
        self.declare_parameter('vision_first_ring_turn_min_hold_sec', 0.50)
        self.declare_parameter('vision_ring_turn_min_hold_sec', 0.30)
        self.declare_parameter('vision_first_ring_turn_min_yaw_deg', 45.0)
        self.declare_parameter('vision_ring_turn_min_yaw_deg', 32.0)
        self.declare_parameter('vision_turn_assist_angle_window_deg', 18.0)
        
        # 读取参数
        self._vision_enabled = bool(self.get_parameter('vision_offset_correction_enabled').value)
        self._vision_length_enabled = bool(
            self.get_parameter('vision_length_correction_enabled').value
        )
        self._vision_length_hit_count = 0
        self._vision_length_last_log_t = 0.0
        self._vision_length_last_target = None
        self._vision_center_hold_elapsed = 0.0
        self._vision_center_hold_last_t = None
        self._vision_center_hold_latched = False
        self._vision_turn_center_hold = 0.0
        self._vision_turn_center_last_t = None

        # IMU 健康检测状态（无论 vision 是否启用都需初始化）
        self._last_imu_yaw = None
        self._last_imu_time = None
        self._imu_healthy = True

        # 融合状态跟踪
        self._last_fusion_mode = None
        self._last_fusion_log_time = 0.0
        self._last_valid_log = None

        # 横向或纵向任一开启，都加载视觉推理模块
        # 纯 SEG 模式强制加载
        pure_mode = bool(self.get_parameter('vision_pure_mode_enabled').value) if self.has_parameter('vision_pure_mode_enabled') else False
        if pure_mode:
            self._vision_enabled = True
            self._vision_length_enabled = True

        if not (self._vision_enabled or self._vision_length_enabled):
            self._vision_node = None
            self._offset_history = []
            self._offset_filter_size = 5
            self._vision_offset_max_sec = float(self.get_parameter('vision_offset_max_sec').value)
            self._vision_corr_elapsed = 0.0
            self._vision_corr_last_t = None
            self._vision_corr_budget_exhausted = False
            self._vision_center_hold_elapsed = 0.0
            self._vision_center_hold_last_t = None
            self._vision_center_hold_latched = False
            self._vision_turn_center_hold = 0.0
            self._vision_turn_center_last_t = None
            self._vision_pure_mode_enabled = False
            self.get_logger().info(
                '[视觉] 模块已禁用（offset/length correction 均关闭）'
            )
            return
        
        model_path = str(self.get_parameter('vision_model_path').value)
        conf = float(self.get_parameter('vision_conf_thres').value)
        iou = float(self.get_parameter('vision_iou_thres').value)
        crop = float(self.get_parameter('vision_crop_ratio').value)
        crop_side = float(self.get_parameter('vision_crop_side_ratio').value)
        http_port = int(self.get_parameter('vision_http_port').value)
        
        self._vision_offset_kp = float(self.get_parameter('vision_offset_kp').value)
        self._vision_max_angular = float(self.get_parameter('vision_max_angular').value)
        
        # 滑动平均滤波器（防止过度修正）
        self._offset_history = []
        self._offset_filter_size = 5  # 取最近 5 帧平均

        # 视觉横向纠偏预算（到时视觉停，IMU 继续）
        self._vision_offset_max_sec = float(self.get_parameter('vision_offset_max_sec').value)
        self._vision_corr_elapsed = 0.0
        self._vision_corr_last_t = None
        self._vision_corr_budget_exhausted = False
        self._vision_center_hold_elapsed = 0.0
        self._vision_center_hold_last_t = None
        self._vision_center_hold_latched = False
        self._vision_turn_center_hold = 0.0
        self._vision_turn_center_last_t = None
        
        # 创建视觉节点（自动启动 HTTP 服务，保存图像到 /tmp/stage2_vision.jpg）
        self._vision_node = VisionLaneCentering(
            self, model_path, conf, iou, crop, http_port, crop_side_ratio=crop_side
        )
        if hasattr(self._vision_node, 'configure_range_estimate'):
            self._vision_node.configure_range_estimate(
                near_m=float(self.get_parameter('vision_range_near_m').value),
                far_m=float(self.get_parameter('vision_range_far_m').value),
                center_band=float(self.get_parameter('vision_range_center_band').value),
                occ_thresh=float(self.get_parameter('vision_range_occ_thresh').value),
                timeout_sec=float(self.get_parameter('vision_timeout_sec').value),
            )
        if hasattr(self._vision_node, 'configure_offset_estimate'):
            self._vision_node.configure_offset_estimate(
                center_band=float(self.get_parameter('vision_center_band').value),
                occ_thresh=float(self.get_parameter('vision_center_occ_thresh').value),
                centroid_bottom_ratio=float(self.get_parameter('vision_centroid_bottom_ratio').value),
            )
        if hasattr(self._vision_node, 'configure_centerline_follow'):
            self._vision_node.configure_centerline_follow(
                sample_rows=int(self.get_parameter('vision_sample_rows').value),
                lookahead_ratio=float(self.get_parameter('vision_lookahead_ratio').value),
                min_mask_pixels_per_row=int(self.get_parameter('vision_min_mask_pixels_per_row').value),
                min_valid_rows=int(self.get_parameter('vision_min_valid_rows').value),
                mask_threshold=float(self.get_parameter('vision_mask_threshold').value),
                offset_filter_alpha=float(self.get_parameter('vision_error_filter_alpha').value),
                enabled=bool(self.get_parameter('vision_centerline_mode_enabled').value),
            )

        self._vision_angular_kp = float(self.get_parameter('vision_angular_kp').value)
        self._vision_angular_kd = float(self.get_parameter('vision_angular_kd').value)
        self._vision_curvature_kp = float(self.get_parameter('vision_curvature_kp').value)
        self._vision_deadband = float(self.get_parameter('vision_deadband').value)
        self._vision_primary_control = bool(self.get_parameter('vision_primary_control').value)
        self._vision_budget_disable_enabled = bool(self.get_parameter('vision_budget_disable_enabled').value)
        self._vision_min_confidence = float(self.get_parameter('vision_min_confidence').value)
        self._vision_primary_max_head_err_deg = float(
            self.get_parameter('vision_primary_max_head_err_deg').value
        )
        # 防御：避免 conf/rows 门限把视觉永久打死
        self._vision_min_confidence = max(0.0, min(1.0, self._vision_min_confidence))
        self._vision_primary_max_head_err_deg = max(5.0, self._vision_primary_max_head_err_deg)
        self._vision_prev_error = 0.0
        self._vision_prev_error_t = None
        self._vision_last_detail_log_t = 0.0

        # Pure Pursuit 控制参数
        self._vision_use_pure_pursuit = bool(self.get_parameter('vision_use_pure_pursuit').value)
        self._vision_lookahead_distance_m = float(self.get_parameter('vision_lookahead_distance_m').value)
        self._vision_wheelbase_m = float(self.get_parameter('vision_wheelbase_m').value)
        self._vision_pursuit_kp = float(self.get_parameter('vision_pursuit_kp').value)

        # 纯 SEG 主控参数
        self._vision_pure_mode_enabled = bool(self.get_parameter('vision_pure_mode_enabled').value)
        self._vision_cruise_speed = float(self.get_parameter('vision_cruise_speed').value)
        self._vision_corner_speed = float(self.get_parameter('vision_corner_speed').value)
        self._vision_min_speed = float(self.get_parameter('vision_min_speed').value)
        self._vision_search_angular = float(self.get_parameter('vision_search_angular').value)
        self._vision_lost_timeout_sec = float(self.get_parameter('vision_lost_timeout_sec').value)
        self._vision_mission_distance_scale = float(self.get_parameter('vision_mission_distance_scale').value)
        self._vision_mission_timeout_sec = float(self.get_parameter('vision_mission_timeout_sec').value)
        self._vision_finish_min_path_m = float(self.get_parameter('vision_finish_min_path_m').value)
        self._vision_finish_x_m = float(self.get_parameter('vision_finish_x_m').value)
        self._vision_finish_x_tol_m = float(self.get_parameter('vision_finish_x_tol_m').value)
        self._vision_finish_y_min_m = float(self.get_parameter('vision_finish_y_min_m').value)
        self._vision_finish_y_max_m = float(self.get_parameter('vision_finish_y_max_m').value)
        self._vision_finish_require_x_rise = bool(self.get_parameter('vision_finish_require_x_rise').value)
        self._vision_curve_speed_thresh = float(self.get_parameter('vision_curve_speed_thresh').value)
        self._vision_error_speed_thresh = float(self.get_parameter('vision_error_speed_thresh').value)
        self._vision_turn_bias_enabled = bool(self.get_parameter('vision_turn_bias_enabled').value)
        self._vision_turn_bias_angular = float(self.get_parameter('vision_turn_bias_angular').value)
        self._vision_turn_bias_enter_rem_m = float(self.get_parameter('vision_turn_bias_enter_rem_m').value)
        self._vision_turn_bias_full_rem_m = float(self.get_parameter('vision_turn_bias_full_rem_m').value)
        self._vision_turn_slowdown_err = float(self.get_parameter('vision_turn_slowdown_err').value)
        self._vision_turn_min_omega_ratio = float(self.get_parameter('vision_turn_min_omega_ratio').value)
        self._vision_overshoot_reverse_gain = float(self.get_parameter('vision_overshoot_reverse_gain').value)
        self._vision_exit_accel_ramp_sec = float(self.get_parameter('vision_exit_accel_ramp_sec').value)
        self._vision_entry_yaw_full_deg = float(self.get_parameter('vision_entry_yaw_full_deg').value)
        self._vision_entry_yaw_taper_deg = float(self.get_parameter('vision_entry_yaw_taper_deg').value)
        self._vision_entry_yaw_exit_deg = float(self.get_parameter('vision_entry_yaw_exit_deg').value)
        self._vision_corner_yaw_full_deg = float(self.get_parameter('vision_corner_yaw_full_deg').value)
        self._vision_corner_yaw_taper_deg = float(self.get_parameter('vision_corner_yaw_taper_deg').value)
        self._vision_entry_max_angular = float(self.get_parameter('vision_entry_max_angular').value)
        self._vision_entry_min_speed = float(self.get_parameter('vision_entry_min_speed').value)
        self._vision_pure_last_valid_t = 0.0
        self._vision_pure_last_log_t = 0.0
        self._vision_pure_mode_name = 'PURE_SEG_IDLE'
        self._vision_last_error = 0.0
        self._vision_last_curve = 0.0
        self._vision_filt_error = 0.0
        self._vision_filt_curve = 0.0
        self._vision_filt_angular = 0.0
        self._vision_has_filt = False
        self._vision_mode_hold = 'PURE_SEG_IDLE'
        self._vision_mode_hold_t = 0.0
        self._vision_lost_flip_t = 0.0
        self._vision_search_sign = 0.0  # 0=未初始化，按 direction 设
        self._vision_had_valid = False
        self._vision_near_error = 0.0
        self._vision_heading_err = 0.0
        self._vision_exit_turn_t = 0.0
        self._vision_in_turn_phase = False
        self._vision_turn_ramp_v0 = 0.12
        self._vision_force_bias_scale = 0.0
        self._vision_entry_turn_done = False
        self._vision_align_active = False
        self._vision_front_hold = 0.0
        self._vision_front_last_t = None
        self._vision_straight_hold = 0.0
        self._vision_straight_last_t = None
        self._vision_turn_cooldown_until = 0.0
        self._vision_last_turn_exit_t = 0.0
        self._vision_soft_boundary = False
        self._vision_last_overshoot = False
        self._vision_scene = 'idle'

        # 纯视觉模式下强制：视觉主控 + 不因预算闭嘴
        if self._vision_pure_mode_enabled:
            self._vision_primary_control = True
            self._vision_budget_disable_enabled = False
            self._vision_enabled = True

        mode_txt = '纯SEG主控' if self._vision_pure_mode_enabled else '混合纠偏'
        ctrl_txt = 'PurePursuit' if self._vision_use_pure_pursuit else 'PD+曲率'
        self.get_logger().info(
            f'[视觉] {mode_txt}/{ctrl_txt} '
            f'cruise={self._vision_cruise_speed:.2f} corner={self._vision_corner_speed:.2f} '
            f'lookahead={self._vision_lookahead_distance_m:.2f}m L={self._vision_wheelbase_m:.2f}m '
            f'kp={self._vision_pursuit_kp:.2f} max_ω={self._vision_max_angular:.2f} '
            f'crop=B{float(self.get_parameter("vision_crop_ratio").value):.0%}'
            f'+S{float(self.get_parameter("vision_crop_side_ratio").value):.0%}'
        )
        self.get_logger().info(
            f'[视觉] HTTP 可视化: http://0.0.0.0:{http_port}/vision_latest.jpg'
        )
    
    def _reset_vision_offset_time_state(self) -> None:
        """新 move/turn 段开始时重置视觉横向纠偏计时。"""
        self._vision_corr_elapsed = 0.0
        self._vision_corr_last_t = None
        self._vision_corr_budget_exhausted = False
        self._vision_center_hold_elapsed = 0.0
        self._vision_center_hold_last_t = None
        self._vision_center_hold_latched = False
        self._vision_turn_center_hold = 0.0
        self._vision_turn_center_last_t = None
        if hasattr(self, '_offset_history'):
            self._offset_history = []

    def _vision_offset_time_allows(self, vision_active: bool) -> bool:
        """
        限制单段视觉横向纠偏最长持续时间。

        vision_active: 当前帧视觉有效且准备输出非零纠偏。
        返回 True 表示仍允许使用视觉横向纠偏。
        """
        max_sec = float(self.get_parameter('vision_offset_max_sec').value)
        self._vision_offset_max_sec = max(0.0, max_sec)
        now = time.time()

        if self._vision_corr_budget_exhausted:
            self._vision_corr_last_t = None
            return False

        if self._vision_offset_max_sec <= 0.0:
            # 0 或负值：不限制
            self._vision_corr_last_t = now if vision_active else None
            return True

        if not vision_active:
            self._vision_corr_last_t = None
            return True

        if self._vision_corr_last_t is None:
            self._vision_corr_last_t = now
        else:
            dt = max(0.0, now - self._vision_corr_last_t)
            # 控制周期异常大时不把整段卡顿算进纠偏时间
            if dt > 0.2:
                dt = 0.0
            self._vision_corr_elapsed += dt
            self._vision_corr_last_t = now

        if self._vision_corr_elapsed >= self._vision_offset_max_sec:
            if not self._vision_corr_budget_exhausted:
                self._vision_corr_budget_exhausted = True
                self.get_logger().info(
                    f'[视觉] 横向预算用尽 {self._vision_offset_max_sec:.2f}s，'
                    f'本段视觉ω=0，回退 IMU（elapsed={self._vision_corr_elapsed:.2f}s）'
                )
                if hasattr(self, '_log_session'):
                    self._log_session(
                        'VIS_TIME_LIMIT',
                        f'elapsed={self._vision_corr_elapsed:.2f}s '
                        f'max={self._vision_offset_max_sec:.2f}s | 视觉横向停，IMU 兜底',
                    )
            self._vision_corr_last_t = None
            return False
        return True

    def _update_center_hold(self, centered: bool) -> bool:
        """中心竖带持续充满达到 hold_sec 后锁存，停止视觉横向纠偏。"""
        hold_sec = max(0.0, float(self.get_parameter('vision_center_hold_sec').value))
        now = time.time()
        if self._vision_center_hold_latched:
            return True
        if hold_sec <= 0.0:
            if centered:
                self._vision_center_hold_latched = True
                return True
            return False
        if not centered:
            self._vision_center_hold_elapsed = 0.0
            self._vision_center_hold_last_t = None
            return False
        if self._vision_center_hold_last_t is None:
            self._vision_center_hold_last_t = now
        else:
            dt = max(0.0, now - self._vision_center_hold_last_t)
            if dt > 0.2:
                dt = 0.0
            self._vision_center_hold_elapsed += dt
            self._vision_center_hold_last_t = now
        if self._vision_center_hold_elapsed >= hold_sec:
            self._vision_center_hold_latched = True
            if hasattr(self, '_log_session'):
                self._log_session(
                    'VIS_CENTER_HOLD',
                    f'centered hold {self._vision_center_hold_elapsed:.2f}s >= {hold_sec:.2f}s | 视觉横向停',
                )
            return True
        return False

    def _get_vision_center_status(self):
        """读取中心竖带状态，失败时返回 (False, 0.0, False)。"""
        if self._vision_node is None or not hasattr(self._vision_node, 'get_latest_center_status'):
            return False, 0.0, False
        centered, center_ratio, _ts, valid = self._vision_node.get_latest_center_status()
        if not valid:
            return False, float(center_ratio or 0.0), False
        return bool(centered), float(center_ratio or 0.0), True

    def vision_turn_assist_ready(self, progress_ratio: float, abs_err_deg: float) -> bool:
        """
        拐弯视觉辅助（方案2严格版 + 入口对齐特殊处理）：

        入口对齐段特殊处理：
        - 近处黄线很多，不能只看近处
        - 要看远处绿色区域是否居中
        - 放宽对齐精度要求（只要进入通道即可）

        普通转弯段：严格检测"转弯完成+正对直道"特征
          1) 已转完名义角的 min_progress_ratio（如80%）
          2) 剩余角误差在 angle_window 内（如<10°）
          3) 视觉检测到完整直道特征：
             - 中心竖带充满（前方有赛道）
             - 左右两侧均有赛道（mask 分布均匀）
             - 前视点居中（横向偏移小）
             - 有效检测行数足够（视野清晰）
          4) 连续稳定 hold_sec 秒
        """
        if not bool(self.get_parameter('vision_turn_assist_enabled').value):
            return False
        if self._vision_node is None:
            return False

        # 基本条件：转够了+角度接近
        min_prog = float(self.get_parameter('vision_turn_assist_min_progress_ratio').value)
        ang_win = float(self.get_parameter('vision_turn_assist_angle_window_deg').value)
        hold_sec = max(0.05, float(self.get_parameter('vision_turn_assist_hold_sec').value))

        # 判断是否是入口对齐段
        is_enter_align = False
        if hasattr(self, 'current_segment') and self.current_segment:
            seg_desc = str(self.current_segment.get('description', ''))
            is_enter_align = 'enter_align' in seg_desc.lower()

        # 入口对齐段：放宽进度和角度要求
        if is_enter_align:
            min_prog = 0.50  # 只要转过一半就开始检测
            ang_win = 20.0   # 角度窗口放宽到20°

        if progress_ratio < min_prog or abs_err_deg > ang_win:
            self._vision_turn_center_hold = 0.0
            self._vision_turn_center_last_t = None
            return False

        # 获取完整视觉状态
        line_status = self._get_vision_line_status()
        if not line_status['valid']:
            self._vision_turn_center_hold = 0.0
            self._vision_turn_center_last_t = None
            return False

        # 入口对齐段：特殊判据（只看远处，放宽要求）
        if is_enter_align:
            # 判据1：远处要有检测（说明看到绿色区域了）
            valid_rows = int(line_status.get('valid_rows', 0))
            if valid_rows < 4:  # 至少4行
                self._vision_turn_center_hold = 0.0
                self._vision_turn_center_last_t = None
                return False

            # 判据2：远处中线大致居中即可（放宽到30%偏移）
            error = float(line_status.get('error', 0.0))
            if abs(error) > 0.30:  # 放宽很多
                self._vision_turn_center_hold = 0.0
                self._vision_turn_center_last_t = None
                return False

            # 判据3：置信度不要求太高（因为近处干扰大）
            confidence = float(line_status.get('confidence', 0.0))
            if confidence < 0.35:  # 降低要求
                self._vision_turn_center_hold = 0.0
                self._vision_turn_center_last_t = None
                return False

            # 入口对齐段：缩短持续时间要求（0.2秒即可）
            hold_sec = 0.20

        else:
            # 普通转弯段：严格判据（和之前一样）
            # 严格判据1：中心竖带必须充满
            centered = bool(line_status.get('centered', False))
            center_ratio = float(line_status.get('center_ratio', 0.0))
            if not centered or center_ratio < 0.60:
                self._vision_turn_center_hold = 0.0
                self._vision_turn_center_last_t = None
                return False

            # 严格判据2：前视点必须居中
            error = float(line_status.get('error', 0.0))
            if abs(error) > 0.15:
                self._vision_turn_center_hold = 0.0
                self._vision_turn_center_last_t = None
                return False

            # 严格判据3：有效行数足够
            valid_rows = int(line_status.get('valid_rows', 0))
            min_rows_for_turn = max(5, int(self.get_parameter('vision_min_valid_rows').value) if self.has_parameter('vision_min_valid_rows') else 5)
            if valid_rows < min_rows_for_turn:
                self._vision_turn_center_hold = 0.0
                self._vision_turn_center_last_t = None
                return False

            # 严格判据4：置信度足够高
            confidence = float(line_status.get('confidence', 0.0))
            if confidence < 0.50:
                self._vision_turn_center_hold = 0.0
                self._vision_turn_center_last_t = None
                return False

            # 严格判据5：边界安全
            boundary_safe = bool(line_status.get('boundary_safe', True))
            if not boundary_safe:
                self._vision_turn_center_hold = 0.0
                self._vision_turn_center_last_t = None
                return False

        # 所有条件满足，累计持续时间
        now = time.time()
        if self._vision_turn_center_last_t is None:
            self._vision_turn_center_last_t = now
        else:
            dt = max(0.0, now - self._vision_turn_center_last_t)
            if dt > 0.2:  # 防止异常长时间间隔
                dt = 0.0
            self._vision_turn_center_hold += dt
            self._vision_turn_center_last_t = now

        # 连续满足足够时长才算完成
        if self._vision_turn_center_hold >= hold_sec:
            if hasattr(self, '_log_session'):
                seg_type = '入口对齐' if is_enter_align else '普通转弯'
                self._log_session(
                    'TURN_VIS_ASSIST',
                    f'✓ {seg_type}完成 hold={self._vision_turn_center_hold:.2f}s '
                    f'prog={progress_ratio:.2f} err={abs_err_deg:.1f}° '
                    f'offset={error:+.3f} rows={valid_rows} conf={confidence:.2f}',
                )
            return True

        return False

    def _vision_offset_to_angular(self, offset: float) -> float:
        """兼容旧接口：把归一化横向误差转角速度。"""
        return self._vision_line_to_angular(error=offset, curve=0.0)

    def _get_vision_line_status(self):
        if self._vision_node is None:
            return {
                'error': 0.0, 'curve': 0.0, 'valid': False, 'confidence': 0.0,
                'remaining_m': None, 'centered': False, 'center_ratio': 0.0,
                'valid_rows': 0, 'timestamp': 0.0, 'age': 999.0,
            }
        if hasattr(self._vision_node, 'get_latest_line_status'):
            return self._vision_node.get_latest_line_status()
        offset, ts, valid = self._vision_node.get_latest_offset()
        age = time.time() - ts if ts > 0 else 999.0
        return {
            'error': float(offset) if valid else 0.0,
            'curve': 0.0,
            'valid': bool(valid),
            'confidence': 1.0 if valid else 0.0,
            'remaining_m': None,
            'centered': abs(float(offset)) < 0.08 if valid else False,
            'center_ratio': max(0.0, 1.0 - abs(float(offset))) if valid else 0.0,
            'valid_rows': 0,
            'timestamp': float(ts),
            'age': float(age),
        }

    def _vision_is_ccw(self) -> bool:
        d = str(getattr(self, 'direction', '') or '').lower()
        return ('counter' in d or 'ccw' in d or '逆' in d)

    def _vision_entry_turn_sign(self) -> float:
        """入口第一弯：顺时针左(+1)，逆时针右(-1)。"""
        return -1.0 if self._vision_is_ccw() else 1.0

    def _vision_ring_turn_sign(self) -> float:
        """入环后固定环向：顺时针右(-1)，逆时针左(+1)。"""
        return -self._vision_entry_turn_sign()

    def _vision_active_turn_sign(self) -> float:
        """当前该用的转向符号：入口未完用入口，否则用环内。"""
        if bool(getattr(self, '_vision_entry_turn_done', False)):
            return self._vision_ring_turn_sign()
        return self._vision_entry_turn_sign()

    def _vision_yaw_progress_deg(self, start_yaw, turn_sign: float) -> float:
        """相对 start_yaw 沿 turn_sign 方向已转角度 (°，>=0)。"""
        if start_yaw is None:
            return 0.0
        nav = self.navigation_yaw() if hasattr(self, 'navigation_yaw') else None
        if nav is None or not hasattr(self, 'angle_error'):
            return 0.0
        dyaw = math.degrees(self.angle_error(nav, start_yaw))
        return max(0.0, dyaw * float(turn_sign))

    def _vision_default_search_sign(self) -> float:
        """丢线搜索默认方向 = 当前阶段环向。"""
        return self._vision_active_turn_sign()

    def _vision_turn_bias_sign(self) -> float:
        """兼容旧调用：等同 active turn sign，并刷新缓存。"""
        s = self._vision_active_turn_sign()
        self._turn_direction_cache = s
        return s

    def _vision_extract_scene(self, line: dict) -> dict:
        """从 line_status 抽出场景量（前边界/直道/居中）。"""
        error = float(line.get('error', 0.0) or 0.0)
        curve = float(line.get('curve', 0.0) or 0.0)
        conf = float(line.get('confidence', 0.0) or 0.0)
        rows = int(line.get('valid_rows', 0) or 0)
        ba = bool(line.get('boundary_ahead', False))
        far_r = float(line.get('boundary_far_ratio', 0.0) or 0.0)
        mid_r = float(line.get('boundary_mid_ratio', 0.0) or 0.0)
        near_r = float(line.get('boundary_near_ratio', 0.0) or 0.0)
        bd = float(line.get('boundary_distance_ratio', 0.0) or 0.0)
        front = float(line.get('front_score', 0.0) or 0.0)
        straight = float(line.get('straight_score', 0.0) or 0.0)
        # 兼容：若视觉层还没出 score，用 ratio 估算
        if front <= 1e-6 and straight <= 1e-6:
            gap = max(0.0, near_r - far_r)
            if near_r > 0.28:
                front = min(1.0, max(0.0, 0.48 - far_r) / 0.48 * 0.4 + min(1.0, gap / 0.45) * 0.35)
            straight = min(1.0, near_r / 0.7) * 0.3 + min(1.0, mid_r / 0.65) * 0.3 + min(1.0, far_r / 0.55) * 0.3
            if ba:
                front = max(front, 0.7)
                straight *= 0.45
        # 软前边界：分数够或硬 ba
        front_trigger = ba or front >= 0.50 or (far_r < 0.32 and near_r > 0.45 and (near_r - far_r) >= 0.20)
        # 直道恢复：远端打开 + 中线还行 + 分数够
        straight_ready = (
            (not ba)
            and straight >= 0.55
            and far_r >= 0.42
            and near_r >= 0.45
            and abs(error) < 0.18
            and rows >= 5
        )
        lm = float(line.get('left_margin', 0.0) or 0.0)
        rm = float(line.get('right_margin', 0.0) or 0.0)
        coff = float(line.get('lane_center_off', 0.0) or 0.0)
        rel_l = float(line.get('rel_left', 0.0) or 0.0)
        rel_r = float(line.get('rel_right', 0.0) or 0.0)
        cfill = float(line.get('center_fill', 0.0) or 0.0)
        cfill5 = float(line.get('center_fill_5', 0.0) or 0.0)
        edge_ang = float(line.get('edge_angle_deg', 90.0) or 90.0)
        perp = float(line.get('perp_score', 0.0) or 0.0)
        apex_has = bool(line.get('apex_has_mask', True))
        apex_err = float(line.get('apex_error', 0.0) or 0.0)
        apex_fill = float(line.get('apex_fill', 0.0) or 0.0)
        apex_l5 = float(line.get('apex_left5_fill', 0.0) or 0.0)
        apex_r5 = float(line.get('apex_right5_fill', 0.0) or 0.0)
        apex_l10 = float(line.get('apex_left10_fill', 0.0) or 0.0)
        apex_r10 = float(line.get('apex_right10_fill', 0.0) or 0.0)
        apex_c10 = float(line.get('apex_center10_fill', 0.0) or 0.0)
        apex_y0 = float(line.get('apex_y0', 0.0) or 0.0)
        apex_y1 = float(line.get('apex_y1', 0.0) or 0.0)
        apex_top_y = float(line.get('apex_top_y', 0.0) or 0.0)
        apex_cx = float(line.get('apex_cx', 0.0) or 0.0)
        apex_left_x = float(line.get('apex_left_x', 0.0) or 0.0)
        apex_right_x = float(line.get('apex_right_x', 0.0) or 0.0)
        apex_sample_x = float(line.get('apex_sample_x', 0.0) or 0.0)
        apex_sample_y = float(line.get('apex_sample_y', 0.0) or 0.0)
        apex_center_src = str(line.get('apex_center_src', 'none') or 'none')
        apex_left5_x0 = float(line.get('apex_left5_x0', 0.0) or 0.0)
        apex_left5_x1 = float(line.get('apex_left5_x1', 0.0) or 0.0)
        apex_right5_x0 = float(line.get('apex_right5_x0', 0.0) or 0.0)
        apex_right5_x1 = float(line.get('apex_right5_x1', 0.0) or 0.0)
        apex_left10_x0 = float(line.get('apex_left10_x0', 0.0) or 0.0)
        apex_left10_x1 = float(line.get('apex_left10_x1', 0.0) or 0.0)
        apex_right10_x0 = float(line.get('apex_right10_x0', 0.0) or 0.0)
        apex_right10_x1 = float(line.get('apex_right10_x1', 0.0) or 0.0)
        apex_c10_x0 = float(line.get('apex_c10_x0', 0.0) or 0.0)
        apex_c10_x1 = float(line.get('apex_c10_x1', 0.0) or 0.0)
        apex_far_row = float(line.get('apex_far_row_fill', 0.0) or 0.0)
        bend = float(line.get('path_bend', curve) or 0.0)
        lane_clear = bool(line.get('lane_clear', False))
        # 用户：中心左右各 10% 竖带占满 → 居中
        if cfill >= 0.55:
            lane_clear = True
        if not lane_clear and rows >= 3:
            lane_clear = (abs(coff) < 0.18 and abs(error) < 0.22) or (abs(coff) < 0.14)
        if not lane_clear and abs(error) < 0.12 and far_r >= 0.45 and rows >= 5:
            lane_clear = True

        centered = (
            lane_clear
            or cfill >= 0.55
            or (abs(error) < 0.12 and rows >= 4)
            or (abs(coff) < 0.14)
        )
        # 前边界两类（都要求远端截断，禁止“只有水平角”就当真弯）
        # A) 弯边：ba / far 截断
        # B) 真垂直前边：角度 + far 空
        edge_perp = self._vision_is_perp_front_edge(edge_ang, far_r, near_r)
        edge_is_front = bool(
            edge_perp
            or (ba and far_r < 0.34)
            or (perp >= 0.60 and far_r < 0.34 and near_r > 0.40)
        )
        # 真转弯几何：必须以 far 截断为主；弯曲/角度只是辅助
        has_bend = abs(bend) >= 0.40 or abs(curve) >= 0.45
        turn_geometry = bool(
            (ba and far_r < 0.36)
            or edge_is_front
            or (perp >= 0.60 and far_r < 0.34 and near_r > 0.40)
            or (has_bend and far_r < 0.34 and near_r > 0.40)
            or (front >= 0.62 and far_r < 0.32 and near_r > 0.40)
        )
        can_straight = (
            (lane_clear or cfill >= 0.55 or abs(coff) < 0.16)
            and far_r >= 0.40
            and (not ba)
            and (not edge_is_front)
            and front < 0.55
            and rows >= 3
            and abs(error) < 0.28
        )
        stick = 0.0
        if coff > 0.14 or error > 0.18:
            stick = -1.0
        elif coff < -0.14 or error < -0.18:
            stick = 1.0

        return {
            'error': error,
            'curve': curve,
            'conf': conf,
            'rows': rows,
            'ba': ba,
            'far': far_r,
            'mid': mid_r,
            'near': near_r,
            'bd': bd,
            'front': front,
            'straight': straight,
            'front_trigger': bool(front_trigger or edge_is_front),
            'straight_ready': bool(straight_ready or can_straight),
            'centered': bool(centered),
            'lane_clear': bool(lane_clear),
            'can_straight': bool(can_straight),
            'left_margin': lm,
            'right_margin': rm,
            'lane_center_off': coff,
            'rel_left': rel_l,
            'rel_right': rel_r,
            'center_fill': cfill,
            'center_fill_5': cfill5,
            'apex_has_mask': bool(apex_has),
            'apex_error': float(apex_err),
            'apex_fill': float(apex_fill),
            'apex_left5_fill': float(apex_l5),
            'apex_right5_fill': float(apex_r5),
            'apex_left10_fill': float(apex_l10),
            'apex_right10_fill': float(apex_r10),
            'apex_center10_fill': float(apex_c10),
            'apex_y0': float(apex_y0),
            'apex_y1': float(apex_y1),
            'apex_top_y': float(apex_top_y),
            'apex_cx': float(apex_cx),
            'apex_left_x': float(apex_left_x),
            'apex_right_x': float(apex_right_x),
            'apex_sample_x': float(apex_sample_x),
            'apex_sample_y': float(apex_sample_y),
            'apex_center_src': str(apex_center_src),
            'apex_left5_x0': float(apex_left5_x0),
            'apex_left5_x1': float(apex_left5_x1),
            'apex_right5_x0': float(apex_right5_x0),
            'apex_right5_x1': float(apex_right5_x1),
            'apex_left10_x0': float(apex_left10_x0),
            'apex_left10_x1': float(apex_left10_x1),
            'apex_right10_x0': float(apex_right10_x0),
            'apex_right10_x1': float(apex_right10_x1),
            'apex_c10_x0': float(apex_c10_x0),
            'apex_c10_x1': float(apex_c10_x1),
            'apex_far_row_fill': float(apex_far_row),
            'path_bend': float(bend),
            'turn_geometry': bool(turn_geometry),
            'edge_angle_deg': edge_ang,
            'perp_score': perp,
            'edge_is_front': bool(edge_is_front),
            'stick': float(stick),
        }

    def _vision_heading_from_geometry(self, line: dict, scene: dict,
                                      turn_sign: float = 0.0,
                                      early_turn: float = 0.0) -> float:
        """
        标准航向 = 近场居中 + 远场拉直 + 路径弯曲。

        SEG 给的是可通行区中线：
          near_error: 脚底下偏不偏（贴边）
          far_error / error: 前方中线偏哪（提前量）
          path_bend/curve: 中线往哪拐

        符号：e>0 中线偏右 → 车应右转 → ω<0
        early_turn: 0~1，弯前提前掺入环向打角（用户：看到转角就要提前打）
        """
        e = float(line.get('error', scene.get('error', 0.0)) or 0.0)
        e_near = float(line.get('near_error', e) or e)
        e_far = float(line.get('far_error', e) or e)
        bend = float(line.get('path_bend', line.get('curve', 0.0)) or 0.0)
        curve = float(scene.get('curve', bend) or 0.0)

        # 中央直行：lane_clear / can_straight → 几乎不打方向
        lane_clear = bool(scene.get('lane_clear', line.get('lane_clear', False)))
        can_straight = bool(scene.get('can_straight', False))
        coff = float(scene.get('lane_center_off', line.get('lane_center_off', 0.0)) or 0.0)
        if (can_straight or (lane_clear and early_turn < 0.20)) and early_turn < 0.30:
            # 用户：中心±10%占满时可直行；若最远端中线没有SEG → 微微回正
            e_far = float(line.get('far_error', e) or e)
            # 注意：far=0.0 是合法值（远端全空），禁止用 `or 1.0` 把它吃掉
            _far_raw = scene.get('far', 1.0)
            far_r = float(_far_raw) if _far_raw is not None else 1.0
            e_soft = coff if abs(coff) > 1e-6 else e
            # 远端中线空/偏：用 far_error 轻微回正（不是打死）
            if far_r < 0.40 or abs(e_far) > 0.18:
                w = float(self.clamp(-0.35 * e_far, 0.10))
            elif abs(e_soft) < 0.10:
                w = 0.0
            else:
                w = float(self.clamp(-0.25 * e_soft, 0.06))
            prev = float(getattr(self, '_vision_filt_angular', 0.0) or 0.0)
            w = 0.80 * prev + 0.20 * w
            self._vision_filt_angular = w
            return float(self.clamp(w, 0.10))

        # 偏离：用中心偏移小纠，硬顶很低（直道禁止大摆）
        if early_turn < 0.35:
            e_use = coff if abs(coff) > abs(e) * 0.5 else e
            if abs(e_use) < 0.08:
                e_use = 0.0
            w = -0.40 * max(-0.45, min(0.45, e_use))
            stick = float(scene.get('stick', 0.0) or 0.0)
            if abs(stick) > 0.5:
                w += stick * 0.06
            max_w = 0.12
            if abs(e_use) > 0.35:
                max_w = 0.16
            prev = float(getattr(self, '_vision_filt_angular', 0.0) or 0.0)
            max_step = 0.035
            if w > prev + max_step:
                w = prev + max_step
            elif w < prev - max_step:
                w = prev - max_step
            w = 0.70 * prev + 0.30 * float(self.clamp(w, max_w))
            self._vision_filt_angular = w
            return float(self.clamp(w, max_w))

        # 死区
        def dead(x, d=0.04):
            return 0.0 if abs(x) < d else x

        e_n = dead(e_near, 0.05)
        e_f = dead(e_far, 0.04)
        e_m = dead(e, 0.04)
        bd = dead(bend, 0.06)

        # 直道：近场稳住 + 远场提前
        # 弯前 early_turn>0：加重远场/弯曲 + 环向前馈
        k_near = 0.45 * (1.0 - 0.4 * early_turn)
        k_far = 0.55 + 0.45 * early_turn
        k_mid = 0.35
        k_bend = 0.25 + 0.45 * early_turn

        w_vis = -(k_near * e_n + k_mid * e_m + k_far * e_f) - k_bend * bd

        # 提前转弯：轻量环向前馈（真 TURN 里再用强前馈）
        if early_turn > 0.08 and abs(turn_sign) > 0.5:
            bias_mag = min(0.22, float(getattr(self, '_vision_turn_bias_angular', 0.28)))
            w_vis += turn_sign * bias_mag * (0.20 + 0.35 * early_turn)

        # 限幅：弯前可稍大
        max_w = 0.14 + 0.16 * max(0.0, min(1.0, early_turn))
        if abs(e_f) > 0.45 or abs(e_m) > 0.50:
            max_w = max(max_w, 0.20)
        w = float(self.clamp(w_vis, max_w))

        prev = float(getattr(self, '_vision_filt_angular', 0.0) or 0.0)
        max_step = 0.05 + 0.06 * early_turn
        if w > prev + max_step:
            w = prev + max_step
        elif w < prev - max_step:
            w = prev - max_step
        alpha = 0.45 if early_turn > 0.3 else 0.30
        w = (1.0 - alpha) * prev + alpha * w
        self._vision_filt_angular = w
        return float(self.clamp(w, max_w))

    def _vision_follow_angular(self, error: float, curve: float = 0.0) -> float:
        """兼容旧接口：无 line 时退化为简单居中。"""
        line = {
            'error': float(error),
            'near_error': float(error),
            'far_error': float(error),
            'path_bend': float(curve),
            'curve': float(curve),
        }
        scene = {'error': float(error), 'curve': float(curve)}
        return self._vision_heading_from_geometry(line, scene, turn_sign=0.0, early_turn=0.0)


    def _vision_edge_to_vehicle_angle_deg(self, edge_ang_from_horizontal: float) -> float:
        """
        图像里边相对水平的角 edge_ang (0=横边, 90=竖边)
        → 边相对车头前进方向的夹角 (0=平行前进, 90=正横在车前)。

        横边(水平) ≈ 与车头夹角 90°；竖边 ≈ 0°/180°。
        用户要求：70~110° 即接近正横 → 准备转弯。
        """
        ea = abs(float(edge_ang_from_horizontal))
        ea = min(90.0, ea)
        # 边方向相对车头：90 - ea
        return 90.0 - ea

    def _vision_is_perp_front_edge(self, edge_ang_from_horizontal: float,
                                   far: float, near: float) -> bool:
        """
        真·垂直前边：边近水平 且 远端确实被截断。
        注意：free-space mask 顶部经常拟合出“假水平边”，绝不能只靠角度。
        """
        veh = self._vision_edge_to_vehicle_angle_deg(edge_ang_from_horizontal)
        # 70~110° ≈ 横在车前
        if not (70.0 <= veh <= 110.0):
            return False
        # 必须远端空 + 近场还有路，否则是直道噪声
        if far >= 0.38:
            return False
        if near < 0.30:
            return False
        return True

    def _vision_early_turn_score(self, scene: dict, line: dict, turn_sign: float) -> float:
        """
        弯前评分：必须以远端截断为主，角度只是加分，不能单独拉高到进弯。
        """
        far = float(scene.get('far', 0.0))
        near = float(scene.get('near', 0.0))
        front = float(scene.get('front', 0.0))
        ba = bool(scene.get('ba', False))
        e_far = float(line.get('far_error', line.get('error', 0.0)) or 0.0)
        bend = float(line.get('path_bend', line.get('curve', 0.0)) or 0.0)
        lane_clear = bool(scene.get('lane_clear', line.get('lane_clear', False)))
        cfill = float(scene.get('center_fill', line.get('center_fill', 0.0)) or 0.0)
        edge_ang = float(scene.get('edge_angle_deg', line.get('edge_angle_deg', 90.0)) or 90.0)
        veh_ang = self._vision_edge_to_vehicle_angle_deg(edge_ang)
        edge_is_front = bool(scene.get('edge_is_front', False)) or self._vision_is_perp_front_edge(
            edge_ang, far, near
        )

        # 远端通 → 强制当直道，early 压掉
        if far >= 0.48 and not ba and front < 0.60:
            return 0.0
        if (lane_clear or cfill >= 0.55) and far >= 0.42 and not ba and not edge_is_front:
            return 0.0

        score = 0.0
        # --- 条件1：远端截断 / ba（主证据）---
        if ba:
            score += 0.45
        if far < 0.36:
            score += (0.36 - far) / 0.36 * 0.40
        if far < 0.28 and near > 0.45:
            score += 0.18
        score += min(1.0, max(0.0, front - 0.30) / 0.50) * 0.20

        # --- 条件2：真垂直前边（已要求 far 截断）---
        if edge_is_front:
            score += 0.25 * (1.0 - abs(veh_ang - 90.0) / 25.0)
        # 角度本身不再单独 +0.55（这是转晕根因）
        score += min(1.0, abs(bend) / 0.70) * 0.06
        score += min(1.0, abs(e_far) / 0.70) * 0.06
        score = max(0.0, min(1.0, score))
        # 远端还开着就强压
        if far >= 0.45:
            score *= 0.15
        elif far >= 0.38:
            score *= 0.45
        return float(score)

    def _vision_turn_bias_scale_from_yaw(self, yaw_prog_deg: float) -> float:
        """
        转角 → 前馈比例。这是防过冲的核心：
          0~35°  全量
          35~55° 线性降到 0.45
          55~70° 线性降到 0.10
          ≥70°   0（只靠视觉回中，禁止再硬拧）
        """
        yp = max(0.0, float(yaw_prog_deg))
        if yp < 35.0:
            return 1.0
        if yp < 55.0:
            return 1.0 - 0.55 * (yp - 35.0) / 20.0
        if yp < 70.0:
            return 0.45 - 0.35 * (yp - 55.0) / 15.0
        return 0.0

    def _vision_turn_angular(self, turn_sign: float, error: float, bias_scale: float,
                             max_w: float = None, yaw_prog_deg: float = 0.0) -> float:
        """
        转弯：环向前馈(随转角衰减) + 弱视觉。
        过冲定义：e 与 turn_sign 同号（弯内侧）→ 前馈立刻为 0。
        """
        # 转角衰减优先于外部 force_bias
        yaw_scale = self._vision_turn_bias_scale_from_yaw(yaw_prog_deg)
        bias_scale = max(0.0, min(1.0, float(bias_scale))) * yaw_scale

        bias_mag = float(getattr(self, '_vision_turn_bias_angular', 0.28) or 0.28)
        # 入口/弯中都不要用太大开环：0.28 足够，再高必过冲
        bias_mag = min(bias_mag, 0.26)
        bias_w = float(turn_sign) * bias_mag * bias_scale

        e = float(error)
        # 视觉只做小修正；大 |e| 时往往是弯中 mask 不可靠，反而降权
        e_use = e
        if abs(e) > 0.55:
            e_use = 0.55 * (1.0 if e > 0 else -1.0)
        vis = -0.40 * e_use

        # 过冲（弯内侧）：e 与 turn_sign 同号
        if e * turn_sign > 0.12:
            ov = min(1.0, abs(e) / 0.28)
            bias_w = 0.0
            vis = -0.55 * e_use  # 只回中
        # 欠转且视觉指向同侧：可略加强，但不超过 bias
        elif e * turn_sign < -0.25 and bias_scale > 0.3:
            vis *= 0.5  # 路径还在外侧时别和前馈叠加过大

        angular = bias_w + vis

        # 转角已大：禁止强制最小 ω，允许回中反号
        if yaw_prog_deg < 45.0 and bias_scale >= 0.40:
            # 仅前半弯锁环向，避免反号
            if (angular * turn_sign) < 0.0:
                angular = 0.75 * bias_w + 0.25 * angular

        if max_w is None:
            max_w = float(getattr(self, '_vision_max_angular', 0.55))
        # 转角越大 max_w 越低
        if yaw_prog_deg >= 65.0:
            max_w = min(max_w, 0.18)
        elif yaw_prog_deg >= 50.0:
            max_w = min(max_w, 0.28)
        else:
            max_w = min(max_w, 0.40)

        angular = float(self.clamp(angular, max_w))
        prev = float(getattr(self, '_vision_filt_angular', 0.0) or 0.0)
        max_step = 0.10 if yaw_prog_deg < 50.0 else 0.06
        if angular > prev + max_step:
            angular = prev + max_step
        elif angular < prev - max_step:
            angular = prev - max_step
        angular = 0.50 * prev + 0.50 * angular
        self._vision_filt_angular = angular
        return float(self.clamp(angular, max_w))

    def _vision_align_angular(self, error: float) -> float:
        """出弯回正：只居中，无环向前馈；比 FOLLOW 稍强但仍限幅。"""
        e = float(error)
        if abs(e) < 0.04:
            e = 0.0
        e_sat = max(-0.50, min(0.50, e))
        w = -0.70 * e_sat
        max_w = 0.18
        prev = float(getattr(self, '_vision_filt_angular', 0.0) or 0.0)
        max_step = 0.05
        if w > prev + max_step:
            w = prev + max_step
        elif w < prev - max_step:
            w = prev - max_step
        angular = 0.65 * prev + 0.35 * float(self.clamp(w, max_w))
        angular = float(self.clamp(angular, max_w))
        self._vision_filt_angular = angular
        return angular

    def _vision_search_direction(self) -> float:
        """丢线搜索：优先当前环向，长时间才允许翻向。"""
        now = time.time()
        last_valid = float(getattr(self, '_vision_pure_last_valid_t', 0.0) or 0.0)
        lost_for = (now - last_valid) if last_valid > 0.0 else 999.0
        ring = self._vision_active_turn_sign()
        if abs(float(getattr(self, '_vision_search_sign', 0.0) or 0.0)) < 1e-6:
            self._vision_search_sign = ring
        # 弯中/入口丢线：死跟环向
        mode = str(getattr(self, '_vision_pure_mode_name', '') or '')
        if mode in ('PURE_SEG_ENTRY', 'PURE_SEG_TURN', 'PURE_SEG_ALIGN') and lost_for < 1.6:
            return ring
        if lost_for > 2.2:
            lost_flip_t = float(getattr(self, '_vision_lost_flip_t', 0.0) or 0.0)
            if lost_flip_t <= 0.0 or (now - lost_flip_t) > 1.8:
                self._vision_search_sign = -float(self._vision_search_sign or ring)
                self._vision_lost_flip_t = now
            return float(self._vision_search_sign)
        return float(self._vision_search_sign or ring)

    def _hold_mode(self, candidate: str) -> str:
        """短时模式保持，防抖。入口/转弯优先级高。"""
        now = time.time()
        hold = str(getattr(self, '_vision_mode_hold', 'PURE_SEG_IDLE') or 'PURE_SEG_IDLE')
        hold_t = float(getattr(self, '_vision_mode_hold_t', 0.0) or 0.0)
        if candidate == hold:
            return hold
        rank = {
            'PURE_SEG_WAIT': 0,
            'PURE_SEG_FOLLOW': 1,
            'PURE_SEG_ALIGN': 2,
            'PURE_SEG_TURN': 3,
            'PURE_SEG_ENTRY': 4,
            'PURE_SEG_SEARCH': 2,
            'PURE_SEG_SEARCH_HARD': 2,
            'PURE_SEG_LOST': 1,
            'PURE_SEG_WEAK': 2,
        }
        # 高优先级立即切；同级/低级需短保持
        if rank.get(candidate, 0) > rank.get(hold, 0):
            self._vision_mode_hold = candidate
            self._vision_mode_hold_t = now
            return candidate
        if (now - hold_t) < 0.18 and rank.get(hold, 0) >= rank.get(candidate, 0):
            return hold
        self._vision_mode_hold = candidate
        self._vision_mode_hold_t = now
        return candidate

    def _vision_line_to_angular(self, error: float, curve: float = 0.0, line_status: dict = None) -> float:
        """兼容旧混合链路：纯SEG下走 follow。"""
        return self._vision_follow_angular(error, curve)

    def _compute_pure_vision_command(self):
        """
        纯 SEG 边界驱动状态机（修正版）：

        ENTRY:
          - 一进 S2 就按扫码入口方向转（顺左/逆右）
          - 出口：入口有效转时 + 最小转角，再看直道几何
          - 禁止：口子上假直道 / 盲等时间 提前结束入口

        FOLLOW:
          - 中线小纠偏；前边界触发 → TURN

        TURN:
          - 固定环向（顺右/逆左）；出弯看直道几何 + 最小转时
          - 过冲：转过最小角后 e 才参与判过冲

        方向永不由左右 mask 猜测。
        """
        line = self._get_vision_line_status()
        now = time.time()
        timeout = float(getattr(self, '_vision_lost_timeout_sec', 0.75))
        hard_lost_sec = max(timeout * 2.2, 1.5)
        vision_timeout = float(self.get_parameter('vision_timeout_sec').value) if self.has_parameter('vision_timeout_sec') else 0.5
        age = float(line.get('age', 999.0))
        valid = bool(line.get('valid', False)) and age < vision_timeout
        min_conf = float(getattr(self, '_vision_min_confidence', 0.22))
        min_rows = int(self.get_parameter('vision_min_valid_rows').value) if self.has_parameter('vision_min_valid_rows') else 2
        conf = float(line.get('confidence', 0.0))
        rows = int(line.get('valid_rows', 0))
        # 必须先取 error，micro_ok 会用到
        error = float(line.get('error', 0.0) or 0.0)
        curve = float(line.get('curve', 0.0) or 0.0)

        quality_ok = valid and conf >= min_conf and rows >= max(2, min_rows)
        weak_ok = valid and rows >= 2 and conf >= max(0.12, min_conf * 0.5)
        # 弯中 mask 常只剩近场几行：极弱也算“还有视野”
        micro_ok = valid and (
            (rows >= 1 and conf >= 0.10)
            or (abs(error) > 0.05 and conf >= 0.15 and age < vision_timeout)
        )
        in_turn_like = bool(getattr(self, '_vision_in_turn_phase', False)) or (
            not bool(getattr(self, '_vision_entry_turn_done', True))
        )
        any_ok = quality_ok or weak_ok or (micro_ok and in_turn_like)

        scene = self._vision_extract_scene(line)
        error = float(scene.get('error', error) or 0.0)
        curve = float(scene.get('curve', curve) or 0.0)
        cruise = float(getattr(self, '_vision_cruise_speed', 0.30))
        corner = float(getattr(self, '_vision_corner_speed', 0.14))
        vmin = float(getattr(self, '_vision_min_speed', 0.08))
        entry_max_w = float(getattr(self, '_vision_entry_max_angular', 0.40) or 0.40)
        max_w = float(getattr(self, '_vision_max_angular', 0.55))

        start_t = float(getattr(self, '_vision_pure_start_t', 0.0) or 0.0)
        t_run = (now - start_t) if start_t > 0 else 0.0
        entry_done = bool(getattr(self, '_vision_entry_turn_done', False))
        turn_sign = self._vision_active_turn_sign()
        cooldown_ok = now >= float(getattr(self, '_vision_turn_cooldown_until', 0.0) or 0.0)

        entry_progress = 0.0
        start_pose = getattr(self, '_vision_pure_start_pose', None)
        if start_pose is not None and self.current_position is not None:
            entry_progress = math.hypot(
                self.current_position[0] - start_pose[0],
                self.current_position[1] - start_pose[1],
            )

        # 入口起点航向
        if getattr(self, '_vision_entry_start_yaw', None) is None:
            y0 = self.navigation_yaw() if hasattr(self, 'navigation_yaw') else None
            if y0 is None:
                y0 = getattr(self, 'current_yaw', None)
            self._vision_entry_start_yaw = y0

        # 入口“有效转时”：只有 quality/weak 帧才累加，盲等不算
        entry_active_t = float(getattr(self, '_vision_entry_active_t', 0.0) or 0.0)
        last_act = getattr(self, '_vision_entry_active_last_t', None)
        if (not entry_done) and any_ok:
            if last_act is None:
                self._vision_entry_active_last_t = now
            else:
                dt = max(0.0, min(0.25, now - last_act))
                entry_active_t += dt
                self._vision_entry_active_t = entry_active_t
                self._vision_entry_active_last_t = now
        else:
            self._vision_entry_active_last_t = None

        # 直道积分：入口未完成时要求更严，且必须有有效帧
        if any_ok and scene['straight_ready'] and (entry_done or entry_active_t >= 1.0):
            last = getattr(self, '_vision_straight_last_t', None)
            if last is None:
                self._vision_straight_last_t = now
            else:
                dt = max(0.0, min(0.25, now - last))
                self._vision_straight_hold = float(getattr(self, '_vision_straight_hold', 0.0)) + dt
                self._vision_straight_last_t = now
        else:
            # 入口早期假直道直接清零
            if not entry_done:
                self._vision_straight_hold = 0.0
            else:
                self._vision_straight_hold = max(0.0, float(getattr(self, '_vision_straight_hold', 0.0)) - 0.05)
            self._vision_straight_last_t = None
        straight_hold = float(getattr(self, '_vision_straight_hold', 0.0))

        # 前边界积分：只有“当前帧真的像前边界”才累加；否则快速清零
        strong_front = any_ok and (
            scene['ba']
            or (scene['front'] >= 0.50 and scene['far'] < 0.38)
            or (scene['front'] >= 0.58)
            or (scene['far'] < 0.28 and scene['near'] > 0.50)
            or bool(scene.get('edge_is_front'))
        )
        if strong_front:
            last = getattr(self, '_vision_front_last_t', None)
            if last is None:
                self._vision_front_last_t = now
            else:
                dt = max(0.0, min(0.25, now - last))
                self._vision_front_hold = float(getattr(self, '_vision_front_hold', 0.0)) + dt
                self._vision_front_last_t = now
        else:
            # 直道上快速遗忘，防止出弯后 fh 残留再触发假转弯
            self._vision_front_hold = max(0.0, float(getattr(self, '_vision_front_hold', 0.0)) - 0.25)
            self._vision_front_last_t = None
        front_hold = float(getattr(self, '_vision_front_hold', 0.0))

        # 本弯有效转时
        turn_active_t = float(getattr(self, '_vision_turn_active_t', 0.0) or 0.0)
        if bool(getattr(self, '_vision_in_turn_phase', False)) and entry_done and any_ok:
            last = getattr(self, '_vision_turn_active_last_t', None)
            if last is None:
                self._vision_turn_active_last_t = now
            else:
                dt = max(0.0, min(0.25, now - last))
                turn_active_t += dt
                self._vision_turn_active_t = turn_active_t
                self._vision_turn_active_last_t = now
        elif not bool(getattr(self, '_vision_in_turn_phase', False)):
            self._vision_turn_active_t = 0.0
            self._vision_turn_active_last_t = None
            turn_active_t = 0.0
        else:
            self._vision_turn_active_last_t = None

        phase = 'init'
        candidate = 'PURE_SEG_FOLLOW'
        linear = cruise
        angular = 0.0
        force_bias = 0.0
        in_entry = False
        yaw_prog = 0.0

        # ========================= 有视觉 =========================
        if any_ok:
            self._vision_pure_last_valid_t = now
            self._vision_had_valid = True
            self._vision_last_error = error
            self._vision_last_curve = curve

            # ---------- ENTRY ----------
            if not entry_done:
                in_entry = True
                turn_sign = self._vision_entry_turn_sign()
                self._turn_direction_cache = turn_sign
                self._vision_search_sign = turn_sign
                self._vision_in_turn_phase = True
                phase = 'entry'
                candidate = 'PURE_SEG_ENTRY'

                yaw_prog = self._vision_yaw_progress_deg(
                    getattr(self, '_vision_entry_start_yaw', None), turn_sign
                )

                # 入口交接：舵机最大转向打死 + 足够线速度（至少弯速，约 0.30）
                linear = max(vmin, corner * 1.0)  # 不慢爬
                force_bias = 1.0
                hard_w = float(max(entry_max_w, max_w))
                angular = float(self.clamp(turn_sign * hard_w, hard_w))
                self._vision_filt_angular = angular

                # 入口结束：有效转时 + 最小转角（主），直道只是辅助
                # 口子上 str 很高不能单独结束
                # 入口目标 ~70°：转够角度后靠 e 居中退出，禁止开环拧到飞
                # 丢线/弱帧禁止出入口（日志里 e=-0.7 rows=0 就出了 = 转晕源头）
                min_active = 2.0
                min_yaw = 58.0
                good_yaw = 68.0
                full_yaw = 78.0   # 到此强制结束前馈，进入 ALIGN

                front_weak = scene['front'] < 0.48 and scene['far'] >= 0.35
                geo_ok = (
                    front_weak
                    and abs(error) < 0.20
                    and scene['straight'] >= 0.50
                    and (scene['far'] >= 0.40 or scene['near'] >= 0.55)
                )
                cfill_now = float(scene.get('center_fill', 0.0) or 0.0)
                entry_hold_ok = entry_active_t >= max(min_active, 0.50)
                vision_solid = quality_ok and rows >= 3 and conf >= min_conf
                exit_ok = (
                    entry_hold_ok
                    and yaw_prog >= min_yaw
                    and vision_solid
                    and abs(error) < 0.45
                    and (
                        (yaw_prog >= full_yaw and abs(error) < 0.35)
                        or (yaw_prog >= good_yaw and abs(error) < 0.22)
                        or (yaw_prog >= good_yaw and geo_ok)
                        or (yaw_prog >= good_yaw and cfill_now >= 0.55 and abs(error) < 0.18)
                    )
                )
                hard_ok = (
                    (yaw_prog >= 88.0 and vision_solid)
                    or (entry_active_t >= 7.0 and yaw_prog >= 70.0)
                )

                if exit_ok or hard_ok:
                    self._vision_entry_turn_done = True
                    self._turn_direction_cache = self._vision_ring_turn_sign()
                    self._vision_search_sign = self._vision_ring_turn_sign()
                    self._vision_in_turn_phase = False
                    self._vision_align_active = True
                    # 入口后必须回正到中心；禁止 cfill 假满立刻出 ALIGN 再假转弯
                    self._vision_align_until = now + 0.80
                    # 入口后只需短冷却防抖；2.8s 会把短边第一个环弯整段挡掉
                    self._vision_turn_cooldown_until = now + 0.60
                    self._vision_front_hold = 0.0
                    self._vision_front_last_t = None
                    self._vision_exit_turn_t = now
                    self._vision_filt_angular = 0.0  # 清前馈残留
                    phase = 'entry_done'
                    candidate = 'PURE_SEG_ALIGN'
                    force_bias = 0.0
                    linear = max(vmin, corner * 0.90)
                    angular = self._vision_align_angular(error)
                    self._log_session(
                        'PURE_SEG_ENTRY_DONE',
                        f't={t_run:.1f}s act={entry_active_t:.1f}s yaw={yaw_prog:.1f} '
                        f'prog={entry_progress:.2f} sh={straight_hold:.2f} str={scene["straight"]:.2f} '
                        f'e={error:+.3f} hard={int(hard_ok)}'
                    )

            # ---------- 入环后 ----------
            else:
                turn_sign = self._vision_ring_turn_sign()
                self._turn_direction_cache = turn_sign
                yaw_prog = 0.0
                if getattr(self, '_vision_corner_start_yaw', None) is not None:
                    yaw_prog = self._vision_yaw_progress_deg(
                        self._vision_corner_start_yaw, turn_sign
                    )

                early = self._vision_early_turn_score(scene, line, turn_sign)
                apex_has = bool(scene.get('apex_has_mask', True))
                apex_err = float(scene.get('apex_error', 0.0) or 0.0)
                cfill5 = float(scene.get('center_fill_5', 0.0) or 0.0)
                turn_geo = bool(scene.get('turn_geometry', False))
                edge_front = bool(scene.get('edge_is_front', False))
                cfill_now = float(scene.get('center_fill', 0.0) or 0.0)
                # far=0.0 合法：对面墙/截断。`x or 1.0` 会把 0.0 误判成 1.0 → 永远不 LOCK
                _far_raw = scene.get('far', 1.0)
                far_now = float(_far_raw) if _far_raw is not None else 1.0
                # 真转弯：远端必须空。禁止 far 还开着就 LOCK（日志转晕主因）
                far_closed = far_now < 0.34
                real_turn = far_closed and bool(
                    turn_geo
                    or edge_front
                    or (scene['ba'] and far_now < 0.34)
                    or (early >= 0.55 and far_now < 0.32)
                    or (strong_front and far_now < 0.30 and front_hold >= 0.12)
                    # far 已经彻底截断 + 前边界很强：即使 turn_geo 抖动也算真弯
                    or (far_now < 0.12 and (scene['ba'] or scene['front'] >= 0.70 or early >= 0.80))
                )

                def _apex_diag(scene_dict: dict) -> str:
                    """顶点窗口详细诊断：最高SEG向下10% + 几何中心窗口。"""
                    return (
                        f'APEX['
                        f'top_y={float(scene_dict.get("apex_top_y", 0.0) or 0.0):.0f} '
                        f'y={float(scene_dict.get("apex_y0", 0.0) or 0.0):.0f}:'
                        f'{float(scene_dict.get("apex_y1", 0.0) or 0.0):.0f} '
                        f'cx={float(scene_dict.get("apex_cx", 0.0) or 0.0):.0f} '
                        f'src={scene_dict.get("apex_center_src", "none")} '
                        f'L={float(scene_dict.get("apex_left_x", 0.0) or 0.0):.0f} '
                        f'R={float(scene_dict.get("apex_right_x", 0.0) or 0.0):.0f} '
                        f'sx={float(scene_dict.get("apex_sample_x", 0.0) or 0.0):.0f} '
                        f'sy={float(scene_dict.get("apex_sample_y", 0.0) or 0.0):.0f} '
                        f'L5={float(scene_dict.get("apex_left5_fill", 0.0) or 0.0):.2f}@'
                        f'{float(scene_dict.get("apex_left5_x0", 0.0) or 0.0):.0f}-'
                        f'{float(scene_dict.get("apex_left5_x1", 0.0) or 0.0):.0f} '
                        f'R5={float(scene_dict.get("apex_right5_fill", 0.0) or 0.0):.2f}@'
                        f'{float(scene_dict.get("apex_right5_x0", 0.0) or 0.0):.0f}-'
                        f'{float(scene_dict.get("apex_right5_x1", 0.0) or 0.0):.0f} '
                        f'L10={float(scene_dict.get("apex_left10_fill", 0.0) or 0.0):.2f} '
                        f'R10={float(scene_dict.get("apex_right10_fill", 0.0) or 0.0):.2f} '
                        f'C10={float(scene_dict.get("apex_center10_fill", 0.0) or 0.0):.2f}@'
                        f'{float(scene_dict.get("apex_c10_x0", 0.0) or 0.0):.0f}-'
                        f'{float(scene_dict.get("apex_c10_x1", 0.0) or 0.0):.0f} '
                        f'fill={float(scene_dict.get("apex_fill", 0.0) or 0.0):.2f} '
                        f'far_row={float(scene_dict.get("apex_far_row_fill", 0.0) or 0.0):.2f} '
                        f'has={int(bool(scene_dict.get("apex_has_mask", False)))}'
                        f']'
                    )

                def _begin_turn(reason: str, phase_name: str = 'turn_enter'):
                    self._vision_in_turn_phase = True
                    self._vision_align_active = False
                    self._vision_align_until = 0.0
                    y0 = self.navigation_yaw() if hasattr(self, 'navigation_yaw') else None
                    self._vision_corner_start_yaw = y0
                    self._vision_turn_active_t = 0.0
                    self._vision_filt_angular = float(self.clamp(turn_sign * max_w, max_w))
                    rn = int(getattr(self, "_vision_ring_turn_count", 0) or 0)
                    h_sec = float(
                        self.get_parameter('vision_first_ring_turn_min_hold_sec' if rn <= 0 else 'vision_ring_turn_min_hold_sec').value
                    )
                    # 增强日志：SEG位置+边缘信息（方案E）
                    left_ratio = float(scene.get('left_ratio', 0.0) or 0.0)
                    right_ratio = float(scene.get('right_ratio', 0.0) or 0.0)
                    left_margin = float(scene.get('left_margin', 0.0) or 0.0)
                    right_margin = float(scene.get('right_margin', 0.0) or 0.0)
                    lane_coff = float(scene.get('lane_center_off', 0.0) or 0.0)
                    near_err = float(scene.get('near_error', 0.0) or 0.0)
                    far_err = float(scene.get('far_error', 0.0) or 0.0)
                    edge_ang = float(scene.get('edge_angle_deg', 90.0) or 90.0)
                    last_turn = getattr(self, '_vision_last_turn_sign', 0.0)
                    # 过冲检测标志：上次转向与当前误差方向不一致
                    possible_overshoot = False
                    if abs(last_turn) > 0.1:
                        if (last_turn < 0 and error > 0.30) or (last_turn > 0 and error < -0.30):
                            possible_overshoot = True
                    self._log_session(
                        'PURE_SEG_TURN_BEGIN',
                        f'{reason} ring_n={rn} hold={h_sec:.2f}s sign={turn_sign:+.0f} '
                        f'e={error:+.3f} curve={curve:+.3f} rows={rows} conf={conf:.2f} | '
                        f'SEG[far={scene["far"]:.2f} mid={scene.get("mid", 0.0):.2f} near={scene.get("near", 0.0):.2f}] '
                        f'EDGE[L={left_ratio:.2f} R={right_ratio:.2f} Lm={left_margin:.2f} Rm={right_margin:.2f}] '
                        f'GEO[ba={int(scene["ba"])} front={scene["front"]:.2f} straight={scene["straight"]:.2f} '
                        f'edge_ang={edge_ang:.0f}° edge_front={int(edge_front)}] '
                        f'{_apex_diag(scene)} '
                        f'PATH[lane_off={lane_coff:+.2f} near_e={near_err:+.2f} far_e={far_err:+.2f}] '
                        f'WARN[early={early:.2f} last_turn={last_turn:+.0f} overshoot_risk={int(possible_overshoot)}]'
                    )
                    return phase_name, 'PURE_SEG_TURN', 1.0, max(vmin, corner), float(
                        self.clamp(turn_sign * max_w, max_w)
                    )

                def _is_center_full() -> bool:
                    """居中：顶点中心线±10%有SEG，且左右都有（防单侧墙面假满）。"""
                    apex_c10 = float(scene.get('apex_center10_fill', 0.0) or 0.0)
                    apex_l10 = float(scene.get('apex_left10_fill', 0.0) or 0.0)
                    apex_r10 = float(scene.get('apex_right10_fill', 0.0) or 0.0)
                    # 顶点中心±10%占比>=0.4，且左右10%都至少有一点 → 真回正
                    return (
                        apex_c10 >= 0.40
                        and apex_l10 >= 0.18
                        and apex_r10 >= 0.18
                    )

                def _finish_turn(reason: str, go_align: bool = False):
                    self._vision_in_turn_phase = False
                    self._vision_corner_start_yaw = None
                    # 完成一次环内转弯：计数+1（后续弯用更短 min_hold）
                    try:
                        self._vision_ring_turn_count = int(
                            getattr(self, '_vision_ring_turn_count', 0) or 0
                        ) + 1
                    except Exception:
                        self._vision_ring_turn_count = 1
                    # 冷却时间：基础0.2s，出弯不居中时自适应延长（方案A+E）
                    base_cooldown = 0.2
                    if abs(error) > 0.30 or yaw_prog > 70.0:
                        # 过冲或大误差：延长到0.6s
                        cooldown = 0.6
                        overshoot_flag = 'OVERSHOOT' if yaw_prog > 70.0 else 'LARGE_ERR'
                    elif abs(error) > 0.22:
                        # 中等误差：延长到0.4s
                        cooldown = 0.4
                        overshoot_flag = 'MED_ERR'
                    else:
                        cooldown = base_cooldown
                        overshoot_flag = 'NORMAL'
                    self._vision_turn_cooldown_until = now + cooldown
                    self._vision_front_hold = 0.0
                    self._vision_front_last_t = None
                    self._vision_exit_turn_t = now
                    self._vision_filt_angular = 0.0
                    # 记录转弯方向（用于诊断）
                    self._vision_last_turn_sign = turn_sign
                    # 增强日志（方案E）
                    self._log_session(
                        'PURE_SEG_TURN_COMPLETE',
                        f'ring_n={int(getattr(self, "_vision_ring_turn_count", 0))-1} reason={reason} '
                        f'yaw={yaw_prog:.1f}° time={turn_active_t:.2f}s '
                        f'exit_e={error:+.3f} exit_curve={curve:+.3f} '
                        f'exit_far={far_now:.2f} exit_near={scene.get("near", 0.0):.2f} '
                        f'exit_cfill={cfill_now:.2f} turn_sign={turn_sign:+.0f} '
                        f'cooldown={cooldown:.2f}s flag={overshoot_flag} go_align={int(go_align)}'
                    )
                    if go_align or abs(error) > 0.22:
                        self._vision_align_active = True
                        self._vision_align_until = now + 0.60
                        return 'align', 'PURE_SEG_ALIGN', 0.0, max(vmin, corner * 0.95), self._vision_align_angular(error)
                    self._vision_align_active = False
                    self._vision_align_until = 0.0
                    return 'straight', 'PURE_SEG_FOLLOW', 0.0, cruise, 0.0

                if bool(getattr(self, '_vision_align_active', False)):
                    # ALIGN：只回正；真前边界(远端空) + 冷却后才打断
                    cool_ok = now >= float(getattr(self, '_vision_turn_cooldown_until', 0.0) or 0.0)
                    hard_corner = cool_ok and real_turn and far_closed and (
                        scene['ba'] or scene['front'] >= 0.60 or edge_front
                    )
                    if hard_corner:
                        phase, candidate, force_bias, linear, angular = _begin_turn(
                            'from_align', 'turn_early'
                        )
                    else:
                        phase = 'align'
                        candidate = 'PURE_SEG_ALIGN'
                        force_bias = 0.0
                        self._vision_in_turn_phase = False
                        linear = max(vmin, min(cruise * 0.90, max(corner, 0.24)))
                        if _is_center_full():
                            self._vision_align_active = False
                            self._vision_align_until = 0.0
                            phase = 'straight'
                            candidate = 'PURE_SEG_FOLLOW'
                            linear = cruise
                            angular = 0.0
                            self._vision_filt_angular = 0.0
                        else:
                            angular = self._vision_heading_from_geometry(
                                line, scene, turn_sign=0.0, early_turn=0.0
                            )
                            if abs(error) > 0.25:
                                angular = float(self.clamp(
                                    0.45 * angular + 0.55 * self._vision_align_angular(error),
                                    0.24,
                                ))
                            align_until = float(getattr(self, '_vision_align_until', 0.0) or 0.0)
                            if now >= align_until and (
                                scene.get('lane_clear')
                                or (abs(error) < 0.16 and far_now >= 0.40)
                            ):
                                self._vision_align_active = False
                                self._vision_align_until = 0.0
                                phase = 'straight'
                                candidate = 'PURE_SEG_FOLLOW'
                                linear = cruise
                                if abs(error) < 0.12:
                                    angular = 0.0
                                    self._vision_filt_angular = 0.0

                elif bool(getattr(self, '_vision_in_turn_phase', False)):
                    # TURN：环向打死，v=0.25
                    # 用户：第1次环弯 min_hold=0.5s；后续弯 0.3s（防过大）
                    phase = 'turn'
                    candidate = 'PURE_SEG_TURN'
                    if getattr(self, '_vision_corner_start_yaw', None) is None:
                        y0 = self.navigation_yaw() if hasattr(self, 'navigation_yaw') else None
                        self._vision_corner_start_yaw = y0
                        self._vision_turn_active_t = 0.0
                        turn_active_t = 0.0
                        yaw_prog = 0.0

                    force_bias = 1.0
                    hard_w = float(max_w)
                    linear = max(vmin, corner)
                    ring_n = int(getattr(self, '_vision_ring_turn_count', 0) or 0)
                    is_first_ring_turn = (ring_n <= 0)
                    # 从配置读取第1弯 vs 后续弯参数
                    min_hold = float(
                        self.get_parameter('vision_first_ring_turn_min_hold_sec' if is_first_ring_turn else 'vision_ring_turn_min_hold_sec').value
                    )
                    min_yaw_turn = float(
                        self.get_parameter('vision_first_ring_turn_min_yaw_deg' if is_first_ring_turn else 'vision_ring_turn_min_yaw_deg').value
                    )
                    # 过冲保护：后续弯更早收（方案A：更激进）
                    overshoot_yaw = 75.0 if is_first_ring_turn else 45.0  # 后续弯45°就检查过冲
                    nearly_yaw = 55.0 if is_first_ring_turn else 38.0
                    hard_yaw = 95.0 if is_first_ring_turn else 65.0  # 后续弯65°强制结束
                    hard_time = 4.5 if is_first_ring_turn else 2.5
                    half_yaw = 50.0 if is_first_ring_turn else 28.0  # 后续弯28°开始收舵

                    overshot = yaw_prog >= overshoot_yaw and abs(error) < 0.25 and far_now >= 0.40
                    # 用户核心判据：转够后顶点±10%有SEG → 停转回正
                    # 注意：进弯瞬间对面/侧向残余SEG常使 C10 已很高（本次日志 C10=0.89@yaw=2°）
                    # 必须先满足 min_hold + min_yaw，否则会“转完没转够就直行”
                    center_full = _is_center_full()
                    min_turn_ok = (turn_active_t >= min_hold) and (yaw_prog >= min_yaw_turn)
                    # 远端重新打开一点：真出弯后前方应有通路；纯墙面填充 C10 时 far 仍接近 0
                    far_reopen = far_now >= 0.18
                    # 横向还在大偏时，即使 C10 假高也不能当“已回正”
                    heading_ok = abs(error) < 0.30
                    center_full_ready = bool(
                        center_full and min_turn_ok and far_reopen and heading_ok
                    )

                    if center_full_ready:
                        # 转够 + 顶点±10%有SEG + 远端已打开：直接退出（不需连续帧确认）
                        # 若 |e| 仍偏大，_finish_turn 内部会自动进 ALIGN
                        phase, candidate, force_bias, linear, angular = _finish_turn(
                            'apex_center_full', go_align=False
                        )
                        self._log_session(
                            'PURE_SEG_CENTER_FULL_EXIT',
                            f'yawp={yaw_prog:.1f} far={far_now:.2f} '
                            f'e={error:+.2f} act={turn_active_t:.1f} why=apex_c10_full '
                            f'ring_n={ring_n} hold={min_hold:.2f} min_yaw={min_yaw_turn:.0f} '
                            f'| {_apex_diag(scene)}'
                        )
                    elif overshot and min_turn_ok:
                        phase, candidate, force_bias, linear, angular = _finish_turn(
                            'overshoot', go_align=True
                        )
                        self._log_session(
                            'PURE_SEG_CENTER_FULL_EXIT',
                            f'yawp={yaw_prog:.1f} far={far_now:.2f} '
                            f'e={error:+.2f} act={turn_active_t:.1f} why=overshoot '
                            f'ring_n={ring_n} hold={min_hold:.2f} | {_apex_diag(scene)}'
                        )
                    else:
                        # 继续转弯：前半打死，后半收舵
                        if yaw_prog < half_yaw:
                            angular = float(self.clamp(turn_sign * hard_w, hard_w))
                        else:
                            base = turn_sign * min(hard_w, 0.50 if is_first_ring_turn else 0.40)
                            vis = float(self.clamp(-0.35 * error, 0.18))
                            angular = float(self.clamp(base + vis, hard_w))
                            if angular * turn_sign < 0.0:
                                angular = turn_sign * 0.18
                        self._vision_filt_angular = angular

                        hard_done = yaw_prog >= hard_yaw or turn_active_t >= hard_time

                        # 周期性诊断日志（每0.3s输出一次转弯状态，方案E）
                        last_turn_log_t = getattr(self, '_vision_last_turn_log_t', 0.0)
                        if now - last_turn_log_t >= 0.3:
                            self._vision_last_turn_log_t = now
                            self._log_session(
                                'PURE_SEG_TURN_PROGRESS',
                                f'ring_n={ring_n} yaw={yaw_prog:.1f}°/{min_yaw_turn:.0f}° '
                                f'time={turn_active_t:.2f}s/{min_hold:.2f}s '
                                f'e={error:+.3f} curve={curve:+.3f} omega={angular:+.2f} | '
                                f'{_apex_diag(scene)} cfull={int(center_full)} '
                                f'cfull_ready={int(center_full_ready)} far_reopen={int(far_reopen)} | '
                                f'SEG[far={far_now:.2f} mid={scene.get("mid", 0.0):.2f} near={scene.get("near", 0.0):.2f}] '
                                f'STATE[min_ok={int(min_turn_ok)} hard={int(hard_done)} '
                                f'overshot={int(overshot)} half_done={int(yaw_prog >= half_yaw)}]'
                            )
                        elif center_full and not center_full_ready:
                            # 进弯初期 C10 已满但还没转够：低频提示，避免误以为“该停了”
                            last_cfull_block_t = float(
                                getattr(self, '_vision_last_cfull_block_t', 0.0) or 0.0
                            )
                            if now - last_cfull_block_t >= 0.25:
                                self._vision_last_cfull_block_t = now
                                self._log_session(
                                    'PURE_SEG_CENTER_FULL_HOLD',
                                    f'yawp={yaw_prog:.1f}/{min_yaw_turn:.0f} '
                                    f'act={turn_active_t:.2f}/{min_hold:.2f} '
                                    f'far={far_now:.2f} far_reopen={int(far_reopen)} '
                                    f'heading_ok={int(heading_ok)} e={error:+.2f} '
                                    f'ring_n={ring_n} | {_apex_diag(scene)}'
                                )

                        if hard_done:
                            phase, candidate, force_bias, linear, angular = _finish_turn(
                                'hard_limit', go_align=True
                            )
                            self._log_session(
                                'PURE_SEG_CENTER_FULL_EXIT',
                                f'yawp={yaw_prog:.1f} far={far_now:.2f} '
                                f'e={error:+.2f} act={turn_active_t:.1f} why=hard_limit '
                                f'ring_n={ring_n} hold={min_hold:.2f} | {_apex_diag(scene)}'
                            )

                else:
                    # FOLLOW：走中线；弯前触发TURN
                    phase = 'follow'
                    candidate = 'PURE_SEG_FOLLOW'
                    force_bias = 0.0
                    self._vision_in_turn_phase = False
                    coff = float(scene.get('lane_center_off', 0.0) or 0.0)

                    # 顶点左右占比
                    apex_l5 = float(scene.get('apex_left5_fill', 0.0) or 0.0)
                    apex_r5 = float(scene.get('apex_right5_fill', 0.0) or 0.0)
                    apex_c10 = float(scene.get('apex_center10_fill', 0.0) or 0.0)
                    both_empty = (apex_l5 < 0.15 and apex_r5 < 0.15)
                    both_full = (apex_c10 >= 0.40)

                    # 弯前硬几何：远端空 + 前边界/early 足够强
                    # 不能只靠 both_empty，否则对面残余SEG会把转弯堵死
                    hard_front = bool(
                        (scene['ba'] and front_hold >= 0.08 and far_now < 0.30)
                        or (edge_front and front_hold >= 0.10 and far_now < 0.30)
                        or (early >= 0.70 and far_now < 0.28)
                        or (strong_front and front_hold >= 0.14 and far_now < 0.25)
                    )
                    # both_empty 是充分条件；hard_front 是短弯/近弯的兜底
                    # 近墙 hard_front 即使 real_turn 因抖动失败也允许 LOCK（短边常见）
                    want_turn = cooldown_ok and far_closed and (
                        (both_empty and real_turn)
                        or (hard_front and real_turn)
                        or (hard_front and early >= 0.70 and far_now < 0.22)
                        or (hard_front and far_now < 0.12 and front_hold >= 0.20)
                    )
                    turn_reason = (
                        'APEX_EMPTY' if both_empty and real_turn else
                        'HARD_FRONT' if hard_front else
                        'NONE'
                    )

                    if want_turn:
                        last_turn = getattr(self, '_vision_last_turn_sign', 0.0)
                        cooldown_left = max(
                            0.0,
                            float(getattr(self, '_vision_turn_cooldown_until', 0.0) or 0.0) - now,
                        )
                        self._log_session(
                            'PURE_SEG_TURN_TRIGGER',
                            f'cooldown_ok={int(cooldown_ok)} cooldown_left={cooldown_left:.2f}s '
                            f'e={error:+.3f} curve={curve:+.3f} last_turn={last_turn:+.0f} | '
                            f'{_apex_diag(scene)} both_empty={int(both_empty)} both_full={int(both_full)} '
                            f'hard_front={int(hard_front)} reason={turn_reason} | '
                            f'SEG[far={far_now:.2f} mid={scene.get("mid", 0.0):.2f} near={scene.get("near", 0.0):.2f}] '
                            f'TRIG[ba={int(scene["ba"])} edge_front={int(edge_front)} early={early:.2f} '
                            f'front_hold={front_hold:.2f} strong_front={int(strong_front)}] '
                            f'GEO[front={scene["front"]:.2f} straight={scene["straight"]:.2f} turn_geo={int(turn_geo)}] '
                            f'PATH[lane_off={coff:+.2f} cfill={cfill_now:.2f}]'
                        )
                        phase, candidate, force_bias, linear, angular = _begin_turn(
                            turn_reason, 'turn_enter'
                        )
                    else:
                        # 弯前已很明显却 LOCK 不了：打诊断，避免 silent straight 冲墙
                        cornerish = bool(
                            far_closed
                            and (
                                hard_front
                                or both_empty
                                or scene['ba']
                                or early >= 0.70
                                or scene['front'] >= 0.70
                            )
                        )
                        if cornerish:
                            last_block_t = float(
                                getattr(self, '_vision_last_turn_block_t', 0.0) or 0.0
                            )
                            if now - last_block_t >= 0.30:
                                self._vision_last_turn_block_t = now
                                cooldown_left = max(
                                    0.0,
                                    float(getattr(self, '_vision_turn_cooldown_until', 0.0) or 0.0) - now,
                                )
                                self._log_session(
                                    'PURE_SEG_TURN_BLOCKED',
                                    f'cooldown_ok={int(cooldown_ok)} cooldown_left={cooldown_left:.2f}s '
                                    f'far_closed={int(far_closed)} real_turn={int(real_turn)} '
                                    f'hard_front={int(hard_front)} both_empty={int(both_empty)} '
                                    f'both_full={int(both_full)} early={early:.2f} '
                                    f'far={far_now:.2f} front={scene["front"]:.2f} ba={int(scene["ba"])} '
                                    f'fh={front_hold:.2f} e={error:+.3f} rows={rows} conf={conf:.2f} | '
                                    f'{_apex_diag(scene)}'
                                )

                        linear = cruise

                        # 弯前禁止 straight 锁零舵：一旦远空+前边界强，只能轻跟线，不能 w=0 直冲
                        approach_corner = bool(
                            far_closed
                            and (
                                scene['ba']
                                or edge_front
                                or early >= 0.45
                                or strong_front
                                or scene['front'] >= 0.50
                                or hard_front
                            )
                        )

                        # ─────── 顶点几何判据 ───────
                        # 1. 左右都没SEG → 准备转弯（上面已处理）
                        # 2. 不对称 → 歪了 → 微调纠偏（速度不变）
                        # 3. ±10%有SEG 且不在弯前 → 回正/直行
                        asymmetric = (not both_empty) and (not both_full) and (
                            (apex_l5 < 0.20 and apex_r5 >= 0.30)
                            or (apex_r5 < 0.20 and apex_l5 >= 0.30)
                        )

                        if approach_corner:
                            # 接近弯角但还没满足 LOCK：保持环向预打，禁止 straight 锁 0
                            # 越近墙预打越大，避免“快撞墙还 w=0”
                            phase = 'approach'
                            if far_now < 0.12 or scene['front'] >= 0.85 or early >= 0.95:
                                pre_w = 0.55
                            elif early >= 0.70 or scene['front'] >= 0.70:
                                pre_w = 0.35
                            else:
                                pre_w = 0.18
                            angular = float(self.clamp(turn_sign * pre_w, max_w))
                            # 再叠一点中线微调，避免纯打死撞边
                            vis = float(self.clamp(-0.20 * error, 0.12))
                            angular = float(self.clamp(angular + vis, max_w))
                            if angular * turn_sign < 0.0:
                                angular = turn_sign * 0.12
                            self._vision_filt_angular = angular
                            force_bias = 0.55 if pre_w >= 0.35 else 0.35
                            # 弯前逼近时主动降速，给 LOCK/预打留时间
                            linear = max(vmin, min(linear, corner * 1.15 if far_now < 0.15 else cruise * 0.85))
                        elif asymmetric:
                            # 不对称：歪了一点点 → 纠偏（速度不变，微微打方向）
                            phase = 'nudge'
                            if apex_l5 < apex_r5:
                                nudge_dir = -0.18  # 向右微调
                            else:
                                nudge_dir = +0.18  # 向左微调
                            angular = float(nudge_dir)
                            self._vision_filt_angular = angular
                            force_bias = 0.0
                        elif both_full and not far_closed:
                            # 真正直道才允许锁 0；弯前 far 已空时不允许
                            phase = 'straight'
                            angular = 0.0
                            self._vision_filt_angular = 0.0
                            force_bias = 0.0
                        else:
                            # 其他情况：用几何控制
                            phase = 'follow'
                            angular = self._vision_heading_from_geometry(
                                line, scene, turn_sign=0.0, early_turn=0.0
                            )
                            force_bias = 0.0

            if weak_ok and not quality_ok:
                linear = max(vmin, linear * 0.70)
                if candidate in ('PURE_SEG_ENTRY', 'PURE_SEG_TURN'):
                    angular = self._vision_turn_angular(
                        turn_sign, error, max(0.70, force_bias),
                        max_w=entry_max_w if in_entry else max_w,
                    )
                if candidate == 'PURE_SEG_FOLLOW':
                    candidate = 'PURE_SEG_WEAK'
                    phase = 'weak'

            mode = self._hold_mode(candidate)
            self.current_speed = linear
            self._vision_scene = phase
            self._vision_bias_scale = float(force_bias)

            line = dict(line)
            line['phase'] = phase
            line['entry_turn'] = bool(in_entry)
            line['force_bias'] = float(force_bias)
            line['entry_prog'] = float(entry_progress)
            line['yaw_prog'] = float(yaw_prog)
            line['cyaw'] = float(yaw_prog)
            line['overshoot'] = bool(phase == 'overshoot')
            line['exit_hold'] = float(straight_hold)
            line['ba'] = int(scene['ba'])
            line['sba'] = int(scene['front_trigger'])
            line['bdist'] = float(scene['bd'])
            line['far'] = float(scene['far'])
            line['mid'] = float(scene['mid'])
            line['near'] = float(scene['near'])
            line['cstd'] = float(line.get('boundary_coverage_std', 0.0) or 0.0)
            line['front'] = float(scene['front'])
            line['str'] = float(scene['straight'])
            line['sign'] = float(turn_sign)
            line['early'] = float(locals().get('early', line.get('early', 0.0)) or 0.0)
            line['e_near'] = float(line.get('near_error', error) or 0.0)
            line['e_far'] = float(line.get('far_error', error) or 0.0)
            line['bend'] = float(line.get('path_bend', curve) or 0.0)
            line['left_margin'] = float(scene.get('left_margin', line.get('left_margin', 0.0)) or 0.0)
            line['right_margin'] = float(scene.get('right_margin', line.get('right_margin', 0.0)) or 0.0)
            line['lane_center_off'] = float(scene.get('lane_center_off', line.get('lane_center_off', 0.0)) or 0.0)
            line['lane_clear'] = bool(scene.get('lane_clear', line.get('lane_clear', False)))
            line['lm'] = line['left_margin']
            line['rm'] = line['right_margin']
            line['coff'] = line['lane_center_off']
            line['center_fill'] = float(scene.get('center_fill', line.get('center_fill', 0.0)) or 0.0)
            line['center_fill_5'] = float(scene.get('center_fill_5', line.get('center_fill_5', 0.0)) or 0.0)
            line['apex_has_mask'] = bool(scene.get('apex_has_mask', line.get('apex_has_mask', True)))
            line['apex_error'] = float(scene.get('apex_error', line.get('apex_error', 0.0)) or 0.0)
            line['apex_fill'] = float(scene.get('apex_fill', line.get('apex_fill', 0.0)) or 0.0)
            line['turn_geometry'] = bool(scene.get('turn_geometry', False))
            line['edge_angle_deg'] = float(scene.get('edge_angle_deg', line.get('edge_angle_deg', 90.0)) or 90.0)
            line['cfill'] = line['center_fill']
            line['cfill5'] = line['center_fill_5']
            line['apex'] = int(line['apex_has_mask'])
            line['aerr'] = line['apex_error']
            line['eang'] = line['edge_angle_deg']
            line['vang'] = float(self._vision_edge_to_vehicle_angle_deg(line['edge_angle_deg']))
            line['clr'] = line['lane_clear']
            line['fh'] = float(front_hold)
            line['sh'] = float(straight_hold)
            line['act'] = float(entry_active_t if in_entry else turn_active_t)
            return float(linear), float(angular), mode, line

        # ========================= 无视觉 =========================
        last_valid = float(getattr(self, '_vision_pure_last_valid_t', 0.0) or 0.0)
        had_valid = bool(getattr(self, '_vision_had_valid', False))
        lost_for = (now - last_valid) if last_valid > 0.0 else 999.0

        # 入口阶段丢线：继续打死转弯 + 保持线速度，禁止几乎停车
        if not entry_done:
            turn_sign = self._vision_entry_turn_sign()
            self._turn_direction_cache = turn_sign
            self._vision_search_sign = turn_sign
            yb = self._vision_yaw_progress_deg(
                getattr(self, '_vision_entry_start_yaw', None), turn_sign)
            hard_w = float(getattr(self, '_vision_entry_max_angular', 0.80) or 0.80)
            hard_w = max(hard_w, float(getattr(self, '_vision_max_angular', 0.80) or 0.80))
            # 转够很多才允许收舵，否则一直打死
            if yb >= 85.0:
                w = 0.0
            else:
                w = float(self.clamp(turn_sign * hard_w, hard_w))
            linear = max(vmin, float(getattr(self, '_vision_corner_speed', 0.30) or 0.30))
            mode = self._hold_mode('PURE_SEG_ENTRY')
            self.current_speed = linear
            self._vision_in_turn_phase = True
            line = dict(line)
            line.update({
                'phase': 'entry_blind',
                'entry_turn': 1,
                'force_bias': 1.0,
                'sign': float(turn_sign),
                'entry_prog': float(entry_progress),
                'yaw_prog': float(yb),
                'front': float(scene.get('front', 0.0)),
                'str': float(scene.get('straight', 0.0)),
                'far': float(scene.get('far', 0.0)),
                'mid': float(scene.get('mid', 0.0)),
                'near': float(scene.get('near', 0.0)),
                'ba': int(scene.get('ba', False)),
                'sba': 0,
                'fh': float(front_hold),
                'sh': 0.0,
                'exit_hold': 0.0,
                'overshoot': False,
                'act': float(entry_active_t),
                'cyaw': 0.0,
                'bdist': float(scene.get('bd', 0.0)),
                'cstd': 0.0,
            })
            return float(linear), float(w), mode, line

        # 入环后丢线
        # 弯中丢线是常态（镜头扫到墙/空地）：开环按环向续转，禁止立刻停车卡死
        ring = self._vision_ring_turn_sign()
        search_dir = self._vision_search_direction()
        lost_for_now = (now - last_valid) if last_valid > 0.0 else 999.0
        entry_done = bool(getattr(self, '_vision_entry_turn_done', False))
        corner = float(getattr(self, '_vision_corner_speed', 0.15))
        vmin = float(getattr(self, '_vision_min_speed', 0.08))

        if (not entry_done) or bool(getattr(self, '_vision_in_turn_phase', False)):
            # 入口/弯中丢线：可续转，但禁止转晕（>78° 必须停转）
            if not entry_done:
                turn_sign = self._vision_entry_turn_sign()
                y0 = getattr(self, '_vision_entry_start_yaw', None)
            else:
                turn_sign = ring
                y0 = getattr(self, '_vision_corner_start_yaw', None)
            cyaw = self._vision_yaw_progress_deg(y0, turn_sign)
            max_w_lost = float(getattr(self, '_vision_max_angular', 0.80) or 0.80)
            scale = 1.0
            # 转够 ~78° / 丢线过久：停转进 ALIGN，绝不再打死到 100°+
            if cyaw >= 78.0 or (cyaw >= 60.0 and lost_for_now > 1.0) or lost_for_now > 2.2:
                search_w = 0.0
                linear_lost = max(vmin, corner * 0.85)
                self._vision_in_turn_phase = False
                self._vision_align_active = True
                self._vision_align_until = now + 0.70
                if entry_done:
                    self._vision_corner_start_yaw = None
                    self._vision_turn_cooldown_until = now + 1.6
                self._vision_filt_angular = 0.0
                phase_lost = 'lost_hold'
                scale = 0.0
            elif cyaw >= 50.0:
                # 后半弯：收舵继续找线
                search_w = float(self.clamp(turn_sign * min(max_w_lost, 0.40), 0.45))
                linear_lost = max(vmin, corner)
                phase_lost = 'lost_turn'
                scale = 0.5
            else:
                # 前半弯丢线：仍打死 + 弯速
                search_w = float(self.clamp(turn_sign * max_w_lost, max_w_lost))
                linear_lost = max(vmin, corner)
                phase_lost = 'lost_turn'
                scale = 1.0
            # 写入 line 诊断
            line = dict(line)
            line['phase'] = phase_lost
            line['sign'] = float(turn_sign)
            line['yaw_prog'] = float(cyaw)
            line['force_bias'] = float(scale)
            line['entry_turn'] = int(not entry_done)
            mode = self._hold_mode('PURE_SEG_TURN' if abs(search_w) > 1e-3 else 'PURE_SEG_ALIGN')
            self.current_speed = linear_lost
            self._vision_scene = phase_lost
            # 补全常用字段
            for k, v in {
                'front': float(scene.get('front', 0.0)), 'str': float(scene.get('straight', 0.0)),
                'far': float(scene.get('far', 0.0)), 'mid': float(scene.get('mid', 0.0)),
                'near': float(scene.get('near', 0.0)), 'ba': int(scene.get('ba', False)),
                'early': 0.0, 'lm': float(scene.get('left_margin', 0.0)),
                'rm': float(scene.get('right_margin', 0.0)),
                'coff': float(scene.get('lane_center_off', 0.0)),
                'cfill': float(scene.get('center_fill', 0.0)),
                'cfill5': float(scene.get('center_fill_5', 0.0)),
                'apex': int(bool(scene.get('apex_has_mask', False))),
                'aerr': float(scene.get('apex_error', 0.0)),
                'eang': float(scene.get('edge_angle_deg', 90.0)),
                'clr': bool(scene.get('lane_clear', False)),
                'entry_prog': float(entry_progress),
                'act': float(cyaw),
            }.items():
                line.setdefault(k, v)
            return float(linear_lost), float(search_w), mode, line

        # 非弯中丢线：慢爬小找，不要永久停车
        if lost_for_now < 0.8:
            search_w = 0.0
            linear_lost = max(vmin * 0.7, 0.06)
        elif lost_for_now < 2.0:
            search_w = self.clamp(ring * 0.10, 0.10)
            linear_lost = max(vmin * 0.5, 0.04)
        elif lost_for_now < 4.0:
            search_w = self.clamp(search_dir * 0.12, 0.12)
            linear_lost = 0.03
        else:
            search_w = self.clamp(search_dir * 0.10, 0.10)
            linear_lost = 0.02

        line = dict(line)
        line.update({
            'phase': 'lost',
            'entry_turn': 0,
            'force_bias': 0.0,
            'sign': float(search_dir),
            'front': float(scene.get('front', 0.0)),
            'str': float(scene.get('straight', 0.0)),
            'far': float(scene.get('far', 0.0)),
            'mid': float(scene.get('mid', 0.0)),
            'near': float(scene.get('near', 0.0)),
            'ba': int(scene.get('ba', False)),
            'sba': 0,
            'fh': float(front_hold),
            'sh': float(straight_hold),
            'exit_hold': float(straight_hold),
            'overshoot': False,
            'entry_prog': float(entry_progress),
            'yaw_prog': 0.0,
            'cyaw': 0.0,
            'bdist': float(scene.get('bd', 0.0)),
            'cstd': 0.0,
            'act': float(turn_active_t),
            'cfill': float(scene.get('center_fill', 0.0)),
            'eang': float(scene.get('edge_angle_deg', 90.0)),
            'coff': float(scene.get('lane_center_off', 0.0)),
            'clr': bool(scene.get('lane_clear', False)),
        })

        if not had_valid:
            linear = max(vmin * 0.5, 0.04)
            mode = self._hold_mode('PURE_SEG_WAIT')
            self.current_speed = linear
            return float(linear), float(search_w), mode, line

        if lost_for <= timeout:
            mode = self._hold_mode('PURE_SEG_SEARCH')
            self.current_speed = linear_lost
            return float(linear_lost), float(search_w), mode, line
        if lost_for <= hard_lost_sec:
            mode = self._hold_mode('PURE_SEG_SEARCH_HARD')
            self.current_speed = linear_lost
            return float(linear_lost), float(search_w), mode, line

        mode = self._hold_mode('PURE_SEG_LOST')
        self.current_speed = linear_lost
        return float(linear_lost), float(search_w), mode, line

    def _estimate_ring_mission_distance(self) -> float:
        """根据 field_track 段长估算纯视觉任务总里程。"""
        total = 0.0
        plan = list(getattr(self, 'plan', []) or [])
        for seg in plan:
            if not isinstance(seg, dict):
                continue
            stype = str(seg.get('type', ''))
            if stype == 'move':
                total += abs(float(seg.get('distance_m', 0.0) or 0.0))
            elif stype == 'arc':
                # 粗估：速度×时间
                speed = abs(float(seg.get('speed', 0.18) or 0.18))
                duration = abs(float(seg.get('duration_sec', 0.0) or 0.0))
                total += speed * duration
            elif stype == 'turn':
                # 转弯段也有前向速度
                speed = abs(float(seg.get('turn_linear_speed', getattr(self, 'turn_linear_speed', 0.12)) or 0.12))
                # 以 90°/0.8rad/s 粗估
                total += speed * 1.2
        if total < 1.0:
            # 兜底：矩形环约 2.8 + 2*0.3 + 0.9 + 入口
            total = 6.0
        scale = float(getattr(self, '_vision_mission_distance_scale', 1.0) or 1.0)
        return max(1.0, total * max(0.3, scale))

    def start_pure_vision_mission(self):
        """启动纯 SEG 跟线任务（替换 field_track 段执行）。"""
        # 先在覆盖 plan 前估算里程
        target_distance = self._estimate_ring_mission_distance()

        self.mission_active = True
        self.mission_finished = False
        self.current_segment = {
            'type': 'vision_follow',
            'description': 'pure_seg_follow',
            'speed': float(getattr(self, '_vision_cruise_speed', 0.22)),
        }
        self.plan = [self.current_segment]
        self.plan_index = 0
        self.segment_started_at = self.get_clock().now().nanoseconds / 1e9
        self.segment_start_pose = self.current_position
        self.segment_start_yaw = self.current_yaw if self.current_yaw is not None else self.navigation_yaw()
        self.segment_heading = self.segment_start_yaw
        self._vision_pure_start_pose = self.current_position
        self._vision_pure_start_t = self.segment_started_at
        self._vision_pure_path_m = 0.0
        self._vision_pure_last_pos = self.current_position
        self._vision_finish_seen_low_x = False
        self._vision_finish_armed = False
        self._vision_pure_last_valid_t = 0.0
        self._vision_pure_last_log_t = 0.0
        self._vision_pure_mode_name = 'PURE_SEG_START'
        self._vision_had_valid = False
        self._vision_has_filt = False
        self._vision_filt_angular = 0.0
        self._vision_lost_flip_t = 0.0
        self._vision_exit_turn_t = 0.0
        self._vision_in_turn_phase = True          # 一进 S2 就准备转
        self._vision_align_active = False
        self._vision_align_until = 0.0
        self._vision_entry_turn_done = False
        self._vision_front_hold = 0.0
        self._vision_front_last_t = None
        self._vision_straight_hold = 0.0
        self._vision_straight_last_t = None
        self._vision_turn_cooldown_until = 0.0
        self._vision_last_turn_exit_t = 0.0
        self._vision_bias_scale = 0.0
        self._vision_force_bias_scale = 0.0
        self._vision_scene = 'entry'
        self._vision_mode_hold = 'PURE_SEG_ENTRY'
        self._vision_mode_hold_t = 0.0
        self._vision_entry_active_t = 0.0
        self._vision_entry_active_last_t = None
        self._vision_turn_active_t = 0.0
        self._vision_turn_active_last_t = None
        self._vision_corner_start_yaw = None
        self._vision_ring_turn_count = 0  # 环内已完成转弯次数；第1次 hold=0.5s，之后 0.3s
        y0 = self.segment_start_yaw
        if y0 is None:
            y0 = self.navigation_yaw() if hasattr(self, 'navigation_yaw') else None
        self._vision_entry_start_yaw = y0
        # 入口符号：顺左 / 逆右
        self._turn_direction_cache = self._vision_entry_turn_sign()
        self._vision_search_sign = self._vision_entry_turn_sign()
        self._vision_pure_target_distance = target_distance

        # 重置避障控制器
        if hasattr(self, '_avoider') and self._avoider is not None:
            self._avoider.reset()
            self.get_logger().info('[MISSION] 避障控制器已重置')

        if hasattr(self, '_set_vision_inference_active'):
            self._set_vision_inference_active(True)
        elif getattr(self, '_vision_node', None) is not None:
            self._vision_node.set_inference_active(True)
        self.publish_state('pure_seg_follow')
        self.publish_feedback(
            f'第二阶段纯SEG启动 direction={self.direction} target≈{self._vision_pure_target_distance:.2f}m'
        )
        self.get_logger().info(
            f'[MISSION] ✓ 纯SEG主控启动 direction={self.direction} '
            f'target={self._vision_pure_target_distance:.2f}m '
            f'timeout={float(getattr(self, "_vision_mission_timeout_sec", 90.0)):.1f}s'
        )
        if hasattr(self, '_log_session'):
            self._log_session(
                'PURE_SEG_START',
                f'direction={self.direction} target={self._vision_pure_target_distance:.2f}m',
            )

    def _vision_update_path_length(self) -> float:
        """累计真实路径长度（逐帧位移积分），绕环不会回零。"""
        pos = self.current_position
        if pos is None:
            return float(getattr(self, '_vision_pure_path_m', 0.0) or 0.0)
        last = getattr(self, '_vision_pure_last_pos', None)
        if last is None:
            self._vision_pure_last_pos = pos
            return float(getattr(self, '_vision_pure_path_m', 0.0) or 0.0)
        step = math.hypot(pos[0] - last[0], pos[1] - last[1])
        # 滤掉跳变
        if 1e-4 < step < 0.35:
            self._vision_pure_path_m = float(getattr(self, '_vision_pure_path_m', 0.0) or 0.0) + step
        self._vision_pure_last_pos = pos
        return float(self._vision_pure_path_m)

    def _vision_check_lap_finish(self, path_m: float) -> tuple:
        """
        一圈结束判据（比赛几何）：
          1) 入口已完成
          2) 路径里程 >= min_path（保证真的跑过一圈，不是刚出发）
          3) 曾到过小 x 区（例如 x<1.5，说明绕出去了）
          4) 回到通道口附近：x ≈ 2.5（默认 2.15~2.85），y 在合理带内

        用户描述：x 从 1→2 说明跑完一圈，到 x≈2.5 触发第三阶段。
        """
        if not bool(getattr(self, '_vision_entry_turn_done', False)):
            return False, 'entry_not_done'
        min_path = float(getattr(self, '_vision_finish_min_path_m', 5.0) or 5.0)
        if path_m < min_path:
            return False, f'path_short:{path_m:.2f}<{min_path:.2f}'

        pos = self.current_position
        if pos is None:
            return False, 'no_pose'
        x, y = float(pos[0]), float(pos[1])

        # 记录是否去过环外侧/小 x
        low_x_thresh = float(getattr(self, '_vision_finish_x_m', 2.50)) - 1.0  # ~1.5
        if x < low_x_thresh:
            self._vision_finish_seen_low_x = True

        require_rise = bool(getattr(self, '_vision_finish_require_x_rise', True))
        if require_rise and not bool(getattr(self, '_vision_finish_seen_low_x', False)):
            return False, f'need_low_x_first x={x:.2f}'

        x_goal = float(getattr(self, '_vision_finish_x_m', 2.50))
        x_tol = float(getattr(self, '_vision_finish_x_tol_m', 0.35))
        y_min = float(getattr(self, '_vision_finish_y_min_m', 1.60))
        y_max = float(getattr(self, '_vision_finish_y_max_m', 3.20))

        in_x = abs(x - x_goal) <= x_tol
        in_y = (y_min <= y <= y_max)
        if in_x and in_y:
            return True, f'lap_done path={path_m:.2f} xy=({x:.2f},{y:.2f})'

        # 也允许：x 已越过 goal（从内侧回到通道口再往外一点）
        if require_rise and bool(getattr(self, '_vision_finish_seen_low_x', False)):
            if x >= (x_goal - 0.10) and in_y:
                return True, f'lap_x_cross path={path_m:.2f} xy=({x:.2f},{y:.2f})'

        return False, f'wait_gate x={x:.2f} y={y:.2f} path={path_m:.2f}'

    def run_pure_vision_mission(self):
        """纯 SEG 控制循环：跟线 + 路径里程/回廊口结束。"""
        if not self.mission_active:
            self.cmd_pub.publish(self.create_twist())
            return

        now = self.get_clock().now().nanoseconds / 1e9
        start_t = float(getattr(self, '_vision_pure_start_t', now) or now)
        timeout = float(getattr(self, '_vision_mission_timeout_sec', 90.0) or 90.0)
        if now - start_t > timeout:
            self.get_logger().warn(f'[视觉] 纯SEG超时 {timeout:.1f}s，结束任务')
            if hasattr(self, '_log_session'):
                self._log_session('PURE_SEG_TIMEOUT', f't={now - start_t:.1f}s')
            self.finish_mission()
            return

        # 路径长度（积分）才是“跑了多远”；欧氏位移绕环会缩回
        path_m = self._vision_update_path_length()
        progress = path_m
        target = float(getattr(self, '_vision_pure_target_distance', 8.0) or 8.0)

        done, why = self._vision_check_lap_finish(path_m)
        if done:
            self.get_logger().info(f'[视觉] 纯SEG一圈完成 {why}')
            if hasattr(self, '_log_session'):
                self._log_session('PURE_SEG_DONE', why)
            self.finish_mission()
            return
        # 兜底：路径特别长也结束（防几何门永远不触发）
        if path_m >= max(target * 1.15, min_path := float(getattr(self, '_vision_finish_min_path_m', 5.0)) + 4.0):
            self.get_logger().info(
                f'[视觉] 纯SEG路径兜底完成 path={path_m:.2f}m target={target:.2f}m'
            )
            if hasattr(self, '_log_session'):
                self._log_session('PURE_SEG_DONE', f'path_fallback={path_m:.2f}/{target:.2f}')
            self.finish_mission()
            return

        # ═══ 激光雷达避障检测 ═══
        # 检查避障控制器是否激活
        if hasattr(self, '_avoider') and self._avoider is not None:
            # 构造 NavState 用于避障判断
            from racing_stage2.avoid_controller import NavState
            nav = NavState(
                position=self.current_position if self.current_position is not None else (0.0, 0.0),
                yaw=self.navigation_yaw() if hasattr(self, 'navigation_yaw') and self.navigation_yaw() is not None else (self.current_yaw if self.current_yaw is not None else 0.0),
                segment_heading=getattr(self, 'segment_heading', None) or getattr(self, 'segment_start_yaw', None) or (self.current_yaw if self.current_yaw is not None else 0.0),
                segment_start_pose=getattr(self, 'segment_start_pose', None) or self.current_position or (0.0, 0.0),
                current_segment={'type': 'move', 'allow_detour': True, 'description': 'pure_seg_follow'},
                projected_distance=progress,
            )

            # 检查避障是否已激活
            was_avoiding = bool(getattr(self._avoider, 'is_active', False))

            # 调用避障控制器的 step 方法
            # 如果避障激活，step() 会返回 True 并直接发布避障指令
            if self._avoider.step(nav):
                # 避障正在进行，直接返回
                if not was_avoiding:
                    # 刚进入避障状态
                    if hasattr(self, '_log_session'):
                        front_dist = getattr(self, 'front_obstacle_distance', float('inf'))
                        left_dist = getattr(self, 'left_clearance_distance', float('inf'))
                        right_dist = getattr(self, 'right_clearance_distance', float('inf'))
                        self._log_session(
                            'PURE_SEG_AVOID_START',
                            f'激光避障启动 | front={self.format_distance(front_dist)}m '
                            f'left={self.format_distance(left_dist)}m '
                            f'right={self.format_distance(right_dist)}m | '
                            f'path={path_m:.2f}m'
                        )
                return

            # 避障刚完成
            if was_avoiding:
                if hasattr(self, '_log_session'):
                    self._log_session(
                        'PURE_SEG_AVOID_DONE',
                        f'激光避障完成，恢复视觉跟线 | path={path_m:.2f}m'
                    )

        # ═══ 视觉跟线控制 ═══
        linear, angular, mode, line = self._compute_pure_vision_command()
        prev_mode = getattr(self, '_vision_pure_mode_name', '')
        self._vision_pure_mode_name = mode
        self.cmd_pub.publish(self.create_twist(linear, angular))

        # 模式切换立即打日志；平时 2Hz
        mode_changed = (prev_mode != mode)
        due = (now - float(getattr(self, '_vision_pure_last_log_t', 0.0) or 0.0) >= 0.5)
        if mode_changed or due:
            self._vision_pure_last_log_t = now
            lost_for = 0.0
            last_valid = float(getattr(self, '_vision_pure_last_valid_t', 0.0) or 0.0)
            if last_valid > 0.0:
                lost_for = max(0.0, time.time() - last_valid)
            rem_m = line.get('remaining_m')
            rem_txt = f'{float(rem_m):.2f}' if rem_m is not None else 'N/A'
            la = line.get('lookahead_point')
            la_txt = (
                f'({float(la[0]):+.2f},{float(la[1]):+.2f})'
                if la is not None and len(la) >= 2 else 'None'
            )
            scene = str(getattr(self, '_vision_scene', '-') or '-')
            bias = float(getattr(self, '_vision_bias_scale', 0.0) or 0.0)
            msg = (
                f'{mode} prog={progress:.2f}/{target:.2f}m '
                f't={now - start_t:.1f}s '
                f'v={linear:+.3f} w={angular:+.3f} '
                f'e={float(line.get("error", 0.0)):+.3f} '
                f'curve={float(line.get("curve", 0.0)):+.3f} '
                f'rows={int(line.get("valid_rows", 0))} '
                f'conf={float(line.get("confidence", 0.0)):.2f} '
                f'age={float(line.get("age", 999.0)):.2f}s '
                f'lost_for={lost_for:.2f}s '
                f'phase={line.get("phase","-")} '
                f'sign={float(line.get("sign", 0.0) or 0.0):+.0f} '
                f'entry={int(bool(line.get("entry_turn", False)))} '
                f'fb={float(line.get("force_bias", 0.0) or 0.0):.2f} '
                f'ep={float(line.get("entry_prog", 0.0) or 0.0):.2f} '
                f'front={float(line.get("front", 0.0) or 0.0):.2f} '
                f'early={float(line.get("early", 0.0) or 0.0):.2f} '
                f'efar={float(line.get("e_far", 0.0) or 0.0):+.2f} '
                f'bend={float(line.get("bend", 0.0) or 0.0):+.2f} '
                f'lm={float(line.get("lm", line.get("left_margin", 0.0)) or 0.0):.2f} '
                f'rm={float(line.get("rm", line.get("right_margin", 0.0)) or 0.0):.2f} '
                f'coff={float(line.get("coff", line.get("lane_center_off", 0.0)) or 0.0):+.2f} '
                f'cfill={float(line.get("cfill", line.get("center_fill", 0.0)) or 0.0):.2f} '
                f'cfill5={float(line.get("cfill5", line.get("center_fill_5", 0.0)) or 0.0):.2f} '
                f'apex={int(bool(line.get("apex", line.get("apex_has_mask", True))))} '
                f'aerr={float(line.get("aerr", line.get("apex_error", 0.0)) or 0.0):+.2f} '
                f'eang={float(line.get("eang", line.get("edge_angle_deg", 90.0)) or 90.0):.0f} '
                f'vang={float(line.get("vang", 0.0) or 0.0):.0f} '
                f'clr={int(bool(line.get("lane_clear", line.get("clr", False))))} '
                f'str={float(line.get("str", 0.0) or 0.0):.2f} '
                f'fh={float(line.get("fh", 0.0) or 0.0):.2f} '
                f'sh={float(line.get("sh", line.get("exit_hold", 0.0)) or 0.0):.2f} '
                f'act={float(line.get("act", 0.0) or 0.0):.1f} '
                f'yawp={float(line.get("yaw_prog", 0.0) or 0.0):.1f} '
                f'ba={int(bool(line.get("boundary_ahead", False) or line.get("ba", 0)))} '
                f'sba={int(bool(line.get("sba", 0)))} '
                f'far={float(line.get("far", line.get("boundary_far_ratio", 0.0)) or 0.0):.2f} '
                f'mid={float(line.get("mid", line.get("boundary_mid_ratio", 0.0)) or 0.0):.2f} '
                f'near={float(line.get("near", line.get("boundary_near_ratio", 0.0)) or 0.0):.2f} '
                f'turn={int(bool(getattr(self,"_vision_in_turn_phase",False)))} '
                f'edone={int(bool(getattr(self,"_vision_entry_turn_done",False)))} '
                f'lowx={int(bool(getattr(self,"_vision_finish_seen_low_x",False)))}'
            )
            if self.current_position is not None:
                msg += f' xy=({self.current_position[0]:.2f},{self.current_position[1]:.2f})'
            self.get_logger().info(f'[视觉] {msg}')
            if hasattr(self, '_log_session'):
                self._log_session('PURE_SEG_CTRL', msg)
            if mode_changed and hasattr(self, '_log_session'):
                self._log_session('PURE_SEG_MODE', f'{prev_mode or "INIT"} -> {mode}')

    def _check_imu_health(self) -> bool:
        """
        检测 IMU 是否正常（检测异常跳变）。
        
        返回：
            True: IMU 正常
            False: IMU 异常（yaw 跳变过大）
        """
        current_yaw = self.navigation_yaw()
        if current_yaw is None:
            return False
        
        current_time = time.time()
        
        # 首次调用，初始化
        if self._last_imu_yaw is None or self._last_imu_time is None:
            self._last_imu_yaw = current_yaw
            self._last_imu_time = current_time
            return True
        
        dt = current_time - self._last_imu_time
        if dt < 0.01:  # 避免除零，控制周期至少 10ms
            return self._imu_healthy
        
        # 检测 yaw 跳变率
        dyaw = abs(self.angle_error(current_yaw, self._last_imu_yaw))
        yaw_rate = dyaw / dt
        max_yaw_rate = math.radians(float(self.get_parameter('imu_max_yaw_rate_deg_s').value))
        
        if yaw_rate > max_yaw_rate:
            if self._imu_healthy:  # 状态变化时记录
                self.get_logger().warn(
                    f'[IMU] 异常跳变检测 {math.degrees(yaw_rate):.1f}°/s > '
                    f'{math.degrees(max_yaw_rate):.1f}°/s'
                )
                self._log_session(
                    'IMU_FAULT',
                    f'异常跳变 {math.degrees(yaw_rate):.1f}°/s | '
                    f'dyaw={math.degrees(dyaw):.2f}° dt={dt:.3f}s'
                )
            self._imu_healthy = False
        else:
            if not self._imu_healthy:  # 恢复时记录
                self.get_logger().info(f'[IMU] 恢复正常 yaw_rate={math.degrees(yaw_rate):.1f}°/s')
                self._log_session('IMU_RECOVER', f'恢复正常 yaw_rate={math.degrees(yaw_rate):.1f}°/s')
            self._imu_healthy = True
        
        self._last_imu_yaw = current_yaw
        self._last_imu_time = current_time
        return self._imu_healthy
    
    def _compute_move_lateral_angular_with_vision(self) -> float:
        """
        move/短边横向控制：
          1) 视觉中线主（多行前瞻 PD + 曲率）
          2) IMU 段航向兜底
          3) 默认不因短预算永久闭嘴
        """
        # === 1. 视觉中线 ===
        vision_available = False
        vision_angular = 0.0
        vision_error = 0.0
        vision_curve = 0.0
        vision_conf = 0.0
        vision_rows = 0
        vision_age = 999.0
        vision_centered = False
        vision_budget_exhausted = False

        line = self._get_vision_line_status()
        vision_timeout = float(self.get_parameter('vision_timeout_sec').value)
        vision_age = float(line.get('age', 999.0))
        min_conf = float(getattr(self, '_vision_min_confidence', 0.35))
        min_rows = int(self.get_parameter('vision_min_valid_rows').value) if self.has_parameter('vision_min_valid_rows') else 4

        # 边界安全状态
        boundary_safe = bool(line.get('boundary_safe', True))
        safety_weight = float(line.get('safety_weight', 1.0))

        if bool(line.get('valid', False)) and vision_age < vision_timeout:
            vision_error = float(line.get('error', 0.0))
            vision_curve = float(line.get('curve', 0.0))
            vision_conf = float(line.get('confidence', 0.0))
            vision_rows = int(line.get('valid_rows', 0))
            vision_centered = bool(line.get('centered', False))
            quality_ok = (vision_conf >= min_conf) and (vision_rows >= max(2, min_rows // 2))
            candidate = self._vision_line_to_angular(vision_error, vision_curve, line_status=line)
            # 居中时视觉ω=0，但仍视为可用（不掉到 LOST）
            if quality_ok and (vision_centered or abs(candidate) <= 1e-6):
                vision_available = True
                vision_angular = 0.0
                if hasattr(self, '_vision_offset_time_allows'):
                    self._vision_offset_time_allows(False)
            elif quality_ok:
                budget_disable = bool(getattr(self, '_vision_budget_disable_enabled', False))
                if budget_disable and hasattr(self, '_vision_offset_time_allows'):
                    if self._vision_offset_time_allows(True):
                        vision_available = True
                        vision_angular = candidate
                    else:
                        vision_available = False
                        vision_angular = 0.0
                        vision_budget_exhausted = True
                else:
                    # 默认：质量合格就持续主控
                    vision_available = True
                    vision_angular = candidate
                    if hasattr(self, '_vision_offset_time_allows'):
                        self._vision_offset_time_allows(False)
            else:
                vision_available = False
                vision_angular = 0.0
                if hasattr(self, '_vision_offset_time_allows'):
                    self._vision_offset_time_allows(False)
        else:
            if hasattr(self, '_vision_offset_time_allows'):
                self._vision_offset_time_allows(False)

        # === 2. IMU 段航向 ===
        imu_angular = 0.0
        heading_error = 0.0
        if self.current_position is not None and self.segment_heading is not None:
            nav_yaw = self.navigation_yaw()
            if nav_yaw is not None:
                heading_error = self.angle_error(self.segment_heading, nav_yaw)
                deadzone = math.radians(float(self.get_parameter('imu_heading_deadzone_deg').value))
                if abs(heading_error) >= deadzone:
                    imu_angular = self.clamp(self.heading_kp * heading_error, self.max_angular_speed)

        imu_healthy = self._check_imu_health()
        fusion_enabled = bool(self.get_parameter('fusion_mode_enabled').value)
        imu_corr_enabled = bool(self.get_parameter('imu_heading_correction_enabled').value)
        vis_corr_enabled = bool(self.get_parameter('vision_offset_correction_enabled').value)
        primary = bool(getattr(self, '_vision_primary_control', True))

        if not fusion_enabled:
            imu_angular = 0.0
            vision_angular = 0.0
            weight_vision = 0.0
            weight_imu = 0.0
            mode = 'FUSION_DISABLED'
        else:
            if not imu_corr_enabled:
                imu_angular = 0.0
            if not vis_corr_enabled:
                vision_angular = 0.0
                vision_available = False

            if primary and vision_available and vis_corr_enabled:
                # 视觉主：有效时几乎全视觉；仅在视觉接近0时混一点 IMU 稳直
                head_abs_deg = abs(math.degrees(heading_error))
                max_head = float(getattr(self, '_vision_primary_max_head_err_deg', 35.0))

                # 边界保护：接近边界时强制降低视觉权重
                if not boundary_safe:
                    weight_vision = min(0.75, safety_weight)  # 上限75%
                    weight_imu = 1.0 - weight_vision
                    mode = f'BOUNDARY_PROTECT(w={safety_weight:.2f})'
                elif abs(vision_angular) < 1e-4 and imu_healthy and imu_corr_enabled:
                    weight_vision = 0.0
                    weight_imu = 1.0
                    mode = 'IMU_ONLY(VIS_CENTER)' if vision_centered else 'IMU_ONLY(VIS_ZERO)'
                elif head_abs_deg > max_head and imu_healthy and imu_corr_enabled:
                    # 车头偏太多时不要死跟视觉，先用 IMU 拉回段航向
                    # 视觉仍保留弱权重，避免完全丢中线
                    blend = min(1.0, (head_abs_deg - max_head) / max(max_head, 1.0))
                    weight_imu = 0.55 + 0.35 * blend
                    weight_vision = 1.0 - weight_imu
                    mode = 'VIS_BLEND(HEAD_ERR)'
                else:
                    weight_vision = 1.0
                    weight_imu = 0.0
                    mode = 'VIS_PRIMARY'
            elif vision_available and imu_healthy and vis_corr_enabled and imu_corr_enabled:
                weight_imu = float(self.get_parameter('fusion_weight_imu').value)
                weight_vision = float(self.get_parameter('fusion_weight_vision').value)
                mode = 'FUSION'
            elif vision_available and vis_corr_enabled:
                weight_vision = 1.0
                weight_imu = 0.0
                mode = 'VIS_ONLY' if not imu_healthy else 'VIS_ONLY(IMU_DISABLED)'
            elif imu_healthy and imu_corr_enabled:
                weight_vision = 0.0
                weight_imu = 1.0
                if vision_budget_exhausted:
                    mode = 'IMU_ONLY(VIS_BUDGET)'
                elif not vision_available:
                    mode = 'IMU_ONLY(VIS_LOST)'
                else:
                    mode = 'IMU_ONLY'
            else:
                weight_vision = 0.0
                weight_imu = 0.0
                mode = 'LOST'

        angular_fused = weight_imu * imu_angular + weight_vision * vision_angular

        # 日志：状态变化 + 2Hz 细节
        if self._last_fusion_mode != mode:
            self._last_fusion_mode = mode
            self._log_session(
                'FUSION_STATE',
                f'{mode} | vis_valid={vision_available} imu_healthy={imu_healthy} '
                f'age={vision_age:.2f}s e={vision_error:+.3f} curve={vision_curve:+.3f} '
                f'rows={vision_rows} conf={vision_conf:.2f}'
            )
        now = time.time()
        if now - float(getattr(self, '_vision_last_detail_log_t', 0.0)) >= 0.5:
            self._vision_last_detail_log_t = now
            self._log_session(
                'VISION_CTRL',
                f'{mode} e={vision_error:+.3f} curve={vision_curve:+.3f} rows={vision_rows} '
                f'conf={vision_conf:.2f} age={vision_age:.2f}s '
                f'vis_w={vision_angular:+.3f} imu_w={imu_angular:+.3f} '
                f'head_err={math.degrees(heading_error):+.1f}deg fused={angular_fused:+.3f} '
                f'wv={weight_vision:.2f} wi={weight_imu:.2f} '
                f'seg={(self.current_segment or {}).get("type","?")}/{(self.current_segment or {}).get("description","?")}'
            )
        return float(angular_fused)

    def _get_vision_angular_for_avoider(self):
        """
        供 avoider 调用的视觉修正回调（用于 leg2 段）。
        
        返回：
            float: 角速度 (rad/s)，或 None（无有效检测/时长用尽）
        """
        if not self._vision_enabled or self._vision_node is None:
            return None
        offset, timestamp, valid = self._vision_node.get_latest_offset()
        if not valid:
            self._vision_offset_time_allows(False)
            return None
        angular = self._vision_offset_to_angular(offset)
        vision_active = abs(angular) > 1e-6
        if not self._vision_offset_time_allows(vision_active):
            return None
        self._log_session(
            'LEG2_VIS',
            f'leg2 视觉修正 offset={offset:+.3f} ω={angular:+.3f} rad/s '
            f't={self._vision_corr_elapsed:.2f}/{self._vision_offset_max_sec:.2f}s',
        )
        return angular

    def _reset_vision_length_state(self) -> None:
        """新 move 段开始时重置纵向视觉状态。"""
        self._vision_length_hit_count = 0
        self._vision_length_last_target = None
        self._reset_vision_offset_time_state()

    def _get_vision_remaining(self):
        """读取视觉剩余距离估计，失败时返回 (None, 0.0, False)。"""
        if self._vision_node is None:
            return None, 0.0, False
        if not bool(getattr(self, '_vision_length_enabled', False)):
            return None, 0.0, False
        if not hasattr(self._vision_node, 'get_latest_remaining'):
            return None, 0.0, False
        remaining_m, free_ratio, _ts, valid = self._vision_node.get_latest_remaining()
        if not valid or remaining_m is None:
            return None, float(free_ratio or 0.0), False
        return float(remaining_m), float(free_ratio), True

    def _vision_adjusted_move_target(self, nominal_target: float, progress: float):
        """
        用视觉剩余距离修正 move 段目标长度。

        返回：
            (target_distance, remaining_m, free_ratio, valid, reason)
        """
        nominal = max(1e-6, float(nominal_target))
        progress = max(0.0, float(progress))
        if not bool(getattr(self, '_vision_length_enabled', False)):
            return nominal, None, 0.0, False, 'disabled'

        remaining_m, free_ratio, valid = self._get_vision_remaining()
        if not valid or remaining_m is None:
            self._vision_length_hit_count = 0
            return nominal, remaining_m, free_ratio, False, 'invalid'

        min_ratio = float(self.get_parameter('vision_length_min_progress_ratio').value)
        min_progress_m = float(self.get_parameter('vision_length_min_progress_m').value)
        if progress < max(min_progress_m, nominal * min_ratio):
            self._vision_length_hit_count = 0
            return nominal, remaining_m, free_ratio, True, 'warmup'

        # odom 名义长度为主，视觉只做小范围修正
        vision_total = progress + max(0.0, remaining_m)
        max_shorten_ratio = float(self.get_parameter('vision_length_max_shorten_ratio').value)
        max_extend_ratio = float(self.get_parameter('vision_length_max_extend_ratio').value)
        max_extend_m = float(self.get_parameter('vision_length_max_extend_m').value)

        min_target = max(progress, nominal * (1.0 - max(0.0, min(0.5, max_shorten_ratio))))
        max_target = nominal + min(
            max(0.0, max_extend_m),
            nominal * max(0.0, max_extend_ratio),
        )
        # 视觉只在“更短/略长”方向轻推，避免远距 mask 把段拉长
        adjusted = nominal
        if vision_total < nominal:
            adjusted = max(min_target, vision_total)
        elif vision_total > nominal:
            # 仅允许很小延长
            adjusted = min(max_target, 0.7 * nominal + 0.3 * vision_total)

        stop_remaining = float(self.get_parameter('vision_length_stop_remaining_m').value)
        if remaining_m <= stop_remaining:
            adjusted = max(min_target, min(adjusted, progress + max(0.0, remaining_m)))

        self._vision_length_last_target = adjusted
        return adjusted, remaining_m, free_ratio, True, 'active'

    def _vision_move_should_finish(self, nominal_target: float, progress: float, dist_tol: float):
        """
        视觉辅助判断 move 段是否应结束。

        返回：
            (should_finish, target_distance, remaining_m, free_ratio, reason)
        """
        target, remaining_m, free_ratio, valid, reason = self._vision_adjusted_move_target(
            nominal_target, progress
        )
        if progress >= target - max(0.0, float(dist_tol)):
            # odom 到点：直接结束
            self._vision_length_hit_count = 0
            return True, target, remaining_m, free_ratio, f'odom_done:{reason}'

        if not valid or reason in ('disabled', 'invalid', 'warmup'):
            self._vision_length_hit_count = 0
            return False, target, remaining_m, free_ratio, reason

        stop_remaining = float(self.get_parameter('vision_length_stop_remaining_m').value)
        need_frames = max(1, int(self.get_parameter('vision_length_confirm_frames').value))
        min_ratio = float(self.get_parameter('vision_length_min_progress_ratio').value)
        min_progress_m = float(self.get_parameter('vision_length_min_progress_m').value)
        progress_ok = progress >= max(min_progress_m, float(nominal_target) * min_ratio)

        if progress_ok and remaining_m is not None and remaining_m <= stop_remaining:
            self._vision_length_hit_count += 1
        else:
            self._vision_length_hit_count = 0

        if self._vision_length_hit_count >= need_frames:
            return True, target, remaining_m, free_ratio, (
                f'vision_end hits={self._vision_length_hit_count}/{need_frames}'
            )
        return False, target, remaining_m, free_ratio, (
            f'tracking hits={self._vision_length_hit_count}/{need_frames}'
        )


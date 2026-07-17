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
        self.declare_parameter('vision_max_angular', 0.35)
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
        self.declare_parameter('vision_angular_kp', 1.25)
        self.declare_parameter('vision_angular_kd', 0.20)
        self.declare_parameter('vision_curvature_kp', 0.45)
        self.declare_parameter('vision_deadband', 0.035)
        self.declare_parameter('vision_primary_control', True)  # 视觉主 / IMU 兜底
        self.declare_parameter('vision_budget_disable_enabled', False)  # 默认不因预算永久闭嘴
        self.declare_parameter('vision_min_confidence', 0.30)
        self.declare_parameter('vision_primary_max_head_err_deg', 30.0)
        
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
        
        self.get_logger().info(
            f'[视觉] 中线主控启用 kp={self._vision_angular_kp:.2f} kd={self._vision_angular_kd:.2f} '
            f'kc={self._vision_curvature_kp:.2f} max_ω={self._vision_max_angular:.2f} '
            f'primary={self._vision_primary_control} budget_disable={self._vision_budget_disable_enabled} '
            f'crop=B{float(self.get_parameter("vision_crop_ratio").value):.0%}'
            f'+S{float(self.get_parameter("vision_crop_side_ratio").value):.0%} '
            f'length_corr={self._vision_length_enabled} '
            f'turn_assist={bool(self.get_parameter("vision_turn_assist_enabled").value)}'
        )
        self.get_logger().info(
            f'[视觉] HTTP 可视化: http://100.114.34.86:{http_port}/vision_latest.jpg (30 FPS)'
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
        拐弯视觉辅助：仅当
          1) 已转完名义角的 min_progress_ratio
          2) 剩余角误差在 angle_window 内
          3) 中心竖带连续充满 hold_sec
        才允许提前结束转弯。否则返回 False，继续走 IMU 目标角。
        """
        if not bool(self.get_parameter('vision_turn_assist_enabled').value):
            return False
        if self._vision_node is None:
            return False
        min_prog = float(self.get_parameter('vision_turn_assist_min_progress_ratio').value)
        ang_win = float(self.get_parameter('vision_turn_assist_angle_window_deg').value)
        hold_sec = max(0.05, float(self.get_parameter('vision_turn_assist_hold_sec').value))
        if progress_ratio < min_prog or abs_err_deg > ang_win:
            self._vision_turn_center_hold = 0.0
            self._vision_turn_center_last_t = None
            return False

        centered, center_ratio, valid = self._get_vision_center_status()
        now = time.time()
        if not (valid and centered):
            self._vision_turn_center_hold = 0.0
            self._vision_turn_center_last_t = None
            return False
        if self._vision_turn_center_last_t is None:
            self._vision_turn_center_last_t = now
        else:
            dt = max(0.0, now - self._vision_turn_center_last_t)
            if dt > 0.2:
                dt = 0.0
            self._vision_turn_center_hold += dt
            self._vision_turn_center_last_t = now
        if self._vision_turn_center_hold >= hold_sec:
            if hasattr(self, '_log_session'):
                self._log_session(
                    'TURN_VIS_ASSIST',
                    f'ready hold={self._vision_turn_center_hold:.2f}s '
                    f'prog={progress_ratio:.2f} err={abs_err_deg:.1f}° ctr={center_ratio:.2f}',
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

    def _vision_line_to_angular(self, error: float, curve: float = 0.0) -> float:
        """
        多行中线 PD + 曲率：
            error/curve 定义：负=赛道中心偏左，正=偏右
            控制：ω = -(Kp*e + Kd*ė + Kc*curve)
            正ω=左转，负ω=右转
        """
        kp = float(getattr(self, '_vision_angular_kp', self._vision_offset_kp))
        kd = float(getattr(self, '_vision_angular_kd', 0.0))
        kc = float(getattr(self, '_vision_curvature_kp', 0.0))
        deadband = float(getattr(self, '_vision_deadband', 0.0))
        e = float(error)
        if abs(e) < deadband:
            e = 0.0
        now = time.time()
        deriv = 0.0
        if self._vision_prev_error_t is not None:
            dt = max(1e-3, now - self._vision_prev_error_t)
            deriv = (e - self._vision_prev_error) / dt
        self._vision_prev_error = e
        self._vision_prev_error_t = now
        angular = -(kp * e + kd * deriv + kc * float(curve))
        return self.clamp(angular, self._vision_max_angular)

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
        if bool(line.get('valid', False)) and vision_age < vision_timeout:
            vision_error = float(line.get('error', 0.0))
            vision_curve = float(line.get('curve', 0.0))
            vision_conf = float(line.get('confidence', 0.0))
            vision_rows = int(line.get('valid_rows', 0))
            vision_centered = bool(line.get('centered', False))
            quality_ok = (vision_conf >= min_conf) and (vision_rows >= max(2, min_rows // 2))
            candidate = self._vision_line_to_angular(vision_error, vision_curve)
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
                if abs(vision_angular) < 1e-4 and imu_healthy and imu_corr_enabled:
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


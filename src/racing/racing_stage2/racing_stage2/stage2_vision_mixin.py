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
        self.declare_parameter('vision_offset_kp', 1.2)
        self.declare_parameter('vision_max_angular', 0.6)
        self.declare_parameter('vision_http_port', 8082)
        # mask 质心纠偏 / 中心竖带到位
        self.declare_parameter('vision_center_band', 0.12)
        self.declare_parameter('vision_center_occ_thresh', 0.40)
        self.declare_parameter('vision_centroid_bottom_ratio', 0.60)
        
        # 融合策略参数
        self.declare_parameter('imu_heading_deadzone_deg', 0.3)
        self.declare_parameter('imu_max_yaw_rate_deg_s', 600.0)
        self.declare_parameter('vision_timeout_sec', 0.5)
        self.declare_parameter('vision_offset_max_sec', 1.0)  # 单段视觉横向纠偏最长持续时间
        self.declare_parameter('fusion_weight_imu', 0.3)
        self.declare_parameter('fusion_weight_vision', 0.7)

        # 视觉纵向定长/修正参数
        self.declare_parameter('vision_length_correction_enabled', True)
        self.declare_parameter('vision_length_min_progress_ratio', 0.45)
        self.declare_parameter('vision_length_min_progress_m', 0.12)
        self.declare_parameter('vision_length_stop_remaining_m', 0.28)
        self.declare_parameter('vision_length_confirm_frames', 3)
        self.declare_parameter('vision_length_max_shorten_ratio', 0.45)
        self.declare_parameter('vision_length_max_extend_ratio', 0.25)
        self.declare_parameter('vision_length_max_extend_m', 0.35)
        self.declare_parameter('vision_range_near_m', 0.15)
        self.declare_parameter('vision_range_far_m', 2.50)
        self.declare_parameter('vision_range_center_band', 0.30)
        self.declare_parameter('vision_range_occ_thresh', 0.12)
        
        # 读取参数
        self._vision_enabled = bool(self.get_parameter('vision_offset_correction_enabled').value)
        self._vision_length_enabled = bool(
            self.get_parameter('vision_length_correction_enabled').value
        )
        self._vision_length_hit_count = 0
        self._vision_length_last_log_t = 0.0
        self._vision_length_last_target = None

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
            self.get_logger().info(
                '[视觉] 模块已禁用（offset/length correction 均关闭）'
            )
            return
        
        model_path = str(self.get_parameter('vision_model_path').value)
        conf = float(self.get_parameter('vision_conf_thres').value)
        iou = float(self.get_parameter('vision_iou_thres').value)
        crop = float(self.get_parameter('vision_crop_ratio').value)
        http_port = int(self.get_parameter('vision_http_port').value)
        
        self._vision_offset_kp = float(self.get_parameter('vision_offset_kp').value)
        self._vision_max_angular = float(self.get_parameter('vision_max_angular').value)
        
        # 滑动平均滤波器（防止过度修正）
        self._offset_history = []
        self._offset_filter_size = 5  # 取最近 5 帧平均

        # 视觉横向纠偏时长限制（防止持续纠偏过冲）
        self._vision_offset_max_sec = float(self.get_parameter('vision_offset_max_sec').value)
        self._vision_corr_elapsed = 0.0
        self._vision_corr_last_t = None
        self._vision_corr_budget_exhausted = False
        
        # 创建视觉节点（自动启动 HTTP 服务，保存图像到 /tmp/stage2_vision.jpg）
        self._vision_node = VisionLaneCentering(self, model_path, conf, iou, crop, http_port)
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
        
        self.get_logger().info(
            f'[视觉] 模块已启用 kp={self._vision_offset_kp:.2f} '
            f'max_ω={self._vision_max_angular:.2f} rad/s '
            f'max_t={self._vision_offset_max_sec:.2f}s '
            f'length_corr={self._vision_length_enabled}'
        )
        self.get_logger().info(
            f'[视觉] HTTP 可视化: http://100.114.34.86:{http_port}/vision_latest.jpg (30 FPS)'
        )
    
    def _reset_vision_offset_time_state(self) -> None:
        """新 move 段开始时重置视觉横向纠偏计时。"""
        self._vision_corr_elapsed = 0.0
        self._vision_corr_last_t = None
        self._vision_corr_budget_exhausted = False
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
                    f'[视觉] 横向纠偏达上限 {self._vision_offset_max_sec:.2f}s，'
                    f'本段停用视觉横向修正（elapsed={self._vision_corr_elapsed:.2f}s）'
                )
                if hasattr(self, '_log_session'):
                    self._log_session(
                        'VIS_TIME_LIMIT',
                        f'elapsed={self._vision_corr_elapsed:.2f}s '
                        f'max={self._vision_offset_max_sec:.2f}s | 本段视觉横向纠偏关闭',
                    )
            self._vision_corr_last_t = None
            return False
        return True

    def _vision_offset_to_angular(self, offset: float) -> float:
        """
        将视觉 offset 转换为角速度（带滑动平均滤波）。
        
        offset: [-1.0, +1.0]（mask 质心相对画面中心；中心竖带到位时为 0）
            -1.0 = 赛道质心在左侧 → 往左打（+ω，ROS 逆时针）
            +1.0 = 赛道质心在右侧 → 往右打（-ω，ROS 顺时针）
            0.0  = 中心竖带已被 mask 占住 / 已居中 → 不纠
        
        角速度方向（ROS REP-103）：
            正值 = 左转（逆时针）
            负值 = 右转（顺时针）
        """
        # 滑动平均滤波（防止单帧跳变）
        self._offset_history.append(offset)
        if len(self._offset_history) > self._offset_filter_size:
            self._offset_history.pop(0)
        
        # 使用滤波后的 offset
        filtered_offset = sum(self._offset_history) / len(self._offset_history)
        
        # 死区：offset < 0.02（2% 图像宽度）时不修正
        if abs(filtered_offset) < 0.02:
            return 0.0
        
        # 标准图像纠偏：目标偏右(offset>0) → 车偏左 → 右转(ω<0)
        # angular = -kp * offset
        angular = -self._vision_offset_kp * filtered_offset
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
直行段横向修正（IMU + 视觉融合：各 30%/70% 或自动降级）。
         
         融合策略：
             - 正常：IMU 30% + 视觉 70%（权重由 yaml 参数控制）
             - 视觉 ω≈0（死区内）：自动切 IMU 100%
             - 视觉失效：IMU 100%
             - IMU 异常：视觉 100%
             - 全失效：保持直行（ω=0）
             - 转弯段：禁止所有纠偏，返回 0.0
        
        返回：
            angular: float, 融合后的角速度 (rad/s)
        """
        # === 0. 转弯段禁止所有纠偏 ===
        if self.current_segment is not None and self.current_segment.get('type') == 'turn':
            return 0.0

        # === 1. 获取视觉输出 ===
        vision_angular = 0.0
        vision_available = False
        vision_offset = 0.0
        vision_age = float('inf')
        
        if self._vision_enabled and self._vision_node is not None:
            offset, timestamp, valid = self._vision_node.get_latest_offset()
            vision_age = time.time() - timestamp if timestamp > 0 else float('inf')
            vision_timeout = float(self.get_parameter('vision_timeout_sec').value)

            if bool(getattr(self, '_vision_corr_budget_exhausted', False)):
                # 本段视觉横向纠偏时长已用尽
                vision_available = False
                vision_angular = 0.0
                if valid and vision_age < vision_timeout:
                    vision_offset = offset
            elif valid and vision_age < vision_timeout:
                # 先算角速度，用于判断是否处于“有效纠偏中”（过死区）
                candidate_angular = self._vision_offset_to_angular(offset)
                vision_active = abs(candidate_angular) > 1e-6
                if self._vision_offset_time_allows(vision_active):
                    vision_available = True
                    vision_offset = offset
                    vision_angular = candidate_angular
                else:
                    # 刚达上限：本段停用视觉横向纠偏
                    vision_available = False
                    vision_offset = offset
                    vision_angular = 0.0
            else:
                # 视觉无效时停止累计纠偏时间
                self._vision_offset_time_allows(False)
        
        # === 2. 计算 IMU 输出 ===
        imu_angular = 0.0
        heading_error = 0.0
        
        if self.current_position is not None and self.segment_heading is not None:
            nav_yaw = self.navigation_yaw()
            if nav_yaw is not None:
                heading_error = self.angle_error(self.segment_heading, nav_yaw)
                # 死区从 1.0° 缩小到 0.3°（更敏感）
                deadzone = math.radians(float(self.get_parameter('imu_heading_deadzone_deg').value))
                if abs(heading_error) >= deadzone:
                    imu_angular = self.clamp(self.heading_kp * heading_error, self.max_angular_speed)
        
        # === 3. IMU 健康检测 ===
        imu_healthy = self._check_imu_health()
        
        # === 3b. 按纠偏开关过滤 ===
        fusion_enabled = bool(self.get_parameter('fusion_mode_enabled').value)

        if not fusion_enabled:
            # 总开关关闭：不启动任何纠偏
            imu_angular = 0.0
            vision_angular = 0.0
            weight_vision = 0.0
            weight_imu = 0.0
            mode = "FUSION_DISABLED"
        else:
            # 总开关开启，再按子开关过滤
            imu_corr_enabled = bool(self.get_parameter('imu_heading_correction_enabled').value)
            vis_corr_enabled = bool(self.get_parameter('vision_offset_correction_enabled').value)

            if not imu_corr_enabled:
                imu_angular = 0.0
            if not vis_corr_enabled:
                vision_angular = 0.0

            # === 4. 融合策略 ===
            if vision_available and imu_healthy and vis_corr_enabled and imu_corr_enabled:
                # 读权重
                weight_imu = float(self.get_parameter('fusion_weight_imu').value)
                weight_vision = float(self.get_parameter('fusion_weight_vision').value)
                # 视觉 ω≈0 时自动切 IMU 100%
                if abs(vision_angular) < 1e-6:
                    weight_imu = 1.0
                    weight_vision = 0.0
                    mode = "IMU_ONLY(VIS_ZERO)"
                else:
                    mode = "FUSION"
            elif vision_available and (not imu_healthy or not imu_corr_enabled) and vis_corr_enabled:
                weight_vision = 1.0
                weight_imu = 0.0
                mode = "VIS_ONLY" if not imu_healthy else "VIS_ONLY(IMU_DISABLED)"
            elif imu_healthy and imu_corr_enabled and (not vision_available or not vis_corr_enabled):
                weight_vision = 0.0
                weight_imu = 1.0
                mode = "IMU_ONLY" if not vision_available else "IMU_ONLY(VIS_DISABLED)"
            else:
                weight_vision = 0.0
                weight_imu = 0.0
                mode = "LOST"
        
        angular_fused = weight_imu * imu_angular + weight_vision * vision_angular
        
        # === 5. 日志记录（状态变化 + 1Hz 采样）===
        # 5.1 状态变化日志
        if self._last_fusion_mode != mode:
            self._last_fusion_mode = mode
            self._log_session(
                'FUSION_STATE',
                f'{mode} | vis_valid={vision_available} imu_healthy={imu_healthy} | '
                f'vis_age={vision_age:.2f}s'
            )
        
        # 5.2 定期采样日志（1Hz）
        current_time = time.time()
        if current_time - self._last_fusion_log_time >= 1.0:
            self._last_fusion_log_time = current_time
            offset_vis_str = f'{vision_offset:+.3f}' if vision_available else 'N/A'
            elapsed = float(getattr(self, '_vision_corr_elapsed', 0.0))
            max_t = float(getattr(self, '_vision_offset_max_sec', 0.0))
            exhausted = bool(getattr(self, '_vision_corr_budget_exhausted', False))
            self._log_session(
                'FUSION',
                f'{mode} | imu_ω={imu_angular:+.3f} vis_ω={vision_angular:+.3f} | '
                f'w=({weight_imu:.2f}/{weight_vision:.2f}) | '
                f'fused_ω={angular_fused:+.3f} | '
                f'err_imu={math.degrees(heading_error):+.1f}° '
                f'offset_vis={offset_vis_str} '
                f'vis_t={elapsed:.2f}/{max_t:.2f}s'
                f'{" EXHAUSTED" if exhausted else ""}'
            )
        
        return angular_fused
    
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

        # 视觉估计“当前已走 + 前方剩余”≈ 真实段长
        vision_total = progress + max(0.0, remaining_m)
        max_shorten_ratio = float(self.get_parameter('vision_length_max_shorten_ratio').value)
        max_extend_ratio = float(self.get_parameter('vision_length_max_extend_ratio').value)
        max_extend_m = float(self.get_parameter('vision_length_max_extend_m').value)

        min_target = max(progress, nominal * (1.0 - max(0.0, min(0.9, max_shorten_ratio))))
        max_target = nominal + min(
            max(0.0, max_extend_m),
            nominal * max(0.0, max_extend_ratio),
        )
        adjusted = max(min_target, min(max_target, vision_total))

        # 若前方几乎到头，允许更积极地提前结束（仍受 min_target 保护）
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


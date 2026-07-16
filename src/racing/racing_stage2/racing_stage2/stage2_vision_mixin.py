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
        
        # 融合策略参数
        self.declare_parameter('imu_heading_deadzone_deg', 0.3)
        self.declare_parameter('imu_max_yaw_rate_deg_s', 600.0)
        self.declare_parameter('vision_timeout_sec', 0.5)
        self.declare_parameter('fusion_weight_imu', 0.3)
        self.declare_parameter('fusion_weight_vision', 0.7)
        
        # 读取参数
        self._vision_enabled = bool(self.get_parameter('vision_offset_correction_enabled').value)

        # IMU 健康检测状态（无论 vision 是否启用都需初始化）
        self._last_imu_yaw = None
        self._last_imu_time = None
        self._imu_healthy = True

        # 融合状态跟踪
        self._last_fusion_mode = None
        self._last_fusion_log_time = 0.0
        self._last_valid_log = None

        if not self._vision_enabled:
            self._vision_node = None
            self.get_logger().info('[视觉] 模块已禁用（vision_offset_correction_enabled=False）')
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
        
        # 创建视觉节点（自动启动 HTTP 服务，保存图像到 /tmp/stage2_vision.jpg）
        self._vision_node = VisionLaneCentering(self, model_path, conf, iou, crop, http_port)
        
        self.get_logger().info(
            f'[视觉] 模块已启用 kp={self._vision_offset_kp:.2f} '
            f'max_ω={self._vision_max_angular:.2f} rad/s'
        )
        self.get_logger().info(
            f'[视觉] HTTP 可视化: http://100.114.34.86:{http_port}/vision_latest.jpg (30 FPS)'
        )
    
    def _vision_offset_to_angular(self, offset: float) -> float:
        """
        将视觉 offset 转换为角速度（带滑动平均滤波）。
        
        offset: [-1.0, +1.0]
            -1.0 = 完全偏左（目标在图像最左侧）→ 需要左转（负角速度）
            +1.0 = 完全偏右（目标在图像最右侧）→ 需要右转（正角速度）
        
        角速度方向：
            负值 = 左转（逆时针）
            正值 = 右转（顺时针）
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
        
        # 线性增益
        angular = -self._vision_offset_kp * filtered_offset  # 负号：偏左→负ω（左转）
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
            
            if valid and vision_age < vision_timeout:
                vision_available = True
                vision_offset = offset
                vision_angular = self._vision_offset_to_angular(offset)
        
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
            self._log_session(
                'FUSION',
                f'{mode} | imu_ω={imu_angular:+.3f} vis_ω={vision_angular:+.3f} | '
                f'w=({weight_imu:.2f}/{weight_vision:.2f}) | '
                f'fused_ω={angular_fused:+.3f} | '
                f'err_imu={math.degrees(heading_error):+.1f}° '
                f'offset_vis={offset_vis_str}'
            )
        
        return angular_fused
    
    def _get_vision_angular_for_avoider(self):
        """
        供 avoider 调用的视觉修正回调（用于 leg2 段）。
        
        返回：
            float: 角速度 (rad/s)，或 None（无有效检测）
        """
        if not self._vision_enabled or self._vision_node is None:
            return None
        offset, timestamp, valid = self._vision_node.get_latest_offset()
        if valid:
            angular = self._vision_offset_to_angular(offset)
            self._log_session('LEG2_VIS', 
                f'leg2 视觉修正 offset={offset:+.3f} ω={angular:+.3f} rad/s')
            return angular
        return None

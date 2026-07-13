#!/usr/bin/env python3
"""
direct_inertial_tester_vision.py — 视觉修正 mixin

为 DirectInertialTester 提供视觉车道居中功能：
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


class DirectInertialTesterVisionMixin:
    """
    视觉修正 mixin，供 DirectInertialTester 混入。
    """
    
    def _setup_vision_centering(self):
        """初始化视觉居中模块"""
        from racing_stage2_param_test.vision_lane_centering import VisionLaneCentering
        
        # 参数声明
        self.declare_parameter('vision_enabled', True)
        self.declare_parameter('vision_model_path', 
            '/home/sunrise/dev_ws/src/racing/racing_stage2_param_test/models/bset.bin')
        self.declare_parameter('vision_conf_thres', 0.25)
        self.declare_parameter('vision_iou_thres', 0.45)
        self.declare_parameter('vision_crop_ratio', 0.4)
        self.declare_parameter('vision_offset_kp', 1.2)
        self.declare_parameter('vision_max_angular', 0.6)
        
        # 读取参数
        self._vision_enabled = bool(self.get_parameter('vision_enabled').value)
        
        if not self._vision_enabled:
            self._vision_node = None
            self.get_logger().info('[视觉] 模块已禁用（vision_enabled=False）')
            return
        
        model_path = str(self.get_parameter('vision_model_path').value)
        conf = float(self.get_parameter('vision_conf_thres').value)
        iou = float(self.get_parameter('vision_iou_thres').value)
        crop = float(self.get_parameter('vision_crop_ratio').value)
        
        self._vision_offset_kp = float(self.get_parameter('vision_offset_kp').value)
        self._vision_max_angular = float(self.get_parameter('vision_max_angular').value)
        
        # 滑动平均滤波器（防止过度修正）
        self._offset_history = []
        self._offset_filter_size = 5  # 取最近 5 帧平均
        
        # 创建视觉节点（自动发布 /vision_debug Image 话题）
        self._vision_node = VisionLaneCentering(self, model_path, conf, iou, crop)
        
        self.get_logger().info(
            f'[视觉] 模块已启用 kp={self._vision_offset_kp:.2f} '
            f'max_ω={self._vision_max_angular:.2f} rad/s'
        )
        self.get_logger().info('[视觉] 可视化已发布到 /vision_debug（RViz2/rqt 订阅此话题）')
    
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
    
    def _compute_move_lateral_angular_with_vision(self) -> float:
        """
        直行段横向修正（视觉优先 + IMU 回退）。
        
        返回：
            angular: float, 角速度 (rad/s)
        """
        # 1. 尝试视觉修正
        if self._vision_enabled and self._vision_node is not None:
            offset, timestamp, valid = self._vision_node.get_latest_offset()
            if valid:
                angular = self._vision_offset_to_angular(offset)
                self._log_session('MOVE_VIS', 
                    f'视觉修正 offset={offset:+.3f} ω={angular:+.3f} rad/s')
                return angular
            else:
                # 视觉丢失（超时或无检测）
                import time
                age = time.time() - timestamp if timestamp > 0 else float('inf')
                self._log_session('MOVE_VIS_LOST', 
                    f'视觉丢失 age={age:.2f}s → 回退 IMU')
        
        # 2. 回退 IMU 航向控制（原逻辑）
        if self.current_position is None or self.segment_heading is None:
            return 0.0
        nav_yaw = self.navigation_yaw()
        if nav_yaw is None:
            return 0.0
        heading_error = self.angle_error(self.segment_heading, nav_yaw)
        # 死区：航向误差 < 1° 时不修正
        if abs(heading_error) < math.radians(1.0):
            return 0.0
        angular = self.clamp(self.heading_kp * heading_error, self.max_angular_speed)
        self._log_session('MOVE_IMU', 
            f'IMU 航向修正 err={math.degrees(heading_error):+.1f}° ω={angular:+.3f} rad/s')
        return angular
    
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

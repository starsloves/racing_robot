#!/usr/bin/env python3
"""
stage1_vision_mixin.py — Stage1 视觉修正 mixin

为 CompetitionController 提供视觉通道导航功能：
1. 初始化视觉模块（VisionCorridorDetector）
2. 提供视觉导航控制接口
3. 融合视觉与IMU的控制策略

要求父类提供：
    - self.get_parameter(name)
    - self.get_logger()
    - self.clamp(value, limit)
    - self.angle_error(target, current)
    - self.create_twist(linear, angular)
"""

import math
import time


class Stage1VisionMixin:
    """
    视觉导航 mixin，供 CompetitionController 混入
    """

    def _setup_vision_corridor(self):
        """初始化视觉通道导航模块"""
        # 默认禁用，避免初始化失败时影响主流程
        self._vision_corridor = None
        self._vision_corridor_enabled = False
        self._vision_corridor_active = False

        try:
            from racing_stage1.vision_corridor_detector import VisionCorridorDetector
        except ImportError as e:
            self.get_logger().warn(f'[Stage1视觉] 无法导入视觉模块: {e}')
            return

        # 参数声明（使用 try-except 避免参数已存在的错误）
        params_to_declare = {
            'vision_corridor_enabled': True,
            'vision_corridor_model_path': '/home/sunrise/dev_ws/src/racing/racing_stage2/models/bset.bin',
            'vision_corridor_conf_thres': 0.25,
            'vision_corridor_iou_thres': 0.45,
            'vision_corridor_crop_ratio': 0.4,
            'vision_corridor_crop_side_ratio': 0.20,
            'vision_corridor_http_port': 8081,
            'vision_corridor_lateral_kp': 1.2,
            'vision_corridor_heading_kp': 1.5,
            'vision_corridor_curvature_kp': 0.8,
            'vision_corridor_max_angular': 0.55,
            'vision_corridor_cruise_speed': 0.20,
            'vision_corridor_approach_speed': 0.12,
            'vision_corridor_entry_threshold_m': 0.25,
            'vision_corridor_timeout_sec': 0.5,
            'vision_corridor_min_confidence': 0.30,
            'vision_corridor_imu_fallback_enabled': True,
            'vision_corridor_sample_rows': 9,
            'vision_corridor_lookahead_ratio': 0.62,
            'vision_corridor_min_valid_rows': 5,
        }

        for param_name, default_value in params_to_declare.items():
            try:
                self.declare_parameter(param_name, default_value)
            except Exception:
                pass  # 参数已存在，跳过

        # 读取参数
        try:
            self._vision_corridor_enabled = bool(self.get_parameter('vision_corridor_enabled').value)
            self._vision_model_path = str(self.get_parameter('vision_corridor_model_path').value)
            self._vision_conf_thres = float(self.get_parameter('vision_corridor_conf_thres').value)
            self._vision_iou_thres = float(self.get_parameter('vision_corridor_iou_thres').value)
            self._vision_crop_ratio = float(self.get_parameter('vision_corridor_crop_ratio').value)
            self._vision_crop_side_ratio = float(self.get_parameter('vision_corridor_crop_side_ratio').value)
            self._vision_http_port = int(self.get_parameter('vision_corridor_http_port').value)
            self._channel_raw_path = str(self.get_parameter('channel_yolo_raw_path').value)
            self._channel_yolo_path = str(self.get_parameter('channel_yolo_preview_path').value)

            self._vision_lateral_kp = float(self.get_parameter('vision_corridor_lateral_kp').value)
            self._vision_heading_kp = float(self.get_parameter('vision_corridor_heading_kp').value)
            self._vision_curvature_kp = float(self.get_parameter('vision_corridor_curvature_kp').value)
            self._vision_max_angular = float(self.get_parameter('vision_corridor_max_angular').value)

            self._vision_cruise_speed = float(self.get_parameter('vision_corridor_cruise_speed').value)
            self._vision_approach_speed = float(self.get_parameter('vision_corridor_approach_speed').value)
            self._vision_entry_threshold_m = float(self.get_parameter('vision_corridor_entry_threshold_m').value)

            self._vision_timeout_sec = float(self.get_parameter('vision_corridor_timeout_sec').value)
            self._vision_min_confidence = float(self.get_parameter('vision_corridor_min_confidence').value)
            self._vision_imu_fallback = bool(self.get_parameter('vision_corridor_imu_fallback_enabled').value)
        except Exception as e:
            self.get_logger().error(f'[Stage1视觉] 参数读取失败: {e}')
            self._vision_corridor_enabled = False
            return

        if not self._vision_corridor_enabled:
            # 视觉分割不参与控制，但保留 detector 的 HTTP 服务，供
            # Stage1 通道 YOLO 原图/结果图在 8081 端口显示。
            self.get_logger().info('[Stage1视觉] 分割控制已禁用，仅保留8081监控服务')

        # 创建视觉检测器
        try:
            self._vision_corridor = VisionCorridorDetector(
                parent_node=self,
                model_path=self._vision_model_path,
                conf_thres=self._vision_conf_thres,
                iou_thres=self._vision_iou_thres,
                crop_ratio=self._vision_crop_ratio,
                http_port=self._vision_http_port,
                crop_side_ratio=self._vision_crop_side_ratio,
                channel_raw_path=self._channel_raw_path,
                channel_yolo_path=self._channel_yolo_path,
            )

            # 更新采样参数
            self._vision_corridor.update_params(
                sample_rows=int(self.get_parameter('vision_corridor_sample_rows').value),
                lookahead_ratio=float(self.get_parameter('vision_corridor_lookahead_ratio').value),
                min_valid_rows=int(self.get_parameter('vision_corridor_min_valid_rows').value),
            )

            self.get_logger().info('[Stage1视觉] 视觉导航模块初始化成功')
        except Exception as e:
            import traceback
            tb = traceback.format_exc()
            self.get_logger().error(f'[Stage1视觉] 初始化失败: {e}\n{tb}')
            self._vision_corridor = None
            self._vision_corridor_enabled = False

        # 状态变量
        self._vision_corridor_active = False
        self._vision_last_valid_time = 0.0
        self._vision_entry_confirmed_frames = 0
        self._vision_entry_confirm_threshold = 3

    def _enable_vision_corridor(self, enable: bool):
        """启用/禁用视觉通道导航"""
        if self._vision_corridor is None:
            return

        self._vision_corridor_active = enable
        self._vision_corridor.set_inference_active(enable)

        status = "启用" if enable else "禁用"
        self.get_logger().info(f'[Stage1视觉导航] 状态: {status}')

    def _get_vision_corridor_control(self):
        """
        获取视觉通道导航控制指令

        Returns:
            Twist: 控制指令
            dict: 状态信息 {valid, reached_entry, confidence, error_msg}
        """
        if self._vision_corridor is None or not self._vision_corridor_active:
            return None, {'valid': False, 'error_msg': '视觉模块未启用'}

        # 获取最新状态
        status = self._vision_corridor.get_latest_corridor_status()

        # 检查有效性
        if not status['valid']:
            # 视觉失效，检查超时
            now = time.time()
            if self._vision_last_valid_time > 0:
                timeout = now - self._vision_last_valid_time
                if timeout > self._vision_timeout_sec:
                    # 超时，降级到IMU直行
                    if self._vision_imu_fallback:
                        self.get_logger().warn(
                            f'[Stage1视觉导航] 视觉超时 {timeout:.2f}s，降级到IMU直行'
                        )
                        from geometry_msgs.msg import Twist
                        fallback_cmd = Twist()
                        fallback_cmd.linear.x = float(self._vision_approach_speed)
                        fallback_cmd.angular.z = 0.0
                        return fallback_cmd, {
                            'valid': True,
                            'reached_entry': False,
                            'confidence': 0.0,
                            'error_msg': 'IMU直行兜底'
                        }

            return None, {'valid': False, 'error_msg': '视觉数据无效'}

        # 更新有效时间
        self._vision_last_valid_time = time.time()

        # 检查置信度
        if status['confidence'] < self._vision_min_confidence:
            self.get_logger().warn(
                f'[Stage1视觉导航] 置信度过低: {status["confidence"]:.2f} < {self._vision_min_confidence}'
            )
            return None, {'valid': False, 'error_msg': '置信度不足'}

        # 提取控制误差
        lateral_error = status['lateral_error']
        heading_error_deg = status['heading_error_deg']
        curvature = status['curvature']
        remaining_m = status['remaining_m']
        boundary_safe = status['boundary_safe']

        # 检查边界安全
        if not boundary_safe:
            self.get_logger().warn('[Stage1视觉导航] 边界不安全，降低速度')

        # 计算角速度（融合横向误差 + 航向误差 + 曲率）
        heading_error_rad = math.radians(heading_error_deg)
        angular_vel = (
            self._vision_lateral_kp * lateral_error +
            self._vision_heading_kp * heading_error_rad +
            self._vision_curvature_kp * curvature
        )
        # 手动限幅，避免调用父类方法
        angular_vel = max(-self._vision_max_angular, min(self._vision_max_angular, angular_vel))

        # 计算线速度（根据剩余距离调整）
        if remaining_m is not None and remaining_m < self._vision_entry_threshold_m:
            # 接近入口，减速
            linear_vel = self._vision_approach_speed
            self._vision_entry_confirmed_frames += 1
            reached_entry = self._vision_entry_confirmed_frames >= self._vision_entry_confirm_threshold
        else:
            # 巡航速度
            linear_vel = self._vision_cruise_speed
            self._vision_entry_confirmed_frames = 0
            reached_entry = False

        # 边界不安全时降低速度
        if not boundary_safe:
            linear_vel *= 0.7

        # 生成控制指令
        from geometry_msgs.msg import Twist
        cmd = Twist()
        cmd.linear.x = float(linear_vel)
        cmd.angular.z = float(angular_vel)

        # 日志输出（每 20 帧一次）
        if hasattr(self, '_vision_log_counter'):
            self._vision_log_counter += 1
        else:
            self._vision_log_counter = 0

        if self._vision_log_counter % 20 == 0:
            self.get_logger().info(
                f'[Stage1视觉导航] Lat={lateral_error:+.3f} | '
                f'Head={heading_error_deg:+.1f}° | '
                f'Remain={remaining_m:.2f}m | '
                f'Conf={status["confidence"]:.2f} | '
                f'Cmd: v={linear_vel:.2f} ω={angular_vel:+.2f} | '
                f'Entry: {reached_entry}'
            )

        return cmd, {
            'valid': True,
            'reached_entry': reached_entry,
            'confidence': status['confidence'],
            'remaining_m': remaining_m,
            'lateral_error': lateral_error,
            'heading_error_deg': heading_error_deg,
            'boundary_safe': boundary_safe,
            'error_msg': None
        }

    def _check_vision_corridor_entry(self):
        """
        检查是否到达通道入口

        Returns:
            bool: 是否到达入口
        """
        if self._vision_corridor is None:
            return False

        status = self._vision_corridor.get_latest_corridor_status()
        if not status['valid']:
            return False

        remaining_m = status['remaining_m']
        if remaining_m is not None and remaining_m < self._vision_entry_threshold_m:
            self._vision_entry_confirmed_frames += 1
            if self._vision_entry_confirmed_frames >= self._vision_entry_confirm_threshold:
                self.get_logger().info(
                    f'[Stage1视觉导航] 到达通道入口 (剩余={remaining_m:.2f}m)'
                )
                return True

        return False

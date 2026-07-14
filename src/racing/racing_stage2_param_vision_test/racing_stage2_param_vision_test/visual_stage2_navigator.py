#!/usr/bin/env python3
"""
纯视觉Stage2导航器 - 实时跟踪赛道线
状态机：WAITING_QR → INITIAL_TURN → STRAIGHT → TURNING → STRAIGHT → ... → COMPLETED
直接集成 VisionLaneCentering 推理，无需 /vision_offset topic
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
from racing_common.racing_logger import RacingLogger
from .vision_lane_centering import VisionLaneCentering


class VisualStage2Navigator(Node):
    def __init__(self):
        super().__init__('visual_stage2_navigator')
        
        # 参数
        self.declare_parameter('direction', 'clockwise')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('look_ahead', 0.30)
        self.declare_parameter('v_straight', 0.50)
        self.declare_parameter('v_turn', 0.15)
        self.declare_parameter('w_turn', 0.50)
        self.declare_parameter('lateral_gain', 2.0)
        self.declare_parameter('lost_line_threshold', 0.5)
        self.declare_parameter('offset_center_threshold', 0.05)
        # 视觉模型参数
        self.declare_parameter('vision_model_path',
            '/home/sunrise/dev_ws/src/racing/racing_stage2_param_test/models/bset.bin')
        self.declare_parameter('vision_conf_thres', 0.25)
        self.declare_parameter('vision_iou_thres', 0.45)
        self.declare_parameter('vision_crop_ratio', 0.4)
        self.declare_parameter('vision_offset_kp', 1.2)
        self.declare_parameter('vision_max_angular', 0.6)
        self.declare_parameter('vision_http_port', 8080)
        self.declare_parameter('vision_timeout_sec', 0.5)
        
        # 订阅（仅保留里程计和扫码）
        self.create_subscription(Odometry, '/visual_odom', self.odom_callback, 10)
        self.create_subscription(String, '/competition_qr_task', self.qr_callback, 10)
        self.create_subscription(Bool, '/stage2_start_trigger', self.start_callback, 10)
        
        # 发布
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/stage2_visual_state', 10)
        
        # 初始化视觉推理
        model_path = self.get_parameter('vision_model_path').value
        conf = self.get_parameter('vision_conf_thres').value
        iou = self.get_parameter('vision_iou_thres').value
        crop = self.get_parameter('vision_crop_ratio').value
        http_port = self.get_parameter('vision_http_port').value
        self._vision_offset_kp = float(self.get_parameter('vision_offset_kp').value)
        self._vision_max_angular = float(self.get_parameter('vision_max_angular').value)
        self._vision_timeout = float(self.get_parameter('vision_timeout_sec').value)
        self._vision_node = VisionLaneCentering(self, model_path, conf, iou, crop, http_port)
        self._offset_history = []
        self._offset_filter_size = 5
        self._vision_enabled = True
        
        # 状态
        self.state = 'WAITING_QR'
        self.direction = self.get_parameter('direction').value
        self.turn_direction = 1 if self.direction == 'clockwise' else -1
        
        # 视觉数据
        self.vision_offset = 0.0
        self.vision_valid = False
        self.last_valid_time = None
        
        # 里程计
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        
        # 转弯状态
        self.turn_start_yaw = 0.0
        self.turn_count = 0
        self.straight_distance = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        
        # 完成判断
        self.start_x = 0.0
        self.start_y = 0.0
        self.completion_stable_time = None
        
        # 日志
        self.logger = RacingLogger(self, 'visual_stage2', 'visual_stage2.log', 'VisualStage2')
        
        # 控制循环 20Hz
        self.create_timer(0.05, self.control_loop)
        
        # 自动启动
        if self.get_parameter('auto_start').value:
            self._auto_start_timer = self.create_timer(0.5, self._auto_start_once)
        
        self.logger.config(f'direction={self.direction} auto_start={self.get_parameter("auto_start").value}')
        self.logger.startup(f'视觉导航器启动 - 方向: {self.direction}, 模型: {model_path}')
        self.get_logger().info(f'✓ 视觉导航器启动 - 方向: {self.direction}')
        self.get_logger().info(f'  日志: {self.logger.path}')
    
    def _update_vision(self):
        """从 VisionLaneCentering 获取最新偏移量"""
        offset, timestamp, valid = self._vision_node.get_latest_offset()
        if valid:
            age = time.time() - timestamp
            valid = age < self._vision_timeout
        if valid:
            self.vision_offset = offset
            self.vision_valid = True
            self.last_valid_time = time.time()
            if not hasattr(self, '_vision_received'):
                self.logger.info('VISION', f'首次成功检测: offset={offset:.3f}')
                self._vision_received = True
        else:
            self.vision_valid = False
    
    def _vision_offset_to_angular(self, offset):
        """偏移量转角速度，带滑动平均滤波"""
        self._offset_history.append(offset)
        if len(self._offset_history) > self._offset_filter_size:
            self._offset_history.pop(0)
        filtered = sum(self._offset_history) / len(self._offset_history)
        angular = filtered * self._vision_offset_kp
        angular = max(-self._vision_max_angular, min(self._vision_max_angular, angular))
        return angular
    
    def _auto_start_once(self):
        if self.state == 'WAITING_QR':
            self.state = 'INITIAL_TURN'
            self.turn_start_yaw = self.yaw
            self.start_x = self.x
            self.start_y = self.y
            self.logger.mission(f'自动启动 - 开始首次转弯')
            self.logger.info('ODOM', f'初始yaw={math.degrees(self.yaw):.1f}°, 位置=({self.x:.2f}, {self.y:.2f})')
            self.get_logger().info('自动启动 - 开始首次转弯')
        if hasattr(self, '_auto_start_timer'):
            self._auto_start_timer.cancel()
            self.destroy_timer(self._auto_start_timer)
    
    def qr_callback(self, msg):
        if self.state != 'WAITING_QR':
            return
        task = msg.data.lower()
        if 'clockwise' in task:
            self.direction = 'clockwise'
            self.turn_direction = 1
        elif 'counterclockwise' in task:
            self.direction = 'counterclockwise'
            self.turn_direction = -1
        else:
            self.get_logger().warn(f'未知方向: {task}')
            return
        self.get_logger().info(f'扫码方向: {self.direction}')
    
    def start_callback(self, msg):
        if msg.data and self.state == 'WAITING_QR':
            self.state = 'INITIAL_TURN'
            self.turn_start_yaw = self.yaw
            self.start_x = self.x
            self.start_y = self.y
            self.get_logger().info('开始首次转弯进入赛道')
    
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y**2 + quat.z**2))
        self.vx = msg.twist.twist.linear.x
        if not hasattr(self, '_odom_received'):
            self.logger.info('ODOM', f'首次接收里程计: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.1f}°')
            self._odom_received = True
    
    def control_loop(self):
        if not hasattr(self, '_control_loop_started'):
            self.logger.info('CONTROL', f'控制循环开始运行, state={self.state}')
            self._control_loop_started = True
        
        if self.state == 'WAITING_QR':
            self._publish_state()
            return
        
        # 每帧更新视觉
        self._update_vision()
        
        cmd = Twist()
        
        if self.state == 'INITIAL_TURN':
            cmd = self._initial_turn()
        elif self.state == 'STRAIGHT':
            cmd = self._straight_tracking()
        elif self.state == 'TURNING':
            cmd = self._corner_turning()
        elif self.state == 'COMPLETED':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        if not hasattr(self, '_last_cmd_log_time') or (time.time() - self._last_cmd_log_time) > 2.0:
            self.logger.telemetry('cmd_vel', f'state={self.state} v={cmd.linear.x:.3f} w={cmd.angular.z:.3f} pos=({self.x:.3f},{self.y:.3f}) yaw={math.degrees(self.yaw):.1f}°')
            self._last_cmd_log_time = time.time()
        
        self.cmd_pub.publish(cmd)
        self._publish_state()
        
        if self.state == 'STRAIGHT':
            dx = self.x - self.last_x
            dy = self.y - self.last_y
            self.straight_distance += math.sqrt(dx**2 + dy**2)
        self.last_x = self.x
        self.last_y = self.y
    
    def _initial_turn(self):
        cmd = Twist()
        target_angle = 85.0
        angle_turned = abs(math.degrees(self.yaw - self.turn_start_yaw))
        
        if angle_turned < target_angle - 5:
            cmd.linear.x = self.get_parameter('v_turn').value
            cmd.angular.z = self.turn_direction * self.get_parameter('w_turn').value
            if not hasattr(self, '_last_turn_log_time') or (time.time() - self._last_turn_log_time) > 1.0:
                self.logger.progress(f'初始转弯: {angle_turned:.1f}°/{target_angle}°, v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}, yaw={math.degrees(self.yaw):.1f}°')
                self.get_logger().info(f'转弯中: {angle_turned:.1f}° / {target_angle}°')
                self._last_turn_log_time = time.time()
        else:
            self.state = 'STRAIGHT'
            self.straight_distance = 0.0
            self.logger.feedback('首次转弯完成，进入直行跟踪')
            self.get_logger().info('首次转弯完成，进入直行跟踪')
        return cmd
    
    def _straight_tracking(self):
        cmd = Twist()
        
        if self._should_start_turning():
            self.state = 'TURNING'
            self.turn_start_yaw = self.yaw
            self.turn_count += 1
            self.get_logger().info(f'检测到拐角，开始转弯 #{self.turn_count}')
            return self._corner_turning()
        
        if self.vision_valid:
            angular = self._vision_offset_to_angular(self.vision_offset)
            cmd.linear.x = self.get_parameter('v_straight').value
            cmd.angular.z = -angular  # 负号：右偏→左转
        else:
            cmd.linear.x = self.get_parameter('v_straight').value * 0.8
            cmd.angular.z = 0.0
        
        if self.turn_count >= 4:
            self._check_completion()
        
        return cmd
    
    def _corner_turning(self):
        cmd = Twist()
        angle_turned = abs(math.degrees(self.yaw - self.turn_start_yaw))
        center_threshold = self.get_parameter('offset_center_threshold').value
        
        condition1 = (angle_turned >= 85.0) and self.vision_valid
        condition2 = self.vision_valid and (abs(self.vision_offset) < center_threshold)
        
        if condition1 or condition2:
            self.state = 'STRAIGHT'
            self.straight_distance = 0.0
            reason = "视觉回中" if condition2 else "角度达标"
            self.get_logger().info(f'转弯#{self.turn_count}完成 ({reason}), 角度: {angle_turned:.1f}°')
        else:
            cmd.linear.x = self.get_parameter('v_turn').value
            # 拐角方向与入口转弯相反：入口左转则拐角右转，入口右转则拐角左转
            cmd.angular.z = -self.turn_direction * self.get_parameter('w_turn').value
        
        return cmd
    
    def _should_start_turning(self):
        if self.straight_distance < 0.40:
            return False
        if self.last_valid_time is not None:
            lost_time = time.time() - self.last_valid_time
            if lost_time > self.get_parameter('lost_line_threshold').value:
                return True
        if self.vision_valid and abs(self.vision_offset) > 0.30:
            return True
        return False
    
    def _check_completion(self):
        dist_to_start = math.sqrt((self.x - self.start_x)**2 + (self.y - self.start_y)**2)
        lateral_error = abs(self.vision_offset) if self.vision_valid else 999.0
        
        if dist_to_start < 0.20 and lateral_error < 0.10:
            if self.completion_stable_time is None:
                self.completion_stable_time = time.time()
            elif time.time() - self.completion_stable_time > 1.0:
                self.state = 'COMPLETED'
                self.get_logger().info('Stage2完成！')
        else:
            self.completion_stable_time = None
    
    def _publish_state(self):
        msg = String()
        msg.data = f'{self.state}|turn:{self.turn_count}|dist:{self.straight_distance:.2f}'
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisualStage2Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class LidarAvoider(Node):
    def __init__(self):
        super().__init__('lidar_avoider')
        
        # --- 核心参数区 ---
        self.safe_distance = 0.5       # 触发避障的危险距离 (米)
        self.ignore_distance = 0.15    # 屏蔽底盘自身的干扰距离 (米)，小于15cm全忽略
        self.forward_speed = 0.2       # 正常直行速度
        self.avoid_speed = 0.1         # 避障时的降速
        self.avoid_angular = 0.8       # 打方向盘的力度
        self.scan_angle_range = 45.0   # 左右视野各 45 度
        # ------------------
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info("🚀 终极版雷达避障已启动！已开启底盘防干扰过滤。")

    def scan_callback(self, msg):
        min_dist = float('inf')
        danger_angle = 0.0
        
        for i, r in enumerate(msg.ranges):
            # 1. 过滤掉无效点和太近的底盘干扰物 (比如翘起的线缆)
            if math.isinf(r) or math.isnan(r) or r < self.ignore_distance:
                continue
                
            # 2. 严格对齐 0~360 度图纸坐标系
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad) % 360.0
            
            # 3. 划定正前方扇形视野 (左前: 0~45, 右前: 315~360)
            is_in_front = (angle_deg <= self.scan_angle_range) or (angle_deg >= (360.0 - self.scan_angle_range))
            
            if is_in_front:
                # 寻找视野内绝对最近的那一个点
                if r < min_dist:
                    min_dist = r
                    danger_angle = angle_deg

        twist = Twist()
        
        # --- 决策执行层 ---
        if min_dist < self.safe_distance:
            # 发现半米内的威胁！
            self.get_logger().warn(f"🧨 锁定最近威胁! 距离: {min_dist:.2f}m, 角度: {danger_angle:.1f}度")
            twist.linear.x = self.avoid_speed
            
            # 严格依据图纸判断左右：0度~180度 是车头左侧
            if danger_angle < 180.0:
                # 障碍物在左边，必须向右打方向盘 (角速度为负)
                twist.angular.z = -self.avoid_angular
                self.get_logger().info("👉 向右避让")
            else:
                # 障碍物在右边 (180~360度)，必须向左打方向盘 (角速度为正)
                twist.angular.z = self.avoid_angular
                self.get_logger().info("👈 向左避让")
        else:
            # 前方 0.5米 内安全，安心直行
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist()) # 退出前刹车
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
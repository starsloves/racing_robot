#!/usr/bin/env python3
"""
雷达测试节点 - 实时监测雷达数据

功能：
1. 订阅 /scan topic
2. 实时显示雷达发布频率
3. 显示前方、左侧、右侧最近障碍物距离和角度
4. 高亮显示异常数据（inf、nan）
5. 彩色输出，便于观察

使用方法：
    ros2 run racing_stage2_param_test lidar_test
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time


class LidarTestNode(Node):
    def __init__(self):
        super().__init__('lidar_test_node')
        
        # 订阅雷达
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # 统计变量
        self.msg_count = 0
        self.last_msg_time = None
        self.start_time = time.time()
        self.last_print_time = time.time()
        
        # 雷达参数（初始化为None，从第一条消息获取）
        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None
        self.range_min = None
        self.range_max = None
        
        self.get_logger().info('=== 雷达测试节点启动 ===')
        self.get_logger().info('订阅 topic: /scan')
        self.get_logger().info('等待雷达数据...\n')
    
    def scan_callback(self, msg: LaserScan):
        """雷达数据回调"""
        self.msg_count += 1
        now = time.time()
        
        # 第一次接收，记录雷达参数
        if self.angle_min is None:
            self.angle_min = msg.angle_min
            self.angle_max = msg.angle_max
            self.angle_increment = msg.angle_increment
            self.range_min = msg.range_min
            self.range_max = msg.range_max
            
            self.get_logger().info('=== 雷达参数 ===')
            self.get_logger().info(f'角度范围: {math.degrees(self.angle_min):.1f}° ~ {math.degrees(self.angle_max):.1f}°')
            self.get_logger().info(f'角度分辨率: {math.degrees(self.angle_increment):.2f}°')
            self.get_logger().info(f'距离范围: {self.range_min:.2f}m ~ {self.range_max:.2f}m')
            self.get_logger().info(f'数据点数: {len(msg.ranges)}\n')
        
        # 计算频率
        if self.last_msg_time is not None:
            dt = now - self.last_msg_time
            freq = 1.0 / dt if dt > 0 else 0.0
        else:
            freq = 0.0
        self.last_msg_time = now
        
        # 每 0.5 秒打印一次
        if now - self.last_print_time < 0.5:
            return
        self.last_print_time = now
        
        # 分析雷达数据
        front_dist, front_angle = self.find_nearest_in_cone(msg, -15.0, 15.0)
        left_dist, left_angle = self.find_nearest_in_cone(msg, 60.0, 120.0)
        right_dist, right_angle = self.find_nearest_in_cone(msg, -120.0, -60.0)
        
        # 统计有效数据点
        valid_count = sum(1 for r in msg.ranges if math.isfinite(r) and self.range_min < r < self.range_max)
        inf_count = sum(1 for r in msg.ranges if math.isinf(r))
        nan_count = sum(1 for r in msg.ranges if math.isnan(r))
        
        # 打印分隔线
        print('\n' + '=' * 80)
        
        # 打印统计信息
        elapsed = now - self.start_time
        avg_freq = self.msg_count / elapsed if elapsed > 0 else 0.0
        
        print(f'[统计] 接收: {self.msg_count} 条 | 频率: {freq:.1f} Hz (平均 {avg_freq:.1f} Hz)')
        print(f'[数据] 有效: {valid_count}/{len(msg.ranges)} | inf: {inf_count} | nan: {nan_count}')
        
        # 打印前方障碍物
        if math.isfinite(front_dist):
            color = '\033[92m' if front_dist > 1.0 else '\033[91m'  # 绿色/红色
            print(f'{color}[前方] 距离: {front_dist:.2f}m @ 角度: {front_angle:.1f}°\033[0m')
        else:
            print(f'\033[93m[前方] 无障碍物（inf）\033[0m')  # 黄色
        
        # 打印左侧障碍物
        if math.isfinite(left_dist):
            color = '\033[92m' if left_dist > 0.5 else '\033[91m'
            print(f'{color}[左侧] 距离: {left_dist:.2f}m @ 角度: {left_angle:.1f}°\033[0m')
        else:
            print(f'\033[93m[左侧] 无障碍物（inf）\033[0m')
        
        # 打印右侧障碍物
        if math.isfinite(right_dist):
            color = '\033[92m' if right_dist > 0.5 else '\033[91m'
            print(f'{color}[右侧] 距离: {right_dist:.2f}m @ 角度: {right_angle:.1f}°\033[0m')
        else:
            print(f'\033[93m[右侧] 无障碍物（inf）\033[0m')
        
        # 打印最近的3个障碍物
        nearest_obstacles = self.find_nearest_n_obstacles(msg, 3)
        if nearest_obstacles:
            print(f'\n[最近障碍物]')
            for i, (dist, angle) in enumerate(nearest_obstacles, 1):
                print(f'  #{i}: {dist:.2f}m @ {angle:.1f}°')
        
        print('=' * 80)
    
    def find_nearest_in_cone(self, msg: LaserScan, angle_min_deg, angle_max_deg):
        """在指定角度范围内查找最近障碍物
        
        Args:
            msg: LaserScan 消息
            angle_min_deg: 最小角度（度）
            angle_max_deg: 最大角度（度）
        
        Returns:
            (距离, 角度) 元组，如果没有返回 (inf, 0.0)
        """
        angle_min_rad = math.radians(angle_min_deg)
        angle_max_rad = math.radians(angle_max_deg)
        
        min_dist = float('inf')
        min_angle = 0.0
        
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            
            # 归一化角度到 [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))
            
            if angle_min_rad <= angle <= angle_max_rad:
                if math.isfinite(r) and self.range_min < r < self.range_max:
                    if r < min_dist:
                        min_dist = r
                        min_angle = math.degrees(angle)
        
        return min_dist, min_angle
    
    def find_nearest_n_obstacles(self, msg: LaserScan, n=3):
        """查找最近的 N 个障碍物
        
        Returns:
            [(距离, 角度), ...] 列表，按距离排序
        """
        obstacles = []
        
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and self.range_min < r < self.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                angle = math.atan2(math.sin(angle), math.cos(angle))
                obstacles.append((r, math.degrees(angle)))
        
        # 按距离排序，取前 N 个
        obstacles.sort(key=lambda x: x[0])
        return obstacles[:n]


def main(args=None):
    rclpy.init(args=args)
    node = LidarTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n\n=== 雷达测试结束 ===')
        elapsed = time.time() - node.start_time
        print(f'总运行时间: {elapsed:.1f}s')
        print(f'总接收消息: {node.msg_count} 条')
        if elapsed > 0:
            print(f'平均频率: {node.msg_count / elapsed:.1f} Hz')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

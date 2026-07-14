#!/usr/bin/env python3
"""
简化视觉里程计 - 启动时清零为相对坐标
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np


class SimpleVisualOdometry(Node):
    def __init__(self):
        super().__init__('simple_visual_odometry')
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_combined', self.odom_callback, 10)
        self.visual_odom_pub = self.create_publisher(
            Odometry, '/visual_odom', 10)
        
        self.origin_set = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0
        
        self.get_logger().info('✓ 视觉里程计（相对坐标）')
    
    def odom_callback(self, msg):
        if not self.origin_set:
            self.origin_x = msg.pose.pose.position.x
            self.origin_y = msg.pose.pose.position.y
            quat = msg.pose.pose.orientation
            self.origin_yaw = np.arctan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y**2 + quat.z**2))
            self.origin_set = True
            self.get_logger().info(f'原点: ({self.origin_x:.2f}, {self.origin_y:.2f})')
        
        x_abs = msg.pose.pose.position.x
        y_abs = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        yaw_abs = np.arctan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y**2 + quat.z**2))
        
        dx = x_abs - self.origin_x
        dy = y_abs - self.origin_y
        cos_o = np.cos(-self.origin_yaw)
        sin_o = np.sin(-self.origin_yaw)
        x_rel = dx * cos_o - dy * sin_o
        y_rel = dx * sin_o + dy * cos_o
        yaw_rel = yaw_abs - self.origin_yaw
        
        visual_odom = Odometry()
        visual_odom.header = msg.header
        visual_odom.header.frame_id = 'odom'
        visual_odom.child_frame_id = 'base_link'
        visual_odom.pose.pose.position.x = x_rel
        visual_odom.pose.pose.position.y = y_rel
        visual_odom.pose.pose.position.z = 0.0
        visual_odom.pose.pose.orientation.w = np.cos(yaw_rel / 2)
        visual_odom.pose.pose.orientation.x = 0.0
        visual_odom.pose.pose.orientation.y = 0.0
        visual_odom.pose.pose.orientation.z = np.sin(yaw_rel / 2)
        visual_odom.twist = msg.twist
        
        self.visual_odom_pub.publish(visual_odom)


def main():
    rclpy.init()
    node = SimpleVisualOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

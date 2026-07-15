#!/usr/bin/env python3
"""
data_recorder.py �?综合数据记录节点

订阅惯导 + 视觉 + 底盘所有关键话题，�?CSV 格式记录�?log/data_records/�?每次新运行覆盖上一次的 latest_record.csv，方便快速复盘�?
记录字段 (CSV �?�?  timestamp, system_time
  �?位姿 �?  wheel_x, wheel_y, wheel_yaw_deg, wheel_vx, wheel_vy, wheel_wz
  ekf_x, ekf_y, ekf_yaw_deg, ekf_vx, ekf_vy, ekf_wz
  imu_yaw_deg, imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz
  �?指令 �?  cmd_stage2_linear, cmd_stage2_angular
  cmd_lane_linear, cmd_lane_angular
  cmd_final_linear, cmd_final_angular
  �?激光雷�?�?  front_dist, front_angle_deg, left_dist, right_dist
  �?状�?�?  phase, mission_state, feedback
  �?避障 �?  avoid_state, avoid_leg_m, avoid_turn_deg
  �?导航�?�?"""

import os
import math
import csv
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32


def resolve_workspace_root():
    cwd = os.getcwd()
    if os.path.isdir(os.path.join(cwd, 'src', 'racing')):
        return cwd
    dev_ws = os.environ.get('DEV_WS', '')
    if dev_ws:
        return dev_ws
    return cwd


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_deg_from_quat(q):
    return math.degrees(normalize_angle(quaternion_to_yaw(q)))


CSV_HEADER = [
    'timestamp', 'system_time',
    'wheel_x', 'wheel_y', 'wheel_yaw_deg',
    'wheel_vx', 'wheel_vy', 'wheel_wz',
    'ekf_x', 'ekf_y', 'ekf_yaw_deg',
    'ekf_vx', 'ekf_vy', 'ekf_wz',
    'imu_yaw_deg', 'imu_ax', 'imu_ay', 'imu_az',
    'imu_gx', 'imu_gy', 'imu_gz',
    'cmd_stage2_linear', 'cmd_stage2_angular',
    'cmd_lane_linear', 'cmd_lane_angular',
    'cmd_final_linear', 'cmd_final_angular',
    'front_dist', 'front_angle_deg', 'left_dist', 'right_dist',
    'phase', 'mission_state', 'feedback',
    'avoid_state', 'avoid_leg_m', 'avoid_turn_deg',
]


class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        self.declare_parameter('record_rate_hz', 20.0)
        self.declare_parameter('record_subdir', 'data_records')
        self.declare_parameter('record_filename', 'latest_record.csv')

        rate_hz = max(1.0, float(self.get_parameter('record_rate_hz').value))
        subdir = str(self.get_parameter('record_subdir').value).strip() or 'data_records'
        filename = str(self.get_parameter('record_filename').value).strip() or 'latest_record.csv'

        root = resolve_workspace_root()
        log_dir = os.path.join(root, 'log', subdir)
        os.makedirs(log_dir, exist_ok=True)
        self._csv_path = os.path.join(log_dir, filename)

        self._csv_file = open(self._csv_path, 'w', newline='', encoding='utf-8')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(CSV_HEADER)
        self._csv_file.flush()

        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.get_logger().info(f'数据记录器启�?-> {self._csv_path}  {stamp}  @ {rate_hz}Hz')

        self.buf = {
            'wheel': {}, 'ekf': {}, 'imu': {},
            'cmd_stage2': None, 'cmd_lane': None, 'cmd_final': None,
            'scan': None, 'phase': 0, 'state': '', 'feedback': '',
        }
        self._last_scan_front = float('inf')
        self._last_scan_front_angle = 0.0
        self._last_scan_left = float('inf')
        self._last_scan_right = float('inf')

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Odometry, '/odom', self._cb_wheel_odom, 10)
        self.create_subscription(Odometry, '/odom_combined', self._cb_ekf_odom, 10)
        self.create_subscription(Imu, '/imu/data', self._cb_imu, sensor_qos)
        self.create_subscription(Twist, '/stage2_cmd_vel', self._cb_stage2_cmd, 10)
        self.create_subscription(Twist, '/lane_cmd_vel', self._cb_lane_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb_final_cmd, 10)
        self.create_subscription(LaserScan, '/scan', self._cb_scan, sensor_qos)
        self.create_subscription(Int32, '/competition_phase', self._cb_phase, 10)
        self.create_subscription(String, '/competition_feedback', self._cb_feedback, 10)
        self.create_subscription(String, '/stage2_state', self._cb_state, 10)

        self._row_count = 0
        self.create_timer(1.0 / rate_hz, self._tick)

    def _cb_wheel_odom(self, msg):
        self.buf['wheel'] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw_deg_from_quat(msg.pose.pose.orientation),
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'wz': msg.twist.twist.angular.z,
        }

    def _cb_ekf_odom(self, msg):
        self.buf['ekf'] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw_deg_from_quat(msg.pose.pose.orientation),
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'wz': msg.twist.twist.angular.z,
        }

    def _cb_imu(self, msg):
        self.buf['imu'] = {
            'yaw': yaw_deg_from_quat(msg.orientation),
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
            'gx': msg.angular_velocity.x,
            'gy': msg.angular_velocity.y,
            'gz': msg.angular_velocity.z,
        }

    def _cb_stage2_cmd(self, msg):
        self.buf['cmd_stage2'] = msg

    def _cb_lane_cmd(self, msg):
        self.buf['cmd_lane'] = msg

    def _cb_final_cmd(self, msg):
        self.buf['cmd_final'] = msg

    def _cb_scan(self, msg):
        self.buf['scan'] = msg
        n = len(msg.ranges)
        self._last_scan_front = float('inf')
        self._last_scan_front_angle = 0.0
        self._last_scan_left = float('inf')
        self._last_scan_right = float('inf')
        for i in range(n):
            d = msg.ranges[i]
            if math.isinf(d) or math.isnan(d) or d <= 0.0:
                continue
            a = math.degrees(msg.angle_min + i * msg.angle_increment)
            a = (a + 180.0) % 360.0 - 180.0
            if -18.0 <= a <= 18.0:
                if d < self._last_scan_front:
                    self._last_scan_front = d
                    self._last_scan_front_angle = a
            if 50.0 <= a <= 80.0:
                if d < self._last_scan_left:
                    self._last_scan_left = d
            if -80.0 <= a <= -50.0:
                if d < self._last_scan_right:
                    self._last_scan_right = d

    def _cb_phase(self, msg):
        self.buf['phase'] = int(msg.data)

    def _cb_feedback(self, msg):
        self.buf['feedback'] = str(msg.data)

    def _cb_state(self, msg):
        self.buf['state'] = str(msg.data)

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        tstr = f'{now:.6f}'
        sys_tstr = f'{time.time():.6f}'

        w = self.buf['wheel']
        e = self.buf['ekf']
        im = self.buf['imu']
        s2 = self.buf['cmd_stage2']
        ln = self.buf['cmd_lane']
        fn = self.buf['cmd_final']

        row = [
            tstr, sys_tstr,
            w.get('x', 0.0), w.get('y', 0.0), w.get('yaw', 0.0),
            w.get('vx', 0.0), w.get('vy', 0.0), w.get('wz', 0.0),
            e.get('x', 0.0), e.get('y', 0.0), e.get('yaw', 0.0),
            e.get('vx', 0.0), e.get('vy', 0.0), e.get('wz', 0.0),
            im.get('yaw', 0.0),
            im.get('ax', 0.0), im.get('ay', 0.0), im.get('az', 0.0),
            im.get('gx', 0.0), im.get('gy', 0.0), im.get('gz', 0.0),
            s2.linear.x if s2 else 0.0,
            s2.angular.z if s2 else 0.0,
            ln.linear.x if ln else 0.0,
            ln.angular.z if ln else 0.0,
            fn.linear.x if fn else 0.0,
            fn.angular.z if fn else 0.0,
            self._last_scan_front, self._last_scan_front_angle,
            self._last_scan_left, self._last_scan_right,
            self.buf['phase'], self.buf['state'], self.buf['feedback'],
            '', 0.0, 0.0,
        ]

        self._csv_writer.writerow(row)
        self._row_count += 1
        if self._row_count % 600 == 0:
            self._csv_file.flush()
            self.get_logger().info(f'已记�?{self._row_count} �?-> {self._csv_path}')

    def destroy_node(self):
        if self._row_count > 0:
            self._csv_file.flush()
        self._csv_file.close()
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.get_logger().info(
            f'数据记录结束 -> {self._csv_path}  �?{self._row_count} �? {stamp}'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

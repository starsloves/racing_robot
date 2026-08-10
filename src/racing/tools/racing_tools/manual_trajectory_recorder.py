#!/usr/bin/env python3
"""Record a manually pushed robot trajectory in the map frame.

This node is intentionally passive: it only reads TF and odometry and never
publishes ``/cmd_vel``.  The filtered JSON output is directly usable as the
Stage1 ``blind_scan_centerline_json`` parameter.
"""

import csv
import json
import math
import os
import time
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def resolve_workspace_root():
    cwd = os.getcwd()
    if os.path.isdir(os.path.join(cwd, 'src', 'racing')):
        return cwd
    configured = os.environ.get('DEV_WS', '')
    return configured if configured else cwd


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ManualTrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('manual_trajectory_recorder')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('record_rate_hz', 10.0)
        self.declare_parameter('sample_distance_m', 0.03)
        self.declare_parameter('output_dir', 'log/tools/manual_trajectories')
        self.declare_parameter('record_name', 'manual_trajectory')
        self.declare_parameter('tf_timeout_sec', 0.05)

        self.map_frame = str(self.get_parameter('map_frame').value).strip() or 'map'
        self.base_frame = str(self.get_parameter('base_frame').value).strip() or 'base_footprint'
        self.odom_topic = str(self.get_parameter('odom_topic').value).strip() or '/odom_combined'
        self.record_rate_hz = max(1.0, float(self.get_parameter('record_rate_hz').value))
        self.sample_distance_m = max(0.001, float(self.get_parameter('sample_distance_m').value))
        output_dir = str(self.get_parameter('output_dir').value).strip()
        self.record_name = str(self.get_parameter('record_name').value).strip() or 'manual_trajectory'
        self.tf_timeout_sec = max(0.0, float(self.get_parameter('tf_timeout_sec').value))

        if not os.path.isabs(output_dir):
            output_dir = os.path.join(resolve_workspace_root(), output_dir)
        self.output_dir = os.path.abspath(output_dir)
        os.makedirs(self.output_dir, exist_ok=True)

        self.csv_path = os.path.join(self.output_dir, f'{self.record_name}.csv')
        self.json_path = os.path.join(self.output_dir, f'{self.record_name}_centerline.json')
        self._csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'sample_index', 'steady_time_sec', 'system_time_utc',
            'map_x_m', 'map_y_m', 'map_yaw_deg',
            'odom_x_m', 'odom_y_m', 'odom_yaw_deg',
            'distance_from_previous_m',
        ])
        self._csv_file.flush()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_odom = None
        self.points = []
        self._last_recorded_xy = None
        self._last_tf_warn_time = 0.0
        self._last_status_time = time.monotonic()
        self._total_distance = 0.0
        self._sample_index = 0
        self._stopped = False

        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)
        self.create_timer(1.0 / self.record_rate_hz, self._record_callback)
        self.get_logger().info(
            '手推轨迹记录器已启动（只读，不发布 /cmd_vel）: '
            f'map={self.map_frame}, base={self.base_frame}, odom={self.odom_topic}'
        )
        self.get_logger().info(
            f'输出 CSV: {self.csv_path}; 停止时输出中心线 JSON: {self.json_path}'
        )
        self.get_logger().info(
            f'采样间距={self.sample_distance_m:.3f}m，手推完成后按 Ctrl+C 结束并写文件'
        )

    def _odom_callback(self, msg):
        self.latest_odom = msg

    def _lookup_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            now = time.monotonic()
            if now - self._last_tf_warn_time >= 3.0:
                self.get_logger().warning(
                    f'等待 {self.map_frame}->{self.base_frame} TF，暂未采样: {exc}'
                )
                self._last_tf_warn_time = now
            return None

    @staticmethod
    def _transform_xy_yaw(transform: TransformStamped):
        t = transform.transform.translation
        q = transform.transform.rotation
        return float(t.x), float(t.y), math.degrees(yaw_from_quaternion(q))

    def _record_callback(self):
        transform = self._lookup_transform()
        if transform is None:
            return

        map_x, map_y, map_yaw_deg = self._transform_xy_yaw(transform)
        if not math.isfinite(map_x) or not math.isfinite(map_y):
            return
        if self._last_recorded_xy is None:
            distance = 0.0
        else:
            distance = math.hypot(
                map_x - self._last_recorded_xy[0],
                map_y - self._last_recorded_xy[1],
            )
            if distance < self.sample_distance_m:
                return

        self._last_recorded_xy = (map_x, map_y)
        self._total_distance += distance
        self._sample_index += 1
        now_steady = self.get_clock().now().nanoseconds / 1e9
        now_utc = datetime.now(timezone.utc).isoformat(timespec='milliseconds')

        odom_x = odom_y = odom_yaw_deg = 0.0
        if self.latest_odom is not None:
            pose = self.latest_odom.pose.pose
            odom_x = float(pose.position.x)
            odom_y = float(pose.position.y)
            odom_yaw_deg = math.degrees(yaw_from_quaternion(pose.orientation))

        self.points.append({'x': round(map_x, 4), 'y': round(map_y, 4)})
        self._csv_writer.writerow([
            self._sample_index,
            f'{now_steady:.6f}',
            now_utc,
            f'{map_x:.4f}', f'{map_y:.4f}', f'{map_yaw_deg:.3f}',
            f'{odom_x:.4f}', f'{odom_y:.4f}', f'{odom_yaw_deg:.3f}',
            f'{distance:.4f}',
        ])
        self._csv_file.flush()

        now = time.monotonic()
        if now - self._last_status_time >= 2.0:
            self.get_logger().info(
                f'已采样 {len(self.points)} 点，轨迹长度={self._total_distance:.2f}m，'
                f'当前位置=({map_x:.2f},{map_y:.2f})'
            )
            self._last_status_time = now

    def _write_json(self):
        payload = self.points
        with open(self.json_path, 'w', encoding='utf-8') as json_file:
            json.dump(payload, json_file, ensure_ascii=True, separators=(',', ':'))
            json_file.write('\n')

    def destroy_node(self):
        if self._stopped:
            return
        self._stopped = True
        try:
            self._write_json()
            self._csv_file.flush()
            self._csv_file.close()
            # launch may have already invalidated the ROS context on Ctrl+C;
            # use plain stdout here so final file paths are always reported.
            print(
                f'[MANUAL_TRAJECTORY] 记录结束：{len(self.points)} 点，'
                f'长度={self._total_distance:.2f}m'
            )
            print(f'[MANUAL_TRAJECTORY] 中心线 JSON: {self.json_path}')
            print(f'[MANUAL_TRAJECTORY] 完整 CSV: {self.csv_path}')
        finally:
            try:
                super().destroy_node()
            except KeyboardInterrupt:
                # A launch shutdown can forward a second SIGINT while ROS is
                # destroying timers/subscriptions; the files are already safe.
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ManualTrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Initial scan-to-map localizer.

只做启动位姿估计，不发布 TF：
- 订阅 /map 和 /scan；
- 在给定初值附近搜索 (x, y, yaw)；
- 将雷达点投到地图，按到地图障碍边缘的距离打分；
- 打印最佳 (x,y,yaw,confidence)，并发布 RViz Marker 可视化。
"""

import math
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class MatchResult:
    x: float
    y: float
    yaw: float
    confidence: float
    mean_dist: float
    inlier_ratio: float
    points_used: int


class InitialScanMapLocalizer(Node):
    def __init__(self):
        super().__init__('initial_scan_map_localizer')
        self._declare_params()
        self._read_params()

        self.map_msg = None
        self.distance_map = None
        self.latest_scan = None
        self.last_result = None
        self.last_marker_scan_xy = None
        self.has_matched = False
        self.last_match_time = 0.0
        self.last_publish_time = 0.0
        self.last_wait_log_time = 0.0

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, map_qos)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data)
        durable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, durable_qos)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, durable_qos)
        self.create_timer(0.5, self._timer_cb)

        self.get_logger().info(
            f'initial_scan_map_localizer ready: map={self.map_topic}, scan={self.scan_topic}, '
            f'init=({self.initial_x:.2f},{self.initial_y:.2f},{math.degrees(self.initial_yaw):.1f}°), '
            f'search=±{self.search_xy_range:.2f}m ±{math.degrees(self.search_yaw_range):.1f}°'
        )

    def _declare_params(self):
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('marker_topic', '/initial_scan_map_localizer/markers')
        self.declare_parameter('pose_topic', '/initial_scan_map_localizer/pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('initial_x', 0.50)
        self.declare_parameter('initial_y', 0.20)
        self.declare_parameter('initial_yaw_deg', 10.0)
        self.declare_parameter('search_xy_range_m', 0.50)
        self.declare_parameter('search_yaw_range_deg', 20.0)
        self.declare_parameter('coarse_xy_step_m', 0.05)
        self.declare_parameter('coarse_yaw_step_deg', 2.0)
        self.declare_parameter('fine_xy_range_m', 0.08)
        self.declare_parameter('fine_yaw_range_deg', 3.0)
        self.declare_parameter('fine_xy_step_m', 0.01)
        self.declare_parameter('fine_yaw_step_deg', 0.5)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('unknown_is_occupied', False)
        self.declare_parameter('min_scan_range_m', 0.15)
        self.declare_parameter('max_scan_range_m', 2.50)
        self.declare_parameter('scan_downsample', 3)
        self.declare_parameter('max_distance_score_m', 0.35)
        self.declare_parameter('inlier_distance_m', 0.08)
        self.declare_parameter('confidence_distance_scale_m', 0.08)
        self.declare_parameter('match_once', True)
        self.declare_parameter('min_points_required', 20)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_republish_sec', 1.0)

    def _read_params(self):
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_yaw = math.radians(float(self.get_parameter('initial_yaw_deg').value))
        self.search_xy_range = float(self.get_parameter('search_xy_range_m').value)
        self.search_yaw_range = math.radians(float(self.get_parameter('search_yaw_range_deg').value))
        self.coarse_xy_step = float(self.get_parameter('coarse_xy_step_m').value)
        self.coarse_yaw_step = math.radians(float(self.get_parameter('coarse_yaw_step_deg').value))
        self.fine_xy_range = float(self.get_parameter('fine_xy_range_m').value)
        self.fine_yaw_range = math.radians(float(self.get_parameter('fine_yaw_range_deg').value))
        self.fine_xy_step = float(self.get_parameter('fine_xy_step_m').value)
        self.fine_yaw_step = math.radians(float(self.get_parameter('fine_yaw_step_deg').value))
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.unknown_is_occupied = bool(self.get_parameter('unknown_is_occupied').value)
        self.min_scan_range = float(self.get_parameter('min_scan_range_m').value)
        self.max_scan_range = float(self.get_parameter('max_scan_range_m').value)
        self.scan_downsample = max(1, int(self.get_parameter('scan_downsample').value))
        self.max_distance_score = float(self.get_parameter('max_distance_score_m').value)
        self.inlier_distance = float(self.get_parameter('inlier_distance_m').value)
        self.confidence_distance_scale = float(self.get_parameter('confidence_distance_scale_m').value)
        self.match_once = bool(self.get_parameter('match_once').value)
        self.min_points_required = int(self.get_parameter('min_points_required').value)
        self.publish_markers = bool(self.get_parameter('publish_markers').value)
        self.marker_republish_sec = max(0.2, float(self.get_parameter('marker_republish_sec').value))

    def _map_cb(self, msg):
        self.map_msg = msg
        self.distance_map = self._build_distance_map(msg)
        self.get_logger().info(
            f'map received: {msg.info.width}x{msg.info.height}, res={msg.info.resolution:.3f}m/cell'
        )

    def _scan_cb(self, msg):
        self.latest_scan = msg

    def _timer_cb(self):
        if self.match_once and self.has_matched:
            self._republish_last_result()
            return
        if self.map_msg is None or self.distance_map is None or self.latest_scan is None:
            self._log_waiting_inputs()
            return
        scan_xy = self._scan_to_points(self.latest_scan)
        if scan_xy.shape[0] < self.min_points_required:
            self.get_logger().warn(f'not enough scan points: {scan_xy.shape[0]}')
            return
        t0 = time.perf_counter()
        coarse = self._search(scan_xy, self.initial_x, self.initial_y, self.initial_yaw,
                              self.search_xy_range, self.search_yaw_range,
                              self.coarse_xy_step, self.coarse_yaw_step)
        fine = self._search(scan_xy, coarse.x, coarse.y, coarse.yaw,
                            self.fine_xy_range, self.fine_yaw_range,
                            self.fine_xy_step, self.fine_yaw_step)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        self.last_result = fine
        self.last_marker_scan_xy = scan_xy
        self.has_matched = True
        self.last_match_time = time.time()
        self.get_logger().info(
            '[LOCALIZE] best: '
            f'x={fine.x:.3f} y={fine.y:.3f} yaw={math.degrees(fine.yaw):.2f}deg '
            f'confidence={fine.confidence:.3f} mean_dist={fine.mean_dist:.3f}m '
            f'inliers={fine.inlier_ratio:.1%} pts={fine.points_used} time={elapsed_ms:.0f}ms'
        )
        self._publish_pose(fine)
        if self.publish_markers:
            self._publish_markers(fine, scan_xy)
        self.last_publish_time = time.time()

    def _republish_last_result(self):
        if self.last_result is None:
            return
        now = time.time()
        if now - self.last_publish_time < self.marker_republish_sec:
            return
        self._publish_pose(self.last_result)
        if self.publish_markers and self.last_marker_scan_xy is not None:
            self._publish_markers(self.last_result, self.last_marker_scan_xy)
        self.last_publish_time = now

    def _log_waiting_inputs(self):
        now = time.time()
        if now - self.last_wait_log_time < 2.0:
            return
        missing = []
        if self.map_msg is None or self.distance_map is None:
            missing.append(self.map_topic)
        if self.latest_scan is None:
            missing.append(self.scan_topic)
        self.get_logger().warn(f'waiting for input: {", ".join(missing)}')
        self.last_wait_log_time = now

    def _build_distance_map(self, msg):
        data = np.asarray(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        occupied = data >= self.occupied_threshold
        if self.unknown_is_occupied:
            occupied |= data < 0
        free_image = np.where(occupied, 0, 255).astype(np.uint8)
        dist_px = cv2.distanceTransform(free_image, cv2.DIST_L2, 5)
        return dist_px.astype(np.float32) * float(msg.info.resolution)

    def _scan_to_points(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        indices = np.arange(ranges.shape[0], dtype=np.float32)
        angles = scan.angle_min + indices * scan.angle_increment
        valid = np.isfinite(ranges)
        valid &= ranges >= self.min_scan_range
        valid &= ranges <= min(self.max_scan_range, scan.range_max)
        selected = np.where(valid)[0][::self.scan_downsample]
        if selected.size == 0:
            return np.zeros((0, 2), dtype=np.float32)
        selected_ranges = ranges[selected]
        selected_angles = angles[selected]
        xs = selected_ranges * np.cos(selected_angles)
        ys = selected_ranges * np.sin(selected_angles)
        return np.column_stack((xs, ys)).astype(np.float32)

    def _search(self, scan_xy, center_x, center_y, center_yaw, xy_range, yaw_range, xy_step, yaw_step):
        xs = self._axis_values(center_x, xy_range, xy_step)
        ys = self._axis_values(center_y, xy_range, xy_step)
        yaws = self._axis_values(center_yaw, yaw_range, yaw_step)
        best = None
        for yaw in yaws:
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            rot_x = cos_y * scan_xy[:, 0] - sin_y * scan_xy[:, 1]
            rot_y = sin_y * scan_xy[:, 0] + cos_y * scan_xy[:, 1]
            for x in xs:
                map_x = rot_x + x
                for y in ys:
                    result = self._score_candidate(map_x, rot_y + y, x, y, yaw)
                    if best is None or result.confidence > best.confidence:
                        best = result
        return best

    def _axis_values(self, center, radius, step):
        if radius <= 0.0 or step <= 0.0:
            return np.asarray([center], dtype=np.float32)
        count = int(round((2.0 * radius) / step)) + 1
        start = center - 0.5 * step * (count - 1)
        return (start + np.arange(count, dtype=np.float32) * step).astype(np.float32)

    def _score_candidate(self, map_x, map_y, x, y, yaw):
        info = self.map_msg.info
        gx = np.floor((map_x - info.origin.position.x) / info.resolution).astype(np.int32)
        gy = np.floor((map_y - info.origin.position.y) / info.resolution).astype(np.int32)
        valid = (gx >= 0) & (gy >= 0) & (gx < info.width) & (gy < info.height)
        distances = np.full(map_x.shape[0], self.max_distance_score, dtype=np.float32)
        if np.any(valid):
            sampled = self.distance_map[gy[valid], gx[valid]]
            distances[valid] = np.minimum(sampled, self.max_distance_score)
        mean_dist = float(np.mean(distances)) if distances.size else self.max_distance_score
        inlier_ratio = float(np.mean(distances <= self.inlier_distance)) if distances.size else 0.0
        distance_conf = math.exp(-mean_dist / max(1e-6, self.confidence_distance_scale))
        confidence = max(0.0, min(1.0, 0.65 * distance_conf + 0.35 * inlier_ratio))
        return MatchResult(float(x), float(y), self._normalize_angle(float(yaw)), confidence,
                           mean_dist, inlier_ratio, int(distances.size))

    def _publish_pose(self, result):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = result.x
        msg.pose.pose.position.y = result.y
        msg.pose.pose.orientation.z = math.sin(result.yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(result.yaw * 0.5)
        cov_xy = max(0.0025, (1.0 - result.confidence) * 0.25)
        cov_yaw = max(0.0003, (1.0 - result.confidence) * 0.10)
        msg.pose.covariance[0] = cov_xy
        msg.pose.covariance[7] = cov_xy
        msg.pose.covariance[35] = cov_yaw
        self.pose_pub.publish(msg)

    def _publish_markers(self, result, scan_xy):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        points_marker = Marker()
        points_marker.header.frame_id = self.map_frame
        points_marker.header.stamp = now
        points_marker.ns = 'initial_scan_match'
        points_marker.id = 1
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.025
        points_marker.scale.y = 0.025
        points_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.85)
        cos_y = math.cos(result.yaw)
        sin_y = math.sin(result.yaw)
        transformed_x = cos_y * scan_xy[:, 0] - sin_y * scan_xy[:, 1] + result.x
        transformed_y = sin_y * scan_xy[:, 0] + cos_y * scan_xy[:, 1] + result.y
        for px, py in zip(transformed_x, transformed_y):
            p = Point()
            p.x = float(px)
            p.y = float(py)
            points_marker.points.append(p)
        markers.markers.append(points_marker)

        arrow = Marker()
        arrow.header.frame_id = self.map_frame
        arrow.header.stamp = now
        arrow.ns = 'initial_scan_match'
        arrow.id = 2
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = 0.45
        arrow.scale.y = 0.05
        arrow.scale.z = 0.05
        arrow.color = ColorRGBA(r=1.0, g=0.2, b=0.0, a=0.9)
        arrow.pose.position.x = result.x
        arrow.pose.position.y = result.y
        arrow.pose.orientation.z = math.sin(result.yaw * 0.5)
        arrow.pose.orientation.w = math.cos(result.yaw * 0.5)
        markers.markers.append(arrow)

        text = Marker()
        text.header.frame_id = self.map_frame
        text.header.stamp = now
        text.ns = 'initial_scan_match'
        text.id = 3
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = result.x
        text.pose.position.y = result.y
        text.pose.position.z = 0.35
        text.scale.z = 0.16
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = (
            f'x={result.x:.2f} y={result.y:.2f}\n'
            f'yaw={math.degrees(result.yaw):.1f}° conf={result.confidence:.2f}'
        )
        markers.markers.append(text)
        self.marker_pub.publish(markers)

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = InitialScanMapLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

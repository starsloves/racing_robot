#!/usr/bin/env python3
"""Passive startup pose diagnostic from the rear map-origin corner.

This node is intentionally diagnostic-only.  It never publishes TF or motion
commands.  A stable rear L-corner is matched to the known map corner (0, 0),
then the corresponding static map->odom transform is reported for review.
"""

import json
import math
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class StartCornerPoseDiagnostic(Node):
    def __init__(self):
        super().__init__('start_corner_pose_diagnostic')
        self._declare_parameters()
        self._read_parameters()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_scan = None
        self.latest_imu = None
        self.latest_odom = None
        self._last_scan_stamp_ns = None
        self._imu_samples = deque(maxlen=self.imu_stable_samples)
        self._pose_samples = deque(maxlen=self.stable_frame_count)
        self._locked = False
        self._shutdown_requested = False
        self._last_status = None
        self._last_status_time = 0.0

        durable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.diagnostic_pub = self.create_publisher(String, self.diagnostic_topic, durable_qos)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, self.imu_topic, self._imu_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, qos_profile_sensor_data)
        self.create_subscription(String, self.qr_task_topic, self._qr_task_callback, durable_qos)
        self._timer = self.create_timer(0.1, self._process_latest_scan)

        self.get_logger().info(
            'START_CORNER_DIAG ready: passive only; rear corner=(%.2f,%.2f), yaw=[%.1f,%.1f]deg' % (
                self.corner_map_x,
                self.corner_map_y,
                math.degrees(self.yaw_min),
                math.degrees(self.yaw_max),
            )
        )

    def _declare_parameters(self):
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('qr_task_topic', 'competition_qr_task')
        self.declare_parameter('diagnostic_topic', 'start_corner_pose_diagnostic')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('corner_map_x', 0.0)
        self.declare_parameter('corner_map_y', 0.0)
        self.declare_parameter('yaw_min_deg', 0.0)
        self.declare_parameter('yaw_max_deg', 90.0)
        self.declare_parameter('rear_half_angle_deg', 100.0)
        self.declare_parameter('min_scan_range_m', 0.15)
        self.declare_parameter('max_scan_range_m', 2.50)
        self.declare_parameter('ransac_iterations', 80)
        self.declare_parameter('max_line_candidates', 8)
        self.declare_parameter('line_inlier_distance_m', 0.025)
        self.declare_parameter('line_fit_rms_max_m', 0.03)
        self.declare_parameter('line_min_span_m', 0.35)
        self.declare_parameter('line_min_points', 12)
        self.declare_parameter('orthogonal_tolerance_deg', 5.0)
        self.declare_parameter('stable_frame_count', 5)
        self.declare_parameter('stable_xy_spread_m', 0.03)
        self.declare_parameter('stable_yaw_spread_deg', 2.0)
        self.declare_parameter('stationary_speed_mps', 0.02)
        self.declare_parameter('imu_stable_samples', 8)
        self.declare_parameter('imu_stable_yaw_deg', 1.0)
        self.declare_parameter('configured_map_to_odom_x', 0.50)
        self.declare_parameter('configured_map_to_odom_y', 0.15)
        self.declare_parameter('configured_map_to_odom_yaw_deg', 20.0)
        self.declare_parameter('configured_initial_map_heading_deg', 20.0)

    def _read_parameters(self):
        get = lambda name: self.get_parameter(name).value
        self.scan_topic = str(get('scan_topic'))
        self.imu_topic = str(get('imu_topic'))
        self.odom_topic = str(get('odom_topic'))
        self.qr_task_topic = str(get('qr_task_topic'))
        self.diagnostic_topic = str(get('diagnostic_topic'))
        self.base_frame = str(get('base_frame'))
        self.corner_map_x = float(get('corner_map_x'))
        self.corner_map_y = float(get('corner_map_y'))
        self.yaw_min = math.radians(float(get('yaw_min_deg')))
        self.yaw_max = math.radians(float(get('yaw_max_deg')))
        self.rear_half_angle = math.radians(float(get('rear_half_angle_deg')))
        self.min_scan_range = float(get('min_scan_range_m'))
        self.max_scan_range = float(get('max_scan_range_m'))
        self.ransac_iterations = int(get('ransac_iterations'))
        self.max_line_candidates = max(2, int(get('max_line_candidates')))
        self.line_inlier_distance = float(get('line_inlier_distance_m'))
        self.line_fit_rms_max = float(get('line_fit_rms_max_m'))
        self.line_min_span = float(get('line_min_span_m'))
        self.line_min_points = int(get('line_min_points'))
        self.orthogonal_tolerance = math.radians(float(get('orthogonal_tolerance_deg')))
        self.stable_frame_count = int(get('stable_frame_count'))
        self.stable_xy_spread = float(get('stable_xy_spread_m'))
        self.stable_yaw_spread = math.radians(float(get('stable_yaw_spread_deg')))
        self.stationary_speed = float(get('stationary_speed_mps'))
        self.imu_stable_samples = int(get('imu_stable_samples'))
        self.imu_stable_yaw = math.radians(float(get('imu_stable_yaw_deg')))
        self.configured_transform = (
            float(get('configured_map_to_odom_x')),
            float(get('configured_map_to_odom_y')),
            math.radians(float(get('configured_map_to_odom_yaw_deg'))),
        )
        self.configured_heading = math.radians(float(get('configured_initial_map_heading_deg')))

    def _scan_callback(self, msg):
        self.latest_scan = msg

    def _imu_callback(self, msg):
        self.latest_imu = msg
        self._imu_samples.append(quaternion_to_yaw(msg.orientation))

    def _odom_callback(self, msg):
        self.latest_odom = msg

    def _qr_task_callback(self, msg):
        if not msg.data.strip() or self._shutdown_requested:
            return
        self.get_logger().info('START_CORNER_DIAG stopping after QR task was locked')
        self._shutdown_requested = True
        self.destroy_timer(self._timer)

    def _process_latest_scan(self):
        if self._locked or self.latest_scan is None:
            return
        stamp = self.latest_scan.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns == self._last_scan_stamp_ns:
            return
        self._last_scan_stamp_ns = stamp_ns

        if self.latest_imu is None or self.latest_odom is None:
            self._publish_status('waiting', 'scan/imu/odom')
            return
        if not self._is_stationary():
            self._pose_samples.clear()
            self._publish_status('rejected', 'vehicle_moving')
            return
        if not self._imu_is_stable():
            self._pose_samples.clear()
            self._publish_status('rejected', 'imu_unstable')
            return

        points = self._scan_points_in_base(self.latest_scan)
        if points is None:
            return
        result, reason = self._solve_corner_pose(points)
        if result is None:
            self._pose_samples.clear()
            self._publish_status('rejected', reason)
            return

        self._pose_samples.append(result)
        if len(self._pose_samples) < self.stable_frame_count:
            self._publish_status('sampling', f'{len(self._pose_samples)}/{self.stable_frame_count}')
            return
        if not self._pose_is_stable():
            self._pose_samples.clear()
            self._publish_status('rejected', 'pose_not_stable')
            return

        final = self._average_samples()
        self._locked = True
        self._publish_valid(final)

    def _is_stationary(self):
        twist = self.latest_odom.twist.twist.linear
        return math.hypot(float(twist.x), float(twist.y)) <= self.stationary_speed

    def _imu_is_stable(self):
        if len(self._imu_samples) < self.imu_stable_samples:
            return False
        reference = self._imu_samples[0]
        return max(abs(normalize_angle(yaw - reference)) for yaw in self._imu_samples) <= self.imu_stable_yaw

    def _scan_points_in_base(self, scan):
        source_frame = scan.header.frame_id or 'laser'
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, source_frame, Time(), timeout=Duration(seconds=0.0)
            )
        except TransformException:
            self._publish_status('waiting', f'tf:{self.base_frame}<-{source_frame}')
            return None

        ranges = np.asarray(scan.ranges, dtype=float)
        indices = np.arange(ranges.size, dtype=float)
        angles = float(scan.angle_min) + indices * float(scan.angle_increment)
        valid = np.isfinite(ranges)
        valid &= ranges >= self.min_scan_range
        valid &= ranges <= min(self.max_scan_range, float(scan.range_max))
        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])
        if xs.size < self.line_min_points * 2:
            self._publish_status('rejected', 'too_few_scan_points')
            return None

        yaw = quaternion_to_yaw(transform.transform.rotation)
        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)
        base_x = math.cos(yaw) * xs - math.sin(yaw) * ys + tx
        base_y = math.sin(yaw) * xs + math.cos(yaw) * ys + ty
        rear_angle = np.arctan2(base_y, base_x)
        rear_delta = np.arctan2(np.sin(rear_angle - math.pi), np.cos(rear_angle - math.pi))
        keep = np.abs(rear_delta) <= self.rear_half_angle
        return np.column_stack((base_x[keep], base_y[keep]))

    def _solve_corner_pose(self, points):
        if points.shape[0] < self.line_min_points * 2:
            return None, 'too_few_rear_points'
        # Do not make the second wall depend on whichever wall happened to
        # win one greedy RANSAC draw.  Extract several disjoint candidates and
        # test every pair; the known rear corner and 0..90 degree heading are
        # then used only as physical validation of a pair.
        candidates = self._line_candidates(points)
        if len(candidates) < 2:
            return None, f'wall_candidates={len(candidates)}'

        best = None
        saw_orthogonal = False
        saw_intersection = False
        saw_heading = False
        for first_index, first in enumerate(candidates[:-1]):
            for second in candidates[first_index + 1:]:
                angle_error = abs(
                    abs(normalize_angle(first['angle'] - second['angle'])) - math.pi / 2.0
                )
                if angle_error > self.orthogonal_tolerance:
                    continue
                saw_orthogonal = True
                corner = self._line_intersection(first, second)
                if corner is None:
                    continue
                saw_intersection = True
                first_ray = first['point'] - corner
                second_ray = second['point'] - corner
                if np.linalg.norm(first_ray) < 1e-4 or np.linalg.norm(second_ray) < 1e-4:
                    continue
                first_ray_angle = math.atan2(float(first_ray[1]), float(first_ray[0]))
                second_ray_angle = math.atan2(float(second_ray[1]), float(second_ray[0]))
                yaw = self._select_map_yaw(first_ray_angle, second_ray_angle)
                if yaw is None:
                    continue
                saw_heading = True
                # The physical startup corner is behind the base.  This gate
                # rejects unrelated interior right angles.
                if float(corner[0]) > 0.15:
                    continue
                score = (
                    int(first['count']) + int(second['count']),
                    min(float(first['span']), float(second['span'])),
                    -float(np.linalg.norm(corner)),
                )
                if best is None or score > best[0]:
                    best = (score, first, second, corner, angle_error, yaw)

        if best is None:
            if not saw_orthogonal:
                return None, f'no_orthogonal_pair candidates={len(candidates)}'
            if not saw_intersection:
                return None, 'wall_intersection'
            if not saw_heading:
                return None, 'yaw_out_of_range'
            return None, 'no_rear_corner_pair'

        _, first, second, corner, angle_error, yaw = best
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_x = self.corner_map_x - (cos_yaw * corner[0] - sin_yaw * corner[1])
        map_y = self.corner_map_y - (sin_yaw * corner[0] + cos_yaw * corner[1])
        odom_pos = self.latest_odom.pose.pose.position
        odom_x = float(odom_pos.x)
        odom_y = float(odom_pos.y)
        map_to_odom_x = map_x - (cos_yaw * odom_x - sin_yaw * odom_y)
        map_to_odom_y = map_y - (sin_yaw * odom_x + cos_yaw * odom_y)
        raw_imu_yaw = quaternion_to_yaw(self.latest_imu.orientation)
        return {
            'corner_base_x': float(corner[0]),
            'corner_base_y': float(corner[1]),
            'map_x': float(map_x),
            'map_y': float(map_y),
            'map_yaw': float(yaw),
            'map_to_odom_x': float(map_to_odom_x),
            'map_to_odom_y': float(map_to_odom_y),
            'map_to_odom_yaw': float(yaw),
            'imu_raw_yaw': float(raw_imu_yaw),
            'imu_map_heading_offset': float(normalize_angle(yaw - raw_imu_yaw)),
            'line1_rms': float(first['rms']),
            'line2_rms': float(second['rms']),
            'orthogonal_error': float(angle_error),
        }, None

    def _line_candidates(self, points):
        """Extract multiple non-overlapping line candidates from one scan."""
        remaining_indices = np.arange(points.shape[0], dtype=np.int32)
        candidates = []
        for _ in range(self.max_line_candidates):
            if remaining_indices.size < self.line_min_points:
                break
            line = self._ransac_line(points[remaining_indices])
            if line is None:
                break
            global_inliers = remaining_indices[line['inliers']]
            line['inliers'] = np.zeros(points.shape[0], dtype=bool)
            line['inliers'][global_inliers] = True
            candidates.append(line)
            remaining_indices = remaining_indices[~line['inliers'][remaining_indices]]
        return candidates

    def _ransac_line(self, points):
        if points.shape[0] < self.line_min_points:
            return None
        generator = np.random.default_rng()
        best_inliers = None
        best_count = 0
        for _ in range(self.ransac_iterations):
            pair = points[generator.choice(points.shape[0], size=2, replace=False)]
            direction = pair[1] - pair[0]
            length = float(np.linalg.norm(direction))
            if length < 1e-4:
                continue
            normal = np.array([-direction[1], direction[0]]) / length
            distances = np.abs((points - pair[0]) @ normal)
            inliers = distances <= self.line_inlier_distance
            count = int(np.count_nonzero(inliers))
            if count > best_count:
                best_count = count
                best_inliers = inliers
        if best_inliers is None or best_count < self.line_min_points:
            return None
        fit_points = points[best_inliers]
        centroid = np.mean(fit_points, axis=0)
        _, _, vh = np.linalg.svd(fit_points - centroid, full_matrices=False)
        direction = vh[0]
        normal = np.array([-direction[1], direction[0]])
        distances = np.abs((fit_points - centroid) @ normal)
        rms = float(math.sqrt(float(np.mean(distances ** 2))))
        projections = (fit_points - centroid) @ direction
        span = float(np.max(projections) - np.min(projections))
        if rms > self.line_fit_rms_max or span < self.line_min_span:
            return None
        all_distances = np.abs((points - centroid) @ normal)
        inliers = all_distances <= self.line_inlier_distance
        return {
            'point': centroid,
            'direction': direction,
            'normal': normal,
            'angle': math.atan2(float(direction[1]), float(direction[0])),
            'rms': rms,
            'span': span,
            'count': int(fit_points.shape[0]),
            'inliers': inliers,
        }

    @staticmethod
    def _line_intersection(first, second):
        matrix = np.column_stack((first['direction'], -second['direction']))
        determinant = float(np.linalg.det(matrix))
        if abs(determinant) < 1e-5:
            return None
        parameters = np.linalg.solve(matrix, second['point'] - first['point'])
        return first['point'] + parameters[0] * first['direction']

    def _select_map_yaw(self, first_ray_angle, second_ray_angle):
        candidates = []
        for first_axis, second_axis in ((0.0, math.pi / 2.0), (math.pi / 2.0, 0.0)):
            candidate = normalize_angle(first_axis - first_ray_angle)
            positive = candidate if candidate >= 0.0 else candidate + 2.0 * math.pi
            first_error = abs(normalize_angle(candidate + first_ray_angle - first_axis))
            second_error = abs(normalize_angle(candidate + second_ray_angle - second_axis))
            if (
                first_error <= self.orthogonal_tolerance
                and second_error <= self.orthogonal_tolerance
                and self.yaw_min - 1e-6 <= positive <= self.yaw_max + 1e-6
            ):
                candidates.append(positive)
        if not candidates:
            return None
        return min(candidates)

    def _pose_is_stable(self):
        samples = list(self._pose_samples)
        xs = [item['map_x'] for item in samples]
        ys = [item['map_y'] for item in samples]
        yaws = [item['map_yaw'] for item in samples]
        return (
            max(xs) - min(xs) <= self.stable_xy_spread
            and max(ys) - min(ys) <= self.stable_xy_spread
            and max(yaws) - min(yaws) <= self.stable_yaw_spread
        )

    def _average_samples(self):
        keys = self._pose_samples[0].keys()
        return {key: float(np.mean([item[key] for item in self._pose_samples])) for key in keys}

    def _publish_status(self, state, reason):
        now = self.get_clock().now().nanoseconds / 1e9
        if state == self._last_status and now - self._last_status_time < 2.0:
            return
        self._last_status = state
        self._last_status_time = now
        payload = {'state': state, 'reason': reason}
        self.diagnostic_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))
        self.get_logger().info(f'START_CORNER_DIAG state={state} reason={reason}')

    def _publish_valid(self, result):
        configured_x, configured_y, configured_yaw = self.configured_transform
        payload = {
            'state': 'valid',
            'corner_base_x': round(result['corner_base_x'], 4),
            'corner_base_y': round(result['corner_base_y'], 4),
            'map_x': round(result['map_x'], 4),
            'map_y': round(result['map_y'], 4),
            'map_yaw_deg': round(math.degrees(result['map_yaw']), 3),
            'map_to_odom_x': round(result['map_to_odom_x'], 4),
            'map_to_odom_y': round(result['map_to_odom_y'], 4),
            'map_to_odom_yaw_deg': round(math.degrees(result['map_to_odom_yaw']), 3),
            'imu_raw_yaw_deg': round(math.degrees(result['imu_raw_yaw']), 3),
            'imu_map_heading_offset_deg': round(math.degrees(result['imu_map_heading_offset']), 3),
            'delta_map_to_odom_x': round(result['map_to_odom_x'] - configured_x, 4),
            'delta_map_to_odom_y': round(result['map_to_odom_y'] - configured_y, 4),
            'delta_map_to_odom_yaw_deg': round(math.degrees(normalize_angle(result['map_to_odom_yaw'] - configured_yaw)), 3),
            'delta_initial_map_heading_deg': round(math.degrees(normalize_angle(result['map_yaw'] - self.configured_heading)), 3),
            'line_rms_m': round(max(result['line1_rms'], result['line2_rms']), 4),
            'orthogonal_error_deg': round(math.degrees(result['orthogonal_error']), 3),
        }
        self.diagnostic_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))
        self.get_logger().info(
            'START_CORNER_DIAG valid corner_base=(%.3f,%.3f) map=(%.3f,%.3f,%.1fdeg) '
            'suggested_map_to_odom=(%.3f,%.3f,%.1fdeg) imu_offset=%+.1fdeg' % (
                result['corner_base_x'], result['corner_base_y'], result['map_x'], result['map_y'],
                math.degrees(result['map_yaw']), result['map_to_odom_x'], result['map_to_odom_y'],
                math.degrees(result['map_to_odom_yaw']), math.degrees(result['imu_map_heading_offset']),
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = StartCornerPoseDiagnostic()
    try:
        # Do not call rclpy.shutdown() from a subscription callback.  It can
        # leave the executor process alive after QR has been locked.  The main
        # loop sees the request after the callback returns and closes cleanly.
        while rclpy.ok() and not node._shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

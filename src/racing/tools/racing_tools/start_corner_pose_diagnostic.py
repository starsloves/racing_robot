#!/usr/bin/env python3
"""Startup localization from the rear map-origin corner.

A stable rear L-corner is matched to the known map corner (0, 0).  The locked
map->odom transform is then published continuously for the competition.  This
node never publishes motion commands.
"""

import json
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


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
        self._gyro_samples = deque(maxlen=self.imu_stable_samples)
        self._pose_samples = deque(maxlen=self.stable_frame_count)
        self._locked = False
        self._map_to_odom = None
        self._locked_map_xy = None
        self._locked_odom_xy = None
        self._locked_map_yaw = None
        self._locked_raw_imu_yaw = None
        self._gyro_relative_yaw = 0.0
        self._locked_gyro_relative_yaw = 0.0
        self._last_imu_stamp = None
        self._live_heading = None
        self._last_live_scan_stamp_ns = None
        self._last_live_heading_time = 0.0
        self._live_heading_pending = deque(maxlen=self.live_heading_stable_frames)
        self._live_heading_last_estimate = None
        self._controller_map_pose = None
        self._controller_map_pose_odom_xy = None
        self._controller_map_pose_time = None
        self._last_status = None
        self._last_status_time = 0.0
        self._tf_broadcaster = TransformBroadcaster(self)

        durable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.diagnostic_pub = self.create_publisher(String, self.diagnostic_topic, durable_qos)
        self.heading_pub = self.create_publisher(Float64, self.live_heading_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, self.imu_topic, self._imu_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.map_pose_topic, self._map_pose_callback, 10)
        self._timer = self.create_timer(0.1, self._process_latest_scan)

        self.get_logger().info(
            'START_CORNER_LOCALIZER ready: rear corner=(%.2f,%.2f), yaw=[%.1f,%.1f]deg' % (
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
        self.declare_parameter('diagnostic_topic', 'start_corner_pose_diagnostic')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom_combined')
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
        self.declare_parameter('corner_min_distance_m', 0.35)
        self.declare_parameter('corner_max_distance_m', 1.20)
        self.declare_parameter('corner_lateral_abs_max_m', 0.25)
        self.declare_parameter('corner_ray_min_gap_m', -0.05)
        self.declare_parameter('orthogonal_tolerance_deg', 5.0)
        self.declare_parameter('stable_frame_count', 5)
        self.declare_parameter('stable_xy_spread_m', 0.03)
        self.declare_parameter('stable_yaw_spread_deg', 2.0)
        self.declare_parameter('stationary_speed_mps', 0.02)
        self.declare_parameter('stationary_angular_speed_rad_s', 0.04)
        self.declare_parameter('imu_stable_samples', 8)
        self.declare_parameter('imu_stable_gyro_rad_s', 0.08)
        self.declare_parameter('live_heading_topic', 'map_heading_lidar')
        self.declare_parameter('map_pose_topic', 'stage1_map_pose')
        self.declare_parameter('map_pose_timeout_sec', 1.0)
        # The wall corner is an absolute startup anchor only.  A moving scan
        # is not a reliable map-heading observation because occlusions and
        # wrong wall pairs can look orthogonal.
        self.declare_parameter('live_heading_enabled', False)
        self.declare_parameter('live_heading_period_sec', 0.20)
        self.declare_parameter('live_heading_max_correction_deg', 35.0)
        self.declare_parameter('live_heading_step_deg', 6.0)
        self.declare_parameter('live_heading_stable_frames', 4)
        self.declare_parameter('live_heading_consistency_deg', 3.0)
        self.declare_parameter('live_heading_stationary_speed_mps', 0.02)
        self.declare_parameter('live_heading_stationary_angular_speed_rad_s', 0.04)
        self.declare_parameter('live_heading_orthogonal_tolerance_deg', 10.0)
        self.declare_parameter('live_heading_min_span_m', 0.45)
        self.declare_parameter('live_heading_min_points', 14)
        self.declare_parameter('live_heading_max_line_rms_m', 0.035)

    def _read_parameters(self):
        get = lambda name: self.get_parameter(name).value
        self.scan_topic = str(get('scan_topic'))
        self.imu_topic = str(get('imu_topic'))
        self.odom_topic = str(get('odom_topic'))
        self.diagnostic_topic = str(get('diagnostic_topic'))
        self.map_frame = str(get('map_frame'))
        self.base_frame = str(get('base_frame'))
        self.odom_frame = str(get('odom_frame'))
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
        self.corner_min_distance = max(0.05, float(get('corner_min_distance_m')))
        self.corner_max_distance = max(self.corner_min_distance, float(get('corner_max_distance_m')))
        self.corner_lateral_abs_max = max(0.05, float(get('corner_lateral_abs_max_m')))
        self.corner_ray_min_gap = float(get('corner_ray_min_gap_m'))
        self.orthogonal_tolerance = math.radians(float(get('orthogonal_tolerance_deg')))
        self.stable_frame_count = int(get('stable_frame_count'))
        self.stable_xy_spread = float(get('stable_xy_spread_m'))
        self.stable_yaw_spread = math.radians(float(get('stable_yaw_spread_deg')))
        self.stationary_speed = float(get('stationary_speed_mps'))
        self.stationary_angular_speed = max(0.005, float(get('stationary_angular_speed_rad_s')))
        self.imu_stable_samples = int(get('imu_stable_samples'))
        self.imu_stable_gyro = max(0.01, float(get('imu_stable_gyro_rad_s')))
        self.live_heading_topic = str(get('live_heading_topic'))
        self.map_pose_topic = str(get('map_pose_topic'))
        self.map_pose_timeout = max(0.10, float(get('map_pose_timeout_sec')))
        self.live_heading_enabled = bool(get('live_heading_enabled'))
        self.live_heading_period = max(0.05, float(get('live_heading_period_sec')))
        self.live_heading_max_correction = math.radians(max(1.0, float(get('live_heading_max_correction_deg'))))
        self.live_heading_step = math.radians(max(0.2, float(get('live_heading_step_deg'))))
        self.live_heading_stable_frames = max(2, int(get('live_heading_stable_frames')))
        self.live_heading_consistency = math.radians(
            max(0.5, float(get('live_heading_consistency_deg')))
        )
        self.live_heading_stationary_speed = max(
            0.0, float(get('live_heading_stationary_speed_mps'))
        )
        self.live_heading_stationary_angular_speed = max(
            0.005, float(get('live_heading_stationary_angular_speed_rad_s'))
        )
        self.live_heading_orthogonal_tolerance = math.radians(
            max(3.0, float(get('live_heading_orthogonal_tolerance_deg')))
        )
        self.live_heading_min_span = max(0.20, float(get('live_heading_min_span_m')))
        self.live_heading_min_points = max(8, int(get('live_heading_min_points')))
        self.live_heading_max_line_rms = max(0.01, float(get('live_heading_max_line_rms_m')))

    def _scan_callback(self, msg):
        self.latest_scan = msg

    def _imu_callback(self, msg):
        self.latest_imu = msg
        self._gyro_samples.append(float(msg.angular_velocity.z))
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self._last_imu_stamp is None:
            self._last_imu_stamp = stamp
        else:
            dt = stamp - self._last_imu_stamp
            if 1e-4 <= dt <= 0.25:
                self._gyro_relative_yaw += float(msg.angular_velocity.z) * dt
            self._last_imu_stamp = stamp

    def _odom_callback(self, msg):
        self.latest_odom = msg

    def _map_pose_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            return
        self._controller_map_pose = (
            float(msg.pose.position.x), float(msg.pose.position.y)
        )
        if self.latest_odom is not None:
            odom = self.latest_odom.pose.pose.position
            self._controller_map_pose_odom_xy = (float(odom.x), float(odom.y))
        self._controller_map_pose_time = self.get_clock().now().nanoseconds / 1e9
        self._publish_tf()

    def _process_latest_scan(self):
        if self.latest_scan is None:
            return
        if self._locked:
            now = self.get_clock().now().nanoseconds / 1e9
            stamp_ns = int(self.latest_scan.header.stamp.sec) * 1_000_000_000 + int(self.latest_scan.header.stamp.nanosec)
            if (self.live_heading_enabled and stamp_ns != self._last_live_scan_stamp_ns
                    and now - self._last_live_heading_time >= self.live_heading_period):
                self._last_live_scan_stamp_ns = stamp_ns
                if self._is_stationary_for_heading():
                    points = self._scan_points_in_base(self.latest_scan)
                    if points is not None:
                        self._update_live_heading(points, now)
                else:
                    # During motion the scan contains changing occlusions and
                    # dynamic objects.  Never let a guessed wall pair rewrite
                    # the absolute map heading while the chassis is moving.
                    self._live_heading_pending.clear()
                    self._live_heading_last_estimate = None
            self._publish_tf()
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
        self._map_to_odom = (
            final['map_to_odom_x'], final['map_to_odom_y'], final['map_to_odom_yaw']
        )
        self._locked_map_xy = (final['map_x'], final['map_y'])
        self._locked_odom_xy = (final['odom_x'], final['odom_y'])
        self._locked_map_yaw = final['map_yaw']
        self._locked_raw_imu_yaw = final['imu_raw_yaw']
        self._locked_gyro_relative_yaw = self._gyro_relative_yaw
        self._live_heading = final['map_yaw']
        self.heading_pub.publish(Float64(data=float(self._live_heading)))
        self._publish_valid(final)
        self._publish_tf()

    @staticmethod
    def _orientation_error(first, second):
        """Smallest error between unoriented line angles (period pi)."""
        return abs((first - second + math.pi * 0.5) % math.pi - math.pi * 0.5)

    @staticmethod
    def _nearest_heading_equivalent(angle, reference):
        """Resolve the 180-degree wall ambiguity using the IMU prior."""
        candidates = [angle + index * math.pi for index in range(-2, 3)]
        return min(candidates, key=lambda item: abs(normalize_angle(item - reference)))

    def _estimate_live_heading(self, points, prior):
        if points is None or points.shape[0] < self.live_heading_min_points * 2:
            return None
        candidates = self._line_candidates(points)
        candidates = [item for item in candidates
                      if item['count'] >= self.live_heading_min_points
                      and item['span'] >= self.live_heading_min_span
                      and item['rms'] <= self.live_heading_max_line_rms]
        if len(candidates) < 2:
            return None
        best = None
        axes = ((0.0, math.pi / 2.0), (math.pi / 2.0, 0.0))
        for first_index, first in enumerate(candidates[:-1]):
            for second in candidates[first_index + 1:]:
                observed_error = self._orientation_error(first['angle'], second['angle'])
                if abs(observed_error - math.pi / 2.0) > self.live_heading_orthogonal_tolerance:
                    continue
                for first_axis, second_axis in axes:
                    first_heading = first_axis - first['angle']
                    second_heading = second_axis - second['angle']
                    if self._orientation_error(first_heading, second_heading) > self.live_heading_orthogonal_tolerance:
                        continue
                    heading = self._nearest_heading_equivalent(first_heading, prior)
                    second_equivalent = self._nearest_heading_equivalent(second_heading, heading)
                    residual = abs(normalize_angle(heading - second_equivalent))
                    if residual > self.live_heading_orthogonal_tolerance:
                        continue
                    prior_error = abs(normalize_angle(heading - prior))
                    if prior_error > self.live_heading_max_correction:
                        continue
                    support = int(first['count']) + int(second['count'])
                    span = min(float(first['span']), float(second['span']))
                    score = support + 8.0 * span - 12.0 * prior_error
                    if best is None or score > best[0]:
                        best = (score, heading)
        return None if best is None else normalize_angle(best[1])

    def _update_live_heading(self, points, now):
        if self._live_heading is None:
            return
        # The last accepted lidar heading is the only valid prior here.  A
        # stationary gyro bias must not move the pair-selection prior and make
        # a different orthogonal wall pair look preferable.
        prior = self._live_heading
        estimate = self._estimate_live_heading(points, prior)
        self._last_live_heading_time = now
        if estimate is None:
            self._live_heading_pending.clear()
            self._live_heading_last_estimate = None
            return
        if (self._live_heading_last_estimate is None or
                abs(normalize_angle(estimate - self._live_heading_last_estimate))
                > self.live_heading_consistency):
            self._live_heading_pending.clear()
        self._live_heading_pending.append(estimate)
        self._live_heading_last_estimate = estimate
        if len(self._live_heading_pending) < self.live_heading_stable_frames:
            return
        stable_estimate = math.atan2(
            sum(math.sin(item) for item in self._live_heading_pending),
            sum(math.cos(item) for item in self._live_heading_pending),
        )
        delta = normalize_angle(stable_estimate - self._live_heading)
        if abs(delta) > self.live_heading_max_correction:
            self._live_heading_pending.clear()
            self._live_heading_last_estimate = None
            return
        if abs(delta) < math.radians(0.1):
            self._live_heading_pending.clear()
            return
        delta = max(-self.live_heading_step, min(self.live_heading_step, delta))
        self._live_heading = normalize_angle(self._live_heading + delta)
        self._live_heading_pending.clear()
        self.heading_pub.publish(Float64(data=float(self._live_heading)))
        if self._locked_map_xy is not None and self._locked_odom_xy is not None and self.latest_odom is not None:
            odom = self.latest_odom.pose.pose.position
            dx = float(odom.x) - self._locked_odom_xy[0]
            dy = float(odom.y) - self._locked_odom_xy[1]
            start_x, start_y = self._locked_map_xy
            map_base_x = start_x + math.cos(self._live_heading) * dx - math.sin(self._live_heading) * dy
            map_base_y = start_y + math.sin(self._live_heading) * dx + math.cos(self._live_heading) * dy
            self._map_to_odom = (
                map_base_x - (math.cos(self._live_heading) * float(odom.x)
                              - math.sin(self._live_heading) * float(odom.y)),
                map_base_y - (math.sin(self._live_heading) * float(odom.x)
                              + math.cos(self._live_heading) * float(odom.y)),
                self._live_heading,
            )

    def _is_stationary_for_heading(self):
        if self.latest_odom is None:
            return False
        twist = self.latest_odom.twist.twist
        return (
            math.hypot(float(twist.linear.x), float(twist.linear.y))
            <= self.live_heading_stationary_speed
            and abs(float(twist.angular.z)) <= self.live_heading_stationary_angular_speed
        )

    def _is_stationary(self):
        twist = self.latest_odom.twist.twist.linear
        return (
            math.hypot(float(twist.x), float(twist.y)) <= self.stationary_speed
            and abs(float(self.latest_odom.twist.twist.angular.z)) <= self.stationary_angular_speed
        )

    def _imu_is_stable(self):
        if len(self._gyro_samples) < self.imu_stable_samples:
            return False
        reference = sum(self._gyro_samples) / len(self._gyro_samples)
        return max(abs(rate - reference) for rate in self._gyro_samples) <= self.imu_stable_gyro

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
        fallback_best = None
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
                # The physical startup corner is a known rear corner close to
                # the chassis centreline.  These gates reject the recurring
                # short interior right-angle false match.
                corner_distance = float(np.linalg.norm(corner))
                if (
                    float(corner[0]) > -0.15
                    or corner_distance < self.corner_min_distance
                    or corner_distance > self.corner_max_distance
                    or abs(float(corner[1])) > self.corner_lateral_abs_max
                ):
                    continue
                fallback_score = (
                    min(float(first['span']), float(second['span'])),
                    int(first['count']) + int(second['count']),
                    -float(np.linalg.norm(corner)),
                )
                fallback_candidate = (
                    fallback_score, first, second, corner, angle_error, yaw
                )
                if fallback_best is None or fallback_score > fallback_best[0]:
                    fallback_best = fallback_candidate
                # The map-origin walls must begin at the inferred corner and
                # extend away from it.  If the intersection lies in the
                # middle of either fitted segment, this is an interior
                # orthogonal object rather than the rear map corner.
                ray_gaps = []
                for line in (first, second):
                    projection = abs(float(np.dot(line['point'] - corner, line['direction'])))
                    ray_gaps.append(projection - 0.5 * float(line['span']))
                if min(ray_gaps) < self.corner_ray_min_gap:
                    continue
                score = (
                    -sum(abs(gap) for gap in ray_gaps),
                    min(float(first['span']), float(second['span'])),
                    int(first['count']) + int(second['count']),
                    -float(np.linalg.norm(corner)),
                )
                if best is None or score > best[0]:
                    best = (score, first, second, corner, angle_error, yaw)

        used_ray_gate_fallback = False
        if best is None and fallback_best is not None:
            # The finite scan segment can be shortened or extrapolated by
            # RANSAC at the real wall ends.  The ray test is therefore a pair
            # preference, never a reason to deadlock startup localization.
            best = fallback_best
            used_ray_gate_fallback = True

        if best is None:
            if not saw_orthogonal:
                return None, f'no_orthogonal_pair candidates={len(candidates)}'
            if not saw_intersection:
                return None, 'wall_intersection'
            if not saw_heading:
                return None, 'yaw_out_of_range'
            return None, 'no_rear_corner_pair'

        _, first, second, corner, angle_error, yaw = best
        ray_gaps = []
        for line in (first, second):
            projection = abs(float(np.dot(line['point'] - corner, line['direction'])))
            ray_gaps.append(projection - 0.5 * float(line['span']))
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
            'odom_x': float(odom_x),
            'odom_y': float(odom_y),
            'imu_raw_yaw': float(raw_imu_yaw),
            'imu_map_heading_offset': float(normalize_angle(yaw - raw_imu_yaw)),
            'line1_rms': float(first['rms']),
            'line2_rms': float(second['rms']),
            'orthogonal_error': float(angle_error),
            'corner_ray_gap_min': float(min(ray_gaps)),
            'corner_ray_gap_max': float(max(ray_gaps)),
            'corner_ray_gate_fallback': bool(used_ray_gate_fallback),
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

    def _publish_tf(self):
        if self._map_to_odom is None:
            return
        x, y, yaw = self._map_to_odom
        if (self._locked_map_yaw is not None and self._controller_map_pose is not None
                and self.latest_odom is not None):
            odom = self.latest_odom.pose.pose.position
            yaw = self._locked_map_yaw
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            map_x, map_y = self._controller_map_pose
            if (self._controller_map_pose_odom_xy is not None
                    and self._controller_map_pose_time is not None
                    and self.get_clock().now().nanoseconds / 1e9
                    - self._controller_map_pose_time > self.map_pose_timeout):
                dx = float(odom.x) - self._controller_map_pose_odom_xy[0]
                dy = float(odom.y) - self._controller_map_pose_odom_xy[1]
                map_x += cos_yaw * dx - sin_yaw * dy
                map_y += sin_yaw * dx + cos_yaw * dy
            x = map_x - (
                cos_yaw * float(odom.x) - sin_yaw * float(odom.y)
            )
            y = map_y - (
                sin_yaw * float(odom.x) + cos_yaw * float(odom.y)
            )
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(yaw * 0.5)
        transform.transform.rotation.w = math.cos(yaw * 0.5)
        self._tf_broadcaster.sendTransform(transform)

    def _publish_valid(self, result):
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
            'odom_x': round(result['odom_x'], 4),
            'odom_y': round(result['odom_y'], 4),
            'imu_raw_yaw_deg': round(math.degrees(result['imu_raw_yaw']), 3),
            'imu_map_heading_offset_deg': round(math.degrees(result['imu_map_heading_offset']), 3),
            'line_rms_m': round(max(result['line1_rms'], result['line2_rms']), 4),
            'orthogonal_error_deg': round(math.degrees(result['orthogonal_error']), 3),
            'corner_ray_gap_min_m': round(result['corner_ray_gap_min'], 4),
            'corner_ray_gap_max_m': round(result['corner_ray_gap_max'], 4),
            'corner_ray_gate_fallback': bool(result['corner_ray_gate_fallback']),
        }
        self.diagnostic_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))
        self.get_logger().info(
            'START_CORNER_LOCALIZER valid corner_base=(%.3f,%.3f) map=(%.3f,%.3f,%.1fdeg) '
            'map_to_odom=(%.3f,%.3f,%.1fdeg) imu_offset=%+.1fdeg '
            'ray_gap_min=%.3fm ray_gate_fallback=%s' % (
                result['corner_base_x'], result['corner_base_y'], result['map_x'], result['map_y'],
                math.degrees(result['map_yaw']), result['map_to_odom_x'], result['map_to_odom_y'],
                math.degrees(result['map_to_odom_yaw']), math.degrees(result['imu_map_heading_offset']),
                result['corner_ray_gap_min'],
                result['corner_ray_gate_fallback'],
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = StartCornerPoseDiagnostic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

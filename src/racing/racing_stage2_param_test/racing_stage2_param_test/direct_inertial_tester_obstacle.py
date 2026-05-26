"""Obstacle perception and debug helpers for the direct inertial tester.

This module keeps scan parsing, obstacle-circle generation, marker publishing,
and debug logging out of the main mission controller file.
"""

import math
import os

from visualization_msgs.msg import Marker, MarkerArray


class DirectInertialTesterObstacleMixin:
    """Mixin for scan-derived obstacle circles and debug output."""

    def log_console_and_debug(self, level, message):
        if level == 'WARN':
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)
        self.write_debug_log(level, message)

    def log_debug_info(self, message):
        self.log_console_and_debug('INFO', message)

    def log_debug_warning(self, message):
        self.log_console_and_debug('WARN', message)

    def format_distance(self, value):
        if not math.isfinite(value):
            return 'inf'
        return f'{value:.2f}'

    def format_yaw_deg(self, yaw):
        if yaw is None or not math.isfinite(yaw):
            return 'nan'
        return f'{math.degrees(self.normalize_angle(yaw)):.1f}'

    def format_position_xy(self):
        if self.current_position is None:
            return '(nan,nan)'
        if isinstance(self.current_position, tuple) and len(self.current_position) >= 2:
            return f'({self.current_position[0]:.2f},{self.current_position[1]:.2f})'
        if hasattr(self.current_position, 'x') and hasattr(self.current_position, 'y'):
            return f'({self.current_position.x:.2f},{self.current_position.y:.2f})'
        return '(invalid,invalid)'

    def is_turn_detour_segment(self):
        """True when the active mission segment is a turn (no DWA on turn legs)."""
        return self.current_segment is not None and self.current_segment.get('type') == 'turn'

    def detour_scan_summary(self):
        return (
            f'front={self.format_distance(self.front_obstacle_distance)}m@{self.front_obstacle_angle_deg:.1f}deg，'
            f'left={self.format_distance(self.left_clearance_distance)}m，'
            f'right={self.format_distance(self.right_clearance_distance)}m，'
            f'{self.format_obstacle_circle_summary()}'
        )

    def format_obstacle_circle_summary(self, circle=None):
        circle = self.active_obstacle_circle if circle is None else circle
        if circle is None:
            return 'circle=none'
        centerline_radius = self.obstacle_circle_centerline_clearance_radius(circle)
        return (
            f'circle=({circle["center_x"]:.2f},{circle["center_y"]:.2f})'
            f'/r={circle["radius"]:.2f}m'
            f'/clear={self.format_distance(centerline_radius)}m'
            f'/closest_x={circle["closest_x"]:.2f}m'
        )

    def sector_closest_obstacle(self, scan_msg, min_angle_deg, max_angle_deg):
        min_distance = float('inf')
        min_angle = 0.0
        for index, distance in enumerate(scan_msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance <= 0.0:
                continue

            angle_deg = math.degrees(scan_msg.angle_min + index * scan_msg.angle_increment)
            angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
            if angle_deg < min_angle_deg or angle_deg > max_angle_deg:
                continue

            if distance < min_distance:
                min_distance = distance
                min_angle = angle_deg

        return min_distance, min_angle

    def point_distance_xy(self, point_a, point_b):
        return math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])

    def cluster_scan_points(self, scan_msg):
        clusters = []
        current_cluster = []
        previous_point = None

        for index, distance in enumerate(scan_msg.ranges):
            if (
                math.isnan(distance)
                or math.isinf(distance)
                or distance < self.obstacle_circle_min_range_m
                or distance > self.obstacle_circle_max_range_m
            ):
                if current_cluster:
                    clusters.append(current_cluster)
                    current_cluster = []
                previous_point = None
                continue

            angle = scan_msg.angle_min + index * scan_msg.angle_increment
            point = (
                distance * math.cos(angle),
                distance * math.sin(angle),
                distance,
            )
            if (
                previous_point is None
                or self.point_distance_xy(previous_point, point)
                <= self.obstacle_circle_cluster_distance_threshold
            ):
                current_cluster.append(point)
            else:
                if current_cluster:
                    clusters.append(current_cluster)
                current_cluster = [point]
            previous_point = point

        if current_cluster:
            clusters.append(current_cluster)

        return clusters

    def scenario_static_obstacle_circle(self):
        """Offline/标定：把场景静态锥桶合成与激光簇一致的圆，避免段首已贴近时簇检测来不及。"""
        static = getattr(self, 'scenario_static_obstacles_world', [])
        target = str(getattr(self, 'scenario_obstacle_segment', '') or '').strip()
        if not static or not target:
            return None
        segment = self.current_segment or {}
        if segment.get('type') != 'move' or segment.get('description', '') != target:
            return None
        sx, sy, sr = static[0]
        if self.current_position is None or self.current_yaw is None:
            return None
        if not self.obstacle_on_active_segment_path(sx, sy, sr):
            return None
        along_obs = self.progress_along_segment_m((float(sx), float(sy)))
        if along_obs is not None:
            progress = self.projected_distance()
            if progress > float(along_obs) + float(sr) + 0.10:
                return None
        wx, wy = float(sx), float(sy)
        rx, ry = float(self.current_position[0]), float(self.current_position[1])
        dx = wx - rx
        dy = wy - ry
        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)
        cx = cos_y * dx + sin_y * dy
        cy = -sin_y * dx + cos_y * dy
        radius = float(sr)
        nearest = math.hypot(dx, dy) - radius
        return {
            'center_x': cx,
            'center_y': cy,
            'radius': radius,
            'span': radius * 2.0,
            'span_x': radius * 2.0,
            'span_y': radius * 2.0,
            'nearest_distance': max(0.01, nearest),
            'angle_deg': math.degrees(math.atan2(cy, max(cx, 1e-6))),
            'closest_x': max(0.01, cx - radius),
            'farthest_x': cx + radius,
            'lateral_min': cy - radius,
            'lateral_max': cy + radius,
            'scenario_static': True,
        }

    def build_obstacle_circles(self, clusters):
        circles = []
        for cluster in clusters:
            if len(cluster) < self.obstacle_circle_min_cluster_points:
                continue

            xs = [point[0] for point in cluster]
            ys = [point[1] for point in cluster]
            span_x = max(xs) - min(xs)
            span_y = max(ys) - min(ys)
            span = self.point_distance_xy(cluster[0], cluster[-1])
            if span > self.obstacle_circle_max_cluster_span_m:
                continue

            center_x = sum(xs) / len(cluster)
            center_y = sum(ys) / len(cluster)
            radius = max(
                self.point_distance_xy((center_x, center_y), point)
                for point in cluster
            )
            radius = max(
                self.obstacle_circle_min_radius_m,
                radius + self.obstacle_circle_padding_m,
            )
            if span_x > 0.45 and span_x > span_y * 1.6:
                radius = min(
                    radius,
                    max(
                        self.obstacle_circle_min_radius_m,
                        span_y * 0.55 + self.obstacle_circle_padding_m,
                    ),
                )
            if radius > self.obstacle_circle_max_radius_m:
                continue

            nearest_distance = min(point[2] for point in cluster)
            circles.append({
                'center_x': center_x,
                'center_y': center_y,
                'radius': radius,
                'span': span,
                'span_x': span_x,
                'span_y': span_y,
                'nearest_distance': nearest_distance,
                'angle_deg': math.degrees(math.atan2(center_y, max(center_x, 1e-6))),
                'closest_x': center_x - radius,
                'farthest_x': center_x + radius,
                'lateral_min': center_y - radius,
                'lateral_max': center_y + radius,
            })

        circles.sort(key=lambda circle: (circle['closest_x'], abs(circle['center_y'])))
        return circles

    def obstacle_circle_centerline_clearance_radius(self, circle=None):
        circle = self.active_obstacle_circle if circle is None else circle
        planning_radius = self.obstacle_circle_planning_radius(circle)
        if circle is None or planning_radius is None:
            return None
        return planning_radius + self.obstacle_circle_path_half_width_m

    def driving_corridor_half_width_m(self):
        return self.obstacle_circle_path_half_width_m + self.obstacle_corridor_body_half_width_m

    def circle_intrudes_corridor(self, circle):
        if circle is None:
            return False
        half_width = self.driving_corridor_half_width_m()
        return not (
            circle['lateral_max'] < -half_width
            or circle['lateral_min'] > half_width
        )

    def circle_is_side_fence(self, circle):
        """Fence/rail beside the lane — not a cone blocking the driving tube."""
        if circle is None:
            return False
        center_y = float(circle['center_y'])
        closest_x = float(circle['closest_x'])
        abs_center_y = abs(center_y)
        half_width = self.driving_corridor_half_width_m()
        if abs_center_y >= self.obstacle_side_fence_center_y_m and closest_x > 0.22:
            return True
        span_x = float(circle.get('span_x', circle.get('span', 0.0)))
        span_y = float(circle.get('span_y', 0.0))
        if span_x > 0.40 and span_x > span_y * 1.5 and abs_center_y > half_width * 0.50:
            return True
        if (
            float(circle['radius']) >= self.obstacle_circle_max_radius_m * 0.90
            and abs_center_y > half_width * 0.65
            and closest_x > 0.30
        ):
            return True
        return False

    def circle_is_opposite_far_wall(self, circle):
        """Wall across the inner loop — ignore until it actually blocks the lane ahead."""
        if circle is None:
            return False
        if float(circle.get('radius', 0.12)) <= 0.22 and self.circle_blocks_segment_path(circle):
            return False
        abs_y = abs(float(circle['center_y']))
        half_width = self.driving_corridor_half_width_m()
        if abs_y > half_width + 0.10:
            return False
        center_x = float(circle['center_x'])
        closest_x = float(circle['closest_x'])
        farthest_x = float(circle.get('farthest_x', closest_x))
        radius = float(circle['radius'])
        # Small cluster straight ahead (~1m) — typical 回字对面内墙，不是锥桶
        if closest_x >= 0.42 and radius <= 0.25 and center_x >= 0.65:
            return True
        if center_x < self.obstacle_opposite_wall_min_center_x_m:
            return False
        depth = farthest_x - closest_x
        return depth > 0.25 and closest_x > 0.45

    def front_is_likely_opposite_wall(self):
        """Forward ray hits the far inner wall of the loop, not an immediate blocker."""
        if not math.isfinite(self.front_obstacle_distance):
            return False
        if abs(self.front_obstacle_angle_deg) > 15.0:
            return False
        circle = self.active_obstacle_circle
        if circle is not None and self.circle_is_opposite_far_wall(circle):
            return True
        if self.is_turn_detour_segment():
            return self.front_obstacle_distance >= self.obstacle_opposite_front_min_distance_m
        return False

    def segment_detour_trigger_distance_m(self):
        if self.is_turn_detour_segment():
            return self.detour_turn_max_trigger_distance_m
        return self.detour_obstacle_detect_distance

    def segment_path_heading_rad(self):
        """Heading of the path the robot is following / will exit onto after a turn."""
        if self.current_segment is None:
            return self.current_yaw
        segment_type = self.current_segment.get('type')
        if segment_type == 'move':
            if self.segment_heading is not None:
                return self.segment_heading
        if segment_type == 'turn' and self.segment_target_yaw is not None:
            return self.segment_target_yaw
        return self.current_yaw

    def circle_offset_in_segment_path_frame(self, circle):
        """Project obstacle circle into the current segment driving / exit heading frame."""
        if circle is None or self.current_yaw is None:
            return None
        path_heading = self.segment_path_heading_rad()
        if path_heading is None:
            return None
        delta = self.angle_error(path_heading, self.current_yaw)
        cos_d = math.cos(delta)
        sin_d = math.sin(delta)
        center_x = float(circle['center_x'])
        center_y = float(circle['center_y'])
        along = cos_d * center_x - sin_d * center_y
        lateral = sin_d * center_x + cos_d * center_y
        return along, lateral

    def circle_blocks_segment_path(self, circle):
        """True only if the circle blocks the segment path (not merely beside the current robot nose)."""
        if circle is None:
            return False
        offset = self.circle_offset_in_segment_path_frame(circle)
        if offset is None:
            return False
        along, lateral = offset
        radius = float(circle['radius'])
        half_width = self.driving_corridor_half_width_m()
        closest_along = along - radius
        farthest_along = along + radius
        if abs(lateral) > half_width + radius * 0.40:
            return False
        if farthest_along < -0.05:
            return False
        min_along = 0.25 if self.is_turn_detour_segment() else 0.12
        closest_x = float(circle.get('closest_x', along))
        if closest_x <= self.avoid_commit_distance_m + 0.12:
            min_along = 0.0
        elif closest_x <= self.segment_detour_trigger_distance_m():
            min_along = min(min_along, 0.04)
        along_seg = self.obstacle_along_segment_m(circle)
        if along_seg is not None and along_seg >= min_along - 0.02:
            min_along = min(min_along, 0.04)
        length = float((self.current_segment or {}).get('distance_m', 99.0))
        center_on_segment = (
            along_seg is not None
            and -radius * 0.35 <= along_seg <= length + radius * 0.35
        )
        if closest_along < min_along:
            if not center_on_segment:
                return False
            closest_along = 0.0
        trigger_distance = self.segment_detour_trigger_distance_m()
        return closest_along <= trigger_distance + 0.08

    def move_segment_remaining_m(self):
        if self.current_segment is None or self.current_segment.get('type') != 'move':
            return None
        target_distance = float(self.current_segment.get('distance_m', 0.0))
        return max(0.0, target_distance - self.projected_distance())

    def approaching_turn_segment_end(self):
        """Near the end of a move leg before the next turn — finish the leg, do not start detour."""
        if self.current_segment is None or self.current_segment.get('type') != 'move':
            return False
        remaining = self.move_segment_remaining_m()
        seg_len = float(self.current_segment.get('distance_m', 0.0))
        # Short legs (e.g. rect_side_2 0.50 m): do not block avoidance for the whole segment.
        gate_m = min(0.55, max(0.12, seg_len * 0.45))
        if remaining is None or remaining > gate_m:
            return False
        next_index = self.plan_index + 1
        if next_index >= len(self.plan):
            return False
        return self.plan[next_index].get('type') == 'turn'

    def previous_segment_is_ring_corner_turn(self):
        prev_index = self.plan_index - 1
        if prev_index < 0 or not hasattr(self, 'plan'):
            return False
        return self.plan[prev_index].get('type') == 'turn'

    def circle_counts_for_detour_trigger(self, circle):
        if circle is None:
            return False
        if not self.circle_intrudes_corridor(circle):
            return False
        if not self.circle_blocks_segment_path(circle):
            return False
        if self.circle_is_side_fence(circle):
            return False
        if self.circle_is_opposite_far_wall(circle):
            return False
        if self.circle_conflicts_with_front_sector(circle):
            return False
        return True

    def planning_obstacle_circles(self):
        """Circles that should occupy the local planner grid (exclude fence/opposite wall)."""
        filtered = []
        for circle in self.detected_obstacle_circles:
            if self.circle_is_side_fence(circle):
                continue
            if self.circle_is_opposite_far_wall(circle):
                continue
            filtered.append(circle)
        return filtered

    def front_counts_for_detour_trigger(self):
        if not math.isfinite(self.front_obstacle_distance):
            return False
        trigger_distance = self.segment_detour_trigger_distance_m()
        if self.front_obstacle_distance > trigger_distance:
            return False
        if self.front_is_likely_opposite_wall():
            return False
        front_angle_limit = min(self.detour_front_test_angle_deg, 12.0)
        if abs(self.front_obstacle_angle_deg) > front_angle_limit:
            return False
        circle = self.active_obstacle_circle
        if circle is not None and (
            self.circle_is_side_fence(circle) or self.circle_is_opposite_far_wall(circle)
        ):
            return self.front_obstacle_distance <= max(
                0.50,
                self.detour_obstacle_clear_distance,
            )
        return True

    def raw_detour_nearest_obstacle_distance_m(self):
        """Unfiltered ranging (for debug / trigger-suppress logging)."""
        nearest = float('inf')
        if math.isfinite(self.front_obstacle_distance):
            nearest = self.front_obstacle_distance
        for circle in self.detected_obstacle_circles:
            if circle['farthest_x'] <= 0.0:
                continue
            if abs(circle['angle_deg']) > self.detour_front_test_angle_deg + 8.0:
                continue
            circle_distance = float(circle.get('nearest_distance', circle['closest_x']))
            nearest = min(nearest, circle_distance)
        return nearest

    def dwa_path_blocker_imminent(self):
        """True only when a validated corridor circle blocks the segment path.

        Does not trigger on raw front-ray hits (side fence / opposite wall / noise).
        """
        circle = self.active_obstacle_circle
        if circle is None:
            return False
        if not self.circle_counts_for_detour_trigger(circle):
            return False
        nearest = float(circle.get('nearest_distance', circle.get('closest_x', float('inf'))))
        if not math.isfinite(nearest) or nearest > self.detour_obstacle_detect_distance:
            return False
        return True

    def obstacle_trigger_suppressed_reason(self):
        nearest_raw = self.raw_detour_nearest_obstacle_distance_m()
        if not math.isfinite(nearest_raw) or nearest_raw > self.detour_obstacle_detect_distance:
            return None
        if self.is_turn_detour_segment():
            return 'turn_segment_no_detour'
        if self.approaching_turn_segment_end():
            circle = self.active_obstacle_circle
            if circle is not None and self.circle_blocks_segment_path(circle):
                return None
            return 'approaching_turn_end'
        # Only true while DWA is running — do not call obstacle_is_active() here
        # (it uses avoidance_should_enter and would recurse).
        if self.avoidance_active:
            return None
        if self.template_path_blocker_imminent():
            return None
        for circle in self.detected_obstacle_circles:
            if not self.circle_counts_for_detour_trigger(circle):
                if self.circle_is_side_fence(circle):
                    return 'side_fence'
                if self.circle_is_opposite_far_wall(circle):
                    return 'opposite_wall'
                if not self.circle_intrudes_corridor(circle):
                    return 'outside_corridor'
                if not self.circle_blocks_segment_path(circle):
                    return 'not_on_segment_path'
                if self.circle_conflicts_with_front_sector(circle):
                    return 'front_sector_conflict'
        if (
            math.isfinite(self.front_obstacle_distance)
            and self.front_obstacle_distance <= self.detour_obstacle_detect_distance
            and not self.front_counts_for_detour_trigger()
        ):
            if self.front_is_likely_opposite_wall():
                return 'opposite_wall_front'
            if (
                self.active_obstacle_circle is None
                and self.front_obstacle_distance > self.front_only_detour_trigger_distance_m()
            ):
                return 'front_only_too_far'
            return 'front_side_fence_mix'
        if self.is_turn_detour_segment() and math.isfinite(nearest_raw):
            if nearest_raw > self.detour_turn_max_trigger_distance_m:
                return 'turn_far_hit'
        return 'filtered'

    def circle_conflicts_with_front_sector(self, circle):
        if circle is None:
            return False
        if not math.isfinite(self.front_obstacle_distance):
            return False
        if circle['closest_x'] > self.obstacle_circle_forward_margin_m:
            return False
        return self.front_obstacle_distance > (
            self.detour_obstacle_detect_distance + self.obstacle_circle_forward_margin_m
        )

    def _obstacle_circle_blocks_at_distance(self, circle, distance_threshold):
        if circle is None:
            return False
        if circle['closest_x'] > distance_threshold:
            return False
        return not self.circle_conflicts_with_front_sector(circle)

    def _front_obstacle_within_distance(self, distance_threshold):
        return (
            math.isfinite(self.front_obstacle_distance)
            and self.front_obstacle_distance <= distance_threshold
        )

    def front_only_detour_trigger_distance_m(self):
        """Without a corridor obstacle circle, only close forward hits may trigger."""
        if self.is_turn_detour_segment():
            return self.detour_turn_max_trigger_distance_m
        return max(self.detour_obstacle_clear_distance + 0.05, 0.55)

    def detour_nearest_obstacle_distance_m(self):
        """Ranging for trigger/clear — validated active circle only (no raw front ray)."""
        scenario_circle = self.scenario_static_obstacle_circle()
        if scenario_circle is not None:
            return float(
                scenario_circle.get('nearest_distance', scenario_circle.get('closest_x', 99.0))
            )
        circle = self.active_obstacle_circle
        if circle is None or not self.circle_counts_for_detour_trigger(circle):
            return float('inf')
        return float(circle.get('nearest_distance', circle['closest_x']))

    def select_active_obstacle_circle(self, circles):
        candidates = []
        for circle in circles:
            if circle['farthest_x'] <= 0.0:
                continue
            if abs(circle['angle_deg']) > self.detour_front_test_angle_deg + 8.0:
                continue
            if not self.circle_intrudes_corridor(circle):
                continue
            if not self.circle_blocks_segment_path(circle):
                continue
            if self.circle_is_side_fence(circle):
                continue
            if self.circle_is_opposite_far_wall(circle):
                continue
            if self.circle_conflicts_with_front_sector(circle):
                continue
            candidates.append(circle)

        if not candidates:
            return None
        return min(
            candidates,
            key=lambda circle: (
                circle['closest_x'],
                abs(circle['center_y']),
                circle['nearest_distance'],
            ),
        )

    def publish_obstacle_circle_markers(self, scan_msg):
        if hasattr(self, '_offline_sim_time'):
            return
        marker_array = MarkerArray()
        frame_id = scan_msg.header.frame_id
        stamp = scan_msg.header.stamp

        for marker_id, circle in enumerate(self.detected_obstacle_circles):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'detected_obstacle_circles'
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(circle['center_x'])
            marker.pose.position.y = float(circle['center_y'])
            marker.pose.position.z = self.obstacle_circle_marker_height_m / 2.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = circle['radius'] * 2.0
            marker.scale.y = circle['radius'] * 2.0
            marker.scale.z = self.obstacle_circle_marker_height_m
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.35
            marker_array.markers.append(marker)

        for marker_id in range(len(self.detected_obstacle_circles), self.last_obstacle_circle_marker_count):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'detected_obstacle_circles'
            marker.id = marker_id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        active_marker = Marker()
        active_marker.header.frame_id = frame_id
        active_marker.header.stamp = stamp
        active_marker.ns = 'active_obstacle_circle'
        active_marker.id = 0
        if self.active_obstacle_circle is None:
            active_marker.action = Marker.DELETE
        else:
            active_marker.type = Marker.CYLINDER
            active_marker.action = Marker.ADD
            active_marker.pose.position.x = float(self.active_obstacle_circle['center_x'])
            active_marker.pose.position.y = float(self.active_obstacle_circle['center_y'])
            active_marker.pose.position.z = self.obstacle_circle_marker_height_m / 2.0
            active_marker.pose.orientation.w = 1.0
            active_marker.scale.x = self.active_obstacle_circle['radius'] * 2.0
            active_marker.scale.y = self.active_obstacle_circle['radius'] * 2.0
            active_marker.scale.z = self.obstacle_circle_marker_height_m * 1.2
            active_marker.color.r = 1.0
            active_marker.color.g = 0.25
            active_marker.color.b = 0.10
            active_marker.color.a = 0.85
        marker_array.markers.append(active_marker)

        self.last_obstacle_circle_marker_count = len(self.detected_obstacle_circles)
        self.obstacle_circle_pub.publish(marker_array)

    def update_obstacle_circles_from_scan(self, scan_msg):
        clusters = self.cluster_scan_points(scan_msg)
        self.detected_obstacle_circles = self.build_obstacle_circles(clusters)
        scenario_circle = self.scenario_static_obstacle_circle()
        if scenario_circle is not None:
            merged = [scenario_circle]
            for circle in self.detected_obstacle_circles:
                dist = math.hypot(
                    float(circle['center_x']) - float(scenario_circle['center_x']),
                    float(circle['center_y']) - float(scenario_circle['center_y']),
                )
                if dist > float(circle['radius']) + float(scenario_circle['radius']) + 0.06:
                    merged.append(circle)
            self.detected_obstacle_circles = merged
        scenario_circle = self.scenario_static_obstacle_circle()
        if scenario_circle is not None:
            self.active_obstacle_circle = scenario_circle
        else:
            self.active_obstacle_circle = self.select_active_obstacle_circle(self.detected_obstacle_circles)
        self.publish_obstacle_circle_markers(scan_msg)

    def obstacle_circle_planning_radius(self, circle=None):
        circle = self.active_obstacle_circle if circle is None else circle
        if circle is None:
            return None
        return circle['radius'] + self.obstacle_circle_planning_margin_m

    def reset_debug_log(self):
        try:
            os.makedirs(os.path.dirname(self.debug_log_path), exist_ok=True)
            with open(self.debug_log_path, 'w', encoding='utf-8') as log_file:
                log_file.write('=== direct_inertial_tester 避障/路径调试日志 ===\n')
                log_file.write(f'log_path={self.debug_log_path}\n')
                log_file.write(
                    '字段说明: TRIGGER=避障触发/抑制 | DECISION=避障进入退出 | '
                    'MOVE=直行贴线(非避障) | CMD=避障控制 | CONFIG=参数 | SEGMENT=段切换\n'
                )
                log_file.write(
                    'MOVE: phase=弯后航向收敛|直行贴弦线|段末停车对齐 | '
                    '横偏左正=在实测弦线左侧 | ω>0=逆时针/左转 ω<0=右转 | '
                    'STEER_FLIP=转向符号反转(内外来回拧)\n'
                )
                log_file.write(
                    'goal_direct: phase=follow|bypass|pass|rejoin|exit|handoff | '
                    'goals=bypass pass rejoin|exit | need_direct_cut | '
                    'zone=角点区域 before_turn|post_apex\n'
                )
        except OSError as exc:
            self.get_logger().warning(f'调试日志文件初始化失败: {exc}')

    def log_detour(self, message):
        text = f'{self.test_feedback_prefix}避障: {message}'
        self.log_console_and_debug('DETOUR', text)

    def publish_feedback(self, text):
        super().publish_feedback(text)

    def log_detour_snapshot(self, tag, extra=''):
        segment = self.current_segment or {}
        base = (
            f'{tag} segment={segment.get("description", "none")} '
            f'type={segment.get("type", "none")} '
            f'pose={self.format_position_xy()} '
            f'yaw={self.format_yaw_deg(self.current_yaw)}deg '
            f'{self.detour_scan_summary()}'
        )
        if extra:
            base = f'{base} {extra}'
        self.get_logger().info(f'{self.test_feedback_prefix}避障: {base}')
        if tag in ('avoid_enter', 'avoid_exit', 'avoid_handoff_ready'):
            self.write_debug_log('DETOUR', base)

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_frame_id = msg.header.frame_id
        self.front_obstacle_distance, self.front_obstacle_angle_deg = self.sector_closest_obstacle(
            msg,
            -self.detour_front_test_angle_deg,
            self.detour_front_test_angle_deg,
        )
        half_window = self.detour_side_test_window_deg / 2.0
        self.left_clearance_distance = self.sector_min_distance(
            msg,
            self.detour_side_center_deg - half_window,
            self.detour_side_center_deg + half_window,
        )
        self.right_clearance_distance = self.sector_min_distance(
            msg,
            -self.detour_side_center_deg - half_window,
            -self.detour_side_center_deg + half_window,
        )
        self.update_obstacle_circles_from_scan(msg)
        self.update_scan_obstacle_cloud_robot(msg)
        if self.current_segment is not None:
            self.maybe_log_obstacle_trigger_edge(self.debug_log_timestamp())

    def snapshot_blocking_circle_for_detour(self):
        circle = self.active_obstacle_circle
        if circle is None:
            self.local_replan_blocking_circle = None
            return
        world_center = self.circle_center_in_world(circle)
        self.local_replan_blocking_circle = {
            'center_x': float(circle['center_x']),
            'center_y': float(circle['center_y']),
            'radius': float(circle['radius']),
            'closest_x': float(circle['closest_x']),
            'farthest_x': float(circle.get('farthest_x', circle['closest_x'])),
            'world_center': world_center,
        }

    def blocking_circle_robot_metrics(self):
        stored = getattr(self, 'local_replan_blocking_circle', None)
        if stored is None or stored.get('world_center') is None:
            return None
        local_center = self.world_to_robot_local_point(stored['world_center'])
        if local_center is None:
            return None
        radius = float(stored['radius'])
        center_x, center_y = float(local_center[0]), float(local_center[1])
        return {
            'center_x': center_x,
            'center_y': center_y,
            'closest_x': center_x - radius,
            'farthest_x': center_x + radius,
            'radius': radius,
        }

    def obstacle_has_been_passed(self):
        """True when the detour obstacle is behind the robot and the forward cone is open."""
        front_clear_distance = max(self.detour_obstacle_clear_distance + 0.15, 0.85)
        if math.isfinite(self.front_obstacle_distance):
            if self.front_obstacle_distance <= front_clear_distance:
                return False

        block = self.blocking_circle_robot_metrics()
        if block is not None:
            if float(block['farthest_x']) > -0.25:
                return False
            if float(block['closest_x']) > 0.15:
                return False

        circle = self.active_obstacle_circle
        if circle is not None and not self.circle_conflicts_with_front_sector(circle):
            if float(circle.get('farthest_x', circle['closest_x'])) > 0.0:
                return False
            if float(circle.get('nearest_distance', circle['closest_x'])) <= self.detour_obstacle_clear_distance:
                return False

        if block is None and circle is None:
            nearest = self.detour_nearest_obstacle_distance_m()
            if not math.isfinite(nearest):
                return False
            return nearest > self.detour_obstacle_detect_distance
        return True

    def obstacle_is_clear_for_rejoin(self):
        return self.obstacle_has_been_passed()

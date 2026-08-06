"""Stage3 return navigation.

Production control has exactly four motion states:
MAP_SEARCH -> VISUAL_APPROACH -> TERMINAL_COMMIT -> COMPLETE.
Map search uses the Stage2 map anchor and IMU heading.  Once P is stable,
vision owns steering.  Terminal commit freezes steering and drives straight
until P leaves the camera, so no late re-acquire, reverse, or odometry run can
pull the chassis away from the intended P area.
"""

import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from racing_common.process_lifecycle import install_parent_death_signal
from racing_common.racing_logger import RacingLogger
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .cmd_vel_stop import (
    init_without_ros_signal_handler,
    install_stop_event,
    publish_stop,
    spin_until_stop,
)
from .vision_p_detector import VisionPDetector


class Stage3ReturnNavigator(Node):
    MAP_SEARCH = 'map_search'
    VISUAL_APPROACH = 'visual_approach'
    TERMINAL_COMMIT = 'terminal_commit'
    COMPLETE = 'complete'

    def __init__(self):
        super().__init__('stage3_return_navigator')
        self._declare_params()
        self._read_params()

        self.log = RacingLogger(
            self, log_subdir='competition_stage3', log_filename='latest.log',
            session_title='Stage3 return navigator', defer_file=True,
        )
        self._activated = False
        self._released = False
        self._handoff_command_announced = False
        self.mission_active = False
        self.mission_finished = False
        self.start_after_time = None
        self.state = self.MAP_SEARCH

        # Position is propagated from the Stage2 map anchor by odometry XY.
        # Heading always comes from IMU; odometry orientation is never read.
        self.current_position = None
        self.current_yaw = None
        self._imu_yaw = None
        self._imu_yaw_offset = 0.0
        self._awaiting_entry_yaw_alignment = False
        self._last_odom_xy = None
        self._entry_anchor_map = None
        self._entry_anchor_odom = None
        self._map_from_odom_yaw = None
        self._pending_anchor = None
        self.odom_frame_id = 'odom'
        self.path_started_at = None
        self._search_path = []
        self._search_path_lengths = []
        self._search_path_progress = 0.0
        self._search_last_angular = 0.0

        self.latest_scan = None
        self._p_detector = VisionPDetector(
            self,
            model_path=self.p_model_path,
            conf_thres=self.p_conf_thres,
            iou_thres=self.p_iou_thres,
            crop_ratio=self.p_crop_ratio,
            http_port=self.p_web_port,
        )
        self._p_hits = 0
        self._p_last_stamp = None
        self._p_offset = 0.0
        self._p_last_angular = 0.0
        self._visual_lost_since = None
        self._terminal_candidate_since = None
        self._terminal_lost_since = None
        self._terminal_commit_yaw = None

        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, latched_qos)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(String, self.direction_topic, self._direction_cb, latched_qos)
        self.create_subscription(
            PointStamped, self.stage3_entry_anchor_topic,
            self._entry_anchor_cb, latched_qos,
        )
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, 50)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        lifecycle_prefix = self.lifecycle_service_prefix.rstrip('/')
        self.create_service(Trigger, f'{lifecycle_prefix}/activate', self._activate_cb)
        self.create_service(Trigger, f'{lifecycle_prefix}/release', self._release_cb)
        self.create_timer(1.0 / self.control_rate_hz, self._control_loop)

        self._publish_state('standby')
        self.log.startup(
            f'Stage3 four-state navigator ready | search_target='
            f'({self.search_target[0]:.2f},{self.search_target[1]:.2f}) '
            f'cmd={self.cmd_topic} odom={self.odom_topic}'
        )

    def _declare_params(self):
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('state_topic', 'stage3_state')
        self.declare_parameter('feedback_topic', 'competition_feedback')
        self.declare_parameter('direction_topic', 'competition_qr_task')
        self.declare_parameter('stage3_entry_anchor_topic', 'stage3_entry_anchor')
        self.declare_parameter('lifecycle_service_prefix', '/competition/stage3')
        self.declare_parameter('require_stage3_entry_anchor', True)
        self.declare_parameter('stage3_entry_map_yaw_deg', -90.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('start_delay_sec', 0.0)

        self.declare_parameter('path_timeout_sec', 60.0)
        self.declare_parameter('search_target_map_x_m', 0.50)
        self.declare_parameter('search_target_map_y_m', 0.20)
        self.declare_parameter('search_linear_speed', 0.20)
        self.declare_parameter('search_min_turn_radius_m', 0.42)
        self.declare_parameter('search_lookahead_m', 0.35)
        self.declare_parameter('search_angular_slew_rate_rad_s2', 0.90)

        self.declare_parameter('p_model_path', '')
        self.declare_parameter('p_conf_thres', 0.25)
        self.declare_parameter('p_iou_thres', 0.45)
        self.declare_parameter('p_crop_ratio', 0.4)
        self.declare_parameter('p_web_port', 8083)
        self.declare_parameter('p_detection_timeout_sec', 0.35)
        self.declare_parameter('p_approach_conf_threshold', 0.50)
        self.declare_parameter('p_approach_consecutive_hits', 2)
        self.declare_parameter('p_approach_linear_speed', 0.34)
        self.declare_parameter('p_approach_min_linear_speed', 0.20)
        self.declare_parameter('p_offset_filter_alpha', 0.55)
        self.declare_parameter('p_centering_angular_gain', 0.52)
        self.declare_parameter('p_centering_max_angular_speed', 0.38)
        self.declare_parameter('p_centering_slew_rate_rad_s2', 0.70)
        self.declare_parameter('p_centering_slow_offset', 0.20)
        self.declare_parameter('p_visual_loss_grace_sec', 0.45)

        self.declare_parameter('terminal_conf_threshold', 0.50)
        self.declare_parameter('terminal_fill_ratio', 0.35)
        self.declare_parameter('terminal_center_offset', 0.40)
        self.declare_parameter('terminal_evidence_hold_sec', 0.15)
        self.declare_parameter('terminal_linear_speed', 0.14)
        self.declare_parameter('terminal_loss_hold_sec', 0.35)

        # Lidar only protects map search.  P approach has a known terminal
        # object ahead, so it must never trigger detour/reverse control there.
        self.declare_parameter('search_emergency_distance_m', 0.28)
        self.declare_parameter('search_emergency_half_width_m', 0.12)
        self.declare_parameter('search_emergency_min_x_m', 0.08)
        self.declare_parameter('search_emergency_max_x_m', 0.45)

    def _read_params(self):
        value = lambda name: self.get_parameter(name).value
        self.odom_topic = str(value('odom_topic'))
        self.imu_topic = str(value('imu_topic'))
        self.scan_topic = str(value('scan_topic'))
        self.cmd_topic = str(value('cmd_topic'))
        self.state_topic = str(value('state_topic'))
        self.feedback_topic = str(value('feedback_topic'))
        self.direction_topic = str(value('direction_topic'))
        self.stage3_entry_anchor_topic = str(value('stage3_entry_anchor_topic'))
        self.lifecycle_service_prefix = str(value('lifecycle_service_prefix'))
        self.require_entry_anchor = bool(value('require_stage3_entry_anchor'))
        self.entry_map_yaw = math.radians(float(value('stage3_entry_map_yaw_deg')))
        self.control_rate_hz = max(5.0, float(value('control_rate_hz')))
        self.start_delay_sec = max(0.0, float(value('start_delay_sec')))

        self.path_timeout_sec = max(1.0, float(value('path_timeout_sec')))
        self.search_target = (
            float(value('search_target_map_x_m')),
            float(value('search_target_map_y_m')),
        )
        self.search_linear = max(0.05, float(value('search_linear_speed')))
        self.search_min_turn_radius = max(0.10, float(value('search_min_turn_radius_m')))
        self.search_lookahead = max(0.10, float(value('search_lookahead_m')))
        self.search_angular_slew = max(
            0.05, float(value('search_angular_slew_rate_rad_s2'))
        )

        self.p_model_path = str(value('p_model_path'))
        self.p_conf_thres = float(value('p_conf_thres'))
        self.p_iou_thres = float(value('p_iou_thres'))
        self.p_crop_ratio = float(value('p_crop_ratio'))
        self.p_web_port = int(value('p_web_port'))
        self.p_detection_timeout = max(0.05, float(value('p_detection_timeout_sec')))
        self.p_approach_conf = min(1.0, max(0.0, float(value('p_approach_conf_threshold'))))
        self.p_hits_required = max(1, int(value('p_approach_consecutive_hits')))
        self.p_approach_linear = max(0.05, float(value('p_approach_linear_speed')))
        self.p_approach_min_linear = min(
            self.p_approach_linear, max(0.05, float(value('p_approach_min_linear_speed')))
        )
        self.p_offset_alpha = min(1.0, max(0.05, float(value('p_offset_filter_alpha'))))
        self.p_centering_gain = max(0.0, float(value('p_centering_angular_gain')))
        self.p_centering_max_angular = max(0.05, float(value('p_centering_max_angular_speed')))
        self.p_centering_slew = max(0.05, float(value('p_centering_slew_rate_rad_s2')))
        self.p_centering_slow_offset = min(0.95, max(0.0, float(value('p_centering_slow_offset'))))
        self.p_visual_loss_grace = max(0.05, float(value('p_visual_loss_grace_sec')))

        self.terminal_conf = min(1.0, max(0.0, float(value('terminal_conf_threshold'))))
        self.terminal_fill = min(1.0, max(0.0, float(value('terminal_fill_ratio'))))
        self.terminal_offset = min(1.0, max(0.0, float(value('terminal_center_offset'))))
        self.terminal_hold = max(0.0, float(value('terminal_evidence_hold_sec')))
        self.terminal_linear = max(0.05, float(value('terminal_linear_speed')))
        self.terminal_loss_hold = max(0.05, float(value('terminal_loss_hold_sec')))

        self.search_emergency_distance = max(0.05, float(value('search_emergency_distance_m')))
        self.search_emergency_half_width = max(0.02, float(value('search_emergency_half_width_m')))
        self.search_emergency_min_x = max(0.01, float(value('search_emergency_min_x_m')))
        self.search_emergency_max_x = max(
            self.search_emergency_min_x, float(value('search_emergency_max_x_m'))
        )

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @classmethod
    def _angle_error(cls, target, current):
        return cls._normalize_angle(target - current)

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    @staticmethod
    def _quat_to_yaw(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _twist(self, linear, angular):
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        if (
            self._activated and not self._handoff_command_announced
            and (abs(command.linear.x) > 1e-4 or abs(command.angular.z) > 1e-4)
        ):
            self._handoff_command_announced = True
            self._publish_state('handoff_command_ready')
        return command

    def _publish_state(self, state):
        self.state_pub.publish(String(data=state))

    def _publish_feedback(self, text):
        self.feedback_pub.publish(String(data=text))
        self.log.feedback(text)

    def _direction_cb(self, msg):
        self.return_direction = str(msg.data).strip().lower()

    def _imu_cb(self, msg):
        self._imu_yaw = self._quat_to_yaw(msg.orientation)
        if self._awaiting_entry_yaw_alignment:
            self._imu_yaw_offset = self._normalize_angle(self.entry_map_yaw - self._imu_yaw)
            self._awaiting_entry_yaw_alignment = False
        self.current_yaw = self._normalize_angle(self._imu_yaw + self._imu_yaw_offset)

    def _scan_cb(self, msg):
        self.latest_scan = msg

    def _odom_cb(self, msg):
        self._last_odom_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        self.odom_frame_id = msg.header.frame_id or self.odom_frame_id
        if self._pending_anchor is not None:
            anchor = self._pending_anchor
            self._pending_anchor = None
            if self._bind_entry_anchor(anchor) and not self._activated:
                # The transient anchor may arrive before the first odometry
                # sample. Complete the standby handshake once it binds.
                self._publish_state('ready')
        if self._entry_anchor_map is not None and self._entry_anchor_odom is not None:
            dx = self._last_odom_xy[0] - self._entry_anchor_odom[0]
            dy = self._last_odom_xy[1] - self._entry_anchor_odom[1]
            cos_yaw = math.cos(self._map_from_odom_yaw)
            sin_yaw = math.sin(self._map_from_odom_yaw)
            self.current_position = (
                self._entry_anchor_map[0] + cos_yaw * dx - sin_yaw * dy,
                self._entry_anchor_map[1] + sin_yaw * dx + cos_yaw * dy,
            )
        else:
            self.current_position = self._lookup_map_xy()

    def _entry_anchor_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != 'map':
            self.log.warn('ENTRY_ANCHOR', f'ignored frame={msg.header.frame_id}')
            return
        anchor = (float(msg.point.x), float(msg.point.y))
        if self._last_odom_xy is None:
            self._pending_anchor = anchor
            return
        if self._bind_entry_anchor(anchor) and not self._activated:
            self._publish_state('ready')

    def _bind_entry_anchor(self, anchor):
        map_from_odom_yaw = self._lookup_map_from_odom_yaw()
        if map_from_odom_yaw is None:
            self._pending_anchor = anchor
            return False
        self._entry_anchor_map = anchor
        self._entry_anchor_odom = self._last_odom_xy
        self._map_from_odom_yaw = map_from_odom_yaw
        self.current_position = anchor
        self.log.mission(
            f'Stage2 entry anchor bound map=({anchor[0]:.3f},{anchor[1]:.3f}) '
            f'odom=({self._last_odom_xy[0]:.3f},{self._last_odom_xy[1]:.3f}) '
            f'map_from_odom_yaw={math.degrees(map_from_odom_yaw):+.1f}deg'
        )
        return True

    def _lookup_map_from_odom_yaw(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', self.odom_frame_id, Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException:
            return None
        return self._quat_to_yaw(transform.transform.rotation)

    def _lookup_map_xy(self):
        for frame in ('base_footprint', 'base_link'):
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', frame, Time(), timeout=Duration(seconds=0.05)
                )
                translation = transform.transform.translation
                return float(translation.x), float(translation.y)
            except TransformException:
                continue
        return None

    def _activate_cb(self, _request, response):
        if self._released:
            response.success = False
            response.message = 'stage3 already released'
            return response
        if self.require_entry_anchor and self._entry_anchor_map is None:
            response.success = False
            response.message = 'waiting for stage3 entry anchor'
            return response
        self._activated = True
        self.log.start_session()
        self._p_detector.start_http_server()
        self._reset_mission()
        response.success = True
        response.message = 'stage3 activated'
        return response

    def _release_cb(self, _request, response):
        self._released = True
        self._activated = False
        self.mission_active = False
        self.cmd_pub.publish(Twist())
        self._publish_state('complete')
        response.success = True
        response.message = 'stage3 released'
        threading.Timer(0.15, lambda: os._exit(0)).start()
        return response

    def _reset_mission(self):
        self.mission_active = False
        self.mission_finished = False
        self._handoff_command_announced = False
        self.state = self.MAP_SEARCH
        self.path_started_at = None
        self._search_path = []
        self._search_path_lengths = []
        self._search_path_progress = 0.0
        self._search_last_angular = 0.0
        self._p_hits = 0
        self._p_last_stamp = None
        self._p_offset = 0.0
        self._p_last_angular = 0.0
        self._visual_lost_since = None
        self._terminal_candidate_since = None
        self._terminal_lost_since = None
        self._terminal_commit_yaw = None
        self._awaiting_entry_yaw_alignment = self._imu_yaw is None
        if self._imu_yaw is not None:
            self._imu_yaw_offset = self._normalize_angle(self.entry_map_yaw - self._imu_yaw)
            self.current_yaw = self.entry_map_yaw
        self._p_detector.set_inference_active(True)
        self.start_after_time = self._now() + self.start_delay_sec
        self._publish_state('armed')

    def _control_loop(self):
        if self._released or self.mission_finished or not self._activated:
            return
        if not self.mission_active:
            if self.start_after_time is None or self._now() < self.start_after_time:
                return
            if self.current_position is None or self.current_yaw is None:
                self._publish_state('waiting_for_pose')
                return
            self.mission_active = True
            self.path_started_at = self._now()
            self._build_search_path(self.current_position, self.current_yaw)
            self.log.mission('S3 control chain started: map_search -> visual_approach -> terminal_commit')

        if self.state == self.MAP_SEARCH:
            self._run_map_search()
        elif self.state == self.VISUAL_APPROACH:
            self._run_visual_approach()
        elif self.state == self.TERMINAL_COMMIT:
            self._run_terminal_commit()

    def _p_detection(self):
        detected, conf, bbox, stamp, offset, fill = self._p_detector.get_p_detection_geometry()
        fresh = time.time() - float(stamp or 0.0) <= self.p_detection_timeout
        return bool(detected and bbox is not None and fresh), float(conf), stamp, float(offset), float(fill)

    def _run_map_search(self):
        detected, conf, stamp, offset, _fill = self._p_detection()
        if detected and conf >= self.p_approach_conf:
            if stamp != self._p_last_stamp:
                self._p_hits += 1
                self._p_last_stamp = stamp
            if self._p_hits >= self.p_hits_required:
                self.state = self.VISUAL_APPROACH
                self._p_offset = offset
                self._p_last_angular = 0.0
                self._visual_lost_since = None
                self._publish_state('visual_approach')
                self.log.mission(
                    f'P acquired: conf={conf:.2f} offset={offset:+.3f}; visual steering owns approach'
                )
                self._publish_feedback('P acquired: visual approach started')
                self._run_visual_approach()
                return
        else:
            self._p_hits = 0
            self._p_last_stamp = None

        if self._search_emergency_obstacle():
            self._search_last_angular = 0.0
            self.cmd_pub.publish(Twist())
            self._publish_state('map_search_safety_hold')
            self.log.warn('MAP_SAFETY', 'near unknown obstacle during map search; holding position')
            return
        if self.path_started_at is not None and self._now() - self.path_started_at > self.path_timeout_sec:
            self._finish('return failed: P search timeout', success=False)
            return
        if self.current_position is None or self.current_yaw is None:
            self._publish_state('waiting_for_pose')
            return

        if len(self._search_path) < 2:
            self._finish('return failed: map search path unavailable', success=False)
            return

        path_progress, cross_track = self._project_to_search_path(self.current_position)
        self._search_path_progress = max(self._search_path_progress, path_progress)
        path_length = self._search_path_lengths[-1]
        at_path_end = self._search_path_progress >= path_length - 0.01
        if at_path_end:
            # The map target only identifies the visual-search area.  Once it
            # has been reached, continue on the final tangent rather than
            # steering back toward a point behind the vehicle.
            target = self._search_path[-1]
            requested_angular = 0.0
        else:
            target = self._search_path_point(
                min(path_length, self._search_path_progress + self.search_lookahead)
            )
            lookahead_yaw = math.atan2(
                target[1] - self.current_position[1],
                target[0] - self.current_position[0],
            )
            heading_error = self._angle_error(lookahead_yaw, self.current_yaw)
            requested_angular = (
                2.0 * self.search_linear * math.sin(heading_error) / self.search_lookahead
            )
        angular_limit = self.search_linear / self.search_min_turn_radius
        requested_angular = self._clamp(requested_angular, angular_limit)
        max_step = self.search_angular_slew / self.control_rate_hz
        angular = self._search_last_angular + self._clamp(
            requested_angular - self._search_last_angular, max_step
        )
        self._search_last_angular = angular
        self._publish_state('map_search')
        self.log.telemetry(
            'MAP_SEARCH',
            f'lookahead=({target[0]:.2f},{target[1]:.2f}) s={self._search_path_progress:.2f}/'
            f'{path_length:.2f} cross={cross_track:.2f} '
            f'v={self.search_linear:.2f} w={angular:.2f} limit={angular_limit:.2f}',
        )
        self.cmd_pub.publish(self._twist(self.search_linear, angular))

    def _build_search_path(self, start, start_yaw):
        """Build the one-way entry arc and tangent straight search path."""
        target = self.search_target
        radius = self.search_min_turn_radius
        candidates = []
        forward = (math.cos(start_yaw), math.sin(start_yaw))
        left_normal = (-forward[1], forward[0])

        for direction in (-1.0, 1.0):
            center = (
                start[0] + direction * radius * left_normal[0],
                start[1] + direction * radius * left_normal[1],
            )
            delta_x = target[0] - center[0]
            delta_y = target[1] - center[1]
            center_distance = math.hypot(delta_x, delta_y)
            if center_distance <= radius + 1e-4:
                continue
            base_scale = radius * radius / (center_distance * center_distance)
            side_scale = (
                radius * math.sqrt(center_distance * center_distance - radius * radius)
                / (center_distance * center_distance)
            )
            for side in (-1.0, 1.0):
                tangent = (
                    center[0] + base_scale * delta_x - side * side_scale * delta_y,
                    center[1] + base_scale * delta_y + side * side_scale * delta_x,
                )
                radial_end = math.atan2(tangent[1] - center[1], tangent[0] - center[0])
                radial_start = math.atan2(start[1] - center[1], start[0] - center[0])
                raw_delta = self._normalize_angle(radial_end - radial_start)
                arc_angle = raw_delta if direction > 0.0 else -raw_delta
                if arc_angle < 0.0:
                    arc_angle += 2.0 * math.pi
                tangent_heading = radial_end + direction * math.pi / 2.0
                straight_heading = math.atan2(target[1] - tangent[1], target[0] - tangent[0])
                if (
                    arc_angle > math.pi
                    or abs(self._angle_error(straight_heading, tangent_heading)) > 1e-3
                ):
                    continue
                straight_length = math.hypot(target[0] - tangent[0], target[1] - tangent[1])
                candidates.append((radius * arc_angle + straight_length, direction, center, radial_start,
                                   arc_angle, tangent, straight_length))

        if not candidates:
            self._search_path = [start, target]
            self.log.warn(
                'MAP_PATH',
                'no forward tangent arc available; using a bounded-curvature straight capture path',
            )
        else:
            _, direction, center, radial_start, arc_angle, tangent, straight_length = min(candidates)
            arc_steps = max(1, int(math.ceil(radius * arc_angle / 0.04)))
            line_steps = max(1, int(math.ceil(straight_length / 0.04)))
            points = [start]
            for step in range(1, arc_steps + 1):
                radial = radial_start + direction * arc_angle * step / arc_steps
                points.append((
                    center[0] + radius * math.cos(radial),
                    center[1] + radius * math.sin(radial),
                ))
            for step in range(1, line_steps + 1):
                ratio = step / line_steps
                points.append((
                    tangent[0] + ratio * (target[0] - tangent[0]),
                    tangent[1] + ratio * (target[1] - tangent[1]),
                ))
            self._search_path = points
            self.log.mission(
                f'MAP_SEARCH path: entry_arc={math.degrees(arc_angle):.1f}deg '
                f'R={radius:.2f}m straight={straight_length:.2f}m '
                f'target=({target[0]:.2f},{target[1]:.2f})'
            )

        lengths = [0.0]
        for first, second in zip(self._search_path, self._search_path[1:]):
            lengths.append(lengths[-1] + math.dist(first, second))
        self._search_path_lengths = lengths
        self._search_path_progress = 0.0

    def _project_to_search_path(self, position):
        best_progress = 0.0
        best_distance = float('inf')
        for index, (first, second) in enumerate(zip(self._search_path, self._search_path[1:])):
            segment_x = second[0] - first[0]
            segment_y = second[1] - first[1]
            segment_length = math.hypot(segment_x, segment_y)
            if segment_length <= 1e-6:
                continue
            ratio = max(0.0, min(1.0,
                ((position[0] - first[0]) * segment_x + (position[1] - first[1]) * segment_y)
                / (segment_length * segment_length),
            ))
            projected_x = first[0] + ratio * segment_x
            projected_y = first[1] + ratio * segment_y
            distance = math.hypot(position[0] - projected_x, position[1] - projected_y)
            if distance < best_distance:
                best_distance = distance
                best_progress = self._search_path_lengths[index] + ratio * segment_length
        return best_progress, best_distance

    def _search_path_point(self, progress):
        for index, (first, second) in enumerate(zip(self._search_path, self._search_path[1:])):
            segment_length = self._search_path_lengths[index + 1] - self._search_path_lengths[index]
            if progress <= self._search_path_lengths[index + 1] or index == len(self._search_path) - 2:
                ratio = max(0.0, min(1.0,
                    (progress - self._search_path_lengths[index]) / max(segment_length, 1e-6),
                ))
                return (
                    first[0] + ratio * (second[0] - first[0]),
                    first[1] + ratio * (second[1] - first[1]),
                )
        return self._search_path[-1]

    def _run_visual_approach(self):
        detected, conf, _stamp, offset, fill = self._p_detection()
        now = self._now()
        if not detected or conf < self.p_approach_conf:
            if self._visual_lost_since is None:
                self._visual_lost_since = now
            if now - self._visual_lost_since <= self.p_visual_loss_grace:
                self._publish_state('visual_loss_grace')
                self.cmd_pub.publish(self._twist(self.p_approach_min_linear, 0.0))
                return
            self.state = self.MAP_SEARCH
            self._p_hits = 0
            self._p_last_stamp = None
            self._visual_lost_since = None
            self._publish_state('map_search')
            self.log.mission('P visual loss exceeded grace period; return to map search without reverse')
            return

        self._visual_lost_since = None
        self._p_offset = self.p_offset_alpha * offset + (1.0 - self.p_offset_alpha) * self._p_offset
        requested_angular = self._clamp(
            -self.p_centering_gain * self._p_offset, self.p_centering_max_angular
        )
        max_step = self.p_centering_slew / self.control_rate_hz
        angular = self._p_last_angular + self._clamp(requested_angular - self._p_last_angular, max_step)
        self._p_last_angular = angular
        excess = max(0.0, abs(self._p_offset) - self.p_centering_slow_offset)
        scale = min(1.0, excess / max(0.05, 1.0 - self.p_centering_slow_offset))
        speed = self.p_approach_linear + scale * (self.p_approach_min_linear - self.p_approach_linear)

        terminal_ready = (
            conf >= self.terminal_conf
            and fill >= self.terminal_fill
            and abs(self._p_offset) <= self.terminal_offset
        )
        if terminal_ready:
            if self._terminal_candidate_since is None:
                self._terminal_candidate_since = now
            elif now - self._terminal_candidate_since >= self.terminal_hold:
                self._enter_terminal_commit(conf, fill)
                return
        else:
            self._terminal_candidate_since = None

        self._publish_state('visual_approach')
        self.log.telemetry(
            'VISUAL_APPROACH',
            f'conf={conf:.2f} off={self._p_offset:+.3f} fill={fill:.2%} '
            f'v={speed:.2f} w={angular:.2f}',
        )
        self.cmd_pub.publish(self._twist(speed, angular))

    def _enter_terminal_commit(self, conf, fill):
        self.state = self.TERMINAL_COMMIT
        self._terminal_lost_since = None
        self._terminal_commit_yaw = self.current_yaw
        self._p_last_angular = 0.0
        self._publish_state('terminal_commit')
        self.log.mission(
            f'TERMINAL_COMMIT: conf={conf:.2f} fill={fill:.2%} off={self._p_offset:+.3f}; '
            f'freeze yaw={math.degrees(self._terminal_commit_yaw):.1f}deg and drive straight'
        )
        self._publish_feedback('P terminal committed: straight final entry')
        self.cmd_pub.publish(self._twist(self.terminal_linear, 0.0))

    def _run_terminal_commit(self):
        detected, conf, _stamp, _offset, _fill = self._p_detection()
        now = self._now()
        if detected and conf >= self.p_approach_conf:
            self._terminal_lost_since = None
            self._publish_state('terminal_commit')
            self.cmd_pub.publish(self._twist(self.terminal_linear, 0.0))
            return
        if self._terminal_lost_since is None:
            self._terminal_lost_since = now
            self.log.segment('TERMINAL_COMMIT P disappeared; keeping fixed straight command for confirmation')
        if now - self._terminal_lost_since < self.terminal_loss_hold:
            self._publish_state('terminal_commit')
            self.cmd_pub.publish(self._twist(self.terminal_linear, 0.0))
            return
        self._finish('return complete: P terminal crossing confirmed', success=True)

    def _search_emergency_obstacle(self):
        scan = self.latest_scan
        if scan is None:
            return False
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            if (
                self.search_emergency_min_x <= x <= self.search_emergency_max_x
                and abs(y) <= self.search_emergency_half_width
                and distance <= self.search_emergency_distance
            ):
                return True
        return False

    def _finish(self, message, success):
        if self.mission_finished:
            return
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self.state = self.COMPLETE
        self._p_detector.release_model('mission_complete')
        self._publish_state('complete' if success else 'failed')
        self._publish_feedback(message)
        map_xy = self._lookup_map_xy()
        if map_xy is not None:
            self.log.real_pose(map_xy[0], map_xy[1], source='map_tf', force=True)
        self.log.task(f'Stage3 finished: {message}')
        timer = threading.Timer(0.50, lambda: os._exit(0))
        timer.daemon = True
        timer.start()

    def destroy_node(self):
        try:
            self._p_detector.set_inference_active(False)
            self._p_detector.stop_http_server()
            self.cmd_pub.publish(Twist())
            self.log.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    install_parent_death_signal()
    init_without_ros_signal_handler(args)
    node = Stage3ReturnNavigator()
    stop_event = threading.Event()
    install_stop_event(
        stop_event, lambda: publish_stop(node.cmd_pub, repeat=10), cli_topics=['/cmd_vel']
    )
    try:
        spin_until_stop(node, stop_event)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            publish_stop(node.cmd_pub, repeat=15)
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

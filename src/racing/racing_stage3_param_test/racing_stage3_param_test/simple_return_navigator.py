"""Stage3 简化返程：左转 230° → 盲驱+锥桶避障 → 墙角检测+航向修正 → 靠近停车"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32, String

from .corner_detector import CornerDetector
from .obstacle_classifier import ObstacleClassifier

# 5 态状态枚举
S_IDLE = 'idle'
S_INITIAL_TURN = 'initial_turn'
S_BLIND_DRIVE = 'blind_drive'
S_CORNER_LOCKED = 'corner_locked'
S_CORNER_CONFIRMED = 'corner_confirmed'
S_APPROACH = 'approach_corner'
S_FINISH = 'finish'

# 避障子状态
AV_FORWARD = 'forward'
AV_AVOIDING = 'avoiding'
AV_COUNTERSTEER = 'countersteering'
AV_RECOVERING = 'recovering'


class SimpleReturnNavigator(Node):
    def __init__(self):
        super().__init__('simple_return_navigator')

        self._declare_params()
        self._read_params()

        self.corner_detector = CornerDetector(self._cfg)
        self.obstacle_classifier = ObstacleClassifier(self._cfg)

        self.state = S_IDLE
        self.phase = 1
        self.target_yaw = None
        self.current_yaw = None

        self.corner_position = None
        self.last_correction_time = 0.0

        self.avoid_state = AV_FORWARD
        self.avoid_turn_direction = 0.0
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.create_subscription(Int32, self.phase_topic, self._phase_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)

        self.latest_scan = None
        self._start_time = None
        self.publish_state(S_IDLE)
        self.create_timer(0.05, self._control_loop)
        self.get_logger().info('simple return navigator ready (stage3 simplified)')

    def _declare_params(self):
        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('state_topic', 'stage3_state')
        self.declare_parameter('test_direction', 'clockwise')

        self.declare_parameter('turn_angular_speed', 0.75)
        self.declare_parameter('turn_heading_tolerance_deg', 3.0)
        self.declare_parameter('turn_kp', 2.0)

        self.declare_parameter('blind_drive_linear_speed', 0.15)
        self.declare_parameter('heading_maintain_kp', 2.4)
        self.declare_parameter('heading_tolerance_deg', 6.0)
        self.declare_parameter('in_place_angle_deg', 8.0)

        self.declare_parameter('douglas_peucker_epsilon', 0.05)
        self.declare_parameter('corner_min_line_length', 0.30)
        self.declare_parameter('corner_angle_tolerance_deg', 15.0)
        self.declare_parameter('corner_min_distance', 0.30)
        self.declare_parameter('corner_max_distance', 2.5)
        self.declare_parameter('corner_min_points', 8)

        self.declare_parameter('confirmation_frames', 3)
        self.declare_parameter('corner_position_tolerance', 0.15)
        self.declare_parameter('correction_cooldown_far_sec', 2.0)
        self.declare_parameter('correction_cooldown_mid_sec', 1.0)
        self.declare_parameter('correction_cooldown_near_sec', 0.5)
        self.declare_parameter('max_correction_angle_deg', 15.0)
        self.declare_parameter('far_distance_threshold', 1.0)
        self.declare_parameter('mid_distance_threshold', 0.5)

        self.declare_parameter('approach_stop_distance', 0.20)
        self.declare_parameter('approach_max_linear_speed', 0.15)
        self.declare_parameter('approach_min_linear_speed', 0.06)
        self.declare_parameter('approach_turn_kp', 1.8)

        self.declare_parameter('cone_max_width', 0.15)
        self.declare_parameter('cone_max_distance', 0.80)
        self.declare_parameter('cone_avoidance_trigger', 0.40)
        self.declare_parameter('wall_min_width', 0.30)
        self.declare_parameter('wall_detect_distance', 2.5)
        self.declare_parameter('phase1_window_min_x', 0.18)
        self.declare_parameter('phase1_window_max_x', 0.85)
        self.declare_parameter('phase1_window_half_width', 0.22)
        self.declare_parameter('phase1_min_cluster_points', 3)
        self.declare_parameter('min_valid_range', 0.15)

        self.declare_parameter('avoid_linear_speed', 0.10)
        self.declare_parameter('avoid_angular_speed', 0.80)
        self.declare_parameter('avoid_min_duration_sec', 0.70)
        self.declare_parameter('avoid_clear_hold_sec', 0.25)
        self.declare_parameter('avoid_min_turn_angle_deg', 18.0)
        self.declare_parameter('recovery_linear_speed', 0.12)
        self.declare_parameter('recovery_turn_linear_speed', 0.08)
        self.declare_parameter('recovery_angular_speed', 0.75)
        self.declare_parameter('recovery_heading_kp', 2.4)
        self.declare_parameter('recovery_max_angular_speed', 1.1)
        self.declare_parameter('recovery_min_angular_speed', 0.5)
        self.declare_parameter('recovery_in_place_angle_deg', 8.0)
        self.declare_parameter('counter_steer_linear_speed', 0.10)
        self.declare_parameter('counter_steer_angular_speed', 0.95)
        self.declare_parameter('counter_steer_duration_scale', 1.35)
        self.declare_parameter('counter_steer_min_duration_sec', 0.45)
        self.declare_parameter('counter_steer_max_duration_sec', 1.20)
        self.declare_parameter('recovery_timeout', 2.5)
        self.declare_parameter('recovery_duration_scale', 0.9)

    def _read_params(self):
        self.phase_topic = str(self.get_parameter('phase_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.test_direction = str(self.get_parameter('test_direction').value).strip().lower()

        self.turn_angular_speed = float(self.get_parameter('turn_angular_speed').value)
        self.turn_heading_tolerance = math.radians(float(self.get_parameter('turn_heading_tolerance_deg').value))
        self.turn_kp = float(self.get_parameter('turn_kp').value)

        self.blind_drive_linear_speed = float(self.get_parameter('blind_drive_linear_speed').value)
        self.heading_maintain_kp = float(self.get_parameter('heading_maintain_kp').value)
        self.heading_tolerance = math.radians(float(self.get_parameter('heading_tolerance_deg').value))
        self.in_place_angle = math.radians(float(self.get_parameter('in_place_angle_deg').value))

        self.confirmation_frames = int(self.get_parameter('confirmation_frames').value)
        self.corner_position_tolerance = float(self.get_parameter('corner_position_tolerance').value)
        self.correction_cooldown_far = float(self.get_parameter('correction_cooldown_far_sec').value)
        self.correction_cooldown_mid = float(self.get_parameter('correction_cooldown_mid_sec').value)
        self.correction_cooldown_near = float(self.get_parameter('correction_cooldown_near_sec').value)
        self.max_correction_angle = math.radians(float(self.get_parameter('max_correction_angle_deg').value))
        self.far_distance_threshold = float(self.get_parameter('far_distance_threshold').value)
        self.mid_distance_threshold = float(self.get_parameter('mid_distance_threshold').value)

        self.approach_stop_distance = float(self.get_parameter('approach_stop_distance').value)
        self.approach_max_linear_speed = float(self.get_parameter('approach_max_linear_speed').value)
        self.approach_min_linear_speed = float(self.get_parameter('approach_min_linear_speed').value)
        self.approach_turn_kp = float(self.get_parameter('approach_turn_kp').value)

        self.cone_avoidance_trigger = float(self.get_parameter('cone_avoidance_trigger').value)
        self.phase1_window_min_x = float(self.get_parameter('phase1_window_min_x').value)
        self.phase1_window_max_x = float(self.get_parameter('phase1_window_max_x').value)
        self.phase1_window_half_width = float(self.get_parameter('phase1_window_half_width').value)

        self.avoid_linear_speed = float(self.get_parameter('avoid_linear_speed').value)
        self.avoid_angular_speed = float(self.get_parameter('avoid_angular_speed').value)
        self.avoid_min_duration_sec = float(self.get_parameter('avoid_min_duration_sec').value)
        self.avoid_clear_hold_sec = float(self.get_parameter('avoid_clear_hold_sec').value)
        self.avoid_min_turn_angle = math.radians(float(self.get_parameter('avoid_min_turn_angle_deg').value))
        self.recovery_linear_speed = float(self.get_parameter('recovery_linear_speed').value)
        self.recovery_turn_linear_speed = float(self.get_parameter('recovery_turn_linear_speed').value)
        self.recovery_angular_speed = float(self.get_parameter('recovery_angular_speed').value)
        self.recovery_heading_kp = float(self.get_parameter('recovery_heading_kp').value)
        self.recovery_max_angular_speed = float(self.get_parameter('recovery_max_angular_speed').value)
        self.recovery_min_angular_speed = float(self.get_parameter('recovery_min_angular_speed').value)
        self.recovery_in_place_angle = math.radians(float(self.get_parameter('recovery_in_place_angle_deg').value))
        self.counter_steer_linear_speed = float(self.get_parameter('counter_steer_linear_speed').value)
        self.counter_steer_angular_speed = float(self.get_parameter('counter_steer_angular_speed').value)
        self.counter_steer_duration_scale = float(self.get_parameter('counter_steer_duration_scale').value)
        self.counter_steer_min_duration = float(self.get_parameter('counter_steer_min_duration_sec').value)
        self.counter_steer_max_duration = float(self.get_parameter('counter_steer_max_duration_sec').value)
        self.recovery_timeout = float(self.get_parameter('recovery_timeout').value)
        self.recovery_duration_scale = float(self.get_parameter('recovery_duration_scale').value)

        self._cfg = {
            'douglas_peucker_epsilon': float(self.get_parameter('douglas_peucker_epsilon').value),
            'corner_min_line_length': float(self.get_parameter('corner_min_line_length').value),
            'corner_angle_tolerance_deg': float(self.get_parameter('corner_angle_tolerance_deg').value),
            'corner_min_distance': float(self.get_parameter('corner_min_distance').value),
            'corner_max_distance': float(self.get_parameter('corner_max_distance').value),
            'corner_min_points': int(self.get_parameter('corner_min_points').value),
            'cone_max_width': float(self.get_parameter('cone_max_width').value),
            'cone_max_distance': float(self.get_parameter('cone_max_distance').value),
            'wall_min_width': float(self.get_parameter('wall_min_width').value),
            'wall_detect_distance': float(self.get_parameter('wall_detect_distance').value),
            'phase1_min_cluster_points': int(self.get_parameter('phase1_min_cluster_points').value),
            'min_valid_range': float(self.get_parameter('min_valid_range').value),
        }

    # ── 工具方法 ──

    def _normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def _angle_error(self, target, current):
        return self._normalize_angle(target - current)

    def _clamp(self, v, limit):
        return max(-limit, min(limit, v))

    def _twist(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        return t

    def publish_state(self, text):
        self.state_pub.publish(String(data=text))

    def _quat_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _now_ros(self):
        return self.get_clock().now()

    # ── 回调 ──

    def _phase_cb(self, msg):
        self.phase = int(msg.data)
        if self.phase == 3 and self.state == S_IDLE:
            if self._start_time is None:
                self._start_time = self._now_sec() + 0.5
                self.get_logger().info('phase=3 detected, will start after 0.5s delay')

    def _start_mission(self):
        self.state = S_INITIAL_TURN
        self.publish_state('initial_turn')
        self.get_logger().info(
            f'mission started, direction={self.test_direction}, '
            f'target_yaw=230°'
        )

    def _imu_cb(self, msg):
        self.current_yaw = self._quat_to_yaw(msg.orientation)

    def _scan_cb(self, msg):
        self.latest_scan = msg

    # ── 主控制循环 (20Hz) ──

    def _control_loop(self):
        if self.phase != 3:
            return
        
        # 延时启动
        if self.state == S_IDLE:
            if self._start_time is not None and self._now_sec() >= self._start_time:
                self._start_mission()
            return
        
        if self.state == S_FINISH:
            return
        
        if self.current_yaw is None:
            return

        if self.state == S_INITIAL_TURN:
            self._do_initial_turn()
        elif self.state in (S_BLIND_DRIVE, S_CORNER_LOCKED, S_CORNER_CONFIRMED):
            self._do_drive_with_corner()
        elif self.state == S_APPROACH:
            self._do_approach_corner()

    # ── Phase 1：初始转向 ──

    def _get_target_yaw(self):
        return math.radians(230.0)

    def _do_initial_turn(self):
        target = self._get_target_yaw()
        error = self._angle_error(target, self.current_yaw)

        if abs(error) < self.turn_heading_tolerance:
            self.target_yaw = target
            self.state = S_BLIND_DRIVE
            self.publish_state('blind_drive')
            self.get_logger().info(
                f'initial turn done, yaw={math.degrees(target):.1f}°, switching to blind drive'
            )
            self.cmd_pub.publish(self._twist())
            return

        angular = self._clamp(self.turn_kp * error, self.turn_angular_speed)
        if abs(angular) < 0.2:
            angular = math.copysign(0.2, error)

        self.get_logger().info(
            f'turning: current={math.degrees(self.current_yaw):.1f}°, '
            f'target={math.degrees(target):.1f}°, error={math.degrees(error):.1f}°, '
            f'angular={angular:.2f}',
            throttle_duration_sec=1.0
        )
        self.cmd_pub.publish(self._twist(0.0, angular))

    # ── Phase 2-4：盲驱 + 墙角检测 + 航向修正 + 锥桶避障 ──

    def _do_drive_with_corner(self):
        if self.avoid_state != AV_FORWARD:
            self._do_avoidance()
            return

        if self.latest_scan is not None:
            self._check_cone_obstacle(self.latest_scan)
            if self.avoid_state != AV_FORWARD:
                self._do_avoidance()
                return

        if self.latest_scan is not None:
            self._update_corner_detection(self.latest_scan)

        cmd = self._maintain_heading()
        self.cmd_pub.publish(cmd)

    def _check_cone_obstacle(self, scan_msg):
        cone = self.obstacle_classifier.find_nearest_cone(
            scan_msg, self.phase1_window_min_x,
            self.phase1_window_max_x, self.phase1_window_half_width,
        )
        if cone is not None and cone[2] < self.cone_avoidance_trigger:
            cx, cy, dist = cone
            danger_angle = math.degrees(math.atan2(cy, max(cx, 1e-6)))
            self.get_logger().info(
                f'cone detected at ({cx:.2f}, {cy:.2f}), dist={dist:.2f}m, '
                f'angle={danger_angle:.1f}° — triggering avoidance'
            )
            self._begin_avoidance(danger_angle)

    def _update_corner_detection(self, scan_msg):
        detected, cx, cy, conf = self.corner_detector.detect(scan_msg)

        if detected:
            confirmed = self.corner_detector.get_confirmed_corner(
                self.confirmation_frames, self.corner_position_tolerance,
            )
            if confirmed is not None:
                self.corner_position = confirmed
                corner_dist = math.hypot(confirmed[0], confirmed[1])

                if self.state != S_CORNER_CONFIRMED:
                    self.state = S_CORNER_CONFIRMED
                    self.publish_state('corner_confirmed')
                    self.get_logger().info(
                        f'corner confirmed at ({confirmed[0]:.2f}, {confirmed[1]:.2f}), '
                        f'dist={corner_dist:.2f}m, conf={conf:.2f}'
                    )

                now = self._now_sec()
                self._try_correct_heading(confirmed[0], confirmed[1], corner_dist, now)

                if corner_dist < self.mid_distance_threshold:
                    self.state = S_APPROACH
                    self.publish_state('approach_corner')
                    self.get_logger().info('switching to approach_corner mode')
                return
            else:
                if self.state == S_BLIND_DRIVE:
                    self.state = S_CORNER_LOCKED
                    self.publish_state('corner_locked')
        else:
            if self.state == S_CORNER_CONFIRMED:
                self.state = S_BLIND_DRIVE
                self.publish_state('blind_drive')
            elif self.state == S_CORNER_LOCKED:
                if len(self.corner_detector._history) == 0:
                    self.state = S_BLIND_DRIVE
                    self.publish_state('blind_drive')

    def _try_correct_heading(self, cx, cy, corner_dist, now):
        if corner_dist > self.far_distance_threshold:
            cooldown = self.correction_cooldown_far
        elif corner_dist > self.mid_distance_threshold:
            cooldown = self.correction_cooldown_mid
        else:
            cooldown = self.correction_cooldown_near

        if now - self.last_correction_time < cooldown:
            return

        ideal_yaw = math.atan2(cy, cx)
        correction = self._angle_error(ideal_yaw, self.target_yaw)
        correction = self._clamp(correction, self.max_correction_angle)

        if abs(correction) < math.radians(3.0):
            return

        self.target_yaw = self._normalize_angle(self.target_yaw + correction)
        self.last_correction_time = now

        self.get_logger().info(
            f'heading corrected: {math.degrees(correction):.1f}° → '
            f'{math.degrees(self.target_yaw):.1f}°, corner_dist={corner_dist:.2f}m'
        )

    def _maintain_heading(self):
        if self.current_yaw is None or self.target_yaw is None:
            return self._twist()

        error = self._angle_error(self.target_yaw, self.current_yaw)

        if abs(error) <= self.heading_tolerance:
            return self._twist(self.blind_drive_linear_speed, 0.0)

        angular = self._clamp(self.heading_maintain_kp * error, self.recovery_max_angular_speed)
        if abs(angular) < self.recovery_min_angular_speed:
            angular = math.copysign(self.recovery_min_angular_speed, error)

        if abs(error) > self.recovery_in_place_angle:
            linear = self.recovery_turn_linear_speed
        else:
            linear = self.recovery_linear_speed

        return self._twist(linear, angular)

    # ── Phase 5：精确靠近墙角 ──

    def _do_approach_corner(self):
        if self.corner_position is None:
            self.state = S_BLIND_DRIVE
            self.publish_state('blind_drive')
            return

        cx, cy = self.corner_position
        dist = math.hypot(cx, cy)

        if dist < self.approach_stop_distance:
            self.state = S_FINISH
            self.publish_state('complete')
            self.cmd_pub.publish(self._twist())
            self.get_logger().info(f'mission complete! distance to corner={dist:.2f}m')
            return

        target_yaw = math.atan2(cy, cx)
        error = self._angle_error(target_yaw, self.current_yaw)

        if abs(error) > math.radians(30.0):
            angular = self._clamp(2.0 * error, 0.8)
            self.cmd_pub.publish(self._twist(0.05, angular))
            return

        speed = max(self.approach_min_linear_speed,
                    min(self.approach_max_linear_speed, dist * 0.5))
        angular = self._clamp(self.approach_turn_kp * error, 0.8)

        self.cmd_pub.publish(self._twist(speed, angular))

    # ── Stage1 4态避障 ──

    def _begin_avoidance(self, danger_angle):
        self.avoid_state = AV_AVOIDING
        self.avoid_turn_direction = -1.0 if danger_angle > 0.0 else 1.0
        self.avoid_started_time = self._now_ros()
        self.avoid_clear_since = None
        self.avoid_entry_yaw = self.current_yaw
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.publish_state('avoiding')
        self.get_logger().info(
            f'avoidance started: turn_dir={self.avoid_turn_direction:.1f}, '
            f'danger_angle={danger_angle:.1f}°'
        )

    def _do_avoidance(self):
        now = self._now_ros()

        if self.avoid_state == AV_AVOIDING:
            turned = False
            if self.current_yaw is not None and self.avoid_entry_yaw is not None:
                turned = abs(self._angle_error(self.current_yaw,
                                               self.avoid_entry_yaw)) >= self.avoid_min_turn_angle

            avoid_elapsed = 0.0
            if self.avoid_started_time is not None:
                avoid_elapsed = (now - self.avoid_started_time).nanoseconds / 1e9

            cone_clear = True
            if self.latest_scan is not None:
                cone = self.obstacle_classifier.find_nearest_cone(
                    self.latest_scan, self.phase1_window_min_x,
                    self.phase1_window_max_x, self.phase1_window_half_width,
                )
                if cone is not None and cone[2] < 0.65:
                    cone_clear = False

            if cone_clear and self.avoid_clear_since is None:
                self.avoid_clear_since = now
            elif not cone_clear:
                self.avoid_clear_since = None

            clear_elapsed = 0.0
            if self.avoid_clear_since is not None:
                clear_elapsed = (now - self.avoid_clear_since).nanoseconds / 1e9

            if avoid_elapsed >= self.avoid_min_duration_sec and clear_elapsed >= self.avoid_clear_hold_sec and turned:
                self._begin_counter_steer()
                return

            self.cmd_pub.publish(
                self._twist(self.avoid_linear_speed,
                            self.avoid_turn_direction * self.avoid_angular_speed)
            )

        elif self.avoid_state == AV_COUNTERSTEER:
            if self.counter_steer_deadline is not None and now >= self.counter_steer_deadline:
                self._begin_recovery()
                return
            self.cmd_pub.publish(
                self._twist(self.counter_steer_linear_speed,
                            -self.avoid_turn_direction * self.counter_steer_angular_speed)
            )

        elif self.avoid_state == AV_RECOVERING:
            if self._recovery_complete():
                self._finish_recovery()
                return

            if self.current_yaw is not None and self.target_yaw is not None:
                error = self._angle_error(self.target_yaw, self.current_yaw)
                angular = self._clamp(self.recovery_heading_kp * error,
                                      self.recovery_max_angular_speed)
                if abs(error) > self.heading_tolerance and abs(angular) < self.recovery_min_angular_speed:
                    angular = math.copysign(self.recovery_min_angular_speed, error)
                linear = self.recovery_turn_linear_speed
                if abs(error) <= self.recovery_in_place_angle:
                    linear = self.recovery_linear_speed
                self.cmd_pub.publish(self._twist(linear, angular))
            else:
                self.cmd_pub.publish(
                    self._twist(self.recovery_linear_speed,
                                -self.avoid_turn_direction * self.recovery_angular_speed)
                )

    def _begin_counter_steer(self):
        if self.avoid_state != AV_AVOIDING:
            return
        now = self._now_ros()
        avoid = 0.0
        if self.avoid_started_time is not None:
            avoid = (now - self.avoid_started_time).nanoseconds / 1e9
        self.last_avoid_duration = avoid

        dur = max(self.counter_steer_min_duration,
                  min(self.counter_steer_max_duration, avoid * self.counter_steer_duration_scale))
        self.avoid_state = AV_COUNTERSTEER
        self.avoid_clear_since = None
        self.counter_steer_deadline = now + Duration(seconds=dur)
        self.recovery_deadline = None
        self.get_logger().info(f'counter-steer started, duration={dur:.2f}s')

    def _begin_recovery(self):
        if self.avoid_state not in (AV_AVOIDING, AV_COUNTERSTEER):
            return
        self.avoid_state = AV_RECOVERING
        self.avoid_clear_since = None
        self.counter_steer_deadline = None
        dur = max(0.15, min(self.recovery_timeout, self.last_avoid_duration * self.recovery_duration_scale))
        self.recovery_deadline = self._now_ros() + Duration(seconds=dur)
        self.get_logger().info(f'recovery started, duration={dur:.2f}s')

    def _recovery_complete(self):
        now = self._now_ros()
        if self.current_yaw is not None and self.target_yaw is not None:
            if abs(self._angle_error(self.target_yaw, self.current_yaw)) <= self.heading_tolerance:
                return True
        if self.recovery_deadline is not None and now >= self.recovery_deadline:
            return True
        return False

    def _finish_recovery(self):
        self.avoid_state = AV_FORWARD
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.get_logger().info('recovery complete, back to forward')
        self.publish_state(self.state)

    def destroy_node(self):
        self.cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleReturnNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
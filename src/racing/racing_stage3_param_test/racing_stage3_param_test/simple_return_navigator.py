"""Stage3 简化返程：左转 → 搜索墙角 → 对齐墙面 → 垂直逼近到 (0.3, 0.7)"""

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
from racing_common.racing_logger import RacingLogger

# 新状态机：搜索 → 对齐 → 垂直逼近
S_IDLE = 'idle'
S_INITIAL_TURN = 'initial_turn'
S_CORNER_SEARCH = 'corner_search'
S_ALIGN_TO_WALL = 'align_to_wall'
S_PERPENDICULAR_APPROACH = 'perpendicular_approach'
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

        self.logger = RacingLogger(self, 'stage3_param_test', session_title='stage3_simple_return')

        self.state = S_IDLE
        self.phase = 1
        self.target_yaw = None
        self.current_yaw = None

        # 墙角跟踪
        self.corner_x = None
        self.corner_y = None
        self.corner_theta = None  # 墙角相对车头角度（弧度）
        self.corner_lost_since = None
        self._detect_counter = 0
        
        # 对齐状态
        self.aligned_since = None  # 首次满足对齐条件的时间

        self.avoid_state = AV_FORWARD
        self.avoid_turn_direction = 0.0
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None

        self.get_logger().info(f'creating cmd publisher on topic: {self.cmd_topic}')
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.create_subscription(Int32, self.phase_topic, self._phase_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)

        self.latest_scan = None
        self._start_time = None
        self.publish_state(S_IDLE)
        self.create_timer(0.05, self._control_loop)
        self.logger.startup(
            f'stage3 simple return navigator ready | '
            f'cmd_topic={self.cmd_topic} imu={self.imu_topic} scan={self.scan_topic}'
        )

    def _declare_params(self):
        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('state_topic', 'stage3_state')
        self.declare_parameter('test_direction', 'clockwise')

        self.declare_parameter('turn_angular_speed', 1.0)
        self.declare_parameter('turn_heading_tolerance_deg', 3.0)
        self.declare_parameter('turn_kp', 2.0)

        # 搜索阶段
        self.declare_parameter('search_linear_speed', 0.15)
        self.declare_parameter('search_max_duration', 15.0)
        
        # 对齐阶段
        self.declare_parameter('align_linear_speed', 0.15)
        self.declare_parameter('align_kp', 2.0)
        self.declare_parameter('align_max_angular_speed', 0.6)
        self.declare_parameter('align_tolerance_deg', 10.0)
        self.declare_parameter('align_hold_duration', 0.5)
        
        # 垂直逼近阶段
        self.declare_parameter('approach_target_cx', 0.3)
        self.declare_parameter('approach_target_cy', 0.7)
        self.declare_parameter('approach_heading_kp', 2.0)
        self.declare_parameter('approach_dist_kp', 0.5)
        self.declare_parameter('approach_stop_tolerance_xy', 0.05)
        self.declare_parameter('approach_stop_tolerance_theta_deg', 5.0)
        self.declare_parameter('approach_max_linear_speed', 0.25)
        self.declare_parameter('approach_min_linear_speed', 0.10)
        self.declare_parameter('corner_lost_timeout', 1.5)
        self.declare_parameter('corner_lost_blind_speed', 0.12)

        self.declare_parameter('douglas_peucker_epsilon', 0.05)
        self.declare_parameter('corner_min_line_length', 0.30)
        self.declare_parameter('corner_angle_tolerance_deg', 15.0)
        self.declare_parameter('corner_min_distance', 0.30)
        self.declare_parameter('corner_max_distance', 2.5)
        self.declare_parameter('corner_min_points', 8)

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

        self.search_linear_speed = float(self.get_parameter('search_linear_speed').value)
        self.search_max_duration = float(self.get_parameter('search_max_duration').value)
        
        self.align_linear_speed = float(self.get_parameter('align_linear_speed').value)
        self.align_kp = float(self.get_parameter('align_kp').value)
        self.align_max_angular_speed = float(self.get_parameter('align_max_angular_speed').value)
        self.align_tolerance = math.radians(float(self.get_parameter('align_tolerance_deg').value))
        self.align_hold_duration = float(self.get_parameter('align_hold_duration').value)
        
        self.approach_target_cx = float(self.get_parameter('approach_target_cx').value)
        self.approach_target_cy = float(self.get_parameter('approach_target_cy').value)
        self.approach_heading_kp = float(self.get_parameter('approach_heading_kp').value)
        self.approach_dist_kp = float(self.get_parameter('approach_dist_kp').value)
        self.approach_stop_tolerance_xy = float(self.get_parameter('approach_stop_tolerance_xy').value)
        self.approach_stop_tolerance_theta = math.radians(float(self.get_parameter('approach_stop_tolerance_theta_deg').value))
        self.approach_max_linear_speed = float(self.get_parameter('approach_max_linear_speed').value)
        self.approach_min_linear_speed = float(self.get_parameter('approach_min_linear_speed').value)
        self.corner_lost_timeout = float(self.get_parameter('corner_lost_timeout').value)
        self.corner_lost_blind_speed = float(self.get_parameter('corner_lost_blind_speed').value)

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
                self.logger.feedback(f'phase=3 detected, will start after 0.5s delay')

    def _start_mission(self):
        self.state = S_INITIAL_TURN
        self.publish_state('initial_turn')
        turn_deg = 75.0 if self.test_direction == 'clockwise' else -105.0
        if self.current_yaw is not None:
            self.target_yaw = self._normalize_angle(self.current_yaw + math.radians(turn_deg))
        else:
            self.target_yaw = math.radians(turn_deg)
        self.logger.mission(
            f'direction={self.test_direction} turn={turn_deg}° '
            f'target_yaw={math.degrees(self.target_yaw):.1f}°'
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
        elif self.state == S_CORNER_SEARCH:
            self._do_corner_search()
        elif self.state == S_ALIGN_TO_WALL:
            self._do_align_to_wall()
        elif self.state == S_PERPENDICULAR_APPROACH:
            self._do_perpendicular_approach()

    # ── Phase 1：初始转向 ──

    def _do_initial_turn(self):
        target = self.target_yaw
        error = self._angle_error(target, self.current_yaw)

        if abs(error) < self.turn_heading_tolerance:
            self.state = S_CORNER_SEARCH
            self.publish_state('corner_search')
            self.logger.segment(
                f'turn done yaw={math.degrees(target):.1f}° → corner_search'
            )
            self.cmd_pub.publish(self._twist())
            return

        angular = self._clamp(self.turn_kp * error, self.turn_angular_speed)
        if abs(angular) < 0.2:
            angular = math.copysign(0.2, error)

        self.logger.telemetry('turn',
            f'cur={math.degrees(self.current_yaw):.1f}° '
            f'tgt={math.degrees(target):.1f}° '
            f'err={math.degrees(error):.1f}° '
            f'ang={angular:.2f}'
        )
        cmd = self._twist(0.2, angular)
        self.cmd_pub.publish(cmd)

    # ── Phase 2：搜索墙角 ──

    def _do_corner_search(self):
        """慢速前进，检测墙角，记录角度 θ"""
        self._detect_counter += 1
        
        # 每 5 帧检测一次
        if self._detect_counter % 5 == 0 and self.latest_scan is not None:
            self._check_cone_obstacle(self.latest_scan)
            if self.avoid_state != AV_FORWARD:
                self._do_avoidance()
                return

            detected, cx, cy, theta, conf = self.corner_detector.detect(self.latest_scan)
            if detected:
                theta_deg = math.degrees(theta)
                
                # ═══ 过滤：只接受车头前方 ±30° 的墙角 ═══
                if abs(theta) > math.radians(30.0):
                    self.logger.feedback(f'corner outside front cone (θ={theta_deg:.1f}°), skip')
                    self.cmd_pub.publish(self._twist(self.search_linear_speed, 0.0))
                    return
                
                # ═══ 通过验证 → 记录 ═══
                self.corner_x = cx
                self.corner_y = cy
                self.corner_theta = theta
                self.corner_lost_since = None
                
                self.logger.segment(
                    f'corner found! ({cx:.2f}, {cy:.2f}) θ={theta_deg:.1f}°'
                )
                
                # 如果墙角已经在正前方（|θ| < 15°），直接进垂直逼近
                if abs(theta) < math.radians(15.0):
                    self.state = S_PERPENDICULAR_APPROACH
                    self.publish_state('perpendicular_approach')
                    self.logger.segment(
                        f'corner already aligned θ={theta_deg:.1f}° → perp_approach'
                    )
                else:
                    self.state = S_ALIGN_TO_WALL
                    self.publish_state('align_to_wall')
                    self.aligned_since = None
                    self.logger.segment(
                        f'corner at θ={theta_deg:.1f}° → align_to_wall'
                    )
                return
        
        # 直走搜索
        self.cmd_pub.publish(self._twist(self.search_linear_speed, 0.0))

    # ── Phase 3：对齐墙面（边走边转，使 θ→0）──

    def _do_align_to_wall(self):
        """边走边转，让墙角角度 θ → 0（正前方）"""
        self._detect_counter += 1
        
        if self._detect_counter % 5 == 0 and self.latest_scan is not None:
            self._check_cone_obstacle(self.latest_scan)
            if self.avoid_state != AV_FORWARD:
                self._do_avoidance()
                return

            detected, cx, cy, theta, conf = self.corner_detector.detect(self.latest_scan)
            if detected:
                self.corner_x = cx
                self.corner_y = cy
                self.corner_theta = theta
                self.corner_lost_since = None
                
                theta_deg = math.degrees(theta)
                
                # 判断是否对齐（|θ| < align_tolerance 且持续 align_hold_duration）
                if abs(theta) < self.align_tolerance:
                    now = self._now_sec()
                    if self.aligned_since is None:
                        self.aligned_since = now
                    hold_time = now - self.aligned_since
                    
                    if hold_time >= self.align_hold_duration:
                        self.state = S_PERPENDICULAR_APPROACH
                        self.publish_state('perpendicular_approach')
                        self.logger.segment(
                            f'aligned! θ={theta_deg:.1f}° hold={hold_time:.2f}s → perp_approach'
                        )
                        return
                else:
                    self.aligned_since = None
            else:
                # 墙角丢失 → 回到搜索
                if self.corner_lost_since is None:
                    self.corner_lost_since = self._now_sec()
                lost_time = self._now_sec() - self.corner_lost_since
                if lost_time > self.corner_lost_timeout:
                    self.state = S_CORNER_SEARCH
                    self.publish_state('corner_search')
                    self.logger.segment(f'corner lost {lost_time:.1f}s → back to search')
                    self.corner_x = None
                    self.corner_y = None
                    self.corner_theta = None
                    return
        
        # 控制：边走边转
        if self.corner_theta is not None:
            angular = self._clamp(-self.align_kp * self.corner_theta, self.align_max_angular_speed)
            self.logger.telemetry('align',
                f'θ={math.degrees(self.corner_theta):.1f}° ang={angular:.2f}'
            )
        else:
            # 盲走保底
            angular = 0.0
        
        self.cmd_pub.publish(self._twist(self.align_linear_speed, angular))

    # ── Phase 4：垂直逼近（保持 θ≈0，距离控制）──

    def _do_perpendicular_approach(self):
        """保持墙角在正前方 θ≈0，垂直逼近到 (0.3, 0.7)"""
        self._detect_counter += 1
        
        if self._detect_counter % 5 == 0 and self.latest_scan is not None:
            self._check_cone_obstacle(self.latest_scan)
            if self.avoid_state != AV_FORWARD:
                self._do_avoidance()
                return

            detected, cx, cy, theta, conf = self.corner_detector.detect(self.latest_scan)
            if detected:
                self.corner_x = cx
                self.corner_y = cy
                self.corner_theta = theta
                self.corner_lost_since = None
                
                # 每帧打印
                self.get_logger().info(f'[CORNER] ({cx:.3f}, {cy:.3f}) θ={math.degrees(theta):.1f}°')
            else:
                # 墙角丢失 → 盲走保底或回搜索
                if self.corner_lost_since is None:
                    self.corner_lost_since = self._now_sec()
                lost_time = self._now_sec() - self.corner_lost_since
                if lost_time > self.corner_lost_timeout:
                    # 超时 → 回搜索
                    self.state = S_CORNER_SEARCH
                    self.publish_state('corner_search')
                    self.logger.segment(f'corner lost {lost_time:.1f}s → back to search')
                    self.corner_x = None
                    self.corner_y = None
                    self.corner_theta = None
                    return
        
        cx = self.corner_x
        cy = self.corner_y
        theta = self.corner_theta
        
        if cx is None or cy is None or theta is None:
            # 盲走保底
            self.cmd_pub.publish(self._twist(self.corner_lost_blind_speed, 0.0))
            return
        
        # 停止条件：位置 + 角度
        if (abs(cx - self.approach_target_cx) < self.approach_stop_tolerance_xy and
            abs(cy - self.approach_target_cy) < self.approach_stop_tolerance_xy and
            abs(theta) < self.approach_stop_tolerance_theta):
            self.state = S_FINISH
            self.publish_state('complete')
            self.cmd_pub.publish(self._twist())
            self.logger.mission(
                f'complete! corner at ({cx:.3f}, {cy:.3f}) θ={math.degrees(theta):.1f}°'
            )
            return
        
        # 控制：保持 θ≈0（航向修正）+ 距离控制
        angular = self._clamp(-self.approach_heading_kp * theta, 0.6)
        
        # 距离误差（cy - target_cy）
        dist_error = cy - self.approach_target_cy
        linear = self._clamp(self.approach_dist_kp * dist_error, self.approach_max_linear_speed)
        linear = max(self.approach_min_linear_speed, linear)
        
        self.cmd_pub.publish(self._twist(linear, angular))
        self.logger.telemetry('perp_approach',
            f'cx={cx:.3f} cy={cy:.3f} θ={math.degrees(theta):.1f}° '
            f'lin={linear:.2f} ang={angular:.2f}'
        )

    # ── Stage1 4态避障 ──

    def _check_cone_obstacle(self, scan_msg):
        cone = self.obstacle_classifier.find_nearest_cone(
            scan_msg, self.phase1_window_min_x,
            self.phase1_window_max_x, self.phase1_window_half_width,
        )
        if cone is not None and cone[2] < self.cone_avoidance_trigger:
            cx, cy, dist = cone
            danger_angle = math.degrees(math.atan2(cy, max(cx, 1e-6)))
            self.logger.corner_avoid(
                f'cone ({cx:.2f},{cy:.2f}) dist={dist:.2f} angle={danger_angle:.1f}°'
            )
            self._begin_avoidance(danger_angle)

    def _begin_avoidance(self, danger_angle):
        self.avoid_state = AV_AVOIDING
        self.avoid_turn_direction = -1.0 if danger_angle > 0.0 else 1.0
        self.avoid_started_time = self._now_ros()
        self.avoid_clear_since = None
        self.avoid_entry_yaw = self.current_yaw
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.publish_state('avoiding')
        self.logger.corner_avoid(
            f'avoid start turn_dir={self.avoid_turn_direction:.1f} danger_angle={danger_angle:.1f}°'
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
        self.logger.corner_avoid(f'counter-steer dur={dur:.2f}s')

    def _begin_recovery(self):
        if self.avoid_state not in (AV_AVOIDING, AV_COUNTERSTEER):
            return
        self.avoid_state = AV_RECOVERING
        self.avoid_clear_since = None
        self.counter_steer_deadline = None
        dur = max(0.15, min(self.recovery_timeout, self.last_avoid_duration * self.recovery_duration_scale))
        self.recovery_deadline = self._now_ros() + Duration(seconds=dur)
        self.logger.corner_avoid(f'recovery dur={dur:.2f}s')

    def _recovery_complete(self):
        now = self._now_ros()
        if self.current_yaw is not None and self.target_yaw is not None:
            if abs(self._angle_error(self.target_yaw, self.current_yaw)) <= self.turn_heading_tolerance:
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
        self.logger.corner_avoid('recovery complete, back to current state')
        # 不改变状态，继续当前阶段（search/align/approach）

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
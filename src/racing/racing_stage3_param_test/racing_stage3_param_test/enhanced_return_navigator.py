"""Stage3 增强返程导航：Pure Pursuit + Stage1 4态避障
- 状态机: idle → armed → running(PurePursuit) → align_yaw → complete
- running 时可中断为 avoiding → countersteer → recovering → running
- 输出 /cmd_vel（phase3_external_control=true 时 competition_controller 会隐让）
- 独立测试：由 phase3_test_trigger 发布 phase=3 触发
"""

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String


class EnhancedReturnNavigator(Node):
    def __init__(self):
        super().__init__('enhanced_return_navigator')

        self._declare_params()
        self._read_params()

        # ── 路点（map 全局坐标系）──
        self.return_waypoints = self._parse_waypoints_json(
            self.return_waypoints_json, 'return_waypoints_json',
            self.pursuit_linear_speed,
        )

        # ── 状态 ──
        self.phase = 1
        self.mission_active = False
        self.mission_finished = False
        self.start_after_time = None

        # 位姿（/odom_combined，map 坐标系）
        self.current_position = None
        self.current_yaw = None
        self.odom_frame_id = 'odom'

        # 路径状态
        self.path_started_at = None
        self.path_index = 0

        # 激光扫描
        self.latest_scan = None

        # ── 避障状态 ──
        self.avoid_state = 'forward'
        self.avoid_turn_direction = 0.0
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None

        # ── Pub/Sub ──
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, qos_latched)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)

        self.create_subscription(Int32, self.phase_topic, self._phase_cb, qos_latched)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)

        self._publish_state('idle')
        self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.get_logger().info(
            f'enhanced return navigator ready | waypoints={len(self.return_waypoints)} '
            f'cmd={self.cmd_topic} odom={self.odom_topic}'
        )

    # ══════════════ 参数 ══════════════

    def _declare_params(self):
        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('state_topic', 'stage3_state')
        self.declare_parameter('feedback_topic', 'competition_feedback')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('start_delay_sec', 0.5)

        # ── 路点 ──
        self.declare_parameter('return_waypoints_json', '[]')
        self.declare_parameter('waypoint_tolerance', 0.20)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('goal_yaw_tolerance_deg', 8.0)
        self.declare_parameter('path_timeout_sec', 60.0)

        # ── Pure Pursuit ──
        self.declare_parameter('pursuit_linear_speed', 0.18)
        self.declare_parameter('pursuit_lookahead_m', 0.45)
        self.declare_parameter('pursuit_heading_stop_deg', 70.0)
        self.declare_parameter('pursuit_turn_kp', 1.8)
        self.declare_parameter('pursuit_turn_linear_speed', 0.08)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('min_angular_speed', 0.45)

        # ── 避障（4态，同 Stage1）──
        self.declare_parameter('avoid_linear_speed', 0.10)
        self.declare_parameter('avoid_angular_speed', 0.80)
        self.declare_parameter('avoid_min_duration_sec', 0.70)
        self.declare_parameter('avoid_clear_hold_sec', 0.25)
        self.declare_parameter('avoid_min_turn_angle_deg', 18.0)
        self.declare_parameter('avoid_safe_distance', 0.50)
        self.declare_parameter('avoid_clear_distance', 0.65)
        self.declare_parameter('emergency_stop_distance', 0.22)

        self.declare_parameter('recovery_linear_speed', 0.12)
        self.declare_parameter('recovery_turn_linear_speed', 0.08)
        self.declare_parameter('recovery_angular_speed', 0.75)
        self.declare_parameter('recovery_heading_kp', 2.4)
        self.declare_parameter('recovery_max_angular_speed', 1.1)
        self.declare_parameter('recovery_min_angular_speed', 0.5)
        self.declare_parameter('recovery_in_place_angle_deg', 8.0)
        self.declare_parameter('recovery_timeout', 2.5)
        self.declare_parameter('recovery_duration_scale', 0.9)

        self.declare_parameter('counter_steer_linear_speed', 0.10)
        self.declare_parameter('counter_steer_angular_speed', 0.95)
        self.declare_parameter('counter_steer_duration_scale', 1.35)
        self.declare_parameter('counter_steer_min_duration_sec', 0.45)
        self.declare_parameter('counter_steer_max_duration_sec', 1.20)

        # ── 激光聚类窗口（避障用）──
        self.declare_parameter('window_min_x', 0.18)
        self.declare_parameter('window_max_x', 0.85)
        self.declare_parameter('window_half_width', 0.22)
        self.declare_parameter('cluster_gap_tolerance', 0.12)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('min_cluster_width', 0.06)
        self.declare_parameter('max_cluster_width', 0.40)
        self.declare_parameter('min_valid_range', 0.15)

        # ── map→odom 偏移参数（同 Stage2 map_overlay，直接传参避免 TF 依赖）──
        self.declare_parameter('test_direction', 'clockwise')
        self.declare_parameter('map_to_odom_x', 0.0)
        self.declare_parameter('map_to_odom_y', 0.0)
        self.declare_parameter('map_to_odom_yaw', 0.0)

    def _read_params(self):
        self.phase_topic = str(self.get_parameter('phase_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.feedback_topic = str(self.get_parameter('feedback_topic').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)

        self.return_waypoints_json = self.get_parameter('return_waypoints_json').value
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.goal_yaw_tolerance = math.radians(float(self.get_parameter('goal_yaw_tolerance_deg').value))
        self.path_timeout_sec = float(self.get_parameter('path_timeout_sec').value)

        self.pursuit_linear_speed = float(self.get_parameter('pursuit_linear_speed').value)
        self.pursuit_lookahead = float(self.get_parameter('pursuit_lookahead_m').value)
        self.pursuit_heading_stop = math.radians(float(self.get_parameter('pursuit_heading_stop_deg').value))
        self.pursuit_turn_kp = float(self.get_parameter('pursuit_turn_kp').value)
        self.pursuit_turn_linear = float(self.get_parameter('pursuit_turn_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.min_angular = float(self.get_parameter('min_angular_speed').value)

        self.avoid_linear_speed = float(self.get_parameter('avoid_linear_speed').value)
        self.avoid_angular_speed = float(self.get_parameter('avoid_angular_speed').value)
        self.avoid_min_duration = float(self.get_parameter('avoid_min_duration_sec').value)
        self.avoid_clear_hold = float(self.get_parameter('avoid_clear_hold_sec').value)
        self.avoid_min_turn_angle = math.radians(float(self.get_parameter('avoid_min_turn_angle_deg').value))
        self.avoid_safe_dist = float(self.get_parameter('avoid_safe_distance').value)
        self.avoid_clear_dist = float(self.get_parameter('avoid_clear_distance').value)
        self.emergency_stop_dist = float(self.get_parameter('emergency_stop_distance').value)

        self.recovery_linear = float(self.get_parameter('recovery_linear_speed').value)
        self.recovery_turn_linear = float(self.get_parameter('recovery_turn_linear_speed').value)
        self.recovery_angular = float(self.get_parameter('recovery_angular_speed').value)
        self.recovery_kp = float(self.get_parameter('recovery_heading_kp').value)
        self.recovery_max_angular = float(self.get_parameter('recovery_max_angular_speed').value)
        self.recovery_min_angular = float(self.get_parameter('recovery_min_angular_speed').value)
        self.recovery_in_place = math.radians(float(self.get_parameter('recovery_in_place_angle_deg').value))
        self.recovery_timeout = float(self.get_parameter('recovery_timeout').value)
        self.recovery_duration_scale = float(self.get_parameter('recovery_duration_scale').value)

        self.counter_linear = float(self.get_parameter('counter_steer_linear_speed').value)
        self.counter_angular = float(self.get_parameter('counter_steer_angular_speed').value)
        self.counter_duration_scale = float(self.get_parameter('counter_steer_duration_scale').value)
        self.counter_min_dur = float(self.get_parameter('counter_steer_min_duration_sec').value)
        self.counter_max_dur = float(self.get_parameter('counter_steer_max_duration_sec').value)

        self.window_min_x = float(self.get_parameter('window_min_x').value)
        self.window_max_x = float(self.get_parameter('window_max_x').value)
        self.window_half_width = float(self.get_parameter('window_half_width').value)
        self.cluster_gap = float(self.get_parameter('cluster_gap_tolerance').value)
        self.min_cluster_pts = int(self.get_parameter('min_cluster_points').value)
        self.min_cluster_w = float(self.get_parameter('min_cluster_width').value)
        self.max_cluster_w = float(self.get_parameter('max_cluster_width').value)
        self.min_range = float(self.get_parameter('min_valid_range').value)
        self.test_direction = str(self.get_parameter('test_direction').value)
        self.map_odom_x = float(self.get_parameter('map_to_odom_x').value)
        self.map_odom_y = float(self.get_parameter('map_to_odom_y').value)
        self.map_odom_yaw = float(self.get_parameter('map_to_odom_yaw').value)

    # ══════════════ 工具 ══════════════

    @staticmethod
    def _normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _angle_error(target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    @staticmethod
    def _clamp(v, limit):
        return max(-limit, min(limit, v))

    @staticmethod
    def _twist(linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        return t

    @staticmethod
    def _quat_to_yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_feedback(self, text):
        self.feedback_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _publish_state(self, text):
        self.state_pub.publish(String(data=text))

    def _parse_waypoints_json(self, raw, param_name, default_speed):
        try:
            data = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().error(f'{param_name} invalid, empty waypoints')
            return []
        if not isinstance(data, list):
            return []
        wps = []
        for i, item in enumerate(data):
            if not isinstance(item, dict):
                continue
            yaw_d = item.get('yaw_deg')
            wps.append({
                'x': float(item.get('x', 0.0)),
                'y': float(item.get('y', 0.0)),
                'speed': float(item.get('speed', default_speed)),
                'yaw_deg': None if yaw_d is None else float(yaw_d),
                'desc': str(item.get('description', f'wp_{i}')),
            })
        return wps

    # ══════════════ 回调 ══════════════

    def _phase_cb(self, msg):
        prev = self.phase
        self.phase = int(msg.data)
        if prev == 3 and self.phase != 3:
            self._reset_mission()
        elif prev != 3 and self.phase == 3:
            self._arm_mission()

    def _odom_cb(self, msg):
        raw_x = float(msg.pose.pose.position.x)
        raw_y = float(msg.pose.pose.position.y)
        raw_yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        self.odom_frame_id = msg.header.frame_id or self.odom_frame_id
        # 同 map_overlay 静态 TF：map_pos = R(odom_pos) + translation
        cos_y = math.cos(self.map_odom_yaw)
        sin_y = math.sin(self.map_odom_yaw)
        self.current_position = (
            cos_y * raw_x - sin_y * raw_y + self.map_odom_x,
            sin_y * raw_x + cos_y * raw_y + self.map_odom_y,
        )
        self.current_yaw = self._normalize_angle(raw_yaw + self.map_odom_yaw)

    def _scan_cb(self, msg):
        self.latest_scan = msg

    # ══════════════ 任务生命周期 ══════════════

    def _arm_mission(self):
        self.mission_active = False
        self.mission_finished = False
        self.path_started_at = None
        self.path_index = 0
        self.avoid_state = 'forward'
        self.start_after_time = self._now_sec() + self.start_delay_sec
        init_yaw_deg = 180.0 if self.test_direction == 'clockwise' else 0.0
        self.current_yaw = math.radians(init_yaw_deg)
        self._publish_state('armed')
        self._publish_feedback(
            f'phase=3 detected, direction={self.test_direction}, '
            f'initial_yaw={init_yaw_deg:.0f}°'
        )

    def _reset_mission(self):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = False
        self.start_after_time = None
        self.path_started_at = None
        self.path_index = 0
        self.avoid_state = 'forward'
        self._publish_state('idle')

    def _start_mission(self):
        if self.current_position is None or self.current_yaw is None:
            self.get_logger().warn('no odom yet, cannot start (waiting for /odom_combined)')
            return
        if not self.return_waypoints:
            self._publish_feedback('no waypoints configured, cannot start')
            self._fail_mission('no return waypoints')
            return
        self.mission_active = True
        self.path_started_at = self._now_sec()
        self.path_index = 0
        self._publish_state('running')
        self._publish_feedback(
            f'return started, {len(self.return_waypoints)} waypoints (map coords), '
            f'current=({self.current_position[0]:.2f},{self.current_position[1]:.2f}) '
            f'yaw={math.degrees(self.current_yaw):.1f}°'
        )
        self.get_logger().info(f'mission_active=True, will publish cmd_vel now')

    def _finish_mission(self):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self._publish_state('complete')
        self._publish_feedback('return complete, reached P point')

    def _fail_mission(self, reason):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self._publish_state('failed')
        self._publish_feedback(f'return failed: {reason}')

    # ══════════════ 主控制循环 ══════════════

    def _control_loop(self):
        if self.phase != 3 or self.mission_finished:
            return

        now = self._now_sec()
        if not self.mission_active:
            if self.start_after_time is None or now < self.start_after_time:
                return
            self._start_mission()
            return

        # 1. 紧急停车
        if self._check_emergency_stop():
            return

        # 2. 避障检测（仅在 running 态）
        if self.avoid_state == 'forward' and self.latest_scan is not None:
            self._check_obstacle()

        # 3. 若在避障态 → 运行避障
        if self.avoid_state != 'forward':
            self._run_avoidance()
            return

        # 4. 正常 Pure Pursuit
        self._run_pursuit()

    def _check_emergency_stop(self):
        if self.latest_scan is None:
            return False
        min_dist = float('inf')
        for i, d in enumerate(self.latest_scan.ranges):
            if math.isinf(d) or math.isnan(d) or d < self.min_range:
                continue
            if d < min_dist:
                min_dist = d
        if min_dist <= self.emergency_stop_dist:
            self.stop_robot()
            self._publish_feedback(f'emergency stop, closest={min_dist:.2f}m')
            return True
        return False

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # ══════════════ Pure Pursuit（map 坐标系）══════════════


    def _advance_waypoint(self, pose):
        """跳过已到达的中间路点"""
        while self.path_index < len(self.return_waypoints) - 1:
            wp = self.return_waypoints[self.path_index]
            dist = math.hypot(wp['x'] - pose[0], wp['y'] - pose[1])
            if dist > self.waypoint_tolerance:
                return
            self.path_index += 1

    def _run_pursuit(self):
        now = self._now_sec()
        if self.path_started_at is not None and now - self.path_started_at > self.path_timeout_sec:
            self._fail_mission('path timeout')
            return

        if self.current_position is None or self.current_yaw is None:
            self.get_logger().warn_throttle(2.0, 'no pose, waiting for odom')
            return

        self._advance_waypoint(self.current_position)
        final_wp = self.return_waypoints[-1]
        final_dist = math.hypot(final_wp['x'] - self.current_position[0],
                                final_wp['y'] - self.current_position[1])

        # ── 到达最终目标 → 对齐航向 ──
        if self.path_index >= len(self.return_waypoints) - 1 and final_dist <= self.goal_tolerance:
            target_yaw_deg = final_wp.get('yaw_deg')
            if target_yaw_deg is not None:
                target_yaw = math.radians(target_yaw_deg)
                yaw_err = self._angle_error(target_yaw, self.current_yaw)
                if abs(yaw_err) > self.goal_yaw_tolerance:
                    angular = self._clamp(self.pursuit_turn_kp * yaw_err, self.max_angular)
                    if abs(angular) < self.min_angular:
                        angular = math.copysign(self.min_angular, yaw_err)
                    self._publish_state('align_yaw')
                    self.cmd_pub.publish(self._twist(self.pursuit_turn_linear, angular))
                    return
            self._finish_mission()
            return

        # ── 追踪当前路点（map → body 坐标系）──
        wp = self.return_waypoints[self.path_index]
        dx = wp['x'] - self.current_position[0]
        dy = wp['y'] - self.current_position[1]
        # 转到车体坐标系
        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)
        tx = cos_y * dx + sin_y * dy
        ty = -sin_y * dx + cos_y * dy
        target_dist = math.hypot(tx, ty)
        heading_err = math.atan2(ty, tx if abs(tx) > 1e-6 else 1e-6)

        self._publish_state(wp['desc'])

        # ── 航向偏差过大 → 边转边走 ──
        if tx <= 0.0 or abs(heading_err) > self.pursuit_heading_stop:
            angular = self._clamp(self.pursuit_turn_kp * heading_err, self.max_angular)
            if abs(angular) < self.min_angular:
                angular = math.copysign(self.min_angular, heading_err)
            self.cmd_pub.publish(self._twist(self.pursuit_turn_linear, angular))
            return

        # ── Pure Pursuit 曲率控制 ──
        pursuit_dist = max(target_dist, self.pursuit_lookahead)
        curvature = 0.0 if pursuit_dist <= 1e-6 else 2.0 * ty / (pursuit_dist * pursuit_dist)
        speed = min(float(wp.get('speed', self.pursuit_linear_speed)), self.pursuit_linear_speed)
        if target_dist < self.pursuit_lookahead:
            speed *= max(0.4, target_dist / max(self.pursuit_lookahead, 1e-6))
        angular = self._clamp(speed * curvature, self.max_angular)
        if abs(angular) < self.min_angular:
            angular = math.copysign(self.min_angular, heading_err)
        self.cmd_pub.publish(self._twist(speed, angular))

    # ══════════════ 避障（4态，同 Stage1）══════════════

    def _clusters_in_window(self, scan_msg):
        clusters = []
        cur = []
        prev_pt = None
        for i, d in enumerate(scan_msg.ranges):
            if math.isinf(d) or math.isnan(d) or d < self.min_range:
                if cur:
                    clusters.append(cur)
                    cur = []
                prev_pt = None
                continue
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = d * math.cos(angle)
            y = d * math.sin(angle)
            if x < self.window_min_x or x > self.window_max_x or abs(y) > self.window_half_width:
                if cur:
                    clusters.append(cur)
                    cur = []
                prev_pt = None
                continue
            pt = (x, y, d)
            if prev_pt is None or math.hypot(prev_pt[0] - pt[0], prev_pt[1] - pt[1]) <= self.cluster_gap:
                cur.append(pt)
            else:
                if cur:
                    clusters.append(cur)
                cur = [pt]
            prev_pt = pt
        if cur:
            clusters.append(cur)
        return clusters

    def _find_nearest_obstacle(self, scan_msg):
        clusters = self._clusters_in_window(scan_msg)
        best = None
        for c in clusters:
            if len(c) < self.min_cluster_pts:
                continue
            nearest = min(p[2] for p in c)
            cx = sum(p[0] for p in c) / len(c)
            cy = sum(p[1] for p in c) / len(c)
            span = math.hypot(c[0][0] - c[-1][0], c[0][1] - c[-1][1])
            if span < self.min_cluster_w or span > self.max_cluster_w:
                continue
            if best is None or nearest < best['dist']:
                best = {'dist': nearest, 'cx': cx, 'cy': cy, 'danger_deg': math.degrees(math.atan2(cy, max(cx, 1e-6)))}
        return best

    def _check_obstacle(self):
        obs = self._find_nearest_obstacle(self.latest_scan)
        if obs is not None and obs['dist'] < self.avoid_safe_dist:
            self._begin_avoidance(obs['danger_deg'])

    def _begin_avoidance(self, danger_deg):
        self.avoid_state = 'avoiding'
        self.avoid_turn_direction = -1.0 if danger_deg > 0.0 else 1.0
        self.avoid_started_time = self.get_clock().now()
        self.avoid_clear_since = None
        self.avoid_entry_yaw = self.current_yaw
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self._publish_state('avoiding')
        self._publish_feedback(f'avoid start dir={self.avoid_turn_direction:.1f} danger={danger_deg:.1f}°')

    def _run_avoidance(self):
        now = self.get_clock().now()

        if self.avoid_state == 'avoiding':
            avoid_elapsed = 0.0
            if self.avoid_started_time is not None:
                avoid_elapsed = (now - self.avoid_started_time).nanoseconds / 1e9

            turned = False
            if self.current_yaw is not None and self.avoid_entry_yaw is not None:
                turned = abs(self._angle_error(self.current_yaw, self.avoid_entry_yaw)) >= self.avoid_min_turn_angle

            cone_clear = True
            if self.latest_scan is not None:
                obs = self._find_nearest_obstacle(self.latest_scan)
                if obs is not None and obs['dist'] < self.avoid_clear_dist:
                    cone_clear = False

            if cone_clear and self.avoid_clear_since is None:
                self.avoid_clear_since = now
            elif not cone_clear:
                self.avoid_clear_since = None

            clear_elapsed = 0.0
            if self.avoid_clear_since is not None:
                clear_elapsed = (now - self.avoid_clear_since).nanoseconds / 1e9

            if avoid_elapsed >= self.avoid_min_duration and clear_elapsed >= self.avoid_clear_hold and turned:
                self._begin_counter_steer()
                return

            self.cmd_pub.publish(self._twist(self.avoid_linear_speed,
                                             self.avoid_turn_direction * self.avoid_angular_speed))

        elif self.avoid_state == 'countersteering':
            if self.counter_steer_deadline is not None and now >= self.counter_steer_deadline:
                self._begin_recovery()
                return
            self.cmd_pub.publish(self._twist(self.counter_linear,
                                             -self.avoid_turn_direction * self.counter_angular))

        elif self.avoid_state == 'recovering':
            if self._recovery_complete():
                self._finish_recovery()
                return

            if self.current_yaw is not None:
                target_yaw = self.avoid_entry_yaw if self.avoid_entry_yaw is not None else 0.0
                error = self._angle_error(target_yaw, self.current_yaw)
                angular = self._clamp(self.recovery_kp * error, self.recovery_max_angular)
                if abs(error) > self.recovery_in_place and abs(angular) < self.recovery_min_angular:
                    angular = math.copysign(self.recovery_min_angular, error)
                linear = self.recovery_turn_linear if abs(error) > self.recovery_in_place else self.recovery_linear
                self.cmd_pub.publish(self._twist(linear, angular))
            else:
                self.cmd_pub.publish(self._twist(self.recovery_linear,
                                                 -self.avoid_turn_direction * self.recovery_angular))

    def _begin_counter_steer(self):
        if self.avoid_state != 'avoiding':
            return
        now = self.get_clock().now()
        avoid_dur = 0.0
        if self.avoid_started_time is not None:
            avoid_dur = (now - self.avoid_started_time).nanoseconds / 1e9
        self.last_avoid_duration = avoid_dur
        dur = max(self.counter_min_dur, min(self.counter_max_dur, avoid_dur * self.counter_duration_scale))
        self.avoid_state = 'countersteering'
        self.avoid_clear_since = None
        self.counter_steer_deadline = now + Duration(seconds=dur)
        self.recovery_deadline = None

    def _begin_recovery(self):
        if self.avoid_state not in ('avoiding', 'countersteering'):
            return
        self.avoid_state = 'recovering'
        self.avoid_clear_since = None
        self.counter_steer_deadline = None
        dur = max(0.15, min(self.recovery_timeout, self.last_avoid_duration * self.recovery_duration_scale))
        self.recovery_deadline = self.get_clock().now() + Duration(seconds=dur)

    def _recovery_complete(self):
        now = self.get_clock().now()
        if self.current_yaw is not None and self.avoid_entry_yaw is not None:
            if abs(self._angle_error(self.avoid_entry_yaw, self.current_yaw)) <= self.recovery_in_place:
                return True
        if self.recovery_deadline is not None and now >= self.recovery_deadline:
            return True
        return False

    def _finish_recovery(self):
        self.avoid_state = 'forward'
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self._publish_state('running')

    def destroy_node(self):
        try:
            self.log.close()
            if rclpy.ok():
                self.cmd_pub.publish(Twist())
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EnhancedReturnNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
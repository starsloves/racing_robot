"""Stage3 增强返程导航：Pure Pursuit + Stage1 4态避障 + A* 全局规划
- 状态机: idle → armed → running(PurePursuit + A*) → align_yaw → complete
- running 时可中断为 avoiding → countersteer → recovering → running
- A* 路径规划避开地图中的禁区（黑色障碍物）
- 输出 /cmd_vel（phase3_external_control=true 时 competition_controller 会隐让）
- 独立测试：由 phase3_test_trigger 发布 phase=3 触发
"""

import json
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from racing_common.racing_logger import RacingLogger
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String

from .global_path_planner import GlobalPathPlanner


class EnhancedReturnNavigator(Node):
    def __init__(self):
        super().__init__('enhanced_return_navigator')

        self._declare_params()
        self._read_params()

        # ── 日志文件 ~/dev_ws/log/enhanced_return_test/latest.log ──
        self.log = RacingLogger(
            self, log_subdir='enhanced_return_test',
            log_filename='latest.log', session_title='Stage3 enhanced return',
        )

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
        self._settled_start = None  # 近目标停滞计时
        self._filtered_heading_err = 0.0  # 航向误差低通滤波

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

        # ── A* 全局路径规划器 ──
        self.global_planner = None
        if self.use_global_planner:
            planner_config = {
                'map_topic': self.map_topic,
                'scan_topic': self.scan_topic,
                'global_frame_id': self.global_frame_id,
                'planner_downsample': self.planner_downsample,
                'planner_occupied_threshold': self.planner_occupied_threshold,
                'planner_unknown_is_occupied': self.planner_unknown_is_occupied,
                'planner_obstacle_inflation_m': self.planner_obstacle_inflation_m,
                'planner_dynamic_obstacle_box_size_m': self.planner_dynamic_obstacle_box_size_m,
                'planner_dynamic_obstacle_inflation_m': self.planner_dynamic_obstacle_inflation_m,
                'planner_dynamic_obstacle_range_m': self.planner_dynamic_obstacle_range_m,
                'planner_replan_period_sec': self.planner_replan_period_sec,
            }
            self.global_planner = GlobalPathPlanner(self, planner_config)
            self.log.startup('A* global planner enabled')

        self._publish_state('idle')
        self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.log.startup(
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
        self.declare_parameter('waypoint_tolerance', 0.18)
        self.declare_parameter('goal_box_x_min', 0.1)
        self.declare_parameter('goal_box_x_max', 0.3)
        self.declare_parameter('goal_box_y_min', 0.1)
        self.declare_parameter('goal_box_y_max', 0.2)
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

        # ── A* 全局路径规划（避开地图禁区）──
        self.declare_parameter('use_global_planner', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('planner_downsample', 4)
        self.declare_parameter('planner_occupied_threshold', 50)
        self.declare_parameter('planner_unknown_is_occupied', False)
        self.declare_parameter('planner_obstacle_inflation_m', 0.14)
        self.declare_parameter('planner_dynamic_obstacle_box_size_m', 0.25)
        self.declare_parameter('planner_dynamic_obstacle_inflation_m', 0.04)
        self.declare_parameter('planner_dynamic_obstacle_range_m', 0.7)
        self.declare_parameter('planner_replan_period_sec', 0.25)

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
        self.goal_box_x_min = float(self.get_parameter('goal_box_x_min').value)
        self.goal_box_x_max = float(self.get_parameter('goal_box_x_max').value)
        self.goal_box_y_min = float(self.get_parameter('goal_box_y_min').value)
        self.goal_box_y_max = float(self.get_parameter('goal_box_y_max').value)
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

        self.use_global_planner = bool(self.get_parameter('use_global_planner').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.global_frame_id = str(self.get_parameter('global_frame_id').value)
        self.planner_downsample = int(self.get_parameter('planner_downsample').value)
        self.planner_occupied_threshold = int(self.get_parameter('planner_occupied_threshold').value)
        self.planner_unknown_is_occupied = bool(self.get_parameter('planner_unknown_is_occupied').value)
        self.planner_obstacle_inflation_m = float(self.get_parameter('planner_obstacle_inflation_m').value)
        self.planner_dynamic_obstacle_box_size_m = float(self.get_parameter('planner_dynamic_obstacle_box_size_m').value)
        self.planner_dynamic_obstacle_inflation_m = float(self.get_parameter('planner_dynamic_obstacle_inflation_m').value)
        self.planner_dynamic_obstacle_range_m = float(self.get_parameter('planner_dynamic_obstacle_range_m').value)
        self.planner_replan_period_sec = float(self.get_parameter('planner_replan_period_sec').value)

        self.test_direction = str(self.get_parameter('test_direction').value)
        self.map_odom_x = float(self.get_parameter('map_to_odom_x').value)
        self.map_odom_y = float(self.get_parameter('map_to_odom_y').value)
        self.map_odom_yaw = float(self.get_parameter('map_to_odom_yaw').value)

    # ══════════════ 工具 ══════════════

    def _in_goal_region(self, x, y):
        """判断 (x,y) 是否在目标矩形区域内"""
        return (self.goal_box_x_min <= x <= self.goal_box_x_max and
                self.goal_box_y_min <= y <= self.goal_box_y_max)

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

    @staticmethod
    def _select_lookahead_point(path_points, lookahead_distance):
        """
        从 A* 路径中选择预瞄点（距离起点 lookahead_distance 处）

        Args:
            path_points: list of tuple(x, y), 路径点列表
            lookahead_distance: float, 预瞄距离（m）

        Returns:
            tuple(x, y): 预瞄点坐标，如果路径太短则返回终点
        """
        if not path_points:
            return None
        if len(path_points) == 1:
            return path_points[0]

        traveled = 0.0
        previous_point = path_points[0]
        for point in path_points[1:]:
            traveled += math.hypot(point[0] - previous_point[0], point[1] - previous_point[1])
            if traveled >= lookahead_distance:
                return point
            previous_point = point

        return path_points[-1]

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_feedback(self, text):
        self.feedback_pub.publish(String(data=text))
        self.log.feedback(text)
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
        self._settled_start = None
        self.avoid_state = 'forward'
        self.start_after_time = self._now_sec() + self.start_delay_sec
        init_yaw_deg = 180.0 if self.test_direction == 'clockwise' else 0.0
        self.current_yaw = math.radians(init_yaw_deg)
        self._publish_state('armed')
        self.log.mission(
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
        self._settled_start = None
        self.avoid_state = 'forward'
        self._publish_state('idle')

    def _start_mission(self):
        if self.current_position is None or self.current_yaw is None:
            self.log.warn('ODOM', 'no odom yet, cannot start (waiting for /odom_combined)')
            return
        if not self.return_waypoints:
            self._publish_feedback('no waypoints configured, cannot start')
            self._fail_mission('no return waypoints')
            return
        self.mission_active = True
        self.path_started_at = self._now_sec()
        self.path_index = 0
        self._publish_state('running')
        self.log.mission(
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
        sys.stderr.write('\n=== ★ STAGE3 RETURN COMPLETE ★ ===\n\n')

    def _fail_mission(self, reason):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self._publish_state('failed')
        self._publish_feedback(f'return failed: {reason}')
        sys.stderr.write(f'\n=== ✗ STAGE3 RETURN FAILED: {reason} ===\n\n')

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
            self.log.timeout(f'path timeout after {self.path_timeout_sec}s')
            self._fail_mission('path timeout')
            return

        if self.current_position is None or self.current_yaw is None:
            self.log.warn('ODOM', 'no pose, waiting for odom')
            return

        self._advance_waypoint(self.current_position)
        
        x, y = self.current_position[0], self.current_position[1]
        in_goal = self._in_goal_region(x, y)
        
        # ── 到达矩形目标区域 → 完成 ──
        if self.path_index >= len(self.return_waypoints) - 1 and in_goal:
            self.log.segment(f'reached goal region at ({x:.3f}, {y:.3f})')
            self._finish_mission()
            return

        # ── 航向控制（heading controller，替代 Pure Pursuit curvature）──
        wp = self.return_waypoints[self.path_index]
        target_x, target_y = wp['x'], wp['y']

        # ── A* 全局路径规划（避开地图禁区）──
        if self.use_global_planner and self.global_planner is not None:
            planned_points = self.global_planner.plan_path(
                self.current_position,
                (target_x, target_y),
                self._now_sec()
            )
            if planned_points is None:
                # 地图未加载，等待
                self.log.warn('PLANNER', 'waiting for map')
                self._publish_state('planner_waiting_for_map')
                self.cmd_pub.publish(self._twist(0.0, 0.0))
                return
            if not planned_points:
                # 路径规划失败，停车
                self.log.warn('PLANNER', f'blocked: no path from ({self.current_position[0]:.2f},{self.current_position[1]:.2f}) to ({target_x:.2f},{target_y:.2f})')
                self._publish_state('planner_blocked')
                self.cmd_pub.publish(self._twist(0.0, 0.0))
                return
            
            # 选择预瞄点
            lookahead_point = self._select_lookahead_point(planned_points, self.pursuit_lookahead)
            if lookahead_point is not None:
                target_x, target_y = lookahead_point
                self.log.telemetry('ASTAR', f'path_pts={len(planned_points)} lookahead=({target_x:.2f},{target_y:.2f})')

        # ── 计算目标相对车体坐标 ──
        dx = target_x - self.current_position[0]
        dy = target_y - self.current_position[1]
        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)
        tx = cos_y * dx + sin_y * dy
        ty = -sin_y * dx + cos_y * dy
        target_dist = math.hypot(tx, ty)
        heading_err = math.atan2(ty, tx if abs(tx) > 1e-6 else 1e-6)

        # ── 航向误差低通滤波（消除抖动）──
        alpha = 0.3
        self._filtered_heading_err = alpha * heading_err + (1.0 - alpha) * self._filtered_heading_err
        heading_err = self._filtered_heading_err

        self._publish_state(wp['desc'])

        # 航向比例控制（平滑输出，无 min_angular 强制）
        angular = self._clamp(self.pursuit_turn_kp * heading_err, self.max_angular)
        if abs(angular) < 1e-4:
            angular = 0.0

        # 速度：大偏差慢行，小偏差全速；接近目标减速
        speed = self.pursuit_linear_speed
        if abs(heading_err) > math.radians(30.0):
            speed = self.pursuit_turn_linear
        elif abs(heading_err) > math.radians(5.0):
            speed = self.pursuit_linear_speed * 0.5
        # 距离减速：dist < 0.30 时线性减速；dist < 0.15 时蠕行
        if target_dist < 0.30:
            speed = min(speed, 0.04 + 0.14 * (target_dist / 0.30))
            speed = max(speed, 0.04)
        if target_dist < 0.15:
            speed = min(speed, 0.03 + 0.06 * (target_dist / 0.15))
            speed = max(speed, 0.03)

        self.log.telemetry('HEADING',
            f'dist={target_dist:.2f} err={math.degrees(heading_err):.1f}° '
            f'spd={speed:.2f} ang={angular:.2f} '
            f'pos=({self.current_position[0]:.2f},{self.current_position[1]:.2f})'
        )
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

    def _classify_cluster(self, cluster):
        """
        分类聚类：'obstacle' 或 'wall'
        
        墙壁特征（根据实测数据调整）：
        - 点数 > 40（实测墙壁 50-60 点）
        - 角度跨度 > 30°（实测墙壁 39-42°）
        - 点密集连续（相邻点平均间距 < 0.012 m，实测墙壁 0.007-0.010m）
        
        Returns:
            str: 'obstacle' 或 'wall'
        """
        if len(cluster) < 40:
            return 'obstacle'
        
        # 角度跨度
        angles = [math.atan2(pt[1], pt[0]) for pt in cluster]
        angle_span = max(angles) - min(angles)
        if angle_span < math.radians(30.0):
            return 'obstacle'
        
        # 连续性：相邻点平均间距（墙壁非常密集）
        distances = []
        for i in range(len(cluster) - 1):
            d = math.hypot(cluster[i+1][0] - cluster[i][0], 
                           cluster[i+1][1] - cluster[i][1])
            distances.append(d)
        
        if distances:
            avg_gap = sum(distances) / len(distances)
            if avg_gap > 0.012:
                return 'obstacle'
        
        return 'wall'

    def _find_nearest_obstacle(self, scan_msg):
        """返回最近的障碍物（墙壁已过滤）"""
        clusters = self._clusters_in_window(scan_msg)
        best = None
        wall_count = 0
        obstacle_count = 0
        
        for c in clusters:
            if len(c) < self.min_cluster_pts:
                continue
            
            # 计算聚类属性用于分类和日志
            nearest = min(p[2] for p in c)
            angles = [math.atan2(pt[1], pt[0]) for pt in c]
            angle_span = max(angles) - min(angles)
            angle_span_deg = math.degrees(angle_span)
            xs = [pt[0] for pt in c]
            ys = [pt[1] for pt in c]
            width = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            
            # 计算平均间距
            distances = []
            for i in range(len(c) - 1):
                d = math.hypot(c[i+1][0] - c[i][0], c[i+1][1] - c[i][1])
                distances.append(d)
            avg_gap = sum(distances) / len(distances) if distances else 0.0
            
            # 分类聚类
            cluster_type = self._classify_cluster(c)
            
            # 日志：显示所有聚类的属性（用于调试）
            self.log.telemetry('CLUSTER', 
                f'type={cluster_type} pts={len(c)} span={angle_span_deg:.1f}° '
                f'width={width:.2f}m gap={avg_gap:.3f}m dist={nearest:.2f}m')
            
            # 墙壁 → 跳过，不纳入障碍物
            if cluster_type == 'wall':
                wall_count += 1
                continue
            
            # 障碍物 → 纳入检测
            obstacle_count += 1
            cx = sum(p[0] for p in c) / len(c)
            cy = sum(p[1] for p in c) / len(c)
            span = math.hypot(c[0][0] - c[-1][0], c[0][1] - c[-1][1])
            if span < self.min_cluster_w or span > self.max_cluster_w:
                continue
            
            obs = {
                'dist': nearest, 
                'cx': cx, 
                'cy': cy, 
                'danger_deg': math.degrees(math.atan2(cy, max(cx, 1e-6))),
                'type': 'obstacle',
                'pts': len(c),
            }
            
            if best is None or nearest < best['dist']:
                best = obs
        
        if wall_count > 0 or obstacle_count > 0:
            self.log.telemetry('FILTER_SUMMARY', 
                f'walls={wall_count} obstacles={obstacle_count}')
        
        return best

    def _check_obstacle(self):
        # 距离终点 0.6m 内不判定避障（终点附近墙壁和边界多）
        if self.current_position is not None:
            goal_center_x = (self.goal_box_x_min + self.goal_box_x_max) / 2.0
            goal_center_y = (self.goal_box_y_min + self.goal_box_y_max) / 2.0
            dist_to_goal = math.hypot(
                self.current_position[0] - goal_center_x,
                self.current_position[1] - goal_center_y
            )
            if dist_to_goal < 0.60:
                self.log.telemetry('AVOID_SKIP', f'near goal ({dist_to_goal:.2f}m), skip obstacle check')
                return
        
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
        self.log.corner_avoid(f'start dir={self.avoid_turn_direction:.1f} danger={danger_deg:.1f}°')
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
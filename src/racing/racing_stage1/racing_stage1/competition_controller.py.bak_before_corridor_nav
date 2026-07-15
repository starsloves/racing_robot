import math

import rclpy
import sys
import os

# 添加 voice_api 路径以支持 CN-TTS
voice_api_path = os.path.join(os.path.dirname(__file__), '../../../voice_driver')
if voice_api_path not in sys.path:
    sys.path.insert(0, voice_api_path)

try:
    from voice_api import CnTtsPlayer
    CN_TTS_AVAILABLE = True
except ImportError:
    CN_TTS_AVAILABLE = False

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32, String

from racing_common.obstacle_marker_publisher import ObstacleMarkerPublisher
from racing_common.racing_logger import RacingLogger


class CompetitionController(Node):
    def __init__(self):
        super().__init__('competition_controller')

        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('stage2_cmd_topic', '/stage2_cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom_combined')  # map 坐标系
        self.declare_parameter('qr_result_topic', 'qr_scan_result')
        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('task_topic', 'competition_qr_task')
        self.declare_parameter('stage2_state_topic', 'stage2_state')
        self.declare_parameter('stage3_state_topic', 'stage3_state')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('blind_linear_speed', 0.2)
        self.declare_parameter('blind_angular_speed', 0.0)
        self.declare_parameter('avoid_linear_speed', 0.1)
        self.declare_parameter('avoid_angular_speed', 0.8)
        self.declare_parameter('avoid_min_duration_sec', 0.7)
        self.declare_parameter('avoid_clear_hold_sec', 0.25)
        self.declare_parameter('avoid_min_turn_angle_deg', 18.0)
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('clear_distance', 0.65)
        self.declare_parameter('scan_angle_deg', 45.0)
        self.declare_parameter('phase1_window_min_x', 0.18)
        self.declare_parameter('phase1_window_max_x', 0.85)
        self.declare_parameter('phase1_window_half_width', 0.22)
        self.declare_parameter('phase1_cluster_gap_tolerance', 0.12)
        self.declare_parameter('phase1_min_cluster_points', 3)
        self.declare_parameter('phase1_min_cluster_width', 0.06)
        self.declare_parameter('phase1_max_cluster_width', 0.40)
        self.declare_parameter('phase1_emergency_min_x', 0.08)
        self.declare_parameter('phase1_emergency_max_x', 0.45)
        self.declare_parameter('phase1_emergency_half_width', 0.12)
        self.declare_parameter('phase1_emergency_min_points', 2)
        self.declare_parameter('min_valid_range', 0.15)
        self.declare_parameter('recovery_linear_speed', 0.12)
        self.declare_parameter('recovery_turn_linear_speed', 0.08)
        self.declare_parameter('recovery_angular_speed', 0.75)
        self.declare_parameter('counter_steer_linear_speed', 0.10)
        self.declare_parameter('counter_steer_angular_speed', 0.95)
        self.declare_parameter('counter_steer_duration_scale', 1.35)
        self.declare_parameter('counter_steer_min_duration_sec', 0.45)
        self.declare_parameter('counter_steer_max_duration_sec', 1.20)
        self.declare_parameter('recovery_heading_kp', 2.4)
        self.declare_parameter('recovery_max_angular_speed', 1.1)
        self.declare_parameter('recovery_min_angular_speed', 0.5)
        self.declare_parameter('recovery_in_place_angle_deg', 8.0)
        self.declare_parameter('heading_tolerance_deg', 6.0)
        self.declare_parameter('recovery_timeout', 2.5)
        self.declare_parameter('recovery_duration_scale', 0.9)
        self.declare_parameter('stage2_cmd_timeout', 0.5)
        self.declare_parameter('transition_stop_duration', 0.0)
        self.declare_parameter('phase2_obstacle_override', False)
        self.declare_parameter('phase2_emergency_stop_distance', 0.22)
        self.declare_parameter('phase3_external_control', True)
        self.declare_parameter('phase3_emergency_stop_distance', 0.22)
        self.declare_parameter('enable_backing', True)
        self.declare_parameter('back_target_x', 2.0)
        self.declare_parameter('back_linear_speed', -0.15)
        self.declare_parameter('back_angular_kp', 1.8)
        self.declare_parameter('back_position_tolerance', 0.15)
        self.declare_parameter('back_path_sample_distance', 0.20)
        self.declare_parameter('back_timeout_sec', 10.0)
        self.declare_parameter('back_align_yaw_deg', 90.0)
        self.declare_parameter('back_align_tolerance_deg', 5.0)
        self.declare_parameter('back_align_timeout_sec', 5.0)

        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.stage2_cmd_topic = self.get_parameter('stage2_cmd_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.qr_result_topic = self.get_parameter('qr_result_topic').value
        self.phase_topic = self.get_parameter('phase_topic').value
        self.task_topic = self.get_parameter('task_topic').value
        self.stage2_state_topic = self.get_parameter('stage2_state_topic').value
        self.stage3_state_topic = self.get_parameter('stage3_state_topic').value
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.blind_linear_speed = float(self.get_parameter('blind_linear_speed').value)
        self.blind_angular_speed = float(self.get_parameter('blind_angular_speed').value)
        self.avoid_linear_speed = float(self.get_parameter('avoid_linear_speed').value)
        self.avoid_angular_speed = float(self.get_parameter('avoid_angular_speed').value)
        self.avoid_min_duration_sec = float(self.get_parameter('avoid_min_duration_sec').value)
        self.avoid_clear_hold_sec = float(self.get_parameter('avoid_clear_hold_sec').value)
        self.avoid_min_turn_angle_rad = math.radians(
            float(self.get_parameter('avoid_min_turn_angle_deg').value)
        )
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        self.scan_angle_deg = float(self.get_parameter('scan_angle_deg').value)
        self.phase1_window_min_x = float(self.get_parameter('phase1_window_min_x').value)
        self.phase1_window_max_x = float(self.get_parameter('phase1_window_max_x').value)
        self.phase1_window_half_width = float(self.get_parameter('phase1_window_half_width').value)
        self.phase1_cluster_gap_tolerance = float(self.get_parameter('phase1_cluster_gap_tolerance').value)
        self.phase1_min_cluster_points = int(self.get_parameter('phase1_min_cluster_points').value)
        self.phase1_min_cluster_width = float(self.get_parameter('phase1_min_cluster_width').value)
        self.phase1_max_cluster_width = float(self.get_parameter('phase1_max_cluster_width').value)
        self.phase1_emergency_min_x = float(self.get_parameter('phase1_emergency_min_x').value)
        self.phase1_emergency_max_x = float(self.get_parameter('phase1_emergency_max_x').value)
        self.phase1_emergency_half_width = float(self.get_parameter('phase1_emergency_half_width').value)
        self.phase1_emergency_min_points = int(self.get_parameter('phase1_emergency_min_points').value)
        self.min_valid_range = float(self.get_parameter('min_valid_range').value)
        self.recovery_linear_speed = float(self.get_parameter('recovery_linear_speed').value)
        self.recovery_turn_linear_speed = float(self.get_parameter('recovery_turn_linear_speed').value)
        self.recovery_angular_speed = float(self.get_parameter('recovery_angular_speed').value)
        self.counter_steer_linear_speed = float(self.get_parameter('counter_steer_linear_speed').value)
        self.counter_steer_angular_speed = float(self.get_parameter('counter_steer_angular_speed').value)
        self.counter_steer_duration_scale = float(self.get_parameter('counter_steer_duration_scale').value)
        self.counter_steer_min_duration_sec = float(self.get_parameter('counter_steer_min_duration_sec').value)
        self.counter_steer_max_duration_sec = float(self.get_parameter('counter_steer_max_duration_sec').value)
        self.recovery_heading_kp = float(self.get_parameter('recovery_heading_kp').value)
        self.recovery_max_angular_speed = float(self.get_parameter('recovery_max_angular_speed').value)
        self.recovery_min_angular_speed = float(self.get_parameter('recovery_min_angular_speed').value)
        self.recovery_in_place_angle_rad = math.radians(
            float(self.get_parameter('recovery_in_place_angle_deg').value)
        )
        self.heading_tolerance_rad = math.radians(float(self.get_parameter('heading_tolerance_deg').value))
        self.recovery_timeout = float(self.get_parameter('recovery_timeout').value)
        self.recovery_duration_scale = float(self.get_parameter('recovery_duration_scale').value)
        self.stage2_cmd_timeout = float(self.get_parameter('stage2_cmd_timeout').value)
        self.transition_stop_duration = float(self.get_parameter('transition_stop_duration').value)
        self.phase2_obstacle_override = bool(self.get_parameter('phase2_obstacle_override').value)
        self.phase2_emergency_stop_distance = float(self.get_parameter('phase2_emergency_stop_distance').value)
        self.phase3_external_control = bool(self.get_parameter('phase3_external_control').value)
        self.phase3_emergency_stop_distance = float(self.get_parameter('phase3_emergency_stop_distance').value)
        self.enable_backing = bool(self.get_parameter('enable_backing').value)
        self.back_target_x = float(self.get_parameter('back_target_x').value)
        self.back_linear_speed = float(self.get_parameter('back_linear_speed').value)
        self.back_angular_kp = float(self.get_parameter('back_angular_kp').value)
        self.back_position_tolerance = float(self.get_parameter('back_position_tolerance').value)
        self.back_path_sample_distance = float(self.get_parameter('back_path_sample_distance').value)
        self.back_timeout_sec = float(self.get_parameter('back_timeout_sec').value)
        self.back_align_yaw_rad = math.radians(float(self.get_parameter('back_align_yaw_deg').value))
        self.back_align_tolerance_rad = math.radians(float(self.get_parameter('back_align_tolerance_deg').value))
        self.back_align_timeout_sec = float(self.get_parameter('back_align_timeout_sec').value)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.phase_pub = self.create_publisher(Int32, self.phase_topic, latched_qos)
        self.task_pub = self.create_publisher(String, self.task_topic, latched_qos)

        self.create_subscription(LaserScan, self.scan_topic, self.lidar_callback, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(String, self.qr_result_topic, self.qr_callback, 10)
        self.create_subscription(Twist, self.stage2_cmd_topic, self.stage2_cmd_callback, 10)
        self.create_subscription(String, self.stage2_state_topic, self.stage2_state_callback, 10)
        self.create_subscription(String, self.stage3_state_topic, self.stage3_state_callback, 10)

        self.phase = 1
        self.mission_finished = False
        self.obstacle_found = False
        self.closest_obstacle_distance = float('inf')
        self.avoid_cmd = Twist()
        self.phase1_motion_state = 'forward'
        self.current_yaw = None
        self.desired_heading = None
        self.avoid_turn_direction = 0.0
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.recovery_uses_heading = False
        self.warned_missing_heading = False
        self.latest_stage2_cmd = Twist()
        self.latest_stage2_cmd_time = None
        self.transition_end_time = None
        self.qr_task = ''
        self.stage2_state = 'idle'
        self.stage3_state = 'idle'

        # 路径记录与后退状态
        self.current_odom = None
        self.path_record = []  # [(x, y, yaw), ...]
        self.last_recorded_position = None
        self.backing_started_time = None
        self.backing_path_index = -1
        self.aligning_started_time = None

        # RacingLogger：日志文件 ~/dev_ws/log/competition_stage1/latest.log
        self.log = RacingLogger(
            self, log_subdir='competition_stage1',
            log_filename='latest.log', session_title='Stage1 competition',
        )
        # CN-TTS 语音播报初始化
        self.tts_player = None
        if CN_TTS_AVAILABLE:
            try:
                self.tts_player = CnTtsPlayer(port='/dev/ttyS1', baudrate=9600, logger=self.get_logger())
                self.log.startup('CN-TTS 语音模块已初始化 (port=/dev/ttyS1, baud=9600)')
                self.get_logger().info('[VOICE] CN-TTS 模块已初始化')
            except Exception as e:
                self.log.warn('VOICE', f'CN-TTS 初始化失败: {e}')
                self.get_logger().warn(f'[VOICE] CN-TTS 初始化失败: {e}')
        else:
            self.log.warn('VOICE', 'CN-TTS 模块不可用（voice_api 未安装）')
            self.get_logger().warn('[VOICE] CN-TTS 模块不可用')


        # 障碍物可视化（rviz2 调试用）
        self.obstacle_markers = ObstacleMarkerPublisher(
            self, topic='/stage1_obstacle_markers', frame_id='base_link', radius=0.13
        )
        self._phase1_last_clusters = []  # 缓存上一帧的聚类结果

        self.publish_phase()
        self.create_timer(1.0 / max(control_rate_hz, 1.0), self.control_loop)

        self.log.startup('competition controller ready: phase1 blind drive, phase2 corridor, phase3 return-to-p')

    def quaternion_to_yaw(self, orientation):
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def angle_error(self, target_angle, current_angle):
        return self.normalize_angle(target_angle - current_angle)

    def create_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        return msg

    def publish_phase(self):
        self.phase_pub.publish(Int32(data=self.phase))

    def begin_phase_transition(self, target_phase, reason):
        if self.phase == target_phase:
            return

        self.phase = target_phase
        self.publish_phase()
        self.stop_robot()
        self.latest_stage2_cmd = Twist()
        self.latest_stage2_cmd_time = None
        if self.transition_stop_duration > 0.0:
            self.transition_end_time = self.get_clock().now() + Duration(seconds=self.transition_stop_duration)
        else:
            self.transition_end_time = None
        self.log.mission(f'phase {target_phase}: {reason}')

    def clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def imu_callback(self, msg):
        self.current_yaw = self.quaternion_to_yaw(msg.orientation)
        if self.phase == 1 and self.desired_heading is None:
            self.desired_heading = self.current_yaw
            self.log.config(f'phase1 heading locked at {math.degrees(self.desired_heading):.1f} deg')

    def odom_callback(self, msg):
        """订阅 /odom_combined 用于路径记录（位置）"""
        self.current_odom = msg
        
        # Phase 1 前进时记录路径（只在 forward/avoiding/countersteering/recovering 时记录）
        # 位置用 odom (x, y)，角度用 IMU (self.current_yaw)
        if self.phase == 1 and self.enable_backing and self.phase1_motion_state in ('forward', 'avoiding', 'countersteering', 'recovering'):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            # 使用纯 IMU 角度，而不是 odom 的 orientation（避免融合后角度不一致）
            yaw = self.current_yaw if self.current_yaw is not None else 0.0
            
            # 采样：距离上次记录点 >= sample_distance 才记录
            if self.last_recorded_position is None:
                self.path_record.append((x, y, yaw))
                self.last_recorded_position = (x, y)
            else:
                dist = math.hypot(x - self.last_recorded_position[0], y - self.last_recorded_position[1])
                if dist >= self.back_path_sample_distance:
                    self.path_record.append((x, y, yaw))
                    self.last_recorded_position = (x, y)
                    # 限制最大路径点数防止内存占用
                    if len(self.path_record) > 1000:
                        self.path_record.pop(0)

    def begin_avoidance(self, danger_angle):
        self.phase1_motion_state = 'avoiding'
        self.avoid_turn_direction = -1.0 if danger_angle > 0.0 else 1.0
        self.avoid_started_time = self.get_clock().now()
        self.avoid_clear_since = None
        self.avoid_entry_yaw = self.current_yaw
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.recovery_uses_heading = False

        if self.desired_heading is None and self.current_yaw is not None:
            self.desired_heading = self.current_yaw

        self.log.feedback(
            f'avoid start dir={self.avoid_turn_direction:.0f} '
            f'danger_angle={danger_angle:.0f}°'
        )

    def begin_counter_steer(self):
        if self.phase1_motion_state != 'avoiding':
            return

        now = self.get_clock().now()
        avoid_duration = 0.0
        if self.avoid_started_time is not None:
            avoid_duration = (now - self.avoid_started_time).nanoseconds / 1e9
        self.last_avoid_duration = avoid_duration

        counter_duration = max(
            self.counter_steer_min_duration_sec,
            avoid_duration * self.counter_steer_duration_scale,
        )
        counter_duration = min(counter_duration, self.counter_steer_max_duration_sec)

        self.phase1_motion_state = 'countersteering'
        self.avoid_clear_since = None
        self.counter_steer_deadline = now + Duration(seconds=counter_duration)
        self.recovery_deadline = None
        self.recovery_uses_heading = False

        self.log.feedback(f'countersteer duration={counter_duration:.2f}s')

    def begin_recovery(self):
        if self.phase1_motion_state not in ('avoiding', 'countersteering'):
            return

        now = self.get_clock().now()
        avoid_duration = self.last_avoid_duration
        if avoid_duration <= 0.0 and self.avoid_started_time is not None:
            avoid_duration = (now - self.avoid_started_time).nanoseconds / 1e9

        self.phase1_motion_state = 'recovering'
        self.avoid_clear_since = None
        self.counter_steer_deadline = None
        self.recovery_uses_heading = self.current_yaw is not None and self.desired_heading is not None
        if self.recovery_uses_heading:
            heading_error = abs(self.angle_error(self.desired_heading, self.current_yaw))
            estimated_duration = max(
                0.6,
                heading_error / max(self.recovery_max_angular_speed, 0.1) * 1.6,
            )
            self.recovery_deadline = now + Duration(seconds=min(self.recovery_timeout, estimated_duration))
        else:
            recovery_duration = max(0.15, avoid_duration * self.recovery_duration_scale)
            recovery_duration = min(recovery_duration, self.recovery_timeout)
            self.recovery_deadline = now + Duration(seconds=recovery_duration)
            if not self.warned_missing_heading:
                self.warned_missing_heading = True
                self.log.warn('HEADING', 'imu heading unavailable, recovery falls back to timed reverse steering')

        deadline_sec = (self.recovery_deadline - now).nanoseconds / 1e9
        self.log.feedback(
            f'recovery start, uses_heading={self.recovery_uses_heading}, '
            f'deadline={deadline_sec:.2f}s'
        )

    def recovery_complete(self):
        now = self.get_clock().now()
        if self.recovery_uses_heading and self.current_yaw is not None and self.desired_heading is not None:
            if abs(self.angle_error(self.desired_heading, self.current_yaw)) <= self.heading_tolerance_rad:
                return True

        if self.recovery_deadline is not None and now >= self.recovery_deadline:
            return True

        return False

    def finish_recovery(self):
        self.phase1_motion_state = 'forward'
        self.avoid_started_time = None
        self.avoid_clear_since = None
        self.avoid_entry_yaw = None
        self.last_avoid_duration = 0.0
        self.counter_steer_deadline = None
        self.recovery_deadline = None
        self.recovery_uses_heading = False

        self.log.feedback('recovery complete, return to forward')

    def avoid_turn_reached(self):
        if self.current_yaw is None or self.avoid_entry_yaw is None:
            return True
        return abs(self.angle_error(self.current_yaw, self.avoid_entry_yaw)) >= self.avoid_min_turn_angle_rad

    def point_distance_xy(self, point_a, point_b):
        return math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])

    def collect_points_in_window(self, scan_msg, min_x, max_x, half_width):
        clusters = []
        current_cluster = []
        previous_point = None

        for index, distance in enumerate(scan_msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance < self.min_valid_range:
                if current_cluster:
                    clusters.append(current_cluster)
                    current_cluster = []
                previous_point = None
                continue

            angle = scan_msg.angle_min + index * scan_msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            if x < min_x or x > max_x or abs(y) > half_width:
                if current_cluster:
                    clusters.append(current_cluster)
                    current_cluster = []
                previous_point = None
                continue

            point = (x, y, distance)
            if previous_point is None or self.point_distance_xy(previous_point, point) <= self.phase1_cluster_gap_tolerance:
                current_cluster.append(point)
            else:
                if current_cluster:
                    clusters.append(current_cluster)
                current_cluster = [point]
            previous_point = point

        if current_cluster:
            clusters.append(current_cluster)

        return clusters

    def describe_cluster(self, cluster):
        nearest_distance = min(point[2] for point in cluster)
        center_x = sum(point[0] for point in cluster) / len(cluster)
        center_y = sum(point[1] for point in cluster) / len(cluster)
        span = self.point_distance_xy(cluster[0], cluster[-1])
        danger_angle_deg = math.degrees(math.atan2(center_y, max(center_x, 1e-6)))
        return {
            'distance': nearest_distance,
            'span': span,
            'danger_angle_deg': danger_angle_deg,
        }

    def find_phase1_forward_obstacle(self, scan_msg):
        clusters = self.collect_points_in_window(
            scan_msg,
            self.phase1_window_min_x,
            self.phase1_window_max_x,
            self.phase1_window_half_width,
        )

        # 发布所有聚类的可视化（rviz2 调试用）
        if clusters:
            self.obstacle_markers.publish_from_clusters(clusters, color='red')
            self._phase1_last_clusters = clusters
        else:
            self.obstacle_markers.clear()
            self._phase1_last_clusters = []

        nearest_obstacle = None
        for cluster in clusters:
            if len(cluster) < self.phase1_min_cluster_points:
                continue

            obstacle = self.describe_cluster(cluster)
            if obstacle['span'] < self.phase1_min_cluster_width:
                continue
            if obstacle['span'] > self.phase1_max_cluster_width:
                continue

            if nearest_obstacle is None or obstacle['distance'] < nearest_obstacle['distance']:
                nearest_obstacle = obstacle

        return nearest_obstacle

    def find_phase1_emergency_obstacle(self, scan_msg):
        clusters = self.collect_points_in_window(
            scan_msg,
            self.phase1_emergency_min_x,
            self.phase1_emergency_max_x,
            self.phase1_emergency_half_width,
        )

        nearest_obstacle = None
        for cluster in clusters:
            if len(cluster) < self.phase1_emergency_min_points:
                continue

            obstacle = self.describe_cluster(cluster)
            if nearest_obstacle is None or obstacle['distance'] < nearest_obstacle['distance']:
                nearest_obstacle = obstacle

        return nearest_obstacle

    def handle_phase1_lidar(self, scan_msg):
        if self.phase1_motion_state == 'avoiding':
            obstacle = self.find_phase1_emergency_obstacle(scan_msg)
        else:
            obstacle = self.find_phase1_forward_obstacle(scan_msg)

        if obstacle is not None:
            self.obstacle_found = True
            self.closest_obstacle_distance = obstacle['distance']

            if self.phase1_motion_state != 'avoiding':
                self.begin_avoidance(obstacle['danger_angle_deg'])
            else:
                self.avoid_clear_since = None

            if self.phase1_motion_state == 'avoiding':
                turn_direction = self.avoid_turn_direction
            else:
                turn_direction = -1.0 if obstacle['danger_angle_deg'] > 0.0 else 1.0

            self.avoid_cmd = self.create_twist(
                self.avoid_linear_speed,
                turn_direction * self.avoid_angular_speed,
            )
            return

        self.obstacle_found = False
        self.closest_obstacle_distance = float('inf')
        
        # 注：无障碍时清空 markers 已在 find_phase1_forward_obstacle() 中处理（line 388）
        # 这里不需要额外清空逻辑
        
        if self.phase1_motion_state == 'avoiding':
            now = self.get_clock().now()
            if self.avoid_clear_since is None:
                self.avoid_clear_since = now

            avoid_elapsed = 0.0
            if self.avoid_started_time is not None:
                avoid_elapsed = (now - self.avoid_started_time).nanoseconds / 1e9
            clear_elapsed = (now - self.avoid_clear_since).nanoseconds / 1e9

            if (
                avoid_elapsed >= self.avoid_min_duration_sec
                and clear_elapsed >= self.avoid_clear_hold_sec
                and self.avoid_turn_reached()
            ):
                self.begin_counter_steer()
            return

        if self.phase1_motion_state != 'recovering':
            self.avoid_cmd = Twist()

    def lidar_callback(self, msg):
        if self.phase == 1:
            self.handle_phase1_lidar(msg)
            return

        min_dist = float('inf')
        danger_angle = 0.0
        found = False

        for index, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance < self.min_valid_range:
                continue

            angle_deg = math.degrees(msg.angle_min + index * msg.angle_increment)
            angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
            if abs(angle_deg) > self.scan_angle_deg:
                continue

            if distance < min_dist:
                min_dist = distance
                danger_angle = angle_deg
                found = distance < self.safe_distance

        if found:
            self.obstacle_found = True
            self.closest_obstacle_distance = min_dist
            if self.phase == 1 and self.phase1_motion_state != 'avoiding':
                self.begin_avoidance(danger_angle)
            elif self.phase == 1 and self.phase1_motion_state == 'avoiding':
                self.avoid_clear_since = None

            if self.phase == 1 and self.phase1_motion_state == 'avoiding':
                turn_direction = self.avoid_turn_direction
            else:
                turn_direction = -1.0 if danger_angle > 0.0 else 1.0

            self.avoid_cmd = self.create_twist(
                self.avoid_linear_speed,
                turn_direction * self.avoid_angular_speed,
            )
            return

        obstacle_cleared = min_dist > self.clear_distance or math.isinf(min_dist)
        self.obstacle_found = False
        self.closest_obstacle_distance = min_dist
        if self.phase == 1 and self.phase1_motion_state == 'avoiding':
            if not obstacle_cleared:
                self.avoid_clear_since = None
                return

            now = self.get_clock().now()
            if self.avoid_clear_since is None:
                self.avoid_clear_since = now

            avoid_elapsed = 0.0
            if self.avoid_started_time is not None:
                avoid_elapsed = (now - self.avoid_started_time).nanoseconds / 1e9
            clear_elapsed = (now - self.avoid_clear_since).nanoseconds / 1e9

            if (
                avoid_elapsed >= self.avoid_min_duration_sec
                and clear_elapsed >= self.avoid_clear_hold_sec
                and self.avoid_turn_reached()
            ):
                self.begin_counter_steer()
            return

        if self.phase != 1 or self.phase1_motion_state != 'recovering':
            self.avoid_cmd = Twist()

    def qr_callback(self, msg):
        if self.phase != 1:
            return

        if self.phase1_motion_state in ('backing', 'aligning'):
            return

        task = msg.data.strip()
        if not task:
            return

        self.qr_task = task
        self.task_pub.publish(String(data=task))
        
        # 播报识别结果（方向 + 数字）
        self._speak_qr_result(task)
        
        # 如果启用后退功能，先进入 backing 状态；否则直接切换 phase2
        if self.enable_backing and len(self.path_record) > 0:
            self.phase1_motion_state = 'backing'
            self.backing_started_time = self.get_clock().now()
            self.backing_path_index = len(self.path_record) - 1
            self.log.mission(
                f'qr detected: {task}, backing mode, '
                f'{len(self.path_record)} waypoints recorded'
            )
        else:
            self.phase1_motion_state = 'forward'
            self.log.mission(f'qr detected: {task}, switched to phase2 (no backing)')
            self.begin_phase_transition(2, f'qr detected: {task}')


    def stage2_state_callback(self, msg):
        self.stage2_state = msg.data.strip()
        if self.phase == 2 and self.stage2_state == 'complete':
            self.log.mission('stage2 complete, entering phase3')
            self.begin_phase_transition(3, 'stage2 complete, switched to phase3 return-to-p')

    def stage3_state_callback(self, msg):
        self.stage3_state = msg.data.strip()
        if self.phase == 3 and self.stage3_state == 'complete':
            self.mission_finished = True
            self.transition_end_time = None
            self.stop_robot()
            self.log.mission('stage3 complete, mission finished at p point')

    def stage2_cmd_callback(self, msg):
        self.latest_stage2_cmd = msg
        self.latest_stage2_cmd_time = self.get_clock().now()

    def stage2_cmd_is_fresh(self):
        if self.latest_stage2_cmd_time is None:
            return False

        age = self.get_clock().now() - self.latest_stage2_cmd_time
        return age.nanoseconds <= int(self.stage2_cmd_timeout * 1e9)

    def control_loop(self):
        if self.mission_finished:
            self.stop_robot()
            return

        if self.phase == 1:
            # backing 状态优先处理
            if self.phase1_motion_state == 'backing':
                self.handle_backing()
                return
            
            # 注释掉对齐转向状态处理，直接跳过
            # if self.phase1_motion_state == 'aligning':
            #     self.handle_backing_align()
            #     return
            
            if self.phase1_motion_state == 'avoiding':
                self.cmd_pub.publish(self.avoid_cmd)
                return

            if self.phase1_motion_state == 'countersteering':
                if self.counter_steer_deadline is not None and self.get_clock().now() >= self.counter_steer_deadline:
                    self.begin_recovery()
                    return

                self.cmd_pub.publish(
                    self.create_twist(
                        self.counter_steer_linear_speed,
                        -self.avoid_turn_direction * self.counter_steer_angular_speed,
                    )
                )
                return

            if self.phase1_motion_state == 'recovering':
                if self.recovery_complete():
                    self.finish_recovery()
                    self.cmd_pub.publish(self.create_twist(self.blind_linear_speed, self.blind_angular_speed))
                    return

                if self.recovery_uses_heading and self.current_yaw is not None and self.desired_heading is not None:
                    heading_error = self.angle_error(self.desired_heading, self.current_yaw)
                    angular_cmd = self.clamp(
                        self.recovery_heading_kp * heading_error,
                        self.recovery_max_angular_speed,
                    )
                    if abs(heading_error) > self.heading_tolerance_rad and abs(angular_cmd) < self.recovery_min_angular_speed:
                        angular_cmd = math.copysign(self.recovery_min_angular_speed, heading_error)

                    linear_cmd = self.recovery_turn_linear_speed
                    if abs(heading_error) <= self.recovery_in_place_angle_rad:
                        linear_cmd = self.recovery_linear_speed

                    self.cmd_pub.publish(self.create_twist(linear_cmd, angular_cmd))
                    return

                self.cmd_pub.publish(
                    self.create_twist(
                        self.recovery_linear_speed,
                        -self.avoid_turn_direction * self.recovery_angular_speed,
                    )
                )
                return

            self.cmd_pub.publish(self.create_twist(self.blind_linear_speed, self.blind_angular_speed))
            return

        if self.phase2_obstacle_override and self.obstacle_found:
            self.cmd_pub.publish(self.avoid_cmd)
            return

        if self.obstacle_found and self.closest_obstacle_distance <= self.phase2_emergency_stop_distance:
            self.stop_robot()
            return

        if self.transition_end_time is not None and self.get_clock().now() < self.transition_end_time:
            self.stop_robot()
            return

        if self.phase == 3:
            if self.obstacle_found and self.closest_obstacle_distance <= self.phase3_emergency_stop_distance:
                self.stop_robot()
                return

            if self.phase3_external_control:
                return

            self.stop_robot()
            return

        if self.stage2_cmd_is_fresh():
            self.cmd_pub.publish(self.latest_stage2_cmd)
            return

        self.stop_robot()


    def handle_backing(self):
        """处理后退逻辑：沿记录路径反向跟踪"""
        if self.current_odom is None or self.current_yaw is None:
            self.stop_robot()
            return
        
        # 超时检查
        if self.backing_started_time is not None:
            elapsed = (self.get_clock().now() - self.backing_started_time).nanoseconds / 1e9
            if elapsed > self.back_timeout_sec:
                self.log.warn('BACKING', f'timeout after {elapsed:.1f}s, entering phase2 directly')
                self.phase1_motion_state = 'forward'
                # 注释掉对齐转向，直接进 Stage2
                # self.phase1_motion_state = 'aligning'
                # self.aligning_started_time = self.get_clock().now()
                self.begin_phase_transition(2, f'qr task={self.qr_task}, backing timeout, skip align')
                return
        
        current_x = self.current_odom.pose.pose.position.x
        current_y = self.current_odom.pose.pose.position.y
        
        # 检查是否到达目标 x 位置（map 坐标系）
        if current_x <= self.back_target_x:
            self.log.segment(f'backing done at map_x={current_x:.2f}m, entering phase2 directly')
            self.phase1_motion_state = 'forward'
            # 注释掉对齐转向，直接进 Stage2
            # self.phase1_motion_state = 'aligning'
            # self.aligning_started_time = self.get_clock().now()
            self.begin_phase_transition(2, f'qr task={self.qr_task}, backing complete, skip align')
            self.stop_robot()
            return
        
        # 检查路径是否倒序遍历完毕
        if self.backing_path_index < 0 or self.backing_path_index >= len(self.path_record):
            self.log.warn('BACKING', 'path exhausted, entering phase2 directly')
            self.phase1_motion_state = 'forward'
            # 注释掉对齐转向，直接进 Stage2
            # self.phase1_motion_state = 'aligning'
            # self.aligning_started_time = self.get_clock().now()
            self.begin_phase_transition(2, f'qr task={self.qr_task}, path exhausted, skip align')
            return
        
        # 获取当前目标路点（包括来时的 yaw）
        target_x, target_y, target_yaw = self.path_record[self.backing_path_index]
        
        # 检查是否接近当前路点，若是则移动到上一个路点（倒序）
        dist_to_target = math.hypot(current_x - target_x, current_y - target_y)
        if dist_to_target < self.back_position_tolerance:
            self.backing_path_index -= 1
            self.log.progress(
                f'backing wp_index={self.backing_path_index}, '
                f'pos=({current_x:.2f}, {current_y:.2f}), '
                f'target_yaw={math.degrees(target_yaw):.1f}°'
            )
            if self.backing_path_index < 0:
                self.log.progress('backing reached start, entering phase2 directly')
                self.phase1_motion_state = 'forward'
                # 注释掉对齐转向，直接进 Stage2
                # self.phase1_motion_state = 'aligning'
                # self.aligning_started_time = self.get_clock().now()
                self.begin_phase_transition(2, f'qr task={self.qr_task}, reached start, skip align')
                return
            target_x, target_y, target_yaw = self.path_record[self.backing_path_index]
        
        # 后退控制：车头朝向 = 路点记录的来时方向（精确复现轨迹）
        # 不使用实时几何方向 atan2(dy, dx)，而是直接用记录的 target_yaw
        heading_error = self.angle_error(target_yaw, self.current_yaw)
        
        angular_z = self.back_angular_kp * heading_error
        angular_z = self.clamp(angular_z, 1.0)
        
        # 倒车（负速度），车头保持来时方向
        self.cmd_pub.publish(self.create_twist(self.back_linear_speed, angular_z))
        
        self.log.progress(
            f'backing: wp={self.backing_path_index}, '
            f'current_x={current_x:.2f}m, '
            f'dist={dist_to_target:.2f}m, '
            f'target_yaw={math.degrees(target_yaw):.1f}°, '
            f'yaw_error={math.degrees(heading_error):.1f}°'
        )
    
    def handle_backing_align(self):
        """后退完成后对齐航向到指定角度，带超时"""
        if self.current_yaw is None:
            self.stop_robot()
            return

        # 超时检查
        if self.aligning_started_time is not None:
            elapsed = (self.get_clock().now() - self.aligning_started_time).nanoseconds / 1e9
            if elapsed > self.back_align_timeout_sec:
                self.log.warn('ALIGN', f'timeout after {elapsed:.1f}s, forcing phase2')
                self.phase1_motion_state = 'forward'
                self.begin_phase_transition(2, f'align timeout, forcing phase2')
                return

        heading_error = self.angle_error(self.back_align_yaw_rad, self.current_yaw)

        # 检查是否对齐完成
        if abs(heading_error) <= self.back_align_tolerance_rad:
            self.log.mission(
                f'backing align done at yaw={math.degrees(self.current_yaw):.1f}°, '
                f'switching to phase2'
            )
            self.phase1_motion_state = 'forward'
            self.begin_phase_transition(2, f'qr task={self.qr_task}, backing+align complete')
            return

        # 对齐转向：大角度时原地转（linear_x=0），小角度时微速前进配合转向
        angular_z = self.clamp(self.recovery_heading_kp * heading_error, self.recovery_max_angular_speed)
        if abs(angular_z) < self.recovery_min_angular_speed:
            angular_z = math.copysign(self.recovery_min_angular_speed, heading_error)

        # 大角度（>30°）原地转，小角度（<8°）微速前进，中间角度慢速前进
        if abs(heading_error) > math.radians(30.0):
            linear_x = 0.0  # 原地转
        elif abs(heading_error) <= self.recovery_in_place_angle_rad:
            linear_x = self.recovery_linear_speed  # 0.12 m/s
        else:
            linear_x = self.recovery_turn_linear_speed  # 0.08 m/s

        self.log.feedback(
            f'aligning yaw={math.degrees(self.current_yaw):.1f}° '
            f'target=90° err={math.degrees(heading_error):.1f}° '
            f'cmd: linear={linear_x:.2f} angular={angular_z:.2f}'
        )
        self.cmd_pub.publish(self.create_twist(linear_x, angular_z))


    def destroy_node(self):
        self.log.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CompetitionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

    def _speak_qr_result(self, task):
        """播报二维码识别结果：方向 + 数字"""
        if self.tts_player is None:
            self.log.warn('VOICE', 'CN-TTS 模块未初始化，无法播报')
            return
        
        try:
            # 提取方向和数字
            direction_text = ""
            numbers_text = ""
            
            if 'clockwise' in task.lower():
                direction_text = "顺时针"
            elif 'counterclockwise' in task.lower():
                direction_text = "逆时针"
            
            # 提取数字（支持多种格式）
            import re as re_local
            numbers = re_local.findall(r'\d+', task)
            if numbers:
                # 数字按个位读，如 123 -> "1 2 3"
                numbers_list = []
                for num in numbers:
                    numbers_list.extend(list(num))
                numbers_text = " ".join(numbers_list)
            
            # 组合播报内容
            if direction_text and numbers_text:
                speak_text = f"{direction_text} {numbers_text}"
            elif direction_text:
                speak_text = direction_text
            elif numbers_text:
                speak_text = numbers_text
            else:
                speak_text = f"任务识别 {task}"
            
            # 详细日志记录
            self.log.mission(f'QR播报开始: 原文="{task}" → 播报="{speak_text}"')
            self.get_logger().info(f'[VOICE] 播报二维码: {speak_text}')
            
            # 执行播报
            self.tts_player.speak_text(speak_text)
            self.log.feedback(f'QR播报完成: "{speak_text}"')
            
        except Exception as e:
            self.log.error('VOICE', f'播报失败: {e}')
            self.get_logger().error(f'[VOICE] 播报异常: {e}')


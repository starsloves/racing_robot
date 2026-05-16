import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, String


class Stage2InertialNavigator(Node):
    def __init__(self):
        super().__init__('stage2_inertial_navigator')

        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('task_topic', 'competition_qr_task')
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('cmd_topic', '/stage2_cmd_vel')
        self.declare_parameter('feedback_topic', 'competition_feedback')
        self.declare_parameter('state_topic', 'stage2_state')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('start_delay_sec', 0.5)
        self.declare_parameter('corridor_linear_speed', 0.18)
        self.declare_parameter('ring_linear_speed', 0.20)
        self.declare_parameter('turn_angular_speed', 0.75)
        self.declare_parameter('turn_kp', 1.8)
        self.declare_parameter('heading_kp', 1.6)
        self.declare_parameter('max_angular_speed', 0.9)
        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('heading_tolerance_deg', 4.0)
        self.declare_parameter('segment_timeout', 12.0)
        self.declare_parameter('odd_is_clockwise', True)
        self.declare_parameter('clockwise_keywords', '顺时针,cw,clockwise')
        self.declare_parameter('counterclockwise_keywords', '逆时针,ccw,counterclockwise')
        self.declare_parameter('pre_loop_plan_json', '[]')
        self.declare_parameter('loop_long_length_m', 3.5)
        self.declare_parameter('loop_short_length_m', 1.15)
        self.declare_parameter('exit_distance_m', 1.0)

        self.phase_topic = self.get_parameter('phase_topic').value
        self.task_topic = self.get_parameter('task_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.corridor_linear_speed = float(self.get_parameter('corridor_linear_speed').value)
        self.ring_linear_speed = float(self.get_parameter('ring_linear_speed').value)
        self.turn_angular_speed = float(self.get_parameter('turn_angular_speed').value)
        self.turn_kp = float(self.get_parameter('turn_kp').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.distance_tolerance = float(self.get_parameter('distance_tolerance').value)
        self.heading_tolerance = math.radians(float(self.get_parameter('heading_tolerance_deg').value))
        self.segment_timeout = float(self.get_parameter('segment_timeout').value)
        self.odd_is_clockwise = bool(self.get_parameter('odd_is_clockwise').value)
        self.clockwise_keywords = self.parse_keywords(self.get_parameter('clockwise_keywords').value)
        self.counterclockwise_keywords = self.parse_keywords(self.get_parameter('counterclockwise_keywords').value)
        self.pre_loop_plan_json = self.get_parameter('pre_loop_plan_json').value
        self.loop_long_length_m = float(self.get_parameter('loop_long_length_m').value)
        self.loop_short_length_m = float(self.get_parameter('loop_short_length_m').value)
        self.exit_distance_m = float(self.get_parameter('exit_distance_m').value)

        self.phase = 1
        self.task_raw = ''
        self.direction = None
        self.current_yaw = None
        self.current_position = None
        self.plan = []
        self.plan_index = -1
        self.current_segment = None
        self.segment_started_at = None
        self.segment_start_pose = None
        self.segment_start_yaw = None
        self.segment_target_yaw = None
        self.segment_heading = None
        self.segment_state_label = 'idle'
        self.start_after_time = None
        self.mission_active = False
        self.mission_finished = False
        self.reported_start = False

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.create_subscription(Int32, self.phase_topic, self.phase_callback, 10)
        self.create_subscription(String, self.task_topic, self.task_callback, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.publish_state('idle')
        self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_loop)

        self.get_logger().info('stage2 inertial navigator ready')

    def parse_keywords(self, raw_value):
        return [item.strip().lower() for item in raw_value.split(',') if item.strip()]

    def quaternion_to_yaw(self, orientation):
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def angle_error(self, target_angle, current_angle):
        return self.normalize_angle(target_angle - current_angle)

    def create_twist(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        return msg

    def clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def publish_feedback(self, text):
        self.feedback_pub.publish(String(data=text))
        self.get_logger().info(text)

    def publish_state(self, text):
        self.segment_state_label = text
        self.state_pub.publish(String(data=text))

    def phase_callback(self, msg):
        previous_phase = self.phase
        self.phase = int(msg.data)
        if previous_phase != self.phase and self.phase != 2:
            self.reset_mission(clear_task=False)
        self.try_start_mission()

    def task_callback(self, msg):
        self.task_raw = msg.data.strip()
        self.direction = self.parse_direction(self.task_raw)
        self.try_start_mission()

    def imu_callback(self, msg):
        self.current_yaw = self.quaternion_to_yaw(msg.orientation)
        self.try_start_mission()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.current_position = (float(position.x), float(position.y))
        self.try_start_mission()

    def parse_direction(self, task_text):
        if not task_text:
            return None

        normalized = task_text.lower()
        for keyword in self.clockwise_keywords:
            if keyword and keyword in normalized:
                return 'clockwise'
        for keyword in self.counterclockwise_keywords:
            if keyword and keyword in normalized:
                return 'counterclockwise'

        try:
            numeric_value = int(task_text)
        except ValueError:
            return None

        is_odd = numeric_value % 2 == 1
        if self.odd_is_clockwise:
            return 'clockwise' if is_odd else 'counterclockwise'
        return 'counterclockwise' if is_odd else 'clockwise'

    def parse_pre_loop_plan(self):
        try:
            raw_segments = json.loads(self.pre_loop_plan_json)
        except json.JSONDecodeError:
            self.get_logger().error('pre_loop_plan_json is invalid, fallback to empty plan')
            return []

        if not isinstance(raw_segments, list):
            self.get_logger().error('pre_loop_plan_json must decode to a list, fallback to empty plan')
            return []

        sanitized_segments = []
        for raw_segment in raw_segments:
            if not isinstance(raw_segment, dict):
                continue
            segment_type = raw_segment.get('type')
            if segment_type == 'turn':
                sanitized_segments.append({
                    'type': 'turn',
                    'angle_deg': float(raw_segment.get('angle_deg', 0.0)),
                    'description': raw_segment.get('description', 'pre_turn'),
                })
            elif segment_type == 'move':
                sanitized_segments.append({
                    'type': 'move',
                    'distance_m': max(0.0, float(raw_segment.get('distance_m', 0.0))),
                    'speed': float(raw_segment.get('speed', self.corridor_linear_speed)),
                    'description': raw_segment.get('description', 'pre_move'),
                })
            elif segment_type == 'pause':
                sanitized_segments.append({
                    'type': 'pause',
                    'duration': max(0.0, float(raw_segment.get('duration', 0.0))),
                    'description': raw_segment.get('description', 'pre_pause'),
                })
        return sanitized_segments

    def build_ring_plan(self):
        half_long = self.loop_long_length_m / 2.0
        initial_turn = 90.0 if self.direction == 'clockwise' else -90.0
        corner_turn = -initial_turn

        return [
            {'type': 'turn', 'angle_deg': initial_turn, 'description': 'loop_enter'},
            {'type': 'move', 'distance_m': half_long, 'speed': self.ring_linear_speed, 'description': 'loop_bottom_half_1'},
            {'type': 'turn', 'angle_deg': corner_turn, 'description': 'corner_1'},
            {'type': 'move', 'distance_m': self.loop_short_length_m, 'speed': self.ring_linear_speed, 'description': 'loop_short_1'},
            {'type': 'turn', 'angle_deg': corner_turn, 'description': 'corner_2'},
            {'type': 'move', 'distance_m': self.loop_long_length_m, 'speed': self.ring_linear_speed, 'description': 'loop_long_top'},
            {'type': 'turn', 'angle_deg': corner_turn, 'description': 'corner_3'},
            {'type': 'move', 'distance_m': self.loop_short_length_m, 'speed': self.ring_linear_speed, 'description': 'loop_short_2'},
            {'type': 'turn', 'angle_deg': corner_turn, 'description': 'corner_4'},
            {'type': 'move', 'distance_m': half_long, 'speed': self.ring_linear_speed, 'description': 'loop_bottom_half_2'},
            {'type': 'turn', 'angle_deg': initial_turn, 'description': 'loop_exit_align'},
            {'type': 'move', 'distance_m': self.exit_distance_m, 'speed': self.corridor_linear_speed, 'description': 'corridor_exit'},
        ]

    def try_start_mission(self):
        if self.mission_active or self.mission_finished:
            return
        if self.phase != 2 or self.direction is None:
            return
        if self.current_yaw is None or self.current_position is None:
            return

        if self.start_after_time is None:
            self.start_after_time = self.get_clock().now().nanoseconds / 1e9 + self.start_delay_sec
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time < self.start_after_time:
            return

        self.plan = self.parse_pre_loop_plan() + self.build_ring_plan()
        self.mission_active = True
        self.reported_start = True
        self.publish_feedback(f'第二阶段启动，方向: {self.direction}')
        self.start_segment(0)

    def reset_mission(self, clear_task):
        self.cmd_pub.publish(Twist())
        self.plan = []
        self.plan_index = -1
        self.current_segment = None
        self.segment_started_at = None
        self.segment_start_pose = None
        self.segment_start_yaw = None
        self.segment_target_yaw = None
        self.segment_heading = None
        self.start_after_time = None
        self.mission_active = False
        self.mission_finished = False
        self.reported_start = False
        self.publish_state('idle')
        if clear_task:
            self.task_raw = ''
            self.direction = None

    def start_segment(self, index):
        if index >= len(self.plan):
            self.finish_mission()
            return

        self.plan_index = index
        self.current_segment = self.plan[index]
        self.segment_started_at = self.get_clock().now().nanoseconds / 1e9
        self.segment_start_pose = self.current_position
        self.segment_start_yaw = self.current_yaw

        if self.current_segment['type'] == 'turn':
            self.segment_target_yaw = self.normalize_angle(
                self.segment_start_yaw + math.radians(self.current_segment['angle_deg'])
            )
            self.publish_state(self.current_segment['description'])
        elif self.current_segment['type'] == 'move':
            self.segment_heading = self.segment_start_yaw
            self.publish_state(self.current_segment['description'])
        elif self.current_segment['type'] == 'pause':
            self.publish_state(self.current_segment['description'])

    def finish_mission(self):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self.publish_state('complete')
        self.publish_feedback('第二阶段完成，已离开环形通道并到达出口')

    def projected_distance(self):
        if self.segment_start_pose is None or self.current_position is None or self.segment_heading is None:
            return 0.0
        dx = self.current_position[0] - self.segment_start_pose[0]
        dy = self.current_position[1] - self.segment_start_pose[1]
        return dx * math.cos(self.segment_heading) + dy * math.sin(self.segment_heading)

    def control_loop(self):
        if not self.mission_active or self.current_segment is None:
            if not self.mission_active:
                self.cmd_pub.publish(Twist())
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.segment_started_at is not None and now_sec - self.segment_started_at > self.segment_timeout:
            self.publish_feedback(f'段超时，强制切换: {self.current_segment.get("description", "unknown")}')
            self.start_segment(self.plan_index + 1)
            return

        segment_type = self.current_segment['type']
        if segment_type == 'turn':
            self.run_turn_segment()
            return
        if segment_type == 'move':
            self.run_move_segment()
            return
        if segment_type == 'pause':
            self.run_pause_segment(now_sec)
            return

        self.start_segment(self.plan_index + 1)

    def run_turn_segment(self):
        if self.current_yaw is None or self.segment_target_yaw is None:
            self.cmd_pub.publish(Twist())
            return

        error = self.angle_error(self.segment_target_yaw, self.current_yaw)
        if abs(error) <= self.heading_tolerance:
            self.cmd_pub.publish(Twist())
            self.start_segment(self.plan_index + 1)
            return

        angular = self.clamp(self.turn_kp * error, self.turn_angular_speed)
        self.cmd_pub.publish(self.create_twist(0.0, angular))

    def run_move_segment(self):
        if self.current_position is None or self.segment_heading is None:
            self.cmd_pub.publish(Twist())
            return

        progress = self.projected_distance()
        target_distance = float(self.current_segment['distance_m'])
        if progress >= target_distance - self.distance_tolerance:
            self.cmd_pub.publish(Twist())
            self.start_segment(self.plan_index + 1)
            return

        heading_error = 0.0 if self.current_yaw is None else self.angle_error(self.segment_heading, self.current_yaw)
        angular = self.clamp(self.heading_kp * heading_error, self.max_angular_speed)
        linear = float(self.current_segment.get('speed', self.corridor_linear_speed))
        self.cmd_pub.publish(self.create_twist(linear, angular))

    def run_pause_segment(self, now_sec):
        duration = float(self.current_segment.get('duration', 0.0))
        self.cmd_pub.publish(Twist())
        if self.segment_started_at is not None and now_sec - self.segment_started_at >= duration:
            self.start_segment(self.plan_index + 1)


def main(args=None):
    rclpy.init(args=args)
    node = Stage2InertialNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
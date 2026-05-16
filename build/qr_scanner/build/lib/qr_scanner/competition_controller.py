import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32, String


class CompetitionController(Node):
    def __init__(self):
        super().__init__('competition_controller')

        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('stage2_cmd_topic', '/stage2_cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('qr_result_topic', 'qr_scan_result')
        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('task_topic', 'competition_qr_task')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('blind_linear_speed', 0.2)
        self.declare_parameter('blind_angular_speed', 0.0)
        self.declare_parameter('avoid_linear_speed', 0.1)
        self.declare_parameter('avoid_angular_speed', 0.8)
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('clear_distance', 0.65)
        self.declare_parameter('scan_angle_deg', 45.0)
        self.declare_parameter('min_valid_range', 0.15)
        self.declare_parameter('recovery_linear_speed', 0.12)
        self.declare_parameter('recovery_angular_speed', 0.75)
        self.declare_parameter('heading_tolerance_deg', 6.0)
        self.declare_parameter('recovery_timeout', 1.2)
        self.declare_parameter('recovery_duration_scale', 0.9)
        self.declare_parameter('stage2_cmd_timeout', 0.5)
        self.declare_parameter('transition_stop_duration', 0.0)

        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.stage2_cmd_topic = self.get_parameter('stage2_cmd_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.qr_result_topic = self.get_parameter('qr_result_topic').value
        self.phase_topic = self.get_parameter('phase_topic').value
        self.task_topic = self.get_parameter('task_topic').value
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.blind_linear_speed = float(self.get_parameter('blind_linear_speed').value)
        self.blind_angular_speed = float(self.get_parameter('blind_angular_speed').value)
        self.avoid_linear_speed = float(self.get_parameter('avoid_linear_speed').value)
        self.avoid_angular_speed = float(self.get_parameter('avoid_angular_speed').value)
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        self.scan_angle_deg = float(self.get_parameter('scan_angle_deg').value)
        self.min_valid_range = float(self.get_parameter('min_valid_range').value)
        self.recovery_linear_speed = float(self.get_parameter('recovery_linear_speed').value)
        self.recovery_angular_speed = float(self.get_parameter('recovery_angular_speed').value)
        self.heading_tolerance_rad = math.radians(float(self.get_parameter('heading_tolerance_deg').value))
        self.recovery_timeout = float(self.get_parameter('recovery_timeout').value)
        self.recovery_duration_scale = float(self.get_parameter('recovery_duration_scale').value)
        self.stage2_cmd_timeout = float(self.get_parameter('stage2_cmd_timeout').value)
        self.transition_stop_duration = float(self.get_parameter('transition_stop_duration').value)

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
        self.create_subscription(String, self.qr_result_topic, self.qr_callback, 10)
        self.create_subscription(Twist, self.stage2_cmd_topic, self.stage2_cmd_callback, 10)

        self.phase = 1
        self.obstacle_found = False
        self.avoid_cmd = Twist()
        self.phase1_motion_state = 'forward'
        self.current_yaw = None
        self.desired_heading = None
        self.avoid_turn_direction = 0.0
        self.avoid_started_time = None
        self.recovery_deadline = None
        self.recovery_uses_heading = False
        self.warned_missing_heading = False
        self.latest_stage2_cmd = Twist()
        self.latest_stage2_cmd_time = None
        self.transition_end_time = None
        self.qr_task = ''

        self.publish_phase()
        self.create_timer(1.0 / max(control_rate_hz, 1.0), self.control_loop)

        self.get_logger().info(
            'competition controller ready: phase1 blind drive, obstacle override, qr triggers phase2'
        )

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

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def imu_callback(self, msg):
        self.current_yaw = self.quaternion_to_yaw(msg.orientation)
        if self.phase == 1 and self.desired_heading is None:
            self.desired_heading = self.current_yaw
            self.get_logger().info(
                f'phase1 heading locked at {math.degrees(self.desired_heading):.1f} deg'
            )

    def begin_avoidance(self, danger_angle):
        self.phase1_motion_state = 'avoiding'
        self.avoid_turn_direction = -1.0 if danger_angle > 0.0 else 1.0
        self.avoid_started_time = self.get_clock().now()
        self.recovery_deadline = None
        self.recovery_uses_heading = False

        if self.desired_heading is None and self.current_yaw is not None:
            self.desired_heading = self.current_yaw

    def begin_recovery(self):
        if self.phase1_motion_state != 'avoiding':
            return

        now = self.get_clock().now()
        avoid_duration = 0.0
        if self.avoid_started_time is not None:
            avoid_duration = (now - self.avoid_started_time).nanoseconds / 1e9

        self.phase1_motion_state = 'recovering'
        self.recovery_uses_heading = self.current_yaw is not None and self.desired_heading is not None
        if self.recovery_uses_heading:
            self.recovery_deadline = now + Duration(seconds=self.recovery_timeout)
            return

        recovery_duration = max(0.15, avoid_duration * self.recovery_duration_scale)
        recovery_duration = min(recovery_duration, self.recovery_timeout)
        self.recovery_deadline = now + Duration(seconds=recovery_duration)
        if not self.warned_missing_heading:
            self.warned_missing_heading = True
            self.get_logger().warn('imu heading unavailable, recovery falls back to timed reverse steering')

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
        self.recovery_deadline = None
        self.recovery_uses_heading = False

    def lidar_callback(self, msg):
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
            if self.phase == 1 and self.phase1_motion_state != 'avoiding':
                self.begin_avoidance(danger_angle)

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
        if self.phase == 1 and self.phase1_motion_state == 'avoiding':
            if obstacle_cleared:
                self.begin_recovery()
            else:
                return

        if self.phase != 1 or self.phase1_motion_state != 'recovering':
            self.avoid_cmd = Twist()

    def qr_callback(self, msg):
        if self.phase != 1:
            return

        task = msg.data.strip()
        if not task:
            return

        self.phase = 2
        self.qr_task = task
        self.publish_phase()
        self.task_pub.publish(String(data=task))
        self.phase1_motion_state = 'forward'
        self.stop_robot()

        if self.transition_stop_duration > 0.0:
            self.transition_end_time = self.get_clock().now() + Duration(seconds=self.transition_stop_duration)
        else:
            self.transition_end_time = None

        self.get_logger().warn(f'qr detected: {task}, switched to phase2')

    def stage2_cmd_callback(self, msg):
        self.latest_stage2_cmd = msg
        self.latest_stage2_cmd_time = self.get_clock().now()

    def stage2_cmd_is_fresh(self):
        if self.latest_stage2_cmd_time is None:
            return False

        age = self.get_clock().now() - self.latest_stage2_cmd_time
        return age.nanoseconds <= int(self.stage2_cmd_timeout * 1e9)

    def control_loop(self):
        if self.phase == 1:
            if self.phase1_motion_state == 'avoiding':
                self.cmd_pub.publish(self.avoid_cmd)
                return

            if self.phase1_motion_state == 'recovering':
                if self.recovery_complete():
                    self.finish_recovery()
                    self.cmd_pub.publish(self.create_twist(self.blind_linear_speed, self.blind_angular_speed))
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

        if self.obstacle_found:
            self.cmd_pub.publish(self.avoid_cmd)
            return

        if self.transition_end_time is not None and self.get_clock().now() < self.transition_end_time:
            self.stop_robot()
            return

        if self.stage2_cmd_is_fresh():
            self.cmd_pub.publish(self.latest_stage2_cmd)
            return

        self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = CompetitionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
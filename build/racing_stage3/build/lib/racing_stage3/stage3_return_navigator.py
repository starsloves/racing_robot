import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String


class Stage3ReturnNavigator(Node):
    def __init__(self):
        super().__init__('stage3_return_navigator')

        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('return_path_topic', '/stage3_return_path')
        self.declare_parameter('state_topic', 'stage3_state')
        self.declare_parameter('feedback_topic', 'competition_feedback')
        self.declare_parameter('start_delay_sec', 0.5)
        self.declare_parameter('return_waypoints_json', '[]')
        self.declare_parameter('return_waypoint_tolerance', 0.20)
        self.declare_parameter('return_goal_tolerance', 0.12)
        self.declare_parameter('return_goal_yaw_tolerance_deg', 8.0)
        self.declare_parameter('return_path_timeout_sec', 60.0)
        self.declare_parameter('pure_pursuit_linear_speed', 0.18)
        self.declare_parameter('pure_pursuit_lookahead_m', 0.45)
        self.declare_parameter('pure_pursuit_heading_stop_deg', 70.0)
        self.declare_parameter('pure_pursuit_turn_kp', 1.8)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('min_angular_speed', 0.45)

        self.phase_topic = self.get_parameter('phase_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.return_path_topic = self.get_parameter('return_path_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.return_waypoints_json = self.get_parameter('return_waypoints_json').value
        self.return_waypoint_tolerance = float(self.get_parameter('return_waypoint_tolerance').value)
        self.return_goal_tolerance = float(self.get_parameter('return_goal_tolerance').value)
        self.return_goal_yaw_tolerance = math.radians(
            float(self.get_parameter('return_goal_yaw_tolerance_deg').value)
        )
        self.return_path_timeout_sec = float(self.get_parameter('return_path_timeout_sec').value)
        self.pure_pursuit_linear_speed = float(self.get_parameter('pure_pursuit_linear_speed').value)
        self.pure_pursuit_lookahead_m = float(self.get_parameter('pure_pursuit_lookahead_m').value)
        self.pure_pursuit_heading_stop = math.radians(
            float(self.get_parameter('pure_pursuit_heading_stop_deg').value)
        )
        self.pure_pursuit_turn_kp = float(self.get_parameter('pure_pursuit_turn_kp').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.min_angular_speed = float(self.get_parameter('min_angular_speed').value)

        self.return_waypoints = self.parse_waypoints_json(
            self.return_waypoints_json,
            'return_waypoints_json',
            self.pure_pursuit_linear_speed,
        )

        self.phase = 1
        self.mission_active = False
        self.mission_finished = False
        self.start_after_time = None
        self.current_position = None
        self.current_yaw = None
        self.odom_frame_id = 'odom'
        self.path_origin_pose = None
        self.path_origin_yaw = None
        self.path_started_at = None
        self.path_index = 0

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = ReliabilityPolicy.RELIABLE

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.return_path_pub = self.create_publisher(Path, self.return_path_topic, path_qos)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.create_subscription(Int32, self.phase_topic, self.phase_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.publish_state('idle')
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('stage3 return navigator ready')

    def parse_waypoints_json(self, raw_json, param_name, default_speed):
        try:
            raw_waypoints = json.loads(raw_json)
        except json.JSONDecodeError:
            self.get_logger().error(f'{param_name} is invalid, fallback to empty waypoints')
            return []

        if not isinstance(raw_waypoints, list):
            self.get_logger().error(f'{param_name} must decode to a list, fallback to empty waypoints')
            return []

        sanitized_waypoints = []
        for index, raw_waypoint in enumerate(raw_waypoints):
            if not isinstance(raw_waypoint, dict):
                continue

            yaw_deg = raw_waypoint.get('yaw_deg')
            sanitized_waypoints.append({
                'x': float(raw_waypoint.get('x', 0.0)),
                'y': float(raw_waypoint.get('y', 0.0)),
                'speed': float(raw_waypoint.get('speed', default_speed)),
                'yaw_deg': None if yaw_deg is None else float(yaw_deg),
                'description': raw_waypoint.get('description', f'return_wp_{index}'),
            })

        return sanitized_waypoints

    def quaternion_to_yaw(self, orientation):
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def angle_error(self, target_angle, current_angle):
        return self.normalize_angle(target_angle - current_angle)

    def yaw_to_quaternion(self, yaw):
        half_yaw = yaw / 2.0
        return (math.sin(half_yaw), math.cos(half_yaw))

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
        self.state_pub.publish(String(data=text))

    def clear_return_path(self):
        path_msg = Path()
        path_msg.header.frame_id = self.odom_frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        self.return_path_pub.publish(path_msg)

    def publish_return_path(self):
        if self.path_origin_pose is None or self.path_origin_yaw is None:
            return
        if not self.return_waypoints:
            return

        path_msg = Path()
        path_msg.header.frame_id = self.odom_frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        origin_x, origin_y = self.path_origin_pose
        origin_yaw = self.path_origin_yaw

        for index, waypoint in enumerate(self.return_waypoints):
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = path_msg.header.frame_id
            pose_msg.header.stamp = path_msg.header.stamp
            pose_msg.pose.position.x = (
                origin_x
                + math.cos(origin_yaw) * waypoint['x']
                - math.sin(origin_yaw) * waypoint['y']
            )
            pose_msg.pose.position.y = (
                origin_y
                + math.sin(origin_yaw) * waypoint['x']
                + math.cos(origin_yaw) * waypoint['y']
            )

            yaw_deg = waypoint.get('yaw_deg')
            if yaw_deg is not None:
                pose_yaw = origin_yaw + math.radians(float(yaw_deg))
            elif index < len(self.return_waypoints) - 1:
                next_waypoint = self.return_waypoints[index + 1]
                pose_yaw = origin_yaw + math.atan2(
                    next_waypoint['y'] - waypoint['y'],
                    next_waypoint['x'] - waypoint['x'],
                )
            elif index > 0:
                previous_waypoint = self.return_waypoints[index - 1]
                pose_yaw = origin_yaw + math.atan2(
                    waypoint['y'] - previous_waypoint['y'],
                    waypoint['x'] - previous_waypoint['x'],
                )
            else:
                pose_yaw = origin_yaw

            orientation_z, orientation_w = self.yaw_to_quaternion(pose_yaw)
            pose_msg.pose.orientation.z = orientation_z
            pose_msg.pose.orientation.w = orientation_w
            path_msg.poses.append(pose_msg)

        self.return_path_pub.publish(path_msg)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.current_position = (float(position.x), float(position.y))
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_frame_id = msg.header.frame_id or self.odom_frame_id

    def phase_callback(self, msg):
        previous_phase = self.phase
        self.phase = int(msg.data)

        if previous_phase != self.phase and self.phase != 3:
            self.reset_mission()
            return

        if previous_phase != self.phase and self.phase == 3:
            self.mission_active = False
            self.mission_finished = False
            self.path_origin_pose = None
            self.path_origin_yaw = None
            self.path_started_at = None
            self.path_index = 0
            self.start_after_time = self.get_clock().now().nanoseconds / 1e9 + self.start_delay_sec
            self.clear_return_path()
            self.publish_state('armed')
            self.publish_feedback('进入阶段三，准备沿固定航点路径返回 P 点')

    def reset_mission(self):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = False
        self.start_after_time = None
        self.path_origin_pose = None
        self.path_origin_yaw = None
        self.path_started_at = None
        self.path_index = 0
        self.clear_return_path()
        self.publish_state('idle')

    def start_return_path(self):
        if self.current_position is None or self.current_yaw is None:
            return
        if not self.return_waypoints:
            self.fail_mission('阶段三未配置返航航点，无法返回 P 点')
            return

        self.mission_active = True
        self.path_origin_pose = self.current_position
        self.path_origin_yaw = self.current_yaw
        self.path_started_at = self.get_clock().now().nanoseconds / 1e9
        self.path_index = 0
        self.publish_return_path()
        self.publish_state('running')
        self.publish_feedback(f'阶段三开始沿固定航点路径返航，共 {len(self.return_waypoints)} 个航点')

    def local_pose(self):
        if self.current_position is None or self.path_origin_pose is None or self.path_origin_yaw is None:
            return None
        dx = self.current_position[0] - self.path_origin_pose[0]
        dy = self.current_position[1] - self.path_origin_pose[1]
        x_local = math.cos(self.path_origin_yaw) * dx + math.sin(self.path_origin_yaw) * dy
        y_local = -math.sin(self.path_origin_yaw) * dx + math.cos(self.path_origin_yaw) * dy
        return (x_local, y_local)

    def local_yaw(self):
        if self.current_yaw is None or self.path_origin_yaw is None:
            return None
        return self.normalize_angle(self.current_yaw - self.path_origin_yaw)

    def maybe_advance_waypoint(self, local_pose):
        while self.path_index < len(self.return_waypoints) - 1:
            waypoint = self.return_waypoints[self.path_index]
            distance = math.hypot(waypoint['x'] - local_pose[0], waypoint['y'] - local_pose[1])
            if distance > self.return_waypoint_tolerance:
                return
            self.path_index += 1

    def finish_mission(self):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self.publish_state('complete')
        self.publish_feedback('阶段三完成，车辆已返回 P 点')

    def fail_mission(self, reason):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self.publish_state('failed')
        self.publish_feedback(reason)

    def control_loop(self):
        if self.phase != 3:
            return
        if self.mission_finished:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if not self.mission_active:
            if self.start_after_time is None or now_sec < self.start_after_time:
                return
            self.start_return_path()
            return

        self.run_return_path_stage()

    def run_return_path_stage(self):
        self.cmd_pub.publish(Twist())
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.path_started_at is not None and now_sec - self.path_started_at > self.return_path_timeout_sec:
            self.fail_mission('阶段三固定航点路径超时，未能返回 P 点')
            return

        local_pose = self.local_pose()
        local_yaw = self.local_yaw()
        if local_pose is None or local_yaw is None:
            self.cmd_pub.publish(Twist())
            return

        self.maybe_advance_waypoint(local_pose)
        final_waypoint = self.return_waypoints[-1]
        final_distance = math.hypot(final_waypoint['x'] - local_pose[0], final_waypoint['y'] - local_pose[1])
        if self.path_index >= len(self.return_waypoints) - 1 and final_distance <= self.return_goal_tolerance:
            target_yaw_deg = final_waypoint.get('yaw_deg')
            if target_yaw_deg is not None:
                yaw_error = self.angle_error(math.radians(target_yaw_deg), local_yaw)
                if abs(yaw_error) > self.return_goal_yaw_tolerance:
                    angular = self.clamp(self.pure_pursuit_turn_kp * yaw_error, self.max_angular_speed)
                    if abs(angular) < self.min_angular_speed:
                        angular = math.copysign(self.min_angular_speed, yaw_error)
                    self.publish_state('align')
                    self.cmd_pub.publish(self.create_twist(0.0, angular))
                    return

            self.finish_mission()
            return

        target_waypoint = self.return_waypoints[self.path_index]
        target_dx = target_waypoint['x'] - local_pose[0]
        target_dy = target_waypoint['y'] - local_pose[1]
        target_x_robot = math.cos(local_yaw) * target_dx + math.sin(local_yaw) * target_dy
        target_y_robot = -math.sin(local_yaw) * target_dx + math.cos(local_yaw) * target_dy
        target_distance = math.hypot(target_x_robot, target_y_robot)
        heading_error = math.atan2(target_y_robot, target_x_robot if abs(target_x_robot) > 1e-6 else 1e-6)

        self.publish_state(target_waypoint['description'])
        if target_x_robot <= 0.0 or abs(heading_error) > self.pure_pursuit_heading_stop:
            angular = self.clamp(self.pure_pursuit_turn_kp * heading_error, self.max_angular_speed)
            if abs(angular) < self.min_angular_speed:
                angular = math.copysign(self.min_angular_speed, heading_error)
            self.cmd_pub.publish(self.create_twist(0.0, angular))
            return

        pursuit_distance = max(target_distance, self.pure_pursuit_lookahead_m)
        curvature = 0.0 if pursuit_distance <= 1e-6 else 2.0 * target_y_robot / (pursuit_distance * pursuit_distance)
        linear_speed = min(float(target_waypoint.get('speed', self.pure_pursuit_linear_speed)), self.pure_pursuit_linear_speed)
        if target_distance < self.pure_pursuit_lookahead_m:
            linear_speed *= max(0.4, target_distance / max(self.pure_pursuit_lookahead_m, 1e-6))
        angular_speed = self.clamp(linear_speed * curvature, self.max_angular_speed)
        self.cmd_pub.publish(self.create_twist(linear_speed, angular_speed))


def main(args=None):
    rclpy.init(args=args)
    node = Stage3ReturnNavigator()
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

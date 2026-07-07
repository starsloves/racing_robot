"""Stage3 param test — map-frame return navigation to P (0.20, 0.20).

参考 Stage2 corridor 导航模式：
- 起点通过 return_start_json 参数传入（JSON 格式，类似 corridor_waypoints_json）
- 终点通过 return_goal_json 参数传入（默认 P 点）
- 可选中间路点通过 return_waypoints_json 传入
- 全程 Pure Pursuit + A* 动态规划
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32, String

from .global_path_planner import GlobalPathPlannerMixin


class Stage3ReturnNavigator(GlobalPathPlannerMixin, Node):
    def __init__(self):
        super().__init__('stage3_return_navigator')

        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('return_path_topic', '/stage3_return_path')
        self.declare_parameter('state_topic', 'stage3_state')
        self.declare_parameter('feedback_topic', 'competition_feedback')
        self.declare_parameter('start_delay_sec', 0.5)
        
        # 起点/终点/路点参数（JSON 格式，类似 Stage2 corridor）
        self.declare_parameter('return_start_json', '')
        self.declare_parameter('return_goal_json', '[{"x":0.20,"y":0.20,"speed":0.10,"yaw_deg":100.0,"description":"p_point"}]')
        self.declare_parameter('return_waypoints_json', '[]')
        
        # 坐标系与容差
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('global_yaw_source', 'odom')
        self.declare_parameter('global_yaw_disagreement_deg', 45.0)
        self.declare_parameter('return_waypoint_tolerance', 0.18)
        self.declare_parameter('return_goal_tolerance', 0.10)
        self.declare_parameter('return_goal_yaw_tolerance_deg', 8.0)
        self.declare_parameter('return_path_timeout_sec', 90.0)
        
        # Pure Pursuit 参数
        self.declare_parameter('pure_pursuit_linear_speed', 0.18)
        self.declare_parameter('pure_pursuit_lookahead_m', 0.45)
        self.declare_parameter('pure_pursuit_heading_stop_deg', 70.0)
        self.declare_parameter('pure_pursuit_turn_kp', 1.8)
        self.declare_parameter('turn_linear_speed', 0.08)
        self.declare_parameter('turn_min_angular_speed', 0.45)
        self.declare_parameter('max_angular_speed', 0.8)
        
        # A* 规划参数
        self.declare_parameter('use_occupancy_grid_planner', True)
        self.declare_parameter('planner_downsample', 4)
        self.declare_parameter('planner_occupied_threshold', 50)
        self.declare_parameter('planner_unknown_is_occupied', True)
        self.declare_parameter('planner_obstacle_inflation_m', 0.14)
        self.declare_parameter('planner_dynamic_obstacle_box_size_m', 0.25)
        self.declare_parameter('planner_dynamic_obstacle_inflation_m', 0.12)
        self.declare_parameter('planner_dynamic_obstacle_range_m', 2.5)
        self.declare_parameter('planner_replan_period_sec', 0.25)

        self.phase_topic = self.get_parameter('phase_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.return_path_topic = self.get_parameter('return_path_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        
        # JSON 参数
        self.return_start_json = str(self.get_parameter('return_start_json').value or '').strip()
        self.return_goal_json = str(self.get_parameter('return_goal_json').value or '').strip()
        self.return_waypoints_json = str(self.get_parameter('return_waypoints_json').value or '').strip()
        
        # 坐标系
        self.global_frame_id = str(self.get_parameter('global_frame_id').value).strip() or 'map'
        self.global_yaw_source = str(self.get_parameter('global_yaw_source').value).strip().lower() or 'odom'
        if self.global_yaw_source not in ('auto', 'odom', 'imu'):
            self.global_yaw_source = 'odom'
        self.global_yaw_disagreement = math.radians(
            float(self.get_parameter('global_yaw_disagreement_deg').value)
        )
        
        # 容差
        self.return_waypoint_tolerance = float(self.get_parameter('return_waypoint_tolerance').value)
        self.return_goal_tolerance = float(self.get_parameter('return_goal_tolerance').value)
        self.return_goal_yaw_tolerance = math.radians(
            float(self.get_parameter('return_goal_yaw_tolerance_deg').value)
        )
        self.return_path_timeout_sec = float(self.get_parameter('return_path_timeout_sec').value)
        
        # Pure Pursuit
        self.pure_pursuit_linear_speed = float(self.get_parameter('pure_pursuit_linear_speed').value)
        self.pure_pursuit_lookahead_m = float(self.get_parameter('pure_pursuit_lookahead_m').value)
        self.pure_pursuit_heading_stop = math.radians(
            float(self.get_parameter('pure_pursuit_heading_stop_deg').value)
        )
        self.pure_pursuit_turn_kp = float(self.get_parameter('pure_pursuit_turn_kp').value)
        self.turn_linear_speed = float(self.get_parameter('turn_linear_speed').value)
        self.turn_min_angular_speed = float(self.get_parameter('turn_min_angular_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        
        # A* 规划
        self.use_occupancy_grid_planner = bool(self.get_parameter('use_occupancy_grid_planner').value)
        self.planner_downsample = max(1, int(self.get_parameter('planner_downsample').value))
        self.planner_occupied_threshold = int(self.get_parameter('planner_occupied_threshold').value)
        self.planner_unknown_is_occupied = bool(self.get_parameter('planner_unknown_is_occupied').value)
        self.planner_obstacle_inflation_m = float(self.get_parameter('planner_obstacle_inflation_m').value)
        self.planner_dynamic_obstacle_box_size_m = max(
            0.0,
            float(self.get_parameter('planner_dynamic_obstacle_box_size_m').value),
        )
        self.planner_dynamic_obstacle_inflation_m = float(
            self.get_parameter('planner_dynamic_obstacle_inflation_m').value
        )
        self.planner_dynamic_obstacle_range_m = float(
            self.get_parameter('planner_dynamic_obstacle_range_m').value
        )
        self.planner_replan_period_sec = float(self.get_parameter('planner_replan_period_sec').value)

        self.return_waypoints = self.load_return_waypoints()
        self._log_return_track_summary()

        self.phase = 1
        self.mission_active = False
        self.mission_finished = False
        self.start_after_time = None
        self.current_position = None
        self.current_yaw = None
        self.current_odom_yaw = None
        self.odom_frame_id = 'odom'
        self.path_origin_pose = None
        self.path_origin_yaw = None
        self.path_started_at = None
        self.path_index = 0

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = ReliabilityPolicy.RELIABLE

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        event_qos = QoSProfile(depth=1)
        event_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        event_qos.reliability = ReliabilityPolicy.RELIABLE

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.return_path_pub = self.create_publisher(Path, self.return_path_topic, path_qos)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.create_subscription(Int32, self.phase_topic, self.phase_callback, event_qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        if self.use_occupancy_grid_planner:
            self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
            self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.init_global_path_planner()
        self.publish_state('idle')
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('stage3 param test return navigator ready (param waypoints)')

    def parse_waypoints_json(self, raw_json, param_name, default_speed):
        try:
            raw_waypoints = json.loads(raw_json)
        except json.JSONDecodeError:
            self.get_logger().error(f'{param_name} is invalid, fallback to empty')
            return []

        if not isinstance(raw_waypoints, list):
            self.get_logger().error(f'{param_name} must decode to a list, fallback to empty')
            return []

        sanitized = []
        for index, raw in enumerate(raw_waypoints):
            if not isinstance(raw, dict):
                continue
            yaw_deg = raw.get('yaw_deg')
            sanitized.append({
                'x': float(raw.get('x', 0.0)),
                'y': float(raw.get('y', 0.0)),
                'speed': float(raw.get('speed', default_speed)),
                'yaw_deg': None if yaw_deg is None else float(yaw_deg),
                'description': raw.get('description', f'wp_{index}'),
            })
        return sanitized

    def load_return_waypoints(self):
        """加载返程路点：起点(JSON) + 中间路点(JSON) + 终点(JSON)"""
        # 加载起点
        start = None
        if self.return_start_json and self.return_start_json not in ('[]', ''):
            start_list = self.parse_waypoints_json(self.return_start_json, 'return_start_json', 0.12)
            if start_list:
                start = start_list[0]

        # 加载中间路点
        mid = []
        if self.return_waypoints_json and self.return_waypoints_json not in ('[]', ''):
            mid = self.parse_waypoints_json(self.return_waypoints_json, 'return_waypoints_json',
                                            self.pure_pursuit_linear_speed)

        # 加载终点
        goal_list = self.parse_waypoints_json(self.return_goal_json, 'return_goal_json', 0.10)
        goal = goal_list[0] if goal_list else {
            'x': 0.20, 'y': 0.20, 'speed': 0.10, 'yaw_deg': 100.0,
            'description': 'p_point',
        }

        # 合并
        all_waypoints = []
        if start is not None and start['description'] != goal['description']:
            all_waypoints.append(start)
        all_waypoints.extend(mid)
        all_waypoints.append(goal)

        if not all_waypoints:
            all_waypoints = [start, goal] if start else [goal]

        return all_waypoints

    def _log_return_track_summary(self):
        if not self.return_waypoints:
            self.get_logger().error('no return waypoints loaded')
            return
        start = self.return_waypoints[0]
        goal = self.return_waypoints[-1]
        self.get_logger().info(
            f'return: {len(self.return_waypoints)} waypoints, '
            f'start=({start["x"]:.2f},{start["y"]:.2f}) '
            f'goal=({goal["x"]:.2f},{goal["y"]:.2f})'
        )

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
        self.publish_path_points([])

    def publish_return_path(self):
        if not self.return_waypoints:
            return
        points = [
            (wp['x'], wp['y'])
            for wp in self.return_waypoints
        ]
        self.publish_path_points(points, self.global_frame_id)

    def imu_callback(self, msg):
        self.current_yaw = self.quaternion_to_yaw(msg.orientation)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.current_position = (float(position.x), float(position.y))
        self.current_odom_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
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
            self.last_plan_points = []
            self.last_plan_signature = None
            self.last_plan_at = 0.0
            self.start_after_time = self.get_clock().now().nanoseconds / 1e9 + self.start_delay_sec
            self.clear_return_path()
            self.publish_state('armed')
            start = self.return_waypoints[0] if self.return_waypoints else None
            goal = self.return_waypoints[-1] if self.return_waypoints else {'x': 0.2, 'y': 0.2}
            if start:
                self.publish_feedback(
                    f'阶段三：从 ({start["x"]:.2f},{start["y"]:.2f}) 返航至 P ({goal["x"]:.2f},{goal["y"]:.2f})'
                )
            else:
                self.publish_feedback(
                    f'阶段三：返航至 P ({goal["x"]:.2f},{goal["y"]:.2f})'
                )

    def reset_mission(self):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = False
        self.start_after_time = None
        self.path_origin_pose = None
        self.path_origin_yaw = None
        self.path_started_at = None
        self.path_index = 0
        self.last_plan_points = []
        self.last_plan_signature = None
        self.last_plan_at = 0.0
        self.clear_return_path()
        self.publish_state('idle')

    def start_return_path(self):
        if self.current_global_position() is None:
            return
        if not self.return_waypoints:
            self.fail_mission('阶段三未配置返航航点，无法返回 P 点')
            return

        self.mission_active = True
        self.path_origin_pose = None
        self.path_origin_yaw = None
        self.path_started_at = self.get_clock().now().nanoseconds / 1e9
        self.path_index = 0
        self.publish_return_path()
        self.publish_state('running')
        self.publish_feedback(
            f'阶段三开始返航，共 {len(self.return_waypoints)} 个航点'
        )

    def navigation_pose(self):
        return self.current_global_position()

    def navigation_yaw(self):
        return self.current_global_yaw()

    def waypoint_target_yaw(self, index):
        waypoint = self.return_waypoints[index]
        yaw_deg = waypoint.get('yaw_deg')
        if yaw_deg is not None:
            return math.radians(float(yaw_deg))

        if index < len(self.return_waypoints) - 1:
            next_waypoint = self.return_waypoints[index + 1]
            return math.atan2(
                next_waypoint['y'] - waypoint['y'],
                next_waypoint['x'] - waypoint['x'],
            )

        if index > 0:
            previous_waypoint = self.return_waypoints[index - 1]
            return math.atan2(
                waypoint['y'] - previous_waypoint['y'],
                waypoint['x'] - previous_waypoint['x'],
            )

        return None

    def waypoint_target_position(self, index):
        waypoint = self.return_waypoints[index]
        return (waypoint['x'], waypoint['y'])

    def maybe_advance_waypoint(self, nav_pose):
        while self.path_index < len(self.return_waypoints) - 1:
            waypoint = self.return_waypoints[self.path_index]
            distance = math.hypot(waypoint['x'] - nav_pose[0], waypoint['y'] - nav_pose[1])
            if distance > self.return_waypoint_tolerance:
                return
            self.path_index += 1

    def finish_mission(self):
        self.cmd_pub.publish(Twist())
        self.mission_active = False
        self.mission_finished = True
        self.publish_state('complete')
        goal = self.return_waypoints[-1]
        self.publish_feedback(
            f'阶段三完成，车辆已返回 P 点 ({goal["x"]:.2f}, {goal["y"]:.2f})'
        )

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

        self.run_return_path_stage(now_sec)

    def run_return_path_stage(self, now_sec):
        if self.path_started_at is not None and now_sec - self.path_started_at > self.return_path_timeout_sec:
            self.fail_mission('阶段三 map 航点路径超时，未能返回 P 点')
            return

        nav_pose = self.navigation_pose()
        nav_yaw = self.navigation_yaw()
        if nav_pose is None or nav_yaw is None:
            self.cmd_pub.publish(Twist())
            return

        self.maybe_advance_waypoint(nav_pose)
        final_target_x, final_target_y = self.waypoint_target_position(len(self.return_waypoints) - 1)
        final_distance = math.hypot(final_target_x - nav_pose[0], final_target_y - nav_pose[1])
        if self.path_index >= len(self.return_waypoints) - 1 and final_distance <= self.return_goal_tolerance:
            target_yaw = self.waypoint_target_yaw(len(self.return_waypoints) - 1)
            if target_yaw is not None:
                yaw_error = self.angle_error(target_yaw, nav_yaw)
                if abs(yaw_error) > self.return_goal_yaw_tolerance:
                    angular = self.clamp(self.pure_pursuit_turn_kp * yaw_error, self.max_angular_speed)
                    if abs(angular) < self.turn_min_angular_speed:
                        angular = math.copysign(self.turn_min_angular_speed, yaw_error)
                    self.publish_state('align')
                    self.cmd_pub.publish(self.create_twist(0.0, angular))
                    return

            self.finish_mission()
            return

        target_waypoint = self.return_waypoints[self.path_index]
        target_x, target_y = self.waypoint_target_position(self.path_index)
        if self.use_occupancy_grid_planner:
            planned_points = self.plan_global_path(nav_pose, (target_x, target_y), now_sec)
            if planned_points is None:
                self.publish_state('planner_waiting_for_map')
                self.cmd_pub.publish(Twist())
                return
            if not planned_points:
                self.publish_state('return_planner_blocked')
                self.cmd_pub.publish(Twist())
                return

            self.publish_path_points(planned_points, self.global_frame_id)
            lookahead_point = self.select_path_lookahead_point(
                planned_points,
                self.pure_pursuit_lookahead_m,
            )
            if lookahead_point is not None:
                target_x, target_y = lookahead_point

        target_dx = target_x - nav_pose[0]
        target_dy = target_y - nav_pose[1]
        target_x_robot = math.cos(nav_yaw) * target_dx + math.sin(nav_yaw) * target_dy
        target_y_robot = -math.sin(nav_yaw) * target_dx + math.cos(nav_yaw) * target_dy
        target_distance = math.hypot(target_x_robot, target_y_robot)
        heading_error = math.atan2(target_y_robot, target_x_robot if abs(target_x_robot) > 1e-6 else 1e-6)

        self.publish_state(target_waypoint['description'])
        if target_x_robot <= 0.0:
            angular = self.clamp(self.pure_pursuit_turn_kp * heading_error, self.max_angular_speed)
            if abs(angular) < self.turn_min_angular_speed:
                angular = math.copysign(self.turn_min_angular_speed, heading_error)
            self.cmd_pub.publish(self.create_twist(min(self.turn_linear_speed, 0.04), angular))
            return

        if abs(heading_error) > self.pure_pursuit_heading_stop:
            angular = self.clamp(self.pure_pursuit_turn_kp * heading_error, self.max_angular_speed)
            if abs(angular) < self.turn_min_angular_speed:
                angular = math.copysign(self.turn_min_angular_speed, heading_error)
            self.cmd_pub.publish(self.create_twist(self.turn_linear_speed, angular))
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

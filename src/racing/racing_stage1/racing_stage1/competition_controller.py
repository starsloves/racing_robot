"""Stage 1 mission adapter for the shared Nav2 navigation stack.

S1 owns the mission state and QR handoff only.  Nav2 owns the complete motion
chain: LaserScan -> costmaps -> Smac/MPPI -> cmd_vel.  Keeping that boundary
here prevents a second planner or safety publisher from competing for
``/cmd_vel``.
"""

import json
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from racing_common.process_lifecycle import install_parent_death_signal
from racing_common.racing_logger import terminal_write


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


class CompetitionController(Node):
    """Small lifecycle/mission bridge around ``NavigateToPose``."""

    MISSION_STANDBY = 'standby'
    MISSION_SEARCH_QR = 'search_qr'
    MISSION_QR_LOCKED = 'qr_locked'
    MISSION_RETURN_TO_ENTRY = 'return_to_entry'
    MISSION_HANDOFF_WAIT = 'handoff_wait'

    def __init__(self):
        super().__init__('competition_controller')
        self._declare_parameters()
        self._read_parameters()

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._group = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(String, self.stage1_state_topic, latched)
        self.task_pub = self.create_publisher(String, self.task_topic, latched)
        self.entry_pose_pub = self.create_publisher(PoseStamped, self.entry_pose_topic, latched)
        self.route_pub = self.create_publisher(Path, self.route_topic, latched)
        self.mission_route_pub = self.create_publisher(Path, self.mission_route_topic, latched)

        prefix = self.lifecycle_service_prefix.rstrip('/')
        self.create_service(
            Trigger, f'{prefix}/activate', self._activate_cb, callback_group=self._group)
        self.create_service(
            Trigger, f'{prefix}/release', self._release_cb, callback_group=self._group)

        self.create_subscription(
            String, self.diagnostic_topic, self._diagnostic_cb, latched,
            callback_group=self._group)
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_cb, latched,
            callback_group=self._group)
        self.create_subscription(
            String, self.qr_result_topic, self._qr_cb, 10,
            callback_group=self._group)
        self.create_subscription(
            Path, self.planner_plan_topic, self._plan_cb, 10,
            callback_group=self._group)
        if self.planner_plan_fallback_topic != self.planner_plan_topic:
            self.create_subscription(
                Path, self.planner_plan_fallback_topic, self._plan_cb, 10,
                callback_group=self._group)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self, NavigateToPose, self.navigate_action, callback_group=self._group)
        self.nav2_state_client = self.create_client(
            GetState,
            f'/{self.nav2_lifecycle_node.strip("/")}/get_state',
            callback_group=self._group,
        )

        self._lock = threading.RLock()
        self._released = False
        self._activation_requested = False
        self._motion_enabled = False
        self._ready_published = False
        self._running_published = False
        self._localizer_valid = False
        self._map_received = False
        self._mission_state = self.MISSION_STANDBY
        self._qr_latched = False
        self._qr_task = ''
        self._start_pose = None
        self._current_pose = None
        self._goal_kind = None
        self._goal_generation = 0
        self._goal_handle = None
        self._goal_future = None
        self._cancel_waiting = False
        self._goal_result = None
        self._goal_retry_at = 0.0
        self._goal_retry_count = 0
        self._nav2_state_future = None
        self._nav2_active = False
        self._pending_entry = False
        self._entry_goal_done = False
        self._entry_stable_since = None
        self._entry_announced = False
        self._last_plan = None
        self._last_pose_log_at = 0.0
        self._shutdown_timer = None
        self._timer = self.create_timer(0.1, self._tick, callback_group=self._group)

        self.get_logger().info(
            'S1 mission adapter ready: Nav2 owns /cmd_vel; waiting for localization and map')
        terminal_write('[STARTUP] S1 Nav2 任务适配器已启动，等待定位与 Supervisor activate')

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        defaults = {
            'imu_topic': '/imu/data',  # retained for launch compatibility; TF carries yaw
            'map_topic': '/map',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'diagnostic_topic': 'start_corner_pose_diagnostic',
            'qr_result_topic': 'qr_scan_result',
            'task_topic': 'competition_qr_task',
            'stage2_entry_pose_topic': 'stage2_entry_pose',
            'stage1_state_topic': 'stage1_state',
            'route_topic': 'stage1_route',
            'mission_route_topic': 'stage1_mission_route',
            'planner_plan_topic': '/planner_server/plan',
            'planner_plan_fallback_topic': '/plan',
            'navigate_action': '/navigate_to_pose',
            'nav2_lifecycle_node': 'bt_navigator',
            'lifecycle_service_prefix': '/competition/stage1',
            'qr_goal_x_m': 4.50,
            'qr_goal_y_m': 1.60,
            'qr_goal_yaw_deg': 0.0,
            'channel_entry_x_m': 2.50,
            'channel_entry_y_m': 2.50,
            'channel_entry_yaw_deg': 90.0,
            'channel_entry_tolerance_m': 0.20,
            'channel_entry_yaw_tolerance_deg': 15.0,
            'entry_stable_sec': 0.25,
            'action_server_timeout_sec': 0.0,
            'goal_retry_delay_sec': 1.0,
            'goal_retry_limit': 3,
            'pose_log_period_sec': 0.50,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('standby', True)

    def _read_parameters(self):
        value = lambda name: self.get_parameter(name).value
        self.map_topic = str(value('map_topic'))
        self.map_frame = str(value('map_frame'))
        self.base_frame = str(value('base_frame'))
        self.diagnostic_topic = str(value('diagnostic_topic'))
        self.qr_result_topic = str(value('qr_result_topic'))
        self.task_topic = str(value('task_topic'))
        self.entry_pose_topic = str(value('stage2_entry_pose_topic'))
        self.stage1_state_topic = str(value('stage1_state_topic'))
        self.route_topic = str(value('route_topic'))
        self.mission_route_topic = str(value('mission_route_topic'))
        self.planner_plan_topic = str(value('planner_plan_topic'))
        self.planner_plan_fallback_topic = str(value('planner_plan_fallback_topic'))
        self.navigate_action = str(value('navigate_action'))
        self.nav2_lifecycle_node = str(value('nav2_lifecycle_node'))
        self.lifecycle_service_prefix = str(value('lifecycle_service_prefix'))
        self.qr_goal = (float(value('qr_goal_x_m')), float(value('qr_goal_y_m')))
        self.qr_goal_yaw = math.radians(float(value('qr_goal_yaw_deg')))
        self.entry_goal = (float(value('channel_entry_x_m')), float(value('channel_entry_y_m')))
        self.entry_yaw = math.radians(float(value('channel_entry_yaw_deg')))
        self.entry_tolerance = max(0.05, float(value('channel_entry_tolerance_m')))
        self.entry_yaw_tolerance = math.radians(
            max(1.0, float(value('channel_entry_yaw_tolerance_deg'))))
        self.entry_stable_sec = max(0.0, float(value('entry_stable_sec')))
        self.action_server_timeout = max(0.0, float(value('action_server_timeout_sec')))
        self.goal_retry_delay = max(0.1, float(value('goal_retry_delay_sec')))
        self.goal_retry_limit = max(0, int(value('goal_retry_limit')))
        self.pose_log_period = max(0.2, float(value('pose_log_period_sec')))

    # ------------------------------------------------------------------
    # State and TF
    # ------------------------------------------------------------------
    def _publish_state(self, state):
        self._mission_state = state
        self.state_pub.publish(String(data=state))

    def _diagnostic_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if str(payload.get('state', '')).strip().lower() != 'valid':
            return
        with self._lock:
            self._localizer_valid = True

    def _map_cb(self, _msg):
        with self._lock:
            self._map_received = True

    def _poll_nav2_state_locked(self):
        if self._nav2_state_future is not None or not self.nav2_state_client.service_is_ready():
            return
        future = self.nav2_state_client.call_async(GetState.Request())
        self._nav2_state_future = future
        future.add_done_callback(self._nav2_state_cb)

    def _nav2_state_cb(self, future):
        try:
            active = future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        except Exception:
            active = False
        with self._lock:
            self._nav2_state_future = None
            if active != self._nav2_active:
                self._nav2_active = active
                self.get_logger().info(
                    f'Nav2 lifecycle state={"active" if active else "not_active"}')
            else:
                self._nav2_active = active

    def _lookup_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.0))
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return float(translation.x), float(translation.y), normalize_angle(yaw)

    def _log_real_pose(self, pose):
        """Expose the same map TF pose used by S1 and the web monitor."""
        now = time.monotonic()
        if now - self._last_pose_log_at < self.pose_log_period:
            return
        self._last_pose_log_at = now
        x, y, yaw = pose
        terminal_write(
            f'[POSE_REAL] real_map=({x:.3f},{y:.3f}) '
            f'yaw={math.degrees(yaw):.1f}deg source=map_tf'
        )

    def _publish_mission_route(self):
        message = Path()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        points = []
        if self._start_pose is not None:
            points.append((self._start_pose[0], self._start_pose[1], self._start_pose[2]))
        points.append((self.qr_goal[0], self.qr_goal[1], self.qr_goal_yaw))
        if self._mission_state in (
                self.MISSION_RETURN_TO_ENTRY, self.MISSION_HANDOFF_WAIT):
            points.append((self.entry_goal[0], self.entry_goal[1], self.entry_yaw))
        for x, y, yaw in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            message.poses.append(pose)
        self.mission_route_pub.publish(message)

    def _plan_cb(self, msg):
        if not msg.poses:
            return
        with self._lock:
            self._last_plan = msg
            self.route_pub.publish(msg)

    # ------------------------------------------------------------------
    # Nav2 action bridge
    # ------------------------------------------------------------------
    def _schedule_goal_retry_locked(self, kind, reason):
        if self._goal_retry_count >= self.goal_retry_limit:
            self._goal_result = 'failed'
            self.get_logger().error(
                f'Nav2 {kind} goal failed after {self._goal_retry_count} retries: {reason}')
            return
        self._goal_retry_count += 1
        self._goal_retry_at = time.monotonic() + self.goal_retry_delay
        self._goal_result = None
        if kind == 'entry':
            self._pending_entry = True
        self.get_logger().warning(
            f'Nav2 {kind} goal retry {self._goal_retry_count}/{self.goal_retry_limit} '
            f'in {self.goal_retry_delay:.1f}s: {reason}')

    def _make_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        if hasattr(self, '_clock'):
            goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
        goal.pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
        return goal

    def _send_goal_locked(self, kind, target, yaw):
        if (self._released or not self._nav2_active or
                self._goal_handle is not None or self._goal_future is not None):
            return False
        if not self.nav_client.wait_for_server(timeout_sec=self.action_server_timeout):
            return False
        if kind != self._goal_kind:
            self._goal_retry_count = 0
            self._goal_retry_at = 0.0
        self._goal_generation += 1
        generation = self._goal_generation
        self._goal_kind = kind
        goal = self._make_goal(target[0], target[1], yaw)
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=lambda feedback: self._feedback_cb(feedback, generation))
        self._goal_future = future
        future.add_done_callback(
            lambda result: self._goal_response_cb(result, generation, kind))
        self.get_logger().info(
            f'Nav2 goal sent kind={kind} target=({target[0]:.2f},{target[1]:.2f},'
            f'{math.degrees(yaw):.1f}deg)')
        terminal_write(
            f'[PLAN] 发送目标 kind={kind} 目标=({target[0]:.2f},{target[1]:.2f},'
            f'{math.degrees(yaw):.1f}deg) 规划中...')
        return True

    def _feedback_cb(self, feedback, generation):
        if generation != self._goal_generation:
            return
        distance = getattr(feedback.feedback, 'distance_remaining', None)
        if distance is not None:
            self.get_logger().debug(f'Nav2 {self._goal_kind} distance={distance:.2f}m')
            now = time.monotonic()
            if getattr(self, '_last_feedback_log', 0) + 2.0 < now:
                self._last_feedback_log = now
                terminal_write(
                    f'[NAV] {self._goal_kind} 剩余距离={distance:.2f}m')

    def _goal_response_cb(self, future, generation, kind):
        try:
            handle = future.result()
        except Exception as exc:
            with self._lock:
                if generation == self._goal_generation:
                    self._goal_future = None
                    self._schedule_goal_retry_locked(kind, f'send error: {exc}')
            self.get_logger().error(f'Nav2 goal request failed: {exc}')
            return
        with self._lock:
            if generation != self._goal_generation or self._released:
                if handle.accepted:
                    handle.cancel_goal_async().add_done_callback(
                        lambda _: self._stale_cancel_done_cb())
                else:
                    self._stale_cancel_done_cb()
                return
            self._goal_future = None
            if not handle.accepted:
                self._goal_handle = None
                self._schedule_goal_retry_locked(kind, 'goal rejected')
                return
            self._goal_handle = handle
            terminal_write(f'[PLAN] 目标已接受 kind={kind} 路径规划中...')
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda result: self._goal_result_cb(result, generation, kind))

    def _goal_result_cb(self, future, generation, kind):
        try:
            status = future.result().status
        except Exception as exc:
            status = GoalStatus.STATUS_UNKNOWN
            self.get_logger().error(f'Nav2 result failed: {exc}')
        with self._lock:
            if generation != self._goal_generation or self._released:
                return
            self._goal_handle = None
            self._goal_future = None
            self._goal_result = status
            if status != GoalStatus.STATUS_SUCCEEDED:
                self._schedule_goal_retry_locked(kind, f'goal ended status={status}')
                return
            self._goal_retry_count = 0
            self._goal_retry_at = 0.0
            if kind == 'entry':
                self._entry_goal_done = True
                self._entry_stable_since = None
            else:
                self.get_logger().info('QR search goal reached; waiting for QR result')

    def _cancel_goal_locked(self):
        handle = self._goal_handle
        if handle is None:
            self._cancel_waiting = self._goal_future is not None
            return
        generation = self._goal_generation
        self._cancel_waiting = True
        future = handle.cancel_goal_async()
        future.add_done_callback(lambda _: self._cancel_done_cb(generation))

    def _cancel_done_cb(self, generation):
        with self._lock:
            if generation != self._goal_generation:
                return
            self._goal_handle = None
            self._goal_future = None
            self._cancel_waiting = False

    def _stale_cancel_done_cb(self):
        with self._lock:
            self._goal_handle = None
            self._goal_future = None
            self._cancel_waiting = False

    # ------------------------------------------------------------------
    # QR and lifecycle
    # ------------------------------------------------------------------
    def _qr_cb(self, msg):
        task = msg.data.strip()
        with self._lock:
            if (not task or self._qr_latched or self._released or
                    not self._motion_enabled or self._mission_state != self.MISSION_SEARCH_QR):
                return
            self._qr_latched = True
            self._qr_task = task
            self.task_pub.publish(String(data=task))
            self._publish_state(self.MISSION_QR_LOCKED)
            self._pending_entry = True
            self._entry_goal_done = False
            self._entry_stable_since = None
            self._goal_generation += 1
            self._cancel_goal_locked()
            self._publish_state(self.MISSION_RETURN_TO_ENTRY)
            self._publish_mission_route()
            self.get_logger().info(
                f'QR locked task={task}; switching Nav2 goal to entry '
                f'({self.entry_goal[0]:.2f},{self.entry_goal[1]:.2f},'
                f'{math.degrees(self.entry_yaw):.1f}deg)')

    def _activate_cb(self, _request, response):
        with self._lock:
            if self._released:
                response.success = False
                response.message = 'stage1 already released'
                return response
            self._activation_requested = True
            response.success = True
            response.message = 'stage1 Nav2 mission activation armed'
        return response

    def _release_cb(self, _request, response):
        with self._lock:
            if self._released:
                response.success = True
                response.message = 'stage1 already released'
                return response
            self._released = True
            self._motion_enabled = False
            self._goal_generation += 1
            self._cancel_goal_locked()
            self._publish_state('complete')
            self._shutdown_timer = self.create_timer(
                0.15, self._shutdown_after_release, callback_group=self._group)
        response.success = True
        response.message = 'stage1 Nav2 goal cancelled; process will exit'
        return response

    def _shutdown_after_release(self):
        if rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # Main state loop
    # ------------------------------------------------------------------
    def _tick(self):
        with self._lock:
            if self._released:
                return
            self._poll_nav2_state_locked()
            pose = self._lookup_pose()
            if pose is not None:
                self._current_pose = pose
                self._log_real_pose(pose)
                if self._start_pose is None:
                    self._start_pose = pose
                    self._publish_mission_route()

            action_ready = self.nav_client.wait_for_server(timeout_sec=0.0)
            if (not self._ready_published and self._localizer_valid and
                    self._map_received and pose is not None and action_ready and
                    self._nav2_active):
                self._ready_published = True
                self._publish_state('ready')
                self.get_logger().info(
                    'S1 ready: map, startup localization, TF and Nav2 lifecycle are active')
                terminal_write('[S1] Nav2 就绪，等待 Supervisor 激活')
            elif not self._ready_published:
                missing = []
                if not self._localizer_valid: missing.append('localizer')
                if not self._map_received: missing.append('map')
                if pose is None: missing.append('TF')
                if not action_ready: missing.append('Nav2_action')
                if not self._nav2_active: missing.append('Nav2_lifecycle')
                now = time.monotonic()
                if getattr(self, '_last_wait_log', 0) + 5.0 < now:
                    self._last_wait_log = now
                    self.get_logger().info(
                        f'S1 waiting for: {", ".join(missing)}')
                    terminal_write(f'[S1] 等待: {", ".join(missing)}')

            if not (self._ready_published and self._activation_requested and pose is not None):
                return
            if not self._nav2_active:
                return
            if not self._motion_enabled:
                self._motion_enabled = True
                self._running_published = True
                self._publish_state('running')
                self._publish_state(self.MISSION_SEARCH_QR)
                self._publish_mission_route()
                self.get_logger().info('S1 running: Nav2 owns /cmd_vel; QR scanning armed')

            if self._mission_state == self.MISSION_SEARCH_QR:
                if (self._goal_handle is None and self._goal_future is None and
                        self._goal_result is None and time.monotonic() >= self._goal_retry_at):
                    self._send_goal_locked('qr_search', self.qr_goal, self.qr_goal_yaw)
                return

            if (self._pending_entry and not self._cancel_waiting and
                    self._goal_handle is None and self._goal_future is None and
                    time.monotonic() >= self._goal_retry_at):
                self._pending_entry = False
                self._goal_result = None
                self._send_goal_locked('entry', self.entry_goal, self.entry_yaw)
                return

            if self._mission_state == self.MISSION_RETURN_TO_ENTRY and self._entry_goal_done:
                self._check_entry_locked(pose)

    def _check_entry_locked(self, pose):
        distance = math.hypot(pose[0] - self.entry_goal[0], pose[1] - self.entry_goal[1])
        yaw_error = abs(normalize_angle(pose[2] - self.entry_yaw))
        if distance > self.entry_tolerance or yaw_error > self.entry_yaw_tolerance:
            self._entry_stable_since = None
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self._entry_stable_since is None:
            self._entry_stable_since = now
            return
        if now - self._entry_stable_since < self.entry_stable_sec or self._entry_announced:
            return
        message = PoseStamped()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = pose[0]
        message.pose.position.y = pose[1]
        message.pose.orientation.z = math.sin(pose[2] * 0.5)
        message.pose.orientation.w = math.cos(pose[2] * 0.5)
        self.entry_pose_pub.publish(message)
        self._entry_announced = True
        self._publish_state(self.MISSION_HANDOFF_WAIT)
        self.state_pub.publish(String(data='handoff_ready'))
        self._publish_mission_route()
        self.get_logger().info(
            f'S1 handoff_ready pose=({pose[0]:.3f},{pose[1]:.3f}) '
            f'yaw={math.degrees(pose[2]):.1f}deg')

    def destroy_node(self):
        try:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    install_parent_death_signal()
    rclpy.init(args=args)
    node = CompetitionController()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

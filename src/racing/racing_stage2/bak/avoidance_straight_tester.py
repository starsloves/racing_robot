"""直行避障台架节点：任务编�?+ ROS；S1 几何�?s1_geometry�?""

import math
import threading

import rclpy

from racing_stage2.cmd_vel_stop import (
    init_without_ros_signal_handler,
    install_stop_event,
    publish_stop,
    spin_until_stop,
)
from racing_stage2.direct_inertial_tester import DirectInertialTester
from racing_stage2.s1_geometry import cross_segment_m
from racing_stage2.session_file_log import SessionFileLog


class AvoidanceStraightTester(DirectInertialTester):
    """直行避障台架 —�?Stage1 风格边转边避（继承自 DirectInertialTester）�?
    状态机见父�?_try_avoid_step()�?    直行 ω=0 不纠航，日志写入 session file�?    """

    def __init__(self):
        super().__init__()
        self._last_wait_log_sec = 0.0
        self._load_straight_test_parameters()
        self._session_log = SessionFileLog(
            self._session_log_subdir,
            self._session_log_filename,
            session_title='avoidance straight test session',
        )
        self._extend_segment_timeout()
        self._log_ready()

    def _load_straight_test_parameters(self) -> None:
        self.declare_parameter('straight_distance_m', 3.0)
        self.declare_parameter('straight_speed', 0.20)
        self.declare_parameter('straight_settle_sec', 0.30)
        self.declare_parameter('repeat_straight', False)
        # 覆盖父类默认值：直行台架�?leg1=0.30 leg2=0.40
        self.declare_parameter('avoid_leg_heading_offset_deg', 30.0)
        self.declare_parameter('avoid_leg1_distance_m', 0.30)
        self.declare_parameter('avoid_leg2_distance_m', 0.40)
        self.declare_parameter('avoid_leg_linear_speed', 0.10)
        self.declare_parameter('avoid_turn_linear_speed', 0.08)
        self.declare_parameter('avoid_leg_distance_tol_m', 0.04)
        self.declare_parameter('avoid_turn_angular_speed', 0.40)
        self.declare_parameter('avoid_turn_settle_sec', 0.35)
        self.declare_parameter('avoid_telemetry_interval_sec', 0.20)
        self.declare_parameter('session_log_subdir', 'avoidance_straight_test')
        self.declare_parameter('session_log_filename', 'latest.log')
        self.declare_parameter('wheel_odom_topic', '/odom')

        self.straight_distance_m = max(
            0.1, float(self.get_parameter('straight_distance_m').value)
        )
        self.straight_speed = max(0.05, float(self.get_parameter('straight_speed').value))
        self.straight_settle_sec = max(
            0.0, float(self.get_parameter('straight_settle_sec').value)
        )
        self.repeat_straight = bool(self.get_parameter('repeat_straight').value)

        self._session_log_subdir = (
            str(self.get_parameter('session_log_subdir').value).strip()
            or 'avoidance_straight_test'
        )
        self._session_log_filename = (
            str(self.get_parameter('session_log_filename').value).strip() or 'latest.log'
        )

        # 重新加载几何参数（用本类重新声明的默认值）
        self._avoid_offset_deg = float(self.get_parameter('avoid_leg_heading_offset_deg').value)
        self._avoid_offset_rad = math.radians(self._avoid_offset_deg)
        self._avoid_leg1_distance_m = max(
            0.05, float(self.get_parameter('avoid_leg1_distance_m').value)
        )
        self._avoid_leg2_distance_m = max(
            0.05, float(self.get_parameter('avoid_leg2_distance_m').value)
        )
        self._avoid_leg_linear_speed = max(
            0.02, float(self.get_parameter('avoid_leg_linear_speed').value)
        )
        self._avoid_turn_linear_speed = max(
            0.02, float(self.get_parameter('avoid_turn_linear_speed').value)
        )
        self._avoid_distance_tol_m = max(
            0.0, float(self.get_parameter('avoid_leg_distance_tol_m').value)
        )
        self._avoid_turn_angular_speed = max(
            0.1, float(self.get_parameter('avoid_turn_angular_speed').value)
        )
        self._avoid_turn_settle_sec = max(
            0.1, float(self.get_parameter('avoid_turn_settle_sec').value)
        )
        self._avoid_telemetry_interval_sec = max(
            0.05, float(self.get_parameter('avoid_telemetry_interval_sec').value)
        )

    def _extend_segment_timeout(self) -> None:
        travel_sec = self.straight_distance_m / self.straight_speed
        leg_travel = self._avoid_leg1_distance_m + self._avoid_leg2_distance_m
        avoid_budget_sec = leg_travel / self._avoid_leg_linear_speed + 15.0
        self.segment_timeout = max(
            self.segment_timeout,
            travel_sec * 2.0 + avoid_budget_sec,
        )

    def _log_ready(self) -> None:
        self._log_info(
            f'{self.test_feedback_prefix}节点已就绪：'
            f'直行 {self.straight_distance_m:.2f}m @ {self.straight_speed:.2f}m/s�?
            f'Stage1边转边避 ±{self._avoid_offset_deg:.0f}deg�?
            f'{self._avoid_leg1_distance_m:.2f}m→∓{self._avoid_offset_deg*2:.0f}deg�?
            f'{self._avoid_leg2_distance_m:.2f}m→回 ψ₀�?
            f'ω�?{self._avoid_turn_angular_speed:.2f} 转弯v={self._avoid_turn_linear_speed:.2f} 直行v={self._avoid_leg_linear_speed:.2f}�?
            f'触发={self.detour_obstacle_distance:.2f}m�?
            f'日志={self._session_log.path}'
        )

    def destroy_node(self):
        if getattr(self, '_session_log', None) is not None:
            self._session_log.close()
            self._session_log = None
        super().destroy_node()

    def _log_info(self, message: str) -> None:
        self.get_logger().info(message)
        self._session_log.write(f'[INFO] {message}')

    def log_detour(self, message: str) -> None:
        line = f'{self.test_feedback_prefix}避障: {message}'
        self.get_logger().info(line)
        self._session_log.write(f'[DETOUR] {line}')

    def _log_telemetry(self, message: str) -> None:
        line = f'{self.test_feedback_prefix} {message}'
        self.get_logger().info(line)
        self._session_log.write(f'[TELEM] {line}')

    def publish_feedback(self, text: str) -> None:
        super().publish_feedback(text)
        self._session_log.write(f'[FEEDBACK] {text}')

    def publish_stop(self):
        publish_stop(self.cmd_pub)

    # ─── 直行测试专用覆盖 ─────────────────────────────────────────

    def build_inertial_plan(self, nav_succeeded):
        del nav_succeeded
        return self._build_straight_plan()

    def build_ring_plan(self):
        return []

    def rectangle_segment_label(self, segment):
        description = str((segment or {}).get('description', 'unknown'))
        labels = {
            'straight_settle': '启动停稳',
            'straight_test': f'直行测试 {self.straight_distance_m:.2f}m',
        }
        return labels.get(description, super().rectangle_segment_label(segment))

    def _build_straight_plan(self):
        segments = []
        if self.straight_settle_sec > 0.0:
            segments.append({
                'type': 'pause',
                'duration': self.straight_settle_sec,
                'description': 'straight_settle',
            })
        segments.append({
            'type': 'move',
            'distance_m': self.straight_distance_m,
            'speed': self.straight_speed,
            'description': 'straight_test',
            'allow_detour': True,
        })
        return segments

    def _pose_diagnostic(self) -> str:
        if self.current_position is None:
            return 'pose=不可�?
        cross_cm = self._cross_track_m() * 100.0
        return (
            f'x={self.current_position[0]:.3f} y={self.current_position[1]:.3f} '
            f'yaw={self.format_yaw_deg(self.current_yaw)}deg '
            f'along={self.projected_distance():.3f}m cross={cross_cm:+.1f}cm'
        )

    def _cross_track_m(self) -> float:
        if (
            self.segment_start_pose is None
            or self.current_position is None
            or self.segment_heading is None
        ):
            return 0.0
        return cross_segment_m(
            self.segment_start_pose,
            self.segment_heading,
            self.current_position,
        )

    def _update_move_progress_logs(self) -> None:
        if self.current_segment is None or self.current_segment.get('type') != 'move':
            return

        target_distance = max(
            1e-6, float(self.current_segment.get('distance_m', 0.0))
        )
        progress = max(0.0, min(self.projected_distance(), target_distance))
        ratio = progress / target_distance
        bucket = -1
        if ratio >= 0.75:
            bucket = 3
        elif ratio >= 0.50:
            bucket = 2
        elif ratio >= 0.25:
            bucket = 1

        if bucket > self.last_progress_bucket:
            self.last_progress_bucket = bucket
            if bucket >= 0:
                self._log_info(
                    f'{self.test_feedback_prefix}进度 {bucket * 25}% '
                    f'({progress:.2f}/{target_distance:.2f}m) | {self._pose_diagnostic()}'
                )

        if (
            not self._avoid_active
            and progress >= target_distance - self.distance_tolerance
            and self.last_progress_bucket < 4
        ):
            self.last_progress_bucket = 4
            self.publish_feedback(
                f'{self.test_feedback_prefix}直行到位 | {self._pose_diagnostic()}'
            )

    def run_move_segment(self):
        self._update_move_progress_logs()

        if self._try_avoid_step():
            return

        if self.current_position is None or self.segment_heading is None:
            self.cmd_pub.publish(self.create_twist())
            return

        target_distance = float(self.current_segment['distance_m'])
        if self.projected_distance() >= target_distance - self.distance_tolerance:
            self.cmd_pub.publish(self.create_twist())
            self.start_segment(self.plan_index + 1)
            return

        linear = float(self.current_segment.get('speed', self.corridor_linear_speed))
        self.cmd_pub.publish(self.create_twist(linear, 0.0))

    def try_start_mission(self):
        if self.mission_active or self.mission_finished:
            return

        self.phase = 2
        missing = self._missing_pose_inputs()
        if missing:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_wait_log_sec >= 3.0:
                self._last_wait_log_sec = now
                self.publish_feedback(
                    f'{self.test_feedback_prefix}等待: {", ".join(missing)}'
                )
                self._log_info(
                    f'等待 {", ".join(missing)} | 检�?'
                    f'ros2 topic hz {self._wheel_odom_topic or self.odom_topic}'
                )
                self.reported_waiting_pose = True
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self.start_after_time is None:
            self.start_after_time = now + self.start_delay_sec
            if not self.reported_start_delay:
                self.publish_feedback(
                    f'{self.test_feedback_prefix}{self.start_delay_sec:.2f}s 后开�?| '
                    f'yaw={self.format_yaw_deg(self.current_yaw)}deg'
                )
                self.reported_start_delay = True
            return

        if now < self.start_after_time:
            return

        self.mission_active = True
        self.reported_start = True
        self.publish_feedback(
            f'{self.test_feedback_prefix}开始直�?{self.straight_distance_m:.2f}m '
            f'@ {self.straight_speed:.2f}m/s'
        )
        self.begin_inertial_plan_after_nav(nav_succeeded=True)

    def finish_mission(self):
        if self.repeat_straight:
            self.cmd_pub.publish(self.create_twist())
            self.detour_detection_locked = False
            self.detour_resume_yaw = None
            self._reset_avoid()
            self.publish_feedback(
                f'{self.test_feedback_prefix}重复直行 {self.straight_distance_m:.2f}m'
            )
            self.mission_active = True
            self.mission_finished = False
            self.begin_inertial_plan_after_nav(nav_succeeded=True)
            return

        self.cmd_pub.publish(self.create_twist())
        self.mission_active = False
        self.mission_finished = True
        self.reset_corridor_path_state()
        self._reset_avoid()
        self.publish_state('complete')
        self.publish_feedback(f'{self.test_feedback_prefix}测试结束')


def main(args=None):
    init_without_ros_signal_handler(args)
    node = AvoidanceStraightTester()
    stop_event = threading.Event()
    request_stop = install_stop_event(
        stop_event,
        node.publish_stop,
        cli_topics=['/cmd_vel', '/stage2_cmd_vel'],
    )
    try:
        spin_until_stop(node, stop_event)
    except KeyboardInterrupt:
        request_stop()
    finally:
        request_stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

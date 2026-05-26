"""Offline test runner: same DirectInertialTester control, hardware_sim sensors only."""

from __future__ import annotations

import csv
import math
import os

import rclpy
from rclpy.parameter import Parameter

from racing_stage2_param_test.direct_inertial_tester import (
    FINISH_WORLD_DIST_M,
    DirectInertialTester,
)
from racing_stage2_param_test.hardware_sim import (
    DiffDriveSim,
    build_imu,
    build_laserscan,
    build_odometry,
)
from racing_stage2_param_test.launch_param_loader import load_direct_inertial_test_params
from racing_stage2_param_test.plot_ring_trajectory import plot_trajectory
from racing_stage2_param_test.ring_track import (
    RING_CHANNEL_ENTRY_YAW_RAD,
    SCENARIO_SPECS,
    nominal_mission_finish_pose,
    scenario_obstacles,
)
from racing_stage2_param_test.test_log_paths import (
    debug_log_path,
    ensure_test_log_dir,
    ring_plot_path,
    scenario_log_dir,
    scenario_result_path,
    trajectory_csv_path,
)

OFFLINE_STUCK_NET_MOVE_M = 0.035
OFFLINE_STUCK_TIME_SEC = 10.0


def dict_to_parameter_overrides(param_dict: dict) -> list:
    overrides = []
    for key, value in param_dict.items():
        if isinstance(value, bool):
            overrides.append(Parameter(key, Parameter.Type.BOOL, value))
        elif isinstance(value, int) and not isinstance(value, bool):
            overrides.append(Parameter(key, Parameter.Type.INTEGER, value))
        elif isinstance(value, float):
            overrides.append(Parameter(key, Parameter.Type.DOUBLE, value))
        else:
            overrides.append(Parameter(key, Parameter.Type.STRING, str(value)))
    return overrides


class OfflineDirectInertialTester(DirectInertialTester):
    """Thin offline wrapper: capture cmd_vel, inject simulated sensors, external tick."""

    def __init__(self, world_obstacles, trajectory_path, debug_path, scenario_name=''):
        self.scenario_name = str(scenario_name)
        key = self.scenario_name.strip().lower()
        spec = SCENARIO_SPECS.get(key, ('', 0.0))
        self.scenario_obstacle_segment = spec[0]
        self.sim = DiffDriveSim(x=0.0, y=0.0, yaw=RING_CHANNEL_ENTRY_YAW_RAD)
        self.world_obstacles = list(world_obstacles)
        self.scenario_static_obstacles_world = [
            (float(obstacle['x']), float(obstacle['y']), float(obstacle['r']))
            for obstacle in self.world_obstacles
        ]
        self.trajectory_log_path = trajectory_path
        self._trajectory_header_written = False
        self._pending_cmd = (0.0, 0.0)
        self._offline_sim_time = 0.0

        super().__init__()
        self.set_parameters(dict_to_parameter_overrides(load_direct_inertial_test_params(debug_path)))
        self.apply_vehicle_parameters_from_ros()
        self.phase = 2
        self.task_raw = self.test_direction_raw
        self.direction = self.test_direction

        self.debug_log_path = debug_path
        self.reset_debug_log()
        self.log_parameter_snapshot()

        for timer in list(getattr(self, 'timers', [])):
            try:
                timer.cancel()
            except Exception:
                pass

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        self.record_trajectory(linear_x, angular_z)
        self._pending_cmd = (linear_x, angular_z)

    def publish_feedback(self, text):
        self.get_logger().info(text)

    def publish_state(self, text):
        self.segment_state_label = text

    def publish_emergency_stop(self, reason='estop'):
        self.get_logger().warn(f'离线仿真停车: {reason}')

    def publish_watchdog_zero_hold(self, reason='cmd_vel_watchdog_timeout'):
        del reason

    def publish_path_points(self, points, frame_id=None):
        del points, frame_id

    def clear_corridor_path(self):
        pass

    def destroy_node(self):
        try:
            from rclpy.node import Node

            Node.destroy_node(self)
        except Exception:
            pass

    def record_trajectory(self, linear_x=0.0, angular_z=0.0):
        if not hasattr(self, 'sim'):
            return
        phase = self.avoidance_phase if getattr(self, 'avoidance_active', False) else 'idle'
        row = {
            'x': f'{self.sim.x:.4f}',
            'y': f'{self.sim.y:.4f}',
            'segment': (self.current_segment or {}).get('description', ''),
            'dwa_phase': phase,
            'state': phase,
        }
        write_header = not self._trajectory_header_written
        open_mode = 'a' if self._trajectory_header_written else 'w'
        with open(self.trajectory_log_path, open_mode, newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=row.keys())
            if write_header:
                writer.writeheader()
                self._trajectory_header_written = True
            writer.writerow(row)

    def inject_sensors(self):
        stamp = self.get_clock().now().to_msg()
        self.current_position = (self.sim.x, self.sim.y)
        self.current_yaw = self.sim.yaw
        self.odom_callback(build_odometry(self.sim, stamp))
        self.imu_callback(build_imu(self.sim, stamp))
        self.scan_callback(build_laserscan(self.sim, self.world_obstacles, stamp))

    def integration_step(self, dt):
        linear, angular = self._pending_cmd
        self.sim.step(linear, angular, dt)


def final_pose_finish_distance_m(
    final_pose,
    direction='clockwise',
    first_leg_m=1.10,
    side_leg_m=0.50,
    top_leg_m=2.80,
):
    if final_pose is None:
        return None
    finish = nominal_mission_finish_pose(direction, first_leg_m, side_leg_m, top_leg_m)
    return math.hypot(final_pose[0] - finish[0], final_pose[1] - finish[1])


def scenario_passes(metrics, scenario, mission_finished=False, final_pose=None):
    if metrics.get('stuck', False):
        return False
    if metrics['backward_steps'] != 0 or metrics['min_clearance_m'] <= -0.02:
        return False
    key = scenario.strip().lower()
    if key == 'full_ring_no_obstacle':
        if not mission_finished or final_pose is None:
            return False
        dist = final_pose_finish_distance_m(final_pose)
        return dist is not None and dist <= 0.25
    if key in SCENARIO_SPECS:
        if not mission_finished or final_pose is None:
            return False
        dist = final_pose_finish_distance_m(final_pose)
        return dist is not None and dist <= FINISH_WORLD_DIST_M
    return True


def evaluate_trajectory(rows, obstacles, body_margin=0.20):
    metrics = {
        'backward_steps': 0,
        'min_clearance_m': float('inf'),
        'first_leg_max_x': None,
        'first_leg_min_x': None,
        'first_leg_end_y': None,
        'final_x': None,
        'final_y': None,
    }
    if not rows:
        return metrics

    first = [row for row in rows if row.get('segment') == 'rect_first_leg']
    prev_x = None
    xs = []
    for row in first:
        x = float(row['x'])
        y = float(row['y'])
        xs.append(x)
        # 底边沿 -X（通道后 enter_align）；x 增大视为倒车
        if prev_x is not None and x > prev_x + 0.002:
            metrics['backward_steps'] += 1
        prev_x = x
    if xs:
        metrics['first_leg_max_x'] = max(xs)
        metrics['first_leg_min_x'] = min(xs)
        metrics['first_leg_end_y'] = float(first[-1]['y'])

    for row in rows:
        x = float(row['x'])
        y = float(row['y'])
        for obstacle in obstacles:
            clearance = math.hypot(x - obstacle['x'], y - obstacle['y']) - obstacle['r'] - body_margin
            metrics['min_clearance_m'] = min(metrics['min_clearance_m'], clearance)

    if rows:
        metrics['final_x'] = float(rows[-1]['x'])
        metrics['final_y'] = float(rows[-1]['y'])

    if metrics['min_clearance_m'] == float('inf'):
        metrics['min_clearance_m'] = 0.0
    return metrics


def offline_stuck_watchdog_update(node, dt, checkpoint_xy, idle_elapsed_sec, net_move_m, stuck_time_sec):
    if not getattr(node, 'mission_active', False) or getattr(node, 'mission_finished', False):
        return checkpoint_xy, 0.0, False
    if not hasattr(node, 'sim'):
        return checkpoint_xy, 0.0, False

    x, y = float(node.sim.x), float(node.sim.y)
    if checkpoint_xy is None:
        return (x, y), 0.0, False

    phase = getattr(node, 'avoidance_phase', 'idle')
    now_sec = node.control_now_sec() if hasattr(node, 'control_now_sec') else 0.0
    if phase == 'exit' and hasattr(node, 'goal_direct_phase_elapsed_sec'):
        phase_elapsed = node.goal_direct_phase_elapsed_sec(now_sec)
        dist_e = (
            node.distance_to_segment_plan_end_m()
            if hasattr(node, 'distance_to_segment_plan_end_m')
            else float('inf')
        )
        if phase_elapsed >= 25.0 and dist_e > getattr(node, 'SEGMENT_END_REACH_M', 0.12) * 2.0:
            segment = (getattr(node, 'current_segment', None) or {}).get('description', '?')
            node.write_debug_log(
                'DECISION',
                (
                    f'OFFLINE_STUCK_ABORT segment={segment} phase=exit '
                    f'pos=({x:.3f},{y:.3f}) exit={phase_elapsed:.1f}s dist_E={dist_e:.2f}m'
                ),
            )
            return checkpoint_xy, phase_elapsed, True

    if phase == 'next_leg' and hasattr(node, 'goal_direct_phase_elapsed_sec'):
        phase_elapsed = node.goal_direct_phase_elapsed_sec(now_sec)
        dist_goal = node.distance_to_active_goal_m() if hasattr(node, 'distance_to_active_goal_m') else float('inf')
        if phase_elapsed >= 12.0 and dist_goal > node.goal_reach_tolerance_m() * 2.0:
            segment = (getattr(node, 'current_segment', None) or {}).get('description', '?')
            node.write_debug_log(
                'DECISION',
                (
                    f'OFFLINE_STUCK_ABORT segment={segment} phase={phase} '
                    f'pos=({x:.3f},{y:.3f}) next_leg={phase_elapsed:.1f}s '
                    f'goal_dist={dist_goal:.2f}m'
                ),
            )
            return checkpoint_xy, phase_elapsed, True

    cx, cy = checkpoint_xy
    if math.hypot(x - cx, y - cy) >= net_move_m:
        return (x, y), 0.0, False

    elapsed = float(idle_elapsed_sec) + float(dt)
    if elapsed >= stuck_time_sec:
        segment = (getattr(node, 'current_segment', None) or {}).get('description', '?')
        phase = getattr(node, 'avoidance_phase', 'idle')
        node.write_debug_log(
            'DECISION',
            (
                f'OFFLINE_STUCK_ABORT segment={segment} phase={phase} '
                f'pos=({x:.3f},{y:.3f}) idle_for={elapsed:.1f}s '
                f'net<{net_move_m:.3f}m limit={stuck_time_sec:.1f}s'
            ),
        )
        return checkpoint_xy, elapsed, True
    return checkpoint_xy, elapsed, False


def run_offline_test(
    scenario='rect_first_leg_50',
    max_steps=35000,
    shutdown_context=True,
    stuck_time_sec=OFFLINE_STUCK_TIME_SEC,
    stuck_net_move_m=OFFLINE_STUCK_NET_MOVE_M,
):
    ensure_test_log_dir()
    scenario_log_dir(scenario)
    trajectory_csv = trajectory_csv_path(scenario)
    output_png = ring_plot_path(scenario)
    debug_path = debug_log_path(scenario)
    obstacles = scenario_obstacles(scenario)

    for path in (trajectory_csv, debug_path):
        if os.path.isfile(path):
            os.remove(path)

    if not rclpy.ok():
        rclpy.init()
    node = OfflineDirectInertialTester(obstacles, trajectory_csv, debug_path, scenario_name=scenario)
    dt = 1.0 / max(node.control_rate_hz, 1.0)

    mission_finished = False
    stuck = False
    stuck_elapsed_sec = 0.0
    stuck_checkpoint_xy = None
    try:
        for _ in range(max_steps):
            if node.mission_finished:
                mission_finished = True
                break

            node.inject_sensors()
            node.try_start_mission()
            node.control_loop()
            node.integration_step(dt)
            node.offline_sim_advance(dt)

            stuck_checkpoint_xy, stuck_elapsed_sec, stuck = offline_stuck_watchdog_update(
                node,
                dt,
                stuck_checkpoint_xy,
                stuck_elapsed_sec,
                stuck_net_move_m,
                stuck_time_sec,
            )
            if stuck:
                break
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if shutdown_context and rclpy.ok():
            rclpy.shutdown()

    rows = []
    if os.path.isfile(trajectory_csv):
        with open(trajectory_csv, newline='', encoding='utf-8') as handle:
            sample = handle.readline()
            handle.seek(0)
            if sample.strip().startswith('x,'):
                rows = list(csv.DictReader(handle))
            else:
                for line in handle:
                    parts = [part.strip() for part in line.split(',')]
                    if len(parts) < 2:
                        continue
                    rows.append(
                        {
                            'x': parts[0],
                            'y': parts[1],
                            'segment': parts[2] if len(parts) > 2 else '',
                            'dwa_phase': parts[3] if len(parts) > 3 else '',
                            'state': parts[4] if len(parts) > 4 else '',
                        }
                    )

    metrics = evaluate_trajectory(rows, obstacles)
    metrics['stuck'] = bool(stuck)
    metrics['stuck_time_sec'] = float(stuck_elapsed_sec if stuck else 0.0)
    plot_trajectory(
        trajectory_csv,
        output_png,
        scenario=scenario,
        first_leg_m=node.rectangle_first_leg_m,
        side_leg_m=node.rectangle_side_leg_m,
        top_leg_m=node.rectangle_top_leg_m,
        obstacles=obstacles,
    )
    final_pose = None
    if metrics['final_x'] is not None and metrics['final_y'] is not None:
        final_pose = (metrics['final_x'], metrics['final_y'])
    result_path = scenario_result_path(scenario)
    with open(result_path, 'w', encoding='utf-8') as handle:
        handle.write(f'scenario={scenario}\n')
        handle.write(f'folder={scenario_log_dir(scenario).name}\n')
        handle.write(f'mission_finished={mission_finished}\n')
        handle.write(f'stuck={str(metrics["stuck"]).lower()}\n')
        if metrics['stuck']:
            handle.write(f'stuck_time_sec={metrics["stuck_time_sec"]:.1f}\n')
        handle.write(
            f'backward={metrics["backward_steps"]} '
            f'clearance={metrics["min_clearance_m"]:.3f}m\n'
        )
        if final_pose is not None:
            finish = nominal_mission_finish_pose(
                node.test_direction,
                node.rectangle_first_leg_m,
                node.rectangle_side_leg_m,
                node.rectangle_top_leg_m,
            )
            dist_finish = final_pose_finish_distance_m(
                final_pose,
                node.test_direction,
                node.rectangle_first_leg_m,
                node.rectangle_side_leg_m,
                node.rectangle_top_leg_m,
            )
            dist_origin = math.hypot(final_pose[0], final_pose[1])
            handle.write(
                f'final=({final_pose[0]:.3f},{final_pose[1]:.3f}) '
                f'dist_finish={dist_finish:.3f}m '
                f'dist_origin={dist_origin:.3f}m '
                f'nominal_finish=({finish[0]:.3f},{finish[1]:.3f})\n'
            )
        handle.write(f'trajectory={trajectory_csv}\n')
        handle.write(f'plot={output_png}\n')
    return metrics, trajectory_csv, output_png, mission_finished, final_pose

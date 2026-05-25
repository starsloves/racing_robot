#!/usr/bin/env python3
"""离线闭环：与 direct_inertial_test.launch 相同参数与 DirectInertialTester 控制逻辑。

仅替换：
- 差速底盘积分（代替 /odom_combined）
- 射线圆障碍激光（代替 /scan）

不替换：段状态机、转弯、直行、corridor_track 避障、参数与 build_ring_plan。
"""

import argparse
import csv
import math
import os
import sys

import rclpy
from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Header

from racing_stage2_param_test.direct_inertial_tester import DirectInertialTester
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

# 与 LSN10 / inertial_stage2 常用量一致
SIM_SCAN_ANGLE_MIN = -math.pi
SIM_SCAN_ANGLE_MAX = math.pi
SIM_SCAN_NUM_RAYS = 360
SIM_SCAN_RANGE_MIN = 0.20
SIM_SCAN_RANGE_MAX = 3.0

# 离线专用：位姿长时间几乎不变 → 避障/控制异常，提前终止本场景（不改实车算法）。
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


class DiffDriveSim:
    """差速积分：接收与实车相同的 (v, ω) 指令，带简单加速度限幅。"""

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.linear = 0.0
        self.angular = 0.0

    def step(
        self,
        target_linear,
        target_angular,
        dt,
        max_lin_accel=0.8,
        max_ang_accel=2.5,
    ):
        self.linear += max(-max_lin_accel * dt, min(max_lin_accel * dt, target_linear - self.linear))
        self.angular += max(-max_ang_accel * dt, min(max_ang_accel * dt, target_angular - self.angular))
        self.x += self.linear * math.cos(self.yaw) * dt
        self.y += self.linear * math.sin(self.yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.angular * dt), math.cos(self.yaw + self.angular * dt))


def ray_circle_range(ox, oy, radius, rx, ry, ray_angle, max_range):
    dx = ox - rx
    dy = oy - ry
    ray_dx = math.cos(ray_angle)
    ray_dy = math.sin(ray_angle)
    projection = dx * ray_dx + dy * ray_dy
    if projection <= 0.0:
        return max_range
    perp = abs(dx * ray_dy - dy * ray_dx)
    if perp >= radius:
        return max_range
    chord_half = math.sqrt(max(radius * radius - perp * perp, 0.0))
    hit = projection - chord_half
    if hit <= 0.0:
        return max_range
    return min(hit, max_range)


def build_laserscan(robot, obstacles, stamp, frame_id='base_scan'):
    angle_min = SIM_SCAN_ANGLE_MIN
    angle_max = SIM_SCAN_ANGLE_MAX
    num_rays = SIM_SCAN_NUM_RAYS
    angle_increment = (angle_max - angle_min) / max(num_rays - 1, 1)
    max_range = SIM_SCAN_RANGE_MAX
    ranges = []
    for index in range(num_rays):
        local_angle = angle_min + index * angle_increment
        global_angle = robot.yaw + local_angle
        nearest = max_range
        for obstacle in obstacles:
            nearest = min(
                nearest,
                ray_circle_range(
                    obstacle['x'],
                    obstacle['y'],
                    obstacle['r'],
                    robot.x,
                    robot.y,
                    global_angle,
                    max_range,
                ),
            )
        ranges.append(float(nearest))

    scan = LaserScan()
    scan.header = Header()
    scan.header.stamp = stamp
    scan.header.frame_id = frame_id
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = angle_increment
    scan.range_min = SIM_SCAN_RANGE_MIN
    scan.range_max = max_range
    scan.ranges = ranges
    return scan


class OfflineRingHarness(DirectInertialTester):
    """仅仿真器与轨迹记录；控制与 DirectInertialTester 实车完全一致。"""

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

        # 实车用 timer 驱动 control_loop；离线由 run_offline_test 显式步进。
        for timer in list(getattr(self, 'timers', [])):
            try:
                timer.cancel()
            except Exception:
                pass

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        self.record_trajectory(linear_x, angular_z)
        self._pending_cmd = (linear_x, angular_z)

    # 批量离线：不向 ROS 话题发布，避免多场景连跑时 publisher context 失效。
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

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.sim.x
        odom.pose.pose.position.y = self.sim.y
        odom.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.sim.yaw / 2.0),
            w=math.cos(self.sim.yaw / 2.0),
        )
        self.odom_callback(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        imu.orientation = odom.pose.pose.orientation
        imu.angular_velocity = Vector3(x=0.0, y=0.0, z=self.sim.angular)
        imu.linear_acceleration = Vector3(x=self.sim.linear, y=0.0, z=0.0)
        self.imu_callback(imu)

        scan = build_laserscan(self.sim, self.world_obstacles, stamp)
        self.scan_callback(scan)

    def integration_step(self, dt):
        linear, angular = self._pending_cmd
        self.sim.step(linear, angular, dt)


def final_pose_finish_distance_m(final_pose, direction='clockwise'):
    if final_pose is None:
        return None
    finish = nominal_mission_finish_pose(direction)
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
        return dist is not None and dist <= 0.40
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
    prev_y = None
    xs = []
    for row in first:
        x = float(row['x'])
        y = float(row['y'])
        xs.append(x)
        if prev_y is not None and y < prev_y - 0.002:
            metrics['backward_steps'] += 1
        prev_y = y
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
    """任务进行中：net_move_m 内位移不足且持续 stuck_time_sec → 判定卡死。"""
    if not getattr(node, 'mission_active', False) or getattr(node, 'mission_finished', False):
        return checkpoint_xy, 0.0, False
    if not hasattr(node, 'sim'):
        return checkpoint_xy, 0.0, False

    x, y = float(node.sim.x), float(node.sim.y)
    if checkpoint_xy is None:
        return (x, y), 0.0, False

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
    node = OfflineRingHarness(obstacles, trajectory_csv, debug_path, scenario_name=scenario)
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

            # 与实车一致：位姿/激光回调 → 任务启动判定 → control_loop → 底盘积分 → 仿真时钟。
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
            finish = nominal_mission_finish_pose()
            dist_finish = final_pose_finish_distance_m(final_pose)
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


def main(argv=None):
    parser = argparse.ArgumentParser(description='离线回字绕障（DirectInertialTester 实车同代码，跑完整一圈）')
    parser.add_argument('--scenario', default='rect_first_leg_50')
    parser.add_argument('--max-steps', type=int, default=35000)
    parser.add_argument(
        '--stuck-time-sec',
        type=float,
        default=OFFLINE_STUCK_TIME_SEC,
        help='离线卡死判定：净位移低于 net-move 持续超过此秒数则终止场景',
    )
    parser.add_argument(
        '--stuck-net-move-m',
        type=float,
        default=OFFLINE_STUCK_NET_MOVE_M,
        help='离线卡死判定：相对检查点的净位移低于此值视为几乎未动',
    )
    args = parser.parse_args(argv)

    metrics, traj, png, mission_finished, final_pose = run_offline_test(
        args.scenario,
        max_steps=args.max_steps,
        stuck_time_sec=args.stuck_time_sec,
        stuck_net_move_m=args.stuck_net_move_m,
    )
    print(f'scenario={args.scenario}')
    print(
        f'backward={metrics["backward_steps"]} clearance={metrics["min_clearance_m"]:.3f}m '
        f'stuck={str(metrics["stuck"]).lower()}'
    )
    if metrics['stuck']:
        print(f'stuck_time_sec={metrics["stuck_time_sec"]:.1f}')
    if metrics['first_leg_max_x'] is not None:
        print(
            f'first_leg x=[{metrics["first_leg_min_x"]:.3f},{metrics["first_leg_max_x"]:.3f}] '
            f'end_y={metrics["first_leg_end_y"]:.3f}'
        )
    if final_pose is not None:
        dist = math.hypot(final_pose[0], final_pose[1])
        print(
            f'mission_finished={mission_finished} final=({final_pose[0]:.3f},{final_pose[1]:.3f}) '
            f'dist_origin={dist:.3f}m'
        )
    print(f'轨迹 CSV: {traj}')
    print(f'轨迹图: {png}')
    ok = scenario_passes(metrics, args.scenario, mission_finished, final_pose)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

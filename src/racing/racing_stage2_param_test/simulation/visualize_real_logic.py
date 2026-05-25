#!/usr/bin/env python3
"""绘制 offline_ring_test / real_logic_sim 输出的轨迹 CSV（与实车同逻辑仿真）。"""

import argparse
import os
import sys

from racing_stage2_param_test.plot_ring_trajectory import plot_trajectory
from racing_stage2_param_test.ring_track import scenario_obstacles
from racing_stage2_param_test.test_log_paths import ring_plot_path, trajectory_csv_path


def main(argv=None):
    parser = argparse.ArgumentParser(description='可视化 offline 仿真轨迹')
    parser.add_argument('--scenario', default='rect_first_leg_50')
    parser.add_argument('--csv', default='', help='轨迹 CSV（默认 log/<scenario>_trajectory.csv）')
    parser.add_argument('--png', default='', help='输出 PNG（默认 log/<scenario>_trajectory.png）')
    parser.add_argument('--first-leg-m', type=float, default=1.10)
    parser.add_argument('--side-leg-m', type=float, default=0.50)
    parser.add_argument('--top-leg-m', type=float, default=2.80)
    args = parser.parse_args(argv)

    csv_path = args.csv or str(trajectory_csv_path(args.scenario))
    png_path = args.png or str(ring_plot_path(args.scenario))

    if not os.path.isfile(csv_path):
        print(f'找不到轨迹 CSV: {csv_path}', file=sys.stderr)
        print('请先运行: ros2 run racing_stage2_param_test offline_ring_test --scenario', args.scenario)
        return 1

    obstacles = []
    try:
        obstacles = scenario_obstacles(
            args.scenario,
            first_leg_m=args.first_leg_m,
            side_leg_m=args.side_leg_m,
            top_leg_m=args.top_leg_m,
        )
    except ValueError:
        pass

    plot_trajectory(
        csv_path,
        png_path,
        scenario=args.scenario,
        first_leg_m=args.first_leg_m,
        side_leg_m=args.side_leg_m,
        top_leg_m=args.top_leg_m,
        obstacles=obstacles,
    )
    print(f'已保存: {png_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

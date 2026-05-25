#!/usr/bin/env python3
"""回字参考路线与场景障碍（使用 ring_track 名义几何，非旧版 (0,0) 折线）。"""

import argparse
import os
import sys

import matplotlib.pyplot as plt

from racing_stage2_param_test.plot_ring_trajectory import plot_trajectory
from racing_stage2_param_test.ring_track import (
    list_scenario_names,
    ring_drive_segments,
    ring_nominal_polyline,
    scenario_obstacles,
)
from racing_stage2_param_test.test_log_paths import ring_plot_path, test_log_root


def plot_reference_ring(
    output_path,
    direction='clockwise',
    first_leg_m=1.10,
    side_leg_m=0.50,
    top_leg_m=2.80,
    scenarios=None,
):
    fig, ax = plt.subplots(figsize=(10, 8))

    corners = ring_nominal_polyline(direction, first_leg_m, side_leg_m, top_leg_m)
    if len(corners) >= 2:
        px, py = zip(*corners)
        ax.plot(px, py, 'b--', linewidth=1.5, label='nominal corners')

    for segment in ring_drive_segments(direction, first_leg_m, side_leg_m, top_leg_m):
        sx, sy = segment['start']
        ex, ey = segment['end']
        ax.plot([sx, ex], [sy, ey], 'b-', linewidth=1.0, alpha=0.35)
        mx = (sx + ex) / 2.0
        my = (sy + ey) / 2.0
        ax.text(mx, my, segment['name'], fontsize=8, ha='center')

    scenarios = scenarios or []
    for scenario in scenarios:
        for obstacle in scenario_obstacles(scenario, direction, first_leg_m, side_leg_m, top_leg_m):
            ax.add_patch(
                plt.Circle(
                    (obstacle['x'], obstacle['y']),
                    obstacle['r'],
                    color='red',
                    alpha=0.35,
                )
            )
            ax.text(obstacle['x'], obstacle['y'], scenario, fontsize=7, ha='center')

    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.25)
    ax.legend(loc='best')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Stage2 ring reference (ring_track)')

    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    fig.savefig(output_path, dpi=120, bbox_inches='tight')
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description='绘制 ring_track 名义路线与场景障碍')
    parser.add_argument('--output', default=str(test_log_root() / 'ring_reference.png'))
    parser.add_argument('--trajectory-csv', help='叠加 offline/实车 CSV 轨迹')
    parser.add_argument('--scenario', default='')
    args = parser.parse_args(argv)

    if args.trajectory_csv:
        png = args.output or str(ring_plot_path(args.scenario))
        plot_trajectory(
            args.trajectory_csv,
            png,
            scenario=args.scenario,
            obstacles=scenario_obstacles(args.scenario) if args.scenario else None,
        )
        print(f'轨迹图: {png}')
        return 0

    scenarios = list_scenario_names('per_segment_50')
    plot_reference_ring(args.output, scenarios=scenarios)
    print(f'参考图: {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

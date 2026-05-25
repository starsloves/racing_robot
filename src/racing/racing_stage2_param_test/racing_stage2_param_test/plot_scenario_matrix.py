#!/usr/bin/env python3
"""全场景障碍位置一张图 + 可选叠加离线轨迹。"""

import argparse
import csv
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from racing_stage2_param_test.ring_track import (
    DRIVEN_CW_SEGMENT_ENDPOINTS,
    SCENARIO_SPECS,
    full_ring_plan_polyline,
    list_scenario_names,
    ring_drive_segments,
    scenario_expects_corner_shortcut,
    scenario_obstacles,
)
from racing_stage2_param_test.test_log_paths import scenario_log_dir, summary_log_dir

SEGMENT_COLORS: Dict[str, str] = {
    'rect_first_leg': '#e74c3c',
    'rect_side_1': '#e67e22',
    'rect_top': '#f1c40f',
    'rect_side_2': '#2ecc71',
    'rect_return_origin': '#3498db',
}

def load_trajectory_xy(csv_path: str) -> Tuple[List[float], List[float]]:
    if not os.path.isfile(csv_path):
        return [], []
    with open(csv_path, newline='', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    xs = [float(row['x']) for row in rows if row.get('x')]
    ys = [float(row['y']) for row in rows if row.get('y')]
    return xs, ys


def obstacle_scenarios(group: str = '') -> List[str]:
    names = list_scenario_names(group) if group else sorted(SCENARIO_SPECS.keys())
    return [name for name in names if name != 'full_ring_no_obstacle']


def plot_scenario_matrix(
    output_png: str,
    group: str = '',
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
    overlay_trajectories: Optional[Sequence[str]] = None,
    title: str = 'Stage2 obstacle scenario matrix',
):
    scenarios = obstacle_scenarios(group)
    fig, ax = plt.subplots(figsize=(11, 9))

    plan = full_ring_plan_polyline(direction, first_leg_m, side_leg_m, top_leg_m)
    if len(plan) >= 2:
        px, py = zip(*plan)
        ax.plot(px, py, 'b--', linewidth=1.2, alpha=0.55, label='nominal ring')

    for segment_name, (start, end) in DRIVEN_CW_SEGMENT_ENDPOINTS.items():
        color = SEGMENT_COLORS.get(segment_name, '#555555')
        ax.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color=color,
            linewidth=2.5,
            alpha=0.85,
            solid_capstyle='round',
        )
        mx = (start[0] + end[0]) / 2.0
        my = (start[1] + end[1]) / 2.0
        ax.text(mx, my, segment_name.replace('rect_', ''), fontsize=8, color=color, ha='center')

    for segment in ring_drive_segments(direction, first_leg_m, side_leg_m, top_leg_m):
        sx, sy = segment['start']
        ax.plot(sx, sy, 'ko', markersize=3, alpha=0.35)

    for scenario in scenarios:
        spec = SCENARIO_SPECS.get(scenario)
        if not spec:
            continue
        segment_name, ratio = spec
        obstacles = scenario_obstacles(scenario, direction, first_leg_m, side_leg_m, top_leg_m)
        if not obstacles:
            continue
        obstacle = obstacles[0]
        color = SEGMENT_COLORS.get(segment_name, '#c0392b')
        pct = int(round(float(ratio) * 100.0))
        corner = scenario_expects_corner_shortcut(scenario, direction, first_leg_m, side_leg_m, top_leg_m)
        edge = '#8e44ad' if corner else color
        ax.add_patch(
            plt.Circle(
                (obstacle['x'], obstacle['y']),
                obstacle['r'],
                facecolor=color,
                edgecolor=edge,
                linewidth=1.6 if corner else 0.8,
                alpha=0.55,
            )
        )
        label = f'{pct}%'
        if corner:
            label += '*'
        ax.text(
            obstacle['x'],
            obstacle['y'],
            label,
            fontsize=7,
            ha='center',
            va='center',
            color='black',
            fontweight='bold',
        )

    overlay_trajectories = overlay_trajectories or []
    for scenario in overlay_trajectories:
        csv_path = scenario_log_dir(scenario) / 'trajectory.csv'
        xs, ys = load_trajectory_xy(str(csv_path))
        if xs and ys:
            ax.plot(xs, ys, '-', linewidth=0.9, alpha=0.45, label=f'traj {scenario}')

    legend_handles = [
        Line2D([0], [0], color=color, linewidth=3, label=seg.replace('rect_', ''))
        for seg, color in SEGMENT_COLORS.items()
    ]
    legend_handles.append(
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#8e44ad', markersize=8, label='corner_shortcut*')
    )
    ax.legend(handles=legend_handles, loc='upper left', fontsize=8)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.25)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    count = len(scenarios)
    ax.set_title(f'{title} ({count} scenarios, * = corner_shortcut geom)')

    os.makedirs(os.path.dirname(output_png) or '.', exist_ok=True)
    fig.savefig(output_png, dpi=140, bbox_inches='tight')
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description='绘制全场景障碍位置矩阵图')
    parser.add_argument('--group', default='', help='场景组（空=全部有障场景）')
    parser.add_argument(
        '--output',
        default=str(summary_log_dir() / 'obstacle_scenario_matrix.png'),
    )
    parser.add_argument('--overlay', nargs='*', help='叠加轨迹 CSV 对应场景名')
    args = parser.parse_args(argv)

    plot_scenario_matrix(args.output, group=args.group, overlay_trajectories=args.overlay)
    print(f'障碍矩阵图: {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

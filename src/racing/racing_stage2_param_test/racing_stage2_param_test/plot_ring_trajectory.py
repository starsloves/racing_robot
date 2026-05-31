#!/usr/bin/env python3
"""离线/实车轨迹图：只画实际行驶路径（trajectory.csv）与场景障碍。"""

import csv
import os
from typing import List, Optional, Sequence

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt

from racing_stage2_param_test.ring_track import scenario_obstacles


def load_trajectory(csv_path: str) -> List[dict]:
    if not os.path.isfile(csv_path):
        return []
    with open(csv_path, newline='', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def plot_trajectory(
    csv_path: str,
    output_png: str,
    scenario: str = '',
    direction: str = 'clockwise',
    config_path: Optional[str] = None,
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
    obstacles: Optional[Sequence[dict]] = None,
):
    del first_leg_m, side_leg_m, top_leg_m
    rows = load_trajectory(csv_path)
    xs = [float(row['x']) for row in rows if row.get('x')]
    ys = [float(row['y']) for row in rows if row.get('y')]

    fig, ax = plt.subplots(figsize=(7, 6))

    if xs and ys:
        ax.plot(xs, ys, 'g-', linewidth=1.5, label='trajectory')
        ax.plot(xs[0], ys[0], 'go', markersize=6, label='start')
        ax.plot(xs[-1], ys[-1], 'rs', markersize=6, label='end')

    obs = list(obstacles or [])
    if not obs and scenario:
        obs = scenario_obstacles(scenario, direction, config_path=config_path)
    if obs:
        obstacle = obs[0]
        ax.add_patch(
            plt.Circle(
                (obstacle['x'], obstacle['y']),
                obstacle['r'],
                color='red',
                alpha=0.35,
                label='obstacle',
            )
        )

    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.25)
    ax.legend(loc='best', fontsize=9)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    if scenario:
        ax.set_title(scenario)

    os.makedirs(os.path.dirname(output_png) or '.', exist_ok=True)
    fig.savefig(output_png, dpi=120, bbox_inches='tight')
    plt.close(fig)

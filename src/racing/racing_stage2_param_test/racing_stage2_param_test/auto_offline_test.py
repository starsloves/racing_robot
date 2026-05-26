#!/usr/bin/env python3
"""批量离线绕障场景。"""

import argparse
import sys

import rclpy

from racing_stage2_param_test.offline_runner import (
    OFFLINE_STUCK_TIME_SEC,
    run_offline_test,
    scenario_passes,
)
from racing_stage2_param_test.plot_scenario_matrix import plot_scenario_matrix
from racing_stage2_param_test.ring_track import SCENARIO_GROUPS, list_scenario_names
from racing_stage2_param_test.test_log_paths import summary_log_dir, test_log_root


def resolve_scenarios(group: str, explicit: list) -> list:
    if group:
        return list_scenario_names(group)
    if explicit:
        return list(explicit)
    return list_scenario_names('standard')


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--group',
        choices=sorted(SCENARIO_GROUPS.keys()),
        help='预定义场景组（覆盖 --scenarios）',
    )
    parser.add_argument('--scenarios', nargs='+', help='显式场景列表')
    parser.add_argument(
        '--plot-matrix',
        action='store_true',
        help='测试结束后生成全场景障碍位置矩阵图',
    )
    parser.add_argument(
        '--stuck-time-sec',
        type=float,
        default=OFFLINE_STUCK_TIME_SEC,
        help='离线卡死判定秒数（见 offline_ring_test.OFFLINE_STUCK_TIME_SEC）',
    )
    args = parser.parse_args(argv)

    scenarios = resolve_scenarios(args.group or '', args.scenarios or [])
    matrix_group = args.group or ''
    summary_log_dir()

    if not rclpy.ok():
        rclpy.init()

    summary_path = summary_log_dir() / 'test_summary.txt'
    lines = []
    exit_code = 0
    try:
        for index, scenario in enumerate(scenarios):
            print(f'[{index + 1}/{len(scenarios)}] 开始 {scenario}...', flush=True)
            shutdown = index == len(scenarios) - 1
            metrics, _, _, mission_finished, final_pose = run_offline_test(
                scenario,
                max_steps=35000,
                shutdown_context=shutdown,
                stuck_time_sec=args.stuck_time_sec,
            )
            ok = scenario_passes(metrics, scenario, mission_finished, final_pose)
            status = 'PASS' if ok else 'FAIL'
            if not ok:
                exit_code = 1
            finish = 'yes' if mission_finished else 'no'
            stuck_flag = 'yes' if metrics.get('stuck', False) else 'no'
            line = (
                f'{scenario}: {status} finished={finish} stuck={stuck_flag} '
                f'backward={metrics["backward_steps"]} '
                f'clearance={metrics["min_clearance_m"]:.3f}m'
            )
            lines.append(line)
            print(line, flush=True)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    with open(summary_path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    print(f'汇总: {summary_path}', flush=True)

    if args.plot_matrix:
        matrix_png = summary_log_dir() / 'obstacle_scenario_matrix.png'
        plot_scenario_matrix(
            str(matrix_png),
            group=matrix_group or '',
        )
        print(f'障碍矩阵图: {matrix_png}')

    return exit_code


if __name__ == '__main__':
    sys.exit(main())

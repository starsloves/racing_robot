#!/usr/bin/env python3
"""离线仿真批量入口（与 offline_ring_test 相同闭环，勿使用旧 A* 重实现）。"""

import argparse
import sys

import rclpy

from racing_stage2_param_test.offline_ring_test import run_offline_test

LEGACY_SCENARIO_MAP = {
    'real_logic_scenario_side1': 'rect_side_1_50',
    'real_logic_scenario_top': 'rect_top_50',
    'real_logic_scenario_side2': 'rect_side_2_50',
    'real_logic_scenario_return': 'rect_return_50',
}

DEFAULT_SCENARIOS = [
    'rect_first_leg_50',
    'rect_side_1_50',
    'rect_top_50',
    'rect_side_2_50',
    'rect_return_50',
]


def resolve_scenario(name: str) -> str:
    key = name.strip()
    return LEGACY_SCENARIO_MAP.get(key, key)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='回字绕障离线仿真（DirectInertialTester 实车同代码路径）',
    )
    parser.add_argument(
        '--scenario',
        default='rect_first_leg_50',
        help='场景名（见 ring_track.SCENARIO_SPECS）或 legacy real_logic_scenario_*',
    )
    parser.add_argument('--scenarios', nargs='+', help='批量场景（覆盖 --scenario）')
    parser.add_argument('--max-steps', type=int, default=12000)
    args = parser.parse_args(argv)

    scenarios = [resolve_scenario(s) for s in (args.scenarios or [args.scenario])]

    if not rclpy.ok():
        rclpy.init()

    exit_code = 0
    try:
        for index, scenario in enumerate(scenarios):
            shutdown = index == len(scenarios) - 1
            metrics, traj, png = run_offline_test(
                scenario,
                max_steps=args.max_steps,
                shutdown_context=shutdown,
            )
            ok = metrics['backward_steps'] == 0 and metrics['min_clearance_m'] > -0.02
            status = 'PASS' if ok else 'FAIL'
            if not ok:
                exit_code = 1
            print(
                f'{scenario}: {status} backward={metrics["backward_steps"]} '
                f'clearance={metrics["min_clearance_m"]:.3f}m'
            )
            print(f'  CSV: {traj}')
            print(f'  PNG: {png}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())

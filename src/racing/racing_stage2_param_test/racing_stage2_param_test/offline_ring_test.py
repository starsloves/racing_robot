#!/usr/bin/env python3
"""CLI entry for offline ring tests (delegates to offline_runner)."""

import argparse
import sys

from racing_stage2_param_test.offline_runner import (
    OFFLINE_STUCK_NET_MOVE_M,
    OFFLINE_STUCK_TIME_SEC,
    run_offline_test,
    scenario_passes,
)
from racing_stage2_param_test.ring_track import SCENARIO_SPECS


def main(argv=None):
    parser = argparse.ArgumentParser(description='离线回字绕障（DirectInertialTester 实车同代码）')
    parser.add_argument('--scenario', default='rect_first_leg_50')
    parser.add_argument('--max-steps', type=int, default=35000)
    parser.add_argument('--stuck-time-sec', type=float, default=OFFLINE_STUCK_TIME_SEC)
    parser.add_argument('--stuck-net-move-m', type=float, default=OFFLINE_STUCK_NET_MOVE_M)
    args = parser.parse_args(argv)

    key = args.scenario.strip().lower()
    if key not in SCENARIO_SPECS:
        valid = ', '.join(sorted(SCENARIO_SPECS))
        print(f'未知场景 "{args.scenario}"，可选: {valid}', file=sys.stderr)
        return 1

    metrics, trajectory_csv, output_png, mission_finished, final_pose = run_offline_test(
        scenario=key,
        max_steps=args.max_steps,
        stuck_time_sec=args.stuck_time_sec,
        stuck_net_move_m=args.stuck_net_move_m,
    )
    passed = scenario_passes(metrics, key, mission_finished, final_pose)
    print(
        f'scenario={key} pass={passed} mission_finished={mission_finished} '
        f'backward={metrics["backward_steps"]} clearance={metrics["min_clearance_m"]:.3f}m '
        f'stuck={metrics.get("stuck", False)} trajectory={trajectory_csv} plot={output_png}'
    )
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())

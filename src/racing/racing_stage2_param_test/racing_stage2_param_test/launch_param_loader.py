"""direct_inertial_test.launch 与 offline_ring_test 共用参数（与实车 param 测试一致）。"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory


def load_direct_inertial_test_params(debug_log_path_value: str = '') -> dict:
    stage2_share = get_package_share_directory('racing_stage2')
    yaml_path = os.path.join(stage2_share, 'config', 'inertial_stage2.yaml')
    with open(yaml_path, encoding='utf-8') as handle:
        doc = yaml.safe_load(handle)
    params = dict(doc['stage2_inertial_navigator']['ros__parameters'])
    params.update({
        'test_direction': 'clockwise',
        'test_start_mode': 'auto',
        'assume_channel_entry_yaw': True,
        'rectangle_first_leg_m': 1.10,
        'rectangle_side_leg_m': 0.50,
        'rectangle_top_leg_m': 2.80,
        'detour_obstacle_detect_distance': 1.00,
        'detour_obstacle_clear_distance': 0.65,
        'avoid_watch_distance_m': 0.45,
        'avoid_commit_distance_m': 0.30,
        'avoid_corner_prefer_inside': True,
        'avoid_pass_clearance_m': 0.12,
        'avoid_bypass_max_lateral_m': 0.35,
        'avoid_goal_pass_margin_m': 0.12,
        'avoid_goal_cut_segment_len_m': 0.68,
        'avoid_goal_reach_tol_m': 0.07,
        'avoid_goal_exit_inward_margin_m': 0.17,
    })
    if debug_log_path_value:
        params['debug_log_path'] = debug_log_path_value
    return params

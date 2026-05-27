"""从 ROS 参数重载运行时字段（离线仿真加载 yaml+launch 后与实车一致）。"""

import math


def sync_stage2_runtime_parameters(node):
    node.control_rate_hz = float(node.get_parameter('control_rate_hz').value)
    node.start_delay_sec = float(node.get_parameter('start_delay_sec').value)
    node.corridor_linear_speed = float(node.get_parameter('corridor_linear_speed').value)
    node.ring_linear_speed = float(node.get_parameter('ring_linear_speed').value)
    node.turn_linear_speed = float(node.get_parameter('turn_linear_speed').value)
    node.turn_angular_speed = float(node.get_parameter('turn_angular_speed').value)
    node.turn_min_angular_speed = float(node.get_parameter('turn_min_angular_speed').value)
    node.turn_kp = float(node.get_parameter('turn_kp').value)
    node.heading_kp = float(node.get_parameter('heading_kp').value)
    node.max_angular_speed = float(node.get_parameter('max_angular_speed').value)
    node.distance_tolerance = float(node.get_parameter('distance_tolerance').value)
    node.heading_tolerance = math.radians(float(node.get_parameter('heading_tolerance_deg').value))
    node.segment_timeout = float(node.get_parameter('segment_timeout').value)
    node.pre_loop_plan_json = node.get_parameter('pre_loop_plan_json').value
    node.use_corridor_path = bool(node.get_parameter('use_corridor_path').value)
    node.post_corridor_path_plan_json = node.get_parameter('post_corridor_path_plan_json').value
    node.corridor_path_skip_pre_loop_plan = bool(
        node.get_parameter('corridor_path_skip_pre_loop_plan').value
    )
    node.detour_obstacle_distance = float(node.get_parameter('detour_obstacle_distance').value)


def sync_tester_runtime_parameters(node):
    node.test_direction_raw = str(node.get_parameter('test_direction').value).strip()
    node.test_direction = node.resolve_test_direction(node.test_direction_raw)
    node.test_start_mode = str(node.get_parameter('test_start_mode').value).strip().lower() or 'auto'
    node.assume_channel_entry_yaw = bool(node.get_parameter('assume_channel_entry_yaw').value)
    node.test_feedback_prefix = (
        str(node.get_parameter('test_feedback_prefix').value).strip() or '惯导参数测试'
    )
    node.rectangle_first_leg_m = max(0.0, float(node.get_parameter('rectangle_first_leg_m').value))
    node.rectangle_side_leg_m = max(0.0, float(node.get_parameter('rectangle_side_leg_m').value))
    node.rectangle_top_leg_m = max(0.0, float(node.get_parameter('rectangle_top_leg_m').value))
    node.obstacle_circle_topic = (
        str(node.get_parameter('obstacle_circle_topic').value).strip() or 'detected_obstacle_circles'
    )
    node.obstacle_circle_cluster_distance_threshold = max(
        0.01,
        float(node.get_parameter('obstacle_circle_cluster_distance_threshold').value),
    )
    node.obstacle_circle_min_cluster_points = max(
        1,
        int(node.get_parameter('obstacle_circle_min_cluster_points').value),
    )
    node.obstacle_circle_min_range_m = max(
        0.01,
        float(node.get_parameter('obstacle_circle_min_range_m').value),
    )
    node.obstacle_circle_max_range_m = max(
        node.obstacle_circle_min_range_m,
        float(node.get_parameter('obstacle_circle_max_range_m').value),
    )
    node.obstacle_circle_padding_m = max(
        0.0,
        float(node.get_parameter('obstacle_circle_padding_m').value),
    )
    node.obstacle_circle_min_radius_m = max(
        0.0,
        float(node.get_parameter('obstacle_circle_min_radius_m').value),
    )
    node.obstacle_circle_max_radius_m = max(
        node.obstacle_circle_min_radius_m,
        float(node.get_parameter('obstacle_circle_max_radius_m').value),
    )
    node.obstacle_circle_max_cluster_span_m = max(
        0.05,
        float(node.get_parameter('obstacle_circle_max_cluster_span_m').value),
    )
    node.obstacle_circle_marker_height_m = max(
        0.01,
        float(node.get_parameter('obstacle_circle_marker_height_m').value),
    )
    node.obstacle_circle_path_half_width_m = max(
        0.01,
        float(node.get_parameter('obstacle_circle_path_half_width_m').value),
    )
    node.obstacle_corridor_body_half_width_m = max(
        0.0,
        float(node.get_parameter('obstacle_corridor_body_half_width_m').value),
    )
    node.obstacle_side_fence_center_y_m = max(
        node.obstacle_circle_path_half_width_m,
        float(node.get_parameter('obstacle_side_fence_center_y_m').value),
    )
    node.obstacle_opposite_wall_min_center_x_m = max(
        0.40,
        float(node.get_parameter('obstacle_opposite_wall_min_center_x_m').value),
    )
    node.detour_turn_max_trigger_distance_m = max(
        0.25,
        float(node.get_parameter('detour_turn_max_trigger_distance_m').value),
    )
    node.obstacle_opposite_front_min_distance_m = max(
        0.45,
        float(node.get_parameter('obstacle_opposite_front_min_distance_m').value),
    )
    node.obstacle_circle_planning_margin_m = max(
        0.0,
        float(node.get_parameter('obstacle_circle_planning_margin_m').value),
    )
    node.obstacle_circle_forward_margin_m = max(
        0.0,
        float(node.get_parameter('obstacle_circle_forward_margin_m').value),
    )
    node.detour_follow_min_linear_m = max(
        0.05,
        float(node.get_parameter('detour_follow_min_linear_m').value),
    )
    node.avoid_bypass_max_lateral_m = max(
        0.28,
        float(node.get_parameter('avoid_bypass_max_lateral_m').value),
    )
    node.avoid_watch_distance_m = max(
        0.20,
        float(node.get_parameter('avoid_watch_distance_m').value),
    )
    node.avoid_commit_distance_m = max(
        0.12,
        min(
            node.avoid_watch_distance_m - 0.05,
            float(node.get_parameter('avoid_commit_distance_m').value),
        ),
    )
    node.avoid_bias_yaw_deg = max(8.0, float(node.get_parameter('avoid_bias_yaw_deg').value))
    node.avoid_bias_yaw_max_deg = max(
        node.avoid_bias_yaw_deg,
        float(node.get_parameter('avoid_bias_yaw_max_deg').value),
    )
    node.avoid_pass_clearance_m = max(
        0.04,
        float(node.get_parameter('avoid_pass_clearance_m').value),
    )
    node.avoid_parallel_front_margin_default_m = max(
        0.10,
        float(node.get_parameter('avoid_parallel_front_margin_m').value),
    )
    node.avoid_rejoin_heading_tol = math.radians(
        max(2.0, float(node.get_parameter('avoid_rejoin_heading_tol_deg').value))
    )
    node.avoid_rejoin_lateral_tol_m = max(
        0.03,
        float(node.get_parameter('avoid_rejoin_lateral_tol_m').value),
    )
    node.avoid_corner_zone_before_m = max(
        0.25,
        float(node.get_parameter('avoid_corner_zone_before_m').value),
    )
    node.avoid_corner_zone_after_m = max(
        0.20,
        float(node.get_parameter('avoid_corner_zone_after_m').value),
    )
    node.avoid_corner_apex_box_m = max(
        0.20,
        float(node.get_parameter('avoid_corner_apex_box_m').value),
    )
    node.avoid_corner_prefer_inside = bool(node.get_parameter('avoid_corner_prefer_inside').value)
    node.avoid_corner_outside_margin_m = max(
        0.05,
        float(node.get_parameter('avoid_corner_outside_margin_m').value),
    )
    node.avoid_speed_out_mps = max(0.04, float(node.get_parameter('avoid_speed_out_mps').value))
    node.avoid_speed_pass_mps = max(0.04, float(node.get_parameter('avoid_speed_pass_mps').value))
    node.avoid_speed_rejoin_mps = max(0.04, float(node.get_parameter('avoid_speed_rejoin_mps').value))
    node.avoid_corner_speed_mps = max(0.04, float(node.get_parameter('avoid_corner_speed_mps').value))
    node.avoid_corner_speed_slow_mps = max(
        0.04,
        float(node.get_parameter('avoid_corner_speed_slow_mps').value),
    )
    node.avoid_max_angular_speed = max(
        0.15,
        float(node.get_parameter('avoid_max_angular_speed').value),
    )
    node.avoid_perception_loss_hold_sec = max(
        0.15,
        float(node.get_parameter('avoid_perception_loss_hold_sec').value),
    )
    node.avoid_approach_creep_speed_mps = max(
        0.04,
        float(node.get_parameter('avoid_approach_creep_speed_mps').value),
    )
    node.avoid_approach_speed_ratio = min(
        1.0,
        max(0.10, float(node.get_parameter('avoid_approach_speed_ratio').value)),
    )
    bypass_offset_raw = float(node.get_parameter('avoid_goal_bypass_offset_m').value)
    node.avoid_goal_bypass_offset_m = (
        bypass_offset_raw if bypass_offset_raw > 1e-3 else node.avoid_bypass_max_lateral_m
    )
    node.avoid_goal_pass_margin_m = max(
        0.06,
        float(node.get_parameter('avoid_goal_pass_margin_m').value),
    )
    node.avoid_goal_cut_segment_len_m = max(
        0.40,
        float(node.get_parameter('avoid_goal_cut_segment_len_m').value),
    )
    node.avoid_goal_reach_tol_m = max(
        0.04,
        float(node.get_parameter('avoid_goal_reach_tol_m').value),
    )
    heading_kp_raw = float(node.get_parameter('avoid_goal_heading_kp').value)
    node.avoid_goal_heading_kp = (
        heading_kp_raw if heading_kp_raw > 1e-3 else node.heading_kp
    )
    node.avoid_goal_exit_inward_margin_m = max(
        0.05,
        float(node.get_parameter('avoid_goal_exit_inward_margin_m').value),
    )

    detect_raw = float(node.get_parameter('detour_obstacle_detect_distance').value)
    clear_raw = float(node.get_parameter('detour_obstacle_clear_distance').value)
    min_detect = node.detour_obstacle_distance + 0.12
    if detect_raw > 0.0:
        node.detour_obstacle_detect_distance = max(min_detect, detect_raw)
    else:
        node.detour_obstacle_detect_distance = max(1.00, min_detect)
    if clear_raw > 0.0:
        node.detour_obstacle_clear_distance = clear_raw
    else:
        node.detour_obstacle_clear_distance = node.detour_obstacle_detect_distance - 0.35
    node.detour_obstacle_clear_distance = max(
        node.detour_obstacle_distance + 0.05,
        min(node.detour_obstacle_clear_distance, node.detour_obstacle_detect_distance - 0.08),
    )

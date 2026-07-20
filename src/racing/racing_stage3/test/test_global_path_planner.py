from types import SimpleNamespace

import numpy as np

from racing_stage3.global_path_planner import GlobalPathPlanner


def _make_planner():
    planner = GlobalPathPlanner.__new__(GlobalPathPlanner)
    planner.latest_map = None
    planner._test_grid = np.zeros((6, 6), dtype=bool)
    planner.latest_scan = None
    planner.scan_frame_id = None
    planner.last_plan_points = []
    planner.last_plan_signature = None
    planner.last_plan_at = 0.0
    planner.planner_replan_period_sec = 1.0
    planner.planner_dynamic_obstacle_inflation_m = 0.0
    planner.planner_dynamic_obstacle_box_size_m = 0.25
    planner.planner_dynamic_obstacle_range_m = 0.7
    planner.global_frame_id = 'map'
    planner.node = SimpleNamespace()
    planner._build_static_planner_grid = lambda: (
        planner._test_grid.copy(),
        1.0,
        0.0,
        0.0,
    )
    return planner


def test_plan_path_sets_signature_and_reuses_cache_without_scan():
    planner = _make_planner()

    first_path = planner.plan_path((0.5, 0.5), (4.5, 4.5), now_sec=10.0)
    second_path = planner.plan_path((0.5, 0.5), (4.5, 4.5), now_sec=10.5)

    assert first_path
    assert second_path == first_path
    assert planner.last_plan_signature == ((0, 0), (4, 4))
    assert planner.last_plan_at == 10.0


def test_explicit_forbidden_rectangles_ignore_other_map_artwork():
    planner = GlobalPathPlanner.__new__(GlobalPathPlanner)
    planner.latest_scan = None
    planner.scan_frame_id = None
    planner.forbidden_rectangles = [(1.0, 2.0, 1.0, 2.0)]
    planner.planner_downsample = 1
    planner.planner_obstacle_inflation_m = 0.0
    planner.planner_occupied_threshold = 50
    planner.planner_unknown_is_occupied = False
    planner.static_planner_grid = None
    planner.static_planner_resolution = None
    planner.static_planner_origin = None
    planner.latest_map = SimpleNamespace(
        info=SimpleNamespace(
            width=4,
            height=4,
            resolution=1.0,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
        ),
        # This would be a U-shaped/map-artwork obstacle under raw-map planning.
        data=[100] * 16,
    )

    occupied, _resolution, _origin_x, _origin_y = planner._build_static_planner_grid()

    assert occupied[1, 1]
    assert not occupied[0, 0]
    assert not occupied[3, 3]

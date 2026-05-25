"""回字形测试赛道几何（与 build_ring_plan / AGENTS 一致）。

坐标约定（顺时针，odom）：
- (0, 0) = 入环点（转弯开始）；实车转弯带线速度，走弧线后位姿会变。
- rect_first_leg 从弯后位姿起沿 +Y 直行，里程从弯后点计，不是从 (0,0) 硬钉死。
"""

import math
from typing import Dict, List, Optional, Tuple

Point = Tuple[float, float]

MOVE_SEGMENT_NAMES = (
    'rect_first_leg',
    'rect_side_1',
    'rect_top',
    'rect_side_2',
    'rect_return_origin',
)

SEGMENT_LABELS_ZH = {
    'rect_first_leg': '底边 +Y',
    'rect_side_1': '左边 +X',
    'rect_top': '顶边 -X',
    'rect_side_2': '右边 -Y',
    'rect_return_origin': '回程底边',
}

CLOCKWISE_INWARD_BYPASS_SIDE: Dict[str, int] = {
    'rect_first_leg': -1,
    'rect_side_1': -1,
    'rect_top': -1,
    # 实测弦线横偏：场内为 +Y → cross 为负（见 signed_cross_track_on_driven_segment）
    'rect_side_2': -1,
    'rect_return_origin': -1,
}

RING_ENTRY_POINT: Point = (0.0, 0.0)
RING_CHANNEL_ENTRY_YAW_RAD = 0.0
RING_FIRST_LEG_YAW_RAD = math.pi / 2.0

# 与 inertial_stage2.yaml / 实车 turn 段一致
DEFAULT_TURN_LINEAR_MPS = 0.08
DEFAULT_TURN_ANGULAR_RPS = 0.65
DEFAULT_ENTRY_TURN_DEG = 90.0

CORRIDOR_ENTRY_LENGTH_M = 0.80


def post_turn_pose_after_enter_align(
    direction: str = 'clockwise',
    entry_point: Point = RING_ENTRY_POINT,
    turn_linear_mps: float = DEFAULT_TURN_LINEAR_MPS,
    turn_angular_rps: float = DEFAULT_TURN_ANGULAR_RPS,
    entry_turn_deg: float = DEFAULT_ENTRY_TURN_DEG,
) -> Point:
    """入环转弯弧线终点（与实车 turn_linear + turn_angular 一致，坐标会偏移）。"""
    angle = math.radians(abs(float(entry_turn_deg)))
    if turn_angular_rps < 1e-6:
        return entry_point
    radius = float(turn_linear_mps) / float(turn_angular_rps)
    if direction == 'clockwise':
        dx = radius * (1.0 - math.cos(angle))
        dy = radius * math.sin(angle)
    else:
        dx = -radius * (1.0 - math.cos(angle))
        dy = radius * math.sin(angle)
    return (entry_point[0] + dx, entry_point[1] + dy)


def ring_post_turn_origin(
    direction: str = 'clockwise',
    turn_linear_mps: float = DEFAULT_TURN_LINEAR_MPS,
    turn_angular_rps: float = DEFAULT_TURN_ANGULAR_RPS,
) -> Point:
    return post_turn_pose_after_enter_align(
        direction,
        turn_linear_mps=turn_linear_mps,
        turn_angular_rps=turn_angular_rps,
    )


RING_POST_TURN_ORIGIN: Point = ring_post_turn_origin()

# 无障整圈离线实测直行段端点（clockwise, 1.10/0.50/2.80 m）。
# 场景锥桶必须落在此线上，否则车会“蹭到但未触发避障”。
DRIVEN_CW_SEGMENT_ENDPOINTS: Dict[str, Tuple[Point, Point]] = {
    'rect_first_leg': ((0.134, 0.129), (0.134, 1.197)),
    'rect_side_1': ((0.263, 1.349), (0.723, 1.352)),
    'rect_top': ((0.874, 1.222), (0.874, -1.542)),
    'rect_side_2': ((0.745, -1.694), (0.285, -1.696)),
    'rect_return_origin': ((0.134, -1.566), (0.134, -0.507)),
}


def nominal_mission_finish_pose(direction: str = 'clockwise') -> Point:
    """整圈结束时的名义终点（回程底边段末端，非通道入口原点）。"""
    if direction == 'clockwise' and 'rect_return_origin' in DRIVEN_CW_SEGMENT_ENDPOINTS:
        return DRIVEN_CW_SEGMENT_ENDPOINTS['rect_return_origin'][1]
    endpoints = segment_endpoints_nominal(direction)
    return endpoints['rect_return_origin'][1]


def driven_segment_endpoints(
    segment_name: str,
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> Optional[Tuple[Point, Point]]:
    """当前赛段实测直行线端点（clockwise 用 DRIVEN_CW，否则名义折线）。"""
    if direction == 'clockwise' and segment_name in DRIVEN_CW_SEGMENT_ENDPOINTS:
        return DRIVEN_CW_SEGMENT_ENDPOINTS[segment_name]
    endpoints = segment_endpoints_nominal(direction, first_leg_m, side_leg_m, top_leg_m)
    if segment_name not in endpoints:
        return None
    return endpoints[segment_name]


def signed_cross_track_on_driven_segment(
    world_xy: Point,
    segment_name: str,
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> Optional[float]:
    """相对实测直行线的有符号横偏（左正），与 segment_lateral_offset_m 同号约定。"""
    endpoints = driven_segment_endpoints(
        segment_name, direction, first_leg_m, side_leg_m, top_leg_m
    )
    if endpoints is None:
        return None
    sx, sy = float(endpoints[0][0]), float(endpoints[0][1])
    ex, ey = float(endpoints[1][0]), float(endpoints[1][1])
    px, py = float(world_xy[0]), float(world_xy[1])
    vx, vy = ex - sx, ey - sy
    length_sq = vx * vx + vy * vy
    if length_sq < 1e-9:
        return 0.0
    length = math.sqrt(length_sq)
    ux, uy = vx / length, vy / length
    wx, wy = px - sx, py - sy
    return wx * (-uy) + wy * ux


def driven_segment_length_m(
    segment_name: str,
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> Optional[float]:
    endpoints = driven_segment_endpoints(
        segment_name, direction, first_leg_m, side_leg_m, top_leg_m
    )
    if endpoints is None:
        return None
    sx, sy = float(endpoints[0][0]), float(endpoints[0][1])
    ex, ey = float(endpoints[1][0]), float(endpoints[1][1])
    return math.hypot(ex - sx, ey - sy)


def point_on_driven_segment(
    segment_name: str,
    along_m: float,
    lateral_m: float = 0.0,
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> Optional[Point]:
    """弦线上沿程 + 左正法向横偏（与 signed_cross_track 同号）。"""
    endpoints = driven_segment_endpoints(
        segment_name, direction, first_leg_m, side_leg_m, top_leg_m
    )
    if endpoints is None:
        return None
    sx, sy = float(endpoints[0][0]), float(endpoints[0][1])
    ex, ey = float(endpoints[1][0]), float(endpoints[1][1])
    vx, vy = ex - sx, ey - sy
    length = math.hypot(vx, vy)
    if length < 1e-9:
        return sx, sy
    ux, uy = vx / length, vy / length
    nx, ny = -uy, ux
    t = max(0.0, min(1.0, float(along_m) / length))
    px = sx + ux * (t * length)
    py = sy + uy * (t * length)
    return px + nx * float(lateral_m), py + ny * float(lateral_m)


def segment_end_goal_world(
    segment_name: str,
    direction: str = 'clockwise',
    inward_margin_m: float = 0.17,
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> Optional[Point]:
    """段末目标点：沿弦线终点，再向场内缩 inward_margin_m。"""
    endpoints = driven_segment_endpoints(
        segment_name, direction, first_leg_m, side_leg_m, top_leg_m
    )
    if endpoints is None:
        return None
    sx, sy = float(endpoints[0][0]), float(endpoints[0][1])
    ex, ey = float(endpoints[1][0]), float(endpoints[1][1])
    vx, vy = ex - sx, ey - sy
    length = math.hypot(vx, vy)
    if length < 1e-9:
        return ex, ey
    ux, uy = vx / length, vy / length
    nx, ny = -uy, ux
    preferred = preferred_bypass_side_for_segment(segment_name, direction) or -1
    margin = max(0.0, float(inward_margin_m))
    # 与 point_on_driven_segment 同号：lateral = preferred * margin（preferred=-1 → 场内 +Y）
    inward_lat = float(preferred) * margin
    back = min(margin * 0.35, length * 0.12)
    ix = ex - ux * back + nx * inward_lat
    iy = ey - uy * back + ny * inward_lat
    return ix, iy


def progress_on_driven_segment_m(
    world_xy: Point,
    segment_name: str,
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> Optional[float]:
    """沿实测直行线的里程（0=起点端，length=终点端）。"""
    endpoints = driven_segment_endpoints(
        segment_name, direction, first_leg_m, side_leg_m, top_leg_m
    )
    if endpoints is None:
        return None
    sx, sy = float(endpoints[0][0]), float(endpoints[0][1])
    ex, ey = float(endpoints[1][0]), float(endpoints[1][1])
    px, py = float(world_xy[0]), float(world_xy[1])
    vx, vy = ex - sx, ey - sy
    length_sq = vx * vx + vy * vy
    if length_sq < 1e-9:
        return 0.0
    t = ((px - sx) * vx + (py - sy) * vy) / length_sq
    t = max(0.0, min(1.0, float(t)))
    return t * math.sqrt(length_sq)


def expected_point_on_driven_segment(
    segment_name: str,
    ratio: float = 0.5,
    direction: str = 'clockwise',
) -> Optional[Point]:
    """障碍/标定用：落在实测行驶直行线上，而非纯名义拐角折线。"""
    if direction != 'clockwise':
        endpoints = segment_endpoints_nominal(direction)
        if segment_name not in endpoints:
            return None
        start, end = endpoints[segment_name]
    elif segment_name not in DRIVEN_CW_SEGMENT_ENDPOINTS:
        return None
    else:
        start, end = DRIVEN_CW_SEGMENT_ENDPOINTS[segment_name]
    t = max(0.0, min(1.0, float(ratio)))
    return (
        start[0] + t * (end[0] - start[0]),
        start[1] + t * (end[1] - start[1]),
    )


def ring_nominal_corners(
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
    turn_linear_mps: float = DEFAULT_TURN_LINEAR_MPS,
    turn_angular_rps: float = DEFAULT_TURN_ANGULAR_RPS,
) -> List[Point]:
    """回字拐角（弯后起点起算）。"""
    first = float(first_leg_m)
    side = float(side_leg_m)
    top = float(top_leg_m)
    origin = post_turn_pose_after_enter_align(
        direction,
        turn_linear_mps=turn_linear_mps,
        turn_angular_rps=turn_angular_rps,
    )
    ox, oy = origin
    if direction == 'clockwise':
        return [
            origin,
            (ox, oy + first),
            (ox + side, oy + first),
            (ox + side, oy + first - top),
            (ox, oy + first - top),
            origin,
        ]
    return [
        origin,
        (ox, oy - first),
        (ox - side, oy - first),
        (ox - side, oy - first + top),
        (ox, oy - first + top),
        origin,
    ]


def ring_nominal_polyline(
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> List[Point]:
    """闭合拐角折线（6 点）。"""
    return ring_nominal_corners(direction, first_leg_m, side_leg_m, top_leg_m)


def segment_endpoints_nominal(
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
    turn_linear_mps: float = DEFAULT_TURN_LINEAR_MPS,
    turn_angular_rps: float = DEFAULT_TURN_ANGULAR_RPS,
) -> Dict[str, Tuple[Point, Point]]:
    corners = ring_nominal_corners(
        direction,
        first_leg_m,
        side_leg_m,
        top_leg_m,
        turn_linear_mps=turn_linear_mps,
        turn_angular_rps=turn_angular_rps,
    )
    return {
        MOVE_SEGMENT_NAMES[index]: (corners[index], corners[index + 1])
        for index in range(len(MOVE_SEGMENT_NAMES))
    }


def ring_drive_segments(
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> List[dict]:
    """入环转弯后按行驶顺序的直行段（每段独立，避免回程与去程底边画成一条线）。"""
    endpoints = segment_endpoints_nominal(direction, first_leg_m, side_leg_m, top_leg_m)
    segments = []
    for name in MOVE_SEGMENT_NAMES:
        start, end = endpoints[name]
        segments.append(
            {
                'name': name,
                'label': SEGMENT_LABELS_ZH.get(name, name),
                'start': start,
                'end': end,
                'length_m': math.hypot(end[0] - start[0], end[1] - start[1]),
            }
        )
    return segments


# 离线测试一律跑整圈；绘图也一律叠整圈名义线。
FULL_RING_PLOT_SCENARIOS = frozenset({'full_ring_no_obstacle'})

_SEGMENT_SCENARIO_PREFIX = {
    'rect_first_leg': 'rect_first_leg',
    'rect_side_1': 'rect_side_1',
    'rect_top': 'rect_top',
    'rect_side_2': 'rect_side_2',
    'rect_return_origin': 'rect_return',
}

# 各直行段障碍沿程比例（相对段内里程 0→1）。
# 短竖边 (~0.46 m)：50%/75% 常触发 corner_shortcut（镜像回正超出段末）。
# 长边 (底边/顶边/回程)：额外 90% 压测段末绕障。
_SEGMENT_OBSTACLE_RATIOS: Dict[str, Tuple[float, ...]] = {
    'rect_first_leg': (0.25, 0.50, 0.75, 0.90),
    'rect_side_1': (0.25, 0.50, 0.75),
    'rect_top': (0.25, 0.50, 0.75, 0.90),
    'rect_side_2': (0.25, 0.50, 0.75),
    'rect_return_origin': (0.25, 0.50, 0.75, 0.90),
}


def _segment_scenario_prefix(segment_name: str) -> str:
    return _SEGMENT_SCENARIO_PREFIX.get(segment_name, segment_name)


def build_scenario_specs() -> Dict[str, Tuple[str, float]]:
    specs: Dict[str, Tuple[str, float]] = {'full_ring_no_obstacle': ('', 0.0)}
    for segment_name in MOVE_SEGMENT_NAMES:
        prefix = _segment_scenario_prefix(segment_name)
        for ratio in _SEGMENT_OBSTACLE_RATIOS.get(segment_name, (0.50,)):
            pct = int(round(float(ratio) * 100.0))
            specs[f'{prefix}_{pct}'] = (segment_name, float(ratio))
    return specs


SCENARIO_SPECS = build_scenario_specs()

_SEGMENT_PREFIX_ZH = {
    'rect_first_leg': '底边',
    'rect_side_1': '左边',
    'rect_top': '顶边',
    'rect_side_2': '右边',
    'rect_return': '回程',
}


def build_scenario_folder_zh() -> Dict[str, str]:
    folders: Dict[str, str] = {'full_ring_no_obstacle': '无障整圈'}
    for name, (segment_name, ratio) in SCENARIO_SPECS.items():
        if not segment_name:
            continue
        prefix = _SEGMENT_SCENARIO_PREFIX.get(segment_name, segment_name)
        label = _SEGMENT_PREFIX_ZH.get(prefix, prefix)
        pct = int(round(float(ratio) * 100.0))
        folders[name] = f'{label}_{pct}%'
    return folders


SCENARIO_FOLDER_ZH = build_scenario_folder_zh()
SUMMARY_FOLDER_ZH = '汇总'


def scenario_folder_name(scenario: str) -> str:
    key = scenario.strip().lower()
    return SCENARIO_FOLDER_ZH.get(key, scenario.strip() or 'unknown')

SCENARIO_GROUPS: Dict[str, Tuple[str, ...]] = {
    # 快速冒烟：无障 + 底边三档
    'smoke': (
        'full_ring_no_obstacle',
        'rect_first_leg_25',
        'rect_first_leg_50',
        'rect_first_leg_75',
    ),
    # 默认批量：各段中线 + 底边全档（与历史默认接近，多 25/75）
    'standard': (
        'rect_first_leg_25',
        'rect_first_leg_50',
        'rect_first_leg_75',
        'rect_side_1_50',
        'rect_side_2_50',
        'rect_top_50',
        'rect_return_50',
        'full_ring_no_obstacle',
    ),
    # 每段仅 50% 障碍
    'per_segment_50': tuple(
        name for name, (seg, ratio) in SCENARIO_SPECS.items()
        if seg and abs(ratio - 0.50) < 1e-6
    ),
    # 段末 / corner_shortcut 压测
    'corner_stress': (
        'rect_first_leg_75',
        'rect_first_leg_90',
        'rect_side_1_75',
        'rect_side_2_75',
        'rect_top_90',
        'rect_return_90',
    ),
    # 全矩阵（19 个有障 + 无障）
    'full': tuple(sorted(SCENARIO_SPECS.keys())),
}


def list_scenario_names(group: str = '') -> List[str]:
    key = group.strip().lower()
    if not key:
        return sorted(SCENARIO_SPECS.keys())
    if key not in SCENARIO_GROUPS:
        valid = ', '.join(sorted(SCENARIO_GROUPS))
        raise ValueError(f'未知场景组 "{group}"，可选: {valid}')
    return list(SCENARIO_GROUPS[key])


def scenario_expects_corner_shortcut(
    scenario: str,
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
    mirror_half_m: float = 0.26,
) -> Optional[bool]:
    """几何预判：镜像半宽回正点是否超出段末（与 corridor_mirror_geometry 一致）。"""
    key = scenario.strip().lower()
    if key not in SCENARIO_SPECS:
        return None
    segment_name, ratio = SCENARIO_SPECS[key]
    if not segment_name:
        return None
    s_obs = obstacle_along_segment_nominal(
        segment_name,
        ratio,
        direction,
        first_leg_m,
        side_leg_m,
        top_leg_m,
    )
    if s_obs is None:
        return None
    endpoints = segment_endpoints_nominal(direction, first_leg_m, side_leg_m, top_leg_m)
    start, end = endpoints[segment_name]
    s_end = math.hypot(end[0] - start[0], end[1] - start[1])
    return (s_obs + mirror_half_m) > s_end


def full_ring_plan_polyline(
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
    turn_linear_mps: float = DEFAULT_TURN_LINEAR_MPS,
    turn_angular_rps: float = DEFAULT_TURN_ANGULAR_RPS,
) -> List[Point]:
    """整圈名义折线（入环弯后起点 → 各直行段拐角 → 回起点）。"""
    return ring_nominal_corners(
        direction,
        first_leg_m,
        side_leg_m,
        top_leg_m,
        turn_linear_mps=turn_linear_mps,
        turn_angular_rps=turn_angular_rps,
    )


def scenario_plan_polyline(
    scenario: str,
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> List[Point]:
    """当前测试场景对应直行段名义线（转弯完成后那一段，不含整圈）。"""
    key = scenario.strip().lower()
    if key not in SCENARIO_SPECS:
        return []
    segment_name, _ratio = SCENARIO_SPECS[key]
    endpoints = segment_endpoints_nominal(direction, first_leg_m, side_leg_m, top_leg_m)
    if segment_name not in endpoints:
        return []
    driven = expected_point_on_driven_segment(segment_name, 0.0, direction)
    driven_end = expected_point_on_driven_segment(segment_name, 1.0, direction)
    if driven is not None and driven_end is not None:
        return [driven, driven_end]
    start, end = endpoints[segment_name]
    return [start, end]


def ring_drive_reference_polyline(
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> List[Point]:
    """整圈行驶顺序（仅当需要画全环时用）。"""
    points = [RING_POST_TURN_ORIGIN]
    for segment in ring_drive_segments(direction, first_leg_m, side_leg_m, top_leg_m):
        points.append(segment['end'])
    return points


def corridor_to_ring_entry_polyline(
    corridor_length_m: float = CORRIDOR_ENTRY_LENGTH_M,
) -> List[Point]:
    """通道示意：沿 -X 进入 (0,0)，在 (0,0) 处完成入环转弯（不计入段内里程）。"""
    length = max(0.2, float(corridor_length_m))
    return [(-length, 0.0), RING_POST_TURN_ORIGIN]


def nominal_move_heading(
    segment_name: str,
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> Optional[float]:
    endpoints = segment_endpoints_nominal(direction, first_leg_m, side_leg_m, top_leg_m)
    if segment_name not in endpoints:
        return None
    start, end = endpoints[segment_name]
    return math.atan2(end[1] - start[1], end[0] - start[0])


def obstacle_along_segment_nominal(
    segment_name: str,
    ratio: float = 0.5,
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> Optional[float]:
    endpoints = segment_endpoints_nominal(direction, first_leg_m, side_leg_m, top_leg_m)
    if segment_name not in endpoints:
        return None
    start, end = endpoints[segment_name]
    span = math.hypot(end[0] - start[0], end[1] - start[1])
    return max(0.0, min(1.0, float(ratio))) * span


def expected_point_on_segment(
    segment_name: str,
    ratio: float = 0.5,
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> Optional[Point]:
    endpoints = segment_endpoints_nominal(direction, first_leg_m, side_leg_m, top_leg_m)
    if segment_name not in endpoints:
        return None
    start, end = endpoints[segment_name]
    t = max(0.0, min(1.0, float(ratio)))
    return (
        start[0] + t * (end[0] - start[0]),
        start[1] + t * (end[1] - start[1]),
    )


def preferred_bypass_side_for_segment(
    segment_name: str,
    direction: str = 'clockwise',
) -> Optional[int]:
    if direction == 'clockwise':
        return CLOCKWISE_INWARD_BYPASS_SIDE.get(segment_name)
    mirrored = {key: -value for key, value in CLOCKWISE_INWARD_BYPASS_SIDE.items()}
    return mirrored.get(segment_name)


def scenario_obstacles(
    scenario: str,
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
    radius: float = 0.12,
) -> List[dict]:
    key = scenario.strip().lower()
    if key not in SCENARIO_SPECS:
        valid = ', '.join(sorted(SCENARIO_SPECS))
        raise ValueError(f'未知场景 "{scenario}"，可选: {valid}')
    if key == 'full_ring_no_obstacle':
        return []
    segment_name, ratio = SCENARIO_SPECS[key]
    center = expected_point_on_driven_segment(
        segment_name,
        ratio=ratio,
        direction=direction,
    )
    if center is None:
        center = expected_point_on_segment(
            segment_name,
            ratio=ratio,
            direction=direction,
            first_leg_m=first_leg_m,
            side_leg_m=side_leg_m,
            top_leg_m=top_leg_m,
        )
    if center is None:
        return []
    return [
        {
            'segment': segment_name,
            'ratio': ratio,
            'x': float(center[0]),
            'y': float(center[1]),
            'r': float(radius),
            'label': key,
        }
    ]

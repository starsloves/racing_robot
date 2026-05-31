"""Load ring move segments from YAML world-coordinate config (S, E, ψ)."""

from __future__ import annotations

import math
import os
from typing import Dict, List, Optional, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

from ament_index_python.packages import get_package_share_directory

from . import world_segment

Point = Tuple[float, float]

MOVE_SEGMENT_ORDER = (
    'rect_first_leg',
    'rect_side_1',
    'rect_top',
    'rect_side_2',
    'rect_return_origin',
)

_CACHE: Dict[str, dict] = {}


def _package_config_path(filename: str) -> str:
    share = get_package_share_directory('racing_stage2_param_test')
    return os.path.join(share, 'config', filename)


def default_config_path(direction: str) -> str:
    d = str(direction or 'clockwise').strip().lower()
    if d in ('counterclockwise', 'ccw', 'anticlockwise'):
        return _package_config_path('field_track_counterclockwise.yaml')
    return _package_config_path('field_track_clockwise.yaml')


def resolve_config_path(direction: str, config_path: Optional[str] = None) -> str:
    explicit = str(config_path or '').strip()
    if explicit:
        return os.path.abspath(explicit)
    return default_config_path(direction)


def _along_length(start_xy: Point, end_xy: Point, heading_rad: float) -> float:
    dx = float(end_xy[0]) - float(start_xy[0])
    dy = float(end_xy[1]) - float(start_xy[1])
    along = dx * math.cos(heading_rad) + dy * math.sin(heading_rad)
    return max(1e-6, float(along))


def load_field_track_document(
    direction: str = 'clockwise',
    config_path: Optional[str] = None,
) -> dict:
    path = resolve_config_path(direction, config_path)
    cache_key = path
    if cache_key in _CACHE:
        return _CACHE[cache_key]
    if yaml is None:
        raise RuntimeError('PyYAML required for field_track config')
    with open(path, 'r', encoding='utf-8') as handle:
        doc = yaml.safe_load(handle)
    if not isinstance(doc, dict):
        raise ValueError(f'field_track config must be a mapping: {path}')
    _CACHE[cache_key] = doc
    return doc


def field_move_segment_specs(
    direction: str = 'clockwise',
    config_path: Optional[str] = None,
) -> Dict[str, dict]:
    """Per move segment: start_xy, end_xy, heading_rad, length_m (along ψ)."""
    doc = load_field_track_document(direction, config_path)
    raw = doc.get('segments') or {}
    out: Dict[str, dict] = {}
    for name in MOVE_SEGMENT_ORDER:
        entry = raw.get(name)
        if not isinstance(entry, dict):
            continue
        sx, sy = entry['start']
        ex, ey = entry['end']
        start_xy = (float(sx), float(sy))
        end_xy = (float(ex), float(ey))
        heading_rad = math.radians(float(entry['heading_deg']))
        length_m = _along_length(start_xy, end_xy, heading_rad)
        out[name] = {
            'start_xy': start_xy,
            'end_xy': end_xy,
            'heading_rad': heading_rad,
            'length_m': length_m,
        }
    return out


def field_segment_endpoints(
    direction: str = 'clockwise',
    config_path: Optional[str] = None,
) -> Dict[str, Tuple[Point, Point]]:
    specs = field_move_segment_specs(direction, config_path)
    return {name: (spec['start_xy'], spec['end_xy']) for name, spec in specs.items()}


def field_mission_finish_xy(
    direction: str = 'clockwise',
    config_path: Optional[str] = None,
) -> Point:
    doc = load_field_track_document(direction, config_path)
    finish_name = str(doc.get('mission_finish_segment', 'rect_return_origin'))
    specs = field_move_segment_specs(direction, config_path)
    if finish_name not in specs:
        finish_name = 'rect_return_origin'
    return specs[finish_name]['end_xy']


def field_channel_entry_xy(
    direction: str = 'clockwise',
    config_path: Optional[str] = None,
) -> Point:
    doc = load_field_track_document(direction, config_path)
    entry = doc.get('channel_entry') or {}
    xy = entry.get('xy', [2.50, 3.20])
    return (float(xy[0]), float(xy[1]))


def field_channel_entry_yaw_rad(
    direction: str = 'clockwise',
    config_path: Optional[str] = None,
) -> float:
    doc = load_field_track_document(direction, config_path)
    entry = doc.get('channel_entry') or {}
    return math.radians(float(entry.get('yaw_deg', 90.0)))


def _normalize_angle_rad(angle_rad: float) -> float:
    angle = float(angle_rad)
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def field_entry_turn_deg(
    direction: str = 'clockwise',
    config_path: Optional[str] = None,
) -> float:
    """rect_enter_align：Stage1 末 ψ=90°；顺时针 +90° → 180°，逆时针 −90° → 0°。"""
    doc = load_field_track_document(direction, config_path)
    if doc.get('entry_turn_deg') is not None:
        return float(doc['entry_turn_deg'])
    return 90.0 if str(direction) == 'clockwise' else -90.0


def field_corner_turn_deg(
    direction: str = 'clockwise',
    config_path: Optional[str] = None,
) -> float:
    """环上四角：顺时针 −90°（右转），逆时针 +90°（左转）。"""
    doc = load_field_track_document(direction, config_path)
    if doc.get('corner_turn_deg') is not None:
        return float(doc['corner_turn_deg'])
    return -90.0 if str(direction) == 'clockwise' else 90.0


def build_ring_move_distances(
    direction: str = 'clockwise',
    config_path: Optional[str] = None,
) -> Dict[str, float]:
    specs = field_move_segment_specs(direction, config_path)
    return {name: float(spec['length_m']) for name, spec in specs.items()}


def field_ring_polyline(
    direction: str = 'clockwise',
    config_path: Optional[str] = None,
) -> List[Point]:
    """折线：channel_entry → 各段 S→E（离线绘图）。"""
    points: List[Point] = [field_channel_entry_xy(direction, config_path)]
    endpoints = field_segment_endpoints(direction, config_path)
    for name in MOVE_SEGMENT_ORDER:
        if name not in endpoints:
            continue
        start_xy, end_xy = endpoints[name]
        if not points or math.hypot(points[-1][0] - start_xy[0], points[-1][1] - start_xy[1]) > 0.02:
            points.append(start_xy)
        points.append(end_xy)
    return points

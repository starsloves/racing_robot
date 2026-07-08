"""Return path waypoints in map frame. 保留 YAML 加载功能供备用配置使用。"""

from __future__ import annotations

import json
import os
from typing import Any, Dict, List, Optional

import yaml
from ament_index_python.packages import get_package_share_directory


def _package_config_path(filename: str) -> str:
    share = get_package_share_directory('racing_stage3_param_test')
    return os.path.join(share, 'config', filename)


def default_config_path() -> str:
    return ''


def resolve_config_path(config_path: Optional[str] = None) -> str:
    explicit = str(config_path or '').strip()
    if explicit:
        return os.path.abspath(explicit)
    return ''


def load_return_track_doc(config_path: str) -> Dict[str, Any]:
    with open(config_path, 'r', encoding='utf-8') as handle:
        doc = yaml.safe_load(handle) or {}
    if not isinstance(doc, dict):
        raise ValueError(f'return track config must be a mapping: {config_path}')
    return doc


def return_waypoints_from_doc(doc: Dict[str, Any], default_speed: float) -> List[Dict[str, Any]]:
    waypoints = []
    for index, item in enumerate(doc.get('waypoints') or []):
        if not isinstance(item, dict):
            continue
        xy = item.get('xy') or [item.get('x'), item.get('y')]
        if not isinstance(xy, (list, tuple)) or len(xy) < 2:
            continue
        yaw_deg = item.get('yaw_deg')
        waypoints.append({
            'x': float(xy[0]),
            'y': float(xy[1]),
            'speed': float(item.get('speed', default_speed)),
            'yaw_deg': None if yaw_deg is None else float(yaw_deg),
            'description': str(item.get('description', f'return_wp_{index}')),
        })
    return waypoints


def return_waypoints_json_from_doc(doc: Dict[str, Any], default_speed: float) -> str:
    return json.dumps(return_waypoints_from_doc(doc, default_speed), ensure_ascii=False)


def mission_start_xy(doc: Dict[str, Any]) -> Optional[tuple]:
    xy = doc.get('mission_start_xy')
    if isinstance(xy, (list, tuple)) and len(xy) >= 2:
        return float(xy[0]), float(xy[1])
    return None


def goal_xy(doc: Dict[str, Any]) -> Optional[tuple]:
    xy = doc.get('goal_xy')
    if isinstance(xy, (list, tuple)) and len(xy) >= 2:
        return float(xy[0]), float(xy[1])
    return None
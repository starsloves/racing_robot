"""Segment-local geometry in odom (P0 + segment heading ψ)."""

from __future__ import annotations

import math
from typing import Tuple

Point = Tuple[float, float]


def unit_vectors(heading_rad: float) -> Tuple[float, float, float, float]:
    """Return (cos ψ, sin ψ, left_normal_x, left_normal_y)."""
    cos_h = math.cos(float(heading_rad))
    sin_h = math.sin(float(heading_rad))
    return cos_h, sin_h, -sin_h, cos_h


def along_m(
    world_xy: Point,
    origin_xy: Point,
    heading_rad: float,
) -> float:
    dx = float(world_xy[0]) - float(origin_xy[0])
    dy = float(world_xy[1]) - float(origin_xy[1])
    cos_h, sin_h, _, _ = unit_vectors(heading_rad)
    return dx * cos_h + dy * sin_h


def lateral_m(
    world_xy: Point,
    origin_xy: Point,
    heading_rad: float,
) -> float:
    """Signed lateral offset; left of segment direction is positive."""
    dx = float(world_xy[0]) - float(origin_xy[0])
    dy = float(world_xy[1]) - float(origin_xy[1])
    _, _, nx, ny = unit_vectors(heading_rad)
    return dx * nx + dy * ny


def point_on_segment(
    origin_xy: Point,
    heading_rad: float,
    along_m_value: float,
    lateral_m_value: float = 0.0,
) -> Point:
    cos_h, sin_h, nx, ny = unit_vectors(heading_rad)
    ox, oy = float(origin_xy[0]), float(origin_xy[1])
    px = ox + cos_h * float(along_m_value) + nx * float(lateral_m_value)
    py = oy + sin_h * float(along_m_value) + ny * float(lateral_m_value)
    return px, py


def clamp_along(along_m_value: float, segment_length_m: float) -> float:
    length = max(0.0, float(segment_length_m))
    return max(0.0, min(float(along_m_value), length))


def lateral_pd_omega(lateral_m: float, heading_rad: float, gain: float) -> float:
    """ω 项：把有符号横偏拉回弦线。ψ≈180° 时校正方向与 ψ≈0° 相反。"""
    c = math.cos(float(heading_rad))
    if c > 0.5:
        sign = -1.0
    elif c < -0.5:
        sign = 1.0
    else:
        sign = -1.0
    return float(gain) * sign * (-float(lateral_m))

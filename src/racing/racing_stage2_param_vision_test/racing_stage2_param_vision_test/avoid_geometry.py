"""avoid_geometry.py — 避障几何：航向偏移与两脚路点。"""

import math
from dataclasses import dataclass


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def cross_segment_m(origin_xy, heading_rad, position_xy) -> float:
    dx = position_xy[0] - origin_xy[0]
    dy = position_xy[1] - origin_xy[1]
    return -dx * math.sin(heading_rad) + dy * math.cos(heading_rad)


@dataclass(frozen=True)
class AvoidPlan:
    psi0: float
    psi1: float
    psi2: float
    psi3: float            # 回正目标航向（默认=psi0，可独立调偏补偿打滑）
    leg1_distance_m: float
    leg2_distance_m: float


def obstacle_is_left(danger_angle_deg: float) -> bool:
    return float(danger_angle_deg) > 0.0


def build_avoid_plan(
    psi0_rad: float,
    leg1_distance_m: float,
    leg2_distance_m: float,
    offset_away_rad: float,
    offset_back_rad: float,
    offset_recover_rad: float,
    obstacle_left: bool,
) -> AvoidPlan:
    psi0 = normalize_angle(float(psi0_rad))
    sign = -1.0 if obstacle_left else 1.0
    psi1 = normalize_angle(psi0 + sign * offset_away_rad)
    psi2 = normalize_angle(psi0 - sign * offset_back_rad)
    psi3 = normalize_angle(psi2 + sign * offset_recover_rad)
    return AvoidPlan(
        psi0=psi0,
        psi1=psi1,
        psi2=psi2,
        psi3=psi3,
        leg1_distance_m=float(leg1_distance_m),
        leg2_distance_m=float(leg2_distance_m),
    )

"""S1 几何：仅 ψ₀ 与两脚航向、脚长（无世界路点）。"""

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
class S1Plan:
    psi0: float
    psi1: float
    psi2: float
    leg1_distance_m: float
    leg2_distance_m: float


def obstacle_is_left(danger_angle_deg: float) -> bool:
    return float(danger_angle_deg) > 0.0


def build_s1_plan(
    psi0_rad: float,
    leg1_distance_m: float,
    leg2_distance_m: float,
    offset_rad: float,
    obstacle_left: bool,
) -> S1Plan:
    psi0 = normalize_angle(float(psi0_rad))
    delta = -offset_rad if obstacle_left else offset_rad
    return S1Plan(
        psi0=psi0,
        psi1=normalize_angle(psi0 + delta),
        psi2=normalize_angle(psi0 - delta),
        leg1_distance_m=float(leg1_distance_m),
        leg2_distance_m=float(leg2_distance_m),
    )

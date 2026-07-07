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
    psi3: float            # 回正目标航向（corner_mode 时 = psi2）
    leg1_distance_m: float # 直行段长度（corner_mode 时为斜边）
    leg2_distance_m: float # 正常模式第二段；corner_mode 时为 0
    corner_mode: bool = False  # True = 转角避障（单腿直行）


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
    corner_mode: bool = False,
) -> AvoidPlan:
    """构建避障路径规划。
    
    角度计算逻辑（所有角度都是增量，不是绝对值）：
      sign = -1.0 if obstacle_left else 1.0
      
      ψ₁ = ψ₀ + sign × away   (从原始航向偏开)
      ψ₂ = ψ₁ - sign × back   (从偏开位置往回转)
      ψ₃ = ψ₂ + sign × recover (从回转位置回正)
    
    示例(障碍在右, ψ₀=90°, away=40°, back=50°, recover=15°):
      sign = +1.0
      ψ₁ = 90 + 40 = 130° (左偏40°)
      ψ₂ = 130 - 50 = 80° (右转50°)
      ψ₃ = 80 + 15 = 95°   (左转15°回正)
    """
    psi0 = normalize_angle(float(psi0_rad))
    sign = -1.0 if obstacle_left else 1.0
    psi1 = normalize_angle(psi0 + sign * offset_away_rad)

    if corner_mode:
        # 转角避障：从 psi1 同方向继续转 back，使 ψ₂ 指向下个段方向而非回 ψ₀
        psi2 = normalize_angle(psi1 + sign * offset_back_rad)
        psi3 = psi2  # 不回正到 psi0
    else:
        # 正常避障：从 psi1 往回转 back_deg，然后回正
        psi2 = normalize_angle(psi1 - sign * offset_back_rad)
        psi3 = normalize_angle(psi2 + sign * offset_recover_rad)

    return AvoidPlan(
        psi0=psi0,
        psi1=psi1,
        psi2=psi2,
        psi3=psi3,
        leg1_distance_m=float(leg1_distance_m),
        leg2_distance_m=0.0 if corner_mode else float(leg2_distance_m),
        corner_mode=corner_mode,
    )

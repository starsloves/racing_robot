"""World-plan navigation for stage2 param test (pose + waypoint targets).

Single odom world frame:
  - Channel entry: IMU raw 不可信 → 名义 nav=90° + offset（无 Stage1 时必需）。
  - 转弯 / 直行 / 完成：统一用 nav_yaw = raw − offset。
  - 每段 move 的 S→E：``segment_endpoints_world``（含 turn 弧线，与 launch 边长一致）。
  - 沿程 / 横偏在物理弦线 S→ψ 上量；段完成 = 走近 E + 本段走够。

See ``docs/NAVIGATION.md``.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from . import world_segment
from .ring_track import (
    build_ring_plan_for_sim,
    nominal_move_heading_rad,
    segment_endpoints_world,
    simulate_plan_world_poses,
)

Point = Tuple[float, float]


class DirectInertialTesterWorldPlanMixin:
    """Fixed world waypoint frame for move segments."""

    SEGMENT_END_REACH_M = 0.12

    def reset_segment_world_plan(self):
        self.segment_plan_start_xy: Optional[Point] = None
        self.segment_plan_end_xy: Optional[Point] = None
        self.segment_plan_heading_rad: Optional[float] = None
        self.segment_plan_length_m: float = 0.0
        self.segment_along_baseline_m: float = 0.0

    def segment_move_along_tolerance_m(self) -> float:
        length = max(1e-6, float(self.segment_plan_length_m))
        return min(float(self.distance_tolerance), max(0.015, length * 0.03))

    def navigation_heading_from_physical_rad(self, psi_rad: float) -> float:
        """物理弦线 ψ → 导航航向（nav 系，与 turn 段 start+angle 一致）。"""
        if getattr(self, 'assume_channel_entry_yaw', True):
            offset = float(getattr(self, 'world_yaw_offset_rad', 0.0))
            return self.normalize_angle(float(psi_rad) - offset)
        return float(psi_rad)

    def _world_plan_origin_xy(self) -> Point:
        origin = getattr(self, 'ring_origin_world', None)
        if origin is not None:
            return (float(origin[0]), float(origin[1]))
        return (0.0, 0.0)

    def lookup_move_segment_world_plan(self, segment_name: str):
        """Nominal sim (避障预览 / 绘图); 任务进度用 ``world_ring_segment_endpoints``。"""
        name = str(segment_name or '').strip()
        if not name:
            return None
        geo = self._ring_track_geometry_kwargs()
        plan = build_ring_plan_for_sim(
            geo['direction'],
            geo['first_leg_m'],
            geo['side_leg_m'],
            geo['top_leg_m'],
        )
        sim = simulate_plan_world_poses(
            plan,
            origin_xy=geo.get('origin_xy', (0.0, 0.0)),
            origin_yaw=geo.get('origin_yaw', 0.0),
        )
        move_segments = sim.get('move_segments') or {}
        if name not in move_segments:
            return None
        start_xy, end_xy, heading = move_segments[name]
        length = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
        return {
            'start_xy': (float(start_xy[0]), float(start_xy[1])),
            'end_xy': (float(end_xy[0]), float(end_xy[1])),
            'heading_rad': float(heading),
            'length_m': max(1e-6, float(length)),
        }

    def _capture_segment_world_plan_frame(self):
        return (
            self.segment_plan_start_xy,
            self.segment_plan_end_xy,
            self.segment_plan_heading_rad,
            self.segment_plan_length_m,
            getattr(self, 'segment_heading', None),
            getattr(self, 'segment_start_pose', None),
        )

    def _restore_segment_world_plan_frame(self, saved):
        if saved is None:
            return
        (
            self.segment_plan_start_xy,
            self.segment_plan_end_xy,
            self.segment_plan_heading_rad,
            self.segment_plan_length_m,
            seg_heading,
            seg_start,
        ) = saved
        if seg_heading is not None:
            self.segment_heading = seg_heading
        if seg_start is not None:
            self.segment_start_pose = seg_start

    def apply_move_segment_world_plan(self, segment_name: str) -> bool:
        """Nominal sim anchor (避障 pre-corner 预览)."""
        spec = self.lookup_move_segment_world_plan(segment_name)
        if spec is None:
            return False
        geo = self._ring_track_geometry_kwargs()
        all_eps = self._segment_endpoints_for_ring(geo)
        if segment_name not in all_eps:
            sx, sy = spec['start_xy']
            psi = float(spec['heading_rad'])
            length = float(spec['length_m'])
            ex = sx + math.cos(psi) * length
            ey = sy + math.sin(psi) * length
        else:
            (sx, sy), (ex, ey) = all_eps[segment_name]
            psi = math.atan2(float(ey) - float(sy), float(ex) - float(sx))
            length = math.hypot(float(ex) - float(sx), float(ey) - float(sy))
        self.segment_plan_start_xy = (float(sx), float(sy))
        self.segment_plan_end_xy = (float(ex), float(ey))
        self.segment_plan_heading_rad = float(psi)
        self.segment_plan_length_m = max(1e-6, float(length))
        self.segment_heading = self.navigation_heading_from_physical_rad(psi)
        self.segment_start_pose = self.segment_plan_start_xy
        return True

    def establish_world_waypoint_move_plan(self) -> bool:
        """固定世界 S→E：sim 链（含 turn 弧线），入口 ring_origin + 进通道 raw yaw。"""
        segment = self.current_segment or {}
        name = str(segment.get('description', '')).strip()
        if not name:
            return False
        geo = self._ring_track_geometry_kwargs()
        all_eps = self._segment_endpoints_for_ring(geo)
        if name not in all_eps:
            return False
        (sx, sy), (ex, ey) = all_eps[name]
        length = math.hypot(float(ex) - float(sx), float(ey) - float(sy))
        param_len = float(segment.get('distance_m', length))
        if param_len > 1e-6 and abs(length - param_len) > 0.05:
            length = param_len
        psi = math.atan2(float(ey) - float(sy), float(ex) - float(sx))
        self.segment_plan_start_xy = (float(sx), float(sy))
        self.segment_plan_end_xy = (float(ex), float(ey))
        self.segment_plan_heading_rad = float(psi)
        self.segment_plan_length_m = max(1e-6, float(length))
        self.segment_heading = self.navigation_heading_from_physical_rad(psi)
        self.segment_start_pose = (float(sx), float(sy))
        if self.current_position is not None:
            entry_along = self.progress_along_segment_m(self.current_position)
            self.segment_along_baseline_m = max(0.0, float(entry_along or 0.0))
        else:
            self.segment_along_baseline_m = 0.0
        return True

    def snap_move_plan_start_to_current_pose(self) -> bool:
        """Turn 结束后把 plan S 对齐当前 odom，消除弧线终点与名义 S 的横偏。"""
        if self.current_position is None or not self._segment_plan_frame_ready():
            return False
        end_xy = self.segment_plan_end_xy
        psi = float(self.segment_plan_heading_rad)
        if end_xy is None or self.segment_plan_start_xy is None:
            return False
        cx, cy = float(self.current_position[0]), float(self.current_position[1])
        ex, ey = float(end_xy[0]), float(end_xy[1])
        old_s = self.segment_plan_start_xy
        old_lat = world_segment.lateral_m((cx, cy), old_s, psi)
        along = world_segment.along_m((cx, cy), old_s, psi)
        if abs(old_lat) < 0.06 and -0.05 <= along <= float(self.segment_plan_length_m) + 0.05:
            return False
        self.segment_plan_start_xy = (cx, cy)
        self.segment_start_pose = (cx, cy)
        dx, dy = ex - cx, ey - cy
        along_remain = dx * math.cos(psi) + dy * math.sin(psi)
        if along_remain > 0.05:
            self.segment_plan_length_m = max(1e-6, float(along_remain))
        self.segment_along_baseline_m = 0.0
        if hasattr(self, 'write_debug_log'):
            self.write_debug_log(
                'MOVE',
                (
                    f'reanchor 弯后 S→odom ({cx:.2f},{cy:.2f}) '
                    f'原横偏={old_lat:+.2f}m L={self.segment_plan_length_m:.2f}m'
                ),
            )
        return True

    def load_move_segment_world_plan(self):
        self.reset_segment_world_plan()
        segment = self.current_segment or {}
        if segment.get('type') != 'move':
            return
        if not self.establish_world_waypoint_move_plan():
            return
        prev_turn = (
            int(getattr(self, 'plan_index', 0)) > 0
            and hasattr(self, 'plan')
            and self.plan[int(self.plan_index) - 1].get('type') == 'turn'
        )
        if prev_turn:
            self.snap_move_plan_start_to_current_pose()
        if hasattr(self, 'write_debug_log'):
            psi_deg = math.degrees(float(self.segment_plan_heading_rad or 0.0))
            s = self.segment_plan_start_xy
            e = self.segment_plan_end_xy
            nav_psi_deg = math.degrees(float(self.segment_heading or 0.0))
            self.write_debug_log(
                'MOVE',
                (
                    f'SIM_CHAIN {segment.get("description", "?")} '
                    f'S=({s[0]:.3f},{s[1]:.3f}) E=({e[0]:.3f},{e[1]:.3f}) '
                    f'ψ_phys={psi_deg:.1f}deg ψ_nav={nav_psi_deg:.1f}deg '
                    f'L={self.segment_plan_length_m:.2f}m'
                ),
            )

    def world_navigation_yaw_raw(self) -> Optional[float]:
        imu_yaw = getattr(self, 'current_yaw', None)
        if imu_yaw is not None:
            return float(imu_yaw)
        odom_yaw = getattr(self, 'current_odom_yaw', None)
        if odom_yaw is not None:
            return float(odom_yaw)
        return None

    def world_navigation_yaw(self) -> Optional[float]:
        """导航航向 nav = raw − offset；入口名义 90°（无 Stage1 时 anchor）。"""
        raw_yaw = self.world_navigation_yaw_raw()
        if raw_yaw is None:
            return None
        if getattr(self, 'assume_channel_entry_yaw', True):
            offset = float(getattr(self, 'world_yaw_offset_rad', 0.0))
            return self.normalize_angle(float(raw_yaw) - offset)
        return float(raw_yaw)

    def world_plan_heading_after_turn(self, turn_index: int) -> Optional[float]:
        if not hasattr(self, 'plan'):
            return None
        next_index = int(turn_index) + 1
        if next_index >= len(self.plan):
            return None
        next_seg = self.plan[next_index]
        if next_seg.get('type') != 'move':
            return None
        geo = self._ring_track_geometry_kwargs()
        all_eps = self._segment_endpoints_for_ring(geo)
        name = str(next_seg.get('description', '')).strip()
        if name not in all_eps:
            return None
        (sx, sy), (ex, ey) = all_eps[name]
        return math.atan2(float(ey) - float(sy), float(ex) - float(sx))

    def _segment_endpoints_for_ring(self, geo: dict):
        return segment_endpoints_world(
            geo['direction'],
            geo['first_leg_m'],
            geo['side_leg_m'],
            geo['top_leg_m'],
            origin_xy=geo['origin_xy'],
            origin_yaw=geo['origin_yaw'],
            turn_linear_mps=geo.get('turn_linear_mps', 0.08),
            turn_angular_rps=geo.get('turn_angular_rps', 0.65),
        )

    def _segment_plan_frame_ready(self) -> bool:
        return (
            self.segment_plan_start_xy is not None
            and self.segment_plan_heading_rad is not None
            and self.segment_plan_length_m > 0.0
        )

    def segment_plan_origin_xy(self) -> Optional[Point]:
        return self.segment_plan_start_xy

    def segment_plan_heading(self) -> Optional[float]:
        return self.segment_plan_heading_rad

    def segment_lateral_offset_m(self):
        if self.current_position is None or not self._segment_plan_frame_ready():
            return 0.0
        return world_segment.lateral_m(
            self.current_position,
            self.segment_plan_start_xy,
            self.segment_plan_heading_rad,
        )

    def progress_along_segment_m(self, world_xy: Point) -> Optional[float]:
        if not self._segment_plan_frame_ready():
            return None
        return world_segment.along_m(
            world_xy,
            self.segment_plan_start_xy,
            self.segment_plan_heading_rad,
        )

    def projected_distance(self):
        if self.current_position is None or not self._segment_plan_frame_ready():
            return 0.0
        along = self.progress_along_segment_m(self.current_position)
        if along is None:
            return 0.0
        return world_segment.clamp_along(along, self.segment_plan_length_m)

    def segment_travel_progress_m(self) -> float:
        """本段实际前进量：沿程投影减去进段 baseline，防越界 E 后误报到位。"""
        if self.current_position is None or not self._segment_plan_frame_ready():
            return 0.0
        along = self.progress_along_segment_m(self.current_position)
        if along is None:
            return 0.0
        baseline = float(getattr(self, 'segment_along_baseline_m', 0.0))
        traveled = max(0.0, float(along) - baseline)
        return world_segment.clamp_along(traveled, self.segment_plan_length_m)

    def segment_progress_to_world(self, along_m, lateral_m=0.0) -> Optional[Point]:
        if not self._segment_plan_frame_ready():
            return None
        return world_segment.point_on_segment(
            self.segment_plan_start_xy,
            self.segment_plan_heading_rad,
            float(along_m),
            float(lateral_m),
        )

    def distance_to_segment_plan_end_m(self) -> float:
        if self.current_position is None or self.segment_plan_end_xy is None:
            return float('inf')
        ex, ey = self.segment_plan_end_xy
        return math.hypot(self.current_position[0] - ex, self.current_position[1] - ey)

    def distance_to_navigation_target_m(self) -> float:
        target = self.navigation_target_xy()
        if target is None or self.current_position is None:
            return float('inf')
        return math.hypot(target[0] - self.current_position[0], target[1] - self.current_position[1])

    def navigation_target_xy(self) -> Optional[Point]:
        if getattr(self, 'avoidance_active', False):
            goal = self.active_avoidance_goal_xy()
            if goal is not None:
                return (float(goal[0]), float(goal[1]))
        if (self.current_segment or {}).get('type') == 'move':
            return self.segment_plan_end_xy
        return None

    def segment_move_complete_on_plan(
        self,
        along_tol: Optional[float] = None,
        lat_tol: float = 0.10,
        end_reach_m: Optional[float] = None,
    ) -> bool:
        if not self._segment_plan_frame_ready():
            return False
        tol = float(along_tol if along_tol is not None else self.segment_move_along_tolerance_m())
        reach = float(
            end_reach_m if end_reach_m is not None else self.SEGMENT_END_REACH_M
        )
        along = self.segment_travel_progress_m()
        length = self.segment_plan_length_m
        end_dist = self.distance_to_segment_plan_end_m()
        end_ok = end_dist <= reach
        along_ok = along + tol >= length
        integrated = float(getattr(self, 'segment_integrated_distance_m', 0.0))
        if along_ok:
            lat = abs(self.segment_lateral_offset_m())
            if lat > lat_tol and end_dist > reach:
                return False
            return True
        if end_ok and integrated + tol >= length:
            return True
        if end_ok and along + tol >= length * 0.70:
            return True
        return False

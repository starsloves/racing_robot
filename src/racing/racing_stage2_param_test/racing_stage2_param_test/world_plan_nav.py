"""World-plan navigation for stage2 param test (pose + waypoint targets).

All straight segments use fixed nominal S→E from ``ring_track`` (odom frame).
Progress and lateral error are projections onto that plan chord, clamped along
length — not re-anchored per-segment odometry entry.

See ``docs/NAVIGATION.md``.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from . import world_segment
from .ring_track import build_ring_plan_for_sim, simulate_plan_world_poses

Point = Tuple[float, float]


class DirectInertialTesterWorldPlanMixin:
    """Nominal world polyline frame for move segments."""

    SEGMENT_END_REACH_M = 0.12

    def reset_segment_world_plan(self):
        self.segment_plan_start_xy: Optional[Point] = None
        self.segment_plan_end_xy: Optional[Point] = None
        self.segment_plan_heading_rad: Optional[float] = None
        self.segment_plan_length_m: float = 0.0

    def load_move_segment_world_plan(self):
        """Cache nominal S, E, ψ for the active move segment."""
        self.reset_segment_world_plan()
        segment = self.current_segment or {}
        if segment.get('type') != 'move':
            return
        name = str(segment.get('description', ''))
        if not name:
            return
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
            return
        start_xy, end_xy, heading = move_segments[name]
        length = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
        self.segment_plan_start_xy = (float(start_xy[0]), float(start_xy[1]))
        self.segment_plan_end_xy = (float(end_xy[0]), float(end_xy[1]))
        self.segment_plan_heading_rad = float(heading)
        self.segment_plan_length_m = max(1e-6, float(length))
        self.segment_heading = float(heading)
        self.segment_start_pose = self.segment_plan_start_xy

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

    # ------------------------------------------------------------------ geometry (nominal plan)
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
        along = world_segment.along_m(
            self.current_position,
            self.segment_plan_start_xy,
            self.segment_plan_heading_rad,
        )
        return world_segment.clamp_along(along, self.segment_plan_length_m)

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
        """Current go-to point: avoidance waypoint, else segment end E."""
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
        tol = float(along_tol if along_tol is not None else self.distance_tolerance)
        reach = float(
            end_reach_m if end_reach_m is not None else self.SEGMENT_END_REACH_M
        )
        along = self.projected_distance()
        length = self.segment_plan_length_m
        lat = abs(self.segment_lateral_offset_m())
        end_ok = self.distance_to_segment_plan_end_m() <= reach
        along_ok = along + tol >= length
        if not along_ok and not end_ok:
            return False
        if lat > lat_tol and not end_ok:
            return False
        return True

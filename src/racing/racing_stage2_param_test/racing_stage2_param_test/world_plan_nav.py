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
from .ring_track import move_segment_world_spec

Point = Tuple[float, float]


class DirectInertialTesterWorldPlanMixin:
    """Nominal world polyline frame for move segments."""

    SEGMENT_END_REACH_M = 0.12

    def reset_segment_world_plan(self):
        self.segment_plan_start_xy: Optional[Point] = None
        self.segment_plan_end_xy: Optional[Point] = None
        self.segment_plan_heading_rad: Optional[float] = None
        self.segment_plan_length_m: float = 0.0

    def lookup_move_segment_world_plan(self, segment_name: str):
        """Nominal S, E, ψ, length from field_track YAML."""
        name = str(segment_name or '').strip()
        if not name:
            return None
        geo = self._ring_track_geometry_kwargs()
        spec = move_segment_world_spec(
            name,
            geo['direction'],
            geo.get('config_path'),
        )
        if spec is None:
            return None
        return {
            'start_xy': spec['start_xy'],
            'end_xy': spec['end_xy'],
            'heading_rad': float(spec['heading_rad']),
            'length_m': float(spec['length_m']),
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
        """Load nominal plan frame for ``segment_name`` (current mission segment unchanged)."""
        spec = self.lookup_move_segment_world_plan(segment_name)
        if spec is None:
            return False
        self.segment_plan_start_xy = spec['start_xy']
        self.segment_plan_end_xy = spec['end_xy']
        self.segment_plan_heading_rad = spec['heading_rad']
        self.segment_plan_length_m = spec['length_m']
        self.segment_heading = spec['heading_rad']
        self.segment_start_pose = spec['start_xy']
        return True

    def load_move_segment_world_plan(self):
        """Cache nominal S, E, ψ for the active move segment."""
        self.reset_segment_world_plan()
        segment = self.current_segment or {}
        if segment.get('type') != 'move':
            return
        name = str(segment.get('description', ''))
        if not name:
            return
        if not self.apply_move_segment_world_plan(name):
            return

    def world_navigation_yaw_raw(self) -> Optional[float]:
        """Raw yaw from odom (preferred) or IMU, before channel-entry alignment."""
        odom_yaw = getattr(self, 'current_odom_yaw', None)
        if odom_yaw is not None:
            return float(odom_yaw)
        imu_yaw = getattr(self, 'current_yaw', None)
        if imu_yaw is not None:
            return float(imu_yaw)
        return None

    def world_navigation_yaw(self) -> Optional[float]:
        """Yaw aligned to nominal world plan (same frame as S→E)."""
        raw_yaw = self.world_navigation_yaw_raw()
        if raw_yaw is None:
            return None
        if getattr(self, 'assume_channel_entry_yaw', True):
            offset = float(getattr(self, 'world_yaw_offset_rad', 0.0))
            return self.normalize_angle(float(raw_yaw) - offset)
        return float(raw_yaw)

    def _entry_odom_to_map_rotation_rad(self) -> float:
        """Rotate odom Δ into map: entry odom +X ≈ map +Y when raw0≈0."""
        raw0 = getattr(self, 'world_yaw_entry_raw', None)
        if raw0 is None:
            return 0.0
        return self.normalize_angle(
            self.nominal_channel_entry_yaw_rad() - float(raw0)
        )

    def _odom_delta_to_map_delta(self, dx: float, dy: float) -> Point:
        rot = self._entry_odom_to_map_rotation_rad()
        cos_r = math.cos(rot)
        sin_r = math.sin(rot)
        return (
            cos_r * float(dx) - sin_r * float(dy),
            sin_r * float(dx) + cos_r * float(dy),
        )

    def world_navigation_xy(self) -> Optional[Point]:
        """Map XY: R(entry_rot) · (odom − anchor) + corridor_goal."""
        if self.current_position is None:
            return None
        ox, oy = float(self.current_position[0]), float(self.current_position[1])
        anchor = getattr(self, 'world_pos_anchor_odom_xy', None)
        if anchor is None:
            return ox, oy
        nx, ny = self.nominal_channel_entry_xy()
        mdx, mdy = self._odom_delta_to_map_delta(
            ox - float(anchor[0]),
            oy - float(anchor[1]),
        )
        return nx + mdx, ny + mdy

    def world_plan_heading_after_turn(self, turn_index: int) -> Optional[float]:
        """Absolute world ψ for the move leg following a turn segment."""
        if not hasattr(self, 'plan'):
            return None
        next_index = int(turn_index) + 1
        if next_index >= len(self.plan):
            return None
        next_seg = self.plan[next_index]
        if next_seg.get('type') != 'move':
            return None
        name = str(next_seg.get('description', ''))
        spec = self.lookup_move_segment_world_plan(name)
        if spec is None:
            return None
        return float(spec['heading_rad'])

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
        world_xy = self.world_navigation_xy()
        if world_xy is None or not self._segment_plan_frame_ready():
            return 0.0
        return world_segment.lateral_m(
            world_xy,
            self.segment_plan_start_xy,
            self.segment_plan_heading_rad,
        )

    def segment_lateral_pd_omega(self, lateral_m: float, gain: float) -> float:
        heading = self.segment_plan_heading_rad
        if heading is None:
            return float(gain) * (-float(lateral_m))
        return world_segment.lateral_pd_omega(lateral_m, heading, gain)

    def progress_along_segment_m(self, world_xy: Point) -> Optional[float]:
        if not self._segment_plan_frame_ready():
            return None
        return world_segment.along_m(
            world_xy,
            self.segment_plan_start_xy,
            self.segment_plan_heading_rad,
        )

    def projected_distance(self):
        world_xy = self.world_navigation_xy()
        if world_xy is None or not self._segment_plan_frame_ready():
            return 0.0
        along = world_segment.along_m(
            world_xy,
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
        world_xy = self.world_navigation_xy()
        if world_xy is None or self.segment_plan_end_xy is None:
            return float('inf')
        ex, ey = self.segment_plan_end_xy
        return math.hypot(world_xy[0] - ex, world_xy[1] - ey)

    def distance_to_navigation_target_m(self) -> float:
        target = self.navigation_target_xy()
        world_xy = self.world_navigation_xy()
        if target is None or world_xy is None:
            return float('inf')
        return math.hypot(target[0] - world_xy[0], target[1] - world_xy[1])

    def navigation_target_xy(self) -> Optional[Point]:
        """Current go-to point: avoidance waypoint, else segment end E."""
        if getattr(self, 'avoidance_active', False):
            goal = self.active_avoidance_goal_xy()
            if goal is not None:
                return (float(goal[0]), float(goal[1]))
        if (self.current_segment or {}).get('type') == 'move':
            return self.segment_plan_end_xy
        return None

    def next_move_segment_start_xy(self) -> Optional[Point]:
        """下一段 move 的 yaml 起点 S（转弯时作为 map 目标位置）。"""
        if not hasattr(self, 'plan'):
            return None
        for i in range(int(getattr(self, 'plan_index', 0)) + 1, len(self.plan)):
            seg = self.plan[i]
            if seg.get('type') != 'move':
                continue
            spec = self.lookup_move_segment_world_plan(str(seg.get('description', '')))
            if spec is not None:
                return spec['start_xy']
        return None

    def navigation_target_yaw_plan(self) -> Optional[float]:
        """map 系目标航向：move=段 ψ，turn=转完后的 plan_yaw。"""
        seg = self.current_segment or {}
        if seg.get('type') == 'turn':
            tgt = getattr(self, 'segment_target_yaw', None)
            return float(tgt) if tgt is not None else None
        if seg.get('type') == 'move':
            h = self.segment_heading
            return float(h) if h is not None else None
        return None

    def nav_core_map_state(self) -> dict:
        """全程 map 系：当前点/目标点/当前角/目标角（用户读日志用）。"""
        cur_xy = self.world_navigation_xy()
        cur_yaw = self.world_navigation_yaw()
        seg = self.current_segment or {}
        st = seg.get('type')

        tgt_xy = self.navigation_target_xy()
        if tgt_xy is None and st == 'turn':
            tgt_xy = self.next_move_segment_start_xy()
        tgt_yaw = self.navigation_target_yaw_plan()

        dist_m = float('inf')
        if cur_xy is not None and tgt_xy is not None:
            dist_m = math.hypot(
                float(tgt_xy[0]) - float(cur_xy[0]),
                float(tgt_xy[1]) - float(cur_xy[1]),
            )
        ang_err_deg = float('nan')
        if cur_yaw is not None and tgt_yaw is not None:
            ang_err_deg = math.degrees(self.angle_error(tgt_yaw, cur_yaw))

        return {
            'cur_xy': cur_xy,
            'tgt_xy': tgt_xy,
            'cur_yaw_rad': cur_yaw,
            'tgt_yaw_rad': tgt_yaw,
            'dist_m': dist_m,
            'ang_err_deg': ang_err_deg,
        }

    def format_nav_core_line(self) -> str:
        """一行四要素：当前点、目标点、当前角、目标角（均为 map 世界坐标）。"""
        s = self.nav_core_map_state()
        cur_xy = s['cur_xy']
        tgt_xy = s['tgt_xy']
        cur_yaw = s['cur_yaw_rad']
        tgt_yaw = s['tgt_yaw_rad']
        cur_p = (
            f'({cur_xy[0]:.2f},{cur_xy[1]:.2f})'
            if cur_xy is not None
            else 'nan'
        )
        tgt_p = (
            f'({tgt_xy[0]:.2f},{tgt_xy[1]:.2f})'
            if tgt_xy is not None
            else 'nan'
        )
        cur_a = (
            f'{self.format_yaw_deg(cur_yaw)}°'
            if cur_yaw is not None
            else 'nan'
        )
        tgt_a = (
            f'{self.format_yaw_deg(tgt_yaw)}°'
            if tgt_yaw is not None
            else 'nan'
        )
        dist_m = s['dist_m']
        dist_text = f'{dist_m:.2f}m' if math.isfinite(dist_m) else 'nan'
        ang_err = s['ang_err_deg']
        ang_text = f'{ang_err:+.1f}°' if math.isfinite(ang_err) else 'nan'
        return (
            f'当前点={cur_p} 目标点={tgt_p} '
            f'当前角={cur_a} 目标角={tgt_a} '
            f'距目标点={dist_text} 角差={ang_text}'
        )

    def segment_move_complete_on_plan(
        self,
        along_tol: Optional[float] = None,
        lat_tol: float = 0.10,
        end_reach_m: Optional[float] = None,
    ) -> bool:
        if not self._segment_plan_frame_ready():
            return False
        reach = float(
            end_reach_m if end_reach_m is not None else self.SEGMENT_END_REACH_M
        )
        dist_e = self.distance_to_segment_plan_end_m()
        if not math.isfinite(dist_e) or dist_e > reach:
            return False
        lat = abs(self.segment_lateral_offset_m())
        if lat > lat_tol:
            return False
        return True

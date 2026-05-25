"""Segment coordinate frames for ring test (single semantic layer).

Three layers — do not mix without explicit conversion:

1. **World** — odom ``(x, y, yaw)``. Hardware truth from ``/odom_combined``.
2. **Chord** — ``(along_m, lateral_m)`` on the measured driven segment line
   (``DRIVEN_CW_SEGMENT_ENDPOINTS``). Use for goals, clearance, handoff, corner.
3. **Plan progress** — chord ``along_m`` scaled to mission ``distance_m``, minus
   per-segment entry anchor (set after turn / corner handoff). Use for
   ``projected_distance()`` and segment completion.

World goals are derived from chord via ``point_on_driven_segment`` only at the
control boundary (go-to-point), not via ad-hoc normal offsets on vertical legs.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from .ring_track import (
    driven_segment_endpoints,
    driven_segment_length_m,
    point_on_driven_segment,
    progress_on_driven_segment_m,
    signed_cross_track_on_driven_segment,
)

Point = Tuple[float, float]


@dataclass(frozen=True)
class ChordPose:
    """Pose on the driven chord of one move segment."""

    segment_name: str
    along_m: float
    lateral_m: float
    driven_length_m: float

    @property
    def at_segment_end(self) -> bool:
        return self.along_m >= self.driven_length_m - 1e-3

    def plan_progress_m(self, plan_distance_m: float, entry_along_m: float = 0.0) -> float:
        """Map chord along → mission plan ``distance_m`` frame (minus entry anchor)."""
        rel = max(0.0, float(self.along_m) - float(entry_along_m))
        driven_len = max(1e-6, float(self.driven_length_m))
        plan_len = max(1e-6, float(plan_distance_m))
        return rel * (plan_len / driven_len)


def chord_pose_at_world(
    world_xy: Point,
    segment_name: str,
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> Optional[ChordPose]:
    """Project world XY onto the driven chord → canonical task position."""
    along = progress_on_driven_segment_m(
        world_xy,
        segment_name,
        direction,
        first_leg_m,
        side_leg_m,
        top_leg_m,
    )
    lateral = signed_cross_track_on_driven_segment(
        world_xy,
        segment_name,
        direction,
        first_leg_m,
        side_leg_m,
        top_leg_m,
    )
    driven_len = driven_segment_length_m(
        segment_name,
        direction,
        first_leg_m,
        side_leg_m,
        top_leg_m,
    )
    if along is None or lateral is None or driven_len is None:
        return None
    return ChordPose(
        segment_name=segment_name,
        along_m=float(along),
        lateral_m=float(lateral),
        driven_length_m=float(driven_len),
    )


def world_from_chord(
    along_m: float,
    lateral_m: float,
    segment_name: str,
    direction: str = 'clockwise',
    first_leg_m: float = 1.10,
    side_leg_m: float = 0.50,
    top_leg_m: float = 2.80,
) -> Optional[Point]:
    """Chord (along, lateral) → world XY (same sign as signed_cross_track)."""
    return point_on_driven_segment(
        segment_name,
        along_m,
        lateral_m,
        direction,
        first_leg_m,
        side_leg_m,
        top_leg_m,
    )


"""Stage 2 segment executor.

There is deliberately no global closest-point search in this module.  The
active segment alone owns its start distance, reference heading and terminal
condition, so an observation from one side of the rectangle cannot advance a
different side or corner.
"""

from dataclasses import dataclass, replace
import math
from typing import Dict, List, Optional, Tuple


def wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class TrackPoint:
    x: float
    y: float
    yaw: float
    curvature: float
    speed: float


@dataclass(frozen=True)
class TrackCommand:
    linear: float
    angular: float
    state: str
    progress_m: float
    cross_track_m: float
    heading_error_rad: float
    target_speed: float
    complete: bool = False
    safe_stop: bool = False
    reason: str = ''
    segment: str = ''
    segment_progress_m: float = 0.0
    segment_target_m: float = 0.0
    turn_progress_rad: float = 0.0
    turn_target_rad: float = 0.0
    entry_boundary_trigger: str = ''
    entry_boundary_window_min_m: float = 0.0
    entry_boundary_window_max_m: float = 0.0
    entry_boundary_top_y_ratio: float = 0.0
    entry_boundary_angle_deg: float = 90.0
    entry_boundary_confirm_frames: int = 0
    line_heading_angular: float = 0.0
    vision_angular: float = 0.0
    yaw_rate_damping_angular: float = 0.0
    arc_reference_yaw_rad: float = 0.0
    arc_final_heading_error_rad: float = 0.0
    arc_base_angular: float = 0.0
    arc_damping_angular: float = 0.0
    arc_cutoff_active: bool = False
    arc_completion_reason: str = ''


class ImuDistancePose:
    """Integrate `/odom_combined` scalar travel in the IMU yaw frame."""

    def __init__(self, max_step_m: float = 0.12):
        self.max_step_m = max(0.02, max_step_m)
        self.pose: Optional[Tuple[float, float]] = None
        self.total_distance_m = 0.0
        self._last_position: Optional[Tuple[float, float]] = None
        self._last_yaw: Optional[float] = None

    def reset(self, odom_position: Tuple[float, float], yaw: float) -> None:
        self.pose = (0.0, 0.0)
        self.total_distance_m = 0.0
        self._last_position = odom_position
        self._last_yaw = yaw

    def update(self, odom_position: Tuple[float, float], yaw: float) -> Tuple[float, float]:
        if self.pose is None or self._last_position is None or self._last_yaw is None:
            self.reset(odom_position, yaw)
            return self.pose
        step = math.hypot(odom_position[0] - self._last_position[0],
                          odom_position[1] - self._last_position[1])
        mid_yaw = wrap_angle(self._last_yaw + 0.5 * wrap_angle(yaw - self._last_yaw))
        self._last_position = odom_position
        self._last_yaw = yaw
        # A delayed EKF reset must not inject a false segment completion.
        if step > self.max_step_m:
            return self.pose
        self.pose = (self.pose[0] + step * math.cos(mid_yaw),
                     self.pose[1] + step * math.sin(mid_yaw))
        self.total_distance_m += step
        return self.pose


class RoundedRectangleTrack:
    """Compatibility-only sampled geometry for offline inspection tools."""

    def __init__(self, start: Tuple[float, float], heading: float,
                 clockwise: bool, corner_radius: float = 0.18,
                 sample_step: float = 0.03):
        self.points: List[TrackPoint] = [TrackPoint(start[0], start[1], heading, 0.0, 0.30)]
        self._build(start, heading, clockwise, corner_radius, sample_step)
        self.arc_lengths = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.arc_lengths.append(self.arc_lengths[-1] + math.hypot(b.x - a.x, b.y - a.y))
        self.total_length = self.arc_lengths[-1]

    def _build(self, start, heading, clockwise, radius, step):
        spans = (1.10, 0.80, 2.59, 0.80, 1.49)
        sign = -1.0 if clockwise else 1.0
        x, y, yaw = start[0], start[1], heading
        for i, span in enumerate(spans):
            tangent = max(0.0, span - radius * (int(i > 0) + int(i < len(spans) - 1)))
            for n in range(1, max(1, math.ceil(max(tangent, step) / step)) + 1):
                d = min(tangent, n * step)
                self.points.append(TrackPoint(x + d * math.cos(yaw), y + d * math.sin(yaw), yaw, 0.0, 0.30))
            x += tangent * math.cos(yaw)
            y += tangent * math.sin(yaw)
            if i == len(spans) - 1:
                break
            count = max(2, math.ceil(math.pi * radius / (2.0 * step)))
            cx = x - sign * radius * math.sin(yaw)
            cy = y + sign * radius * math.cos(yaw)
            initial = yaw - sign * math.pi / 2.0
            for n in range(1, count + 1):
                a = initial + sign * math.pi * n / (2.0 * count)
                self.points.append(TrackPoint(cx + radius * math.cos(a), cy + radius * math.sin(a),
                                              wrap_angle(yaw + sign * math.pi * n / (2.0 * count)),
                                              sign / radius, 0.18))
            x, y = self.points[-1].x, self.points[-1].y
            yaw = self.points[-1].yaw


@dataclass(frozen=True)
class _SegmentSpec:
    name: str
    kind: str
    target_m: float
    speed_mps: float
    turn_rad: float = 0.0


@dataclass
class _ActiveSegment:
    spec: _SegmentSpec
    start_distance: float
    start_position: Tuple[float, float]
    start_heading: float
    last_yaw: Optional[float] = None
    arc_entry_yaw: Optional[float] = None
    turn_progress_rad: float = 0.0


class Stage2TrackController:
    """One-segment-at-a-time distance/IMU trajectory executor."""

    ENTRY = 'ENTRY_ARC'
    ALIGN = 'ENTRY_ALIGN'
    TRACK = 'TRACK'
    COMPLETE = 'COMPLETE'
    SAFE_STOP = 'SAFE_STOP'

    def __init__(self, *, max_speed: float = 0.34,
                 corner_speed: float = 0.18,
                 max_lateral_accel: float = 0.55,
                 stanley_heading_kp: float = 1.8,
                 line_heading_kp: float = 0.80,
                 stanley_cross_kp: float = 2.2,
                 curvature_kp: float = 1.0,
                 max_angular: float = 0.75,
                 entry_angular: float = 0.75,
                 corner_angular: float = 0.75,
                 entry_linear: float = 0.18,
                 entry_min_linear: float = 0.04,
                 entry_medium_distance_m: float = 0.85,
                 entry_boundary_trigger_enabled: bool = False,
                 entry_boundary_guard_half_width_m: float = 0.15,
                 entry_boundary_top_y_ratio: float = 0.18,
                 entry_boundary_max_angle_deg: float = 20.0,
                 entry_boundary_confirm_frames: int = 3,
                 top_boundary_trigger_enabled: bool = False,
                 top_boundary_guard_half_width_m: float = 0.15,
                 top_boundary_top_y_ratio: float = 0.18,
                 top_boundary_max_angle_deg: float = 20.0,
                 top_boundary_confirm_frames: int = 3,
                 side_arc_vision_enabled: bool = True,
                 side_arc_vision_trigger_lead_m: float = 0.002,
                 side_arc_vision_trigger_speed_mps: float = 0.45,
                 turn_force_map_x_enabled: bool = False,
                 turn_force_min_map_x: float = 0.50,
                 turn_force_max_map_x: float = 4.00,
                 top_long_distance_m: float = 2.59,
                 exit_medium_distance_m: float = 1.49,
                 entry_heading_kp: float = 1.6,
                 yaw_rate_damping: float = 0.45,
                 entry_yaw_rate_tolerance: float = 0.10,
                 entry_align_max_distance_m: float = 0.45,
                 entry_align_error_tolerance: float = 0.08,
                 entry_align_hold_sec: float = 0.20,
                 entry_align_visual_kp: float = 0.55,
                 entry_arc_exit_lead_deg: float = 20.0,
                 left_side_arc_exit_lead_deg: float = 20.0,
                 right_side_arc_exit_lead_deg: float = 20.0,
                 exit_turn_90_exit_lead_deg: float = 20.0,
                 entry_arc_linear: Optional[float] = None,
                 entry_arc_angular: Optional[float] = None,
                 left_side_arc_linear: Optional[float] = None,
                 left_side_arc_angular: Optional[float] = None,
                 right_side_arc_linear: Optional[float] = None,
                 right_side_arc_angular: Optional[float] = None,
                 exit_turn_90_linear: Optional[float] = None,
                 exit_turn_90_angular: Optional[float] = None,
                 direction_arc_profiles: Optional[Dict[str, Dict[str, Dict[str, float]]]] = None,
                 direction_track_profiles: Optional[Dict[str, Dict[str, float]]] = None,
                 vision_lateral_scale_m: float = 0.30,
                 vision_lateral_weight: float = 0.35,
                 vision_correction_max_angular: float = 0.10,
                 vision_lateral_deadband: float = 0.06,
                 vision_lateral_release_deadband: float = 0.035,
                 vision_heading_gain: float = 0.22,
                 vision_confirm_frames: int = 3,
                 vision_max_age_sec: float = 0.60,
                 vision_max_frame_delta: float = 0.25,
                 vision_opposition_threshold: float = 0.08,
                 vision_camera_offset: float = 0.0,
                 vision_max_angular_step: float = 0.12,
                 lookahead_m: float = 0.45,
                 heading_slowdown_deg: float = 10.0,
                 finish_tolerance_m: float = 0.10):
        del max_lateral_accel, curvature_kp, entry_min_linear, lookahead_m, finish_tolerance_m
        self.max_speed = max(0.01, max_speed)
        self.corner_speed = max(0.02, corner_speed)
        self.heading_kp = max(0.1, stanley_heading_kp)
        self.line_heading_kp = max(0.1, line_heading_kp)
        # Kept in the constructor for launch compatibility. Inertial
        # cross-track is not a valid lateral measurement and is never used
        # for steering; only a validated visual offset may affect a line.
        self.cross_kp = max(0.0, stanley_cross_kp)
        self.max_angular = max(0.1, max_angular)
        def arc_speed(value: Optional[float], fallback: float) -> float:
            return max(0.01, fallback if value is None else value)

        def arc_angular(value: Optional[float], fallback: float) -> float:
            return min(self.max_angular, max(0.1, fallback if value is None else value))

        self.entry_speed = arc_speed(entry_arc_linear, entry_linear)
        self.entry_angular = arc_angular(entry_arc_angular, entry_angular)
        self.corner_speed = arc_speed(left_side_arc_linear, corner_speed)
        self.corner_angular = arc_angular(left_side_arc_angular, corner_angular)
        base_arc_cruise = {
            'entry_arc': (self.entry_speed, self.entry_angular),
            'left_side_arc': (self.corner_speed, self.corner_angular),
            'right_side_arc': (
                arc_speed(right_side_arc_linear, corner_speed),
                arc_angular(right_side_arc_angular, corner_angular),
            ),
            'exit_turn_90': (
                arc_speed(exit_turn_90_linear, entry_linear),
                arc_angular(exit_turn_90_angular, entry_angular),
            ),
        }
        base_arc_cutoff_leads = {
            'entry_arc': self._arc_cutoff_lead(entry_arc_exit_lead_deg),
            'left_side_arc': self._arc_cutoff_lead(left_side_arc_exit_lead_deg),
            'right_side_arc': self._arc_cutoff_lead(right_side_arc_exit_lead_deg),
            'exit_turn_90': self._arc_cutoff_lead(exit_turn_90_exit_lead_deg),
        }
        self._base_arc_cruise = dict(base_arc_cruise)
        self._base_arc_cutoff_leads = dict(base_arc_cutoff_leads)
        self._direction_arc_profiles = {}
        for direction, overrides in (direction_arc_profiles or {}).items():
            cruise = dict(base_arc_cruise)
            leads = dict(base_arc_cutoff_leads)
            for segment, values in (overrides or {}).items():
                if segment not in cruise:
                    continue
                current_linear, current_angular = cruise[segment]
                linear = values.get('linear')
                angular = values.get('angular')
                if linear is not None:
                    current_linear = arc_speed(linear, current_linear)
                if angular is not None:
                    current_angular = arc_angular(angular, current_angular)
                cruise[segment] = (current_linear, current_angular)
                lead = values.get('exit_lead_deg')
                if lead is not None:
                    leads[segment] = self._arc_cutoff_lead(lead)
            self._direction_arc_profiles[str(direction).lower()] = (cruise, leads)
        self._arc_cruise = dict(base_arc_cruise)
        self._arc_cutoff_leads = dict(base_arc_cutoff_leads)
        self.entry_medium_distance = max(0.05, entry_medium_distance_m)
        self.entry_boundary_trigger_enabled = bool(entry_boundary_trigger_enabled)
        self.entry_boundary_guard_half_width = max(0.0, entry_boundary_guard_half_width_m)
        self.entry_boundary_top_y_ratio = max(0.0, min(1.0, entry_boundary_top_y_ratio))
        self.entry_boundary_max_angle_deg = max(0.0, min(90.0, entry_boundary_max_angle_deg))
        self.entry_boundary_confirm_target = max(1, int(entry_boundary_confirm_frames))
        self.top_boundary_trigger_enabled = bool(top_boundary_trigger_enabled)
        self.top_boundary_guard_half_width = max(0.0, top_boundary_guard_half_width_m)
        self.top_boundary_top_y_ratio = max(0.0, min(1.0, top_boundary_top_y_ratio))
        self.top_boundary_max_angle_deg = max(0.0, min(90.0, top_boundary_max_angle_deg))
        self.top_boundary_confirm_target = max(1, int(top_boundary_confirm_frames))
        self.side_arc_vision_enabled = bool(side_arc_vision_enabled)
        self.side_arc_vision_trigger_lead = max(0.0, side_arc_vision_trigger_lead_m)
        self.side_arc_vision_trigger_speed = max(0.01, side_arc_vision_trigger_speed_mps)
        self.turn_force_map_x_enabled = bool(turn_force_map_x_enabled)
        self.turn_force_min_map_x = float(turn_force_min_map_x)
        self.turn_force_max_map_x = float(turn_force_max_map_x)
        if self.turn_force_min_map_x > self.turn_force_max_map_x:
            self.turn_force_min_map_x, self.turn_force_max_map_x = (
                self.turn_force_max_map_x, self.turn_force_min_map_x
            )
        self.top_long_distance = max(0.20, top_long_distance_m)
        self.exit_medium_distance = max(0.05, exit_medium_distance_m)
        self._base_turn_force_min_map_x = self.turn_force_min_map_x
        self._base_turn_force_max_map_x = self.turn_force_max_map_x
        self._base_exit_medium_distance = self.exit_medium_distance
        self._direction_track_profiles = {}
        for direction, overrides in (direction_track_profiles or {}).items():
            min_x = self._base_turn_force_min_map_x
            max_x = self._base_turn_force_max_map_x
            exit_distance = self._base_exit_medium_distance
            if overrides.get('turn_force_min_map_x') is not None:
                min_x = float(overrides['turn_force_min_map_x'])
            if overrides.get('turn_force_max_map_x') is not None:
                max_x = float(overrides['turn_force_max_map_x'])
            if min_x > max_x:
                min_x, max_x = max_x, min_x
            if overrides.get('exit_medium_distance_m') is not None:
                exit_distance = max(0.05, float(overrides['exit_medium_distance_m']))
            self._direction_track_profiles[str(direction).lower()] = {
                'turn_force_min_map_x': min_x,
                'turn_force_max_map_x': max_x,
                'exit_medium_distance_m': exit_distance,
            }
        # Each corner has only three trajectory inputs: its linear speed,
        # angular speed, and remaining-angle cutoff.  At the cutoff the
        # steering command goes directly to zero; residual chassis yaw carries
        # the vehicle to the IMU turn target without low-rate steering or a
        # counter-steer correction.
        self.heading_slowdown = math.radians(max(1.0, heading_slowdown_deg))
        self.entry_heading_kp = max(0.1, entry_heading_kp)
        self.yaw_rate_damping = max(0.0, yaw_rate_damping)
        self.yaw_rate_settle = max(0.01, entry_yaw_rate_tolerance)
        self.entry_align_max_distance = max(0.05, entry_align_max_distance_m)
        self.entry_align_error_tolerance = max(0.01, entry_align_error_tolerance)
        self.entry_align_hold_sec = max(0.0, entry_align_hold_sec)
        self.entry_align_visual_kp = max(0.05, entry_align_visual_kp)
        self.vision_lateral_scale_m = max(0.0, vision_lateral_scale_m)
        self.vision_lateral_weight = max(0.0, min(1.0, vision_lateral_weight))
        self.vision_correction_max_angular = min(
            self.max_angular, max(0.01, vision_correction_max_angular)
        )
        self.vision_lateral_deadband = max(0.0, min(0.50, vision_lateral_deadband))
        self.vision_lateral_release_deadband = max(
            0.0, min(self.vision_lateral_deadband, vision_lateral_release_deadband)
        )
        self.vision_heading_gain = max(0.0, vision_heading_gain)
        self.vision_confirm_frames = max(1, int(vision_confirm_frames))
        self.vision_max_age_sec = max(0.05, min(2.0, vision_max_age_sec))
        self.vision_max_frame_delta = max(0.01, min(1.0, vision_max_frame_delta))
        self.vision_opposition_threshold = max(0.0, min(1.0, vision_opposition_threshold))
        self.vision_camera_offset = max(-0.50, min(0.50, vision_camera_offset))
        self.vision_max_angular_step = max(0.01, min(self.max_angular, vision_max_angular_step))
        self.distance_tolerance = 0.025
        self.max_arc_overrun_m = 0.16
        self.max_line_overrun_m = 0.12
        self.line_heading_tolerance = math.radians(6.0)
        self.state = self.SAFE_STOP
        self.safe_reason = 'not_started'
        self._specs: List[_SegmentSpec] = []
        self._index = 0
        self._active: Optional[_ActiveSegment] = None
        self._entry_heading = 0.0
        self._entry_align_settled_at: Optional[float] = None
        self._entry_align_best_heading: Optional[float] = None
        self._entry_align_best_error = float('inf')
        self._fallback_distance = 0.0
        self._fallback_position: Optional[Tuple[float, float]] = None
        self._entry_boundary_trigger = ''
        self._entry_boundary_confirmed_frames = 0
        self._entry_boundary_top_y = 0.0
        self._entry_boundary_angle_deg = 90.0
        self._clockwise = True
        self._vision_last_timestamp = 0.0
        self._vision_confirmed_frames = 0
        self._vision_last_lateral = None
        self._vision_lateral_active = False
        self._vision_last_angular = 0.0

    def _build_specs(self, clockwise: bool) -> List[_SegmentSpec]:
        sign = -1.0 if clockwise else 1.0
        # Only the two medium lines and the top line are straight commands.
        # Each nominal 0.80 m side is one continuous 180 degree bridge arc,
        # never two 90 degree arcs with a zero-length line between them.
        return [
            _SegmentSpec('entry_medium', 'LINE', self.entry_medium_distance, self.max_speed),
            _SegmentSpec('left_side_arc', 'ARC', self._arc_target_m('left_side_arc', math.pi),
                         self._arc_cruise['left_side_arc'][0], sign * math.pi),
            _SegmentSpec('top_long', 'LINE', self.top_long_distance, self.max_speed),
            _SegmentSpec('right_side_arc', 'ARC', self._arc_target_m('right_side_arc', math.pi),
                         self._arc_cruise['right_side_arc'][0], sign * math.pi),
            _SegmentSpec('exit_medium', 'LINE', self.exit_medium_distance, self.max_speed),
            # Leave the rectangle toward -Y before handing control to Stage3.
            # This final 90-degree arc has its own speed/angular configuration.
            _SegmentSpec('exit_turn_90', 'ARC', self._arc_target_m('exit_turn_90', math.pi / 2.0),
                         self._arc_cruise['exit_turn_90'][0], -sign * math.pi / 2.0),
            _SegmentSpec('stage3_handoff_line', 'LINE', float('inf'), self.max_speed),
        ]

    @staticmethod
    def _arc_cutoff_lead(lead_deg: float) -> float:
        return math.radians(max(0.0, min(179.0, lead_deg)))

    def _arc_target_m(self, segment: str, turn_rad: float) -> float:
        linear, angular = self._arc_cruise[segment]
        return abs(turn_rad) * linear / angular

    def _apply_direction_arc_profile(self, direction: str) -> None:
        profile = self._direction_arc_profiles.get(str(direction).lower())
        if profile is None:
            self._arc_cruise = dict(self._base_arc_cruise)
            self._arc_cutoff_leads = dict(self._base_arc_cutoff_leads)
            return
        cruise, leads = profile
        self._arc_cruise = dict(cruise)
        self._arc_cutoff_leads = dict(leads)

    def _apply_direction_track_profile(self, direction: str) -> None:
        profile = self._direction_track_profiles.get(str(direction).lower())
        if profile is None:
            self.turn_force_min_map_x = self._base_turn_force_min_map_x
            self.turn_force_max_map_x = self._base_turn_force_max_map_x
            self.exit_medium_distance = self._base_exit_medium_distance
            return
        self.turn_force_min_map_x = profile['turn_force_min_map_x']
        self.turn_force_max_map_x = profile['turn_force_max_map_x']
        self.exit_medium_distance = profile['exit_medium_distance_m']

    def arc_profile_summary(self, direction: str) -> str:
        profile = self._direction_arc_profiles.get(str(direction).lower())
        cruise, leads = profile if profile is not None else (
            self._base_arc_cruise,
            self._base_arc_cutoff_leads,
        )
        parts = []
        for segment in ('entry_arc', 'left_side_arc', 'right_side_arc', 'exit_turn_90'):
            linear, angular = cruise[segment]
            parts.append(
                f'{segment}:v={linear:.2f},w={angular:.2f},'
                f'lead={math.degrees(leads[segment]):.1f}deg'
            )
        return '; '.join(parts)

    def track_profile_summary(self, direction: str) -> str:
        profile = self._direction_track_profiles.get(str(direction).lower())
        if profile is None:
            min_x = self._base_turn_force_min_map_x
            max_x = self._base_turn_force_max_map_x
            exit_distance = self._base_exit_medium_distance
        else:
            min_x = profile['turn_force_min_map_x']
            max_x = profile['turn_force_max_map_x']
            exit_distance = profile['exit_medium_distance_m']
        return (
            f'turn_force_min_x={min_x:.3f},'
            f'turn_force_max_x={max_x:.3f},'
            f'exit_medium={exit_distance:.3f}m'
        )

    def start(self, direction: str, position: Tuple[float, float], yaw: float,
              now: float, distance_m: float = 0.0) -> None:
        del now
        clockwise = str(direction).lower().startswith('clock')
        self._clockwise = clockwise
        normalized_direction = 'clockwise' if clockwise else 'counterclockwise'
        self._apply_direction_arc_profile(normalized_direction)
        self._apply_direction_track_profile(normalized_direction)
        entry_sign = 1.0 if clockwise else -1.0
        self._entry_heading = wrap_angle(yaw + entry_sign * math.pi / 2.0)
        self._specs = [_SegmentSpec('entry_arc', 'ARC', self._arc_target_m('entry_arc', math.pi / 2.0),
                                    self._arc_cruise['entry_arc'][0], entry_sign * math.pi / 2.0)]
        self._specs.extend(self._build_specs(clockwise))
        self._index = 0
        self._active = _ActiveSegment(self._specs[0], distance_m, position, yaw)
        self._fallback_distance = distance_m
        self._fallback_position = position
        self._entry_align_settled_at = None
        self._entry_align_best_heading = None
        self._entry_align_best_error = float('inf')
        self._entry_boundary_trigger = ''
        self._entry_boundary_confirmed_frames = 0
        self._entry_boundary_top_y = 0.0
        self._entry_boundary_angle_deg = 90.0
        self._reset_line_vision_state()
        self.state = self.ENTRY
        self.safe_reason = ''

    def _distance(self, position: Tuple[float, float], supplied: Optional[float]) -> float:
        if supplied is not None:
            self._fallback_distance = float(supplied)
            self._fallback_position = position
            return self._fallback_distance
        if self._fallback_position is not None:
            self._fallback_distance += math.hypot(position[0] - self._fallback_position[0],
                                                  position[1] - self._fallback_position[1])
        self._fallback_position = position
        return self._fallback_distance

    def _clamp_angular(self, value: float, entry: bool = False) -> float:
        limit = self.entry_angular if entry else self.max_angular
        return max(-limit, min(limit, value))

    def _activate_next(self, position: Tuple[float, float], distance: float,
                       heading_override: Optional[float] = None) -> bool:
        assert self._active is not None
        finished = self._active.spec
        self._index += 1
        if self._index >= len(self._specs):
            self.state = self.COMPLETE
            self._active = None
            return False
        next_spec = self._specs[self._index]
        if heading_override is not None:
            heading = wrap_angle(heading_override)
        elif finished.name == 'entry_arc':
            heading = self._entry_heading
            self.state = self.ALIGN if next_spec.kind == 'ALIGN' else self.TRACK
        elif finished.kind == 'ALIGN':
            heading = self._entry_heading if heading_override is None else heading_override
            self.state = self.TRACK
        elif finished.kind == 'ARC':
            heading = wrap_angle(self._active.start_heading + finished.turn_rad)
        else:
            heading = self._active.start_heading
        self._active = _ActiveSegment(next_spec, distance, position, heading)
        if next_spec.kind == 'LINE':
            self._reset_line_vision_state()
        return True

    def _reset_line_vision_state(self) -> None:
        self._vision_last_timestamp = 0.0
        self._vision_confirmed_frames = 0
        self._vision_last_lateral = None
        self._vision_lateral_active = False
        self._vision_last_angular = 0.0

    def _visual_line_angular(self, visual) -> float:
        """Return a robust visual assist for a straight segment.

        Near-field centerline error estimates lateral displacement.  The
        far-minus-near term estimates heading relative to the lane.  A raw
        mixed image-center error cannot distinguish those two effects and can
        pull the vehicle away from a straight after a corner.
        """
        fresh = (
            bool(visual and visual.get('valid', False))
            and float(visual.get('age', 999.0) or 999.0) <= self.vision_max_age_sec
            and float(visual.get('confidence', 0.0) or 0.0) >= 0.35
        )
        if not fresh:
            self._vision_confirmed_frames = 0
            self._vision_last_lateral = None
            self._vision_lateral_active = False
            self._vision_last_angular = 0.0
            return 0.0

        near = float(visual.get('near_error', visual.get('error', 0.0)) or 0.0)
        far = float(visual.get('far_error', visual.get('error', 0.0)) or 0.0)
        lateral = max(-1.0, min(1.0, near - self.vision_camera_offset))
        heading = max(-1.0, min(1.0, far - near))
        opposed = near * far < 0.0 and min(abs(near), abs(far)) >= self.vision_opposition_threshold
        timestamp = float(visual.get('timestamp', 0.0) or 0.0)

        # Count only new inference frames.  A 20 Hz control loop must not turn
        # one SEG frame into three confirmations.
        is_new_frame = timestamp > self._vision_last_timestamp + 1e-6
        if is_new_frame:
            stable = (
                not opposed
                and (self._vision_last_lateral is None
                     or abs(lateral - self._vision_last_lateral) <= self.vision_max_frame_delta)
            )
            self._vision_confirmed_frames = self._vision_confirmed_frames + 1 if stable else 0
            self._vision_last_lateral = lateral if stable else None
            self._vision_last_timestamp = timestamp

        if opposed or self._vision_confirmed_frames < self.vision_confirm_frames:
            self._vision_lateral_active = False
            self._vision_last_angular = 0.0
            return 0.0

        # Hold the result between inference frames.  This makes the angular
        # slew limit a property of the SEG stream, not of the faster control
        # timer that consumes it.
        if not is_new_frame:
            return self._vision_last_angular

        threshold = (
            self.vision_lateral_release_deadband if self._vision_lateral_active
            else self.vision_lateral_deadband
        )
        lateral_term = lateral if abs(lateral) >= threshold else 0.0
        self._vision_lateral_active = abs(lateral_term) > 0.0
        raw = -0.8 * self.vision_lateral_weight * self.vision_lateral_scale_m * lateral_term
        raw -= self.vision_heading_gain * heading
        target = max(-self.vision_correction_max_angular,
                     min(self.vision_correction_max_angular, raw))
        lower = self._vision_last_angular - self.vision_max_angular_step
        upper = self._vision_last_angular + self.vision_max_angular_step
        angular = max(lower, min(upper, target))
        self._vision_last_angular = angular
        return angular

    def _command(self, linear=0.0, angular=0.0, *, active: Optional[_ActiveSegment] = None,
                 distance=0.0, cross=0.0, heading_error=0.0, complete=False,
                 safe_stop=False, reason='', turn=0.0, line_heading_angular=0.0,
                 vision_angular=0.0, yaw_rate_damping_angular=0.0,
                 arc_reference_yaw=0.0, arc_final_heading_error=0.0,
                 arc_base_angular=0.0, arc_damping_angular=0.0,
                 arc_cutoff_active=False, arc_completion_reason=''):
        spec = active.spec if active is not None else None
        progress = max(0.0, distance - active.start_distance) if active is not None else 0.0
        return TrackCommand(linear, angular, self.state, distance, cross, heading_error, linear,
                            complete, safe_stop, reason, spec.name if spec else '', progress,
                            spec.target_m if spec else 0.0, turn, spec.turn_rad if spec else 0.0,
                            self._entry_boundary_trigger,
                            max(0.0, self.entry_medium_distance - self.entry_boundary_guard_half_width),
                            self.entry_medium_distance + self.entry_boundary_guard_half_width,
                            self._entry_boundary_top_y,
                            self._entry_boundary_angle_deg,
                            self._entry_boundary_confirmed_frames,
                            line_heading_angular,
                            vision_angular,
                            yaw_rate_damping_angular,
                            arc_reference_yaw,
                            arc_final_heading_error,
                            arc_base_angular,
                            arc_damping_angular,
                            arc_cutoff_active,
                            arc_completion_reason)

    def _boundary_ready(self, segment: str, progress: float, visual: Optional[dict],
                        map_x: Optional[float]) -> bool:
        """Use the original SEG-in-window, map-x-rebased inertial trigger."""
        is_entry = segment == 'entry_medium'
        nominal_distance = self.entry_medium_distance if is_entry else self.top_long_distance
        guard_half_width = (
            self.entry_boundary_guard_half_width if is_entry
            else self.top_boundary_guard_half_width
        )
        trigger_enabled = (
            self.entry_boundary_trigger_enabled if is_entry
            else self.top_boundary_trigger_enabled
        )
        guard_min = max(0.0, nominal_distance - guard_half_width)
        guard_max = nominal_distance + guard_half_width
        top_y_ratio = (
            self.entry_boundary_top_y_ratio if is_entry
            else self.top_boundary_top_y_ratio
        )
        max_angle_deg = (
            self.entry_boundary_max_angle_deg if is_entry
            else self.top_boundary_max_angle_deg
        )
        confirm_target = (
            self.entry_boundary_confirm_target if is_entry
            else self.top_boundary_confirm_target
        )
        vision_gate_min = max(0.0, nominal_distance - self.side_arc_vision_trigger_lead)
        vision_gate_max = nominal_distance
        self._entry_boundary_top_y = float((visual or {}).get('boundary_top_y_ratio', 0.0) or 0.0)
        boundary_angle = (visual or {}).get('boundary_angle_deg')
        self._entry_boundary_angle_deg = 90.0 if boundary_angle is None else float(boundary_angle)

        force_low_x = (is_entry and self._clockwise) or (not is_entry and not self._clockwise)
        map_force_ready = (
            self.turn_force_map_x_enabled and map_x is not None and (
                map_x <= self.turn_force_min_map_x if force_low_x
                else map_x >= self.turn_force_max_map_x
            )
        )
        if map_force_ready:
            self._entry_boundary_trigger = 'map_x_fallback'
            self._entry_boundary_confirmed_frames = 0
            return True

        if not self.side_arc_vision_enabled:
            self._entry_boundary_confirmed_frames = 0
            if not self.turn_force_map_x_enabled:
                self._entry_boundary_trigger = 'distance_nominal'
                return progress >= nominal_distance - self.distance_tolerance
            if map_x is None and progress >= guard_max - 1e-6:
                self._entry_boundary_trigger = 'distance_fallback_no_map_tf'
                return True
            self._entry_boundary_trigger = 'vision_disabled_wait_map_x'
            return False

        if not trigger_enabled:
            self._entry_boundary_trigger = 'distance_nominal'
            return progress >= nominal_distance - self.distance_tolerance

        if progress < vision_gate_min:
            self._entry_boundary_trigger = 'before_vision_trigger_gate'
            self._entry_boundary_confirmed_frames = 0
            return False
        if progress > vision_gate_max:
            self._entry_boundary_trigger = 'after_vision_trigger_gate'
            self._entry_boundary_confirmed_frames = 0
            if map_x is None and progress >= guard_max - 1e-6:
                self._entry_boundary_trigger = 'distance_fallback_no_map_tf'
                return True
            return False

        visual_ready = (
            bool(visual and visual.get('valid', False))
            and float(visual.get('age', 999.0) or 999.0) <= 0.20
            and float(visual.get('confidence', 0.0) or 0.0) >= 0.35
            and bool(visual.get('boundary_ahead', False))
            and self._entry_boundary_top_y >= top_y_ratio
            and abs(self._entry_boundary_angle_deg) <= max_angle_deg
        )
        if visual_ready:
            self._entry_boundary_confirmed_frames += 1
            self._entry_boundary_trigger = 'vision_candidate'
            if self._entry_boundary_confirmed_frames >= confirm_target:
                self._entry_boundary_trigger = 'vision_confirmed'
                return True
        else:
            self._entry_boundary_confirmed_frames = 0
            self._entry_boundary_trigger = 'vision_rejected'

        if map_x is None and progress >= guard_max - 1e-6:
            self._entry_boundary_trigger = 'distance_fallback_no_map_tf'
            return True
        return False

    def _line_command(self, active, position, yaw, yaw_rate, distance, visual, now, map_x):
        heading_error = wrap_angle(active.start_heading - yaw)
        dx, dy = position[0] - active.start_position[0], position[1] - active.start_position[1]
        inertial_cross = -math.sin(active.start_heading) * dx + math.cos(active.start_heading) * dy
        visual_angular = self._visual_line_angular(visual)
        speed = active.spec.speed_mps
        # The IMU-distance pose is dead reckoning, not an independent lateral
        # observation. Its cross value is logged for diagnosis only.  Vision
        # is the sole lateral correction source once it is fresh/confident.
        heading_angular = self.line_heading_kp * heading_error
        damping_angular = -self.yaw_rate_damping * yaw_rate
        angular = self._clamp_angular(heading_angular + visual_angular + damping_angular)
        progress = distance - active.start_distance
        if active.spec.name in ('entry_medium', 'top_long'):
            is_entry = active.spec.name == 'entry_medium'
            trigger_enabled = (
                self.entry_boundary_trigger_enabled if is_entry
                else self.top_boundary_trigger_enabled
            )
            nominal_distance = self.entry_medium_distance if is_entry else self.top_long_distance
            if (
                self.side_arc_vision_enabled
                and trigger_enabled
                and progress >= max(0.0, nominal_distance - self.side_arc_vision_trigger_lead)
            ):
                speed = min(speed, self.side_arc_vision_trigger_speed)
            ready_for_corner = self._boundary_ready(active.spec.name, progress, visual, map_x)
        else:
            ready_for_corner = progress >= active.spec.target_m - self.distance_tolerance
        if ready_for_corner:
            if self._activate_next(position, distance):
                return self._step_active(position, yaw, yaw_rate, distance, visual, now)
            return self._command(complete=True)
        return self._command(speed, angular, active=active, distance=distance,
                             cross=inertial_cross, heading_error=heading_error,
                             line_heading_angular=heading_angular,
                             vision_angular=visual_angular,
                             yaw_rate_damping_angular=damping_angular)

    def _arc_command(self, active, position, yaw, yaw_rate, distance, visual, now):
        turn_sign = 1.0 if active.spec.turn_rad > 0.0 else -1.0
        if active.last_yaw is None:
            # Each arc is measured from the actual IMU heading at its entry.
            # This makes every 90/180 degree target an explicit full turn,
            # independent of residual heading error from the preceding line.
            active.last_yaw = yaw
            active.arc_entry_yaw = active.last_yaw
        active.turn_progress_rad += turn_sign * wrap_angle(yaw - active.last_yaw)
        active.last_yaw = yaw
        signed_turn = active.turn_progress_rad
        target_turn = abs(active.spec.turn_rad)
        cruise_rate = self._arc_cruise[active.spec.name][1]
        final_yaw = wrap_angle(
            (active.arc_entry_yaw if active.arc_entry_yaw is not None else yaw)
            + turn_sign * target_turn
        )
        final_error = target_turn - signed_turn
        cutoff_active = final_error <= self._arc_cutoff_leads[active.spec.name]
        base_angular = 0.0 if cutoff_active else turn_sign * cruise_rate
        # The cutoff is steering-only.  Each arc's configured linear speed
        # remains continuous until its next segment command takes over.
        linear_speed = active.spec.speed_mps

        # A cutoff is the segment handoff point for every arc.  The following
        # line owns any residual yaw through its existing IMU heading and
        # yaw-rate damping; an arc must never keep driving straight while
        # waiting for passive chassis rotation to reach its nominal angle.
        cutoff_handoff = cutoff_active
        complete_ready = cutoff_handoff
        completion_reason = 'cutoff_handoff'
        if complete_ready:
            if self._activate_next(position, distance):
                next_command = self._step_active(position, yaw, yaw_rate, distance, visual, now)
                return replace(
                    next_command,
                    arc_reference_yaw_rad=final_yaw,
                    arc_final_heading_error_rad=final_error,
                    arc_base_angular=base_angular,
                    arc_damping_angular=0.0,
                    arc_cutoff_active=cutoff_active,
                    arc_completion_reason=completion_reason,
                )
            return self._command(complete=True, arc_completion_reason=completion_reason)
        return self._command(
            linear_speed, base_angular, active=active, distance=distance,
            heading_error=final_error, turn=signed_turn,
            arc_reference_yaw=final_yaw,
            arc_final_heading_error=final_error,
            arc_base_angular=base_angular,
            arc_damping_angular=0.0,
            arc_cutoff_active=cutoff_active,
        )

    def _align_command(self, active, position, yaw, yaw_rate, distance, visual, now):
        progress = distance - active.start_distance
        visual_valid = (
            bool(visual and visual.get('valid', False))
            and float(visual.get('age', 999.0) or 999.0) <= 0.20
            and float(visual.get('confidence', 0.0) or 0.0) >= 0.35
        )
        # Without a trustworthy external line observation, preserve the
        # relative IMU route rather than waiting indefinitely at the entry.
        if not visual_valid:
            self._entry_align_settled_at = None
            if self._activate_next(position, distance, heading_override=self._entry_heading):
                return self._step_active(position, yaw, yaw_rate, distance, visual, now)
            return self._command(complete=True)

        error = float(visual.get('error', 0.0) or 0.0)
        if abs(error) < self._entry_align_best_error:
            self._entry_align_best_error = abs(error)
            self._entry_align_best_heading = yaw
        angular = self._clamp_angular(
            self.entry_align_visual_kp * error - self.yaw_rate_damping * yaw_rate,
            entry=True,
        )
        centered = (abs(error) <= self.entry_align_error_tolerance
                    and abs(yaw_rate) <= self.yaw_rate_settle)
        if centered:
            if self._entry_align_settled_at is None:
                self._entry_align_settled_at = now
            elif now - self._entry_align_settled_at >= self.entry_align_hold_sec:
                # This is the only place Stage2 establishes the heading of
                # its local track. It makes the route robust to Stage1 yaw.
                if self._activate_next(position, distance, heading_override=yaw):
                    return self._step_active(position, yaw, yaw_rate, distance, visual, now)
                return self._command(complete=True)
        else:
            self._entry_align_settled_at = None
        if progress > active.spec.target_m:
            heading = self._entry_align_best_heading
            if heading is None:
                heading = yaw
            if self._activate_next(position, distance, heading_override=heading):
                return self._step_active(position, yaw, yaw_rate, distance, visual, now)
            return self._command(complete=True)
        return self._command(active.spec.speed_mps, angular, active=active,
                             distance=distance, heading_error=error)

    def _step_active(self, position, yaw, yaw_rate, distance, visual, now, map_x=None):
        if self._active is None:
            self.state = self.COMPLETE
            return self._command(complete=True)
        if self._active.spec.kind == 'LINE':
            return self._line_command(self._active, position, yaw, yaw_rate, distance, visual, now, map_x)
        if self._active.spec.kind == 'ALIGN':
            return self._align_command(self._active, position, yaw, yaw_rate, distance, visual, now)
        return self._arc_command(self._active, position, yaw, yaw_rate, distance, visual, now)

    def step(self, now: float, position: Tuple[float, float], yaw: float,
             visual: Optional[dict] = None, yaw_rate: float = 0.0,
             distance_m: Optional[float] = None,
             stage3_handoff_reached: bool = False,
             map_x: Optional[float] = None) -> TrackCommand:
        if self.state == self.SAFE_STOP:
            return self._command(safe_stop=True, reason=self.safe_reason)
        if self.state == self.COMPLETE:
            return self._command(complete=True)
        distance = self._distance(position, distance_m)
        if self.active_segment_name == 'stage3_handoff_line' and stage3_handoff_reached:
            self.state = self.COMPLETE
            self._active = None
            return self._command(complete=True)
        return self._step_active(position, yaw, float(yaw_rate or 0.0), distance, visual, now, map_x)

    def safe_stop(self, reason: str) -> TrackCommand:
        self.state = self.SAFE_STOP
        self.safe_reason = reason
        return self._command(safe_stop=True, reason=reason)

    def hold_active_progress(self, distance_m: float) -> None:
        """Keep the current segment progress fixed while an external override drives."""
        if self._active is None:
            return
        progress = max(0.0, self._fallback_distance - self._active.start_distance)
        self._fallback_distance = float(distance_m)
        self._active.start_distance = self._fallback_distance - progress

    @property
    def active_segment_name(self) -> str:
        """Return the segment currently being evaluated."""
        return self._active.spec.name if self._active is not None else ''

    @property
    def active_segment_progress_m(self) -> float:
        """Return distance progress for the active segment."""
        if self._active is None:
            return 0.0
        return max(0.0, self._fallback_distance - self._active.start_distance)

    @property
    def active_segment_target_m(self) -> float:
        """Return the active segment's nominal target distance."""
        return self._active.spec.target_m if self._active is not None else 0.0

    @property
    def active_segment_heading(self) -> Optional[float]:
        """Return the IMU-referenced heading of the active segment."""
        return self._active.start_heading if self._active is not None else None

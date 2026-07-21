"""Stage 2 segment executor.

There is deliberately no global closest-point search in this module.  The
active segment alone owns its start distance, reference heading and terminal
condition, so an observation from one side of the rectangle cannot advance a
different side or corner.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple


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
                 entry_radius: float = 0.40,
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
                 turn_force_map_x_enabled: bool = False,
                 turn_force_min_map_x: float = 0.50,
                 turn_force_max_map_x: float = 4.00,
                 top_long_distance_m: float = 2.59,
                 exit_medium_distance_m: float = 1.49,
                 entry_tolerance_deg: float = 3.0,
                 settle_sec: float = 0.25,
                 entry_heading_kp: float = 1.6,
                 yaw_rate_damping: float = 0.45,
                 entry_yaw_rate_tolerance: float = 0.10,
                 entry_align_max_distance_m: float = 0.45,
                 entry_align_error_tolerance: float = 0.08,
                 entry_align_hold_sec: float = 0.20,
                 entry_align_visual_kp: float = 0.55,
                 arc_min_completion_ratio: float = 0.70,
                 arc_finish_predict_sec: float = 0.18,
                 arc_mismatch_angle_deg: float = 14.0,
                 entry_stop_prepare_deg: float = 0.0,
                 entry_arc_complete_lead_deg: float = 0.0,
                 exit_turn_arc_complete_lead_deg: float = 0.0,
                 corner_arc_complete_lead_deg: float = 0.0,
                 corner_radius: float = 0.40,
                 vision_lateral_scale_m: float = 0.30,
                 vision_lateral_weight: float = 0.35,
                 vision_correction_max_angular: float = 0.10,
                 vision_lateral_deadband: float = 0.06,
                 vision_lateral_release_deadband: float = 0.035,
                 vision_heading_gain: float = 0.22,
                 vision_confirm_frames: int = 3,
                 vision_max_frame_delta: float = 0.25,
                 vision_opposition_threshold: float = 0.08,
                 vision_camera_offset: float = 0.0,
                 vision_max_angular_step: float = 0.12,
                 lookahead_m: float = 0.45,
                 heading_slowdown_deg: float = 10.0,
                 finish_tolerance_m: float = 0.10):
        del max_lateral_accel, curvature_kp, entry_min_linear, settle_sec, lookahead_m, finish_tolerance_m
        self.max_speed = max(0.01, max_speed)
        self.corner_speed = max(0.02, corner_speed)
        self.heading_kp = max(0.1, stanley_heading_kp)
        self.line_heading_kp = max(0.1, line_heading_kp)
        # Kept in the constructor for launch compatibility. Inertial
        # cross-track is not a valid lateral measurement and is never used
        # for steering; only a validated visual offset may affect a line.
        self.cross_kp = max(0.0, stanley_cross_kp)
        self.max_angular = max(0.1, max_angular)
        self.entry_angular = min(self.max_angular, max(0.1, entry_angular))
        self.corner_angular = min(self.max_angular, max(0.1, corner_angular))
        self.entry_speed = max(0.01, entry_linear)
        self.entry_radius = max(0.05, entry_radius)
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
        self.turn_force_map_x_enabled = bool(turn_force_map_x_enabled)
        self.turn_force_min_map_x = float(turn_force_min_map_x)
        self.turn_force_max_map_x = float(turn_force_max_map_x)
        if self.turn_force_min_map_x > self.turn_force_max_map_x:
            self.turn_force_min_map_x, self.turn_force_max_map_x = (
                self.turn_force_max_map_x, self.turn_force_min_map_x
            )
        self.top_long_distance = max(0.20, top_long_distance_m)
        self.exit_medium_distance = max(0.05, exit_medium_distance_m)
        self.corner_radius = max(0.05, corner_radius)
        self.entry_tolerance = math.radians(max(0.5, entry_tolerance_deg))
        self.angle_tolerance = self.entry_tolerance
        self.heading_slowdown = math.radians(max(1.0, heading_slowdown_deg))
        self.entry_heading_kp = max(0.1, entry_heading_kp)
        self.yaw_rate_damping = max(0.0, yaw_rate_damping)
        self.yaw_rate_settle = max(0.01, entry_yaw_rate_tolerance)
        self.entry_align_max_distance = max(0.05, entry_align_max_distance_m)
        self.entry_align_error_tolerance = max(0.01, entry_align_error_tolerance)
        self.entry_align_hold_sec = max(0.0, entry_align_hold_sec)
        self.entry_align_visual_kp = max(0.05, entry_align_visual_kp)
        self.arc_min_completion_ratio = max(0.35, min(1.0, arc_min_completion_ratio))
        self.arc_finish_predict_sec = max(0.0, min(0.50, arc_finish_predict_sec))
        self.arc_mismatch_angle = math.radians(max(3.0, arc_mismatch_angle_deg))
        self.entry_stop_prepare = math.radians(max(0.0, min(60.0, entry_stop_prepare_deg)))
        self.entry_arc_complete_lead = math.radians(max(0.0, min(60.0, entry_arc_complete_lead_deg)))
        self.exit_turn_arc_complete_lead = math.radians(
            max(0.0, min(60.0, exit_turn_arc_complete_lead_deg))
        )
        self.corner_arc_complete_lead = math.radians(max(0.0, min(90.0, corner_arc_complete_lead_deg)))
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
            _SegmentSpec('left_side_arc', 'ARC', math.pi * self.corner_radius,
                         self.corner_speed, sign * math.pi),
            _SegmentSpec('top_long', 'LINE', self.top_long_distance, self.max_speed),
            _SegmentSpec('right_side_arc', 'ARC', math.pi * self.corner_radius,
                         self.corner_speed, sign * math.pi),
            _SegmentSpec('exit_medium', 'LINE', self.exit_medium_distance, self.max_speed),
            # Leave the rectangle toward -Y before handing control to Stage3.
            # This uses the entry 90 degree speed/angular configuration, while
            # the turn itself always completes the requested 90 degrees.
            _SegmentSpec('exit_turn_90', 'ARC', math.pi * self.entry_radius / 2.0,
                         self.entry_speed, -sign * math.pi / 2.0),
            _SegmentSpec('stage3_handoff_line', 'LINE', float('inf'), self.max_speed),
        ]

    def start(self, direction: str, position: Tuple[float, float], yaw: float,
              now: float, distance_m: float = 0.0) -> None:
        del now
        clockwise = str(direction).lower().startswith('clock')
        self._clockwise = clockwise
        entry_sign = 1.0 if clockwise else -1.0
        self._entry_heading = wrap_angle(yaw + entry_sign * math.pi / 2.0)
        self._specs = [_SegmentSpec('entry_arc', 'ARC', math.pi * self.entry_radius / 2.0,
                                    self.entry_speed, entry_sign * math.pi / 2.0)]
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
        if finished.name == 'entry_arc':
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
            and float(visual.get('age', 999.0) or 999.0) <= 0.20
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
                 vision_angular=0.0, yaw_rate_damping_angular=0.0):
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
                            yaw_rate_damping_angular)

    def _boundary_ready(self, segment: str, progress: float, visual: Optional[dict],
                        map_x: Optional[float]) -> bool:
        """Prefer SEG corner evidence; force the matching physical turn at map x."""
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
        guard_min = max(0.0, nominal_distance - guard_half_width)
        guard_max = nominal_distance + guard_half_width
        self._entry_boundary_top_y = float((visual or {}).get('boundary_top_y_ratio', 0.0) or 0.0)
        boundary_angle = (visual or {}).get('boundary_angle_deg')
        self._entry_boundary_angle_deg = (
            90.0 if boundary_angle is None else float(boundary_angle)
        )

        # Clockwise reaches the low-x end first; counterclockwise reaches the
        # high-x end first. The second 180-degree turn uses the opposite end.
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

        # This global switch selects the turn source for both 180-degree arcs.
        # Keep the map-x switch independent: with TF forcing disabled, retain
        # the previous nominal-distance behavior; with TF unavailable, retain
        # its guarded distance fallback.
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

        if progress < guard_min:
            self._entry_boundary_trigger = 'below_guard_min'
            self._entry_boundary_confirmed_frames = 0
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

        # map x is the configured fallback. Retain a distance-only escape
        # hatch only while map TF is unavailable.
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
            active.last_yaw = active.start_heading
        active.turn_progress_rad += turn_sign * wrap_angle(yaw - active.last_yaw)
        active.last_yaw = yaw
        signed_turn = active.turn_progress_rad
        target_turn = abs(active.spec.turn_rad)
        angular = self._clamp_angular(
            turn_sign * (self.entry_angular if active.spec.name == 'entry_arc' else self.corner_angular),
            entry=active.spec.name == 'entry_arc'
        )
        progress = distance - active.start_distance
        signed_rate = turn_sign * float(yaw_rate or 0.0)
        if active.spec.name == 'entry_arc':
            lead = self.entry_arc_complete_lead
        elif active.spec.name == 'exit_turn_90':
            lead = self.exit_turn_arc_complete_lead
        else:
            lead = self.corner_arc_complete_lead
        angle_ready = signed_turn >= target_turn - max(self.angle_tolerance, lead)
        if angle_ready:
            if self._activate_next(position, distance):
                return self._step_active(position, yaw, yaw_rate, distance, visual, now)
            return self._command(complete=True)
        return self._command(active.spec.speed_mps, angular, active=active, distance=distance,
                             heading_error=target_turn - signed_turn, turn=signed_turn)

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

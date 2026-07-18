"""Deterministic Stage 2 route controller.

Route topology owns the mission.  Segmentation is deliberately limited to
centering a known straight and confirming an early corner in that straight's
end window. IMU constrains turn direction and the maximum safe sweep. SEG
confirms whether the next straight has actually opened, so a fixed yaw target
cannot prematurely finish a corner.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Optional, Tuple


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _angle_error(target: float, current: float) -> float:
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class HybridCommand:
    linear: float
    angular: float
    state: str
    completed: bool = False
    safe_stop: bool = False


@dataclass
class HybridConfig:
    cruise_speed: float = 0.34
    approach_speed: float = 0.24
    turn_speed: float = 0.28
    recover_speed: float = 0.10
    max_angular: float = 0.75
    line_kp: float = 0.72
    curve_kp: float = 0.04
    line_max_angular: float = 0.42
    entry_target_deg: float = 90.0
    ring_target_deg: float = 90.0
    turn_min_deg: float = 25.0
    turn_max_deg: float = 140.0
    turn_kp: float = 1.45
    turn_rate_kd: float = 0.16
    turn_entry_top20_max_fill: float = 0.0
    bridge_turn_angular: float = 0.30
    bridge_long_top20_min_fill: float = 0.25
    bridge_long_max_curve: float = 0.35
    bridge_capture_error: float = 0.28
    bridge_max_deg: float = 145.0
    turn_exit_fill_threshold: float = 0.55
    turn_exit_max_yaw_rate: float = 0.30
    leg_extra_m: float = 0.35
    leg_lengths_csv: str = '1.10,0.80,2.59,0.80,1.49'
    vision_max_age_sec: float = 0.30
    vision_min_confidence: float = 0.35
    vision_min_rows: int = 4
    vision_loss_distance_m: float = 0.25
    required_ring_turns: int = 4

    @classmethod
    def declare_parameters(cls, node) -> None:
        for name, value in vars(cls()).items():
            node.declare_parameter(f'hybrid_{name}', value)

    @classmethod
    def from_node(cls, node) -> 'HybridConfig':
        return cls(**{
            name: node.get_parameter(f'hybrid_{name}').value
            for name in vars(cls()).keys()
        })


class Stage2HybridController:
    ENTRY = 'ENTRY_TURN'
    LEG = 'LEG_TRACK'
    APPROACH = 'CORNER_APPROACH'
    TURN = 'CORNER_TURN'
    RECOVER = 'VISION_RECOVER'
    SAFE = 'SAFE_STOP'
    COMPLETE = 'COMPLETE'

    def __init__(self, config: HybridConfig):
        self.cfg = config
        self.reset('clockwise', None, None, 0.0)

    def reset(self, direction: str, yaw: Optional[float], position: Optional[Tuple[float, float]], now: float) -> None:
        normalized = str(direction or 'clockwise').lower()
        self.entry_sign = -1.0 if ('counter' in normalized or 'ccw' in normalized or '逆' in normalized) else 1.0
        self.ring_sign = -self.entry_sign
        self.state = self.ENTRY
        self.started_at = now
        self.state_started_at = now
        self.turn_start_yaw = yaw
        self.turn_sign = self.entry_sign
        self.turn_target_deg = self.cfg.entry_target_deg
        self.leg_heading_yaw = yaw
        self.turn_count = 0
        self.leg_index = 0
        self.leg_start_path_m = 0.0
        self.path_m = 0.0
        self.last_position = position
        self.last_yaw = yaw
        self.last_yaw_at = now
        self.last_yaw_rate = 0.0
        self.last_angular = 0.0
        self.turn_evidence_frames = 0
        self.bridge_active = False
        self.bridge_start_yaw = yaw
        self.recover_start_path_m = 0.0
        self.safe_reason = ''

    def _leg_lengths(self) -> Tuple[float, ...]:
        try:
            values = tuple(float(item.strip()) for item in self.cfg.leg_lengths_csv.split(','))
            return tuple(value for value in values if value > 0.0) or (1.0,)
        except (AttributeError, ValueError):
            return (1.0,)

    def _leg_target_m(self) -> float:
        lengths = self._leg_lengths()
        return lengths[min(self.leg_index, len(lengths) - 1)]

    def _leg_progress_m(self) -> float:
        return max(0.0, self.path_m - self.leg_start_path_m)

    def turn_gate_status(self) -> Tuple[float, float]:
        return self._leg_progress_m(), self._leg_target_m()

    def is_bridge_active(self) -> bool:
        return self.bridge_active

    def _line_valid(self, line: Dict) -> bool:
        return (
            bool(line.get('valid', False))
            and float(line.get('age', 999.0) or 999.0) <= self.cfg.vision_max_age_sec
            and float(line.get('confidence', 0.0) or 0.0) >= self.cfg.vision_min_confidence
            and int(line.get('valid_rows', 0) or 0) >= self.cfg.vision_min_rows
        )

    def _turn_entry_evidence(self, line: Dict) -> bool:
        """A corner starts as soon as the top 20% of the ROI loses SEG.

        This is deliberately a single fresh-frame rule. Waiting for curve,
        boundary, or multi-frame confirmation lets the chassis run past the
        turn-in point. The top strip is measured independently from the apex
        window used for turn exit.
        """
        return self._line_valid(line) and float(line.get('top20_seg_fill', 1.0) or 0.0) <= self.cfg.turn_entry_top20_max_fill

    def _turn_exit_evidence(self, line: Dict) -> bool:
        """New-straight confirmation at the highest visible centerline.

        Both sides of the apex-centered 30% window must be filled.  This is
        the visual event that says the turn has opened into its following leg.
        """
        threshold = self.cfg.turn_exit_fill_threshold
        return self._line_valid(line) and (
            float(line.get('apex_left30_fill', 0.0) or 0.0) >= threshold
            and float(line.get('apex_right30_fill', 0.0) or 0.0) >= threshold
        )

    def _line_angular(self, line: Dict) -> float:
        error = float(line.get('error', 0.0) or 0.0)
        curve = float(line.get('curve', line.get('path_bend', 0.0)) or 0.0)
        return _clamp(-(self.cfg.line_kp * error + self.cfg.curve_kp * curve), self.cfg.line_max_angular)

    def _update_path(self, position: Optional[Tuple[float, float]]) -> None:
        if position is None:
            return
        if self.last_position is not None:
            step = math.hypot(position[0] - self.last_position[0], position[1] - self.last_position[1])
            if 0.0 < step < 0.20:
                self.path_m += step
        self.last_position = position

    def _update_yaw(self, yaw: Optional[float], now: float) -> float:
        if yaw is None:
            return 0.0
        rate = 0.0
        if self.last_yaw is not None and now > self.last_yaw_at:
            delta = _angle_error(yaw, self.last_yaw)
            rate = delta / (now - self.last_yaw_at)
        self.last_yaw, self.last_yaw_at = yaw, now
        self.last_yaw_rate = rate
        return rate

    def _enter_safe_stop(self, reason: str, now: float) -> HybridCommand:
        self.state, self.state_started_at, self.safe_reason, self.last_angular = self.SAFE, now, reason, 0.0
        return HybridCommand(0.0, 0.0, self.SAFE, safe_stop=True)

    def _start_turn(self, sign: float, target_deg: float, yaw: float, now: float) -> None:
        self.state, self.state_started_at = self.TURN, now
        self.turn_start_yaw, self.turn_sign, self.turn_target_deg = yaw, sign, target_deg
        self.turn_evidence_frames = 0

    def _turn_progress_deg(self, yaw: float) -> float:
        return max(0.0, math.degrees(_angle_error(yaw, self.turn_start_yaw)) * self.turn_sign)

    def _turn_command(self, yaw: float, yaw_rate: float) -> Tuple[float, float]:
        target = self.turn_start_yaw + self.turn_sign * math.radians(self.turn_target_deg)
        error = _angle_error(target, yaw)
        angular = _clamp(self.cfg.turn_kp * error - self.cfg.turn_rate_kd * yaw_rate, self.cfg.max_angular)
        # The nominal IMU angle is a braking reference, not a completion rule.
        # Do not keep forcing the car deeper into the corner after that point:
        # the P/D response naturally counter-steers for overshoot while SEG
        # remains the sole authority that releases the next straight.
        return self.cfg.turn_speed, angular

    def _turn_exit_ready(self, line: Dict, yaw: float, yaw_rate: float) -> bool:
        progress = self._turn_progress_deg(yaw)
        return bool(
            progress >= self.cfg.turn_min_deg
            and self._turn_exit_evidence(line)
            and abs(yaw_rate) <= self.cfg.turn_exit_max_yaw_rate
        )

    def _bridge_progress_deg(self, yaw: float) -> float:
        return max(0.0, math.degrees(_angle_error(yaw, self.bridge_start_yaw)) * self.ring_sign)

    def _long_edge_visible(self, line: Dict) -> bool:
        """Detect the long edge after skipping the short-side state.

        The short side may have no far SEG at all. It is therefore never used
        as an input; the bridge completes only when the following long edge
        itself becomes visible and locally straight.
        """
        return (
            self._line_valid(line)
            and float(line.get('top20_seg_fill', 0.0) or 0.0) >= self.cfg.bridge_long_top20_min_fill
            and abs(float(line.get('curve', line.get('path_bend', 0.0)) or 0.0)) <= self.cfg.bridge_long_max_curve
        )

    def _long_edge_found(self, line: Dict) -> bool:
        return self._long_edge_visible(line) and abs(float(line.get('error', 0.0) or 0.0)) <= self.cfg.bridge_capture_error

    def _bridge_command(self, line: Dict) -> HybridCommand:
        # A long edge may appear while the car is still laterally displaced by
        # the bridge. Stop adding the bridge turn immediately and capture that
        # edge with the ordinary visual controller before declaring it active.
        if self._long_edge_visible(line):
            self.last_angular = self._line_angular(line)
            return HybridCommand(self.cfg.cruise_speed, self.last_angular, self.LEG)
        visual_trim = 0.35 * self._line_angular(line) if self._line_valid(line) else 0.0
        angular = _clamp(
            self.ring_sign * self.cfg.bridge_turn_angular + visual_trim,
            self.cfg.max_angular,
        )
        self.last_angular = angular
        return HybridCommand(self.cfg.turn_speed, angular, self.LEG)

    def _begin_leg(self, yaw: float, now: float) -> None:
        self.state, self.state_started_at = self.LEG, now
        self.leg_heading_yaw, self.leg_start_path_m = yaw, self.path_m
        self.turn_evidence_frames = 0

    def step(self, now: float, line: Dict, yaw: Optional[float], position: Optional[Tuple[float, float]]) -> HybridCommand:
        self._update_path(position)
        yaw_rate = self._update_yaw(yaw, now)
        if self.state == self.SAFE:
            return HybridCommand(0.0, 0.0, self.SAFE, safe_stop=True)
        if self.state == self.COMPLETE:
            return HybridCommand(0.0, 0.0, self.COMPLETE, completed=True)
        if yaw is None:
            return self._enter_safe_stop('missing_imu_yaw', now)
        valid = self._line_valid(line)
        if self.state == self.ENTRY:
            if self.turn_start_yaw is None:
                self.turn_start_yaw = yaw
            linear, angular = self._turn_command(yaw, yaw_rate)
            self.last_angular = angular
            if self._turn_exit_ready(line, yaw, yaw_rate):
                self._begin_leg(yaw, now)
                return HybridCommand(self.cfg.recover_speed, 0.0, self.LEG)
            if self._turn_progress_deg(yaw) >= self.cfg.turn_max_deg:
                return self._enter_safe_stop('entry_visual_exit_not_found', now)
            return HybridCommand(linear, angular, self.ENTRY)

        if self.state == self.LEG:
            progress, target = self.turn_gate_status()
            if self.turn_count >= self.cfg.required_ring_turns and progress >= target:
                self.state = self.COMPLETE
                return HybridCommand(0.0, 0.0, self.COMPLETE, completed=True)
            if self.bridge_active:
                if self._long_edge_found(line):
                    self.bridge_active = False
                    # The bridge contains the second physical corner; resume
                    # directly on the long edge rather than making a short-leg
                    # state that can immediately retrigger another turn.
                    self.turn_count += 1
                    self.leg_index += 1
                    self._begin_leg(yaw, now)
                    self.last_angular = self._line_angular(line)
                    return HybridCommand(self.cfg.cruise_speed, self.last_angular, self.LEG)
                if self._bridge_progress_deg(yaw) >= self.cfg.bridge_max_deg:
                    return self._enter_safe_stop('bridge_long_edge_not_found', now)
                return self._bridge_command(line)
            if not valid:
                self.state, self.state_started_at, self.recover_start_path_m = self.RECOVER, now, self.path_m
                return HybridCommand(self.cfg.recover_speed, 0.0, self.RECOVER)
            if self._turn_entry_evidence(line) or progress >= target + self.cfg.leg_extra_m:
                self._start_turn(self.ring_sign, self.cfg.ring_target_deg, yaw, now)
                self.last_angular = self.ring_sign * min(0.42, self.cfg.max_angular)
                return HybridCommand(self.cfg.turn_speed, self.last_angular, self.TURN)
            self.last_angular = self._line_angular(line)
            return HybridCommand(self.cfg.cruise_speed, self.last_angular, self.LEG)

        if self.state == self.APPROACH:
            progress, target = self.turn_gate_status()
            if self._turn_entry_evidence(line) or progress >= target + self.cfg.leg_extra_m:
                self._start_turn(self.ring_sign, self.cfg.ring_target_deg, yaw, now)
                self.last_angular = self.ring_sign * min(0.42, self.cfg.max_angular)
                return HybridCommand(self.cfg.turn_speed, self.last_angular, self.TURN)
            self.state = self.LEG
            return HybridCommand(self.cfg.cruise_speed, self._line_angular(line) if valid else 0.0, self.LEG)

        if self.state == self.TURN:
            linear, angular = self._turn_command(yaw, yaw_rate)
            self.last_angular = angular
            if self._turn_exit_ready(line, yaw, yaw_rate):
                self.turn_count += 1
                self.leg_index += 1
                self._begin_leg(yaw, now)
                if self.turn_count == 1:
                    # The first ring corner and its following short side lead
                    # straight into the long edge. Treat that pair as one
                    # continuous visual bridge instead of a separate short-leg
                    # tracker plus a second entry trigger.
                    self.bridge_active = True
                    self.bridge_start_yaw = yaw
                    return self._bridge_command(line)
                return HybridCommand(self.cfg.recover_speed, 0.0, self.LEG)
            if self._turn_progress_deg(yaw) >= self.cfg.turn_max_deg:
                return self._enter_safe_stop('corner_visual_exit_not_found', now)
            return HybridCommand(linear, angular, self.TURN)

        if self.state == self.RECOVER:
            if valid:
                self._begin_leg(yaw, now)
                self.last_angular = self._line_angular(line)
                return HybridCommand(self.cfg.recover_speed, self.last_angular, self.LEG)
            if self.path_m - self.recover_start_path_m >= self.cfg.vision_loss_distance_m:
                return self._enter_safe_stop('vision_lost_distance_limit', now)
            heading_error = _angle_error(self.leg_heading_yaw, yaw)
            self.last_angular = _clamp(0.8 * heading_error, 0.18)
            return HybridCommand(self.cfg.recover_speed, self.last_angular, self.RECOVER)

        return self._enter_safe_stop('unknown_state', now)

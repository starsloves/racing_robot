"""Bounded MPPI local avoidance for the Stage2 top-long straight."""

from dataclasses import dataclass
import math
import random
from typing import Dict, Optional, Tuple


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class MppiStraightAvoidanceConfig:
    enabled: bool = True
    linear_speed_mps: float = 0.42
    max_angular_speed_rps: float = 0.80
    horizon_steps: int = 50
    step_sec: float = 0.05
    batch_size: int = 128
    temperature: float = 12.0
    angular_noise_rps: float = 0.45
    vehicle_half_width_m: float = 0.15
    clearance_m: float = 0.07
    recovery_lateral_tolerance_m: float = 0.04
    recovery_heading_tolerance_deg: float = 4.0


@dataclass(frozen=True)
class MppiStraightAvoidanceCommand:
    linear: float
    angular: float
    state: str
    cost: float
    min_clearance_m: float


class MppiStraightAvoidanceController:
    """Sample Ackermann-feasible yaw-rate sequences around a straight reference.

    The caller supplies scan-frame obstacle and fence observations. During one
    detour the controller integrates its commanded displacement in the line
    frame, while IMU yaw provides the actual heading used for each replan.
    """

    _INVALID_COST = 1.0e9

    def __init__(self, config: MppiStraightAvoidanceConfig):
        self.config = config
        self._rng = random.Random(20260723)
        self.reset()

    @property
    def is_active(self) -> bool:
        return self._active

    def reset(self) -> None:
        self._active = False
        self._state = 'idle'
        self._line_lateral_m = 0.0
        self._line_origin_position: Optional[Tuple[float, float]] = None
        self._last_time_sec: Optional[float] = None
        self._last_linear = 0.0
        self._nominal_angular = [0.0] * max(1, int(self.config.horizon_steps))
        self._last_corridor: Optional[Dict[str, float]] = None

    def step(self, *, now_sec: float, yaw: float, line_heading: float,
             line_speed: float, obstacle: Optional[Dict[str, float]],
             corridor: Optional[Dict[str, float]],
             position: Optional[Tuple[float, float]] = None
             ) -> Optional[MppiStraightAvoidanceCommand]:
        if not self.config.enabled:
            self.reset()
            return None

        heading = _wrap_angle(float(yaw) - float(line_heading))
        self._update_line_lateral(float(now_sec), heading, line_heading, position)

        if not self._active:
            if obstacle is None or corridor is None:
                return None
            self._active = True
            self._state = 'avoiding'
            self._line_lateral_m = 0.0
            self._line_origin_position = position
            self._nominal_angular = [0.0] * len(self._nominal_angular)

        if corridor is not None:
            self._last_corridor = dict(corridor)
        if self._last_corridor is None:
            self.reset()
            return None

        world_obstacle = self._obstacle_in_line_frame(obstacle, heading)
        bounds = self._line_bounds(self._last_corridor)
        if world_obstacle is None:
            self._state = 'recovering'
            if (
                abs(self._line_lateral_m) <= self.config.recovery_lateral_tolerance_m
                and abs(heading) <= math.radians(self.config.recovery_heading_tolerance_deg)
            ):
                self.reset()
                return None
        else:
            self._state = 'avoiding'

        linear = min(max(0.05, float(line_speed)), self.config.linear_speed_mps)
        result = self._optimize(linear, heading, world_obstacle, bounds)
        if result is None:
            self._last_linear = 0.0
            return MppiStraightAvoidanceCommand(0.0, 0.0, 'blocked', self._INVALID_COST, 0.0)

        angular, cost, clearance = result
        self._last_linear = linear
        return MppiStraightAvoidanceCommand(linear, angular, self._state, cost, clearance)

    def _update_line_lateral(self, now_sec: float, heading: float,
                              line_heading: float,
                              position: Optional[Tuple[float, float]]) -> None:
        """Use odom xy for cross-track; command integration is only a fallback.

        The Stage2 pose contract permits odometry xy for distance/position but
        never its orientation.  Projecting xy onto the IMU-locked line normal
        therefore captures real tyre slip without introducing odometry yaw.
        """
        if position is not None:
            if self._line_origin_position is None:
                self._line_origin_position = position
            dx = float(position[0]) - float(self._line_origin_position[0])
            dy = float(position[1]) - float(self._line_origin_position[1])
            self._line_lateral_m = -math.sin(line_heading) * dx + math.cos(line_heading) * dy
            self._last_time_sec = now_sec
            return
        if self._last_time_sec is not None:
            dt = _clamp(now_sec - self._last_time_sec, 0.0, 0.15)
            self._line_lateral_m += self._last_linear * math.sin(heading) * dt
        self._last_time_sec = now_sec

    def _obstacle_in_line_frame(self, obstacle: Optional[Dict[str, float]],
                                heading: float) -> Optional[Dict[str, float]]:
        if obstacle is None:
            return None
        center_x = float(obstacle['center_x'])
        center_y = float(obstacle['center_y'])
        return {
            'x': center_x * math.cos(heading) - center_y * math.sin(heading),
            'y': self._line_lateral_m + center_x * math.sin(heading) + center_y * math.cos(heading),
            'radius': 0.5 * float(obstacle['span'])
                      + self.config.vehicle_half_width_m + self.config.clearance_m,
        }

    def _line_bounds(self, corridor: Dict[str, float]):
        return (
            self._line_lateral_m + float(corridor['right'])
            + self.config.vehicle_half_width_m + self.config.clearance_m,
            self._line_lateral_m + float(corridor['left'])
            - self.config.vehicle_half_width_m - self.config.clearance_m,
        )

    def _optimize(self, linear: float, heading: float,
                  obstacle: Optional[Dict[str, float]], bounds):
        candidates = [list(self._nominal_angular)]
        candidates.extend(self._seed_sequences())
        for _ in range(max(1, int(self.config.batch_size) - len(candidates))):
            candidate = []
            noise = 0.0
            for nominal in self._nominal_angular:
                noise = 0.65 * noise + 0.35 * self._rng.gauss(0.0, self.config.angular_noise_rps)
                candidate.append(_clamp(
                    nominal + noise,
                    -self.config.max_angular_speed_rps,
                    self.config.max_angular_speed_rps,
                ))
            candidates.append(candidate)

        scored = [self._rollout(candidate, linear, heading, obstacle, bounds)
                  for candidate in candidates]
        feasible = [(sequence, score) for sequence, score in zip(candidates, scored)
                    if score[0] < self._INVALID_COST]
        if not feasible:
            return None

        min_cost = min(score[0] for _, score in feasible)
        temperature = max(0.1, self.config.temperature)
        weights = [math.exp(-(score[0] - min_cost) / temperature)
                   for _, score in feasible]
        weight_sum = sum(weights)
        optimized = [sum(
            weight * sequence[step_index]
            for weight, (sequence, _) in zip(weights, feasible)
        ) / max(weight_sum, 1.0e-9) for step_index in range(len(self._nominal_angular))]
        best_sequence, (best_cost, best_clearance) = min(
            feasible, key=lambda item: item[1][0]
        )
        weighted_cost, _ = self._rollout(optimized, linear, heading, obstacle, bounds)
        # Symmetric left/right samples can average back into the obstacle. In
        # that case retain the best collision-free sampled control sequence.
        if weighted_cost >= self._INVALID_COST:
            optimized = list(best_sequence)
        self._nominal_angular = optimized[1:] + [0.0]
        return optimized[0], best_cost, best_clearance

    def _seed_sequences(self):
        """Guarantee both physically bounded S-shifts are represented."""
        sequences = []
        count = len(self._nominal_angular)
        for turn_steps in (12, 16, 20):
            for sign in (-1.0, 1.0):
                sequence = [0.0] * count
                for index in range(min(turn_steps, count)):
                    sequence[index] = sign * self.config.max_angular_speed_rps
                for index in range(turn_steps, min(2 * turn_steps, count)):
                    sequence[index] = -sign * self.config.max_angular_speed_rps
                sequences.append(sequence)
        return sequences

    def _rollout(self, angular_sequence, linear: float, heading: float,
                 obstacle: Optional[Dict[str, float]], bounds):
        lower_y, upper_y = bounds
        if lower_y >= upper_y:
            return self._INVALID_COST, 0.0

        x = 0.0
        y = self._line_lateral_m
        theta = heading
        min_clearance = float('inf')
        control_cost = 0.0
        previous_angular = 0.0
        for angular in angular_sequence:
            theta = _wrap_angle(theta + angular * self.config.step_sec)
            x += linear * math.cos(theta) * self.config.step_sec
            y += linear * math.sin(theta) * self.config.step_sec
            if y <= lower_y or y >= upper_y:
                return self._INVALID_COST, 0.0
            if obstacle is not None:
                clearance = math.hypot(x - obstacle['x'], y - obstacle['y']) - obstacle['radius']
                min_clearance = min(min_clearance, clearance)
                if clearance <= 0.0:
                    return self._INVALID_COST, 0.0
            control_cost += 0.35 * angular * angular + 0.18 * (angular - previous_angular) ** 2
            previous_angular = angular

        obstacle_cost = 0.0 if obstacle is None else 2.0 / max(min_clearance, 0.02)
        cost = 32.0 * y * y + 8.0 * theta * theta + control_cost + obstacle_cost
        return cost, min_clearance

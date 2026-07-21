"""IMU-closed-loop, corridor-constrained lane shifts for Stage2 lines."""

from dataclasses import dataclass
import math
from typing import Optional


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class StraightAvoidanceCommand:
    linear: float
    angular: float
    state: str


@dataclass(frozen=True)
class StraightAvoidancePlan:
    """A scan-validated, nonzero-speed offset around one front obstacle."""

    turn_sign: float
    lateral_shift_m: float
    obstacle_distance_m: float
    yaw_offset_rad: float
    required_forward_m: float


class StraightAvoidanceController:
    """Execute a precomputed S lane shift while retaining the line speed."""

    def __init__(self, *, enabled: bool, angular_speed: float,
                 yaw_offset_deg: float, yaw_tolerance_deg: float,
                 start_heading_tolerance_deg: float = 15.0,
                 max_turn_travel_deg: float = 35.0,
                 speed_limit_mps: float = 0.0):
        self.enabled = bool(enabled)
        self.angular_speed = max(0.01, abs(float(angular_speed)))
        self.max_yaw_offset = math.radians(max(0.1, abs(float(yaw_offset_deg))))
        self.yaw_tolerance = math.radians(max(0.1, abs(float(yaw_tolerance_deg))))
        self.start_heading_tolerance = math.radians(
            max(float(yaw_offset_deg), abs(float(start_heading_tolerance_deg)))
        )
        self.max_turn_travel = math.radians(
            max(2.0 * abs(float(yaw_offset_deg)) + self.yaw_tolerance,
                abs(float(max_turn_travel_deg)))
        )
        self.speed_limit_mps = max(0.0, float(speed_limit_mps))
        self._state = 'idle'
        self._line_heading: Optional[float] = None
        self._turn_sign = 0.0
        self._turn_start_yaw: Optional[float] = None
        self._yaw_offset = self.max_yaw_offset
        self._plan: Optional[StraightAvoidancePlan] = None

    @property
    def is_active(self) -> bool:
        return self._state != 'idle'

    @property
    def state(self) -> str:
        return self._state

    def reset(self) -> None:
        self._state = 'idle'
        self._line_heading = None
        self._turn_sign = 0.0
        self._turn_start_yaw = None
        self._yaw_offset = self.max_yaw_offset
        self._plan = None

    def _speed(self, line_speed: float) -> float:
        if self.speed_limit_mps <= 0.0:
            return max(0.0, float(line_speed))
        return min(max(0.0, float(line_speed)), self.speed_limit_mps)

    def _target_reached(self, target_yaw: float, yaw: float) -> bool:
        return abs(_wrap_angle(target_yaw - yaw)) <= self.yaw_tolerance

    def _target_passed(self, target_yaw: float, yaw: float, turn_sign: float) -> bool:
        """Treat a sampled overshoot in the commanded turn direction as complete."""
        return turn_sign * _wrap_angle(target_yaw - yaw) <= 0.0

    def _begin_turn(self, yaw: float) -> None:
        self._turn_start_yaw = yaw

    def _turn_exceeded_limit(self, yaw: float) -> bool:
        if self._turn_start_yaw is None:
            return False
        return abs(_wrap_angle(yaw - self._turn_start_yaw)) > self.max_turn_travel

    @staticmethod
    def plan_for_offset(*, lateral_shift_m: float, obstacle_distance_m: float,
                        linear_speed: float, angular_speed: float,
                        max_yaw_offset_rad: float, forward_margin_m: float
                        ) -> Optional[StraightAvoidancePlan]:
        """Return the smallest fixed-speed S shift that clears the obstacle.

        For an equal-radius ``+theta/-2theta/+theta`` maneuver, final lateral
        displacement is ``2R(1-cos(theta))`` and forward travel is
        ``4R*sin(theta)``.  The complete shift must finish before the obstacle.
        """
        shift = abs(float(lateral_shift_m))
        speed = max(0.01, abs(float(linear_speed)))
        angular = max(0.01, abs(float(angular_speed)))
        radius = speed / angular
        if shift <= 0.0 or shift >= 2.0 * radius:
            return None
        yaw_offset = math.acos(1.0 - shift / (2.0 * radius))
        if yaw_offset > max_yaw_offset_rad:
            return None
        required_forward = 4.0 * radius * math.sin(yaw_offset)
        if obstacle_distance_m < required_forward + max(0.0, forward_margin_m):
            return None
        return StraightAvoidancePlan(
            turn_sign=1.0 if lateral_shift_m > 0.0 else -1.0,
            lateral_shift_m=shift,
            obstacle_distance_m=float(obstacle_distance_m),
            yaw_offset_rad=yaw_offset,
            required_forward_m=required_forward,
        )

    @property
    def plan(self) -> Optional[StraightAvoidancePlan]:
        return self._plan

    def step(self, *, yaw: float, line_heading: float, line_speed: float,
             plan: Optional[StraightAvoidancePlan]) -> Optional[StraightAvoidanceCommand]:
        """Return an override command while active, otherwise ``None``."""
        if not self.enabled:
            self.reset()
            return None

        if self._state == 'idle':
            if plan is None:
                return None
            self._line_heading = _wrap_angle(line_heading)
            # A line can be selected immediately after an arc, while the
            # chassis is still turning.  Do not chase an absolute line yaw
            # from that pose: the normal line controller must settle first.
            if abs(_wrap_angle(self._line_heading - yaw)) > self.start_heading_tolerance:
                self.reset()
                return None
            self._plan = plan
            self._turn_sign = plan.turn_sign
            self._yaw_offset = plan.yaw_offset_rad
            self._state = 'turn_away'
            self._begin_turn(yaw)

        assert self._line_heading is not None
        speed = self._speed(line_speed)
        if self._state == 'turn_away':
            target = _wrap_angle(self._line_heading + self._turn_sign * self._yaw_offset)
            if self._turn_exceeded_limit(yaw):
                self.reset()
                return None
            if self._target_reached(target, yaw) or self._target_passed(
                    target, yaw, self._turn_sign):
                self._state = 'turn_reverse'
                self._begin_turn(yaw)
            else:
                return StraightAvoidanceCommand(speed, self._turn_sign * self.angular_speed,
                                                self._state)

        if self._state == 'turn_reverse':
            target = _wrap_angle(self._line_heading - self._turn_sign * self._yaw_offset)
            reverse_sign = -self._turn_sign
            if self._turn_exceeded_limit(yaw):
                self.reset()
                return None
            if self._target_reached(target, yaw) or self._target_passed(
                    target, yaw, reverse_sign):
                self._state = 'return_heading'
                self._begin_turn(yaw)
            else:
                return StraightAvoidanceCommand(speed, reverse_sign * self.angular_speed,
                                                self._state)

        if self._state == 'return_heading':
            if self._turn_exceeded_limit(yaw):
                self.reset()
                return None
            if self._target_reached(self._line_heading, yaw) or self._target_passed(
                    self._line_heading, yaw, self._turn_sign):
                self.reset()
                return None
            return StraightAvoidanceCommand(speed, self._turn_sign * self.angular_speed,
                                            self._state)

        self.reset()
        return None

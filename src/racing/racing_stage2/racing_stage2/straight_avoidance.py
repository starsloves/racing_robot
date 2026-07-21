"""IMU-closed-loop S avoidance for Stage2 production straight segments."""

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


class StraightAvoidanceController:
    """A fixed-angle S maneuver referenced only to the current line heading."""

    def __init__(self, *, enabled: bool, angular_speed: float,
                 yaw_offset_deg: float, yaw_tolerance_deg: float,
                 right_turn_left_obstacle_angle_deg: float,
                 speed_limit_mps: float = 0.0):
        self.enabled = bool(enabled)
        self.angular_speed = max(0.01, abs(float(angular_speed)))
        self.yaw_offset = math.radians(max(0.1, abs(float(yaw_offset_deg))))
        self.yaw_tolerance = math.radians(max(0.1, abs(float(yaw_tolerance_deg))))
        self.right_turn_left_obstacle_angle_deg = float(
            right_turn_left_obstacle_angle_deg
        )
        self.speed_limit_mps = max(0.0, float(speed_limit_mps))
        self._state = 'idle'
        self._line_heading: Optional[float] = None
        self._turn_sign = 0.0

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

    def _speed(self, line_speed: float) -> float:
        if self.speed_limit_mps <= 0.0:
            return max(0.0, float(line_speed))
        return min(max(0.0, float(line_speed)), self.speed_limit_mps)

    def _target_reached(self, target_yaw: float, yaw: float) -> bool:
        return abs(_wrap_angle(target_yaw - yaw)) <= self.yaw_tolerance

    def step(self, *, yaw: float, line_heading: float, line_speed: float,
             obstacle_angle_deg: Optional[float]) -> Optional[StraightAvoidanceCommand]:
        """Return an override command while active, otherwise ``None``."""
        if not self.enabled:
            self.reset()
            return None

        if self._state == 'idle':
            if obstacle_angle_deg is None:
                return None
            self._line_heading = _wrap_angle(line_heading)
            # Match Stage1: default left; only a clearly left-front obstacle
            # selects a right detour. The selected direction stays locked.
            self._turn_sign = (
                -1.0 if obstacle_angle_deg >= self.right_turn_left_obstacle_angle_deg
                else 1.0
            )
            self._state = 'turn_away'

        assert self._line_heading is not None
        speed = self._speed(line_speed)
        if self._state == 'turn_away':
            target = _wrap_angle(self._line_heading + self._turn_sign * self.yaw_offset)
            if self._target_reached(target, yaw):
                self._state = 'turn_reverse'
            else:
                return StraightAvoidanceCommand(speed, self._turn_sign * self.angular_speed,
                                                self._state)

        if self._state == 'turn_reverse':
            target = _wrap_angle(self._line_heading - self._turn_sign * self.yaw_offset)
            if self._target_reached(target, yaw):
                self._state = 'return_heading'
            else:
                return StraightAvoidanceCommand(speed, -self._turn_sign * self.angular_speed,
                                                self._state)

        if self._state == 'return_heading':
            if self._target_reached(self._line_heading, yaw):
                self.reset()
                return None
            return StraightAvoidanceCommand(speed, self._turn_sign * self.angular_speed,
                                            self._state)

        self.reset()
        return None

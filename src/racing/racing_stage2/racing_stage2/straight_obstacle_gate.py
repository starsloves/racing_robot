"""Temporal confirmation gate for Stage2 straight-line obstacle candidates."""

from dataclasses import dataclass
import math
from typing import Dict, Optional


@dataclass(frozen=True)
class StraightObstacleGateConfig:
    confirm_frames: int = 3
    association_x_m: float = 0.25
    association_y_m: float = 0.12
    association_span_m: float = 0.20
    cooldown_sec: float = 1.0


class StraightObstacleGate:
    """Confirm one spatially consistent scan cluster before an S-shift starts."""

    def __init__(self, config: StraightObstacleGateConfig):
        self._config = config
        self.reset()
        self._cooldown_until = 0.0

    @property
    def hit_count(self) -> int:
        return self._hit_count

    @property
    def state(self) -> str:
        if self._candidate is None:
            return 'clear'
        if self._hit_count >= self._config.confirm_frames:
            return 'confirmed'
        return 'candidate'

    def reset(self) -> None:
        self._candidate: Optional[Dict[str, float]] = None
        self._hit_count = 0

    def start_cooldown(self, now_sec: float) -> None:
        self.reset()
        self._cooldown_until = max(
            self._cooldown_until,
            float(now_sec) + max(0.0, self._config.cooldown_sec),
        )

    def update(self, obstacle: Optional[Dict[str, float]], now_sec: float
               ) -> Optional[Dict[str, float]]:
        """Return a confirmed obstacle only after consistent consecutive scans."""
        if now_sec < self._cooldown_until:
            self.reset()
            return None
        if obstacle is None:
            self.reset()
            return None

        if self._candidate is not None and self._matches(self._candidate, obstacle):
            self._hit_count += 1
        else:
            self._hit_count = 1
        self._candidate = dict(obstacle)

        if self._hit_count < max(1, self._config.confirm_frames):
            return None
        return dict(self._candidate)

    def _matches(self, previous: Dict[str, float], current: Dict[str, float]) -> bool:
        return (
            abs(float(current['center_x']) - float(previous['center_x']))
            <= self._config.association_x_m
            and abs(float(current['center_y']) - float(previous['center_y']))
            <= self._config.association_y_m
            and abs(float(current.get('lateral_span', 0.0))
                    - float(previous.get('lateral_span', 0.0)))
            <= self._config.association_span_m
            and math.isfinite(float(current['distance']))
        )

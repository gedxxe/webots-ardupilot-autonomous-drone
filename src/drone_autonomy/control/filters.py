from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    if low > high:
        raise ValueError("low must be <= high")
    return max(low, min(high, value))


def apply_deadband(value: float, deadband: float) -> float:
    if deadband < 0:
        raise ValueError("deadband must be non-negative")
    return 0.0 if abs(value) <= deadband else value


@dataclass
class LowPassFilter:
    alpha: float
    value: float | None = None

    def __post_init__(self) -> None:
        if not 0.0 < self.alpha <= 1.0:
            raise ValueError("alpha must be in the range (0, 1]")

    def reset(self) -> None:
        self.value = None

    def update(self, sample: float) -> float:
        if self.value is None:
            self.value = sample
        else:
            self.value = (self.alpha * sample) + ((1.0 - self.alpha) * self.value)
        return self.value

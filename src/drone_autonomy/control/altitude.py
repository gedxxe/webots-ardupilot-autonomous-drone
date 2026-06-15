from __future__ import annotations

from dataclasses import dataclass

from drone_autonomy.control.filters import apply_deadband, clamp


@dataclass(frozen=True)
class AltitudeHoldConfig:
    """Altitude-hold gains for mission-level velocity bias.

    This controller does not fuse raw GPS/rangefinder/optical-flow samples.
    It assumes `current_altitude_m` is already the best fused altitude estimate
    provided by ArduPilot EKF or by a simulator adapter that mimics that signal.
    """

    target_altitude_m: float = 1.0
    deadband_m: float = 0.08
    kp: float = 0.55
    max_climb_speed_m_s: float = 0.35
    max_descent_speed_m_s: float = 0.25

    def __post_init__(self) -> None:
        if self.target_altitude_m <= 0.0:
            raise ValueError("target_altitude_m must be positive")
        if self.deadband_m < 0.0:
            raise ValueError("deadband_m must be non-negative")
        if self.kp < 0.0:
            raise ValueError("kp must be non-negative")
        if self.max_climb_speed_m_s < 0.0:
            raise ValueError("max_climb_speed_m_s must be non-negative")
        if self.max_descent_speed_m_s < 0.0:
            raise ValueError("max_descent_speed_m_s must be non-negative")


class AltitudeHoldController:
    """Small proportional altitude corrector for body-frame velocity commands.

    Sign convention is intentionally repeated here because vertical sign bugs are
    easy to introduce: body z velocity is positive down, so being below target
    must produce a negative command to climb.
    """

    def __init__(self, config: AltitudeHoldConfig) -> None:
        self.config = config

    def body_vz_for_altitude(self, current_altitude_m: float) -> float:
        altitude_error_m = self.config.target_altitude_m - current_altitude_m
        altitude_error_m = apply_deadband(altitude_error_m, self.config.deadband_m)

        # Positive altitude error means "we are too low". Body z positive means
        # down, therefore the command is negated to climb back toward target.
        body_vz_m_s = -self.config.kp * altitude_error_m
        return clamp(
            body_vz_m_s,
            -self.config.max_climb_speed_m_s,
            self.config.max_descent_speed_m_s,
        )

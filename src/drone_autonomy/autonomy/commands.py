from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class CommandKind(str, Enum):
    """Command categories understood by future MAVLink/simulator adapters."""

    NONE = "none"
    SET_MODE = "set_mode"
    ARM = "arm"
    TAKEOFF = "takeoff"
    BODY_VELOCITY = "body_velocity"
    LAND = "land"
    HOLD = "hold"


@dataclass(frozen=True)
class VehicleCommand:
    """Vehicle command in a simulator/hardware-neutral format.

    Body velocity fields use ArduPilot-style body axes:
    x forward, y right, z down. Yaw rate is positive right/clockwise from the
    vehicle perspective.
    """

    kind: CommandKind
    reason: str = ""
    mode: str | None = None
    arm: bool | None = None
    altitude_m: float | None = None
    body_vx_m_s: float = 0.0
    body_vy_m_s: float = 0.0
    body_vz_m_s: float = 0.0
    yaw_rate_rad_s: float = 0.0

    @classmethod
    def none(cls, reason: str = "") -> VehicleCommand:
        return cls(kind=CommandKind.NONE, reason=reason)

    @classmethod
    def hold(cls, reason: str = "") -> VehicleCommand:
        return cls(kind=CommandKind.HOLD, reason=reason)

    @classmethod
    def set_mode(cls, mode: str, reason: str = "") -> VehicleCommand:
        return cls(kind=CommandKind.SET_MODE, mode=mode, reason=reason)

    @classmethod
    def arm_vehicle(cls, reason: str = "") -> VehicleCommand:
        return cls(kind=CommandKind.ARM, arm=True, reason=reason)

    @classmethod
    def takeoff(cls, altitude_m: float, reason: str = "") -> VehicleCommand:
        return cls(kind=CommandKind.TAKEOFF, altitude_m=altitude_m, reason=reason)

    @classmethod
    def land(cls, reason: str = "") -> VehicleCommand:
        return cls(kind=CommandKind.LAND, reason=reason)

    @classmethod
    def body_velocity(
        cls,
        *,
        body_vx_m_s: float = 0.0,
        body_vy_m_s: float = 0.0,
        body_vz_m_s: float = 0.0,
        yaw_rate_rad_s: float = 0.0,
        reason: str = "",
    ) -> VehicleCommand:
        """Build a body-frame velocity command.

        No coordinate transform is performed here. MAVLink/Webots adapters must
        preserve the body-frame convention documented on `VehicleCommand`.
        """

        return cls(
            kind=CommandKind.BODY_VELOCITY,
            body_vx_m_s=body_vx_m_s,
            body_vy_m_s=body_vy_m_s,
            body_vz_m_s=body_vz_m_s,
            yaw_rate_rad_s=yaw_rate_rad_s,
            reason=reason,
        )

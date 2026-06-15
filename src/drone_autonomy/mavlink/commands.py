from __future__ import annotations

from dataclasses import dataclass
from time import monotonic
from typing import Any

from pymavlink import mavutil

from drone_autonomy.autonomy.commands import CommandKind, VehicleCommand


# SET_POSITION_TARGET_LOCAL_NED type mask:
# ignore position x/y/z, acceleration x/y/z, and yaw; keep velocity x/y/z and
# yaw-rate active. These bits are documented by MAVLink POSITION_TARGET_TYPEMASK.
VELOCITY_AND_YAWRATE_TYPE_MASK = 1 | 2 | 4 | 64 | 128 | 256 | 1024


@dataclass(frozen=True)
class MavlinkCommandAdapterConfig:
    command_repeat_interval_s: float = 1.0
    body_frame: int = mavutil.mavlink.MAV_FRAME_BODY_NED


class MavlinkCommandAdapter:
    """Translate simulator-neutral `VehicleCommand` objects into MAVLink.

    The adapter is the only layer that may know MAVLink command IDs. Mission
    code must stay independent from ArduPilot transport details.
    """

    def __init__(
        self,
        master: Any,
        config: MavlinkCommandAdapterConfig | None = None,
    ) -> None:
        self.master = master
        self.config = config or MavlinkCommandAdapterConfig()
        self._last_non_velocity_sent_s: dict[tuple[object, ...], float] = {}

    def send(self, command: VehicleCommand, now_s: float | None = None) -> bool:
        """Send one command and return True when a MAVLink message was emitted."""

        now_s = monotonic() if now_s is None else now_s
        if command.kind == CommandKind.NONE:
            return False
        if command.kind == CommandKind.HOLD:
            command = VehicleCommand.body_velocity(reason=command.reason or "hold")
        if command.kind == CommandKind.SET_MODE:
            return self._send_set_mode(command, now_s)
        if command.kind == CommandKind.ARM:
            return self._send_arm(command, now_s)
        if command.kind == CommandKind.TAKEOFF:
            return self._send_takeoff(command, now_s)
        if command.kind == CommandKind.LAND:
            return self._send_land(command, now_s)
        if command.kind == CommandKind.BODY_VELOCITY:
            self._send_body_velocity(command, now_s)
            return True
        raise ValueError(f"Unsupported command kind: {command.kind}")

    def request_message_interval(self, message_id: int, rate_hz: float) -> None:
        """Ask ArduPilot to stream one MAVLink message at the requested rate."""

        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        interval_us = int(1_000_000 / rate_hz)
        self.master.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    def request_default_telemetry(self, rate_hz: float = 20.0) -> None:
        """Request the messages needed by `MavlinkTelemetryAdapter`."""

        self.request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            rate_hz,
        )
        self.request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
            2.0,
        )

    @property
    def _target_system(self) -> int:
        return int(getattr(self.master, "target_system", 1) or 1)

    @property
    def _target_component(self) -> int:
        return int(getattr(self.master, "target_component", 1) or 1)

    def _send_set_mode(self, command: VehicleCommand, now_s: float) -> bool:
        if not command.mode:
            raise ValueError("SET_MODE command requires mode")
        key = (command.kind, command.mode)
        if self._skip_repeated(key, now_s):
            return False

        # pymavlink's mavutil master knows ArduPilot mode mappings after a
        # heartbeat. Prefer it over hand-building custom-mode messages.
        if hasattr(self.master, "set_mode"):
            self.master.set_mode(command.mode)
        else:
            mode_id = self.master.mode_mapping()[command.mode]
            self.master.mav.command_long_send(
                self._target_system,
                self._target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
                0,
                0,
                0,
                0,
                0,
            )
        return True

    def _send_arm(self, command: VehicleCommand, now_s: float) -> bool:
        should_arm = True if command.arm is None else bool(command.arm)
        key = (command.kind, should_arm)
        if self._skip_repeated(key, now_s):
            return False
        self.master.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if should_arm else 0.0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        return True

    def _send_takeoff(self, command: VehicleCommand, now_s: float) -> bool:
        if command.altitude_m is None:
            raise ValueError("TAKEOFF command requires altitude_m")
        key = (command.kind, round(command.altitude_m, 2))
        if self._skip_repeated(key, now_s):
            return False
        self.master.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            command.altitude_m,
        )
        return True

    def _send_land(self, command: VehicleCommand, now_s: float) -> bool:
        key = (command.kind,)
        if self._skip_repeated(key, now_s):
            return False
        self.master.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        return True

    def _send_body_velocity(self, command: VehicleCommand, now_s: float) -> None:
        time_boot_ms = int(now_s * 1000) & 0xFFFFFFFF
        self.master.mav.set_position_target_local_ned_send(
            time_boot_ms,
            self._target_system,
            self._target_component,
            self.config.body_frame,
            VELOCITY_AND_YAWRATE_TYPE_MASK,
            0,
            0,
            0,
            command.body_vx_m_s,
            command.body_vy_m_s,
            command.body_vz_m_s,
            0,
            0,
            0,
            0,
            command.yaw_rate_rad_s,
        )

    def _skip_repeated(self, key: tuple[object, ...], now_s: float) -> bool:
        last_sent_s = self._last_non_velocity_sent_s.get(key)
        if (
            last_sent_s is not None
            and now_s - last_sent_s < self.config.command_repeat_interval_s
        ):
            return True
        self._last_non_velocity_sent_s[key] = now_s
        return False

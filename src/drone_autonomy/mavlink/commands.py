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
    command_ack_required: bool = True
    command_ack_timeout_s: float = 1.0
    command_ack_max_retries: int = 2
    body_frame: int = mavutil.mavlink.MAV_FRAME_BODY_NED

    def __post_init__(self) -> None:
        if self.command_repeat_interval_s < 0.0:
            raise ValueError("command_repeat_interval_s must be non-negative")
        if self.command_ack_timeout_s <= 0.0:
            raise ValueError("command_ack_timeout_s must be positive")
        if self.command_ack_max_retries < 0:
            raise ValueError("command_ack_max_retries must be non-negative")


@dataclass
class _PendingCommandAck:
    key: tuple[object, ...]
    command_id: int
    label: str
    command_long_args: tuple[object, ...]
    last_sent_s: float
    retries_sent: int = 0

    @property
    def attempts(self) -> int:
        return 1 + self.retries_sent


@dataclass(frozen=True)
class CommandAckEvent:
    """Operator-facing status for one MAVLink COMMAND_ACK event."""

    command_id: int
    label: str
    severity: str
    detail: str
    attempts: int


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
        self._pending_acks: dict[tuple[object, ...], _PendingCommandAck] = {}
        self._ack_events: list[CommandAckEvent] = []
        self._ack_failures: list[CommandAckEvent] = []

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
    def pending_ack_count(self) -> int:
        """Number of command ACKs still awaited by the adapter."""

        return len(self._pending_acks)

    def update_message(self, message: Any) -> None:
        """Consume COMMAND_ACK messages from the shared MAVLink receive loop."""

        if self._message_type(message) != "COMMAND_ACK":
            return
        command_id = int(getattr(message, "command", -1))
        result = int(getattr(message, "result", -1))
        pending_key = self._pending_key_for_command(command_id)
        pending = self._pending_acks.get(pending_key) if pending_key is not None else None
        result_name = _mav_result_name(result)

        if pending is None:
            # ArduPilot also ACKs setup commands such as message-interval
            # requests. Those commands are not part of the mission safety
            # contract, so they must not trip the fail-closed path.
            accepted = result in {
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            }
            self._ack_events.append(
                CommandAckEvent(
                    command_id=command_id,
                    label=f"untracked command {command_id}",
                    severity="info" if accepted else "warning",
                    detail=f"untracked COMMAND_ACK result={result_name}",
                    attempts=0,
                )
            )
            return

        label = pending.label
        attempts = pending.attempts

        if result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
            pending.last_sent_s = monotonic()
            self._ack_events.append(
                CommandAckEvent(
                    command_id=command_id,
                    label=label,
                    severity="info",
                    detail=f"COMMAND_ACK in progress result={result_name}",
                    attempts=attempts,
                )
            )
            return

        if pending is not None:
            del self._pending_acks[pending.key]

        accepted = result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        event = CommandAckEvent(
            command_id=command_id,
            label=label,
            severity="info" if accepted else "failure",
            detail=f"COMMAND_ACK result={result_name}",
            attempts=attempts,
        )
        if accepted:
            self._ack_events.append(event)
        else:
            self._ack_failures.append(event)

    def update_ack_timeouts(self, now_s: float | None = None) -> None:
        """Retry pending COMMAND_LONG messages or fail closed after timeout."""

        if not self.config.command_ack_required:
            return
        now_s = monotonic() if now_s is None else now_s
        for key, pending in list(self._pending_acks.items()):
            if now_s - pending.last_sent_s < self.config.command_ack_timeout_s:
                continue

            if pending.retries_sent < self.config.command_ack_max_retries:
                self.master.mav.command_long_send(*pending.command_long_args)
                pending.retries_sent += 1
                pending.last_sent_s = now_s
                self._ack_events.append(
                    CommandAckEvent(
                        command_id=pending.command_id,
                        label=pending.label,
                        severity="warning",
                        detail=(
                            "COMMAND_ACK timeout; retry "
                            f"{pending.retries_sent}/"
                            f"{self.config.command_ack_max_retries}"
                        ),
                        attempts=pending.attempts,
                    )
                )
                continue

            del self._pending_acks[key]
            failure = CommandAckEvent(
                command_id=pending.command_id,
                label=pending.label,
                severity="failure",
                detail=(
                    "COMMAND_ACK timeout after "
                    f"{pending.attempts} send attempt(s)"
                ),
                attempts=pending.attempts,
            )
            self._ack_failures.append(failure)

    def pop_ack_events(self) -> tuple[CommandAckEvent, ...]:
        """Return and clear non-fatal ACK notices."""

        events = tuple(self._ack_events)
        self._ack_events.clear()
        return events

    def pop_ack_failures(self) -> tuple[CommandAckEvent, ...]:
        """Return and clear ACK failures that should fail closed."""

        failures = tuple(self._ack_failures)
        self._ack_failures.clear()
        return failures

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
        return self._send_tracked_command_long(
            key,
            "arm" if should_arm else "disarm",
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            now_s,
            1.0 if should_arm else 0.0,
        )

    def _send_takeoff(self, command: VehicleCommand, now_s: float) -> bool:
        if command.altitude_m is None:
            raise ValueError("TAKEOFF command requires altitude_m")
        key = (command.kind, round(command.altitude_m, 2))
        return self._send_tracked_command_long(
            key,
            "takeoff",
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            now_s,
            0,
            0,
            0,
            0,
            0,
            0,
            command.altitude_m,
        )

    def _send_land(self, command: VehicleCommand, now_s: float) -> bool:
        key = (command.kind,)
        return self._send_tracked_command_long(
            key,
            "land",
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            now_s,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

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

    def _send_tracked_command_long(
        self,
        key: tuple[object, ...],
        label: str,
        command_id: int,
        now_s: float,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> bool:
        if key in self._pending_acks:
            return False
        if self._skip_repeated(key, now_s):
            return False

        args = (
            self._target_system,
            self._target_component,
            command_id,
            0,
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7,
        )
        self.master.mav.command_long_send(*args)
        if self.config.command_ack_required:
            self._pending_acks[key] = _PendingCommandAck(
                key=key,
                command_id=command_id,
                label=label,
                command_long_args=args,
                last_sent_s=now_s,
            )
        return True

    def _skip_repeated(self, key: tuple[object, ...], now_s: float) -> bool:
        last_sent_s = self._last_non_velocity_sent_s.get(key)
        if (
            last_sent_s is not None
            and now_s - last_sent_s < self.config.command_repeat_interval_s
        ):
            return True
        self._last_non_velocity_sent_s[key] = now_s
        return False

    def _pending_key_for_command(self, command_id: int) -> tuple[object, ...] | None:
        for key, pending in self._pending_acks.items():
            if pending.command_id == command_id:
                return key
        return None

    def _message_type(self, message: Any) -> str:
        if hasattr(message, "get_type"):
            return str(message.get_type())
        return str(getattr(message, "message_type", "UNKNOWN"))


def _mav_result_name(result: int) -> str:
    for name, value in vars(mavutil.mavlink).items():
        if name.startswith("MAV_RESULT_") and value == result:
            return name
    return f"unknown({result})"

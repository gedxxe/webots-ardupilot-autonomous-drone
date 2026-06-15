from __future__ import annotations

from dataclasses import dataclass
from time import monotonic, sleep

from pymavlink import mavutil

from drone_autonomy.autonomy.mission import (
    GateAutonomyMission,
    MissionOutput,
    MissionPhase,
)
from drone_autonomy.mavlink.commands import MavlinkCommandAdapter
from drone_autonomy.mavlink.telemetry import CourseFrame, MavlinkTelemetryAdapter
from drone_autonomy.perception.synthetic import SyntheticGateProvider


@dataclass(frozen=True)
class AutonomyRuntimeConfig:
    """Configuration for the process-level autonomy loop.

    Defaults are intentionally conservative: `send_commands=False` means the
    runtime can be pointed at SITL to inspect decisions without moving the
    vehicle.
    """

    connection: str = "udp:127.0.0.1:14550"
    loop_hz: float = 20.0
    max_runtime_s: float = 180.0
    heartbeat_timeout_s: float = 30.0
    status_interval_s: float = 1.0
    detector: str = "none"
    send_commands: bool = False
    course_forward_x: float = 1.0
    course_forward_y: float = 0.0


@dataclass(frozen=True)
class AutonomyRuntimeResult:
    """Final status returned after the blocking runtime exits."""

    completed: bool
    final_phase: MissionPhase
    last_output: MissionOutput | None


class AutonomyRuntime:
    """Blocking process loop that wires MAVLink, perception, and mission logic.

    Blocking belongs here, not inside `GateAutonomyMission`. This keeps the
    mission unit-testable and lets SITL/hardware adapters evolve independently.
    """

    def __init__(self, config: AutonomyRuntimeConfig) -> None:
        self.config = config
        if self.config.loop_hz <= 0.0:
            raise ValueError("loop_hz must be positive")
        if self.config.detector not in {"none", "synthetic"}:
            raise ValueError("detector must be 'none' or 'synthetic'")

    def run(self) -> AutonomyRuntimeResult:
        """Run the complete SITL/hardware-facing autonomy process.

        High-level sequence:
        1. Connect to MAVLink and wait for heartbeat.
        2. Build adapters for telemetry, commands, and detector source.
        3. Repeatedly drain MAVLink, build a telemetry snapshot, run mission,
           and optionally send the returned command.
        """

        master = mavutil.mavlink_connection(self.config.connection)
        heartbeat = master.wait_heartbeat(timeout=self.config.heartbeat_timeout_s)
        if heartbeat is None:
            raise TimeoutError(f"No heartbeat from {self.config.connection}")

        command_mode = "send-commands" if self.config.send_commands else "dry-run"
        print(
            f"autonomy connection={self.config.connection} "
            f"detector={self.config.detector} "
            f"command_mode={command_mode} "
            f"loop_hz={self.config.loop_hz}"
        )

        mission = GateAutonomyMission()
        command_adapter = MavlinkCommandAdapter(master)
        telemetry_adapter = MavlinkTelemetryAdapter(
            CourseFrame(
                forward_x=self.config.course_forward_x,
                forward_y=self.config.course_forward_y,
            )
        )
        telemetry_adapter.update_message(heartbeat)
        synthetic_gate = SyntheticGateProvider() if self.config.detector == "synthetic" else None

        try:
            command_adapter.request_default_telemetry(rate_hz=self.config.loop_hz)
        except Exception as exc:
            print(f"warning: telemetry interval request failed: {exc}")

        start_s = monotonic()
        next_status_s = start_s
        last_output: MissionOutput | None = None

        while monotonic() - start_s <= self.config.max_runtime_s:
            loop_started_s = monotonic()
            # Drain all queued MAVLink messages before building the snapshot so
            # the mission always sees the freshest available fused telemetry.
            self._drain_mavlink(master, telemetry_adapter)

            detection = None
            if synthetic_gate is not None:
                # Synthetic perception is phase-aware and exists only to test
                # mission/MAVLink wiring before YOLO and Webots camera exist.
                detection = synthetic_gate.detect_for_phase(
                    loop_started_s,
                    mission.phase,
                    mission.gate_index,
                )

            telemetry = telemetry_adapter.snapshot(loop_started_s, detection)
            if telemetry is None:
                # Mission must not run until local-position telemetry exists;
                # otherwise altitude and forward distance would be fabricated.
                if loop_started_s >= next_status_s:
                    print("waiting for LOCAL_POSITION_NED telemetry")
                    next_status_s = loop_started_s + self.config.status_interval_s
                self._sleep_until_next_tick(loop_started_s)
                continue

            last_output = mission.update(telemetry)
            if self.config.send_commands:
                # This is the only point where mission output becomes a real
                # MAVLink command. Without `--send-commands`, runtime is dry-run.
                command_adapter.send(last_output.command, now_s=loop_started_s)

            if loop_started_s >= next_status_s:
                sent = "sent" if self.config.send_commands else "dry-run"
                print(
                    f"{sent} phase={last_output.phase.value} "
                    f"gate={last_output.gate_index + 1} "
                    f"cmd={last_output.command.kind.value} "
                    f"detail={last_output.detail}"
                )
                next_status_s = loop_started_s + self.config.status_interval_s

            if last_output.phase == MissionPhase.COMPLETE:
                return AutonomyRuntimeResult(True, mission.phase, last_output)
            if last_output.phase == MissionPhase.FAILSAFE:
                return AutonomyRuntimeResult(False, mission.phase, last_output)

            self._sleep_until_next_tick(loop_started_s)

        return AutonomyRuntimeResult(False, mission.phase, last_output)

    def _drain_mavlink(
        self,
        master: object,
        telemetry_adapter: MavlinkTelemetryAdapter,
    ) -> None:
        """Read all currently available MAVLink messages without blocking."""

        while True:
            message = master.recv_match(blocking=False)
            if message is None:
                return
            telemetry_adapter.update_message(message)

    def _sleep_until_next_tick(self, loop_started_s: float) -> None:
        """Keep the loop near the configured rate without hiding slow ticks."""

        period_s = 1.0 / self.config.loop_hz
        remaining_s = period_s - (monotonic() - loop_started_s)
        if remaining_s > 0.0:
            sleep(remaining_s)

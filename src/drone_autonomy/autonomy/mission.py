from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum

from drone_autonomy.autonomy.commands import CommandKind, VehicleCommand
from drone_autonomy.control.altitude import AltitudeHoldConfig, AltitudeHoldController
from drone_autonomy.control.visual_servo import (
    GateVisualServoController,
    ServoOutput,
    VisualServoConfig,
)
from drone_autonomy.perception.detections import GateDetection


class MissionPhase(str, Enum):
    INIT = "init"
    TAKEOFF = "takeoff"
    SEEK_GATE = "seek_gate"
    CENTER_GATE = "center_gate"
    PASS_GATE = "pass_gate"
    NEXT_GATE_ACQUIRE = "next_gate_acquire"
    BRAKE = "brake"
    FINAL_EXIT = "final_exit"
    LAND = "land"
    COMPLETE = "complete"
    FAILSAFE = "failsafe"


@dataclass(frozen=True)
class GateMissionConfig:
    """Tunable mission policy.

    Distances are local forward distances along the course, not altitude. The
    final exit distance is measured from the moment the last gate is considered
    passed, so `2.0 m` means "fly forward 2 meters after the last gate".
    """

    gate_count: int = 2
    guided_mode_name: str = "GUIDED"
    takeoff_altitude_m: float = 1.0
    takeoff_altitude_tolerance_m: float = 0.15
    takeoff_timeout_s: float = 20.0
    max_detection_age_s: float = 0.35
    required_detection_ticks: int = 2
    required_aligned_ticks: int = 4
    seek_yaw_rate_rad_s: float = 0.30
    gate_pass_distance_m: float = 1.25
    gate_pass_speed_m_s: float = 1.20
    next_gate_acquire_speed_m_s: float = 2.50
    next_gate_acquire_max_distance_m: float = 6.00
    next_gate_acquire_timeout_s: float = 6.00
    brake_settle_s: float = 1.00
    final_exit_distance_m: float = 2.00
    final_exit_speed_m_s: float = 1.50
    min_centering_altitude_m: float = 0.65
    max_centering_altitude_m: float = 2.00
    altitude_hold_enabled: bool = True
    altitude_hold_deadband_m: float = 0.08
    altitude_hold_kp: float = 0.55
    altitude_hold_max_climb_m_s: float = 0.35
    altitude_hold_max_descent_m_s: float = 0.25
    landing_complete_altitude_m: float = 0.15
    mission_timeout_s: float = 180.0

    def __post_init__(self) -> None:
        if self.gate_count < 1:
            raise ValueError("gate_count must be at least 1")
        if self.required_detection_ticks < 1:
            raise ValueError("required_detection_ticks must be at least 1")
        if self.required_aligned_ticks < 1:
            raise ValueError("required_aligned_ticks must be at least 1")
        if self.final_exit_distance_m < 0.0:
            raise ValueError("final_exit_distance_m must be non-negative")
        if self.next_gate_acquire_max_distance_m < 0.0:
            raise ValueError("next_gate_acquire_max_distance_m must be non-negative")
        if self.next_gate_acquire_timeout_s < 0.0:
            raise ValueError("next_gate_acquire_timeout_s must be non-negative")


@dataclass(frozen=True)
class MissionTelemetry:
    """Single-tick input snapshot for the mission state machine.

    The mission is pure and non-blocking: callers provide the latest telemetry
    and detection each tick. `altitude_m` and `forward_position_m` should come
    from fused local-position telemetry when possible. Do not pass raw GPS,
    rangefinder, or optical-flow samples here unless an adapter has already
    fused and validated them.
    """

    now_s: float
    altitude_m: float
    forward_position_m: float
    mode: str
    armed: bool
    landed: bool = False
    gate_detection: GateDetection | None = None


@dataclass(frozen=True)
class MissionOutput:
    """Single-tick decision returned by the mission state machine."""

    phase: MissionPhase
    command: VehicleCommand
    gate_index: int
    detail: str
    servo: ServoOutput | None = None


class GateAutonomyMission:
    """Deterministic, non-blocking state machine for the two-gate task.

    This class performs no I/O, sleeps, MAVLink reads, camera reads, or model
    inference. A runtime loop should call `update()` at a fixed rate with fresh
    telemetry and the latest gate detection.
    """

    def __init__(
        self,
        config: GateMissionConfig | None = None,
        visual_servo: GateVisualServoController | None = None,
    ) -> None:
        self.config = config or GateMissionConfig()
        self.visual_servo = visual_servo or GateVisualServoController(VisualServoConfig())
        self.altitude_hold = AltitudeHoldController(
            AltitudeHoldConfig(
                target_altitude_m=self.config.takeoff_altitude_m,
                deadband_m=self.config.altitude_hold_deadband_m,
                kp=self.config.altitude_hold_kp,
                max_climb_speed_m_s=self.config.altitude_hold_max_climb_m_s,
                max_descent_speed_m_s=self.config.altitude_hold_max_descent_m_s,
            )
        )
        self.phase = MissionPhase.INIT
        self.gate_index = 0
        self.phase_started_s: float | None = None
        self.phase_started_forward_m = 0.0
        self._mission_started_s: float | None = None
        self._detection_ticks = 0
        self._aligned_ticks = 0
        self._brake_next_phase = MissionPhase.SEEK_GATE

    def update(self, telemetry: MissionTelemetry) -> MissionOutput:
        """Advance the mission by one tick and return one command.

        The method is intentionally side-effect-only inside this object. It
        never waits for telemetry, blocks on perception, or sends commands.
        """

        if self._mission_started_s is None:
            self._mission_started_s = telemetry.now_s
        if self.phase_started_s is None:
            self._enter(self.phase, telemetry)

        if (
            self.phase not in {MissionPhase.COMPLETE, MissionPhase.FAILSAFE}
            and telemetry.now_s - self._mission_started_s > self.config.mission_timeout_s
        ):
            self._enter(MissionPhase.FAILSAFE, telemetry)

        if self.phase == MissionPhase.INIT:
            return self._run_init(telemetry)
        if self.phase == MissionPhase.TAKEOFF:
            return self._run_takeoff(telemetry)
        if self.phase == MissionPhase.SEEK_GATE:
            return self._run_seek_gate(telemetry)
        if self.phase == MissionPhase.CENTER_GATE:
            return self._run_center_gate(telemetry)
        if self.phase == MissionPhase.PASS_GATE:
            return self._run_pass_gate(telemetry)
        if self.phase == MissionPhase.NEXT_GATE_ACQUIRE:
            return self._run_next_gate_acquire(telemetry)
        if self.phase == MissionPhase.BRAKE:
            return self._run_brake(telemetry)
        if self.phase == MissionPhase.FINAL_EXIT:
            return self._run_final_exit(telemetry)
        if self.phase == MissionPhase.LAND:
            return self._run_land(telemetry)
        if self.phase == MissionPhase.COMPLETE:
            return MissionOutput(
                phase=self.phase,
                command=VehicleCommand.none("mission complete"),
                gate_index=self.gate_index,
                detail="mission complete",
            )
        return MissionOutput(
            phase=self.phase,
            command=VehicleCommand.land("failsafe landing"),
            gate_index=self.gate_index,
            detail="failsafe landing",
        )

    def _run_init(self, telemetry: MissionTelemetry) -> MissionOutput:
        if telemetry.mode != self.config.guided_mode_name:
            return self._output(
                VehicleCommand.set_mode(
                    self.config.guided_mode_name,
                    "switch to guided before autonomy",
                ),
                "waiting for guided mode",
            )
        if not telemetry.armed:
            return self._output(VehicleCommand.arm_vehicle("arm before takeoff"), "arming")

        self._enter(MissionPhase.TAKEOFF, telemetry)
        return self._run_takeoff(telemetry)

    def _run_takeoff(self, telemetry: MissionTelemetry) -> MissionOutput:
        if telemetry.altitude_m >= (
            self.config.takeoff_altitude_m - self.config.takeoff_altitude_tolerance_m
        ):
            self._enter(MissionPhase.SEEK_GATE, telemetry)
            return self._run_seek_gate(telemetry)

        if self._phase_elapsed_s(telemetry) > self.config.takeoff_timeout_s:
            self._enter(MissionPhase.FAILSAFE, telemetry)
            return self.update(telemetry)

        return self._output(
            VehicleCommand.takeoff(
                self.config.takeoff_altitude_m,
                "climb to mission altitude",
            ),
            "taking off",
        )

    def _run_seek_gate(self, telemetry: MissionTelemetry) -> MissionOutput:
        detection = self._fresh_detection(telemetry)
        if detection is not None:
            self._detection_ticks += 1
        else:
            self._detection_ticks = 0

        if self._detection_ticks >= self.config.required_detection_ticks:
            self._enter(MissionPhase.CENTER_GATE, telemetry)
            return self._run_center_gate(telemetry)

        return self._output(
            self._velocity_with_altitude_hold(
                telemetry,
                yaw_rate_rad_s=self.config.seek_yaw_rate_rad_s,
                reason="searching for gate",
            ),
            f"seeking gate {self.gate_index + 1}",
        )

    def _run_center_gate(self, telemetry: MissionTelemetry) -> MissionOutput:
        detection = self._fresh_detection(telemetry)
        if detection is None:
            self._enter(MissionPhase.SEEK_GATE, telemetry)
            return self._run_seek_gate(telemetry)

        servo = self.visual_servo.update(detection)
        guarded_command = self._apply_centering_altitude_guard(
            servo.command,
            telemetry,
        )
        if guarded_command != servo.command:
            servo = replace(servo, command=guarded_command)
        if servo.pass_ready:
            self._aligned_ticks += 1
        else:
            self._aligned_ticks = 0

        if self._aligned_ticks >= self.config.required_aligned_ticks:
            self._enter(MissionPhase.PASS_GATE, telemetry)
            return self._run_pass_gate(telemetry)

        return self._output(
            servo.command,
            f"centering gate {self.gate_index + 1}",
            servo=servo,
        )

    def _run_pass_gate(self, telemetry: MissionTelemetry) -> MissionOutput:
        if self._phase_forward_delta_m(telemetry) >= self.config.gate_pass_distance_m:
            if self.gate_index + 1 >= self.config.gate_count:
                self._enter(MissionPhase.FINAL_EXIT, telemetry)
                return self._run_final_exit(telemetry)

            self.gate_index += 1
            self._enter(MissionPhase.NEXT_GATE_ACQUIRE, telemetry)
            return self._run_next_gate_acquire(telemetry)

        return self._output(
            self._velocity_with_altitude_hold(
                telemetry,
                body_vx_m_s=self.config.gate_pass_speed_m_s,
                reason="committed gate pass",
            ),
            f"passing gate {self.gate_index + 1}",
        )

    def _run_next_gate_acquire(self, telemetry: MissionTelemetry) -> MissionOutput:
        """Move forward while actively looking for the next gate.

        This replaces blind sprinting. The drone keeps the camera pipeline active
        and commits to centering as soon as the next gate is seen consistently.
        """

        detection = self._fresh_detection(telemetry)
        if detection is not None:
            self._detection_ticks += 1
        else:
            self._detection_ticks = 0

        if self._detection_ticks >= self.config.required_detection_ticks:
            self._brake_next_phase = MissionPhase.CENTER_GATE
            self._enter(MissionPhase.BRAKE, telemetry)
            return self._run_brake(telemetry)

        if (
            self._phase_forward_delta_m(telemetry)
            >= self.config.next_gate_acquire_max_distance_m
            or self._phase_elapsed_s(telemetry) >= self.config.next_gate_acquire_timeout_s
        ):
            self._enter(MissionPhase.SEEK_GATE, telemetry)
            return self._run_seek_gate(telemetry)

        return self._output(
            self._velocity_with_altitude_hold(
                telemetry,
                body_vx_m_s=self.config.next_gate_acquire_speed_m_s,
                reason="adaptive next-gate acquire",
            ),
            f"acquiring gate {self.gate_index + 1}",
        )

    def _run_brake(self, telemetry: MissionTelemetry) -> MissionOutput:
        if self._phase_elapsed_s(telemetry) >= self.config.brake_settle_s:
            next_phase = self._brake_next_phase
            self._brake_next_phase = MissionPhase.SEEK_GATE
            self._enter(next_phase, telemetry)
            if next_phase == MissionPhase.CENTER_GATE:
                return self._run_center_gate(telemetry)
            return self._run_seek_gate(telemetry)

        return self._output(
            self._velocity_with_altitude_hold(
                telemetry,
                reason="brake and settle",
            ),
            "braking",
        )

    def _run_final_exit(self, telemetry: MissionTelemetry) -> MissionOutput:
        if self._phase_forward_delta_m(telemetry) >= self.config.final_exit_distance_m:
            self._enter(MissionPhase.LAND, telemetry)
            return self._run_land(telemetry)

        return self._output(
            self._velocity_with_altitude_hold(
                telemetry,
                body_vx_m_s=self.config.final_exit_speed_m_s,
                reason="clear final gate before landing",
            ),
            "final forward exit distance",
        )

    def _run_land(self, telemetry: MissionTelemetry) -> MissionOutput:
        if telemetry.landed or telemetry.altitude_m <= self.config.landing_complete_altitude_m:
            self._enter(MissionPhase.COMPLETE, telemetry)
            return self.update(telemetry)

        return self._output(VehicleCommand.land("land after final gate"), "landing")

    def _fresh_detection(self, telemetry: MissionTelemetry) -> GateDetection | None:
        detection = telemetry.gate_detection
        if detection is None:
            return None
        if detection.confidence < self.visual_servo.config.min_confidence:
            return None
        if not detection.is_fresh(telemetry.now_s, self.config.max_detection_age_s):
            return None
        return detection

    def _apply_centering_altitude_guard(
        self,
        command: VehicleCommand,
        telemetry: MissionTelemetry,
    ) -> VehicleCommand:
        if command.kind != CommandKind.BODY_VELOCITY:
            return command

        guarded_vz = command.body_vz_m_s
        if (
            telemetry.altitude_m <= self.config.min_centering_altitude_m
            and guarded_vz > 0.0
        ):
            guarded_vz = 0.0
        if (
            telemetry.altitude_m >= self.config.max_centering_altitude_m
            and guarded_vz < 0.0
        ):
            guarded_vz = 0.0

        if guarded_vz == command.body_vz_m_s:
            return command
        return replace(
            command,
            body_vz_m_s=guarded_vz,
            reason=f"{command.reason}; altitude guard",
        )

    def _velocity_with_altitude_hold(
        self,
        telemetry: MissionTelemetry,
        *,
        body_vx_m_s: float = 0.0,
        body_vy_m_s: float = 0.0,
        yaw_rate_rad_s: float = 0.0,
        reason: str = "",
    ) -> VehicleCommand:
        body_vz_m_s = 0.0
        altitude_reason = ""
        if self.config.altitude_hold_enabled:
            body_vz_m_s = self.altitude_hold.body_vz_for_altitude(telemetry.altitude_m)
            if body_vz_m_s != 0.0:
                altitude_reason = "; altitude hold"

        return VehicleCommand.body_velocity(
            body_vx_m_s=body_vx_m_s,
            body_vy_m_s=body_vy_m_s,
            body_vz_m_s=body_vz_m_s,
            yaw_rate_rad_s=yaw_rate_rad_s,
            reason=f"{reason}{altitude_reason}",
        )

    def _enter(self, phase: MissionPhase, telemetry: MissionTelemetry) -> None:
        # Phase-distance checks use the forward position at entry as the zero
        # point. This is why final exit is a forward-distance check, not a
        # height/altitude check.
        self.phase = phase
        self.phase_started_s = telemetry.now_s
        self.phase_started_forward_m = telemetry.forward_position_m
        self._detection_ticks = 0
        self._aligned_ticks = 0
        if phase in {MissionPhase.SEEK_GATE, MissionPhase.CENTER_GATE}:
            self.visual_servo.reset()

    def _phase_elapsed_s(self, telemetry: MissionTelemetry) -> float:
        if self.phase_started_s is None:
            return 0.0
        return telemetry.now_s - self.phase_started_s

    def _phase_forward_delta_m(self, telemetry: MissionTelemetry) -> float:
        return max(0.0, telemetry.forward_position_m - self.phase_started_forward_m)

    def _output(
        self,
        command: VehicleCommand,
        detail: str,
        servo: ServoOutput | None = None,
    ) -> MissionOutput:
        if command.kind == CommandKind.NONE:
            detail = detail or "no command"
        return MissionOutput(
            phase=self.phase,
            command=command,
            gate_index=self.gate_index,
            detail=detail,
            servo=servo,
        )

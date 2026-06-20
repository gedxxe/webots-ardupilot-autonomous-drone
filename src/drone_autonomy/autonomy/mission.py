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

    Takeoff is deliberately ArduPilot-managed: the mission sends a
    `MAV_CMD_NAV_TAKEOFF` target and waits for fused telemetry to settle. Do not
    add a companion-side body-z takeoff controller unless SITL logs prove the
    ArduPilot takeoff command cannot hold the requested altitude.
    """

    gate_count: int = 2
    guided_mode_name: str = "GUIDED"
    takeoff_altitude_m: float = 1.0
    takeoff_settle_tolerance_m: float = 0.06
    takeoff_required_stable_ticks: int = 8
    takeoff_timeout_s: float = 20.0
    max_detection_age_s: float = 0.75
    required_detection_ticks: int = 2
    center_dwell_s: float = 5.00
    center_clearance_required_s: float = 0.30
    center_lost_detection_grace_ticks: int = 10
    seek_yaw_rate_rad_s: float = 0.30
    gate_pass_distance_m: float = 1.25
    gate_pass_speed_m_s: float = 1.20
    next_gate_acquire_speed_m_s: float = 1.50
    next_gate_acquire_min_clear_distance_m: float = 2.00
    next_gate_acquire_min_area_ratio: float = 0.015
    gate_ready_area_ratio: float = 0.060
    next_gate_acquire_max_distance_m: float = 6.00
    next_gate_acquire_timeout_s: float = 6.00
    brake_settle_s: float = 1.00
    brake_ramp_s: float = 0.70
    brake_altitude_hold_enabled: bool = False
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
        if self.max_detection_age_s <= 0.0:
            raise ValueError("max_detection_age_s must be positive")
        if self.required_detection_ticks < 1:
            raise ValueError("required_detection_ticks must be at least 1")
        if self.center_dwell_s < 0.0:
            raise ValueError("center_dwell_s must be non-negative")
        if self.center_clearance_required_s < 0.0:
            raise ValueError("center_clearance_required_s must be non-negative")
        if self.center_lost_detection_grace_ticks < 0:
            raise ValueError("center_lost_detection_grace_ticks must be non-negative")
        if self.brake_settle_s < 0.0:
            raise ValueError("brake_settle_s must be non-negative")
        if self.brake_ramp_s < 0.0:
            raise ValueError("brake_ramp_s must be non-negative")
        if self.brake_ramp_s > self.brake_settle_s and self.brake_settle_s > 0.0:
            raise ValueError("brake_ramp_s must be <= brake_settle_s")
        if self.takeoff_altitude_m <= 0.0:
            raise ValueError("takeoff_altitude_m must be positive")
        if self.takeoff_settle_tolerance_m < 0.0:
            raise ValueError("takeoff_settle_tolerance_m must be non-negative")
        if self.takeoff_required_stable_ticks < 1:
            raise ValueError("takeoff_required_stable_ticks must be at least 1")
        if self.final_exit_distance_m < 0.0:
            raise ValueError("final_exit_distance_m must be non-negative")
        if self.next_gate_acquire_max_distance_m < 0.0:
            raise ValueError("next_gate_acquire_max_distance_m must be non-negative")
        if self.next_gate_acquire_min_clear_distance_m < 0.0:
            raise ValueError(
                "next_gate_acquire_min_clear_distance_m must be non-negative"
            )
        if self.next_gate_acquire_min_area_ratio < 0.0:
            raise ValueError("next_gate_acquire_min_area_ratio must be non-negative")
        if self.gate_ready_area_ratio < 0.0:
            raise ValueError("gate_ready_area_ratio must be non-negative")
        if (
            self.gate_ready_area_ratio > 0.0
            and self.next_gate_acquire_min_area_ratio > self.gate_ready_area_ratio
        ):
            raise ValueError(
                "next_gate_acquire_min_area_ratio must be <= gate_ready_area_ratio"
            )
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
        self._lost_detection_ticks = 0
        self._clearance_started_s: float | None = None
        self._takeoff_stable_ticks = 0
        self._brake_next_phase = MissionPhase.SEEK_GATE
        self._brake_entry_speed_m_s = 0.0

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
        altitude_error_m = self.config.takeoff_altitude_m - telemetry.altitude_m
        in_settle_band = abs(altitude_error_m) <= self.config.takeoff_settle_tolerance_m
        if in_settle_band and not telemetry.landed:
            self._takeoff_stable_ticks += 1
        else:
            self._takeoff_stable_ticks = 0

        if self._takeoff_stable_ticks >= self.config.takeoff_required_stable_ticks:
            self._enter(MissionPhase.SEEK_GATE, telemetry)
            return self._run_seek_gate(telemetry)

        if self._phase_elapsed_s(telemetry) > self.config.takeoff_timeout_s:
            self._enter(MissionPhase.FAILSAFE, telemetry)
            return self.update(telemetry)

        return self._output(
            VehicleCommand.takeoff(
                self.config.takeoff_altitude_m,
                "ArduPilot-managed takeoff to mission altitude",
            ),
            (
                "waiting for ArduPilot takeoff "
                f"stable={self._takeoff_stable_ticks}/"
                f"{self.config.takeoff_required_stable_ticks}"
            ),
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
            self._lost_detection_ticks += 1
            self._clearance_started_s = None
            if self._lost_detection_ticks <= self.config.center_lost_detection_grace_ticks:
                return self._output(
                    self._velocity_with_altitude_hold(
                        telemetry,
                        reason=(
                            "brief gate detection loss "
                            f"{self._lost_detection_ticks}/"
                            f"{self.config.center_lost_detection_grace_ticks}"
                        ),
                    ),
                    f"holding gate {self.gate_index + 1} target loss",
                )
            self._enter(MissionPhase.SEEK_GATE, telemetry)
            return self._run_seek_gate(telemetry)
        self._lost_detection_ticks = 0

        servo = self.visual_servo.update(detection)
        guarded_command = self._apply_centering_altitude_guard(
            servo.command,
            telemetry,
        )
        if guarded_command != servo.command:
            servo = replace(servo, command=guarded_command)

        if servo.clearance_ready:
            if self._clearance_started_s is None:
                self._clearance_started_s = telemetry.now_s
        else:
            self._clearance_started_s = None

        dwell_s = self._phase_elapsed_s(telemetry)
        clearance_s = 0.0
        if self._clearance_started_s is not None:
            clearance_s = telemetry.now_s - self._clearance_started_s

        area_ratio = self._detection_area_ratio(detection)
        area_ready = self._gate_area_ready(area_ratio)
        if (
            self._clearance_started_s is not None
            and dwell_s >= self.config.center_dwell_s
            and clearance_s >= self.config.center_clearance_required_s
            and area_ready
        ):
            self._enter(MissionPhase.PASS_GATE, telemetry)
            return self._run_pass_gate(telemetry)

        area_detail = ""
        if self.config.gate_ready_area_ratio > 0.0:
            area_detail = (
                f" area={min(area_ratio, self.config.gate_ready_area_ratio):0.3f}/"
                f"{self.config.gate_ready_area_ratio:0.3f}"
            )
        return self._output(
            servo.command,
            (
                f"centering gate {self.gate_index + 1} "
                f"dwell={min(dwell_s, self.config.center_dwell_s):0.1f}/"
                f"{self.config.center_dwell_s:0.1f}s "
                f"clearance={min(clearance_s, self.config.center_clearance_required_s):0.1f}/"
                f"{self.config.center_clearance_required_s:0.1f}s"
                f"{area_detail}"
            ),
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

        ignore_detection = (
            self._phase_forward_delta_m(telemetry)
            < self.config.next_gate_acquire_min_clear_distance_m
        )
        detection = None if ignore_detection else self._fresh_detection(telemetry)
        area_ratio = self._detection_area_ratio(detection) if detection is not None else 0.0
        if detection is not None and area_ratio < self.config.next_gate_acquire_min_area_ratio:
            detection = None

        ready_detection = (
            detection is not None and self._gate_area_ready(area_ratio)
        )
        if ready_detection:
            self._detection_ticks += 1
        else:
            self._detection_ticks = 0

        if self._detection_ticks >= self.config.required_detection_ticks:
            self._enter_brake(
                telemetry,
                next_phase=MissionPhase.CENTER_GATE,
                entry_speed_m_s=self.config.next_gate_acquire_speed_m_s,
            )
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
            self._next_gate_acquire_detail(ignore_detection, detection, area_ratio),
        )

    def _run_brake(self, telemetry: MissionTelemetry) -> MissionOutput:
        if self._phase_elapsed_s(telemetry) >= self.config.brake_settle_s:
            next_phase = self._brake_next_phase
            self._brake_next_phase = MissionPhase.SEEK_GATE
            self._enter(next_phase, telemetry)
            if next_phase == MissionPhase.CENTER_GATE:
                return self._run_center_gate(telemetry)
            if next_phase == MissionPhase.LAND:
                return self._run_land(telemetry)
            return self._run_seek_gate(telemetry)

        return self._output(
            self._velocity_with_altitude_hold(
                telemetry,
                body_vx_m_s=self._brake_forward_speed_m_s(telemetry),
                altitude_hold_enabled=self.config.brake_altitude_hold_enabled,
                reason="brake and settle",
            ),
            (
                "braking before landing"
                if self._brake_next_phase == MissionPhase.LAND
                else "braking"
            ),
        )

    def _run_final_exit(self, telemetry: MissionTelemetry) -> MissionOutput:
        if self._phase_forward_delta_m(telemetry) >= self.config.final_exit_distance_m:
            self._enter_brake(
                telemetry,
                next_phase=MissionPhase.LAND,
                entry_speed_m_s=self.config.final_exit_speed_m_s,
            )
            return self._run_brake(telemetry)

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

    def _detection_area_ratio(self, detection: GateDetection) -> float:
        return detection.bbox.normalized_area(self.visual_servo.config.frame_shape())

    def _gate_area_ready(self, area_ratio: float) -> bool:
        return (
            self.config.gate_ready_area_ratio <= 0.0
            or area_ratio >= self.config.gate_ready_area_ratio
        )

    def _next_gate_acquire_detail(
        self,
        ignore_detection: bool,
        detection: GateDetection | None,
        area_ratio: float,
    ) -> str:
        gate_number = self.gate_index + 1
        if ignore_detection:
            return f"clearing previous gate before acquiring gate {gate_number}"
        if detection is None:
            return f"acquiring gate {gate_number}"
        if not self._gate_area_ready(area_ratio):
            return (
                f"approaching gate {gate_number} "
                f"area={area_ratio:0.3f}/"
                f"{self.config.gate_ready_area_ratio:0.3f}"
            )
        return (
            f"acquiring gate {gate_number} "
            f"ready area={area_ratio:0.3f}/"
            f"{self.config.gate_ready_area_ratio:0.3f} "
            f"ticks={self._detection_ticks}/"
            f"{self.config.required_detection_ticks}"
        )

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
        altitude_hold_enabled: bool | None = None,
        reason: str = "",
    ) -> VehicleCommand:
        body_vz_m_s = 0.0
        altitude_reason = ""
        use_altitude_hold = (
            self.config.altitude_hold_enabled
            if altitude_hold_enabled is None
            else altitude_hold_enabled
        )
        if use_altitude_hold:
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

    def _enter_brake(
        self,
        telemetry: MissionTelemetry,
        *,
        next_phase: MissionPhase,
        entry_speed_m_s: float,
    ) -> None:
        """Enter brake with the previous forward speed for a smooth ramp-down."""

        self._brake_next_phase = next_phase
        self._brake_entry_speed_m_s = max(0.0, entry_speed_m_s)
        self._enter(MissionPhase.BRAKE, telemetry)

    def _brake_forward_speed_m_s(self, telemetry: MissionTelemetry) -> float:
        """Ramp forward velocity down instead of stepping immediately to zero."""

        if self.config.brake_ramp_s <= 0.0 or self._brake_entry_speed_m_s <= 0.0:
            return 0.0
        elapsed_s = self._phase_elapsed_s(telemetry)
        if elapsed_s >= self.config.brake_ramp_s:
            return 0.0
        remaining_ratio = 1.0 - (elapsed_s / self.config.brake_ramp_s)
        return self._brake_entry_speed_m_s * remaining_ratio

    def _enter(self, phase: MissionPhase, telemetry: MissionTelemetry) -> None:
        # Phase-distance checks use the forward position at entry as the zero
        # point. This is why final exit is a forward-distance check, not a
        # height/altitude check.
        self.phase = phase
        self.phase_started_s = telemetry.now_s
        self.phase_started_forward_m = telemetry.forward_position_m
        self._detection_ticks = 0
        self._lost_detection_ticks = 0
        self._clearance_started_s = None
        if phase == MissionPhase.TAKEOFF:
            self._takeoff_stable_ticks = 0
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

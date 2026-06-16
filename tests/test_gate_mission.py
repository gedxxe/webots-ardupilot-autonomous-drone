from drone_autonomy.autonomy.commands import CommandKind
from drone_autonomy.autonomy.mission import (
    GateAutonomyMission,
    GateMissionConfig,
    MissionPhase,
    MissionTelemetry,
)
from drone_autonomy.control.visual_servo import (
    GateVisualServoController,
    VisualServoConfig,
)
from drone_autonomy.perception.detections import BoundingBox, GateDetection


def centered_gate(now_s: float) -> GateDetection:
    return GateDetection(
        bbox=BoundingBox(30, 25, 70, 75),
        confidence=0.95,
        observed_at_s=now_s,
    )


def build_mission() -> GateAutonomyMission:
    servo = GateVisualServoController(
        VisualServoConfig(
            frame_width_px=100,
            frame_height_px=100,
            filter_alpha=1.0,
            target_area_ratio=0.10,
        )
    )
    return GateAutonomyMission(
        GateMissionConfig(
            takeoff_required_stable_ticks=1,
            required_detection_ticks=1,
            required_aligned_ticks=1,
            gate_pass_distance_m=1.0,
            next_gate_acquire_speed_m_s=2.5,
            next_gate_acquire_max_distance_m=4.0,
            next_gate_acquire_timeout_s=4.0,
            brake_settle_s=1.0,
            final_exit_distance_m=2.0,
        ),
        servo,
    )


def telemetry(
    now_s: float,
    *,
    altitude_m: float = 1.0,
    forward_position_m: float = 0.0,
    mode: str = "GUIDED",
    armed: bool = True,
    landed: bool = False,
    gate_detection: GateDetection | None = None,
) -> MissionTelemetry:
    return MissionTelemetry(
        now_s=now_s,
        altitude_m=altitude_m,
        forward_position_m=forward_position_m,
        mode=mode,
        armed=armed,
        landed=landed,
        gate_detection=gate_detection,
    )


def test_default_takeoff_policy_is_ardupilot_managed() -> None:
    config = GateMissionConfig()

    assert config.takeoff_altitude_m == 1.0
    assert config.takeoff_settle_tolerance_m == 0.06
    assert config.takeoff_required_stable_ticks == 8


def test_mission_requests_guided_then_arm_then_takeoff() -> None:
    mission = build_mission()

    output = mission.update(telemetry(0.0, altitude_m=0.0, mode="LOITER", armed=False))
    assert output.command.kind == CommandKind.SET_MODE

    output = mission.update(telemetry(1.0, altitude_m=0.0, mode="GUIDED", armed=False))
    assert output.command.kind == CommandKind.ARM

    output = mission.update(telemetry(2.0, altitude_m=0.0, mode="GUIDED", armed=True))
    assert output.phase == MissionPhase.TAKEOFF
    assert output.command.kind == CommandKind.TAKEOFF
    assert output.command.altitude_m == mission.config.takeoff_altitude_m


def test_takeoff_does_not_seek_while_landed_even_if_altitude_is_near_target() -> None:
    mission = GateAutonomyMission(
        GateMissionConfig(
            takeoff_required_stable_ticks=1,
            takeoff_settle_tolerance_m=0.06,
        )
    )

    output = mission.update(telemetry(0.0, altitude_m=1.0, landed=True))

    assert output.phase == MissionPhase.TAKEOFF
    assert output.command.kind == CommandKind.TAKEOFF
    assert output.command.altitude_m == 1.0


def test_takeoff_waits_for_stable_ardupilot_altitude_before_seek() -> None:
    mission = GateAutonomyMission(
        GateMissionConfig(
            takeoff_required_stable_ticks=2,
            takeoff_settle_tolerance_m=0.05,
        )
    )

    output = mission.update(telemetry(0.0, altitude_m=0.20, landed=False))
    assert output.phase == MissionPhase.TAKEOFF
    assert output.command.kind == CommandKind.TAKEOFF
    assert output.command.altitude_m == 1.0

    output = mission.update(telemetry(1.0, altitude_m=0.96))
    assert output.phase == MissionPhase.TAKEOFF
    assert output.command.kind == CommandKind.TAKEOFF

    output = mission.update(telemetry(1.1, altitude_m=1.01))
    assert output.phase == MissionPhase.SEEK_GATE


def test_takeoff_overshoot_does_not_emit_companion_vertical_correction() -> None:
    mission = GateAutonomyMission(
        GateMissionConfig(
            takeoff_required_stable_ticks=1,
            takeoff_settle_tolerance_m=0.05,
        )
    )

    output = mission.update(telemetry(0.0, altitude_m=3.0))

    assert output.phase == MissionPhase.TAKEOFF
    assert output.command.kind == CommandKind.TAKEOFF
    assert output.command.altitude_m == 1.0


def test_nominal_two_gate_sequence_reaches_landing() -> None:
    mission = build_mission()

    output = mission.update(telemetry(0.0, altitude_m=1.0))
    assert output.phase == MissionPhase.SEEK_GATE

    output = mission.update(telemetry(1.0, gate_detection=centered_gate(1.0)))
    assert output.phase == MissionPhase.PASS_GATE
    assert output.command.kind == CommandKind.BODY_VELOCITY

    output = mission.update(telemetry(2.0, forward_position_m=1.2))
    assert output.phase == MissionPhase.NEXT_GATE_ACQUIRE
    assert output.command.body_vx_m_s == 2.5

    output = mission.update(
        telemetry(3.0, forward_position_m=2.0, gate_detection=centered_gate(3.0))
    )
    assert output.phase == MissionPhase.BRAKE
    assert output.command.body_vx_m_s == 0.0
    assert output.gate_index == 1

    output = mission.update(
        telemetry(4.1, forward_position_m=2.0, gate_detection=centered_gate(4.1))
    )
    assert output.phase == MissionPhase.PASS_GATE

    output = mission.update(telemetry(5.2, forward_position_m=3.2))
    assert output.phase == MissionPhase.FINAL_EXIT

    output = mission.update(telemetry(6.2, forward_position_m=5.3))
    assert output.phase == MissionPhase.LAND
    assert output.command.kind == CommandKind.LAND


def test_lost_detection_while_centering_returns_to_seek() -> None:
    mission = build_mission()
    mission.update(telemetry(0.0, altitude_m=1.0))

    output = mission.update(
        telemetry(
            1.0,
            gate_detection=GateDetection(
                bbox=BoundingBox(5, 25, 45, 75),
                confidence=0.95,
                observed_at_s=1.0,
            ),
        )
    )
    assert output.phase == MissionPhase.CENTER_GATE

    output = mission.update(telemetry(1.2))
    assert output.phase == MissionPhase.SEEK_GATE


def test_centering_altitude_guard_blocks_downward_command_near_floor() -> None:
    mission = build_mission()
    mission.update(telemetry(0.0, altitude_m=1.0))

    output = mission.update(
        telemetry(
            1.0,
            altitude_m=0.50,
            gate_detection=GateDetection(
                bbox=BoundingBox(30, 60, 70, 90),
                confidence=0.95,
                observed_at_s=1.0,
            ),
        )
    )

    assert output.phase == MissionPhase.CENTER_GATE
    assert output.command.kind == CommandKind.BODY_VELOCITY
    assert output.command.body_vz_m_s == 0.0


def test_final_exit_uses_forward_distance_not_altitude() -> None:
    mission = build_mission()
    mission.update(telemetry(0.0, altitude_m=1.0))
    mission.update(telemetry(1.0, gate_detection=centered_gate(1.0)))
    mission.update(
        telemetry(2.0, forward_position_m=1.2, gate_detection=centered_gate(2.0))
    )
    mission.update(
        telemetry(3.1, forward_position_m=1.2, gate_detection=centered_gate(3.1))
    )
    output = mission.update(telemetry(4.2, forward_position_m=2.4))
    assert output.phase == MissionPhase.FINAL_EXIT

    output = mission.update(telemetry(5.2, altitude_m=3.0, forward_position_m=2.5))

    assert output.phase == MissionPhase.FINAL_EXIT
    assert output.command.kind == CommandKind.BODY_VELOCITY


def test_altitude_hold_climbs_when_forward_phase_is_below_target() -> None:
    mission = build_mission()
    mission.update(telemetry(0.0, altitude_m=1.0))
    mission.update(telemetry(1.0, gate_detection=centered_gate(1.0)))

    output = mission.update(telemetry(2.0, altitude_m=0.70, forward_position_m=1.2))

    assert output.phase == MissionPhase.NEXT_GATE_ACQUIRE
    assert output.command.kind == CommandKind.BODY_VELOCITY
    assert output.command.body_vz_m_s < 0.0


def test_next_gate_acquire_falls_back_to_seek_after_max_distance() -> None:
    mission = build_mission()
    mission.update(telemetry(0.0, altitude_m=1.0))
    mission.update(telemetry(1.0, gate_detection=centered_gate(1.0)))

    output = mission.update(telemetry(2.0, forward_position_m=1.2))
    assert output.phase == MissionPhase.NEXT_GATE_ACQUIRE

    output = mission.update(telemetry(3.0, forward_position_m=5.3))

    assert output.phase == MissionPhase.SEEK_GATE
    assert output.gate_index == 1


def test_next_gate_acquire_uses_detection_instead_of_blind_distance() -> None:
    mission = build_mission()
    mission.update(telemetry(0.0, altitude_m=1.0))
    mission.update(telemetry(1.0, gate_detection=centered_gate(1.0)))

    output = mission.update(
        telemetry(2.0, forward_position_m=1.4, gate_detection=centered_gate(2.0))
    )

    assert output.phase == MissionPhase.BRAKE
    assert output.gate_index == 1

    output = mission.update(
        telemetry(3.1, forward_position_m=1.4, gate_detection=centered_gate(3.1))
    )

    assert output.phase == MissionPhase.PASS_GATE
    assert output.gate_index == 1

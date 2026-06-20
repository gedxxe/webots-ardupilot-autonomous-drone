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


def small_centered_gate(now_s: float) -> GateDetection:
    return GateDetection(
        bbox=BoundingBox(45, 45, 55, 55),
        confidence=0.95,
        observed_at_s=now_s,
    )


def build_mission() -> GateAutonomyMission:
    servo = GateVisualServoController(
        VisualServoConfig(
            frame_width_px=100,
            frame_height_px=100,
            filter_alpha=1.0,
        )
    )
    return GateAutonomyMission(
        GateMissionConfig(
            takeoff_required_stable_ticks=1,
            required_detection_ticks=1,
            center_dwell_s=0.0,
            center_clearance_required_s=0.0,
            gate_pass_distance_m=1.0,
            next_gate_acquire_speed_m_s=2.5,
            next_gate_acquire_min_clear_distance_m=0.5,
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
    assert output.command.body_vx_m_s == 2.5
    assert output.gate_index == 1

    output = mission.update(
        telemetry(4.1, forward_position_m=2.0, gate_detection=centered_gate(4.1))
    )
    assert output.phase == MissionPhase.PASS_GATE

    output = mission.update(telemetry(5.2, forward_position_m=3.2))
    assert output.phase == MissionPhase.FINAL_EXIT

    output = mission.update(telemetry(6.2, forward_position_m=5.3))
    assert output.phase == MissionPhase.BRAKE
    assert output.detail == "braking before landing"
    assert output.command.body_vx_m_s == 1.5

    output = mission.update(telemetry(7.3, forward_position_m=5.3))
    assert output.phase == MissionPhase.LAND
    assert output.command.kind == CommandKind.LAND


def test_lost_detection_while_centering_uses_grace_before_seek() -> None:
    mission = GateAutonomyMission(
        GateMissionConfig(
            takeoff_required_stable_ticks=1,
            required_detection_ticks=1,
            center_dwell_s=5.0,
            center_clearance_required_s=0.5,
            center_lost_detection_grace_ticks=2,
        ),
        GateVisualServoController(
            VisualServoConfig(
                frame_width_px=100,
                frame_height_px=100,
                filter_alpha=1.0,
            )
        ),
    )
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
    assert output.phase == MissionPhase.CENTER_GATE
    assert output.detail == "holding gate 1 target loss"

    output = mission.update(telemetry(1.3))
    assert output.phase == MissionPhase.CENTER_GATE

    output = mission.update(telemetry(1.4))
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


def test_centering_requires_dwell_before_committed_pass() -> None:
    mission = GateAutonomyMission(
        GateMissionConfig(
            takeoff_required_stable_ticks=1,
            required_detection_ticks=1,
            center_dwell_s=5.0,
            center_clearance_required_s=0.5,
        ),
        GateVisualServoController(
            VisualServoConfig(
                frame_width_px=100,
                frame_height_px=100,
                filter_alpha=1.0,
            )
        ),
    )

    mission.update(telemetry(0.0, altitude_m=1.0))

    output = mission.update(telemetry(1.0, gate_detection=centered_gate(1.0)))
    assert output.phase == MissionPhase.CENTER_GATE
    assert "dwell=0.0/5.0s" in output.detail

    output = mission.update(telemetry(5.9, gate_detection=centered_gate(5.9)))
    assert output.phase == MissionPhase.CENTER_GATE

    output = mission.update(telemetry(6.0, gate_detection=centered_gate(6.0)))
    assert output.phase == MissionPhase.PASS_GATE


def test_centering_requires_clearance_margin_before_committed_pass() -> None:
    mission = GateAutonomyMission(
        GateMissionConfig(
            takeoff_required_stable_ticks=1,
            required_detection_ticks=1,
            center_dwell_s=1.0,
            center_clearance_required_s=0.0,
        ),
        GateVisualServoController(
            VisualServoConfig(
                frame_width_px=100,
                frame_height_px=100,
                filter_alpha=1.0,
                pass_clearance_left_error=0.05,
                pass_clearance_right_error=0.05,
                pass_clearance_up_error=0.05,
                pass_clearance_down_error=0.05,
            )
        ),
    )

    mission.update(telemetry(0.0, altitude_m=1.0))

    off_center = GateDetection(
        bbox=BoundingBox(5, 25, 45, 75),
        confidence=0.95,
        observed_at_s=2.0,
    )
    output = mission.update(telemetry(2.0, gate_detection=off_center))

    assert output.phase == MissionPhase.CENTER_GATE
    assert output.servo is not None
    assert output.servo.clearance_ready is False


def test_centering_requires_ready_area_before_committed_pass() -> None:
    mission = GateAutonomyMission(
        GateMissionConfig(
            takeoff_required_stable_ticks=1,
            required_detection_ticks=1,
            center_dwell_s=0.0,
            center_clearance_required_s=0.0,
            gate_ready_area_ratio=0.060,
        ),
        GateVisualServoController(
            VisualServoConfig(
                frame_width_px=100,
                frame_height_px=100,
                filter_alpha=1.0,
            )
        ),
    )

    mission.update(telemetry(0.0, altitude_m=1.0))

    output = mission.update(telemetry(1.0, gate_detection=small_centered_gate(1.0)))

    assert output.phase == MissionPhase.CENTER_GATE
    assert "area=0.010/0.060" in output.detail


def test_final_exit_uses_forward_distance_not_altitude() -> None:
    mission = build_mission()
    mission.update(telemetry(0.0, altitude_m=1.0))
    mission.update(telemetry(1.0, gate_detection=centered_gate(1.0)))
    mission.update(
        telemetry(2.0, forward_position_m=1.2, gate_detection=centered_gate(2.0))
    )
    mission.update(
        telemetry(3.1, forward_position_m=1.8, gate_detection=centered_gate(3.1))
    )
    mission.update(
        telemetry(4.2, forward_position_m=1.8, gate_detection=centered_gate(4.2))
    )
    output = mission.update(telemetry(5.3, forward_position_m=3.0))
    assert output.phase == MissionPhase.FINAL_EXIT

    output = mission.update(telemetry(6.3, altitude_m=3.0, forward_position_m=3.1))

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


def test_brake_ramps_forward_speed_down_before_settle_transition() -> None:
    mission = build_mission()
    mission.update(telemetry(0.0, altitude_m=1.0))
    mission.update(telemetry(1.0, gate_detection=centered_gate(1.0)))
    mission.update(telemetry(2.0, forward_position_m=1.2))

    output = mission.update(
        telemetry(3.0, forward_position_m=2.0, gate_detection=centered_gate(3.0))
    )
    assert output.phase == MissionPhase.BRAKE
    assert output.command.body_vx_m_s == 2.5

    output = mission.update(telemetry(3.35, forward_position_m=2.2))
    assert output.phase == MissionPhase.BRAKE
    assert 1.2 < output.command.body_vx_m_s < 1.3

    output = mission.update(telemetry(3.8, forward_position_m=2.3))
    assert output.phase == MissionPhase.BRAKE
    assert output.command.body_vx_m_s == 0.0


def test_brake_disables_companion_altitude_hold_by_default() -> None:
    mission = build_mission()
    mission.update(telemetry(0.0, altitude_m=1.0))
    mission.update(telemetry(1.0, gate_detection=centered_gate(1.0)))
    mission.update(telemetry(2.0, forward_position_m=1.2))

    output = mission.update(
        telemetry(
            3.0,
            altitude_m=0.70,
            forward_position_m=2.0,
            gate_detection=centered_gate(3.0),
        )
    )

    assert output.phase == MissionPhase.BRAKE
    assert output.command.body_vz_m_s == 0.0


def test_brake_altitude_hold_can_be_enabled_for_drift_cases() -> None:
    mission = GateAutonomyMission(
        GateMissionConfig(
            takeoff_required_stable_ticks=1,
            required_detection_ticks=1,
            center_dwell_s=0.0,
            center_clearance_required_s=0.0,
            gate_pass_distance_m=1.0,
            next_gate_acquire_speed_m_s=2.5,
            next_gate_acquire_min_clear_distance_m=0.5,
            next_gate_acquire_max_distance_m=4.0,
            next_gate_acquire_timeout_s=4.0,
            brake_settle_s=1.0,
            brake_altitude_hold_enabled=True,
        ),
        GateVisualServoController(
            VisualServoConfig(
                frame_width_px=100,
                frame_height_px=100,
                filter_alpha=1.0,
            )
        ),
    )
    mission.update(telemetry(0.0, altitude_m=1.0))
    mission.update(telemetry(1.0, gate_detection=centered_gate(1.0)))
    mission.update(telemetry(2.0, forward_position_m=1.2))

    output = mission.update(
        telemetry(
            3.0,
            altitude_m=0.70,
            forward_position_m=2.0,
            gate_detection=centered_gate(3.0),
        )
    )

    assert output.phase == MissionPhase.BRAKE
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

    assert output.phase == MissionPhase.NEXT_GATE_ACQUIRE
    assert output.detail == "clearing previous gate before acquiring gate 2"

    output = mission.update(
        telemetry(2.5, forward_position_m=2.0, gate_detection=centered_gate(2.5))
    )

    assert output.phase == MissionPhase.BRAKE
    assert output.gate_index == 1

    output = mission.update(
        telemetry(3.6, forward_position_m=2.0, gate_detection=centered_gate(3.6))
    )

    assert output.phase == MissionPhase.PASS_GATE
    assert output.gate_index == 1


def test_next_gate_acquire_waits_for_ready_area_before_braking() -> None:
    mission = build_mission()
    mission.update(telemetry(0.0, altitude_m=1.0))
    mission.update(telemetry(1.0, gate_detection=centered_gate(1.0)))
    mission.update(telemetry(2.0, forward_position_m=1.2))

    output = mission.update(
        telemetry(2.6, forward_position_m=2.0, gate_detection=small_centered_gate(2.6))
    )

    assert output.phase == MissionPhase.NEXT_GATE_ACQUIRE
    assert output.command.body_vx_m_s == 2.5
    assert output.detail == "acquiring gate 2"

    mid_area_gate = GateDetection(
        bbox=BoundingBox(40, 37, 60, 63),
        confidence=0.95,
        observed_at_s=3.0,
    )
    output = mission.update(
        telemetry(3.0, forward_position_m=2.5, gate_detection=mid_area_gate)
    )

    assert output.phase == MissionPhase.NEXT_GATE_ACQUIRE
    assert output.detail == "approaching gate 2 area=0.052/0.060"

    output = mission.update(
        telemetry(3.5, forward_position_m=3.0, gate_detection=centered_gate(3.5))
    )

    assert output.phase == MissionPhase.BRAKE

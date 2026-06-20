from drone_autonomy.autonomy.commands import CommandKind
from drone_autonomy.control.visual_servo import (
    GateVisualServoController,
    VisualServoConfig,
)
from drone_autonomy.perception.detections import BoundingBox, GateDetection


def detection(
    bbox: BoundingBox,
    confidence: float = 0.9,
    observed_at_s: float = 0.0,
) -> GateDetection:
    return GateDetection(
        bbox=bbox,
        confidence=confidence,
        observed_at_s=observed_at_s,
    )


def test_centered_gate_moves_forward_without_lateral_correction() -> None:
    controller = GateVisualServoController(
        VisualServoConfig(
            frame_width_px=100,
            frame_height_px=100,
            filter_alpha=1.0,
        )
    )

    output = controller.update(detection(BoundingBox(35, 35, 65, 65)))

    assert output.command.kind == CommandKind.BODY_VELOCITY
    assert output.command.body_vx_m_s > 0.0
    assert output.command.body_vy_m_s == 0.0
    assert output.command.body_vz_m_s == 0.0
    assert output.command.yaw_rate_rad_s == 0.0
    assert output.pass_ready is True


def test_gate_on_right_commands_right_motion_and_right_yaw() -> None:
    controller = GateVisualServoController(
        VisualServoConfig(frame_width_px=100, frame_height_px=100, filter_alpha=1.0)
    )

    output = controller.update(detection(BoundingBox(60, 35, 90, 65)))

    assert output.command.body_vy_m_s > 0.0
    assert output.command.yaw_rate_rad_s > 0.0


def test_missing_detection_returns_hold() -> None:
    controller = GateVisualServoController()

    output = controller.update(None)

    assert output.command.kind == CommandKind.HOLD
    assert output.is_aligned is False
    assert output.pass_ready is False

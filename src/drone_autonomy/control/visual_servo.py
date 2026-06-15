from __future__ import annotations

from dataclasses import dataclass

from drone_autonomy.autonomy.commands import VehicleCommand
from drone_autonomy.control.filters import LowPassFilter, apply_deadband, clamp
from drone_autonomy.perception.detections import FrameShape, GateDetection


@dataclass(frozen=True)
class VisualServoConfig:
    """Visual-servo tuning for one RGB camera stream.

    The controller only uses normalized image geometry. It does not infer metric
    distance from RGB because that requires camera intrinsics and known gate
    dimensions. Gate "closeness" is only approximated by bounding-box area.
    """

    frame_width_px: int = 1280
    frame_height_px: int = 720
    min_confidence: float = 0.45
    filter_alpha: float = 0.45
    center_deadband_x: float = 0.035
    center_deadband_y: float = 0.060
    aligned_error_x: float = 0.075
    aligned_error_y: float = 0.120
    target_area_ratio: float = 0.100
    max_error_for_forward: float = 0.450
    min_forward_speed_m_s: float = 0.15
    max_forward_speed_m_s: float = 1.20
    lateral_kp: float = 0.90
    vertical_kp: float = 0.55
    yaw_kp: float = 0.80
    max_lateral_speed_m_s: float = 0.65
    max_vertical_speed_m_s: float = 0.35
    max_yaw_rate_rad_s: float = 0.65

    def frame_shape(self) -> FrameShape:
        return FrameShape(width_px=self.frame_width_px, height_px=self.frame_height_px)


@dataclass(frozen=True)
class ServoOutput:
    """Controller output plus diagnostics for logging and test assertions."""

    command: VehicleCommand
    is_aligned: bool
    pass_ready: bool
    error_x: float
    error_y: float
    area_ratio: float


class GateVisualServoController:
    """Image-based visual servo controller for a single hollow gate.

    The detector decides where the gate is. This controller decides only the
    velocity correction needed to center that gate in the camera image.
    """

    def __init__(self, config: VisualServoConfig | None = None) -> None:
        self.config = config or VisualServoConfig()
        self._x_filter = LowPassFilter(self.config.filter_alpha)
        self._y_filter = LowPassFilter(self.config.filter_alpha)
        self._area_filter = LowPassFilter(self.config.filter_alpha)

    def reset(self) -> None:
        self._x_filter.reset()
        self._y_filter.reset()
        self._area_filter.reset()

    def update(self, detection: GateDetection | None) -> ServoOutput:
        if detection is None or detection.confidence < self.config.min_confidence:
            self.reset()
            return ServoOutput(
                command=VehicleCommand.hold("no usable gate detection"),
                is_aligned=False,
                pass_ready=False,
                error_x=0.0,
                error_y=0.0,
                area_ratio=0.0,
            )

        frame = self.config.frame_shape()
        raw_error_x, raw_error_y = detection.bbox.normalized_center_error(frame)
        raw_area_ratio = detection.bbox.normalized_area(frame)

        # Filtering prevents a single noisy YOLO box from causing sharp velocity
        # changes. The mission still owns phase changes and hysteresis.
        error_x = self._x_filter.update(raw_error_x)
        error_y = self._y_filter.update(raw_error_y)
        area_ratio = self._area_filter.update(raw_area_ratio)

        control_x = apply_deadband(error_x, self.config.center_deadband_x)
        control_y = apply_deadband(error_y, self.config.center_deadband_y)

        # Body-frame sign convention:
        # x image error > 0 means the gate is right of center, so move/yaw right.
        # y image error > 0 means the gate is below center, so move down.
        body_vy_m_s = clamp(
            self.config.lateral_kp * control_x,
            -self.config.max_lateral_speed_m_s,
            self.config.max_lateral_speed_m_s,
        )
        body_vz_m_s = clamp(
            self.config.vertical_kp * control_y,
            -self.config.max_vertical_speed_m_s,
            self.config.max_vertical_speed_m_s,
        )
        yaw_rate_rad_s = clamp(
            self.config.yaw_kp * control_x,
            -self.config.max_yaw_rate_rad_s,
            self.config.max_yaw_rate_rad_s,
        )

        # Approach only when the gate is reasonably centered. This avoids
        # charging at the frame while still correcting large lateral errors.
        worst_error = max(abs(error_x), abs(error_y))
        if worst_error >= self.config.max_error_for_forward:
            body_vx_m_s = 0.0
        else:
            quality = 1.0 - (worst_error / self.config.max_error_for_forward)
            body_vx_m_s = self.config.min_forward_speed_m_s + (
                quality
                * (self.config.max_forward_speed_m_s - self.config.min_forward_speed_m_s)
            )

        is_aligned = (
            abs(error_x) <= self.config.aligned_error_x
            and abs(error_y) <= self.config.aligned_error_y
        )
        # `pass_ready` is not a command to switch phase by itself. The mission
        # requires consecutive ready ticks before committing to the pass.
        pass_ready = is_aligned and area_ratio >= self.config.target_area_ratio

        return ServoOutput(
            command=VehicleCommand.body_velocity(
                body_vx_m_s=body_vx_m_s,
                body_vy_m_s=body_vy_m_s,
                body_vz_m_s=body_vz_m_s,
                yaw_rate_rad_s=yaw_rate_rad_s,
                reason="visual gate centering",
            ),
            is_aligned=is_aligned,
            pass_ready=pass_ready,
            error_x=error_x,
            error_y=error_y,
            area_ratio=area_ratio,
        )

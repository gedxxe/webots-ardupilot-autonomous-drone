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
    dimensions. Bounding-box area is exposed only as diagnostics.

    `pass_target_offset_*` shifts the desired gate center before control and
    clearance checks. Use it for camera/body mounting offsets, for example when
    the safe vehicle centerline is not exactly the camera optical center.
    `pass_clearance_*_error` values are normalized image-error tolerances around
    that shifted target. Positive image x means the gate is right of target;
    positive image y means the gate is below target.
    """

    frame_width_px: int = 640
    frame_height_px: int = 480
    min_confidence: float = 0.45
    filter_alpha: float = 0.18
    command_filter_alpha: float = 0.25
    center_deadband_x: float = 0.070
    center_deadband_y: float = 0.090
    aligned_error_x: float = 0.075
    aligned_error_y: float = 0.120
    pass_target_offset_x: float = 0.0
    pass_target_offset_y: float = 0.0
    pass_clearance_left_error: float = 0.090
    pass_clearance_right_error: float = 0.090
    pass_clearance_up_error: float = 0.130
    pass_clearance_down_error: float = 0.130
    max_error_for_forward: float = 0.450
    min_forward_speed_m_s: float = 0.00
    max_forward_speed_m_s: float = 0.30
    lateral_kp: float = 0.35
    vertical_kp: float = 0.28
    yaw_kp: float = 0.18
    max_lateral_speed_m_s: float = 0.22
    max_vertical_speed_m_s: float = 0.16
    max_yaw_rate_rad_s: float = 0.18

    def __post_init__(self) -> None:
        if self.frame_width_px <= 0 or self.frame_height_px <= 0:
            raise ValueError("frame dimensions must be positive")
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("filter_alpha must be in the range (0, 1]")
        if not 0.0 < self.command_filter_alpha <= 1.0:
            raise ValueError("command_filter_alpha must be in the range (0, 1]")
        if self.aligned_error_x < 0.0 or self.aligned_error_y < 0.0:
            raise ValueError("aligned errors must be non-negative")
        if self.max_error_for_forward <= 0.0:
            raise ValueError("max_error_for_forward must be positive")
        clearance_values = (
            self.pass_clearance_left_error,
            self.pass_clearance_right_error,
            self.pass_clearance_up_error,
            self.pass_clearance_down_error,
        )
        if any(value < 0.0 for value in clearance_values):
            raise ValueError("pass clearance errors must be non-negative")
        if self.min_forward_speed_m_s < 0.0:
            raise ValueError("min_forward_speed_m_s must be non-negative")
        if self.max_forward_speed_m_s < self.min_forward_speed_m_s:
            raise ValueError("max_forward_speed_m_s must be >= min_forward_speed_m_s")
        max_values = (
            self.max_lateral_speed_m_s,
            self.max_vertical_speed_m_s,
            self.max_yaw_rate_rad_s,
        )
        if any(value < 0.0 for value in max_values):
            raise ValueError("max speed/rate values must be non-negative")

    def frame_shape(self) -> FrameShape:
        return FrameShape(width_px=self.frame_width_px, height_px=self.frame_height_px)


@dataclass(frozen=True)
class ServoOutput:
    """Controller output plus diagnostics for logging and test assertions."""

    command: VehicleCommand
    is_aligned: bool
    clearance_ready: bool
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
        self._vx_filter = LowPassFilter(self.config.command_filter_alpha, 0.0)
        self._vy_filter = LowPassFilter(self.config.command_filter_alpha, 0.0)
        self._vz_filter = LowPassFilter(self.config.command_filter_alpha, 0.0)
        self._yaw_filter = LowPassFilter(self.config.command_filter_alpha, 0.0)

    def reset(self) -> None:
        self._x_filter.reset()
        self._y_filter.reset()
        self._area_filter.reset()
        self._vx_filter.value = 0.0
        self._vy_filter.value = 0.0
        self._vz_filter.value = 0.0
        self._yaw_filter.value = 0.0

    def update(self, detection: GateDetection | None) -> ServoOutput:
        if detection is None or detection.confidence < self.config.min_confidence:
            self.reset()
            return ServoOutput(
                command=VehicleCommand.hold("no usable gate detection"),
                is_aligned=False,
                clearance_ready=False,
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
        error_x = self._x_filter.update(
            raw_error_x - self.config.pass_target_offset_x
        )
        error_y = self._y_filter.update(
            raw_error_y - self.config.pass_target_offset_y
        )
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

        body_vx_m_s = self._vx_filter.update(body_vx_m_s)
        body_vy_m_s = self._vy_filter.update(body_vy_m_s)
        body_vz_m_s = self._vz_filter.update(body_vz_m_s)
        yaw_rate_rad_s = self._yaw_filter.update(yaw_rate_rad_s)

        is_aligned = (
            abs(error_x) <= self.config.aligned_error_x
            and abs(error_y) <= self.config.aligned_error_y
        )
        # Clearance is an image-space safety gate, not metric obstacle
        # avoidance. The four asymmetric margins let the engineer compensate for
        # camera mounting and drone protrusions without changing mission code.
        clearance_ready = (
            -self.config.pass_clearance_left_error
            <= error_x
            <= self.config.pass_clearance_right_error
            and -self.config.pass_clearance_up_error
            <= error_y
            <= self.config.pass_clearance_down_error
        )
        # Kept for runtime/status compatibility. The mission now combines this
        # clearance validator with a time dwell before committing to forward pass.
        pass_ready = clearance_ready

        return ServoOutput(
            command=VehicleCommand.body_velocity(
                body_vx_m_s=body_vx_m_s,
                body_vy_m_s=body_vy_m_s,
                body_vz_m_s=body_vz_m_s,
                yaw_rate_rad_s=yaw_rate_rad_s,
                reason="visual gate centering",
            ),
            is_aligned=is_aligned,
            clearance_ready=clearance_ready,
            pass_ready=pass_ready,
            error_x=error_x,
            error_y=error_y,
            area_ratio=area_ratio,
        )

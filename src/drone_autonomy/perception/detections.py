from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FrameShape:
    """Pixel dimensions of the camera frame used by detection geometry."""

    width_px: int
    height_px: int

    def __post_init__(self) -> None:
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("frame dimensions must be positive")


@dataclass(frozen=True)
class BoundingBox:
    """Detector bounding box in pixel coordinates.

    The box should wrap the visible gate frame. It should not describe the
    hollow opening only, because the current controller uses box center and area
    as image-space control inputs.
    """

    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def __post_init__(self) -> None:
        if self.x_max <= self.x_min:
            raise ValueError("x_max must be greater than x_min")
        if self.y_max <= self.y_min:
            raise ValueError("y_max must be greater than y_min")

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min

    @property
    def area(self) -> float:
        return self.width * self.height

    @property
    def center_x(self) -> float:
        return (self.x_min + self.x_max) / 2.0

    @property
    def center_y(self) -> float:
        return (self.y_min + self.y_max) / 2.0

    def normalized_center_error(self, frame: FrameShape) -> tuple[float, float]:
        """Return image center error in the range expected by controllers.

        `x > 0` means target appears right of image center. `y > 0` means target
        appears below image center. Values can exceed `[-1, 1]` if a detector
        reports boxes partially outside the frame.
        """

        half_width = frame.width_px / 2.0
        half_height = frame.height_px / 2.0
        return (
            (self.center_x - half_width) / half_width,
            (self.center_y - half_height) / half_height,
        )

    def normalized_area(self, frame: FrameShape) -> float:
        return self.area / float(frame.width_px * frame.height_px)


@dataclass(frozen=True)
class GateDetection:
    """Single gate observation from YOLO, Webots, or a recorded-frame adapter."""

    bbox: BoundingBox
    confidence: float
    observed_at_s: float
    class_name: str = "gate"
    track_id: int | str | None = None

    def __post_init__(self) -> None:
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in the range [0, 1]")

    def is_fresh(self, now_s: float, max_age_s: float) -> bool:
        return 0.0 <= now_s - self.observed_at_s <= max_age_s

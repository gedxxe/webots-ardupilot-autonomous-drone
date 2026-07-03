from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CameraSourceStatus:
    """Last observed state for a simulator or hardware frame source.

    Mission code must never branch on this diagnostic status. Runtime providers
    use it only to explain why perception is fail-closed while waiting for a
    valid camera frame.
    """

    stage: str = "not_started"
    detail: str = "camera source has not attempted a read yet"
    connected: bool = False
    buffered_bytes: int = 0


@dataclass(frozen=True)
class CameraFrame:
    """One camera frame plus the timestamp used by perception.

    `image` is intentionally typed as `object` so the perception contract does
    not force the mission code to import NumPy, OpenCV, Webots, or any camera
    SDK. Concrete adapters should pass a NumPy array shaped as HxWxC when using
    the YOLO detector.
    """

    image: object
    observed_at_s: float
    width_px: int
    height_px: int
    encoding: str
    source: str = ""

    def __post_init__(self) -> None:
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("frame dimensions must be positive")

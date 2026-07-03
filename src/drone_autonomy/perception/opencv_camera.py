from __future__ import annotations

from dataclasses import dataclass
from time import monotonic

from drone_autonomy.perception.frames import CameraFrame, CameraSourceStatus


@dataclass(frozen=True)
class OpenCvCameraConfig:
    """OpenCV camera settings for the Raspberry Pi/C920 path.

    `source` is intentionally a string so operators can use either a numeric
    camera index such as `"0"` or a device/path accepted by OpenCV. Width,
    height, and FPS are requests to the camera driver, not guaranteed results;
    diagnostics must confirm the actual frame size before tuning visual gains.
    """

    source: str = "0"
    backend: str = "default"
    width_px: int = 640
    height_px: int = 480
    fps: float = 30.0
    read_timeout_s: float = 0.05
    open_retry_s: float = 1.0

    def __post_init__(self) -> None:
        if not self.source:
            raise ValueError("OpenCV camera source must not be empty")
        if self.backend not in {"default", "v4l2"}:
            raise ValueError("OpenCV camera backend must be 'default' or 'v4l2'")
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("OpenCV camera frame dimensions must be positive")
        if self.fps <= 0.0:
            raise ValueError("OpenCV camera FPS must be positive")
        if self.read_timeout_s <= 0.0:
            raise ValueError("OpenCV camera read_timeout_s must be positive")
        if self.open_retry_s <= 0.0:
            raise ValueError("OpenCV camera open_retry_s must be positive")


class OpenCvCameraSource:
    """Read frames from a real camera through OpenCV.

    This source has no mission or MAVLink dependency. It only converts camera
    frames into the shared `CameraFrame` contract used by the YOLO pipeline.
    """

    def __init__(self, config: OpenCvCameraConfig) -> None:
        self.config = config
        self._capture: object | None = None
        self._last_open_attempt_s = -999.0
        self.last_status = CameraSourceStatus()

    def read_latest(self, observed_at_s: float) -> CameraFrame | None:
        capture = self._ensure_capture()
        if capture is None:
            return None

        ok, frame_bgr = capture.read()
        if not ok or frame_bgr is None:
            self._set_status("read_failed", "OpenCV returned no frame", connected=True)
            return None

        try:
            import cv2
            import numpy as np
        except ModuleNotFoundError as exc:  # pragma: no cover - environment guard
            self._set_status("opencv_missing", str(exc), connected=False)
            return None

        image = np.asarray(frame_bgr)
        if image.ndim == 2:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        elif image.ndim == 3 and image.shape[2] >= 3:
            image_rgb = cv2.cvtColor(image[:, :, :3], cv2.COLOR_BGR2RGB)
        else:
            self._set_status(
                "invalid_frame",
                f"shape={getattr(image, 'shape', None)}",
                connected=True,
            )
            return None

        height_px, width_px = image_rgb.shape[:2]
        frame = CameraFrame(
            image=image_rgb,
            observed_at_s=observed_at_s,
            width_px=int(width_px),
            height_px=int(height_px),
            encoding="rgb8",
            source=f"opencv:{self.config.source}",
        )
        self._set_status(
            "frame_ready",
            (
                f"{frame.width_px}x{frame.height_px} "
                f"requested={self.config.width_px}x{self.config.height_px}"
            ),
            connected=True,
        )
        return frame

    def close(self) -> None:
        if self._capture is None:
            return
        try:
            self._capture.release()
        finally:
            self._capture = None
            self._set_status("closed", "OpenCV camera released", connected=False)

    def _ensure_capture(self) -> object | None:
        if self._capture is not None and self._capture.isOpened():
            return self._capture
        if self._capture is not None:
            try:
                self._capture.release()
            finally:
                self._capture = None

        now_s = monotonic()
        if now_s - self._last_open_attempt_s < self.config.open_retry_s:
            return None
        self._last_open_attempt_s = now_s

        try:
            import cv2
        except ModuleNotFoundError as exc:  # pragma: no cover - environment guard
            self._set_status("opencv_missing", str(exc), connected=False)
            return None

        source = _parse_opencv_source(self.config.source)
        backend = _opencv_backend(cv2, self.config.backend)
        capture = (
            cv2.VideoCapture(source)
            if backend is None
            else cv2.VideoCapture(source, backend)
        )
        if not capture.isOpened():
            self._set_status(
                "open_failed",
                f"source={self.config.source} backend={self.config.backend}",
                connected=False,
            )
            capture.release()
            return None

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.config.width_px))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.config.height_px))
        capture.set(cv2.CAP_PROP_FPS, float(self.config.fps))
        self._capture = capture
        self._set_status(
            "opened",
            (
                f"source={self.config.source} backend={self.config.backend} "
                f"requested={self.config.width_px}x{self.config.height_px}@"
                f"{self.config.fps:0.1f}"
            ),
            connected=True,
        )
        return capture

    def _set_status(self, stage: str, detail: str, *, connected: bool) -> None:
        self.last_status = CameraSourceStatus(
            stage=stage,
            detail=detail,
            connected=connected,
            buffered_bytes=0,
        )


def _parse_opencv_source(source: str) -> int | str:
    """Return an int for camera-index strings, otherwise the original source."""

    stripped = source.strip()
    if stripped.isdigit():
        return int(stripped)
    return source


def _opencv_backend(cv2: object, backend: str) -> int | None:
    if backend == "default":
        return None
    if backend == "v4l2":
        return int(getattr(cv2, "CAP_V4L2"))
    raise ValueError("OpenCV camera backend must be 'default' or 'v4l2'")

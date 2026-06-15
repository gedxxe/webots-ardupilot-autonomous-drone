from __future__ import annotations

from dataclasses import dataclass

from drone_autonomy.perception.detections import GateDetection
from drone_autonomy.perception.webots_camera import WebotsCameraConfig, WebotsTcpCameraClient
from drone_autonomy.perception.yolo import YoloGateConfig, YoloGateDetector


@dataclass(frozen=True)
class WebotsYoloConfig:
    """Config for the Webots camera plus YOLO perception pipeline."""

    camera: WebotsCameraConfig
    yolo: YoloGateConfig


class WebotsYoloGateProvider:
    """Read a Webots camera frame and run YOLO gate detection.

    The provider is process/runtime glue. It owns I/O resources, while the
    mission still receives only `GateDetection | None`.
    """

    def __init__(self, config: WebotsYoloConfig) -> None:
        self.config = config
        self.camera = WebotsTcpCameraClient(config.camera)
        self.detector = YoloGateDetector(config.yolo)
        self._last_camera_warning_s = -999.0

    def detect(self, now_s: float) -> GateDetection | None:
        frame = self.camera.read_latest(observed_at_s=now_s)
        if frame is None:
            if now_s - self._last_camera_warning_s >= 2.0:
                print(
                    "webots-yolo waiting for camera frame "
                    f"tcp://{self.config.camera.host}:{self.config.camera.port}"
                )
                self._last_camera_warning_s = now_s
            return None
        return self.detector.detect(frame, now_s)

    def close(self) -> None:
        self.camera.close()

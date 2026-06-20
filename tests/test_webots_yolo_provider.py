from __future__ import annotations

from time import monotonic, sleep

from drone_autonomy.perception.detections import BoundingBox, FrameShape, GateDetection
from drone_autonomy.perception.frames import CameraFrame
from drone_autonomy.perception.target_selector import GateCandidate, GateTargetSelectorConfig
from drone_autonomy.perception.webots_camera import WebotsCameraConfig, WebotsCameraStatus
from drone_autonomy.perception.webots_yolo import (
    WebotsYoloConfig,
    WebotsYoloGateProvider,
    _format_raw_prediction_summary,
)
from drone_autonomy.perception.yolo import YoloGateConfig, YoloRawPrediction


class FakeCameraSource:
    def __init__(self, frames: list[CameraFrame]) -> None:
        self.frames = frames
        self.last_status = WebotsCameraStatus(stage="fake_ready", detail="test")
        self.closed = False

    def read_latest(self, observed_at_s: float) -> CameraFrame | None:
        if self.frames:
            return self.frames.pop(0)
        return None

    def close(self) -> None:
        self.closed = True


class FakeDetector:
    def __init__(self) -> None:
        self.detected_frames: list[CameraFrame] = []

    def detect_candidates(self, frame: CameraFrame, now_s: float) -> tuple[GateCandidate, ...]:
        self.detected_frames.append(frame)
        return (
            GateCandidate(
                detection=GateDetection(
                    bbox=BoundingBox(210.0, 140.0, 430.0, 340.0),
                    confidence=0.9,
                    observed_at_s=frame.observed_at_s,
                    class_name="gate",
                ),
                frame=FrameShape(frame.width_px, frame.height_px),
                class_id=0,
            ),
        )


def test_webots_yolo_provider_publishes_background_detection() -> None:
    frame = CameraFrame(
        image=object(),
        observed_at_s=monotonic(),
        width_px=640,
        height_px=480,
        encoding="rgb8_from_gray8",
    )
    camera = FakeCameraSource([frame])
    detector = FakeDetector()
    provider = WebotsYoloGateProvider(
        WebotsYoloConfig(
            camera=WebotsCameraConfig(read_timeout_s=0.01),
            yolo=YoloGateConfig(model_path="fake.pt"),
            selector=GateTargetSelectorConfig(
                stable_window_frames=1,
                required_stable_frames=1,
            ),
            detection_stale_s=5.0,
        ),
        camera=camera,
        detector=detector,
    )

    try:
        deadline_s = monotonic() + 1.0
        detection = None
        while monotonic() < deadline_s:
            detection = provider.detect(monotonic())
            if detection is not None:
                break
            sleep(0.01)

        assert detection is not None
        assert detection.class_name == "gate"
        assert detector.detected_frames == [frame]
    finally:
        provider.close()

    assert camera.closed is True


def test_raw_prediction_summary_counts_classes_before_filtering() -> None:
    summary = _format_raw_prediction_summary(
        (
            YoloRawPrediction(class_id=1, class_name="Dog", confidence=0.80),
            YoloRawPrediction(class_id=3, class_name="Goals-Detection", confidence=0.70),
            YoloRawPrediction(class_id=3, class_name="Goals-Detection", confidence=0.65),
        )
    )

    assert summary == "raw=1:Dogx1 3:Goals-Detectionx2"


def test_webots_yolo_provider_does_not_return_stale_detection() -> None:
    old_frame = CameraFrame(
        image=object(),
        observed_at_s=monotonic() - 10.0,
        width_px=640,
        height_px=480,
        encoding="rgb8_from_gray8",
    )
    provider = WebotsYoloGateProvider(
        WebotsYoloConfig(
            camera=WebotsCameraConfig(read_timeout_s=0.01),
            yolo=YoloGateConfig(model_path="fake.pt"),
            selector=GateTargetSelectorConfig(
                stable_window_frames=1,
                required_stable_frames=1,
            ),
            detection_stale_s=0.1,
        ),
        camera=FakeCameraSource([old_frame]),
        detector=FakeDetector(),
    )

    try:
        sleep(0.05)
        assert provider.detect(monotonic()) is None
    finally:
        provider.close()

from __future__ import annotations

import os
import sys
from threading import Event, Thread, current_thread
from time import monotonic, sleep
from types import SimpleNamespace

from drone_autonomy.perception.detections import BoundingBox, FrameShape, GateDetection
from drone_autonomy.perception.frames import CameraFrame
from drone_autonomy.perception.target_selector import GateCandidate, GateTargetSelectorConfig
from drone_autonomy.perception.webots_camera import WebotsCameraConfig, WebotsCameraStatus
from drone_autonomy.perception.webots_yolo import (
    WebotsYoloConfig,
    WebotsYoloGateProvider,
    _format_raw_prediction_summary,
    _remove_invalid_qt_font_override,
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


class BlockingDetector(FakeDetector):
    """Hold one inference open so provider shutdown ordering can be asserted."""

    def __init__(self) -> None:
        super().__init__()
        self.started = Event()
        self.release = Event()

    def detect_candidates(
        self,
        frame: CameraFrame,
        now_s: float,
    ) -> tuple[GateCandidate, ...]:
        self.started.set()
        self.release.wait()
        return super().detect_candidates(frame, now_s)


class EmptyDetector:
    """Complete inference normally while reporting no gate candidates."""

    def __init__(self) -> None:
        self.completed = Event()

    def detect_candidates(
        self,
        frame: CameraFrame,
        now_s: float,
    ) -> tuple[GateCandidate, ...]:
        self.completed.set()
        return ()


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


def test_invalid_opencv_qt_font_override_falls_back_to_fontconfig(
    monkeypatch: object,
    tmp_path: object,
) -> None:
    missing_font_dir = tmp_path / "missing-opencv-fonts"
    monkeypatch.setenv("QT_QPA_FONTDIR", str(missing_font_dir))

    _remove_invalid_qt_font_override()

    assert "QT_QPA_FONTDIR" not in os.environ

    monkeypatch.setenv("QT_QPA_FONTDIR", str(tmp_path))
    _remove_invalid_qt_font_override()

    assert os.environ["QT_QPA_FONTDIR"] == str(tmp_path)


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


def test_webots_yolo_provider_reports_stalled_frame_progress() -> None:
    observed_at_s = monotonic()
    frame = CameraFrame(
        image=object(),
        observed_at_s=observed_at_s,
        width_px=640,
        height_px=480,
        encoding="rgb8",
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
        camera=FakeCameraSource([frame]),
        detector=FakeDetector(),
    )

    try:
        deadline_s = monotonic() + 1.0
        while provider.detect(monotonic()) is None and monotonic() < deadline_s:
            sleep(0.01)

        assert provider.stale_reason(monotonic()) is None
        stale_reason = provider.stale_reason(observed_at_s + 1.0)
        assert stale_reason is not None
        assert "camera frame stale" in stale_reason
    finally:
        provider.close()


def test_webots_yolo_provider_reports_missing_initial_frame() -> None:
    provider = WebotsYoloGateProvider(
        WebotsYoloConfig(
            camera=WebotsCameraConfig(read_timeout_s=0.01),
            yolo=YoloGateConfig(model_path="fake.pt"),
        ),
        camera=FakeCameraSource([]),
        detector=FakeDetector(),
    )

    try:
        assert provider.stale_reason(monotonic()) == (
            "webots-yolo camera has not produced a frame"
        )
    finally:
        provider.close()


def test_no_gate_candidate_is_healthy_after_inference_completes() -> None:
    frame = CameraFrame(
        image=object(),
        observed_at_s=monotonic(),
        width_px=640,
        height_px=480,
        encoding="rgb8",
    )
    detector = EmptyDetector()
    provider = WebotsYoloGateProvider(
        WebotsYoloConfig(
            camera=WebotsCameraConfig(read_timeout_s=0.01),
            yolo=YoloGateConfig(model_path="fake.pt"),
            detection_stale_s=5.0,
        ),
        camera=FakeCameraSource([frame]),
        detector=detector,
    )

    try:
        assert detector.completed.wait(timeout=1.0)
        assert provider.detect(monotonic()) is None
        assert provider.stale_reason(monotonic()) is None
    finally:
        provider.close()


def test_diagnostics_window_is_pumped_by_runtime_caller(monkeypatch: object) -> None:
    frame = CameraFrame(
        image=object(),
        observed_at_s=monotonic(),
        width_px=640,
        height_px=480,
        encoding="rgb8",
    )
    build_threads: list[str] = []
    window_calls: list[tuple[str, str]] = []

    def fake_build(self: object, *args: object) -> object:
        build_threads.append(current_thread().name)
        return object()

    def record_call(name: str) -> None:
        window_calls.append((name, current_thread().name))

    fake_cv2 = SimpleNamespace(
        WND_PROP_VISIBLE=4,
        imshow=lambda name, canvas: record_call("imshow"),
        pollKey=lambda: (record_call("pollKey"), -1)[1],
        getWindowProperty=lambda name, prop: 1.0,
        destroyWindow=lambda name: record_call("destroyWindow"),
    )

    monkeypatch.setattr(
        WebotsYoloGateProvider,
        "_build_diagnostics_canvas",
        fake_build,
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)
    provider = WebotsYoloGateProvider(
        WebotsYoloConfig(
            camera=WebotsCameraConfig(read_timeout_s=0.01),
            yolo=YoloGateConfig(model_path="fake.pt"),
            selector=GateTargetSelectorConfig(
                stable_window_frames=1,
                required_stable_frames=1,
            ),
            detection_stale_s=5.0,
            diagnostics_window=True,
        ),
        camera=FakeCameraSource([frame]),
        detector=FakeDetector(),
    )

    try:
        deadline_s = monotonic() + 1.0
        while not build_threads and monotonic() < deadline_s:
            sleep(0.01)
        provider.process_events()
    finally:
        provider.close()

    assert build_threads == ["webots-yolo-detector"]
    assert ("imshow", current_thread().name) in window_calls
    assert ("destroyWindow", current_thread().name) in window_calls
    assert all(thread != "webots-yolo-detector" for _, thread in window_calls)


def test_close_waits_for_active_detector_and_closes_camera() -> None:
    frame = CameraFrame(
        image=object(),
        observed_at_s=monotonic(),
        width_px=640,
        height_px=480,
        encoding="rgb8",
    )
    camera = FakeCameraSource([frame])
    detector = BlockingDetector()
    provider = WebotsYoloGateProvider(
        WebotsYoloConfig(
            camera=WebotsCameraConfig(read_timeout_s=0.01),
            yolo=YoloGateConfig(model_path="fake.pt"),
        ),
        camera=camera,
        detector=detector,
    )
    assert detector.started.wait(timeout=1.0)

    close_thread = Thread(target=provider.close, name="provider-close-test")
    close_thread.start()
    sleep(0.02)
    assert close_thread.is_alive()

    detector.release.set()
    close_thread.join(timeout=1.0)

    assert close_thread.is_alive() is False
    assert provider._camera_thread.is_alive() is False
    assert provider._detector_thread.is_alive() is False
    assert camera.closed is True

    # Resource cleanup is intentionally idempotent for nested runtime finally blocks.
    provider.close()

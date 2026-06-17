import pytest

np = pytest.importorskip("numpy")

from drone_autonomy.perception.frames import CameraFrame
from drone_autonomy.perception.yolo import YoloGateConfig, YoloGateDetector


class FakeBoxes:
    xyxy = np.array(
        [
            [10.0, 20.0, 110.0, 220.0],
            [30.0, 40.0, 130.0, 240.0],
        ]
    )
    conf = np.array([0.80, 0.99])
    cls = np.array([0, 1])
    id = None


class FakeResult:
    boxes = FakeBoxes()
    names = {0: "gate", 1: "person"}


class FakeModel:
    names = {0: "gate", 1: "person"}

    def __init__(self) -> None:
        self.last_predict_kwargs: dict[str, object] | None = None

    def predict(self, **kwargs: object) -> list[FakeResult]:
        self.last_predict_kwargs = kwargs
        return [FakeResult()]


def test_yolo_config_defaults_match_bundled_gate_model() -> None:
    config = YoloGateConfig(model_path="models/gate_yolov8n_best.pt")

    assert config.device == "cpu"
    assert config.gate_class_names == ()
    assert config.gate_class_ids == (0,)


def test_yolo_detector_returns_gate_detection_only() -> None:
    model = FakeModel()
    detector = YoloGateDetector(
        YoloGateConfig(
            model_path="fake.pt",
            confidence=0.25,
            image_size_px=320,
            gate_class_names=("gate",),
        ),
        model=model,
    )
    frame = CameraFrame(
        image=np.zeros((480, 640, 3), dtype=np.uint8),
        observed_at_s=7.0,
        width_px=640,
        height_px=480,
        encoding="rgb8",
    )

    detection = detector.detect(frame, now_s=8.0)

    assert detection is not None
    assert detection.class_name == "gate"
    assert detection.confidence == 0.80
    assert detection.observed_at_s == 7.0
    assert detection.bbox.x_min == 10.0
    assert model.last_predict_kwargs is not None
    assert model.last_predict_kwargs["imgsz"] == 320


def test_yolo_detector_can_filter_by_class_id() -> None:
    detector = YoloGateDetector(
        YoloGateConfig(
            model_path="fake.pt",
            gate_class_names=(),
            gate_class_ids=(1,),
        ),
        model=FakeModel(),
    )
    frame = CameraFrame(
        image=np.zeros((480, 640, 3), dtype=np.uint8),
        observed_at_s=1.0,
        width_px=640,
        height_px=480,
        encoding="rgb8",
    )

    detection = detector.detect(frame, now_s=1.0)

    assert detection is not None
    assert detection.class_name == "person"
    assert detection.confidence == 0.99

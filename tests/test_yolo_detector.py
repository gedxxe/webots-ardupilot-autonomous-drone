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
    assert config.gate_class_names == ("Goals-Detection",)
    assert config.gate_class_ids == (3,)


def test_yolo_config_rejects_empty_motion_filter() -> None:
    with pytest.raises(ValueError, match="class filter is empty"):
        YoloGateConfig(
            model_path="models/gate_yolov8n_best.pt",
            gate_class_names=(),
            gate_class_ids=(),
        )


def test_yolo_detector_returns_gate_detection_only() -> None:
    model = FakeModel()
    detector = YoloGateDetector(
        YoloGateConfig(
            model_path="fake.pt",
            confidence=0.25,
            image_size_px=320,
            gate_class_names=("gate",),
            gate_class_ids=(),
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


def test_yolo_detector_default_filter_rejects_non_gate_multiclass_labels() -> None:
    class MultiClassBoxes:
        xyxy = np.array(
            [
                [10.0, 20.0, 110.0, 220.0],
                [30.0, 40.0, 130.0, 240.0],
                [220.0, 140.0, 420.0, 340.0],
            ]
        )
        conf = np.array([0.98, 0.96, 0.70])
        cls = np.array([1, 2, 3])
        id = None

    class MultiClassResult:
        boxes = MultiClassBoxes()
        names = {
            0: "AdvertisementBox",
            1: "Dog",
            2: "Forklift",
            3: "Goals-Detection",
            4: "Table",
        }

    class MultiClassModel(FakeModel):
        names = MultiClassResult.names

        def predict(self, **kwargs: object) -> list[MultiClassResult]:
            self.last_predict_kwargs = kwargs
            return [MultiClassResult()]

    detector = YoloGateDetector(
        YoloGateConfig(model_path="fake.pt"),
        model=MultiClassModel(),
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
    assert detection.class_name == "Goals-Detection"
    assert detection.confidence == 0.70
    assert len(detector.last_candidates) == 1
    assert [prediction.class_id for prediction in detector.last_raw_predictions] == [
        1,
        2,
        3,
    ]
    assert [
        prediction.class_name for prediction in detector.last_raw_predictions
    ] == [
        "Dog",
        "Forklift",
        "Goals-Detection",
    ]


def test_yolo_detector_prefers_closer_larger_gate_over_far_confident_gate() -> None:
    class TwoGateBoxes:
        xyxy = np.array(
            [
                [295.0, 215.0, 345.0, 265.0],
                [220.0, 140.0, 420.0, 340.0],
            ]
        )
        conf = np.array([0.99, 0.70])
        cls = np.array([0, 0])
        id = None

    class TwoGateResult:
        boxes = TwoGateBoxes()
        names = {0: "gate"}

    class TwoGateModel(FakeModel):
        def predict(self, **kwargs: object) -> list[TwoGateResult]:
            self.last_predict_kwargs = kwargs
            return [TwoGateResult()]

    detector = YoloGateDetector(
        YoloGateConfig(
            model_path="fake.pt",
            gate_class_names=("gate",),
            gate_class_ids=(),
        ),
        model=TwoGateModel(),
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
    assert detection.confidence == 0.70
    assert detection.bbox.width == 200.0
    assert len(detector.last_candidates) == 2


def test_yolo_candidates_score_hollow_gate_higher_than_filled_object() -> None:
    class OneGateBoxes:
        xyxy = np.array([[10.0, 10.0, 90.0, 90.0]])
        conf = np.array([0.90])
        cls = np.array([3])
        id = None

    class OneGateResult:
        boxes = OneGateBoxes()
        names = {
            0: "AdvertisementBox",
            1: "Dog",
            2: "Forklift",
            3: "Goals-Detection",
            4: "Table",
        }

    class OneGateModel(FakeModel):
        names = OneGateResult.names

        def predict(self, **kwargs: object) -> list[OneGateResult]:
            self.last_predict_kwargs = kwargs
            return [OneGateResult()]

    detector = YoloGateDetector(
        YoloGateConfig(model_path="fake.pt"),
        model=OneGateModel(),
    )

    hollow = np.zeros((100, 100, 3), dtype=np.uint8)
    hollow[10:90, 10:14] = 255
    hollow[10:90, 86:90] = 255
    hollow[10:14, 10:90] = 255
    hollow[86:90, 10:90] = 255
    detector.detect_candidates(
        CameraFrame(
            image=hollow,
            observed_at_s=1.0,
            width_px=100,
            height_px=100,
            encoding="rgb8",
        ),
        now_s=1.0,
    )
    hollow_score = detector.last_candidates[0].appearance.frame_score

    filled = np.zeros((100, 100, 3), dtype=np.uint8)
    filled[10:90, 10:90] = 255
    detector.detect_candidates(
        CameraFrame(
            image=filled,
            observed_at_s=2.0,
            width_px=100,
            height_px=100,
            encoding="rgb8",
        ),
        now_s=2.0,
    )
    filled_score = detector.last_candidates[0].appearance.frame_score

    assert hollow_score > 0.08
    assert filled_score < hollow_score

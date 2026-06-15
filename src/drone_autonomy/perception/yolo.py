from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from drone_autonomy.perception.detections import BoundingBox, GateDetection
from drone_autonomy.perception.frames import CameraFrame


@dataclass(frozen=True)
class YoloGateConfig:
    """YOLO gate detector settings.

    Keep class filtering explicit. If `gate_class_names` and `gate_class_ids`
    are both empty, every detected class is accepted. For real-world use, prefer
    a trained model with a `gate` class and leave the default filter enabled.
    """

    model_path: str
    confidence: float = 0.35
    image_size_px: int = 640
    device: str = ""
    gate_class_names: tuple[str, ...] = ("gate",)
    gate_class_ids: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        if not self.model_path:
            raise ValueError("model_path is required for YOLO detection")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in the range [0, 1]")
        if self.image_size_px <= 0:
            raise ValueError("image_size_px must be positive")


class YoloGateDetector:
    """Convert YOLO model output into the repository `GateDetection` contract.

    This class has no MAVLink or mission dependency. It can be reused by Webots
    simulation and by a future Raspberry Pi camera adapter.
    """

    def __init__(self, config: YoloGateConfig, model: Any | None = None) -> None:
        self.config = config
        self._allowed_names = {name.lower() for name in config.gate_class_names if name}
        self._allowed_ids = set(config.gate_class_ids)
        if model is None:
            try:
                from ultralytics import YOLO
            except ModuleNotFoundError as exc:  # pragma: no cover - environment guard
                raise RuntimeError(
                    "Ultralytics YOLO is required for --detector webots-yolo. "
                    "Install vision extras: pip install -e '.[vision]'"
                ) from exc
            model = YOLO(config.model_path)
        self._model = model

    def detect(self, frame: CameraFrame | object, now_s: float) -> GateDetection | None:
        """Run YOLO on one frame and return the best gate detection."""

        image = frame.image if isinstance(frame, CameraFrame) else frame
        observed_at_s = frame.observed_at_s if isinstance(frame, CameraFrame) else now_s

        predict_kwargs: dict[str, object] = {
            "source": image,
            "conf": self.config.confidence,
            "imgsz": self.config.image_size_px,
            "verbose": False,
        }
        if self.config.device:
            predict_kwargs["device"] = self.config.device

        results = self._model.predict(**predict_kwargs)
        if not results:
            return None
        return self._best_detection(results[0], observed_at_s, image)

    def _best_detection(
        self,
        result: object,
        observed_at_s: float,
        image: object,
    ) -> GateDetection | None:
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return None

        xyxy_rows = _to_rows(getattr(boxes, "xyxy", []))
        confs = _to_values(getattr(boxes, "conf", []))
        classes = _to_values(getattr(boxes, "cls", []))
        track_ids = _to_values(getattr(boxes, "id", []))
        names = getattr(result, "names", getattr(self._model, "names", {}))
        frame_width, frame_height = _image_dimensions(image)

        best: tuple[float, float, GateDetection] | None = None
        for index, coords in enumerate(xyxy_rows):
            if len(coords) < 4:
                continue

            confidence = float(confs[index]) if index < len(confs) else 0.0
            class_id = int(classes[index]) if index < len(classes) else None
            class_name = _class_name(names, class_id)
            if not self._is_allowed_class(class_id, class_name):
                continue

            bbox = _safe_bbox(coords, frame_width, frame_height)
            if bbox is None:
                continue

            detection = GateDetection(
                bbox=bbox,
                confidence=confidence,
                observed_at_s=observed_at_s,
                class_name=class_name,
                track_id=track_ids[index] if index < len(track_ids) else None,
            )
            score = (confidence, bbox.area)
            if best is None or score > (best[0], best[1]):
                best = (confidence, bbox.area, detection)

        return best[2] if best is not None else None

    def _is_allowed_class(self, class_id: int | None, class_name: str) -> bool:
        if self._allowed_ids and class_id not in self._allowed_ids:
            return False
        if self._allowed_names and class_name.lower() not in self._allowed_names:
            return False
        return True


def _to_values(value: object) -> list[object]:
    if value is None:
        return []
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "tolist"):
        raw = value.tolist()
    else:
        raw = value
    if raw is None:
        return []
    if isinstance(raw, list):
        return raw
    if isinstance(raw, tuple):
        return list(raw)
    return [raw]


def _to_rows(value: object) -> list[list[float]]:
    rows = _to_values(value)
    normalized: list[list[float]] = []
    for row in rows:
        if hasattr(row, "tolist"):
            row = row.tolist()
        if isinstance(row, tuple):
            row = list(row)
        if isinstance(row, list):
            normalized.append([float(item) for item in row])
    return normalized


def _class_name(names: object, class_id: int | None) -> str:
    if class_id is None:
        return "unknown"
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    if isinstance(names, list) and 0 <= class_id < len(names):
        return str(names[class_id])
    return str(class_id)


def _image_dimensions(image: object) -> tuple[int | None, int | None]:
    shape = getattr(image, "shape", None)
    if shape is None or len(shape) < 2:
        return None, None
    return int(shape[1]), int(shape[0])


def _safe_bbox(
    coords: list[float],
    frame_width: int | None,
    frame_height: int | None,
) -> BoundingBox | None:
    x_min, y_min, x_max, y_max = coords[:4]
    if frame_width is not None:
        x_min = max(0.0, min(float(frame_width), x_min))
        x_max = max(0.0, min(float(frame_width), x_max))
    if frame_height is not None:
        y_min = max(0.0, min(float(frame_height), y_min))
        y_max = max(0.0, min(float(frame_height), y_max))
    if x_max <= x_min or y_max <= y_min:
        return None
    return BoundingBox(x_min=x_min, y_min=y_min, x_max=x_max, y_max=y_max)

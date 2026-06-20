from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from drone_autonomy.perception.detections import BoundingBox, FrameShape, GateDetection
from drone_autonomy.perception.frames import CameraFrame
from drone_autonomy.perception.target_selector import (
    GateAppearance,
    GateCandidate,
    GateTargetContext,
    GateTargetSelector,
    GateTargetSelectorConfig,
)
from drone_autonomy.perception.yolo_profile import (
    DEFAULT_GATE_CLASS_IDS,
    DEFAULT_GATE_CLASS_NAMES,
)


@dataclass(frozen=True)
class YoloRawPrediction:
    """One raw YOLO box before class filtering.

    This is diagnostic state only. Mission and control code must continue to use
    `GateCandidate`/`GateDetection`, never raw model output.
    """

    class_id: int | None
    class_name: str
    confidence: float


@dataclass(frozen=True)
class YoloGateConfig:
    """YOLO gate detector settings.

    Keep class filtering fail-closed. The repository simulation profile accepts
    the observed gate label spelling and class id from the bundled multi-class
    model. If retraining changes the class map, update these fields together
    instead of letting non-gate labels become flight targets.
    """

    model_path: str
    confidence: float = 0.35
    image_size_px: int = 640
    device: str = "cpu"
    gate_class_names: tuple[str, ...] = DEFAULT_GATE_CLASS_NAMES
    gate_class_ids: tuple[int, ...] = DEFAULT_GATE_CLASS_IDS
    allow_all_classes: bool = False

    def __post_init__(self) -> None:
        if not self.model_path:
            raise ValueError("model_path is required for YOLO detection")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in the range [0, 1]")
        if self.image_size_px <= 0:
            raise ValueError("image_size_px must be positive")
        if (
            not self.allow_all_classes
            and not any(name.strip() for name in self.gate_class_names)
            and not self.gate_class_ids
        ):
            raise ValueError(
                "YOLO gate class filter is empty. Set gate_class_names and/or "
                "gate_class_ids; accepting every class during motion tests is unsafe."
            )


class YoloGateDetector:
    """Convert YOLO model output into the repository `GateDetection` contract.

    This class has no MAVLink or mission dependency. It can be reused by Webots
    simulation and by a future Raspberry Pi camera adapter.
    """

    def __init__(self, config: YoloGateConfig, model: Any | None = None) -> None:
        self.config = config
        self._allowed_names = {name.lower() for name in config.gate_class_names if name}
        self._allowed_ids = set(config.gate_class_ids)
        self.last_candidates: tuple[GateCandidate, ...] = ()
        self.last_raw_predictions: tuple[YoloRawPrediction, ...] = ()
        self._compat_selector = GateTargetSelector(
            GateTargetSelectorConfig(required_stable_frames=1, stable_window_frames=1)
        )
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
        """Run YOLO and return one selected gate for legacy/direct callers.

        Production Webots autonomy uses `detect_candidates()` followed by
        `GateTargetSelector`, because target validation/tracking should remain a
        separate pipeline stage.
        """

        candidates = self.detect_candidates(frame, now_s)
        result = self._compat_selector.update(
            candidates,
            context=GateTargetContext(phase="seek_gate", gate_index=0),
        )
        return result.detection

    def detect_candidates(
        self,
        frame: CameraFrame | object,
        now_s: float,
    ) -> tuple[GateCandidate, ...]:
        """Run YOLO on one frame and return class-filtered gate candidates."""

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
            self.last_candidates = ()
            self.last_raw_predictions = ()
            return ()
        return self._raw_candidates(results[0], observed_at_s, image)

    def _raw_candidates(
        self,
        result: object,
        observed_at_s: float,
        image: object,
    ) -> tuple[GateCandidate, ...]:
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            self.last_candidates = ()
            self.last_raw_predictions = ()
            return ()

        xyxy_rows = _to_rows(getattr(boxes, "xyxy", []))
        confs = _to_values(getattr(boxes, "conf", []))
        classes = _to_values(getattr(boxes, "cls", []))
        track_ids = _to_values(getattr(boxes, "id", []))
        names = getattr(result, "names", getattr(self._model, "names", {}))
        frame_width, frame_height = _image_dimensions(image)

        frame = (
            FrameShape(frame_width, frame_height)
            if frame_width is not None and frame_height is not None
            else None
        )
        if frame is None:
            self.last_candidates = ()
            self.last_raw_predictions = ()
            return ()

        candidates: list[GateCandidate] = []
        raw_predictions: list[YoloRawPrediction] = []
        for index, coords in enumerate(xyxy_rows):
            if len(coords) < 4:
                continue

            confidence = float(confs[index]) if index < len(confs) else 0.0
            class_id = int(classes[index]) if index < len(classes) else None
            class_name = _class_name(names, class_id)
            raw_predictions.append(
                YoloRawPrediction(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=confidence,
                )
            )
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
            candidates.append(
                GateCandidate(
                    detection=detection,
                    frame=frame,
                    class_id=class_id,
                    appearance=_gate_appearance(image, bbox),
                )
            )

        self.last_candidates = tuple(candidates)
        self.last_raw_predictions = tuple(raw_predictions)
        return self.last_candidates

    def _is_allowed_class(self, class_id: int | None, class_name: str) -> bool:
        if self.config.allow_all_classes:
            return True
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


def _gate_appearance(image: object, bbox: BoundingBox) -> GateAppearance | None:
    """Score whether the candidate crop looks like a hollow rectangular gate.

    This is a lightweight sanity check for false positives where YOLO predicts
    the gate class on another object. It uses only image-space edge placement:
    a gate should have stronger vertical/horizontal edge support near the box
    border than in the center opening.
    """

    shape = getattr(image, "shape", None)
    if shape is None or len(shape) < 2:
        return None

    x_min = max(0, int(round(bbox.x_min)))
    y_min = max(0, int(round(bbox.y_min)))
    x_max = min(int(shape[1]), int(round(bbox.x_max)))
    y_max = min(int(shape[0]), int(round(bbox.y_max)))
    if x_max - x_min < 8 or y_max - y_min < 8:
        return None

    try:
        crop = image[y_min:y_max, x_min:x_max]
    except (IndexError, TypeError):
        return None
    gray = _to_grayscale_array(crop)
    if gray is None:
        return None

    height, width = gray.shape[:2]
    if height < 8 or width < 8:
        return None

    edge = _edge_map(gray)
    if edge is None:
        return None

    band_x = max(2, int(width * 0.14))
    band_y = max(2, int(height * 0.14))
    inner_x0 = max(band_x, int(width * 0.28))
    inner_x1 = min(width - band_x, int(width * 0.72))
    inner_y0 = max(band_y, int(height * 0.28))
    inner_y1 = min(height - band_y, int(height * 0.72))
    if inner_x1 <= inner_x0 or inner_y1 <= inner_y0:
        return None

    left = _density(edge[:, :band_x])
    right = _density(edge[:, width - band_x :])
    top = _density(edge[:band_y, :])
    bottom = _density(edge[height - band_y :, :])
    interior = _density(edge[inner_y0:inner_y1, inner_x0:inner_x1])

    vertical_support = min(left, right)
    # The bottom bar can be partly hidden by the ground, so top support carries
    # more weight than bottom support for the current Webots gate perspective.
    horizontal_support = (0.75 * top) + (0.25 * bottom)
    border_edge_score = (left + right + top + bottom) / 4.0
    hollow_bonus = _clamp01(border_edge_score - interior)
    frame_score = _clamp01(
        (0.32 * border_edge_score)
        + (0.28 * vertical_support)
        + (0.22 * horizontal_support)
        + (0.18 * hollow_bonus)
    )

    return GateAppearance(
        frame_score=frame_score,
        border_edge_score=border_edge_score,
        interior_edge_score=interior,
        vertical_support=vertical_support,
        horizontal_support=horizontal_support,
    )


def _to_grayscale_array(crop: object) -> object | None:
    shape = getattr(crop, "shape", None)
    if shape is None or len(shape) < 2:
        return None
    try:
        import numpy as np
    except ModuleNotFoundError:  # pragma: no cover - YOLO path needs NumPy anyway
        return None

    array = np.asarray(crop)
    if array.size == 0:
        return None
    if array.ndim == 2:
        gray = array.astype("float32", copy=False)
    elif array.ndim == 3 and array.shape[2] >= 3:
        channels = array[:, :, :3].astype("float32", copy=False)
        gray = (
            0.299 * channels[:, :, 0]
            + 0.587 * channels[:, :, 1]
            + 0.114 * channels[:, :, 2]
        )
    else:
        return None

    max_value = float(np.max(gray)) if gray.size else 0.0
    if max_value <= 1.0:
        gray = gray * 255.0
    return gray


def _edge_map(gray: object) -> object | None:
    try:
        import numpy as np
    except ModuleNotFoundError:  # pragma: no cover - YOLO path needs NumPy anyway
        return None

    array = np.asarray(gray, dtype="float32")
    if array.ndim != 2 or min(array.shape) < 2:
        return None

    edge = np.zeros_like(array, dtype="float32")
    edge[:, 1:] += np.abs(array[:, 1:] - array[:, :-1])
    edge[1:, :] += np.abs(array[1:, :] - array[:-1, :])

    high = float(np.percentile(edge, 90))
    threshold = max(6.0, high * 0.35)
    if threshold <= 0.0:
        return None
    return edge >= threshold


def _density(mask: object) -> float:
    try:
        import numpy as np
    except ModuleNotFoundError:  # pragma: no cover - YOLO path needs NumPy anyway
        return 0.0

    array = np.asarray(mask)
    if array.size == 0:
        return 0.0
    return float(np.count_nonzero(array)) / float(array.size)


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))

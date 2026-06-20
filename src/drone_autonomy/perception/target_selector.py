from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import sqrt

from drone_autonomy.perception.detections import BoundingBox, FrameShape, GateDetection


@dataclass(frozen=True)
class GateTargetContext:
    """Mission context visible to perception without importing mission classes."""

    phase: str = "seek_gate"
    gate_index: int = 0


@dataclass(frozen=True)
class GateCandidate:
    """One YOLO gate candidate before target validation and tracking."""

    detection: GateDetection
    frame: FrameShape
    class_id: int | None = None
    appearance: GateAppearance | None = None


@dataclass(frozen=True)
class GateAppearance:
    """Image-space evidence that a candidate looks like a hollow gate frame.

    These scores are deliberately heuristic. They are not a replacement for a
    correctly trained detector; they are a fail-safe layer for cases where YOLO
    assigns the gate class to non-gate objects.
    """

    frame_score: float
    border_edge_score: float
    interior_edge_score: float
    vertical_support: float
    horizontal_support: float


@dataclass(frozen=True)
class CandidateEvaluation:
    """Validation and scoring result for one gate candidate."""

    candidate: GateCandidate
    accepted: bool
    reasons: tuple[str, ...]
    aspect_ratio: float
    area_ratio: float
    center_error_x: float
    center_error_y: float
    area_score: float
    center_score: float
    confidence_score: float
    lock_score: float
    appearance_score: float
    score: float


@dataclass(frozen=True)
class GateSelectionResult:
    """Target selector output used by mission and diagnostics."""

    detection: GateDetection | None
    evaluations: tuple[CandidateEvaluation, ...]
    selected: CandidateEvaluation | None
    validation_roi: BoundingBox | None
    context: GateTargetContext
    stable_hits: int
    lost_frames: int


@dataclass(frozen=True)
class GateTargetSelectorConfig:
    """Geometry, scoring, and tracking policy for gate target selection.

    This layer deliberately uses normalized image geometry only. It treats area
    as a monocular proxy for nearer gate priority, but it does not claim metric
    distance without camera intrinsics and known gate dimensions.
    """

    min_seek_confidence: float = 0.40
    min_track_confidence: float = 0.30
    min_area_ratio: float = 0.0015
    max_area_ratio: float = 0.85
    min_aspect_ratio: float = 0.35
    max_aspect_ratio: float = 4.00
    max_seek_center_error_x: float = 0.96
    max_seek_center_error_y: float = 0.96
    max_track_center_error_x: float = 0.90
    max_track_center_error_y: float = 0.90
    target_area_score_ratio: float = 0.12
    area_weight: float = 0.52
    center_weight: float = 0.26
    confidence_weight: float = 0.12
    lock_weight: float = 0.10
    appearance_weight: float = 0.00
    min_appearance_score: float = 0.00
    track_lock_weight_boost: float = 0.12
    smooth_alpha: float = 0.35
    stable_window_frames: int = 5
    required_stable_frames: int = 3
    max_lost_frames: int = 5
    min_lock_iou_for_track_confidence: float = 0.20

    def __post_init__(self) -> None:
        for name, value in (
            ("min_seek_confidence", self.min_seek_confidence),
            ("min_track_confidence", self.min_track_confidence),
            ("min_area_ratio", self.min_area_ratio),
            ("max_area_ratio", self.max_area_ratio),
            ("min_aspect_ratio", self.min_aspect_ratio),
            ("max_aspect_ratio", self.max_aspect_ratio),
            ("target_area_score_ratio", self.target_area_score_ratio),
            ("smooth_alpha", self.smooth_alpha),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.max_area_ratio <= self.min_area_ratio:
            raise ValueError("max_area_ratio must be greater than min_area_ratio")
        if self.max_aspect_ratio <= self.min_aspect_ratio:
            raise ValueError("max_aspect_ratio must be greater than min_aspect_ratio")
        if not 0.0 < self.smooth_alpha <= 1.0:
            raise ValueError("smooth_alpha must be in the range (0, 1]")
        if self.stable_window_frames < 1:
            raise ValueError("stable_window_frames must be at least 1")
        if not 1 <= self.required_stable_frames <= self.stable_window_frames:
            raise ValueError(
                "required_stable_frames must be in the range "
                "1..stable_window_frames"
            )
        if self.max_lost_frames < 0:
            raise ValueError("max_lost_frames must be non-negative")
        if self.min_lock_iou_for_track_confidence < 0.0:
            raise ValueError("min_lock_iou_for_track_confidence must be non-negative")
        weights = (
            self.area_weight,
            self.center_weight,
            self.confidence_weight,
            self.lock_weight,
            self.appearance_weight,
            self.track_lock_weight_boost,
        )
        if any(weight < 0.0 for weight in weights):
            raise ValueError("selector weights must be non-negative")
        if not 0.0 <= self.min_appearance_score <= 1.0:
            raise ValueError("min_appearance_score must be in the range [0, 1]")


class GateTargetSelector:
    """Validate, score, track, and smooth raw YOLO gate candidates.

    The mission should receive only a selected `GateDetection | None`. All
    details about candidate rejection, ROI boundaries, and score components stay
    here so they can be visualized without leaking YOLO-specific behavior into
    mission logic.
    """

    _INACTIVE_PHASES = {
        "init",
        "takeoff",
        "pass_gate",
        "final_exit",
        "land",
        "complete",
        "failsafe",
    }
    _TRACK_PHASES = {"center_gate"}

    def __init__(self, config: GateTargetSelectorConfig | None = None) -> None:
        self.config = config or GateTargetSelectorConfig()
        self._stable_history: deque[bool] = deque(maxlen=self.config.stable_window_frames)
        self._smoothed_detection: GateDetection | None = None
        self._lost_frames = 0
        self._last_gate_index: int | None = None
        self.last_result = GateSelectionResult(
            detection=None,
            evaluations=(),
            selected=None,
            validation_roi=None,
            context=GateTargetContext(),
            stable_hits=0,
            lost_frames=0,
        )

    def reset(self) -> None:
        self._stable_history.clear()
        self._smoothed_detection = None
        self._lost_frames = 0
        self.last_result = GateSelectionResult(
            detection=None,
            evaluations=(),
            selected=None,
            validation_roi=None,
            context=GateTargetContext(),
            stable_hits=0,
            lost_frames=0,
        )

    def update(
        self,
        candidates: tuple[GateCandidate, ...],
        *,
        context: GateTargetContext,
    ) -> GateSelectionResult:
        if self._last_gate_index is None:
            self._last_gate_index = context.gate_index
        elif context.gate_index != self._last_gate_index:
            self.reset()
            self._last_gate_index = context.gate_index

        frame = candidates[0].frame if candidates else None
        validation_roi = self._validation_roi(frame, context) if frame is not None else None

        if context.phase in self._INACTIVE_PHASES:
            result = GateSelectionResult(
                detection=None,
                evaluations=(),
                selected=None,
                validation_roi=validation_roi,
                context=context,
                stable_hits=sum(self._stable_history),
                lost_frames=self._lost_frames,
            )
            self.last_result = result
            return result

        evaluations = tuple(self._evaluate(candidate, context) for candidate in candidates)
        accepted = [evaluation for evaluation in evaluations if evaluation.accepted]
        selected = max(accepted, key=lambda evaluation: evaluation.score) if accepted else None

        if selected is None:
            self._lost_frames += 1
            if self._lost_frames > self.config.max_lost_frames:
                self._smoothed_detection = None
                self._stable_history.append(False)
            result = GateSelectionResult(
                detection=None,
                evaluations=evaluations,
                selected=None,
                validation_roi=validation_roi,
                context=context,
                stable_hits=sum(self._stable_history),
                lost_frames=self._lost_frames,
            )
            self.last_result = result
            return result

        self._lost_frames = 0
        self._stable_history.append(True)
        self._smoothed_detection = self._smoothed(selected.candidate.detection)
        stable_hits = sum(self._stable_history)
        detection = (
            self._smoothed_detection
            if stable_hits >= self.config.required_stable_frames
            else None
        )

        result = GateSelectionResult(
            detection=detection,
            evaluations=evaluations,
            selected=selected,
            validation_roi=validation_roi,
            context=context,
            stable_hits=stable_hits,
            lost_frames=self._lost_frames,
        )
        self.last_result = result
        return result

    def _evaluate(
        self,
        candidate: GateCandidate,
        context: GateTargetContext,
    ) -> CandidateEvaluation:
        detection = candidate.detection
        bbox = detection.bbox
        area_ratio = bbox.normalized_area(candidate.frame)
        center_error_x, center_error_y = bbox.normalized_center_error(candidate.frame)
        aspect_ratio = bbox.width / bbox.height
        lock_score = _bbox_iou(
            self._smoothed_detection.bbox if self._smoothed_detection else None,
            bbox,
        )
        confidence_threshold = self._confidence_threshold(context, lock_score)

        reasons: list[str] = []
        if detection.confidence < confidence_threshold:
            reasons.append("confidence")
        if area_ratio < self.config.min_area_ratio:
            reasons.append("area_small")
        if area_ratio > self.config.max_area_ratio:
            reasons.append("area_large")
        if not self.config.min_aspect_ratio <= aspect_ratio <= self.config.max_aspect_ratio:
            reasons.append("aspect")

        max_center_x, max_center_y = self._center_limits(context)
        if abs(center_error_x) > max_center_x or abs(center_error_y) > max_center_y:
            reasons.append("roi")

        appearance_score = (
            candidate.appearance.frame_score if candidate.appearance is not None else 0.0
        )
        if self.config.min_appearance_score > 0.0:
            if candidate.appearance is None:
                reasons.append("appearance_missing")
            elif appearance_score < self.config.min_appearance_score:
                reasons.append("appearance")

        area_score = min(1.0, area_ratio / self.config.target_area_score_ratio)
        center_distance = min(
            1.0,
            sqrt(center_error_x**2 + center_error_y**2) / sqrt(2.0),
        )
        center_score = 1.0 - center_distance
        confidence_score = detection.confidence
        area_weight = self.config.area_weight
        center_weight = self.config.center_weight
        confidence_weight = self.config.confidence_weight
        lock_weight = self.config.lock_weight
        if context.phase in self._TRACK_PHASES:
            lock_weight += self.config.track_lock_weight_boost
            area_weight = max(0.0, area_weight - self.config.track_lock_weight_boost)

        score = (
            area_weight * area_score
            + center_weight * center_score
            + confidence_weight * confidence_score
            + lock_weight * lock_score
            + self.config.appearance_weight * appearance_score
        )
        return CandidateEvaluation(
            candidate=candidate,
            accepted=not reasons,
            reasons=tuple(reasons),
            aspect_ratio=aspect_ratio,
            area_ratio=area_ratio,
            center_error_x=center_error_x,
            center_error_y=center_error_y,
            area_score=area_score,
            center_score=center_score,
            confidence_score=confidence_score,
            lock_score=lock_score,
            appearance_score=appearance_score,
            score=score,
        )

    def _confidence_threshold(
        self,
        context: GateTargetContext,
        lock_score: float,
    ) -> float:
        if (
            context.phase in self._TRACK_PHASES
            or lock_score >= self.config.min_lock_iou_for_track_confidence
        ):
            return self.config.min_track_confidence
        return self.config.min_seek_confidence

    def _center_limits(self, context: GateTargetContext) -> tuple[float, float]:
        if context.phase in self._TRACK_PHASES:
            return (
                self.config.max_track_center_error_x,
                self.config.max_track_center_error_y,
            )
        return (
            self.config.max_seek_center_error_x,
            self.config.max_seek_center_error_y,
        )

    def _validation_roi(
        self,
        frame: FrameShape,
        context: GateTargetContext,
    ) -> BoundingBox:
        max_center_x, max_center_y = self._center_limits(context)
        half_width = frame.width_px / 2.0
        half_height = frame.height_px / 2.0
        return BoundingBox(
            x_min=half_width * (1.0 - max_center_x),
            y_min=half_height * (1.0 - max_center_y),
            x_max=half_width * (1.0 + max_center_x),
            y_max=half_height * (1.0 + max_center_y),
        )

    def _smoothed(self, detection: GateDetection) -> GateDetection:
        if self._smoothed_detection is None:
            return detection

        alpha = self.config.smooth_alpha
        previous = self._smoothed_detection.bbox
        current = detection.bbox
        bbox = BoundingBox(
            x_min=_ema(previous.x_min, current.x_min, alpha),
            y_min=_ema(previous.y_min, current.y_min, alpha),
            x_max=_ema(previous.x_max, current.x_max, alpha),
            y_max=_ema(previous.y_max, current.y_max, alpha),
        )
        confidence = _ema(
            self._smoothed_detection.confidence,
            detection.confidence,
            alpha,
        )
        return GateDetection(
            bbox=bbox,
            confidence=confidence,
            observed_at_s=detection.observed_at_s,
            class_name=detection.class_name,
            track_id=detection.track_id,
        )


def _ema(previous: float, current: float, alpha: float) -> float:
    return (alpha * current) + ((1.0 - alpha) * previous)


def _bbox_iou(left: BoundingBox | None, right: BoundingBox) -> float:
    if left is None:
        return 0.0
    x_min = max(left.x_min, right.x_min)
    y_min = max(left.y_min, right.y_min)
    x_max = min(left.x_max, right.x_max)
    y_max = min(left.y_max, right.y_max)
    if x_max <= x_min or y_max <= y_min:
        return 0.0
    intersection = (x_max - x_min) * (y_max - y_min)
    union = left.area + right.area - intersection
    if union <= 0.0:
        return 0.0
    return intersection / union

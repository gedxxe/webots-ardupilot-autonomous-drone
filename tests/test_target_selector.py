from drone_autonomy.perception.detections import BoundingBox, FrameShape, GateDetection
from drone_autonomy.perception.target_selector import (
    GateAppearance,
    GateCandidate,
    GateTargetContext,
    GateTargetSelector,
    GateTargetSelectorConfig,
)


def candidate(
    bbox: BoundingBox,
    *,
    confidence: float = 0.8,
    frame: FrameShape = FrameShape(640, 480),
) -> GateCandidate:
    return GateCandidate(
        detection=GateDetection(
            bbox=bbox,
            confidence=confidence,
            observed_at_s=1.0,
        ),
        frame=frame,
        class_id=0,
    )


def test_selector_requires_stable_window_before_output() -> None:
    selector = GateTargetSelector(
        GateTargetSelectorConfig(stable_window_frames=5, required_stable_frames=3)
    )
    context = GateTargetContext(phase="seek_gate", gate_index=0)
    gate = candidate(BoundingBox(220, 140, 420, 340))

    assert selector.update((gate,), context=context).detection is None
    assert selector.update((gate,), context=context).detection is None

    result = selector.update((gate,), context=context)

    assert result.detection is not None
    assert result.stable_hits == 3
    assert result.selected is not None
    assert result.selected.accepted is True


def test_selector_rejects_invalid_geometry() -> None:
    selector = GateTargetSelector(
        GateTargetSelectorConfig(stable_window_frames=1, required_stable_frames=1)
    )
    context = GateTargetContext(phase="seek_gate", gate_index=0)
    tiny = candidate(BoundingBox(319, 239, 321, 241), confidence=0.99)
    too_wide = candidate(BoundingBox(10, 220, 630, 230), confidence=0.99)

    result = selector.update((tiny, too_wide), context=context)

    assert result.detection is None
    reasons = {reason for evaluation in result.evaluations for reason in evaluation.reasons}
    assert "area_small" in reasons
    assert "aspect" in reasons


def test_selector_prefers_nearer_larger_gate_over_far_confident_gate() -> None:
    selector = GateTargetSelector(
        GateTargetSelectorConfig(stable_window_frames=1, required_stable_frames=1)
    )
    context = GateTargetContext(phase="seek_gate", gate_index=0)
    far_confident = candidate(BoundingBox(295, 215, 345, 265), confidence=0.99)
    near = candidate(BoundingBox(220, 140, 420, 340), confidence=0.70)

    result = selector.update((far_confident, near), context=context)

    assert result.detection is not None
    assert result.detection.confidence == 0.70
    assert result.selected is not None
    assert result.selected.candidate is near


def test_selector_can_reject_low_gate_appearance_score() -> None:
    selector = GateTargetSelector(
        GateTargetSelectorConfig(
            stable_window_frames=1,
            required_stable_frames=1,
            min_appearance_score=0.20,
        )
    )
    context = GateTargetContext(phase="seek_gate", gate_index=0)
    false_gate = GateCandidate(
        detection=GateDetection(
            bbox=BoundingBox(220, 140, 420, 340),
            confidence=0.90,
            observed_at_s=1.0,
        ),
        frame=FrameShape(640, 480),
        class_id=3,
        appearance=GateAppearance(
            frame_score=0.05,
            border_edge_score=0.05,
            interior_edge_score=0.05,
            vertical_support=0.05,
            horizontal_support=0.05,
        ),
    )

    result = selector.update((false_gate,), context=context)

    assert result.detection is None
    assert result.evaluations[0].accepted is False
    assert "appearance" in result.evaluations[0].reasons

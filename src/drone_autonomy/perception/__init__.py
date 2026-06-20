"""Perception contracts for camera-based autonomy."""

from drone_autonomy.perception.detections import BoundingBox, FrameShape, GateDetection
from drone_autonomy.perception.frames import CameraFrame
from drone_autonomy.perception.gate_detector import GateDetector, NullGateDetector
from drone_autonomy.perception.target_selector import (
    CandidateEvaluation,
    GateCandidate,
    GateSelectionResult,
    GateTargetContext,
    GateTargetSelector,
    GateTargetSelectorConfig,
)

__all__ = [
    "BoundingBox",
    "CameraFrame",
    "CandidateEvaluation",
    "FrameShape",
    "GateCandidate",
    "GateDetection",
    "GateSelectionResult",
    "GateDetector",
    "GateTargetContext",
    "GateTargetSelector",
    "GateTargetSelectorConfig",
    "NullGateDetector",
]

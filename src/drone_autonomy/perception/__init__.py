"""Perception contracts for camera-based autonomy."""

from drone_autonomy.perception.detections import BoundingBox, FrameShape, GateDetection
from drone_autonomy.perception.gate_detector import GateDetector, NullGateDetector

__all__ = [
    "BoundingBox",
    "FrameShape",
    "GateDetection",
    "GateDetector",
    "NullGateDetector",
]

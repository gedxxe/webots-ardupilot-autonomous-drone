from __future__ import annotations

from typing import Protocol

from drone_autonomy.perception.detections import GateDetection


class GateDetector(Protocol):
    """Protocol for YOLO, Webots, or recorded-frame gate detectors.

    Implementations may block internally on their own frame source, but the
    recommended runtime architecture is to keep camera/model inference outside
    the mission state machine and pass only the latest detection into `update()`.
    """

    def detect(self, frame: object, now_s: float) -> GateDetection | None:
        """Return the best gate detection in the frame, or None."""


class NullGateDetector:
    """Placeholder detector used until YOLO/Webots camera integration exists."""

    def detect(self, frame: object, now_s: float) -> GateDetection | None:
        return None

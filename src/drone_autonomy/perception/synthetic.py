from __future__ import annotations

from dataclasses import dataclass

from drone_autonomy.autonomy.mission import MissionPhase
from drone_autonomy.perception.detections import BoundingBox, GateDetection


@dataclass(frozen=True)
class SyntheticGateConfig:
    """Synthetic detections for SITL smoke tests that bypass real perception."""

    frame_width_px: int = 1280
    frame_height_px: int = 720
    confidence: float = 0.95
    detection_delay_s: float = 0.25


class SyntheticGateProvider:
    """State-aware gate provider for simulator smoke tests only.

    This intentionally depends on mission phase because it is not a real
    perception module. Use it to validate MAVLink/runtime wiring without a
    trained model, camera stream, or visible gate.
    """

    def __init__(self, config: SyntheticGateConfig | None = None) -> None:
        self.config = config or SyntheticGateConfig()
        self._active_gate_index: int | None = None
        self._active_since_s = 0.0

    def detect_for_phase(
        self,
        now_s: float,
        phase: MissionPhase,
        gate_index: int,
    ) -> GateDetection | None:
        if phase not in {
            MissionPhase.SEEK_GATE,
            MissionPhase.NEXT_GATE_ACQUIRE,
            MissionPhase.BRAKE,
            MissionPhase.CENTER_GATE,
        }:
            self._active_gate_index = None
            return None

        # Keep the fake detection continuous across SEEK_GATE -> CENTER_GATE.
        # Keying on phase would make the detector disappear at the exact moment
        # the mission starts centering, causing a seek/center oscillation.
        if gate_index != self._active_gate_index:
            self._active_gate_index = gate_index
            self._active_since_s = now_s
            return None

        if now_s - self._active_since_s < self.config.detection_delay_s:
            return None

        width = float(self.config.frame_width_px)
        height = float(self.config.frame_height_px)
        return GateDetection(
            bbox=BoundingBox(
                x_min=width * 0.35,
                y_min=height * 0.25,
                x_max=width * 0.65,
                y_max=height * 0.75,
            ),
            confidence=self.config.confidence,
            observed_at_s=now_s,
        )

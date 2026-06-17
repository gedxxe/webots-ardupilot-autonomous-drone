from drone_autonomy.autonomy.mission import MissionPhase
from drone_autonomy.perception.synthetic import SyntheticGateConfig, SyntheticGateProvider


def test_synthetic_detection_persists_from_seek_to_center() -> None:
    provider = SyntheticGateProvider(SyntheticGateConfig(detection_delay_s=0.25))

    assert provider.detect_for_phase(0.0, MissionPhase.SEEK_GATE, 0) is None
    assert provider.detect_for_phase(0.20, MissionPhase.SEEK_GATE, 0) is None

    seek_detection = provider.detect_for_phase(0.30, MissionPhase.SEEK_GATE, 0)
    assert seek_detection is not None

    center_detection = provider.detect_for_phase(0.35, MissionPhase.CENTER_GATE, 0)
    assert center_detection is not None


def test_synthetic_detection_resets_between_gates() -> None:
    provider = SyntheticGateProvider(SyntheticGateConfig(detection_delay_s=0.25))

    assert provider.detect_for_phase(0.0, MissionPhase.SEEK_GATE, 0) is None
    assert provider.detect_for_phase(0.30, MissionPhase.SEEK_GATE, 0) is not None

    assert provider.detect_for_phase(1.0, MissionPhase.NEXT_GATE_ACQUIRE, 1) is None
    assert provider.detect_for_phase(1.20, MissionPhase.NEXT_GATE_ACQUIRE, 1) is None
    assert provider.detect_for_phase(1.30, MissionPhase.NEXT_GATE_ACQUIRE, 1) is not None

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_iris_camera_wrapper_only_selects_profile() -> None:
    script = (REPO_ROOT / "scripts" / "run_iris_camera_yolo.sh").read_text()

    assert 'AUTONOMY_PROFILE="${AUTONOMY_PROFILE:-iris-camera-yolo}"' in script
    assert "YOLO_GATE_CLASS_NAMES" not in script
    assert "YOLO_GATE_CLASS_IDS" not in script
    assert "MISSION_GATE_READY_AREA" not in script
    assert "VISUAL_MAX_FORWARD_SPEED" not in script


def test_generic_runner_has_no_duplicated_shell_default_table() -> None:
    script = (REPO_ROOT / "scripts" / "run_autonomy_sitl.sh").read_text()

    assert "autonomy_defaults.sh" not in script
    assert "DEFAULT_MISSION_" not in script
    assert "DEFAULT_VISUAL_" not in script
    assert "DEFAULT_GATE_SELECTOR_" not in script

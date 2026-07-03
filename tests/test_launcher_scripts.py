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
    assert "MAVLINK_BAUD" in script
    assert "append_arg_if_nonempty --baud MAVLINK_BAUD" in script
    assert "MAVLINK_HEARTBEAT_STALE" in script
    assert "MAVLINK_LOCAL_POSITION_STALE" in script
    assert "AUTONOMY_LOG_JSONL" in script
    assert "append_arg_if_nonempty --log-jsonl AUTONOMY_LOG_JSONL" in script
    assert "OPENCV_CAMERA_SOURCE" in script
    assert "append_arg_if_nonempty --opencv-camera-source OPENCV_CAMERA_SOURCE" in script
    assert "--opencv-diagnostics-window" in script


def test_raspi_hardware_wrapper_is_dry_run_scaffold() -> None:
    script = (REPO_ROOT / "scripts" / "run_raspi_hardware.sh").read_text()

    assert "configs/raspi_runtime.env" in script
    assert "AUTONOMY_ENV_FILE" in script
    assert 'SEND_COMMANDS="${SEND_COMMANDS:-0}"' in script
    assert "run_autonomy_sitl.sh" in script


def test_ncnn_export_helper_documents_generated_model_target() -> None:
    script = (REPO_ROOT / "scripts" / "export_yolo_ncnn.py").read_text()

    assert '"format": "ncnn"' in script
    assert "models/gate_yolov8n_best.pt" in script
    assert "YOLO_MODEL_PATH" in script


def test_opencv_probe_uses_shared_camera_source() -> None:
    script = (REPO_ROOT / "scripts" / "probe_opencv_camera.py").read_text()

    assert "OpenCvCameraSource" in script
    assert "opencv_camera_ok" in script
    assert "src" in script


def test_opencv_yolo_probe_reuses_runtime_pipeline() -> None:
    script = (REPO_ROOT / "scripts" / "probe_opencv_yolo.py").read_text()

    assert "OpenCvCameraSource" in script
    assert "WebotsYoloGateProvider" in script
    assert "GateTargetSelectorConfig" in script
    assert "MAVLink" in script


def test_preflight_check_is_offline_tooling() -> None:
    script = (REPO_ROOT / "scripts" / "preflight_check.py").read_text()

    assert "run_preflight" in script
    assert "runtime_config_from_env" in script
    assert "Does not connect to MAVLink or cameras" in script

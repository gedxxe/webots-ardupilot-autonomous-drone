from pathlib import Path

from drone_autonomy.runtime.config import AutonomyRuntimeConfig
from drone_autonomy.runtime.env_config import (
    load_env_file,
    runtime_config_from_env,
    runtime_env_keys,
)
from drone_autonomy.runtime.preflight import has_blocking_errors, run_preflight


def test_preflight_accepts_dry_run_simulation_yolo_model() -> None:
    config = AutonomyRuntimeConfig(
        detector="webots-yolo",
        yolo_model_path="models/gate_yolov8n_best.pt",
        send_commands=False,
    )

    issues = run_preflight(config, repo_root=Path(__file__).resolve().parents[1])

    assert not has_blocking_errors(issues)
    assert any(issue.code == "dry_run" for issue in issues)


def test_preflight_rejects_empty_yolo_class_filter() -> None:
    config = AutonomyRuntimeConfig(
        detector="webots-yolo",
        yolo_model_path="models/gate_yolov8n_best.pt",
        yolo_gate_class_names=(),
        yolo_gate_class_ids=(),
    )

    issues = run_preflight(
        config,
        repo_root=Path(__file__).resolve().parents[1],
        check_model_file=False,
    )

    assert has_blocking_errors(issues)
    assert any(issue.code == "yolo_class_filter_empty" for issue in issues)


def test_env_config_parser_uses_runtime_defaults_and_repo_root(tmp_path: Path) -> None:
    env_file = tmp_path / "autonomy_runtime.env"
    env_file.write_text(
        '\n'.join(
            [
                'DETECTOR="webots-yolo"',
                'YOLO_MODEL_PATH="${REPO_ROOT}/models/gate_yolov8n_best.pt"',
                'SEND_COMMANDS="1"',
                'AUTONOMY_LOG_JSONL="${REPO_ROOT}/logs/test.jsonl"',
                'YOLO_GATE_CLASS_IDS="3"',
            ]
        ),
        encoding="utf-8",
    )

    values = load_env_file(env_file, repo_root=tmp_path)
    config = runtime_config_from_env(values)

    assert config.detector == "webots-yolo"
    assert Path(config.yolo_model_path) == tmp_path / "models/gate_yolov8n_best.pt"
    assert config.send_commands is True
    assert Path(config.log_jsonl_path) == tmp_path / "logs/test.jsonl"
    assert config.yolo_gate_class_ids == (3,)
    assert config.loop_hz == AutonomyRuntimeConfig().loop_hz


def test_preflight_env_keys_are_known_by_generic_launcher() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    runner = (repo_root / "scripts" / "run_autonomy_sitl.sh").read_text(
        encoding="utf-8"
    )

    missing = [key for key in runtime_env_keys() if key not in runner]

    assert missing == []


def test_preflight_script_simulation_profile_mentions_iris_workflow() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    script = (repo_root / "scripts" / "preflight_check.py").read_text(
        encoding="utf-8"
    )

    assert "run_iris_camera_yolo.sh" in script
    assert '"detector": "webots-yolo"' in script
    assert "send_commands={int(config.send_commands)}" in script

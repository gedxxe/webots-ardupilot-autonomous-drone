#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from drone_autonomy.runtime.config import AutonomyRuntimeConfig
from drone_autonomy.runtime.env_config import load_env_file, runtime_config_from_env
from drone_autonomy.runtime.preflight import has_blocking_errors, run_preflight


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Offline runtime config preflight. Does not connect to MAVLink or cameras.",
    )
    parser.add_argument(
        "--profile",
        choices=["simulation", "raspi"],
        default="simulation",
        help=(
            "Config group to inspect. simulation mirrors the current "
            "run_iris_camera_yolo.sh workflow."
        ),
    )
    parser.add_argument(
        "--env",
        type=Path,
        default=None,
        help="Explicit env file path. Overrides --profile selection.",
    )
    parser.add_argument(
        "--no-model-file-check",
        action="store_true",
        help="Skip YOLO_MODEL_PATH existence check for cross-machine review.",
    )
    args = parser.parse_args()

    env_path = args.env or _default_env_path(args.profile)
    env_values = load_env_file(env_path, repo_root=REPO_ROOT)
    try:
        config = runtime_config_from_env(env_values)
    except ValueError as exc:
        print(f"preflight status=error env={env_path} detail={exc}")
        return 2
    config = _apply_profile_defaults(args.profile, env_values, config)

    issues = run_preflight(
        config,
        repo_root=REPO_ROOT,
        check_model_file=not args.no_model_file_check,
    )
    status = "error" if has_blocking_errors(issues) else "ok"
    print(
        f"preflight profile={args.profile} "
        f"env={env_path} "
        f"status={status} "
        f"detector={config.detector} "
        f"connection={config.connection} "
        f"send_commands={int(config.send_commands)}"
    )
    if not env_path.exists():
        print("warning env_missing using runtime defaults only")
    for issue in issues:
        print(f"{issue.severity} {issue.code}: {issue.message}")
    return 2 if has_blocking_errors(issues) else 0


def _default_env_path(profile: str) -> Path:
    if profile == "raspi":
        local = REPO_ROOT / "configs" / "raspi_runtime.env"
        return local if local.exists() else local.with_suffix(".env.example")
    local = REPO_ROOT / "configs" / "autonomy_runtime.env"
    return local if local.exists() else local.with_suffix(".env.example")


def _apply_profile_defaults(
    profile: str,
    env_values: dict[str, str],
    config: AutonomyRuntimeConfig,
) -> AutonomyRuntimeConfig:
    if profile != "simulation":
        return config
    if env_values.get("AUTONOMY_PROFILE", "iris-camera-yolo") != "iris-camera-yolo":
        return config

    updates: dict[str, object] = {
        "detector": "webots-yolo",
        "webots_diagnostics_window": True,
    }
    bundled_model = REPO_ROOT / "models" / "gate_yolov8n_best.pt"
    if not getattr(config, "yolo_model_path") and bundled_model.exists():
        updates["yolo_model_path"] = str(bundled_model)
    return replace(config, **updates)


if __name__ == "__main__":
    raise SystemExit(main())

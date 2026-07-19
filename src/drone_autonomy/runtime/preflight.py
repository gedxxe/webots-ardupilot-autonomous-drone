from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from drone_autonomy.runtime.config import AutonomyRuntimeConfig

_DETECTOR_CHOICES = {"none", "synthetic", "webots-yolo", "opencv-yolo"}
_YOLO_DETECTORS = {"webots-yolo", "opencv-yolo"}


@dataclass(frozen=True)
class PreflightIssue:
    """One offline configuration issue found before a runtime launch."""

    severity: str
    code: str
    message: str


def run_preflight(
    config: AutonomyRuntimeConfig,
    *,
    repo_root: Path | None = None,
    check_model_file: bool = True,
) -> tuple[PreflightIssue, ...]:
    """Validate launch configuration without touching MAVLink or cameras.

    This is a guardrail for operators and CI. It never arms, connects, reads
    frames, or blocks the mission loop; launch scripts decide whether to run it.
    """

    issues: list[PreflightIssue] = []
    _require_positive(issues, "loop_hz", config.loop_hz)
    _require_positive(issues, "max_runtime", config.max_runtime_s)
    _require_positive(issues, "heartbeat_timeout", config.heartbeat_timeout_s)
    _require_positive(
        issues,
        "mavlink_heartbeat_stale",
        config.mavlink_heartbeat_stale_s,
    )
    _require_positive(
        issues,
        "mavlink_local_position_stale",
        config.mavlink_local_position_stale_s,
    )
    _require_positive(issues, "command_ack_timeout", config.command_ack_timeout_s)
    if config.command_ack_max_retries < 0:
        issues.append(
            PreflightIssue(
                "error",
                "command_ack_retries_negative",
                "COMMAND_ACK_MAX_RETRIES must be zero or greater.",
            )
        )

    _validate_domain_configs(issues, config)

    if config.detector not in _DETECTOR_CHOICES:
        issues.append(
            PreflightIssue(
                "error",
                "detector_invalid",
                "DETECTOR must be one of none, synthetic, webots-yolo, opencv-yolo.",
            )
        )

    if config.detector in _YOLO_DETECTORS:
        _validate_yolo_config(issues, config, repo_root, check_model_file)

    if config.detector == "webots-yolo":
        _require_port(issues, "WEBOTS_CAMERA_PORT", config.webots_camera_port)
        if config.webots_camera_encoding not in {"gray8", "rgb24"}:
            issues.append(
                PreflightIssue(
                    "error",
                    "webots_encoding_invalid",
                    "WEBOTS_CAMERA_ENCODING must be gray8 or rgb24.",
                )
            )

    if config.detector == "opencv-yolo":
        if not config.opencv_camera_source:
            issues.append(
                PreflightIssue(
                    "error",
                    "opencv_source_empty",
                    "OPENCV_CAMERA_SOURCE must not be empty for opencv-yolo.",
                )
            )
        if config.opencv_camera_backend not in {"default", "v4l2"}:
            issues.append(
                PreflightIssue(
                    "error",
                    "opencv_backend_invalid",
                    "OPENCV_CAMERA_BACKEND must be default or v4l2.",
                )
            )
        _require_positive(issues, "opencv_camera_width", config.opencv_camera_width_px)
        _require_positive(issues, "opencv_camera_height", config.opencv_camera_height_px)
        _require_positive(issues, "opencv_camera_fps", config.opencv_camera_fps)
        _require_positive(
            issues,
            "opencv_camera_read_timeout",
            config.opencv_camera_read_timeout_s,
        )
        _require_positive(
            issues,
            "opencv_camera_open_retry",
            config.opencv_camera_open_retry_s,
        )

    _require_positive(issues, "visual_frame_width", config.visual_frame_width_px)
    _require_positive(issues, "visual_frame_height", config.visual_frame_height_px)
    if not config.send_commands:
        issues.append(
            PreflightIssue(
                "info",
                "dry_run",
                "SEND_COMMANDS=0: runtime will not send MAVLink motion commands.",
            )
        )
    if config.send_commands and _looks_like_hardware_connection(config.connection):
        issues.append(
            PreflightIssue(
                "warning",
                "hardware_send_commands",
                "SEND_COMMANDS=1 on a serial hardware endpoint requires an external safety runbook.",
            )
        )

    return tuple(issues)


def has_blocking_errors(issues: tuple[PreflightIssue, ...]) -> bool:
    """Return true when preflight found configuration errors."""

    return any(issue.severity == "error" for issue in issues)


def _validate_domain_configs(
    issues: list[PreflightIssue],
    config: AutonomyRuntimeConfig,
) -> None:
    """Reuse domain dataclass validation without opening runtime resources."""

    builders = (
        ("mission_config", config.mission_config),
        ("visual_servo_config", config.visual_servo_config),
        ("target_selector_config", config.target_selector_config),
    )
    for code, builder in builders:
        try:
            builder()
        except ValueError as exc:
            issues.append(PreflightIssue("error", f"{code}_invalid", str(exc)))


def _validate_yolo_config(
    issues: list[PreflightIssue],
    config: AutonomyRuntimeConfig,
    repo_root: Path | None,
    check_model_file: bool,
) -> None:
    if not config.yolo_model_path:
        issues.append(
            PreflightIssue(
                "error",
                "yolo_model_missing",
                "YOLO_MODEL_PATH is required for webots-yolo/opencv-yolo.",
            )
        )
    elif check_model_file:
        model_path = _resolve_path(config.yolo_model_path, repo_root)
        if not model_path.exists():
            issues.append(
                PreflightIssue(
                    "error",
                    "yolo_model_not_found",
                    f"YOLO_MODEL_PATH does not exist: {model_path}",
                )
            )

    if not 0.0 <= config.yolo_confidence <= 1.0:
        issues.append(
            PreflightIssue(
                "error",
                "yolo_confidence_range",
                "YOLO_CONFIDENCE must be in the range [0, 1].",
            )
        )
    _require_positive(issues, "yolo_imgsz", config.yolo_image_size_px)
    if not config.yolo_gate_class_names and not config.yolo_gate_class_ids:
        issues.append(
            PreflightIssue(
                "error",
                "yolo_class_filter_empty",
                "YOLO gate class filter is empty; set names and/or ids.",
            )
        )


def _require_positive(
    issues: list[PreflightIssue],
    name: str,
    value: float | int,
) -> None:
    if value <= 0:
        issues.append(
            PreflightIssue(
                "error",
                f"{name}_not_positive",
                f"{name.upper()} must be positive.",
            )
        )


def _require_port(issues: list[PreflightIssue], name: str, value: int) -> None:
    if value < 1 or value > 65535:
        issues.append(
            PreflightIssue(
                "error",
                f"{name.lower()}_invalid",
                f"{name} must be in the range 1..65535.",
            )
        )


def _resolve_path(raw_path: str, repo_root: Path | None) -> Path:
    path = Path(raw_path).expanduser()
    if path.is_absolute() or repo_root is None:
        return path
    return repo_root / path


def _looks_like_hardware_connection(connection: str) -> bool:
    return connection.startswith("/dev/") or connection.upper().startswith("COM")

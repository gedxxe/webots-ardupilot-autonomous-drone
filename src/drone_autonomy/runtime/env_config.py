from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Callable

from drone_autonomy.runtime.config import AutonomyRuntimeConfig

_Parser = Callable[[str], object]


def load_env_file(path: Path, *, repo_root: Path) -> dict[str, str]:
    """Load the simple KEY=VALUE env files used by repo launch scripts.

    This parser intentionally supports only the subset used in tracked config
    templates: comments, optional `export`, quotes, and `${REPO_ROOT}` expansion.
    It is for diagnostics/preflight, not a replacement for Bash sourcing.
    """

    values: dict[str, str] = {}
    if not path.exists():
        return values

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].strip()
        if "=" not in line:
            continue
        key, raw_value = line.split("=", 1)
        key = key.strip()
        if not key:
            continue
        value = _strip_inline_comment(raw_value.strip())
        value = _strip_quotes(value)
        value = value.replace("${REPO_ROOT}", str(repo_root)).replace(
            "$REPO_ROOT",
            str(repo_root),
        )
        values[key] = value
    return values


def runtime_config_from_env(
    values: dict[str, str],
    *,
    base: AutonomyRuntimeConfig | None = None,
) -> AutonomyRuntimeConfig:
    """Build `AutonomyRuntimeConfig` from env-style strings."""

    updates: dict[str, object] = {}
    defaults = base or AutonomyRuntimeConfig()
    for env_name, (field_name, parser) in _ENV_TO_FIELD.items():
        if env_name in values:
            updates[field_name] = parser(values[env_name])
    return replace(defaults, **updates)


def runtime_env_keys() -> tuple[str, ...]:
    """Return env keys that map to `AutonomyRuntimeConfig` fields."""

    return tuple(_ENV_TO_FIELD)


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"invalid boolean value: {value!r}")


def _parse_names(value: str) -> tuple[str, ...]:
    return tuple(name.strip() for name in value.split(",") if name.strip())


def _parse_ids(value: str) -> tuple[int, ...]:
    if not value.strip():
        return ()
    return tuple(int(item.strip()) for item in value.split(",") if item.strip())


def _strip_quotes(value: str) -> str:
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
        return value[1:-1]
    return value


def _strip_inline_comment(value: str) -> str:
    quote: str | None = None
    for index, char in enumerate(value):
        if char in {'"', "'"}:
            quote = None if quote == char else char if quote is None else quote
        if char == "#" and quote is None and (index == 0 or value[index - 1].isspace()):
            return value[:index].rstrip()
    return value


_ENV_TO_FIELD: dict[str, tuple[str, _Parser]] = {
    "MAVLINK_CONNECTION": ("connection", str),
    "MAVLINK_BAUD": ("mavlink_baud", int),
    "LOOP_HZ": ("loop_hz", float),
    "MAX_RUNTIME": ("max_runtime_s", float),
    "MAVLINK_HEARTBEAT_STALE": ("mavlink_heartbeat_stale_s", float),
    "MAVLINK_LOCAL_POSITION_STALE": ("mavlink_local_position_stale_s", float),
    "COMMAND_ACK_REQUIRED": ("command_ack_required", _parse_bool),
    "COMMAND_ACK_TIMEOUT": ("command_ack_timeout_s", float),
    "COMMAND_ACK_MAX_RETRIES": ("command_ack_max_retries", int),
    "AUTONOMY_LOG_JSONL": ("log_jsonl_path", str),
    "DETECTOR": ("detector", str),
    "SEND_COMMANDS": ("send_commands", _parse_bool),
    "COURSE_FORWARD_X": ("course_forward_x", float),
    "COURSE_FORWARD_Y": ("course_forward_y", float),
    "WEBOTS_CAMERA_HOST": ("webots_camera_host", str),
    "WEBOTS_CAMERA_PORT": ("webots_camera_port", int),
    "WEBOTS_CAMERA_ENCODING": ("webots_camera_encoding", str),
    "WEBOTS_CAMERA_IDLE_RECONNECT": ("webots_camera_idle_reconnect_s", float),
    "WEBOTS_DETECTION_STALE": ("webots_detection_stale_s", float),
    "WEBOTS_DIAGNOSTICS_WINDOW": ("webots_diagnostics_window", _parse_bool),
    "OPENCV_CAMERA_SOURCE": ("opencv_camera_source", str),
    "OPENCV_CAMERA_BACKEND": ("opencv_camera_backend", str),
    "OPENCV_CAMERA_WIDTH": ("opencv_camera_width_px", int),
    "OPENCV_CAMERA_HEIGHT": ("opencv_camera_height_px", int),
    "OPENCV_CAMERA_FPS": ("opencv_camera_fps", float),
    "OPENCV_CAMERA_READ_TIMEOUT": ("opencv_camera_read_timeout_s", float),
    "OPENCV_CAMERA_OPEN_RETRY": ("opencv_camera_open_retry_s", float),
    "OPENCV_DETECTION_STALE": ("opencv_detection_stale_s", float),
    "OPENCV_DIAGNOSTICS_WINDOW": ("opencv_diagnostics_window", _parse_bool),
    "YOLO_MODEL_PATH": ("yolo_model_path", str),
    "YOLO_CONFIDENCE": ("yolo_confidence", float),
    "YOLO_IMGSZ": ("yolo_image_size_px", int),
    "YOLO_DEVICE": ("yolo_device", str),
    "YOLO_GATE_CLASS_NAMES": ("yolo_gate_class_names", _parse_names),
    "YOLO_GATE_CLASS_IDS": ("yolo_gate_class_ids", _parse_ids),
    "GATE_SELECTOR_MIN_SEEK_CONFIDENCE": (
        "gate_selector_min_seek_confidence",
        float,
    ),
    "GATE_SELECTOR_MIN_TRACK_CONFIDENCE": (
        "gate_selector_min_track_confidence",
        float,
    ),
    "GATE_SELECTOR_MIN_AREA_RATIO": ("gate_selector_min_area_ratio", float),
    "GATE_SELECTOR_MIN_ASPECT_RATIO": ("gate_selector_min_aspect_ratio", float),
    "GATE_SELECTOR_MAX_ASPECT_RATIO": ("gate_selector_max_aspect_ratio", float),
    "GATE_SELECTOR_MIN_APPEARANCE_SCORE": (
        "gate_selector_min_appearance_score",
        float,
    ),
    "GATE_SELECTOR_APPEARANCE_WEIGHT": ("gate_selector_appearance_weight", float),
    "GATE_SELECTOR_STABLE_WINDOW": ("gate_selector_stable_window_frames", int),
    "GATE_SELECTOR_REQUIRED_STABLE": ("gate_selector_required_stable_frames", int),
    "MISSION_TAKEOFF_ALTITUDE": ("mission_takeoff_altitude_m", float),
    "MISSION_TAKEOFF_SETTLE_TOLERANCE": (
        "mission_takeoff_settle_tolerance_m",
        float,
    ),
    "MISSION_TAKEOFF_STABLE_TICKS": (
        "mission_takeoff_required_stable_ticks",
        int,
    ),
    "MISSION_TAKEOFF_TIMEOUT": ("mission_takeoff_timeout_s", float),
    "MISSION_MAX_DETECTION_AGE": ("mission_max_detection_age_s", float),
    "MISSION_REQUIRED_DETECTION_TICKS": ("mission_required_detection_ticks", int),
    "MISSION_CENTER_DWELL": ("mission_center_dwell_s", float),
    "MISSION_CENTER_CLEARANCE_REQUIRED": (
        "mission_center_clearance_required_s",
        float,
    ),
    "MISSION_CENTER_LOST_GRACE_TICKS": (
        "mission_center_lost_detection_grace_ticks",
        int,
    ),
    "MISSION_SEEK_YAW_RATE": ("mission_seek_yaw_rate_rad_s", float),
    "MISSION_GATE_PASS_DISTANCE": ("mission_gate_pass_distance_m", float),
    "MISSION_GATE_PASS_SPEED": ("mission_gate_pass_speed_m_s", float),
    "MISSION_NEXT_GATE_ACQUIRE_SPEED": (
        "mission_next_gate_acquire_speed_m_s",
        float,
    ),
    "MISSION_NEXT_GATE_CLEAR_DISTANCE": (
        "mission_next_gate_acquire_min_clear_distance_m",
        float,
    ),
    "MISSION_NEXT_GATE_MIN_AREA": ("mission_next_gate_acquire_min_area_ratio", float),
    "MISSION_GATE_READY_AREA": ("mission_gate_ready_area_ratio", float),
    "MISSION_NEXT_GATE_MAX_DISTANCE": (
        "mission_next_gate_acquire_max_distance_m",
        float,
    ),
    "MISSION_NEXT_GATE_TIMEOUT": ("mission_next_gate_acquire_timeout_s", float),
    "MISSION_BRAKE_SETTLE": ("mission_brake_settle_s", float),
    "MISSION_BRAKE_RAMP": ("mission_brake_ramp_s", float),
    "MISSION_BRAKE_ALTITUDE_HOLD": (
        "mission_brake_altitude_hold_enabled",
        _parse_bool,
    ),
    "MISSION_FINAL_EXIT_DISTANCE": ("mission_final_exit_distance_m", float),
    "MISSION_FINAL_EXIT_SPEED": ("mission_final_exit_speed_m_s", float),
    "MISSION_MIN_CENTERING_ALTITUDE": (
        "mission_min_centering_altitude_m",
        float,
    ),
    "MISSION_MAX_CENTERING_ALTITUDE": (
        "mission_max_centering_altitude_m",
        float,
    ),
    "MISSION_ALTITUDE_HOLD_ENABLED": (
        "mission_altitude_hold_enabled",
        _parse_bool,
    ),
    "MISSION_ALTITUDE_HOLD_DEADBAND": (
        "mission_altitude_hold_deadband_m",
        float,
    ),
    "MISSION_ALTITUDE_HOLD_KP": ("mission_altitude_hold_kp", float),
    "MISSION_ALTITUDE_HOLD_MAX_CLIMB_SPEED": (
        "mission_altitude_hold_max_climb_m_s",
        float,
    ),
    "MISSION_ALTITUDE_HOLD_MAX_DESCENT_SPEED": (
        "mission_altitude_hold_max_descent_m_s",
        float,
    ),
    "MISSION_LANDING_COMPLETE_ALTITUDE": (
        "mission_landing_complete_altitude_m",
        float,
    ),
    "MISSION_TIMEOUT": ("mission_timeout_s", float),
    "VISUAL_FRAME_WIDTH": ("visual_frame_width_px", int),
    "VISUAL_FRAME_HEIGHT": ("visual_frame_height_px", int),
    "VISUAL_MIN_CONFIDENCE": ("visual_min_confidence", float),
    "VISUAL_FILTER_ALPHA": ("visual_filter_alpha", float),
    "VISUAL_COMMAND_FILTER_ALPHA": ("visual_command_filter_alpha", float),
    "VISUAL_CENTER_DEADBAND_X": ("visual_center_deadband_x", float),
    "VISUAL_CENTER_DEADBAND_Y": ("visual_center_deadband_y", float),
    "VISUAL_ALIGNED_ERROR_X": ("visual_aligned_error_x", float),
    "VISUAL_ALIGNED_ERROR_Y": ("visual_aligned_error_y", float),
    "VISUAL_PASS_TARGET_OFFSET_X": ("visual_pass_target_offset_x", float),
    "VISUAL_PASS_TARGET_OFFSET_Y": ("visual_pass_target_offset_y", float),
    "VISUAL_PASS_CLEARANCE_LEFT": ("visual_pass_clearance_left_error", float),
    "VISUAL_PASS_CLEARANCE_RIGHT": ("visual_pass_clearance_right_error", float),
    "VISUAL_PASS_CLEARANCE_UP": ("visual_pass_clearance_up_error", float),
    "VISUAL_PASS_CLEARANCE_DOWN": ("visual_pass_clearance_down_error", float),
    "VISUAL_MAX_ERROR_FOR_FORWARD": ("visual_max_error_for_forward", float),
    "VISUAL_MIN_FORWARD_SPEED": ("visual_min_forward_speed_m_s", float),
    "VISUAL_MAX_FORWARD_SPEED": ("visual_max_forward_speed_m_s", float),
    "VISUAL_LATERAL_KP": ("visual_lateral_kp", float),
    "VISUAL_VERTICAL_KP": ("visual_vertical_kp", float),
    "VISUAL_YAW_KP": ("visual_yaw_kp", float),
    "VISUAL_MAX_LATERAL_SPEED": ("visual_max_lateral_speed_m_s", float),
    "VISUAL_MAX_VERTICAL_SPEED": ("visual_max_vertical_speed_m_s", float),
    "VISUAL_MAX_YAW_RATE": ("visual_max_yaw_rate_rad_s", float),
}

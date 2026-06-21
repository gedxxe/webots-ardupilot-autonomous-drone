#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${AUTONOMY_ENV_FILE:-${REPO_ROOT}/configs/autonomy_runtime.env}"

CONFIG_KEYS=(
  AUTONOMY_PROFILE
  MAVLINK_CONNECTION
  MAVLINK_BAUD
  DETECTOR
  SEND_COMMANDS
  LOOP_HZ
  MAX_RUNTIME
  COURSE_FORWARD_X
  COURSE_FORWARD_Y
  WEBOTS_CAMERA_HOST
  WEBOTS_CAMERA_PORT
  WEBOTS_CAMERA_ENCODING
  WEBOTS_CAMERA_IDLE_RECONNECT
  WEBOTS_DETECTION_STALE
  WEBOTS_DIAGNOSTICS_WINDOW
  YOLO_CONFIDENCE
  YOLO_IMGSZ
  YOLO_GATE_CLASS_NAMES
  YOLO_GATE_CLASS_IDS
  YOLO_DEVICE
  YOLO_MODEL_PATH
  YOLO_CONFIG_DIR
  GATE_SELECTOR_MIN_SEEK_CONFIDENCE
  GATE_SELECTOR_MIN_TRACK_CONFIDENCE
  GATE_SELECTOR_MIN_AREA_RATIO
  GATE_SELECTOR_MIN_ASPECT_RATIO
  GATE_SELECTOR_MAX_ASPECT_RATIO
  GATE_SELECTOR_MIN_APPEARANCE_SCORE
  GATE_SELECTOR_APPEARANCE_WEIGHT
  GATE_SELECTOR_STABLE_WINDOW
  GATE_SELECTOR_REQUIRED_STABLE
  MISSION_MAX_DETECTION_AGE
  MISSION_REQUIRED_DETECTION_TICKS
  MISSION_CENTER_DWELL
  MISSION_CENTER_CLEARANCE_REQUIRED
  MISSION_CENTER_LOST_GRACE_TICKS
  MISSION_SEEK_YAW_RATE
  MISSION_GATE_PASS_DISTANCE
  MISSION_GATE_PASS_SPEED
  MISSION_NEXT_GATE_ACQUIRE_SPEED
  MISSION_NEXT_GATE_CLEAR_DISTANCE
  MISSION_NEXT_GATE_MIN_AREA
  MISSION_GATE_READY_AREA
  MISSION_NEXT_GATE_MAX_DISTANCE
  MISSION_NEXT_GATE_TIMEOUT
  MISSION_BRAKE_SETTLE
  MISSION_BRAKE_RAMP
  MISSION_BRAKE_ALTITUDE_HOLD
  MISSION_FINAL_EXIT_DISTANCE
  MISSION_FINAL_EXIT_SPEED
  VISUAL_FRAME_WIDTH
  VISUAL_FRAME_HEIGHT
  VISUAL_MIN_CONFIDENCE
  VISUAL_FILTER_ALPHA
  VISUAL_COMMAND_FILTER_ALPHA
  VISUAL_CENTER_DEADBAND_X
  VISUAL_CENTER_DEADBAND_Y
  VISUAL_ALIGNED_ERROR_X
  VISUAL_ALIGNED_ERROR_Y
  VISUAL_PASS_TARGET_OFFSET_X
  VISUAL_PASS_TARGET_OFFSET_Y
  VISUAL_PASS_CLEARANCE_LEFT
  VISUAL_PASS_CLEARANCE_RIGHT
  VISUAL_PASS_CLEARANCE_UP
  VISUAL_PASS_CLEARANCE_DOWN
  VISUAL_MAX_ERROR_FOR_FORWARD
  VISUAL_MIN_FORWARD_SPEED
  VISUAL_MAX_FORWARD_SPEED
  VISUAL_LATERAL_KP
  VISUAL_VERTICAL_KP
  VISUAL_YAW_KP
  VISUAL_MAX_LATERAL_SPEED
  VISUAL_MAX_VERTICAL_SPEED
  VISUAL_MAX_YAW_RATE
  PYTHON_BIN
)

capture_external_overrides() {
  local key
  for key in "${CONFIG_KEYS[@]}"; do
    if [[ -v ${key} ]]; then
      printf -v "EXTERNAL_${key}" '%s' "${!key}"
      printf -v "EXTERNAL_${key}_SET" '%s' "1"
    fi
  done
}

restore_external_overrides() {
  local key
  local value_key
  local set_key
  for key in "${CONFIG_KEYS[@]}"; do
    value_key="EXTERNAL_${key}"
    set_key="EXTERNAL_${key}_SET"
    if [[ "${!set_key:-}" == "1" ]]; then
      printf -v "${key}" '%s' "${!value_key}"
    fi
  done
}

capture_external_overrides

if [[ -f "${ENV_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
fi

restore_external_overrides

BUNDLED_YOLO_MODEL_PATH="${REPO_ROOT}/models/gate_yolov8n_best.pt"

apply_profile_defaults() {
  case "${AUTONOMY_PROFILE:-}" in
    "")
      ;;
    "iris-camera-yolo")
      # Profile values are applied after the local env file is loaded. Inline
      # env still wins, but stale local DETECTOR/diagnostics values cannot make
      # this profile silently stop using the iris camera YOLO path.
      if [[ "${EXTERNAL_DETECTOR_SET:-}" != "1" ]]; then
        DETECTOR="webots-yolo"
      fi
      if [[ "${EXTERNAL_WEBOTS_DIAGNOSTICS_WINDOW_SET:-}" != "1" ]]; then
        WEBOTS_DIAGNOSTICS_WINDOW="1"
      fi
      if [[ -z "${MAVLINK_CONNECTION:-}" ]]; then
        MAVLINK_CONNECTION="udp:127.0.0.1:14551"
      fi
      if [[ -z "${WEBOTS_CAMERA_ENCODING:-}" ]]; then
        WEBOTS_CAMERA_ENCODING="rgb24"
      fi
      if [[ -z "${SEND_COMMANDS:-}" ]]; then
        SEND_COMMANDS="0"
      fi
      ;;
    *)
      echo "Unknown AUTONOMY_PROFILE='${AUTONOMY_PROFILE}'" >&2
      exit 2
      ;;
  esac
}

append_arg_if_set() {
  local option="$1"
  local key="$2"
  if [[ -v ${key} ]]; then
    cmd+=("${option}" "${!key}")
  fi
}

append_arg_if_nonempty() {
  local option="$1"
  local key="$2"
  if [[ -n "${!key:-}" ]]; then
    cmd+=("${option}" "${!key}")
  fi
}

apply_profile_defaults

if [[ -z "${YOLO_MODEL_PATH:-}" && "${DETECTOR:-}" == "webots-yolo" && -f "${BUNDLED_YOLO_MODEL_PATH}" ]]; then
  YOLO_MODEL_PATH="${BUNDLED_YOLO_MODEL_PATH}"
fi

if [[ "${DETECTOR:-}" == "webots-yolo" ]]; then
  export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-${REPO_ROOT}/.tmp_ultralytics}"
  mkdir -p "${YOLO_CONFIG_DIR}"
fi

resolve_autonomy_launcher() {
  # Prefer running the repo source directly. A stale drone-autonomy console
  # script in PATH can point at an older editable install and hide current fixes.
  local python_bin=""
  if [[ -n "${PYTHON_BIN:-}" ]]; then
    python_bin="${PYTHON_BIN}"
  elif [[ -n "${VIRTUAL_ENV:-}" && -x "${VIRTUAL_ENV}/bin/python" ]]; then
    python_bin="${VIRTUAL_ENV}/bin/python"
  elif [[ -x "${REPO_ROOT}/.venv/bin/python" ]]; then
    python_bin="${REPO_ROOT}/.venv/bin/python"
  elif command -v python3 >/dev/null 2>&1; then
    python_bin="$(command -v python3)"
  fi

  if [[ -n "${python_bin}" ]]; then
    if ! "${python_bin}" -c "from pymavlink import mavutil" >/dev/null 2>&1; then
      cat >&2 <<EOF
The selected Python cannot import pymavlink.

Selected Python:
  ${python_bin}

Fix from the repo root:
  python3 -m venv .venv
  source .venv/bin/activate
  pip install --upgrade pip
  pip install -e ".[dev]"

If you want to reuse an existing ArduPilot virtualenv:
  source /media/gedxxe/DATA/venv-ardupilot/bin/activate
  pip install -e ".[dev]"
EOF
      exit 127
    fi

    export PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
    autonomy_launcher=("${python_bin}" -m drone_autonomy.cli)
    return
  fi

  if command -v drone-autonomy >/dev/null 2>&1; then
    autonomy_launcher=(drone-autonomy)
    return
  fi

  cat >&2 <<EOF
No usable autonomy launcher was found.

Fix from the repo root:
  python3 -m venv .venv
  source .venv/bin/activate
  pip install --upgrade pip
  pip install -e ".[dev]"

If you want to reuse an existing ArduPilot virtualenv:
  source /media/gedxxe/DATA/venv-ardupilot/bin/activate
  pip install -e ".[dev]"
EOF
  exit 127
}

resolve_autonomy_launcher

cmd=(
  "${autonomy_launcher[@]}"
  --mode autonomy
)

append_arg_if_nonempty --connection MAVLINK_CONNECTION
append_arg_if_nonempty --baud MAVLINK_BAUD
append_arg_if_nonempty --detector DETECTOR
append_arg_if_nonempty --loop-hz LOOP_HZ
append_arg_if_nonempty --max-runtime MAX_RUNTIME
append_arg_if_nonempty --course-forward-x COURSE_FORWARD_X
append_arg_if_nonempty --course-forward-y COURSE_FORWARD_Y
append_arg_if_nonempty --webots-camera-host WEBOTS_CAMERA_HOST
append_arg_if_nonempty --webots-camera-port WEBOTS_CAMERA_PORT
append_arg_if_nonempty --webots-camera-encoding WEBOTS_CAMERA_ENCODING
append_arg_if_nonempty --webots-camera-idle-reconnect WEBOTS_CAMERA_IDLE_RECONNECT
append_arg_if_nonempty --webots-detection-stale WEBOTS_DETECTION_STALE
append_arg_if_nonempty --yolo-model YOLO_MODEL_PATH
append_arg_if_nonempty --yolo-confidence YOLO_CONFIDENCE
append_arg_if_nonempty --yolo-imgsz YOLO_IMGSZ
append_arg_if_set --gate-class-names YOLO_GATE_CLASS_NAMES
append_arg_if_set --gate-class-ids YOLO_GATE_CLASS_IDS
append_arg_if_nonempty --yolo-device YOLO_DEVICE
append_arg_if_nonempty --gate-selector-min-seek-confidence GATE_SELECTOR_MIN_SEEK_CONFIDENCE
append_arg_if_nonempty --gate-selector-min-track-confidence GATE_SELECTOR_MIN_TRACK_CONFIDENCE
append_arg_if_nonempty --gate-selector-min-area-ratio GATE_SELECTOR_MIN_AREA_RATIO
append_arg_if_nonempty --gate-selector-min-aspect-ratio GATE_SELECTOR_MIN_ASPECT_RATIO
append_arg_if_nonempty --gate-selector-max-aspect-ratio GATE_SELECTOR_MAX_ASPECT_RATIO
append_arg_if_nonempty --gate-selector-min-appearance-score GATE_SELECTOR_MIN_APPEARANCE_SCORE
append_arg_if_nonempty --gate-selector-appearance-weight GATE_SELECTOR_APPEARANCE_WEIGHT
append_arg_if_nonempty --gate-selector-stable-window GATE_SELECTOR_STABLE_WINDOW
append_arg_if_nonempty --gate-selector-required-stable GATE_SELECTOR_REQUIRED_STABLE
append_arg_if_nonempty --mission-max-detection-age MISSION_MAX_DETECTION_AGE
append_arg_if_nonempty --mission-required-detection-ticks MISSION_REQUIRED_DETECTION_TICKS
append_arg_if_nonempty --mission-center-dwell MISSION_CENTER_DWELL
append_arg_if_nonempty --mission-center-clearance-required MISSION_CENTER_CLEARANCE_REQUIRED
append_arg_if_nonempty --mission-center-lost-grace-ticks MISSION_CENTER_LOST_GRACE_TICKS
append_arg_if_nonempty --mission-seek-yaw-rate MISSION_SEEK_YAW_RATE
append_arg_if_nonempty --mission-gate-pass-distance MISSION_GATE_PASS_DISTANCE
append_arg_if_nonempty --mission-gate-pass-speed MISSION_GATE_PASS_SPEED
append_arg_if_nonempty --mission-next-gate-acquire-speed MISSION_NEXT_GATE_ACQUIRE_SPEED
append_arg_if_nonempty --mission-next-gate-clear-distance MISSION_NEXT_GATE_CLEAR_DISTANCE
append_arg_if_nonempty --mission-next-gate-min-area MISSION_NEXT_GATE_MIN_AREA
append_arg_if_nonempty --mission-gate-ready-area MISSION_GATE_READY_AREA
append_arg_if_nonempty --mission-next-gate-max-distance MISSION_NEXT_GATE_MAX_DISTANCE
append_arg_if_nonempty --mission-next-gate-timeout MISSION_NEXT_GATE_TIMEOUT
append_arg_if_nonempty --mission-brake-settle MISSION_BRAKE_SETTLE
append_arg_if_nonempty --mission-brake-ramp MISSION_BRAKE_RAMP
append_arg_if_nonempty --mission-final-exit-distance MISSION_FINAL_EXIT_DISTANCE
append_arg_if_nonempty --mission-final-exit-speed MISSION_FINAL_EXIT_SPEED
append_arg_if_nonempty --visual-frame-width VISUAL_FRAME_WIDTH
append_arg_if_nonempty --visual-frame-height VISUAL_FRAME_HEIGHT
append_arg_if_nonempty --visual-min-confidence VISUAL_MIN_CONFIDENCE
append_arg_if_nonempty --visual-filter-alpha VISUAL_FILTER_ALPHA
append_arg_if_nonempty --visual-command-filter-alpha VISUAL_COMMAND_FILTER_ALPHA
append_arg_if_nonempty --visual-center-deadband-x VISUAL_CENTER_DEADBAND_X
append_arg_if_nonempty --visual-center-deadband-y VISUAL_CENTER_DEADBAND_Y
append_arg_if_nonempty --visual-aligned-error-x VISUAL_ALIGNED_ERROR_X
append_arg_if_nonempty --visual-aligned-error-y VISUAL_ALIGNED_ERROR_Y
append_arg_if_nonempty --visual-pass-target-offset-x VISUAL_PASS_TARGET_OFFSET_X
append_arg_if_nonempty --visual-pass-target-offset-y VISUAL_PASS_TARGET_OFFSET_Y
append_arg_if_nonempty --visual-pass-clearance-left VISUAL_PASS_CLEARANCE_LEFT
append_arg_if_nonempty --visual-pass-clearance-right VISUAL_PASS_CLEARANCE_RIGHT
append_arg_if_nonempty --visual-pass-clearance-up VISUAL_PASS_CLEARANCE_UP
append_arg_if_nonempty --visual-pass-clearance-down VISUAL_PASS_CLEARANCE_DOWN
append_arg_if_nonempty --visual-max-error-for-forward VISUAL_MAX_ERROR_FOR_FORWARD
append_arg_if_nonempty --visual-min-forward-speed VISUAL_MIN_FORWARD_SPEED
append_arg_if_nonempty --visual-max-forward-speed VISUAL_MAX_FORWARD_SPEED
append_arg_if_nonempty --visual-lateral-kp VISUAL_LATERAL_KP
append_arg_if_nonempty --visual-vertical-kp VISUAL_VERTICAL_KP
append_arg_if_nonempty --visual-yaw-kp VISUAL_YAW_KP
append_arg_if_nonempty --visual-max-lateral-speed VISUAL_MAX_LATERAL_SPEED
append_arg_if_nonempty --visual-max-vertical-speed VISUAL_MAX_VERTICAL_SPEED
append_arg_if_nonempty --visual-max-yaw-rate VISUAL_MAX_YAW_RATE

if [[ "${WEBOTS_DIAGNOSTICS_WINDOW:-0}" == "1" ]]; then
  cmd+=(--webots-diagnostics-window)
fi

if [[ "${MISSION_BRAKE_ALTITUDE_HOLD:-0}" == "1" ]]; then
  cmd+=(--mission-brake-altitude-hold)
fi

if [[ "${SEND_COMMANDS:-0}" == "1" ]]; then
  cmd+=(--send-commands)
else
  echo "SEND_COMMANDS=0: running dry-run only; no MAVLink motion commands will be sent." >&2
fi

exec "${cmd[@]}"

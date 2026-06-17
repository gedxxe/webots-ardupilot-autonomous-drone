#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${REPO_ROOT}/configs/autonomy_runtime.env"

CONFIG_KEYS=(
  MAVLINK_CONNECTION
  DETECTOR
  SEND_COMMANDS
  LOOP_HZ
  MAX_RUNTIME
  COURSE_FORWARD_X
  COURSE_FORWARD_Y
  WEBOTS_CAMERA_HOST
  WEBOTS_CAMERA_PORT
  WEBOTS_CAMERA_ENCODING
  YOLO_CONFIDENCE
  YOLO_IMGSZ
  YOLO_GATE_CLASS_NAMES
  YOLO_GATE_CLASS_IDS
  YOLO_DEVICE
  YOLO_MODEL_PATH
  YOLO_CONFIG_DIR
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

MAVLINK_CONNECTION="${MAVLINK_CONNECTION:-udp:127.0.0.1:14550}"
DETECTOR="${DETECTOR:-synthetic}"
SEND_COMMANDS="${SEND_COMMANDS:-0}"
LOOP_HZ="${LOOP_HZ:-20}"
MAX_RUNTIME="${MAX_RUNTIME:-180}"
COURSE_FORWARD_X="${COURSE_FORWARD_X:-1.0}"
COURSE_FORWARD_Y="${COURSE_FORWARD_Y:-0.0}"
WEBOTS_CAMERA_HOST="${WEBOTS_CAMERA_HOST:-127.0.0.1}"
WEBOTS_CAMERA_PORT="${WEBOTS_CAMERA_PORT:-5599}"
WEBOTS_CAMERA_ENCODING="${WEBOTS_CAMERA_ENCODING:-gray8}"
YOLO_CONFIDENCE="${YOLO_CONFIDENCE:-0.35}"
YOLO_IMGSZ="${YOLO_IMGSZ:-640}"
YOLO_GATE_CLASS_NAMES="${YOLO_GATE_CLASS_NAMES:-}"
YOLO_GATE_CLASS_IDS="${YOLO_GATE_CLASS_IDS:-0}"
YOLO_DEVICE="${YOLO_DEVICE:-cpu}"
YOLO_MODEL_PATH="${YOLO_MODEL_PATH:-}"

DEFAULT_YOLO_MODEL_PATH="${REPO_ROOT}/models/gate_yolov8n_best.pt"
if [[ -z "${YOLO_MODEL_PATH}" && "${DETECTOR}" == "webots-yolo" && -f "${DEFAULT_YOLO_MODEL_PATH}" ]]; then
  YOLO_MODEL_PATH="${DEFAULT_YOLO_MODEL_PATH}"
fi

if [[ "${DETECTOR}" == "webots-yolo" ]]; then
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
  --connection "${MAVLINK_CONNECTION}"
  --detector "${DETECTOR}"
  --loop-hz "${LOOP_HZ}"
  --max-runtime "${MAX_RUNTIME}"
  --course-forward-x "${COURSE_FORWARD_X}"
  --course-forward-y "${COURSE_FORWARD_Y}"
  --webots-camera-host "${WEBOTS_CAMERA_HOST}"
  --webots-camera-port "${WEBOTS_CAMERA_PORT}"
  --webots-camera-encoding "${WEBOTS_CAMERA_ENCODING}"
  --yolo-confidence "${YOLO_CONFIDENCE}"
  --yolo-imgsz "${YOLO_IMGSZ}"
  --gate-class-names "${YOLO_GATE_CLASS_NAMES}"
  --gate-class-ids "${YOLO_GATE_CLASS_IDS}"
)

if [[ -n "${YOLO_MODEL_PATH:-}" ]]; then
  cmd+=(--yolo-model "${YOLO_MODEL_PATH}")
fi

if [[ -n "${YOLO_DEVICE}" ]]; then
  cmd+=(--yolo-device "${YOLO_DEVICE}")
fi

if [[ "${SEND_COMMANDS}" == "1" ]]; then
  cmd+=(--send-commands)
else
  echo "SEND_COMMANDS=0: running dry-run only; no MAVLink motion commands will be sent." >&2
fi

exec "${cmd[@]}"

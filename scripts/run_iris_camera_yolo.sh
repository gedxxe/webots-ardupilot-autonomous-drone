#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Profile for the vendored ArduPilot iris_camera.wbt world. Keep defaults
# dry-run unless the caller explicitly sets SEND_COMMANDS=1.
export DETECTOR="${DETECTOR:-webots-yolo}"
export YOLO_MODEL_PATH="${YOLO_MODEL_PATH:-${REPO_ROOT}/models/gate_yolov8n_best.pt}"
export YOLO_GATE_CLASS_NAMES="${YOLO_GATE_CLASS_NAMES:-}"
export YOLO_GATE_CLASS_IDS="${YOLO_GATE_CLASS_IDS:-0}"
export YOLO_DEVICE="${YOLO_DEVICE:-cpu}"
export WEBOTS_CAMERA_HOST="${WEBOTS_CAMERA_HOST:-127.0.0.1}"
export WEBOTS_CAMERA_PORT="${WEBOTS_CAMERA_PORT:-5599}"
export WEBOTS_CAMERA_ENCODING="${WEBOTS_CAMERA_ENCODING:-gray8}"
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-${REPO_ROOT}/.tmp_ultralytics}"
export SEND_COMMANDS="${SEND_COMMANDS:-0}"

exec "${REPO_ROOT}/scripts/run_autonomy_sitl.sh"

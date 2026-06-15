#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${REPO_ROOT}/configs/autonomy_runtime.env"

if [[ -f "${ENV_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
fi

MAVLINK_CONNECTION="${MAVLINK_CONNECTION:-udp:127.0.0.1:14550}"
DETECTOR="${DETECTOR:-synthetic}"
SEND_COMMANDS="${SEND_COMMANDS:-0}"
LOOP_HZ="${LOOP_HZ:-20}"
MAX_RUNTIME="${MAX_RUNTIME:-180}"
COURSE_FORWARD_X="${COURSE_FORWARD_X:-1.0}"
COURSE_FORWARD_Y="${COURSE_FORWARD_Y:-0.0}"

resolve_autonomy_launcher() {
  if command -v drone-autonomy >/dev/null 2>&1; then
    autonomy_launcher=(drone-autonomy)
    return
  fi

  local python_bin=""
  if [[ -n "${PYTHON_BIN:-}" ]]; then
    python_bin="${PYTHON_BIN}"
  elif [[ -n "${VIRTUAL_ENV:-}" && -x "${VIRTUAL_ENV}/bin/python" ]]; then
    python_bin="${VIRTUAL_ENV}/bin/python"
  elif [[ -x "${REPO_ROOT}/.venv/bin/python" ]]; then
    python_bin="${REPO_ROOT}/.venv/bin/python"
  elif command -v python3 >/dev/null 2>&1; then
    python_bin="$(command -v python3)"
  else
    echo "drone-autonomy is not in PATH and python3 was not found." >&2
    echo "Install this repo first: python3 -m venv .venv && source .venv/bin/activate && pip install -e '.[dev]'" >&2
    exit 127
  fi

  if ! "${python_bin}" -c "from pymavlink import mavutil" >/dev/null 2>&1; then
    cat >&2 <<EOF
drone-autonomy is not in PATH, and the selected Python cannot import pymavlink.

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
)

if [[ "${SEND_COMMANDS}" == "1" ]]; then
  cmd+=(--send-commands)
else
  echo "SEND_COMMANDS=0: running dry-run only; no MAVLink motion commands will be sent." >&2
fi

exec "${cmd[@]}"

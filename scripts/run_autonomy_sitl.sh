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

cmd=(
  drone-autonomy
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

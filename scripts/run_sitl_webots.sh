#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${REPO_ROOT}/configs/sitl_webots.env"

if [[ -f "${ENV_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
else
  echo "Missing ${ENV_FILE}. Copy configs/sitl_webots.env.example first." >&2
  exit 1
fi

: "${ARDUPILOT_HOME:?ARDUPILOT_HOME is required}"
: "${WEBOTS_EXAMPLE_RELATIVE:?WEBOTS_EXAMPLE_RELATIVE is required}"
: "${WEBOTS_PARAM_FILE:?WEBOTS_PARAM_FILE is required}"
: "${MAVLINK_OUT:?MAVLINK_OUT is required}"
: "${ARDUPILOT_VEHICLE:=ArduCopter}"
: "${ARDUPILOT_MODEL:=webots-python}"

PARAM_PATH="${ARDUPILOT_HOME}/${WEBOTS_EXAMPLE_RELATIVE}/${WEBOTS_PARAM_FILE}"

if [[ ! -x "${ARDUPILOT_HOME}/Tools/autotest/sim_vehicle.py" ]]; then
  echo "sim_vehicle.py not found under ${ARDUPILOT_HOME}" >&2
  exit 1
fi

if [[ ! -f "${PARAM_PATH}" ]]; then
  echo "Webots param file not found: ${PARAM_PATH}" >&2
  exit 1
fi

cd "${ARDUPILOT_HOME}"

./Tools/autotest/sim_vehicle.py \
  -v "${ARDUPILOT_VEHICLE}" \
  -w \
  --model "${ARDUPILOT_MODEL}" \
  --add-param-file="${PARAM_PATH}" \
  --out="${MAVLINK_OUT}"

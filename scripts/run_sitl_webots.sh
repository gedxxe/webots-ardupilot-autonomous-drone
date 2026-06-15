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
: "${WEBOTS_PARAM_FILE:?WEBOTS_PARAM_FILE is required}"
: "${MAVLINK_OUT:?MAVLINK_OUT is required}"
: "${ARDUPILOT_VEHICLE:=ArduCopter}"
: "${ARDUPILOT_MODEL:=webots-python}"

if [[ -z "${WEBOTS_EXAMPLE_HOME:-}" ]]; then
  : "${WEBOTS_EXAMPLE_RELATIVE:?WEBOTS_EXAMPLE_RELATIVE is required when WEBOTS_EXAMPLE_HOME is not set}"
  WEBOTS_EXAMPLE_HOME="${ARDUPILOT_HOME}/${WEBOTS_EXAMPLE_RELATIVE}"
fi

PARAM_PATH="${WEBOTS_EXAMPLE_HOME}/${WEBOTS_PARAM_FILE}"
WORLD_PATH="${WEBOTS_EXAMPLE_HOME}/${WEBOTS_WORLD:-worlds/iris.wbt}"

if [[ ! -x "${ARDUPILOT_HOME}/Tools/autotest/sim_vehicle.py" ]]; then
  echo "sim_vehicle.py not found under ${ARDUPILOT_HOME}" >&2
  exit 1
fi

if [[ ! -f "${PARAM_PATH}" ]]; then
  echo "Webots param file not found: ${PARAM_PATH}" >&2
  exit 1
fi

if [[ ! -f "${WORLD_PATH}" ]]; then
  echo "Webots world file not found: ${WORLD_PATH}" >&2
  exit 1
fi

cd "${ARDUPILOT_HOME}"

echo "Using ArduPilot home: ${ARDUPILOT_HOME}"
echo "Open this Webots world before SITL connects: ${WORLD_PATH}"
echo "Using Webots params: ${PARAM_PATH}"

./Tools/autotest/sim_vehicle.py \
  -v "${ARDUPILOT_VEHICLE}" \
  -w \
  --model "${ARDUPILOT_MODEL}" \
  --add-param-file="${PARAM_PATH}" \
  --out="${MAVLINK_OUT}"

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
MAVLINK_OUT_EXTRA="${MAVLINK_OUT_EXTRA-udp:127.0.0.1:14551}"

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
echo "Primary MAVLink out: ${MAVLINK_OUT}"

sim_vehicle_cmd=(
  ./Tools/autotest/sim_vehicle.py
  -v "${ARDUPILOT_VEHICLE}"
  -w
  --model "${ARDUPILOT_MODEL}"
  --add-param-file="${PARAM_PATH}"
  --out="${MAVLINK_OUT}"
)

if [[ -n "${MAVLINK_OUT_EXTRA:-}" ]]; then
  IFS=',' read -r -a extra_mavlink_outs <<< "${MAVLINK_OUT_EXTRA}"
  for extra_out in "${extra_mavlink_outs[@]}"; do
    extra_out="${extra_out//[[:space:]]/}"
    if [[ -n "${extra_out}" ]]; then
      echo "Extra MAVLink out: ${extra_out}"
      sim_vehicle_cmd+=(--out="${extra_out}")
    fi
  done
fi

exec "${sim_vehicle_cmd[@]}"

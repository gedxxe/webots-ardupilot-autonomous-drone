#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_ENV_FILE="${REPO_ROOT}/configs/raspi_runtime.env"
EXAMPLE_ENV_FILE="${REPO_ROOT}/configs/raspi_runtime.env.example"

if [[ -f "${LOCAL_ENV_FILE}" ]]; then
  export AUTONOMY_ENV_FILE="${LOCAL_ENV_FILE}"
else
  export AUTONOMY_ENV_FILE="${EXAMPLE_ENV_FILE}"
  cat >&2 <<EOF
configs/raspi_runtime.env was not found.
Using the tracked dry-run example instead:
  ${EXAMPLE_ENV_FILE}

Create a local hardware config before real tests:
  cp configs/raspi_runtime.env.example configs/raspi_runtime.env
EOF
fi

# This script is a hardware scaffold, not a flight enable switch. Inline
# SEND_COMMANDS=1 is still possible for a deliberate test, but the template and
# default path remain dry-run.
export AUTONOMY_PROFILE="${AUTONOMY_PROFILE:-}"
export SEND_COMMANDS="${SEND_COMMANDS:-0}"

exec "${REPO_ROOT}/scripts/run_autonomy_sitl.sh"

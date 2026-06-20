#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Profile for the vendored ArduPilot iris_camera.wbt world. The generic runner
# reads configs/autonomy_runtime.env first, then applies this profile only for
# profile-owned defaults such as detector mode and diagnostics. Model path,
# thresholds, class filters, and mission tuning remain configurable from the env
# file or inline overrides.
export AUTONOMY_PROFILE="${AUTONOMY_PROFILE:-iris-camera-yolo}"

exec "${REPO_ROOT}/scripts/run_autonomy_sitl.sh"

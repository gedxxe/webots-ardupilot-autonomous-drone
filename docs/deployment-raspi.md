# Raspberry Pi Deployment Scaffold

This document prepares the repository for later Raspberry Pi deployment without
claiming real-hardware flight readiness. The current hardware path can connect
to Pixhawk over MAVLink, read C920 frames through OpenCV, run YOLO plus
`GateTargetSelector`, and stay in dry-run while diagnostics are validated.
Command-sending flight remains blocked until the safety gates below are closed.

Production decision:

```text
OS: Raspberry Pi OS 64-bit standard with desktop
Model runtime target: NCNN-exported YOLO model
Safety policy: fail closed; do not send motion commands on ambiguous state
```

The standard desktop image is accepted even though it is heavier than Lite
because it simplifies field diagnostics, camera preview, and OpenCV windows
during early hardware tuning.

## Quick Run Order

Run these from the repository root on the Raspberry Pi:

```bash
source .venv/bin/activate

# 1. Offline config check. No MAVLink, no camera, no arming.
python scripts/preflight_check.py --profile raspi

# 2. Camera-only probe.
python scripts/probe_opencv_camera.py --source /dev/video0 --backend v4l2

# 3. Camera + YOLO + GateTargetSelector probe. No MAVLink.
python scripts/probe_opencv_yolo.py \
  --source /dev/video0 \
  --backend v4l2 \
  --model models/gate_yolov8n_best_ncnn_model \
  --diagnostics-window

# 4. Hardware scaffold. Dry-run by default.
bash scripts/run_raspi_hardware.sh
```

Optional structured log:

```text
AUTONOMY_LOG_JSONL="${REPO_ROOT}/logs/raspi-dry-run.jsonl"
```

`preflight_check.py` and probe scripts are diagnostics only. They do not replace
props-off checks, operator kill switch, EKF checks, or ArduPilot failsafes.

## Current Hardware Status

Implemented:

- serial MAVLink endpoint support through `MAVLINK_CONNECTION`,
- serial baud support through `MAVLINK_BAUD`,
- Raspberry Pi runtime env template,
- hardware launcher wrapper that defaults to dry-run,
- OpenCV camera frame source for C920 dry-run validation,
- `opencv-yolo` detector mode that reuses YOLO and `GateTargetSelector`,
- NCNN export helper at `scripts/export_yolo_ncnn.py`,
- offline profile preflight at `scripts/preflight_check.py`,
- OpenCV camera plus YOLO probe at `scripts/probe_opencv_yolo.py`,
- optional JSONL runtime log through `AUTONOMY_LOG_JSONL`,
- MAVLink stale-telemetry fail-closed guards,
- tracked `COMMAND_ACK` retry/fail-closed policy for arm/disarm, takeoff, and
  land,
- documentation for EKF ownership and safety boundaries.

Not implemented yet:

- hardware-validated C920 camera tuning,
- measured Raspberry Pi 5 inference FPS/latency using the exported NCNN model,
- hardware flight safety procedure validated on props-off / tethered / open-area
  tests,
- heartbeat-loss field procedure and hardware validation,
- real Pixhawk serial validation of the implemented `COMMAND_ACK` retry policy,
- `COMMAND_ACK` tracking for `set_mode`; mode transition is currently validated
  from heartbeat telemetry.

## Hardware Assumptions

Target stack:

```text
Pixhawk 6C Mini
Raspberry Pi 5
Logitech C920 Pro RGB camera
GPS
rangefinder + MTF01 optical flow
hexacopter frame
```

Target OS:

```text
Raspberry Pi OS 64-bit standard with desktop
```

Do not use Ubuntu 24.04 for the first production path unless a later dependency
forces it. Ubuntu remains useful for Webots/SITL development on the PC side.

Default MAVLink endpoint:

```text
MAVLINK_CONNECTION="/dev/ttyACM0"
MAVLINK_BAUD="115200"
```

If Linux exposes the Pixhawk as the second USB ACM device, use:

```text
MAVLINK_CONNECTION="/dev/ttyACM1"
```

The baud must match the ArduPilot serial configuration for the selected USB or
telemetry port. `115200` is the conservative starting default, not a universal
law.

## One-Time Raspberry Pi Setup

From the repository root on the Raspberry Pi:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip git v4l-utils

python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e ".[dev,vision]"
```

If OpenCV camera access needs more system packages on the chosen OS image,
install them before enabling `DETECTOR="opencv-yolo"`. Do not enable motion
commands until a simple frame-grab test and dry-run diagnostics are repeatable.

## NCNN Model Export

Production inference on Raspberry Pi 5 should use an NCNN-exported model, not
the raw `.pt` model as the final runtime target.

Export from the repo root:

```bash
source .venv/bin/activate
pip install "ultralytics[export]"
python scripts/export_yolo_ncnn.py --model models/gate_yolov8n_best.pt --imgsz 640
```

Expected output:

```text
ncnn_export_ok path=models/gate_yolov8n_best_ncnn_model
```

The exported directory is ignored by git. When the C920 detector mode is being
validated in dry-run, point `YOLO_MODEL_PATH` at that directory:

```text
YOLO_MODEL_PATH="${REPO_ROOT}/models/gate_yolov8n_best_ncnn_model"
```

Keep the class filter active:

```text
YOLO_GATE_CLASS_NAMES="Goals-Detection"
YOLO_GATE_CLASS_IDS="3"
```

Before any motion test, measure and record:

- camera FPS,
- YOLO inference latency,
- selected target age,
- raw class counts,
- end-to-end detection-to-command latency.

## Hardware Runtime Config

Create a local config file:

```bash
cp configs/raspi_runtime.env.example configs/raspi_runtime.env
nano configs/raspi_runtime.env
```

Keep this default during early tests:

```text
DETECTOR="none"
SEND_COMMANDS="0"
COMMAND_ACK_REQUIRED="1"
COMMAND_ACK_TIMEOUT="1.0"
COMMAND_ACK_MAX_RETRIES="2"
```

`DETECTOR="none"` means the mission will not receive gate detections. That is
intentional until the C920/OpenCV path has been validated locally on the Pi.

The template documents the NCNN target model path, but leaves it commented until
the exported model exists and diagnostics prove it loads correctly.

To start C920 perception in dry-run after exporting the model, set:

```text
DETECTOR="opencv-yolo"
YOLO_MODEL_PATH="${REPO_ROOT}/models/gate_yolov8n_best_ncnn_model"
SEND_COMMANDS="0"
```

Keep diagnostics enabled during hardware tuning:

```text
OPENCV_DIAGNOSTICS_WINDOW="1"
```

If `OPENCV_CAMERA_SOURCE="0"` does not open the C920, use `v4l2-ctl` to inspect
available devices and set the source explicitly, for example:

```text
OPENCV_CAMERA_SOURCE="/dev/video0"
```

Probe the camera without MAVLink or YOLO first:

```bash
python scripts/probe_opencv_camera.py \
  --source /dev/video0 \
  --backend v4l2 \
  --width 640 \
  --height 480 \
  --fps 30
```

Expected:

```text
opencv_camera_ok source=opencv:/dev/video0 size=640x480 encoding=rgb8
```

Probe camera plus YOLO without MAVLink:

```bash
python scripts/probe_opencv_yolo.py \
  --source /dev/video0 \
  --backend v4l2 \
  --model models/gate_yolov8n_best_ncnn_model \
  --diagnostics-window
```

Run the hardware scaffold:

```bash
source .venv/bin/activate
bash scripts/run_raspi_hardware.sh
```

The wrapper loads `configs/raspi_runtime.env` when present. If it is missing, it
falls back to the tracked example in dry-run mode and prints a warning.

Side-by-side rule:

```text
SITL/Webots tuning:       configs/autonomy_runtime.env
Raspberry Pi tuning:      configs/raspi_runtime.env
SITL launcher:            scripts/run_iris_camera_yolo.sh
Raspberry Pi launcher:    scripts/run_raspi_hardware.sh
```

Do not copy Raspberry Pi OpenCV variables into `configs/autonomy_runtime.env`
unless you intentionally want to run `opencv-yolo` on a PC. Do not put Webots
TCP camera variables into `configs/raspi_runtime.env`; hardware uses
`OPENCV_*` camera variables.

## MAVLink Smoke Tests

Heartbeat check:

```bash
source .venv/bin/activate
drone-autonomy --connection /dev/ttyACM0 --baud 115200 --mode heartbeat
```

If that fails and `/dev/ttyACM1` exists:

```bash
drone-autonomy --connection /dev/ttyACM1 --baud 115200 --mode heartbeat
```

Listen to messages:

```bash
drone-autonomy --connection /dev/ttyACM0 --baud 115200 --mode listen --count 20
```

Expected before any flight command testing:

- heartbeat arrives,
- `LOCAL_POSITION_NED` or equivalent local-position telemetry is available,
- mode and armed state decode correctly,
- rangefinder/optical-flow/GPS health is visible in ArduPilot logs or GCS.

## Safety Boundary

Do not set `SEND_COMMANDS=1` on hardware just because SITL works.

Before real command sending, the missing hardware procedure must define:

- props-off bench test,
- manual RC takeover,
- kill switch / disarm path,
- guided mode entry and exit,
- geofence or test-area boundary,
- battery failsafe behavior,
- heartbeat-loss behavior,
- log collection,
- one-axis sign tests with the vehicle restrained or safely isolated.

The current script intentionally does not bypass those steps.

## Fail-Closed Production Rules

The production posture is zero tolerance for silent or ambiguous failures, not a
claim that the system has zero possible error. The companion should refuse to
send motion commands when any of these are true:

- no fresh heartbeat,
- no fresh fused local-position telemetry,
- camera frame source missing or stale,
- detector output stale,
- YOLO class filter empty,
- selected class is not the configured gate class,
- target selector is unstable,
- tracked mission command is rejected by `COMMAND_ACK`,
- tracked mission command does not receive `COMMAND_ACK` before retry budget is
  exhausted,
- safety operator has not explicitly enabled `SEND_COMMANDS=1`.

Heartbeat and local-position stale guards are implemented through:

```text
MAVLINK_HEARTBEAT_STALE
MAVLINK_LOCAL_POSITION_STALE
COMMAND_ACK_REQUIRED
COMMAND_ACK_TIMEOUT
COMMAND_ACK_MAX_RETRIES
```

ACK tracking applies to mission `COMMAND_LONG` commands that should be accepted
or rejected explicitly by ArduPilot:

```text
arm/disarm
takeoff
land
```

Velocity setpoints are streamed as `SET_POSITION_TARGET_LOCAL_NED` and do not
produce `COMMAND_ACK`. Guided mode entry currently uses pymavlink's `set_mode()`
helper; the mission validates the resulting mode through heartbeat telemetry
instead of treating it as a tracked ACK command.

Field safety procedure gates are still not complete, so hardware testing must
stay dry-run or props-off.

## Sensor Fusion Policy

The Raspberry Pi companion must not raw-blend GPS, rangefinder, and optical-flow
samples inside the mission state machine.

Recommended split:

```text
ArduPilot EKF:
  attitude, altitude, local position, GPS/rangefinder/flow fusion, failsafes

Raspberry Pi companion:
  camera inference, target selection, mission phase logic, body-frame setpoints

MAVLink adapter:
  convert VehicleCommand to guided commands, read fused telemetry
```

Mission fields such as altitude and forward position must come from fused
telemetry, currently represented by `LOCAL_POSITION_NED` in the adapter.

## C920/OpenCV Detector

The hardware camera adapter reuses the same perception contract:

```text
C920 RGB frame
-> YOLO raw GateCandidate list
-> GateTargetSelector
-> GateDetection | None
-> MissionTelemetry.gate_detection
```

Implementation constraints:

- do not import OpenCV or Ultralytics into `GateAutonomyMission`,
- do not bypass `GateTargetSelector`,
- keep class filtering fail-closed,
- keep diagnostics available before enabling motion,
- keep the simulation `webots-yolo` path working unchanged.

Detector name:

```text
opencv-yolo
```

Keep this mode in dry-run until it has been tested with the real camera frame
source, NCNN model, diagnostics overlay, and class filter on the Pi.

## Production Readiness Checklist

This checklist is intentionally not marked complete yet:

- [x] C920/OpenCV frame source implemented.
- [ ] C920 frame source can show realtime diagnostics on the actual Pi.
- [ ] NCNN model exported and loaded successfully on Raspberry Pi 5.
- [ ] Raspberry Pi 5 FPS/latency measured with the NCNN model.
- [ ] YOLO class filter confirmed for the deployed model.
- [ ] OpenCV diagnostics show only `Goals-Detection` as the selected target.
- [ ] Dry-run hardware mission loop receives fused local position.
- [ ] Body-frame command signs are verified safely.
- [x] `COMMAND_ACK` and retry policy are implemented for tracked mission
      `COMMAND_LONG` commands.
- [ ] Implemented ACK policy is validated over the real Pixhawk serial path.
- [ ] Heartbeat-loss field procedure is validated with real hardware.
- [ ] Hardware command tests are performed with a documented safety procedure.

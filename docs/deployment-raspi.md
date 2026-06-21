# Raspberry Pi Deployment Scaffold

This document prepares the repository for later Raspberry Pi deployment without
claiming real-hardware flight readiness. The current hardware path is a dry-run
scaffold: it can connect to Pixhawk over MAVLink and run the same autonomy
runtime shape, but the Logitech C920/OpenCV detector is not implemented yet.

## Current Hardware Status

Implemented:

- serial MAVLink endpoint support through `MAVLINK_CONNECTION`,
- serial baud support through `MAVLINK_BAUD`,
- Raspberry Pi runtime env template,
- hardware launcher wrapper that defaults to dry-run,
- documentation for EKF ownership and safety boundaries.

Not implemented yet:

- real Logitech C920/OpenCV frame source,
- hardware YOLO detector mode,
- hardware flight safety procedure validated on props-off / tethered / open-area
  tests,
- heartbeat-loss failsafe inside this companion process,
- `COMMAND_ACK` retry policy.

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
sudo apt install -y python3-venv python3-pip git

python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e ".[dev,vision]"
```

If OpenCV camera access needs system packages on the chosen OS image, install
them before implementing the C920 adapter. Do not add a hardware detector mode
until a simple frame-grab test is repeatable.

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
```

`DETECTOR="none"` means the mission will not receive gate detections. That is
intentional until the real C920/OpenCV adapter exists.

Run the hardware scaffold:

```bash
source .venv/bin/activate
bash scripts/run_raspi_hardware.sh
```

The wrapper loads `configs/raspi_runtime.env` when present. If it is missing, it
falls back to the tracked example in dry-run mode and prints a warning.

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

## Future Phase: C920/OpenCV Detector

The hardware camera adapter should reuse the same perception contract:

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

Recommended future detector name:

```text
opencv-yolo
```

Do not add that mode until it can be tested with a real camera frame source.

## Production Readiness Checklist

This checklist is intentionally not marked complete yet:

- [ ] C920 frame source implemented.
- [ ] C920 frame source can show realtime diagnostics.
- [ ] YOLO class filter confirmed for the deployed model.
- [ ] OpenCV diagnostics show only `Goals-Detection` as the selected target.
- [ ] Dry-run hardware mission loop receives fused local position.
- [ ] Body-frame command signs are verified safely.
- [ ] `COMMAND_ACK` and retry policy are implemented.
- [ ] Lost-heartbeat process failsafe is implemented.
- [ ] Hardware command tests are performed with a documented safety procedure.

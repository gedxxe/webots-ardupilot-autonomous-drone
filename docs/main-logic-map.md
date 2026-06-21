# Main Logic Map

Dokumen ini memetakan jalur utama kode simulasi agar runtime tidak terlihat seperti black box.

## Entry Point

CLI entry point:

```text
drone-autonomy
```

Source:

```text
src/drone_autonomy/cli.py
```

Mode yang tersedia:

- `heartbeat`: cek MAVLink heartbeat.
- `listen`: print raw MAVLink messages.
- `autonomy`: jalankan runtime loop.

## Runtime Loop

Source:

```text
src/drone_autonomy/runtime/autonomy_loop.py
```

Loop utama melakukan ini setiap tick:

```text
drain MAVLink messages
-> update telemetry adapter
-> get latest gate detection
-> build MissionTelemetry
-> mission.update(telemetry)
-> optionally send VehicleCommand to ArduPilot
-> print phase/status
```

Runtime boleh blocking karena runtime adalah process loop. Mission state machine tidak boleh blocking.

## Mission State Machine

Source:

```text
src/drone_autonomy/autonomy/mission.py
```

Mission input:

```text
MissionTelemetry(
  now_s,
  altitude_m,
  forward_position_m,
  mode,
  armed,
  landed,
  gate_detection,
)
```

Mission output:

```text
MissionOutput(
  phase,
  command,
  gate_index,
  detail,
  servo,
)
```

Mission phases:

```text
INIT
TAKEOFF
SEEK_GATE
CENTER_GATE
PASS_GATE
NEXT_GATE_ACQUIRE
BRAKE
FINAL_EXIT
LAND
COMPLETE
FAILSAFE
```

Important:

- `TAKEOFF` sends `MAV_CMD_NAV_TAKEOFF` to `1.0 m` and waits for telemetry
  settle; it does not command companion-side body-z velocity.
- `CENTER_GATE` keeps visual servoing active until the dwell timer and
  clearance validator both pass and the selected bbox reaches
  `MISSION_GATE_READY_AREA`.
- `PASS_GATE` is a committed forward-only body-velocity segment plus altitude
  hold. Lateral/yaw visual corrections are disabled during this committed pass.
- `NEXT_GATE_ACQUIRE` first clears the previous obstacle by forward distance,
  then uses `MISSION_NEXT_GATE_MIN_AREA` and `MISSION_GATE_READY_AREA` before
  counting gate 2 detections. It is not a hardcoded timed sprint.
- `BRAKE` after gate 2 detection reduces overshoot before centering.
- `FINAL_EXIT` measures forward distance, not altitude, then enters `BRAKE`
  before `LAND`.

## MAVLink Telemetry Adapter

Source:

```text
src/drone_autonomy/mavlink/telemetry.py
```

Responsibilities:

- Convert `HEARTBEAT` to mode and armed state.
- Convert `LOCAL_POSITION_NED` to altitude and forward position.
- Convert `EXTENDED_SYS_STATE` to landed state.

It does not fuse raw sensors. Fusion belongs to ArduPilot EKF or simulator-equivalent telemetry.

## MAVLink Command Adapter

Source:

```text
src/drone_autonomy/mavlink/commands.py
```

Responsibilities:

- Convert `VehicleCommand.set_mode()` to MAVLink mode command.
- Convert `VehicleCommand.arm_vehicle()` to arm/disarm command.
- Convert `VehicleCommand.takeoff()` to takeoff command.
- Convert `VehicleCommand.land()` to land command.
- Convert body velocity command to `SET_POSITION_TARGET_LOCAL_NED`.

Internal velocity convention:

```text
body_vx_m_s > 0: forward
body_vy_m_s > 0: right
body_vz_m_s > 0: down
yaw_rate_rad_s > 0: yaw right/clockwise
```

## Detector Layer

Current simulator wiring:

```text
src/drone_autonomy/perception/synthetic.py
```

Synthetic detector is only for wiring tests. It returns centered gate boxes based on mission phase.
It must keep a fake detection continuous across `SEEK_GATE -> CENTER_GATE`; if
it keys detections by phase, the mission will oscillate between seek and center.

Implemented Webots plus YOLO detector path:

```text
webots/worlds/iris_camera.wbt
-> TCP camera stream 127.0.0.1:5599
-> src/drone_autonomy/perception/webots_camera.py
-> src/drone_autonomy/perception/yolo.py
-> src/drone_autonomy/perception/target_selector.py
-> GateDetection
-> MissionTelemetry.gate_detection
```

`YoloGateDetector` extracts class-filtered raw candidates. `GateTargetSelector`
validates geometry and hollow-gate appearance, scores candidates, tracks the
previous target, smooths the selected bbox, and only then publishes a
`GateDetection`.

Current profile launcher:

```text
scripts/run_iris_camera_yolo.sh
-> AUTONOMY_PROFILE=iris-camera-yolo
-> scripts/run_autonomy_sitl.sh
-> configs/autonomy_runtime.env
-> models/gate_yolov8n_best.pt
-> YOLO class name Goals-Detection and class id 3
```

This repo's `iris_camera.wbt` profile requests `rgb24` from the vendored Webots
controller. `gray8` is still supported only for upstream-compatible fallback
worlds. A future real-hardware path should replace only the frame source with a
C920/OpenCV adapter.

The detector must not command the drone.

## Where Configuration Currently Lives

Runtime CLI flags:

```text
src/drone_autonomy/cli.py
```

`--connection` selects the MAVLink endpoint. `--baud` is passed to pymavlink
for serial endpoints such as `/dev/ttyACM0`; UDP SITL endpoints ignore that
setting.

Runtime shell env example:

```text
configs/autonomy_runtime.env.example
```

Raspberry Pi dry-run shell env example:

```text
configs/raspi_runtime.env.example
```

This file is only the git-tracked template. For real local tuning, copy it to:

```text
configs/autonomy_runtime.env
```

or pass one-shot overrides inline when launching a script.

Mission tuning defaults:

```text
GateMissionConfig in src/drone_autonomy/autonomy/mission.py
```

Runtime-exposed mission and clearance tuning:

```text
MISSION_CENTER_DWELL
MISSION_CENTER_CLEARANCE_REQUIRED
MISSION_REQUIRED_DETECTION_TICKS
MISSION_CENTER_LOST_GRACE_TICKS
MISSION_MAX_DETECTION_AGE
MISSION_SEEK_YAW_RATE
MISSION_GATE_PASS_DISTANCE
MISSION_GATE_PASS_SPEED
MISSION_NEXT_GATE_CLEAR_DISTANCE
MISSION_NEXT_GATE_ACQUIRE_SPEED
MISSION_NEXT_GATE_MIN_AREA
MISSION_GATE_READY_AREA
MISSION_BRAKE_SETTLE
MISSION_BRAKE_RAMP
MISSION_BRAKE_ALTITUDE_HOLD
GATE_SELECTOR_MIN_APPEARANCE_SCORE
GATE_SELECTOR_APPEARANCE_WEIGHT
VISUAL_FRAME_WIDTH
VISUAL_FRAME_HEIGHT
VISUAL_MAX_ERROR_FOR_FORWARD
VISUAL_PASS_TARGET_OFFSET_X/Y
VISUAL_PASS_CLEARANCE_LEFT/RIGHT/UP/DOWN
```

Code-level defaults live in `src/drone_autonomy/runtime/config.py` and the
domain config dataclasses. The tracked env example is the documented operator
preset, not a second hidden source of truth. Actual simulation tuning should be
done in `configs/autonomy_runtime.env` or through inline environment variables.
`scripts/run_autonomy_sitl.sh` reads that env file and only passes values that
are explicitly set into `src/drone_autonomy/cli.py`.

`scripts/run_raspi_hardware.sh` reuses the same generic runner with
`AUTONOMY_ENV_FILE=configs/raspi_runtime.env` when present. It defaults to
dry-run and `DETECTOR="none"` until a real C920/OpenCV detector mode is
implemented.

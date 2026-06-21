# Development Roadmap

## Stage 1: Simulator Baseline

- Done: vendored ArduPilot Webots baseline tree.
- Done: baseline Iris world path documented.
- Done: SITL launch script points ArduPilot at the vendored Webots params/world.
- User-validated: autonomy dry-run can run against SITL.

## Stage 2: Companion Telemetry

- Done: Python companion app receives heartbeat.
- Done: `listen` mode can inspect raw MAVLink messages.
- Done: runtime waits for `LOCAL_POSITION_NED` before mission decisions.
- Done: connection settings are centralized in config/env files.
- Done: serial MAVLink baud can be set with `MAVLINK_BAUD` / `--baud` for
  Raspberry Pi USB Pixhawk tests.

## Stage 3: Basic Autonomy Commands

- Done: guided mode, arm, takeoff, land, and body-frame velocity command helpers.
- Pending: `COMMAND_ACK` parsing, retry policy, and command timeout handling.

## Stage 4: Navigation Logic

- Done: deterministic mission state machine for the two-gate task.
- Done: local forward-distance projection from `LOCAL_POSITION_NED`.
- Done: adaptive next-gate acquire and brake-before-center.
- Pending: lost-heartbeat failsafe in the process runtime.
- Pending: automatic course-frame calibration.

## Stage 5: Sensors and Perception

- Done: synthetic detector for wiring tests.
- Done: Webots `iris_camera.wbt` TCP camera adapter.
- Done: YOLO-to-`GateDetection` adapter.
- Done: trained YOLOv8n gate model at `models/gate_yolov8n_best.pt`.
- Pending: validate the trained model against the actual `iris_camera.wbt`
  viewpoint and lighting.
- Pending: custom two-gate Webots world and gate assets.
- Done: true RGB Webots stream for the current `iris_camera.wbt` profile.
- Pending: real C920/OpenCV camera source.

## Stage 6: Hardware Readiness

- Done: mission/perception/adapter boundaries are separated.
- Done: Raspberry Pi dry-run env template and launcher scaffold.
- Done: paper-oriented mathematical foundations document for current behavior.
- Pending: C920/OpenCV hardware camera source.
- Pending: hardware safety procedure and validated command-sending launch profile.
- Pending: heartbeat-loss process failsafe and `COMMAND_ACK` retry policy.
- Required invariant: preserve the same high-level autonomy API across SITL and hardware.

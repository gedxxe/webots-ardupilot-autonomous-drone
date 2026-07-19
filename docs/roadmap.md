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
- Done: `COMMAND_ACK` parsing, retry policy, and command timeout handling for
  tracked mission `COMMAND_LONG` commands: arm/disarm, takeoff, and land.
- Done: accepted tracked commands are latched by exact command identity so
  repeated mission output does not resend takeoff/arm/land after acceptance.
- Pending: hardware validation of ACK behavior over the real Pixhawk serial
  path.
- Pending: optional `set_mode` ACK tracking. Current validation is telemetry
  based because the code uses pymavlink's `set_mode()` helper.

## Stage 4: Navigation Logic

- Done: deterministic mission state machine for the two-gate task.
- Done: local forward-distance projection from `LOCAL_POSITION_NED`.
- Done: adaptive next-gate acquire and brake-before-center.
- Done: MAVLink heartbeat and local-position stale guards fail closed before
  mission updates.
- Done: camera-frame and inference-progress stale guards pause mission updates
  and command hold without confusing a healthy no-candidate frame with a dead
  perception pipeline.
- Pending: full lost-heartbeat field failsafe policy with operator procedure.
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
- Done: OpenCV diagnostics overlays use background inference but keep HighGUI
  event processing and window cleanup on the runtime main thread.
- Done: real C920/OpenCV camera source for dry-run perception.
- Pending: validate C920/OpenCV behavior on Raspberry Pi 5 hardware.

## Stage 6: Hardware Readiness

- Done: mission/perception/adapter boundaries are separated.
- Done: Raspberry Pi dry-run env template and launcher scaffold.
- Done: paper-oriented mathematical foundations document for current behavior.
- Done: production deployment direction locked to Raspberry Pi OS 64-bit
  standard plus NCNN-exported YOLO model.
- Done: NCNN export helper for the trained gate model.
- Done: C920/OpenCV hardware camera source and `opencv-yolo` detector mode.
- Done: offline preflight checker for simulation and Raspberry Pi env profiles.
- Done: OpenCV camera plus YOLO probe that does not touch MAVLink.
- Done: optional JSONL runtime diagnostics logging.
- Done: tracked command ACK retry/fail-closed policy at the MAVLink adapter and
  runtime level.
- Pending: Raspberry Pi 5 NCNN FPS/latency measurement.
- Pending: hardware safety procedure and validated command-sending launch profile.
- Pending: heartbeat-loss field procedure and hardware validation of the ACK
  retry policy.
- Required invariant: preserve the same high-level autonomy API across SITL and hardware.

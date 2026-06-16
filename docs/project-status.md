# Project Status and Agent Ground Truth

Last audited status: 2026-06-16, after adding the Webots TCP camera plus YOLO
perception path.

This document is the short source of truth for AI agents and maintainers. If a
claim conflicts with this file, verify the code and update this file before
continuing.

## Implemented

- Vendored ArduPilot `Webots_Python` baseline under `webots/`.
- Baseline Webots file audit: 36 official upstream tracked files, 0 missing, 0
  extra local tracked files.
- Mission state machine for the two-gate task:
  `takeoff -> seek -> center -> pass -> adaptive acquire -> brake -> center ->
  pass -> forward exit 2 m -> land`.
- Hybrid low-altitude takeoff profile: short navigation-takeoff bootstrap to
  get airborne, then bounded body-frame vertical velocity, settle tolerance, and
  required stable ticks.
- Body-frame `VehicleCommand` contract:
  forward/right/down velocity plus yaw-rate.
- Altitude-hold velocity bias from fused `LOCAL_POSITION_NED` altitude.
- Adaptive next-gate acquire instead of a blind sprint after gate 1.
- Brake-before-center after gate 2 is detected during adaptive acquire.
- MAVLink telemetry adapter for heartbeat, mode/armed state, landed state,
  altitude, and local forward position.
- MAVLink command adapter for guided mode, arm, takeoff, land, and body-frame
  velocity.
- Runtime modes:
  `heartbeat`, `listen`, and `autonomy`.
- Detector modes:
  `none`, `synthetic`, and `webots-yolo`.
- Synthetic detector for mission/MAVLink wiring only.
- Webots TCP camera frame source for upstream `iris_camera.wbt` port `5599`.
- YOLO-to-`GateDetection` adapter with confidence and class filtering.
- `webots-yolo` runtime glue that keeps camera/model details outside the
  mission state machine.
- Experimental `RobotstadiumGoal` Webots PROTO and two goal instances in
  `webots/worlds/iris_camera.wbt`.

## Not Implemented Yet

- A trained/provided gate YOLO model file.
- A validated competition-grade two-gate Webots course. The current goal objects
  are experimental geometry for perception/world iteration.
- True RGB Webots stream. Upstream `iris_camera.wbt` currently streams
  grayscale; the adapter expands `gray8` to three channels for YOLO.
- Real-hardware C920/OpenCV camera source.
- `COMMAND_ACK` parsing and retry policy.
- Lost-heartbeat failsafe in the process runtime.
- User-editable mission tuning file.
- Automatic course-frame calibration.
- Hardware launch/deployment profile.

## Safety Defaults

- `SEND_COMMANDS="0"` is the default. This runs dry-run decisions only.
- Use `SEND_COMMANDS="1"` only in SITL after heartbeat, local-position telemetry,
  detector behavior, and body-frame signs are verified.
- `webots-yolo` must use class filtering during motion tests. Do not accept every
  class unless the model detects only gates.
- Local `configs/*.env` files are ignored and can become stale. When behavior
  conflicts with docs, compare them against the matching `.env.example`.
- Real-hardware behavior is future work unless a dedicated hardware adapter and
  safety procedure are added.

## Architecture Invariants

- `GateAutonomyMission` must remain I/O-free and non-blocking.
- Mission code must not import Webots, OpenCV, Ultralytics, NumPy, or MAVLink.
- Perception returns `GateDetection | None`; it does not mutate mission phase.
- MAVLink command sending happens only in the runtime/adapter layer.
- Sensor fusion belongs to ArduPilot EKF or an explicit adapter. Mission code
  consumes fused telemetry, not raw GPS/rangefinder/optical-flow samples.
- RGB-only gate detection is image alignment, not reliable metric distance,
  unless camera intrinsics and gate dimensions are explicitly calibrated.

## Primary Docs

- `docs/run-simulation.md`: exact Ubuntu/Webots/SITL runbook.
- `docs/webots-yolo-pipeline.md`: Webots camera stream to YOLO to
  `GateDetection`.
- `docs/main-logic-map.md`: code path and ownership map.
- `docs/perception-contract.md`: detector input/output contract.
- `docs/simulation-audit.md`: readiness and risk checklist.
- `docs/troubleshooting.md`: failure modes and fixes.
- `docs/webots-source-sync.md`: how the vendored Webots tree was sourced and how
  to re-audit it.

## Working Tree Discipline

- Keep documentation/status updates separate from Webots world design commits
  when possible.
- If a Webots GUI save updates version headers, camera positions, hidden dynamic
  fields, or object translations, review the diff before staging.
- Do not commit generated Webots project files, caches, or accidental camera
  pose churn as part of autonomy logic changes.

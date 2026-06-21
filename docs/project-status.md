# Project Status and Agent Ground Truth

Last audited status: 2026-06-21, after adding Raspberry Pi deployment
scaffold, serial MAVLink baud configuration, and paper-oriented mathematical
documentation without changing the simulation mission logic.

This document is the short source of truth for AI agents and maintainers. If a
claim conflicts with this file, verify the code and update this file before
continuing.

## Implemented

- Vendored ArduPilot `Webots_Python` baseline under `webots/`.
- Baseline Webots file audit: 36 official upstream tracked files, 0 missing, 0
  extra local tracked files.
- Mission state machine for the two-gate task:
  `takeoff -> seek -> center dwell -> clearance check -> pass -> clear forward
  -> acquire -> brake -> center dwell -> clearance check -> pass -> forward
  exit 2 m -> brake -> land`.
- ArduPilot-managed takeoff profile: mission sends `MAV_CMD_NAV_TAKEOFF` to
  `1.0 m` and waits for `+/-0.06 m` telemetry settle over `8` non-landed ticks
  before seeking gates. No companion-side body-z takeoff controller is active.
- Body-frame `VehicleCommand` contract:
  forward/right/down velocity plus yaw-rate.
- Altitude-hold velocity bias from fused `LOCAL_POSITION_NED` altitude.
- Configurable center dwell before pass commit. Default is `5.0 s`.
- Configurable image-space gate clearance margins through
  `VISUAL_PASS_TARGET_OFFSET_*` and `VISUAL_PASS_CLEARANCE_*`.
- Low-pass filtering on both image error and centering velocity commands to
  reduce rigid camera/body shaking in Webots.
- Visual servoing is currently filtered proportional control, not PID or
  feed-forward. Do not describe it as PID/FF; those are future upgrades only
  after logs prove filtered setpoints are insufficient.
- Stable forward-only committed gate pass after dwell/clearance passes.
- Post-gate clear distance before gate 2 acquisition instead of a blind timed
  sprint after gate 1.
- Configurable bbox-area readiness guard through `MISSION_NEXT_GATE_MIN_AREA`
  and `MISSION_GATE_READY_AREA` so a far centered gate cannot immediately start
  the pass/landing sequence.
- Brake-before-center after gate 2 is detected during adaptive acquire.
- Brake-before-land after final forward exit, so `LAND` is not issued while the
  mission is still commanding forward velocity.
- `BRAKE` ramps forward speed down through `MISSION_BRAKE_RAMP` instead of
  stepping immediately to zero. Companion altitude correction is disabled during
  brake by default through `MISSION_BRAKE_ALTITUDE_HOLD=0` to avoid vertical
  bounce while decelerating.
- MAVLink telemetry adapter for heartbeat, mode/armed state, landed state,
  altitude, and local forward position.
- MAVLink command adapter for guided mode, arm, takeoff, land, and body-frame
  velocity.
- MAVLink runtime supports serial baud configuration through `MAVLINK_BAUD` /
  `--baud`. UDP SITL behavior remains on `udp:127.0.0.1:14551` by default.
- SITL launcher can expose an additional MAVLink UDP output through
  `MAVLINK_OUT_EXTRA`, so Mission Planner and the autonomy runtime can use
  separate local ports.
- Runtime modes:
  `heartbeat`, `listen`, and `autonomy`.
- Detector modes:
  `none`, `synthetic`, and `webots-yolo`.
- Synthetic detector for mission/MAVLink wiring only.
- Synthetic detector keeps fake detections continuous across
  `SEEK_GATE -> CENTER_GATE` to avoid phase oscillation during wiring tests.
- Webots TCP camera frame source for upstream `iris_camera.wbt` port `5599`.
- `iris_camera.wbt` camera device is explicitly named `camera` to match the
  ArduPilot Webots controller argument `--camera camera`.
- Webots TCP camera client preserves partial frames across normal socket
  timeouts, so polling faster than the camera FPS does not reconnect on every
  camera tick.
- Webots camera diagnostics report whether the client is failing to connect,
  waiting for header bytes, waiting for payload bytes, or receiving valid
  frames. `scripts/probe_webots_camera.py` checks the stream without YOLO.
- Webots camera TCP has a configurable idle reconnect watchdog through
  `WEBOTS_CAMERA_IDLE_RECONNECT`.
- YOLO raw candidate extraction with explicit class filtering.
- `GateTargetSelector` validates candidate geometry, applies stable-window
  target validation, applies an optional hollow-gate appearance sanity score,
  prefers nearer/larger gates, tracks target continuity, and smooths the
  selected bbox before publishing `GateDetection`.
- `webots-yolo` runtime glue that keeps camera/model details outside the
  mission state machine.
- `webots-yolo` runs camera ingestion and YOLO inference in background workers.
  The mission loop only reads the latest fresh detection snapshot; it does not
  block on TCP frame reads or model inference.
- YOLO target selection is no longer embedded in `YoloGateDetector`; selection
  belongs to `perception/target_selector.py`.
- Optional OpenCV diagnostics window can show accepted YOLO candidates, selected
  target, validator ROI, rejection reasons, score, area ratio, aspect ratio,
  hollow-gate appearance score, image-center error, and pass-clearance
  target/margins.
- Trained YOLOv8n gate model at `models/gate_yolov8n_best.pt`.
- Iris camera YOLO launcher profile at `scripts/run_iris_camera_yolo.sh`.
- Raspberry Pi dry-run deployment scaffold at `scripts/run_raspi_hardware.sh`
  and `configs/raspi_runtime.env.example`.
- Paper-oriented mathematical behavior documentation in
  `docs/mathematical-foundations.md`.
- Local `iris_camera.wbt` requests true RGB camera streaming with
  `--camera-format rgb24`; `gray8` remains supported for upstream compatibility.
- Experimental `RobotstadiumGoal` Webots PROTO and two goal instances in
  `webots/worlds/iris_camera.wbt`.

## Not Implemented Yet

- A validated competition-grade two-gate Webots course. The current goal objects
  are experimental geometry for perception/world iteration.
- Real-hardware C920/OpenCV camera source.
- `COMMAND_ACK` parsing and retry policy.
- Lost-heartbeat failsafe in the process runtime.
- Dedicated YAML/TOML mission tuning file. Current tuning is exposed through
  CLI flags and `configs/autonomy_runtime.env`.
- Automatic course-frame calibration.
- Validated hardware flight launch profile. The current Raspberry Pi path is
  only a dry-run deployment scaffold until the camera adapter and safety
  procedure are implemented.

## Safety Defaults

- `SEND_COMMANDS="0"` is the default. This runs dry-run decisions only.
- Use `SEND_COMMANDS="1"` only in SITL after heartbeat, local-position telemetry,
  detector behavior, and body-frame signs are verified.
- `scripts/run_raspi_hardware.sh` also defaults to dry-run and loads
  `configs/raspi_runtime.env` when present. The tracked Raspberry Pi template
  uses `DETECTOR="none"` because real C920/OpenCV perception is not implemented
  yet.
- `webots-yolo` must use fail-closed class filtering during motion tests. Do
  not accept every class unless the model detects only gates.
- The current gate model should be filtered by class name
  `YOLO_GATE_CLASS_NAMES="Goals-Detection"` and class id
  `YOLO_GATE_CLASS_IDS="3"`. Dog/Forklift/Table detections must never be
  converted into `GateDetection`.
- If a non-gate object is labeled by YOLO as `cls=3:Goals-Detection`, class
  filtering alone cannot reject it. First confirm the realtime frame is `rgb8`
  and inspect the diagnostics `raw=...` line. The optional
  `GATE_SELECTOR_MIN_APPEARANCE_SCORE` guard is disabled by default and should
  be used only after RGB input and raw YOLO class output are understood.
- If retraining changes the class order, recheck model metadata before
  changing `YOLO_GATE_CLASS_IDS`. Name-only filtering is acceptable only after
  confirming the gate label spelling.
- Local `configs/*.env` files are ignored and can become stale. When behavior
  conflicts with docs, compare them against the matching `.env.example`.
- For the current `iris_camera.wbt` profile, `VISUAL_FRAME_WIDTH="640"` and
  `VISUAL_FRAME_HEIGHT="480"` must match the streamed frame geometry.
- Gate pass tuning lives in `MISSION_CENTER_DWELL`,
  `MISSION_CENTER_CLEARANCE_REQUIRED`, `MISSION_REQUIRED_DETECTION_TICKS`,
  `MISSION_CENTER_LOST_GRACE_TICKS`, `MISSION_GATE_PASS_DISTANCE`,
  `MISSION_GATE_PASS_SPEED`, `MISSION_NEXT_GATE_CLEAR_DISTANCE`,
  `MISSION_NEXT_GATE_MIN_AREA`, `MISSION_GATE_READY_AREA`, and
  `MISSION_BRAKE_SETTLE` / `MISSION_BRAKE_RAMP`.
- Visual smoothness tuning lives in `VISUAL_FILTER_ALPHA`,
  `VISUAL_COMMAND_FILTER_ALPHA`, `VISUAL_MAX_FORWARD_SPEED`, and the
  `VISUAL_*_KP` / `VISUAL_MAX_*` fields. `VISUAL_MAX_ERROR_FOR_FORWARD`
  controls when CENTER_GATE is allowed to approach while still off-center.
- For jerky CENTER_GATE motion, tune speed/rate limits and command filtering
  before increasing gains or adding PID/FF terms.
- `scripts/run_autonomy_sitl.sh` launches this repo's Python module from `src/`
  before falling back to any `drone-autonomy` executable in `PATH`. It reads
  local `configs/autonomy_runtime.env` and forwards only explicitly set values
  into the CLI; mission/control defaults stay in Python runtime/domain config.
- `scripts/run_iris_camera_yolo.sh` selects
  `AUTONOMY_PROFILE="iris-camera-yolo"`. The profile enforces `webots-yolo`
  detector mode and diagnostics by default, while model path, class filters,
  thresholds, speeds, and gains remain tunable from env or inline overrides.
- Real-hardware behavior is future work unless a dedicated hardware adapter and
  safety procedure are added.
- When Mission Planner is open, keep it on `udp:127.0.0.1:14550` and run the
  autonomy runtime on a separate SITL output such as `udp:127.0.0.1:14551`.
  Do not make both processes consume the same UDP endpoint.

## Architecture Invariants

- `GateAutonomyMission` must remain I/O-free and non-blocking.
- Mission code must not import Webots, OpenCV, Ultralytics, NumPy, or MAVLink.
- Perception returns `GateDetection | None`; it does not mutate mission phase.
- YOLO model adapters should return raw candidates. `GateTargetSelector` owns
  validation/tracking/smoothing and is the only layer that reduces multiple
  candidate gates to one mission target.
- Background perception workers must use bounded-latest state, not unbounded
  frame queues. If perception is slower than camera FPS, old frames are dropped.
- Mission centering should tolerate brief detection loss before re-entering
  scan; do not restore immediate `CENTER_GATE -> SEEK_GATE` on a single missed
  detection.
- Mission must not commit to `PASS_GATE` until center dwell has elapsed and the
  visual-servo clearance validator is currently true for the required clearance
  window.
- Do not restore the old pass rule that depended on bounding-box area alone or
  alignment tick count. The current area guard is only an image-space readiness
  check; RGB bbox area is not reliable metric distance without calibration.
- MAVLink command sending happens only in the runtime/adapter layer.
- Sensor fusion belongs to ArduPilot EKF or an explicit adapter. Mission code
  consumes fused telemetry, not raw GPS/rangefinder/optical-flow samples.
- Do not reintroduce split takeoff bootstrap plus companion body-z velocity
  without a new SITL log-based reason; that approach overshot in testing.
- RGB-only gate detection is image alignment, not reliable metric distance,
  unless camera intrinsics and gate dimensions are explicitly calibrated.
- Do not use confidence-only gate selection in two-gate scenes. Larger/nearer
  gate priority, ROI validation, and target continuity are required.

## Primary Docs

- `docs/run-simulation.md`: exact Ubuntu/Webots/SITL runbook.
- `docs/tuning-guide.md`: technical tuning reference for YOLO, OpenCV
  overlays, gate acquire, pass commit, and visual servo parameters.
- `docs/mathematical-foundations.md`: equations and state-machine model that
  match the implemented code behavior.
- `docs/deployment-raspi.md`: staged Raspberry Pi deployment scaffold and
  hardware safety boundary.
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

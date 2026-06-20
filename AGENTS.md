# Agent Notes

These notes are for AI agents and future maintainers working on this repository.

Before making architecture claims, read `docs/project-status.md`. It records
what is implemented, what is still missing, and which boundaries must not be
blurred.

Before changing tuning defaults or explaining tuning, read
`docs/tuning-guide.md`. It defines the difference between camera geometry,
YOLO inference size, OpenCV overlay boxes, gate acquire thresholds, and visual
servo gains.

## Project Intent

Build a simple but robust autonomous drone pipeline for a two-gate task:

1. Take off to about 1 meter.
2. Search for the first hollow gate using a normal RGB camera.
3. Center on the gate while approaching.
4. Pass through the first gate.
5. Move forward while actively acquiring the second gate.
6. Center and pass through the second gate.
7. Fly forward about 2 meters after the last gate.
8. Land.

The same high-level code should work in Webots/ArduPilot SITL and later on a real Pixhawk 6C Mini, Raspberry Pi 5, Logitech C920 Pro, and hexacopter frame.

## Non-Negotiable Boundaries

- Keep `webots/` as a full vendored copy of ArduPilot's `Webots_Python`
  example. Do not replace it with partial copies.
- Do not modify ArduPilot source by default.
- Keep hardware, simulator, perception, control, and mission logic separated.
- Do not hard-wire YOLO, OpenCV, Webots, or MAVLink details into the mission state machine.
- Use body-frame velocity commands as the mission output contract.
- Prefer deterministic state machines over hidden loops with global state.
- Every control sign convention must be documented before it is used.

## Control Frame

Internal velocity commands use ArduPilot-friendly body axes:

- `body_vx_m_s`: positive forward.
- `body_vy_m_s`: positive right.
- `body_vz_m_s`: positive down.
- `yaw_rate_rad_s`: positive right/clockwise from the vehicle perspective.

Adapters must translate this contract correctly when sending MAVLink commands.

## Perception Contract

The gate detector should return one `GateDetection` per frame:

- Bounding box around the hollow gate structure.
- Confidence score.
- Observation timestamp.
- Optional track id.

The detector should not decide flight phases. It only reports perception.

Current detector modes:

- `none`: no detections; mission should keep seeking.
- `synthetic`: fake centered boxes for wiring tests only.
- `webots-yolo`: Webots TCP camera stream plus YOLO wrapper. This repo's
  `iris_camera.wbt` requests `rgb24` from the vendored Webots controller via
  `--camera-format rgb24`.

Current `webots-yolo` perception pipeline:

1. Webots TCP camera frame.
2. YOLO class-filtered raw `GateCandidate` list.
3. `GateTargetSelector` geometry validation, stability window, target tracking,
   bbox smoothing, and selected `GateDetection`.
4. Mission consumes only `GateDetection | None`.

Current trained simulation model:

- Path: `models/gate_yolov8n_best.pt`.
- YOLO class filter accepted by the repo profile: class name
  `Goals-Detection` plus class id `3`.
- Current observed class id after the multi-class retrain: `3`.
- Preferred world for current simulation work: `webots/worlds/iris_camera.wbt`.

## Current Strategy

The first implementation uses image-based visual servoing:

- Takeoff is ArduPilot-managed: send `MAV_CMD_NAV_TAKEOFF` to `1.0 m`, then
  wait for fused telemetry to settle. Do not reintroduce the removed `0.35 m`
  bootstrap plus companion body-z velocity loop; it overshot in SITL.
- Horizontal image error drives right/left body velocity and yaw rate.
- Vertical image error drives climb/descent through body z velocity.
- Centering velocity output is low-pass filtered. Keep this smoothing unless
  SITL logs prove it is too sluggish; direct P-to-velocity made the camera view
  shake and prevented stable clearance.
- The visual servo controller is filtered proportional control, not PID or
  feed-forward. Do not claim PID/FF is implemented. If motion is jerky, tune
  speed/rate limits and filter alpha values before proposing new control terms.
- Forward speed during `CENTER_GATE` is intentionally slow. The committed
  crossing happens in `PASS_GATE`, not during visual centering.
- Centering must dwell for `MISSION_CENTER_DWELL` seconds before a gate pass
  can commit. The default `5.0 s` is intentional to let ArduPilot and the
  vehicle settle naturally while visual servoing continues.
- Gate pass commit requires the visual-servo clearance validator to remain true
  for `MISSION_CENTER_CLEARANCE_REQUIRED` seconds. Clearance is controlled by
  `VISUAL_PASS_TARGET_OFFSET_*` and `VISUAL_PASS_CLEARANCE_*`; tune those for
  camera/body/GPS protrusion margins instead of editing mission code.
- Gate pass commit also requires the selected bbox area to reach
  `MISSION_GATE_READY_AREA`. This is an image-space readiness guard, not metric
  distance.
- During `PASS_GATE`, command stable forward-only body velocity plus altitude
  hold. Do not keep lateral/yaw visual corrections active while committed to
  crossing the gate.
- After gate 1, move forward for `MISSION_NEXT_GATE_CLEAR_DISTANCE` before gate
  2 detections are allowed to trigger centering. This is a configurable
  post-obstacle clear distance, not a hardcoded sprint timer.
- During `NEXT_GATE_ACQUIRE`, detections below `MISSION_NEXT_GATE_MIN_AREA` are
  ignored as too small/noisy; detections between that value and
  `MISSION_GATE_READY_AREA` keep the drone moving forward instead of braking.
- `WEBOTS_DETECTION_STALE` and `MISSION_MAX_DETECTION_AGE` both affect whether
  a background YOLO result can reach mission logic. If slow inference is tuned,
  keep these values consistent instead of only changing one layer.
- Distance after gates uses local forward position from telemetry, not frame count.
- Final exit distance is forward travel after the last gate, not altitude.
- The mission brakes before issuing `LAND`; do not describe landing as
  immediate while forward velocity is still commanded.
- `BRAKE` ramps forward body velocity down over `MISSION_BRAKE_RAMP` and does
  not run companion altitude correction by default. Do not reintroduce abrupt
  zero-forward brake commands or brake-time altitude P correction unless logs
  justify it.
- Altitude correction uses fused telemetry from ArduPilot/simulator adapters; do not raw-blend GPS, rangefinder, and optical-flow samples in mission code.

## Anti-Hallucination Rules

- Treat `docs/project-status.md` as the first status checkpoint.
- If a module is a placeholder, say it is a placeholder in code or docs.
- If telemetry is required but not wired yet, model it explicitly as input.
- If camera calibration is unknown, avoid pretending to know metric distance from RGB alone.
- If sensor fusion is required, document whether it is ArduPilot EKF fusion or a local estimator before adding code.
- If TAKEOFF behavior is discussed, state that current code delegates takeoff
  altitude control to ArduPilot and does not run a companion-side vertical
  velocity takeoff controller.
- If gate-pass behavior is discussed, state that current code uses
  center-dwell plus configurable image-space clearance and area readiness, then
  a forward-distance committed pass. Do not describe it as an area-only or
  alignment-tick pass rule.
- If synthetic detector oscillates between `seek_gate` and `center_gate`, check
  detector state continuity across phase changes before changing mission logic.
- `scripts/run_autonomy_sitl.sh` should prefer this repo's `src/` module launch
  over a `drone-autonomy` executable from `PATH`; stale console scripts can hide
  current code. Inline env values, for example `SEND_COMMANDS=1`, should
  override local config files.
- `scripts/run_iris_camera_yolo.sh` should select
  `AUTONOMY_PROFILE="iris-camera-yolo"` instead of hardcoding every tuning
  value. The generic runner reads `configs/autonomy_runtime.env` and forwards
  only explicitly set values; do not reintroduce duplicated shell fallback
  defaults for mission/control tuning.
- Do not tell operators to tune `configs/autonomy_runtime.env.example` directly.
  That file is the tracked template. Real experiment tuning belongs in
  `configs/autonomy_runtime.env` or inline env overrides.
- If a custom two-gate Webots world is absent, do not invent world file names.
- If Webots assets look incomplete, re-sync from ArduPilot
  `libraries/SITL/examples/Webots_Python` instead of patching paths ad hoc.
- For `iris_camera.wbt`, keep `Camera { name "camera" ... }` aligned with the
  Iris `controllerArgs` value `--camera camera`; otherwise the Webots controller
  cannot start the TCP camera stream on port `5599`.
- Repeated Webots `Connected to camera client` / `Camera client disconnected`
  messages point at TCP camera client buffering/timeout behavior before YOLO
  model quality. The client should preserve partial frames across normal
  timeouts.
- For `webots-yolo waiting for camera frame`, inspect the printed
  `status=... detail=...` suffix or run `scripts/probe_webots_camera.py` before
  changing detection thresholds.
- `WEBOTS_CAMERA_IDLE_RECONNECT` controls the TCP stream watchdog. Prefer this
  config over hidden timing constants when Webots connects but sends no bytes.
- `webots-yolo` uses background camera and detector workers with bounded-latest
  snapshots. Do not replace this with an unbounded frame queue or mission-loop
  blocking reads.
- `WEBOTS_DETECTION_STALE` limits how long a background YOLO result can be
  reused by the mission loop.
- YOLO target selection intentionally prefers larger/nearer gate boxes over
  confidence-only selection, then uses center proximity and target-lock overlap.
- If a non-gate object is shown as `cls=3:Goals-Detection`, treat it as a YOLO
  model false positive, not a class-filter failure. Class filtering cannot
  recover after the model emits the wrong class. First confirm diagnostics show
  `frame ... rgb8` and inspect the `raw=...` class summary. The target selector's
  `GATE_SELECTOR_MIN_APPEARANCE_SCORE` guard is disabled by default; use it only
  after RGB input and raw YOLO class output are understood.
- Do not bypass `GateTargetSelector` by feeding raw YOLO boxes directly to
  mission/control.
- When changing OpenCV diagnostics, keep validator boundaries visible:
  validation ROI, crosshair, rejected candidates with reasons, accepted
  candidates, selected target, hollow-gate appearance score, and area reference
  boxes for far/ready gates.
- `WEBOTS_DIAGNOSTICS_WINDOW=1` enables the optional OpenCV diagnostics view.
- `VISUAL_FRAME_WIDTH=640` and `VISUAL_FRAME_HEIGHT=480` match the current
  `iris_camera.wbt` stream. If the camera resolution changes, update these
  runtime values before tuning gains.
- `YOLO_IMGSZ=640` is Ultralytics inference/letterbox size, not camera
  resolution. Do not describe it as a 640x640 camera frame.
- If `webots-yolo` is discussed, state that it requires an external model file
  unless `models/gate_yolov8n_best.pt` is present, and that this repo's
  `iris_camera.wbt` should stream `rgb24`. If diagnostics show
  `rgb8_from_gray8`, the run is still using the old grayscale path.
- For the bundled gate model, do not assume the class name is `gate`; use
  `YOLO_GATE_CLASS_NAMES="Goals-Detection"` and `YOLO_GATE_CLASS_IDS="3"`
  unless the model metadata is rechecked and a different gate label/id is
  confirmed.
- If the model is replaced with a multi-class dataset, recheck the class map.
  `Goals-Detection` may no longer be id `0`; for the current known map
  `AdvertisementBox, Dog, Forklift, Goals-Detection, Table`, the gate id is
  `3`. Do not clear both filters; an empty filter is unsafe and should fail.
- If Dog, Forklift, AdvertisementBox, or Table appears as the selected target,
  audit runtime `webots-yolo class_filter ...` output before changing mission
  logic. Non-gate classes reaching `GateDetection` means the class filter is
  wrong or disabled.
- If Webots world/proto files are modified in the working tree, verify whether
  they are intentional user design changes before staging. Do not mix accidental
  Webots GUI churn with autonomy/code documentation commits.
- If real-hardware behavior is discussed, mark it as a future adapter unless implemented.

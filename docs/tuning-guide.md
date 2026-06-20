# Autonomy Tuning Guide

This guide is the technical reference for tuning the current
`iris_camera.wbt + webots-yolo` simulation profile.

Use this order:

1. Keep `SEND_COMMANDS=0`.
2. Tune perception and diagnostics until the selected target is correct.
3. Tune mission thresholds and pass clearance.
4. Tune visual-servo smoothness.
5. Enable `SEND_COMMANDS=1` only in SITL.

Tune persistent values in:

```text
configs/autonomy_runtime.env
```

Runtime precedence:

```text
inline env before command
-> iris-camera-yolo profile-owned defaults
-> configs/autonomy_runtime.env
-> Python runtime defaults
```

`scripts/run_iris_camera_yolo.sh` selects the iris-camera YOLO profile. That
profile owns detector mode and diagnostics defaults. It does not own mission
thresholds, clearance margins, gains, or custom model settings. Keep those in
`configs/autonomy_runtime.env` or pass them inline for a single run.

Use inline environment variables for one-shot experiments:

```bash
MISSION_GATE_READY_AREA=0.050 \
WEBOTS_DIAGNOSTICS_WINDOW=1 \
SEND_COMMANDS=0 \
bash scripts/run_iris_camera_yolo.sh
```

Do not tune `configs/autonomy_runtime.env.example` directly. It is the tracked
template used to recreate the local file.

## Camera Size vs YOLO Image Size

The current Webots camera frame is:

```text
VISUAL_FRAME_WIDTH="640"
VISUAL_FRAME_HEIGHT="480"
```

Those values are the real frame geometry used by visual servoing, normalized
image error, OpenCV overlays, and area ratios.

`YOLO_IMGSZ="640"` is different. It is the Ultralytics inference size. YOLO
letterboxes/resizes the incoming 640x480 frame internally to an inference size
of 640. It is not a camera width/height pair and it should not be written as
`640x480`.

Practical rule:

- Change `VISUAL_FRAME_WIDTH/HEIGHT` only if the camera stream resolution
  changes.
- Change `YOLO_IMGSZ` only for detector speed/accuracy tradeoff.
- Keep `YOLO_IMGSZ=640` for the current YOLOv8n gate model unless profiling
  shows the CPU/GPU cannot keep up.

## Class Filter

Current model metadata:

```text
0=AdvertisementBox
1=Dog
2=Forklift
3=Goals-Detection
4=Table
```

Current safe filter:

```text
YOLO_GATE_CLASS_NAMES="Goals-Detection"
YOLO_GATE_CLASS_IDS="3"
```

Do not clear both values. If Dog, Forklift, AdvertisementBox, or Table appears
as the selected target, stop motion tests and fix the filter before tuning
mission logic.

If a Dog/Forklift/Table visually appears in the frame but the overlay label is
still `cls=3:Goals-Detection`, the model itself is misclassifying that object as
the gate class. Class filtering cannot recover from a wrong class emitted by the
model. Use the appearance sanity filter below as a guard, and plan to improve
the dataset/model if the false positives persist.

Expected startup line:

```text
webots-yolo class_filter names=Goals-Detection ids=3
```

## OpenCV Diagnostics Overlay

Enable diagnostics:

```bash
WEBOTS_DIAGNOSTICS_WINDOW=1 SEND_COMMANDS=0 bash scripts/run_iris_camera_yolo.sh
```

Overlay meanings:

```text
cyan crosshair        image center used by visual servoing
magenta rectangle    pass-clearance target window
blue box             far-gate area threshold
orange box           ready/pass area threshold
red candidate        rejected candidate with reason
yellow candidate     accepted but not selected candidate
green candidate      selected stable target
cls=id:name          YOLO class id and label for that candidate
raw=...              YOLO raw class counts before this pipeline class-filters
g=...                optional hollow-gate appearance score from the crop
a=...                bbox area ratio relative to frame area
e=(x,y)              normalized center error from target
```

The magenta rectangle is controlled by:

```text
VISUAL_PASS_TARGET_OFFSET_X
VISUAL_PASS_TARGET_OFFSET_Y
VISUAL_PASS_CLEARANCE_LEFT
VISUAL_PASS_CLEARANCE_RIGHT
VISUAL_PASS_CLEARANCE_UP
VISUAL_PASS_CLEARANCE_DOWN
```

The blue/orange area boxes are controlled by:

```text
MISSION_NEXT_GATE_MIN_AREA
MISSION_GATE_READY_AREA
```

## Gate 2 Acquire

Gate 2 acquire is intentionally not a blind sprint. After gate 1 pass, the
mission moves forward and waits until the next gate is plausible.

Main variables:

```text
MISSION_NEXT_GATE_CLEAR_DISTANCE
MISSION_NEXT_GATE_ACQUIRE_SPEED
MISSION_NEXT_GATE_MIN_AREA
MISSION_GATE_READY_AREA
MISSION_NEXT_GATE_MAX_DISTANCE
MISSION_NEXT_GATE_TIMEOUT
WEBOTS_DETECTION_STALE
MISSION_MAX_DETECTION_AGE
GATE_SELECTOR_REQUIRED_STABLE
MISSION_REQUIRED_DETECTION_TICKS
```

Behavior:

- Before `MISSION_NEXT_GATE_CLEAR_DISTANCE`, gate detections do not trigger
  centering.
- Below `MISSION_NEXT_GATE_MIN_AREA`, detections are treated as too small/noisy.
- Between `MISSION_NEXT_GATE_MIN_AREA` and `MISSION_GATE_READY_AREA`, the drone
  keeps moving forward while tracking the gate.
- At or above `MISSION_GATE_READY_AREA`, stable detections can trigger
  brake-before-center.
- If `MISSION_NEXT_GATE_MAX_DISTANCE` or `MISSION_NEXT_GATE_TIMEOUT` is reached,
  the mission falls back to seek behavior.

If gate 2 is lost too easily:

1. Confirm the selected class is `cls=3:Goals-Detection`.
2. Increase `WEBOTS_DETECTION_STALE` slightly, for example `0.75 -> 1.0`.
3. Reduce `MISSION_NEXT_GATE_MIN_AREA` slightly if the gate is visible but too
   small.
4. Reduce `MISSION_GATE_READY_AREA` if the gate is centered but never reaches
   ready.
5. Reduce `MISSION_NEXT_GATE_ACQUIRE_SPEED` if the drone outruns perception.

Do not solve gate-2 loss by increasing control gains first.

## Variable Reference

These are the variables that are intended to be tuned by an operator. If a value
is not listed here, treat it as code-level behavior and audit the code before
changing it.

### Runtime And Safety

| Variable | Used by | Purpose and rationale | Tune when |
| --- | --- | --- | --- |
| `MAVLINK_CONNECTION` | runtime | MAVLink endpoint consumed by autonomy. Use `14551` so Mission Planner can stay on `14550`. | Change only if SITL exposes a different extra output. |
| `SEND_COMMANDS` | runtime | Safety switch. `0` runs perception and mission decisions without moving the vehicle. | Set `1` only after diagnostics show the correct target in SITL. |
| `LOOP_HZ` | runtime | Mission/control tick rate. Higher rates reduce command latency but cannot make YOLO faster. | Keep `20` unless CPU load or telemetry rate proves otherwise. |
| `MAX_RUNTIME` | runtime | Process-level timeout. Prevents a runaway unattended test. | Increase only for longer courses after failsafe behavior is verified. |
| `COURSE_FORWARD_X/Y` | telemetry adapter | Projects `LOCAL_POSITION_NED` into course-forward distance. Gate pass, gate-2 clear, and final exit depend on this. | Change if the Webots course is not aligned with local +X/North. |

### Camera And YOLO

| Variable | Used by | Purpose and rationale | Tune when |
| --- | --- | --- | --- |
| `YOLO_MODEL_PATH` | YOLO adapter | Model file used for gate candidates. The default is the bundled `models/gate_yolov8n_best.pt`. | Point to a new `best.pt` after retraining. |
| `YOLO_CONFIDENCE` | YOLO adapter | First confidence cutoff before candidate geometry checks. Keeps low-confidence boxes out of the selector. | Lower slightly if real gates never appear; raise if many false candidates pass into diagnostics. |
| `YOLO_IMGSZ` | YOLO adapter | Ultralytics inference/letterbox size, not camera resolution. | Lower for speed, raise for accuracy only after measuring FPS. |
| `YOLO_GATE_CLASS_NAMES` | YOLO adapter | Accepted class labels. For the current model this must include `Goals-Detection`. | Update after checking model metadata or `data.yaml`. |
| `YOLO_GATE_CLASS_IDS` | YOLO adapter | Accepted numeric class ids. Current multi-class model uses id `3` for `Goals-Detection`. | Set to the confirmed gate id, or `""` only for audited name-only filtering. |
| `YOLO_DEVICE` | YOLO adapter | Ultralytics device selection such as `cpu` or `cuda:0`. | Change when Ubuntu has a working GPU stack. |
| `WEBOTS_CAMERA_HOST/PORT` | Webots camera client | TCP endpoint for ArduPilot Webots camera stream. Current `iris_camera.wbt` uses `127.0.0.1:5599`. | Change only if the world/controller port changes. |
| `WEBOTS_CAMERA_ENCODING` | Webots camera client | Stream payload format. This repo's `iris_camera.wbt` requests `rgb24`; `gray8` is only for upstream-compatible fallback worlds. | Keep `rgb24` unless diagnostics prove the world/controller is sending grayscale. |
| `WEBOTS_CAMERA_IDLE_RECONNECT` | Webots camera client | Watchdog for a connected socket that stops sending bytes. Avoids reconnect loops on normal short timeouts. | Increase if a slow Webots run reports idle reconnects despite a valid stream. |
| `WEBOTS_DETECTION_STALE` | Webots YOLO provider | Maximum age for reusing the latest background YOLO result. Bridges camera/model FPS to the mission loop before mission validation. | Increase slightly for slow inference; keep `MISSION_MAX_DETECTION_AGE` aligned or higher enough for the mission to accept it. |
| `WEBOTS_DIAGNOSTICS_WINDOW` | Webots YOLO provider | Enables OpenCV overlay for class labels, candidate reasons, ROI, area guides, and clearance box. | Keep `1` during simulation tuning; disable for headless runs. |

### Gate Target Selector

| Variable | Used by | Purpose and rationale | Tune when |
| --- | --- | --- | --- |
| `GATE_SELECTOR_MIN_SEEK_CONFIDENCE` | target selector | Confidence required to lock a new target while seeking/acquiring. Higher than track confidence to avoid new false locks. | Lower if true gates are red with `confidence`; raise if non-gate boxes survive class filtering. |
| `GATE_SELECTOR_MIN_TRACK_CONFIDENCE` | target selector | Confidence allowed while tracking an already locked target. Lower value prevents brief YOLO dips from dropping the gate. | Lower if a stable gate flickers during centering; do not use it to accept wrong classes. |
| `GATE_SELECTOR_MIN_AREA_RATIO` | target selector | Rejects tiny boxes before scoring. This is generic geometry sanity, separate from gate-2 acquire area. | Lower only if far true gates are rejected with `area_small`. |
| `GATE_SELECTOR_MIN_ASPECT_RATIO` / `MAX_ASPECT_RATIO` | target selector | Rejects boxes whose width/height cannot plausibly represent the gate frame. | Adjust only after verifying real gate boxes are rejected with `aspect`. |
| `GATE_SELECTOR_MIN_APPEARANCE_SCORE` | target selector | Optional guard that rejects candidates whose crop does not look like a hollow rectangular frame. Default `0.0` disables it. | Use only after confirming the realtime stream is `rgb8` and YOLO raw output still mislabels non-gates as `3:Goals-Detection`. |
| `GATE_SELECTOR_APPEARANCE_WEIGHT` | target selector | Optional scoring weight for hollow-gate evidence when multiple candidates survive validation. Default `0.0` disables score influence. | Increase slightly only after confirming appearance scores separate real gates from false class-3 objects. |
| `GATE_SELECTOR_STABLE_WINDOW` | target selector | Number of recent frames considered for stable target validation. | Increase for noisy detections; decrease if the detector is stable but response is too delayed. |
| `GATE_SELECTOR_REQUIRED_STABLE` | target selector | Required valid frames inside the stability window before publishing `GateDetection`. | Raise to suppress flicker; lower if gate 2 is valid but acquisition reacts too late. |

### Gate 1 And Gate 2 Mission Policy

| Variable | Used by | Purpose and rationale | Affects |
| --- | --- | --- | --- |
| `MISSION_MAX_DETECTION_AGE` | all detection-using phases | Final mission-side freshness gate for selected `GateDetection`. Prevents stale perception from steering the drone after the image has changed. | Gate 1 and gate 2. |
| `MISSION_REQUIRED_DETECTION_TICKS` | `SEEK_GATE`, `NEXT_GATE_ACQUIRE` | Number of consecutive mission ticks required before switching into centering or brake-before-center. Adds temporal confirmation beyond one YOLO frame. | Gate 1 lock and gate 2 acquire. |
| `MISSION_CENTER_DWELL` | `CENTER_GATE` | Minimum time to keep visual centering before pass commit. Gives ArduPilot and the frame time to settle instead of passing on a single good frame. | Gate 1 and gate 2. |
| `MISSION_CENTER_CLEARANCE_REQUIRED` | `CENTER_GATE` | Time the clearance validator must remain true. Adds hysteresis so one lucky centered frame cannot start a pass. | Gate 1 and gate 2. |
| `MISSION_CENTER_LOST_GRACE_TICKS` | `CENTER_GATE` | Ticks tolerated after a brief detection loss before returning to seek. Prevents one missed YOLO frame from causing scan/center oscillation. | Gate 1 and gate 2. |
| `MISSION_SEEK_YAW_RATE` | `SEEK_GATE` | Yaw scan rate while no stable target exists. Positive value scans right/clockwise in body-frame convention. | Gate 1 seek and fallback seek after gate 2 acquire timeout. |
| `MISSION_GATE_READY_AREA` | `CENTER_GATE`, `NEXT_GATE_ACQUIRE` | Bbox area ratio required before brake/center or pass commit. This is image-space readiness, not metric distance. | Gate 1 pass and gate 2 acquisition/pass. |
| `MISSION_GATE_PASS_DISTANCE` | `PASS_GATE` | Forward distance commanded after pass commit. Must clear the physical gate depth and drone body, but should not become a blind sprint. | Gate 1 and gate 2 crossing. |
| `MISSION_GATE_PASS_SPEED` | `PASS_GATE` | Forward speed during committed gate crossing. Visual lateral/yaw corrections are intentionally off during this segment. | Gate 1 and gate 2 crossing. |
| `MISSION_NEXT_GATE_CLEAR_DISTANCE` | `NEXT_GATE_ACQUIRE` | Minimum forward clear distance after gate 1 before gate 2 detections can count. Prevents relocking the just-passed gate. | Gate 2 only. |
| `MISSION_NEXT_GATE_ACQUIRE_SPEED` | `NEXT_GATE_ACQUIRE` | Forward speed while looking for gate 2 after clearing gate 1. Keeps motion active without a blind timed sprint. | Gate 2 only. |
| `MISSION_NEXT_GATE_MIN_AREA` | `NEXT_GATE_ACQUIRE` | Far-gate candidate area threshold. Below this, gate 2 is considered too small/noisy to trigger centering. | Gate 2 only. |
| `MISSION_NEXT_GATE_MAX_DISTANCE` | `NEXT_GATE_ACQUIRE` | Maximum forward distance spent acquiring gate 2 before falling back to seek. Prevents endless forward travel. | Gate 2 only. |
| `MISSION_NEXT_GATE_TIMEOUT` | `NEXT_GATE_ACQUIRE` | Maximum time spent in active gate-2 acquire before fallback. Redundant with distance on purpose because telemetry or speed can be imperfect. | Gate 2 only. |
| `MISSION_BRAKE_SETTLE` | `BRAKE` | Time spent commanding zero forward velocity before entering center or land. Reduces overshoot and prevents landing while still moving forward. | Before gate 2 centering and before landing. |
| `MISSION_BRAKE_RAMP` | `BRAKE` | Time used to ramp forward speed down from the previous phase speed to zero. This reduces pitch/altitude jerk from an abrupt velocity step. Keep `<= MISSION_BRAKE_SETTLE`. | Before gate 2 centering and before landing. |
| `MISSION_BRAKE_ALTITUDE_HOLD` | `BRAKE` | Optional `0/1` switch for companion altitude correction during brake. Default `0` sends vertical velocity zero but does not add proportional altitude correction, reducing vertical bounce while decelerating. | Enable only if brake logs show altitude drift is worse than bounce. |
| `MISSION_FINAL_EXIT_DISTANCE` | `FINAL_EXIT` | Forward distance after gate 2 before landing. This is not altitude. | After gate 2. |
| `MISSION_FINAL_EXIT_SPEED` | `FINAL_EXIT` | Forward speed after gate 2 before brake and land. | After gate 2. |

Gate 2 should normally progress like this:

```text
clear previous gate distance
-> ignore tiny boxes below MISSION_NEXT_GATE_MIN_AREA
-> approach while area is between min and ready
-> brake before CENTER_GATE once area is ready and stable
```

If gate 2 is detected but never acquired, tune area/stability first. If gate 2
is acquired too early and lands after the wrong object, raise
`MISSION_NEXT_GATE_MIN_AREA`, raise `MISSION_GATE_READY_AREA`, or increase
`MISSION_NEXT_GATE_CLEAR_DISTANCE` after confirming the selected class is still
`Goals-Detection`.

### Visual Servo And Clearance

| Variable | Used by | Purpose and rationale | Tune when |
| --- | --- | --- | --- |
| `VISUAL_FRAME_WIDTH/HEIGHT` | visual servo and diagnostics | Real camera frame geometry for normalized errors and bbox area. Current Webots stream is `640x480`. | Change only when the camera stream resolution changes. |
| `VISUAL_MIN_CONFIDENCE` | mission fresh-detection filter | Minimum selected `GateDetection` confidence accepted by centering. Separate from YOLO and selector thresholds. | Lower only if selected gate confidence is consistently below this after class/selector validation. |
| `VISUAL_FILTER_ALPHA` | visual servo | Low-pass alpha for image error and area. Lower is smoother but slower. | Reduce if bbox jitter causes motion; increase if the target lags visibly. |
| `VISUAL_COMMAND_FILTER_ALPHA` | visual servo | Low-pass alpha for velocity commands. This directly smooths drone/camera shake. | Reduce first when motion is twitchy; increase only if response becomes too sluggish. |
| `VISUAL_CENTER_DEADBAND_X/Y` | visual servo | Error region where lateral/vertical correction is zero. Prevents small jitter from becoming continuous commands. | Increase for small oscillation; decrease if it never centers tightly enough. |
| `VISUAL_PASS_TARGET_OFFSET_X/Y` | visual servo | Moves the desired gate center away from camera optical center to compensate body/camera/GPS offsets. | Tune after measuring where the drone body safely fits through the gate. |
| `VISUAL_PASS_CLEARANCE_LEFT/RIGHT/UP/DOWN` | visual servo | Asymmetric image-error margins around the pass target. This is the OpenCV magenta box. | Tighten for more conservative pass alignment; loosen only if safe physical clearance is verified. |
| `VISUAL_MIN_FORWARD_SPEED` | visual servo | Minimum forward speed during centering. Current default is zero so centering can pause if alignment is poor. | Raise only if a moving approach is desired and oscillation is controlled. |
| `VISUAL_MAX_FORWARD_SPEED` | visual servo | Maximum forward speed during `CENTER_GATE`; committed crossing uses `MISSION_GATE_PASS_SPEED`. | Lower if centering overshoots; raise only after stable target lock. |
| `VISUAL_LATERAL_KP` | visual servo | Maps horizontal image error to body-right velocity. | Tune after max lateral speed and command smoothing are reasonable. |
| `VISUAL_VERTICAL_KP` | visual servo | Maps vertical image error to body-down velocity. Positive image y commands descent. | Tune cautiously; altitude guard and ArduPilot altitude hold still constrain it. |
| `VISUAL_YAW_KP` | visual servo | Maps horizontal image error to yaw-rate correction. | Lower if yaw shakes the camera; raise if the gate stays off-axis. |
| `VISUAL_MAX_LATERAL_SPEED` | visual servo | Saturation for body-right correction. Limits aggressive side motion. | Reduce if side-to-side oscillation appears. |
| `VISUAL_MAX_VERTICAL_SPEED` | visual servo | Saturation for body-down correction. Limits climb/descent from camera error. | Reduce if altitude oscillates during centering. |
| `VISUAL_MAX_YAW_RATE` | visual servo | Saturation for yaw correction. Limits camera shake from yaw. | Reduce if yaw oscillation causes target loss. |

## Pass Commit

Pass commit requires all of these:

```text
MISSION_CENTER_DWELL elapsed
MISSION_CENTER_CLEARANCE_REQUIRED satisfied
MISSION_GATE_READY_AREA reached
```

Then `PASS_GATE` commands stable forward-only body velocity plus altitude hold.
It does not keep lateral/yaw visual corrections active while crossing.

Main variables:

```text
MISSION_CENTER_DWELL
MISSION_CENTER_CLEARANCE_REQUIRED
MISSION_REQUIRED_DETECTION_TICKS
MISSION_GATE_READY_AREA
MISSION_GATE_PASS_DISTANCE
MISSION_GATE_PASS_SPEED
VISUAL_PASS_TARGET_OFFSET_X
VISUAL_PASS_TARGET_OFFSET_Y
VISUAL_PASS_CLEARANCE_LEFT
VISUAL_PASS_CLEARANCE_RIGHT
VISUAL_PASS_CLEARANCE_UP
VISUAL_PASS_CLEARANCE_DOWN
```

If the drone never exits `CENTER_GATE`, look at the diagnostics overlay:

- If the target is outside the magenta rectangle, tune pass offset/clearance.
- If the bbox is smaller than the orange ready box, tune `MISSION_GATE_READY_AREA`.
- If target selection flickers, tune selector/detection before tuning control.

## Visual Servo Smoothness

Visual servoing uses image error, not metric distance. The current controller is
intentionally conservative to avoid camera shake and lateral oscillation.

The implementation is not PID/FF. It is:

```text
YOLO bbox -> target selector smoothing
-> image error low-pass
-> deadband
-> proportional velocity/yaw command
-> speed/rate saturation
-> command low-pass
-> ArduPilot body velocity command
```

That choice is deliberate for the current stage. A full PID or feed-forward
controller can help later, but only after the visual signal and vehicle response
are logged. With YOLO boxes, an untuned PID/FF layer can make motion worse:

- Integral can wind up while detections flicker or while commands saturate.
- Derivative amplifies bbox jitter and camera vibration.
- Feed-forward needs a measured relationship between command, vehicle response,
  camera motion, and detector latency. Guessing it often creates overshoot.
- ArduPilot already closes inner attitude/rate loops; the companion should send
  smooth setpoints, not fight the firmware with high-frequency corrections.

Main variables:

```text
VISUAL_FILTER_ALPHA
VISUAL_COMMAND_FILTER_ALPHA
VISUAL_CENTER_DEADBAND_X
VISUAL_CENTER_DEADBAND_Y
VISUAL_MAX_ERROR_FOR_FORWARD
VISUAL_MAX_FORWARD_SPEED
VISUAL_LATERAL_KP
VISUAL_VERTICAL_KP
VISUAL_YAW_KP
VISUAL_MAX_LATERAL_SPEED
VISUAL_MAX_VERTICAL_SPEED
VISUAL_MAX_YAW_RATE
```

If the drone shakes while target selection is stable:

1. Confirm the OpenCV selected box is stable. If it is not stable, tune selector
   stability and detection stale time first.
2. Reduce `VISUAL_MAX_LATERAL_SPEED` and `VISUAL_MAX_YAW_RATE`.
3. Reduce `VISUAL_COMMAND_FILTER_ALPHA` for smoother command changes.
4. Reduce `VISUAL_FILTER_ALPHA` only if the bbox itself jitters while the
   selected target identity is stable.
5. Reduce `VISUAL_LATERAL_KP` or `VISUAL_YAW_KP` only after speed/rate limits
   and filters are reasonable.

If the drone is too sluggish:

1. Increase max speeds slightly.
2. Increase gains slightly.
3. Keep diagnostics open and watch `servo_err=(x,y)`.

Recommended jerky-motion starting preset:

```bash
VISUAL_MAX_FORWARD_SPEED=0.20 \
VISUAL_MAX_LATERAL_SPEED=0.14 \
VISUAL_MAX_YAW_RATE=0.10 \
VISUAL_COMMAND_FILTER_ALPHA=0.16 \
VISUAL_FILTER_ALPHA=0.14 \
SEND_COMMANDS=1 \
bash scripts/run_iris_camera_yolo.sh
```

If this becomes too slow but no longer shakes, increase one value at a time:

```text
VISUAL_MAX_FORWARD_SPEED  +0.05 m/s per test
VISUAL_MAX_LATERAL_SPEED  +0.03 m/s per test
VISUAL_MAX_YAW_RATE       +0.03 rad/s per test
VISUAL_COMMAND_FILTER_ALPHA +0.03 per test
```

Do not change `MISSION_GATE_PASS_SPEED` to fix centering jerk. That speed is
used only after pass commit, when lateral/yaw visual corrections are off.

### Smoothness Variables That Collide

Some variables look related but affect different failure modes:

| If you see this | Tune first | Avoid changing first |
| --- | --- | --- |
| Green selected box jumps between objects | `GATE_SELECTOR_STABLE_WINDOW`, `GATE_SELECTOR_REQUIRED_STABLE`, class filter, `WEBOTS_DETECTION_STALE` | `VISUAL_*_KP` |
| Selected box is stable but camera shakes left/right | `VISUAL_MAX_LATERAL_SPEED`, `VISUAL_MAX_YAW_RATE`, `VISUAL_COMMAND_FILTER_ALPHA` | `MISSION_CENTER_DWELL` |
| Drone approaches before it is centered | Lower `VISUAL_MAX_ERROR_FOR_FORWARD`, lower `VISUAL_MAX_FORWARD_SPEED` | `MISSION_GATE_PASS_DISTANCE` |
| Drone never approaches during centering | Raise `VISUAL_MAX_ERROR_FOR_FORWARD` slightly, raise `VISUAL_MAX_FORWARD_SPEED` slightly | `MISSION_GATE_READY_AREA` |
| Drone passes too early | Raise `MISSION_GATE_READY_AREA`, tighten `VISUAL_PASS_CLEARANCE_*`, increase `MISSION_CENTER_CLEARANCE_REQUIRED` | `VISUAL_LATERAL_KP` |
| Drone brakes/lands while still moving | Increase `MISSION_BRAKE_SETTLE`, lower final/pass speed | Detector thresholds |
| Drone bounces up/down during BRAKE | Increase `MISSION_BRAKE_RAMP`, keep `MISSION_BRAKE_ALTITUDE_HOLD=0`, lower incoming speed | Visual PID/PD |

## Brake Smoothness

`BRAKE` is not a visual-servo phase. It exists to bleed off forward motion
before gate-2 centering or before landing. Jerky motion here is usually caused
by an abrupt forward velocity step or by vertical correction fighting the
vehicle's pitch/altitude transient during deceleration.

Current brake behavior:

```text
entry speed from previous phase
-> ramp body_vx_m_s down over MISSION_BRAKE_RAMP seconds
-> hold body_vz_m_s at 0 by default
-> wait MISSION_BRAKE_SETTLE seconds before next phase
```

Default:

```text
MISSION_BRAKE_SETTLE=1.00
MISSION_BRAKE_RAMP=0.70
MISSION_BRAKE_ALTITUDE_HOLD=0
```

If brake still kicks the drone:

1. Set `MISSION_BRAKE_RAMP` closer to `MISSION_BRAKE_SETTLE`, for example
   `0.90` when settle is `1.00`.
2. Lower the speed entering brake:
   `MISSION_NEXT_GATE_ACQUIRE_SPEED`, `MISSION_FINAL_EXIT_SPEED`, or
   `MISSION_GATE_PASS_SPEED`, depending on which transition jerks.
3. Increase `MISSION_BRAKE_SETTLE` if the drone needs more time to settle after
   the ramp ends.
4. Keep `MISSION_BRAKE_ALTITUDE_HOLD=0` unless logs show clear altitude drift
   during brake. If enabled, the companion altitude P controller can reintroduce
   vertical bounce.

Example smoother brake test:

```bash
MISSION_BRAKE_SETTLE=1.50 \
MISSION_BRAKE_RAMP=1.20 \
MISSION_BRAKE_ALTITUDE_HOLD=0 \
MISSION_NEXT_GATE_ACQUIRE_SPEED=1.20 \
MISSION_FINAL_EXIT_SPEED=1.20 \
SEND_COMMANDS=1 \
bash scripts/run_iris_camera_yolo.sh
```

### When PID/FF Becomes Worth It

Consider a dedicated controller upgrade only after the current filtered-P path
is logged and the remaining problem is clearly setpoint tracking, not detection
or phase logic. A safe upgrade path would be:

1. Add command slew-rate limiting before PID/FF.
2. Add PI only on lateral/yaw with anti-windup and reset-on-detection-loss.
3. Add D only on filtered measurement, not raw YOLO bbox error.
4. Add feed-forward only from measured vehicle response or optical-flow/visual
   odometry, not from guessed constants.
5. Keep all new terms disabled by default and covered by SITL tests.

Until those logs exist, the current best practical tuning lever is smoother
velocity setpoints, not a more complex controller.

## Selector Thresholds

The selector runs after YOLO class filtering and before mission/control sees a
target.

Main variables:

```text
GATE_SELECTOR_MIN_SEEK_CONFIDENCE
GATE_SELECTOR_MIN_TRACK_CONFIDENCE
GATE_SELECTOR_MIN_AREA_RATIO
GATE_SELECTOR_MIN_ASPECT_RATIO
GATE_SELECTOR_MAX_ASPECT_RATIO
GATE_SELECTOR_MIN_APPEARANCE_SCORE
GATE_SELECTOR_APPEARANCE_WEIGHT
GATE_SELECTOR_STABLE_WINDOW
GATE_SELECTOR_REQUIRED_STABLE
```

Use OpenCV rejection reasons:

```text
confidence  lower confidence threshold or improve model/lighting
area_small  lower area threshold if the real gate is still far
aspect      adjust aspect limits only if the true gate bbox shape is valid
roi         confirm the gate is in the expected region before loosening ROI
appearance  lower appearance threshold only if the true hollow gate is rejected
appearance_missing  confirm the detector is receiving image-backed candidates
```

If a non-gate object is accepted, do not loosen selector thresholds. First check
the diagnostics `raw=...` line. A raw non-gate class such as `1:Dog` or
`2:Forklift` should be filtered out; if it is not, the class filter is wrong.
If the non-gate object is already raw-labeled `3:Goals-Detection`, confirm the
frame line is `rgb8` and only then consider `GATE_SELECTOR_MIN_APPEARANCE_SCORE`
as an optional guard. Treat persistent false class-3 detections as
dataset/model/domain work, not mission logic.

## What Not To Tune In Code

Do not edit these files just to tune experiments:

```text
scripts/run_autonomy_sitl.sh
scripts/run_iris_camera_yolo.sh
src/drone_autonomy/autonomy/mission.py
src/drone_autonomy/control/visual_servo.py
```

Use `configs/autonomy_runtime.env` or inline env overrides. Edit code only when
the behavior contract itself is wrong.

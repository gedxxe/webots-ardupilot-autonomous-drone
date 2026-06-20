# Webots YOLO Gate Pipeline

This document describes the real perception path for simulation:

```text
Webots iris_camera.wbt
-> ArduPilot Webots TCP camera stream
-> background WebotsTcpCameraClient worker
-> latest-frame snapshot
-> background YoloGateDetector worker
-> raw GateCandidate list
-> GateTargetSelector validation/tracking
-> latest-detection snapshot
-> GateDetection
-> MissionTelemetry.gate_detection
-> GateAutonomyMission
```

The mission state machine does not import YOLO, Webots, NumPy, threading, or
camera code. The runtime only reads the latest fresh detection snapshot. That
boundary is intentional so the same mission can later use a Raspberry Pi 5 plus
Logitech C920 adapter.

## Concurrency Model

`webots-yolo` uses two background workers:

- Camera worker: owns the Webots TCP socket and publishes only the newest frame.
- Detector worker: consumes the newest frame, runs YOLO, validates/tracks gate
  candidates, and publishes only the newest selected `GateDetection | None`.

This is a bounded-latest design. It intentionally drops old frames instead of
building an unbounded queue, because drone control needs the newest perception
more than old image history. The mission loop remains deterministic and
single-threaded; it never blocks on TCP reads or YOLO inference.

Detection reuse is limited by:

```text
WEBOTS_DETECTION_STALE="0.75"
```

If the latest detection is older than this threshold, the mission receives
`None` and continues its normal seek behavior.

## Current Webots Camera Source

`webots/worlds/iris_camera.wbt` already configures the Iris controller with:

```text
--camera camera
--camera-port 5599
--camera-format rgb24
```

The `Camera` device in the Iris `extensionSlot` must also be named `camera`.
The Webots controller looks it up with `robot.getDevice("camera")`; if a custom
world omits that exact device name, the TCP camera stream will not start.

This repo's vendored Webots controller streams frames over TCP:

```text
host: 127.0.0.1
port: 5599
format: uint16 width, uint16 height, then rgb24 pixels
```

The client preserves partial frame bytes across normal socket timeouts. Webots
streams at camera FPS, while the autonomy loop may poll faster; a timeout before
the next frame is not treated as a broken camera stream.

If a connected socket receives no bytes for `WEBOTS_CAMERA_IDLE_RECONNECT`
seconds, the client reconnects. This avoids getting stuck behind a stale TCP
connection while keeping the mission state machine independent from camera I/O.

Use the camera-only probe before blaming YOLO:

```bash
python scripts/probe_webots_camera.py --host 127.0.0.1 --port 5599
```

Important: upstream ArduPilot's original Webots helper streams grayscale by
default. This repo's vendored copy adds `--camera-format rgb24` for
`iris_camera.wbt` so simulation input is closer to ordinary RGB video. If the
diagnostics window shows `rgb8_from_gray8`, you are still on the old grayscale
path or `WEBOTS_CAMERA_ENCODING` is wrong.

## Current Gate Model

The trained simulation model is stored in the repo:

```text
models/gate_yolov8n_best.pt
```

Model metadata observed during audit:

```text
class id 0 -> AdvertisementBox
class id 1 -> Dog
class id 2 -> Forklift
class id 3 -> Goals-Detection
class id 4 -> Table
```

Use fail-closed name and id filtering for this model:

```text
YOLO_GATE_CLASS_NAMES="Goals-Detection"
YOLO_GATE_CLASS_IDS="3"
```

## Install Vision Dependencies

YOLO dependencies are optional because heartbeat, listen, and synthetic tests do
not need them.

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
source .venv/bin/activate
pip install -e ".[dev,vision]"
```

If you reuse the ArduPilot virtualenv:

```bash
source /media/gedxxe/DATA/venv-ardupilot/bin/activate
cd /media/gedxxe/DATA/WeBots_Ardupilot
pip install -e ".[dev,vision]"
```

## Webots World

Use the camera world:

```bash
webots webots/worlds/iris_camera.wbt
```

Expected Webots console clue:

```text
Camera stream started at 127.0.0.1:5599
```

For SITL config, set:

```text
WEBOTS_WORLD="worlds/iris_camera.wbt"
WEBOTS_PARAM_FILE="params/iris.parm"
```

## Autonomy Runtime Config

Edit `configs/autonomy_runtime.env`:

```text
DETECTOR="webots-yolo"
YOLO_MODEL_PATH="${REPO_ROOT}/models/gate_yolov8n_best.pt"
YOLO_CONFIDENCE="0.35"
YOLO_IMGSZ="640"
YOLO_GATE_CLASS_NAMES="Goals-Detection"
YOLO_GATE_CLASS_IDS="3"
YOLO_DEVICE="cpu"
YOLO_CONFIG_DIR="${REPO_ROOT}/.tmp_ultralytics"

GATE_SELECTOR_MIN_SEEK_CONFIDENCE="0.40"
GATE_SELECTOR_MIN_TRACK_CONFIDENCE="0.30"
GATE_SELECTOR_MIN_AREA_RATIO="0.0015"
GATE_SELECTOR_MIN_ASPECT_RATIO="0.35"
GATE_SELECTOR_MAX_ASPECT_RATIO="4.0"
GATE_SELECTOR_MIN_APPEARANCE_SCORE="0.0"
GATE_SELECTOR_APPEARANCE_WEIGHT="0.0"
GATE_SELECTOR_STABLE_WINDOW="5"
GATE_SELECTOR_REQUIRED_STABLE="3"

WEBOTS_CAMERA_HOST="127.0.0.1"
WEBOTS_CAMERA_PORT="5599"
WEBOTS_CAMERA_ENCODING="rgb24"
WEBOTS_DETECTION_STALE="0.75"

MISSION_MAX_DETECTION_AGE="0.75"
MISSION_REQUIRED_DETECTION_TICKS="2"
MISSION_CENTER_LOST_GRACE_TICKS="10"
MISSION_BRAKE_SETTLE="1.00"
MISSION_BRAKE_RAMP="0.70"
MISSION_BRAKE_ALTITUDE_HOLD="0"

SEND_COMMANDS="0"
```

`YOLO_IMGSZ` is the Ultralytics inference size. It is not the camera frame
geometry. The current Webots frame is 640x480 and is controlled by
`VISUAL_FRAME_WIDTH` / `VISUAL_FRAME_HEIGHT`; YOLO letterboxes/resizes that
frame internally for inference.

Use `SEND_COMMANDS="0"` until the detector is visibly stable. The runtime will
run perception and mission decisions without moving the vehicle.

## Dry-Run

Fast profile command:

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
source .venv/bin/activate
bash scripts/run_iris_camera_yolo.sh
```

That wrapper selects `AUTONOMY_PROFILE="iris-camera-yolo"`. The generic runner
then reads `configs/autonomy_runtime.env`, applies profile-owned defaults for
detector/diagnostics, and forwards only the env values that are actually set.
Do not add experiment thresholds directly to the wrapper script.

Equivalent explicit command:

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
source .venv/bin/activate
DETECTOR=webots-yolo \
YOLO_MODEL_PATH="${PWD}/models/gate_yolov8n_best.pt" \
YOLO_GATE_CLASS_NAMES="Goals-Detection" \
YOLO_GATE_CLASS_IDS="3" \
SEND_COMMANDS=0 \
scripts/run_autonomy_sitl.sh
```

Expected startup lines include:

```text
autonomy connection=udp:127.0.0.1:14551 detector=webots-yolo command_mode=dry-run loop_hz=20.0
webots-yolo camera=tcp://127.0.0.1:5599 encoding=rgb24 model=.../models/gate_yolov8n_best.pt
```

If the model path is wrong, the gate is not visible, or the class filter is
wrong, the mission should remain in `seek_gate` and continue scanning. That is
safe behavior.

## Detector Class Filtering

For the current bundled model:

```text
YOLO_GATE_CLASS_NAMES="Goals-Detection"
YOLO_GATE_CLASS_IDS="3"
```

This means only detections whose YOLO class name is `Goals-Detection` and whose
id is `3` become `GateDetection`. The id guard prevents
Dog/Forklift/Table from becoming targets with the current model.

For a replacement multi-class model, recheck the class map. If the training
labels are ordered like this:

```text
AdvertisementBox, Dog, Forklift, Goals-Detection, Table
```

then `Goals-Detection` is usually class id `3`, not `0`. The current repo
default already uses the matching name and id:

```bash
YOLO_GATE_CLASS_NAMES="Goals-Detection" \
YOLO_GATE_CLASS_IDS="3" \
WEBOTS_DIAGNOSTICS_WINDOW=1 \
SEND_COMMANDS=0 \
bash scripts/run_iris_camera_yolo.sh
```

If retraining changes the numeric id but keeps the label stable, use name-only
filtering after confirming the metadata:

```bash
YOLO_GATE_CLASS_NAMES="Goals-Detection" \
YOLO_GATE_CLASS_IDS="" \
WEBOTS_DIAGNOSTICS_WINDOW=1 \
SEND_COMMANDS=0 \
bash scripts/run_iris_camera_yolo.sh
```

If a future model uses a literal class name `gate`, use:

```text
YOLO_GATE_CLASS_NAMES="gate"
YOLO_GATE_CLASS_IDS=""
```

Do not accept every class during motion tests. If class filtering is empty and
the model sees unrelated objects, the mission may center on the wrong target.
The runtime now rejects an empty YOLO gate filter instead of silently accepting
all classes.

## Target Selection

YOLO returns raw gate candidates. `GateTargetSelector` then performs validation,
tracking, smoothing, and target selection before mission/control sees a
`GateDetection`.

Candidate validation checks:

- class-filtered YOLO confidence,
- minimum area ratio,
- aspect-ratio range,
- hollow-gate appearance score from image edges,
- candidate center inside the current validation ROI,
- stable target hits across a small frame window.

When two gate boxes are visible, the selector does not choose by confidence
alone. The scoring policy prioritizes:

1. Larger bounding-box area as a monocular proxy for the nearer gate.
2. Closer center to the image crosshair.
3. YOLO confidence.
4. Optional hollow-gate appearance evidence if enabled.
5. Small lock bonus for overlap with the previously selected target.

This keeps the drone focused on the gate it is currently approaching instead of
jumping to the farther gate behind it.

If Dog, Forklift, or another object is displayed as `cls=3:Goals-Detection`, the
model has already emitted the wrong class. The class filter is still working; it
cannot know the object was visually a dog after YOLO labels it as class 3. The
diagnostics overlay prints `raw=...` to show the raw class counts before this
pipeline class-filters them. A raw non-gate class should disappear before target
selection; `raw=3:Goals-Detection` on a non-gate is model/input-domain behavior.
The appearance score can be enabled as a second guard for this case, but it is
off by default. First verify the RGB stream; then tune
`GATE_SELECTOR_MIN_APPEARANCE_SCORE` only if false positives persist.

## Diagnostics Window

Enable the optional OpenCV view:

```bash
WEBOTS_DIAGNOSTICS_WINDOW=1 SEND_COMMANDS=0 bash scripts/run_iris_camera_yolo.sh
```

The window draws:

- validation ROI boundary,
- image crosshair,
- pass-clearance target and margin box from `VISUAL_PASS_TARGET_OFFSET_*` and
  `VISUAL_PASS_CLEARANCE_*`,
- far/ready area reference boxes from `MISSION_NEXT_GATE_MIN_AREA` and
  `MISSION_GATE_READY_AREA`,
- rejected candidates in red with rejection reasons,
- accepted candidates in yellow,
- selected stable target in green,
- score, hollow-gate appearance score, area ratio, aspect ratio, center error,
  stable count, and lost count.

Use this only on a desktop session with GUI support. Leave it disabled for
headless runs.

## Safe Progression

1. Run `webots-yolo` with `SEND_COMMANDS="0"`.
2. Confirm the status changes from `seeking gate` to `centering gate` only when
   the visible gate is in frame.
3. Confirm wrong objects do not become gates.
4. If two gates are visible, confirm the selected box remains on the nearer gate.
5. Confirm body-frame signs using synthetic or very low-speed Webots tests.
6. Only then set `SEND_COMMANDS="1"` in SITL.

This pipeline is not a substitute for hardware safety. Real flight still needs
an explicit hardware camera adapter, flight envelope limits, emergency stop
procedure, and environment-specific tuning.

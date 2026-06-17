# Webots YOLO Gate Pipeline

This document describes the real perception path for simulation:

```text
Webots iris_camera.wbt
-> ArduPilot Webots TCP camera stream
-> WebotsTcpCameraClient
-> YoloGateDetector
-> GateDetection
-> MissionTelemetry.gate_detection
-> GateAutonomyMission
```

The mission state machine does not import YOLO, Webots, NumPy, or camera code.
That boundary is intentional so the same mission can later use a Raspberry Pi 5
plus Logitech C920 adapter.

## Current Webots Camera Source

`webots/worlds/iris_camera.wbt` already configures the Iris controller with:

```text
--camera camera
--camera-port 5599
```

The `Camera` device in the Iris `extensionSlot` must also be named `camera`.
The Webots controller looks it up with `robot.getDevice("camera")`; if a custom
world omits that exact device name, the TCP camera stream will not start.

The upstream ArduPilot Webots controller streams frames over TCP:

```text
host: 127.0.0.1
port: 5599
format: uint16 width, uint16 height, then gray8 pixels
```

The client preserves partial frame bytes across normal socket timeouts. Webots
streams at camera FPS, while the autonomy loop may poll faster; a timeout before
the next frame is not treated as a broken camera stream.

Important: this upstream stream is grayscale. The adapter expands it to three
identical channels before YOLO inference. This is acceptable for simulation
wiring and shape-based gate tests, but it is not a true RGB camera validation.
A future C920/OpenCV source should provide real RGB frames without changing the
YOLO detector or mission state machine.

## Current Gate Model

The trained simulation model is stored in the repo:

```text
models/gate_yolov8n_best.pt
```

Model metadata observed during audit:

```text
class id 0 -> Goals-Detection
```

Use class id filtering for this model:

```text
YOLO_GATE_CLASS_NAMES=""
YOLO_GATE_CLASS_IDS="0"
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
YOLO_GATE_CLASS_NAMES=""
YOLO_GATE_CLASS_IDS="0"
YOLO_DEVICE="cpu"
YOLO_CONFIG_DIR="${REPO_ROOT}/.tmp_ultralytics"

WEBOTS_CAMERA_HOST="127.0.0.1"
WEBOTS_CAMERA_PORT="5599"
WEBOTS_CAMERA_ENCODING="gray8"

SEND_COMMANDS="0"
```

Use `SEND_COMMANDS="0"` until the detector is visibly stable. The runtime will
run perception and mission decisions without moving the vehicle.

## Dry-Run

Fast profile command:

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
source .venv/bin/activate
bash scripts/run_iris_camera_yolo.sh
```

Equivalent explicit command:

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
source .venv/bin/activate
DETECTOR=webots-yolo \
YOLO_MODEL_PATH="${PWD}/models/gate_yolov8n_best.pt" \
YOLO_GATE_CLASS_NAMES="" \
YOLO_GATE_CLASS_IDS="0" \
SEND_COMMANDS=0 \
scripts/run_autonomy_sitl.sh
```

Expected startup lines include:

```text
autonomy connection=udp:127.0.0.1:14550 detector=webots-yolo command_mode=dry-run loop_hz=20.0
webots-yolo camera=tcp://127.0.0.1:5599 encoding=gray8 model=.../models/gate_yolov8n_best.pt
```

If the model path is wrong, the gate is not visible, or the class filter is
wrong, the mission should remain in `seek_gate` and continue scanning. That is
safe behavior.

## Detector Class Filtering

For the bundled model:

```text
YOLO_GATE_CLASS_NAMES=""
YOLO_GATE_CLASS_IDS="0"
```

This means only detections whose YOLO class id is `0` become `GateDetection`.
This is required because the current model class name is `Goals-Detection`, not
`gate`.

If a future model uses a literal class name `gate`, use:

```text
YOLO_GATE_CLASS_NAMES="gate"
YOLO_GATE_CLASS_IDS=""
```

Do not accept every class during motion tests. If class filtering is empty and
the model sees unrelated objects, the mission may center on the wrong target.

## Safe Progression

1. Run `webots-yolo` with `SEND_COMMANDS="0"`.
2. Confirm the status changes from `seeking gate` to `centering gate` only when
   the visible gate is in frame.
3. Confirm wrong objects do not become gates.
4. Confirm body-frame signs using synthetic or very low-speed Webots tests.
5. Only then set `SEND_COMMANDS="1"` in SITL.

This pipeline is not a substitute for hardware safety. Real flight still needs
an explicit hardware camera adapter, flight envelope limits, emergency stop
procedure, and environment-specific tuning.

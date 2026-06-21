# Troubleshooting Q&A

## `drone-autonomy: command not found`

The package entry point is not installed in the active shell. Activate the
virtual environment and install the package:

```bash
cd /path/to/WeBots_Ardupilot
source .venv/bin/activate
pip install -e ".[dev]"
```

Alternative:

```bash
python -m drone_autonomy.cli --mode heartbeat
```

If the error came from `scripts/run_autonomy_sitl.sh`, update to the latest repo
version first. The script now prefers running `python -m drone_autonomy.cli`
from this repo's `src/` tree before using any `drone-autonomy` executable from
`PATH`.

If you want to reuse the ArduPilot virtualenv on the DATA partition:

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
source /media/gedxxe/DATA/venv-ardupilot/bin/activate
pip install -e ".[dev]"
scripts/run_autonomy_sitl.sh
```

Or set an explicit Python path in `configs/autonomy_runtime.env`:

```text
PYTHON_BIN="/media/gedxxe/DATA/venv-ardupilot/bin/python"
```

## `ModuleNotFoundError: No module named 'pymavlink'`

Dependencies are not installed in the active Python environment.

```bash
source .venv/bin/activate
pip install -e ".[dev]"
```

Then verify:

```bash
python -c "from pymavlink import mavutil; print('ok')"
```

## `No heartbeat from udp:127.0.0.1:14551`

Check these in order:

1. SITL is running.
2. SITL was launched with `MAVLINK_OUT_EXTRA="udp:127.0.0.1:14551"` or the
   launcher default extra output is active.
3. You are using the same endpoint in `drone-autonomy`.
4. No firewall or network namespace is blocking UDP.

Run:

```bash
drone-autonomy --mode listen --connection udp:127.0.0.1:14551 --count 5
```

If no messages print, the issue is before the autonomy code.

## `No heartbeat from /dev/ttyACM0`

This is the Raspberry Pi/Pixhawk serial path, not SITL UDP.

Check these in order:

1. Pixhawk is connected over USB and powered.
2. Linux created `/dev/ttyACM0` or `/dev/ttyACM1`.
3. Your user can read/write the device, or you are using the correct udev/dialout setup.
4. `MAVLINK_BAUD` matches the ArduPilot serial setting. Start with `115200`.

Smoke test:

```bash
drone-autonomy --mode heartbeat --connection /dev/ttyACM0 --baud 115200
```

Fallback device:

```bash
drone-autonomy --mode heartbeat --connection /dev/ttyACM1 --baud 115200
```

The Raspberry Pi launcher is still a dry-run scaffold. It does not enable real
C920 detection and should not be treated as a validated flight launcher.

## Mission Planner disconnects while autonomy is running

This is usually UDP endpoint contention, not a blocking mission loop. Mission
Planner and the Python autonomy process should not both consume the same local
UDP endpoint.

Use one MAVLink output for Mission Planner and another for autonomy:

```text
# configs/sitl_webots.env
MAVLINK_OUT="udp:127.0.0.1:14550"
MAVLINK_OUT_EXTRA="udp:127.0.0.1:14551"
```

Then point this repo's autonomy runtime at the second port:

```text
# configs/autonomy_runtime.env
MAVLINK_CONNECTION="udp:127.0.0.1:14551"
```

The `scripts/run_iris_camera_yolo.sh` profile uses `14551` unless local config
or inline env says otherwise. If Mission Planner only reconnects after stopping
autonomy, check whether you launched the generic runner or passed
`MAVLINK_CONNECTION=udp:127.0.0.1:14550`.

## Runtime prints `waiting for LOCAL_POSITION_NED telemetry`

The runtime received heartbeat but has not received local position.

Check:

- ArduPilot SITL has finished booting.
- Vehicle is not stuck before EKF/local-position initialization.
- Telemetry stream request was accepted.
- `LOCAL_POSITION_NED` appears in listen mode.

Inspect raw messages:

```bash
drone-autonomy --mode listen --connection udp:127.0.0.1:14551 --count 50
```

If `LOCAL_POSITION_NED` never appears, fix SITL/stream configuration first.

## Dry-run stays at `cmd=set_mode` or `cmd=arm`

This is expected if `--send-commands` is not used.

Dry-run prints decisions but does not command ArduPilot. To progress automatically in SITL:

```bash
drone-autonomy --mode autonomy --detector synthetic --send-commands
```

Only do this in SITL.

## Vehicle does not take off after `--send-commands`

Check:

- SITL is in a healthy EKF state.
- Mode actually changed to `GUIDED`.
- ArduPilot accepted arm command.
- MAVProxy console does not show prearm failures.
- Runtime first prints `sent phase=takeoff ... cmd=takeoff`.
- TAKEOFF remains on `cmd=takeoff` until fused altitude is stable near `1.0 m`.
- ArduPilot accepts the guided takeoff command while armed.

The expected takeoff profile is:

```text
cmd=takeoff -> ArduPilot-managed target 1.0 m
```

The mission intentionally does not send body-z velocity during TAKEOFF. If it
stays on the ground and only prints repeated `cmd=takeoff`, check MAVProxy for
prearm, mode, EKF, or command rejection messages before changing mission logic.

Current code does not yet parse `COMMAND_ACK`, so MAVProxy output is the best immediate clue.

## Vehicle overshoots the 1 m takeoff target

The previous hybrid takeoff idea, `0.35 m` bootstrap plus companion body-z
velocity, overshot in SITL and has been removed. TAKEOFF now delegates altitude
control to ArduPilot through `MAV_CMD_NAV_TAKEOFF`.

First log the runtime phase and altitude:

```bash
drone-autonomy --mode listen --connection udp:127.0.0.1:14551 --count 80
```

Then watch whether the transition is:

```text
takeoff/cmd=takeoff -> seek_gate
```

If SITL still overshoots badly, do not tune removed companion gains. Check
ArduPilot altitude parameters, vehicle mass/thrust model, Webots physics, and
whether `LOCAL_POSITION_NED.z` has the expected sign.

## Vehicle moves the wrong direction

Stop the test and verify body-frame velocity signs.

Internal convention:

```text
body_vx_m_s > 0: forward
body_vy_m_s > 0: right
body_vz_m_s > 0: down
yaw_rate_rad_s > 0: yaw right/clockwise
```

If Webots or ArduPilot behavior disagrees, fix the MAVLink command adapter before continuing.

## Gate pass, next-gate clear, or final exit distance feels wrong

Check `COURSE_FORWARD_X` and `COURSE_FORWARD_Y`.

The mission measures forward distance by projecting `LOCAL_POSITION_NED` onto
the configured course direction. If course direction is wrong, gate pass
distance, next-gate clear distance, and final forward exit distance are wrong.

## Synthetic detector completes too easily

That is expected. Synthetic detector is not perception. It returns centered boxes to test mission and MAVLink wiring only.

Use `--detector webots-yolo` with a trained gate model before evaluating gate behavior.

## Synthetic detector alternates between `seek_gate` and `center_gate`

This should not happen after the synthetic provider fix. The synthetic detector
keeps its fake detection active across `SEEK_GATE -> CENTER_GATE`; it should not
reset only because the mission started centering.

If oscillation returns, check that the script is running this repo's current
source and not a stale `drone-autonomy` executable:

```bash
SEND_COMMANDS=1 DETECTOR=synthetic scripts/run_autonomy_sitl.sh
```

The launcher should use `python -m drone_autonomy.cli` from this repo's `src/`
tree. Inline values such as `SEND_COMMANDS=1` override
`configs/autonomy_runtime.env`, so use the inline command above for a one-shot
motion test even when your local env file keeps the safe default
`SEND_COMMANDS="0"`.

## Vehicle overshoots gate 2 in synthetic test

Synthetic detection has no real depth or camera timing. The real mitigation is:

- adaptive next-gate acquire,
- brake-before-center,
- tuned acquire speed,
- real detector timing in Webots.

Tune after the real world and camera adapter exist.

## `--detector webots-yolo` says `--yolo-model` is required

Set the model path in `configs/autonomy_runtime.env`:

```text
DETECTOR="webots-yolo"
YOLO_MODEL_PATH="${REPO_ROOT}/models/gate_yolov8n_best.pt"
YOLO_GATE_CLASS_NAMES="Goals-Detection"
YOLO_GATE_CLASS_IDS="3"
```

The profile script `scripts/run_iris_camera_yolo.sh` enforces the `webots-yolo`
detector and diagnostics defaults, then the runner uses this model path if it is
present. Keep custom model paths and class filters in
`configs/autonomy_runtime.env` or inline env.

## `Ultralytics YOLO is required`

Install the optional vision dependencies in the active Python environment:

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
source .venv/bin/activate
pip install -e ".[dev,vision]"
```

If you use `venv-ardupilot`, activate that environment first and run the same
install command from this repo.

## Ultralytics cannot write its settings directory

If Ultralytics fails while creating a user settings directory, point it at a
writable local path before running the autonomy profile:

```bash
export YOLO_CONFIG_DIR="$PWD/.tmp_ultralytics"
bash scripts/run_iris_camera_yolo.sh
```

The `.tmp_ultralytics/` folder is ignored by git.

## `webots-yolo` stays in `seeking gate`

Check these in order:

1. Webots opened `webots/worlds/iris_camera.wbt`, not `iris.wbt`.
2. The Iris `extensionSlot` contains `Camera { name "camera" ... }`.
3. The Iris `controllerArgs` contain `--camera camera` and
   `--camera-port 5599`.
4. Webots console printed `Camera stream started at 127.0.0.1:5599`.
5. `WEBOTS_CAMERA_PORT="5599"` in `configs/autonomy_runtime.env`.
6. `YOLO_MODEL_PATH` points to a real `.pt` model.
7. The model class filter matches your training labels.

Current model-safe class filter:

```text
YOLO_GATE_CLASS_NAMES="Goals-Detection"
YOLO_GATE_CLASS_IDS="3"
```

The current `models/gate_yolov8n_best.pt` model reports class name
`Goals-Detection`. In the current multi-class metadata, that label is id `3`;
id `0` is not a gate.

For a multi-class model, do not assume `Goals-Detection` is still id `0`. YOLO
class ids follow the training `data.yaml` class order. For example, this order:

```text
AdvertisementBox, Dog, Forklift, Goals-Detection, Table
```

usually means:

```text
0=AdvertisementBox
1=Dog
2=Forklift
3=Goals-Detection
4=Table
```

In that case, keep both the matching name and id filters active:

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

An explicit empty `YOLO_GATE_CLASS_IDS=""` is valid only when using
class-name-only filtering after a class-map audit.

Do not clear both filters during motion tests. The runtime rejects an empty
YOLO gate filter because otherwise the mission may center on Dog, Forklift,
AdvertisementBox, Table, or any other non-gate object.

If a Dog/Forklift/Table is drawn as `cls=3:Goals-Detection`, first inspect the
diagnostics `raw=...` line. If it also shows `3:Goals-Detection`, the model
itself is emitting the gate class for that object; the class filter is not able
to reject it because the label and id already match the allowed gate class.
Before adding more filters, confirm the realtime stream is true RGB:

```text
webots-yolo camera frame ready 640x480 encoding=rgb8
```

If the frame line says `rgb8_from_gray8`, the run is still using the old
grayscale path and the model is being tested on a different input domain than
normal RGB video.

Only after RGB is confirmed, use `GATE_SELECTOR_MIN_APPEARANCE_SCORE` as an
optional second guard:

```bash
GATE_SELECTOR_MIN_APPEARANCE_SCORE=0.08 \
GATE_SELECTOR_APPEARANCE_WEIGHT=0.08 \
WEBOTS_DIAGNOSTICS_WINDOW=1 \
SEND_COMMANDS=0 \
bash scripts/run_iris_camera_yolo.sh
```

The default value is `0.0`, so this guard is disabled unless you opt in. If the
real gate is rejected with reason `appearance`, lower that threshold or disable
it again with `GATE_SELECTOR_MIN_APPEARANCE_SCORE=0.0`. If false class-3
detections remain common on confirmed RGB frames, the durable fix is dataset
cleanup/retraining, not a looser class filter.

If runtime prints:

```text
webots-yolo waiting for camera frame tcp://127.0.0.1:5599
```

read the `status=... detail=...` suffix:

- `connect_failed`: Webots is not listening on that host/port, or the controller
  did not start.
- `waiting_for_header`: TCP connected, but no frame header has arrived yet.
- `waiting_for_payload`: TCP connected and a frame is partially buffered.
- `header_idle_reconnect` or `payload_idle_reconnect`: the client stayed
  connected but no new bytes arrived before the idle watchdog, so it closed the
  socket and will reconnect on the next tick.
- `stream_closed`: Webots closed the connection.
- `decode_error` or `invalid_header`: stream format does not match
  `WEBOTS_CAMERA_ENCODING`.

Fix Webots world/port/encoding first before tuning YOLO.

Run the camera-only probe:

```bash
python scripts/probe_webots_camera.py --host 127.0.0.1 --port 5599
```

Expected:

```text
camera frame ok source=tcp://127.0.0.1:5599 size=640x480 encoding=rgb8
```

If the probe succeeds but autonomy stays on `waiting_for_header`, check for a
second client or stale autonomy process holding the single Webots camera stream:

```bash
ss -tnp | grep 5599
```

Then stop old `python`, `drone-autonomy`, or probe processes before rerunning.
Also confirm Webots simulation is still running, not paused. If the machine is
slow, increase the idle watchdog without changing code:

```bash
WEBOTS_CAMERA_IDLE_RECONNECT=4.0 SEND_COMMANDS=0 bash scripts/run_iris_camera_yolo.sh
```

## YOLO is slow or autonomy loop feels stuck

`webots-yolo` runs camera reading and YOLO inference in background workers. The
mission loop should not block on either task. If the vehicle still appears stuck
in `seek_gate`, distinguish these cases:

- No `webots-yolo camera frame ready ...`: camera stream is still the issue.
- Camera is ready, but no centering: YOLO is not producing accepted gate
  detections.
- Detection appears and disappears: check class filter, confidence, gate
  visibility, and `WEBOTS_DETECTION_STALE`.

For a slow CPU/GPU, increase stale tolerance cautiously:

```bash
WEBOTS_DETECTION_STALE=1.5 \
MISSION_MAX_DETECTION_AGE=1.5 \
SEND_COMMANDS=0 \
bash scripts/run_iris_camera_yolo.sh
```

Do not use a very large stale value during motion tests; old detections can make
the drone center on where the gate used to be, not where it is now in the frame.

## Drone oscillates while centering on a gate

Current centering is intentionally conservative:

- target selection prefers the nearer/larger gate instead of highest confidence
  alone,
- centering velocity gains and max speeds are limited,
- centering commands are low-pass filtered through `VISUAL_COMMAND_FILTER_ALPHA`,
- brief detection loss in `CENTER_GATE` does not immediately return to scanning,
- runtime status prints `servo_err=(x,y)`, `area`, `aligned`, `clearance`, and
  `pass_ready`,
- `CENTER_GATE` must finish `MISSION_CENTER_DWELL` and clearance validation
  before `PASS_GATE`,
- `CENTER_GATE` also requires `MISSION_GATE_READY_AREA`, so a far centered gate
  cannot start the committed pass sequence.

Use the diagnostics window to confirm the selected target is stable:

```bash
WEBOTS_DIAGNOSTICS_WINDOW=1 SEND_COMMANDS=0 bash scripts/run_iris_camera_yolo.sh
```

If the selected box jumps between two gates, inspect the candidate scores in the
window before tuning control gains. If the selected box is stable but motion
still oscillates, reduce these first instead of increasing gains:

```bash
VISUAL_MAX_FORWARD_SPEED=0.20 \
VISUAL_MAX_LATERAL_SPEED=0.16 \
VISUAL_MAX_YAW_RATE=0.12 \
VISUAL_COMMAND_FILTER_ALPHA=0.18 \
WEBOTS_DIAGNOSTICS_WINDOW=1 \
SEND_COMMANDS=1 \
bash scripts/run_iris_camera_yolo.sh
```

If the selected box is stable but the mission never exits `CENTER_GATE`, inspect
the magenta pass-clearance overlay and the ready-area reference box. Tune these
values from env, not code:

```bash
VISUAL_PASS_TARGET_OFFSET_X=0.0 \
VISUAL_PASS_TARGET_OFFSET_Y=0.0 \
VISUAL_PASS_CLEARANCE_LEFT=0.08 \
VISUAL_PASS_CLEARANCE_RIGHT=0.08 \
VISUAL_PASS_CLEARANCE_UP=0.12 \
VISUAL_PASS_CLEARANCE_DOWN=0.12 \
MISSION_GATE_READY_AREA=0.055 \
WEBOTS_DIAGNOSTICS_WINDOW=1 \
SEND_COMMANDS=0 \
bash scripts/run_iris_camera_yolo.sh
```

If gate 2 is visible but still far away, the mission should print
`approaching gate 2 area=.../...` and keep moving forward. Tune these area
guards if it brakes too early or too late:

```bash
MISSION_NEXT_GATE_MIN_AREA=0.015 \
MISSION_GATE_READY_AREA=0.060 \
WEBOTS_DIAGNOSTICS_WINDOW=1 \
SEND_COMMANDS=0 \
bash scripts/run_iris_camera_yolo.sh
```

## Candidate is visible but rejected by validator

The OpenCV diagnostics window colors rejected candidates red and prints reason
labels:

- `confidence`: below `GATE_SELECTOR_MIN_SEEK_CONFIDENCE` or
  `GATE_SELECTOR_MIN_TRACK_CONFIDENCE`.
- `area_small` or `area_large`: bbox area ratio outside selector limits.
- `aspect`: bbox width/height outside selector limits.
- `roi`: candidate center outside the current validation ROI.
- `appearance`: candidate crop does not look enough like a hollow gate frame.
- `appearance_missing`: candidate did not carry image-backed appearance data.

Tune validator thresholds from the shell, not by editing code:

```bash
GATE_SELECTOR_MIN_SEEK_CONFIDENCE=0.35 \
GATE_SELECTOR_MIN_AREA_RATIO=0.001 \
GATE_SELECTOR_MIN_APPEARANCE_SCORE=0.08 \
WEBOTS_DIAGNOSTICS_WINDOW=1 \
SEND_COMMANDS=0 \
bash scripts/run_iris_camera_yolo.sh
```

If a false target is accepted, tighten the relevant threshold. If a real gate is
red, loosen only the threshold named by the rejection reason.

If the same camera stream works in the official ArduPilot example but not in a
copied or custom map, compare the `Iris` node first. A camera stream requires
both the controller args and a camera device with the exact same name. Opening
`iris.wbt` or copying only the vehicle without the camera `extensionSlot` gives
normal SITL physics but no camera TCP stream.

## Webots repeats `Connected to camera client` then `Camera client disconnected`

This means the autonomy process reached the Webots TCP camera server, but the
client did not keep the stream open long enough to assemble a full frame.

The current camera client keeps partial frame bytes across normal socket
timeouts. Make sure you are running the latest repo source through
`scripts/run_autonomy_sitl.sh`, not an old installed `drone-autonomy` command.
The launcher should prefer:

```text
python -m drone_autonomy.cli
```

Expected healthy Webots behavior is one camera client connection that stays
open while autonomy runs. A disconnect at process shutdown is normal.

## YOLO detections look worse than expected in Webots

This repo's `webots/worlds/iris_camera.wbt` should request `--camera-format
rgb24`, and `scripts/run_iris_camera_yolo.sh` should use
`WEBOTS_CAMERA_ENCODING=rgb24`. The diagnostics frame line should therefore
show:

```text
frame 640x480 rgb8
```

If it shows `rgb8_from_gray8`, you are still on the old grayscale path. Check
that Webots opened this repo's `webots/worlds/iris_camera.wbt`, that the Iris
controller args include `--camera-format rgb24`, and that local
`configs/autonomy_runtime.env` did not override `WEBOTS_CAMERA_ENCODING` back to
`gray8`.

If offline RGB video detects correctly but realtime Webots RGB still labels many
objects as `3:Goals-Detection`, compare the diagnostics `raw=...` line with the
overlay class labels. `raw=1:Dog` or `raw=2:Forklift` should be filtered out.
`raw=3:Goals-Detection` on a non-gate means model/domain quality, not selector
leakage.

## `scripts/run_sitl_webots.sh` says env file missing

Create it:

```bash
cp configs/sitl_webots.env.example configs/sitl_webots.env
nano configs/sitl_webots.env
```

Verify `ARDUPILOT_HOME` points to the actual ArduPilot checkout.

## Webots does not connect to SITL

Check:

- Webots world is the ArduPilot Webots Python example or a compatible custom world.
- Webots simulation is running before SITL starts.
- ArduPilot was launched with `--model webots-python`.
- The `iris.parm` file path in `configs/sitl_webots.env` is correct.

## Iris vehicle or parts are missing in Webots

Most likely the vendored Webots tree was replaced by a partial copy or Webots is
opening the wrong world path.

For baseline testing, open the world from this repo:

```text
<repo>/webots/worlds/iris.wbt
```

Make sure these files exist:

```text
webots/controllers/ardupilot_vehicle_controller/ardupilot_vehicle_controller.py
webots/params/iris.parm
webots/protos/Iris.proto
webots/protos/meshes/iris.dae
webots/worlds/iris.wbt
```

The safe config is:

```text
ARDUPILOT_HOME="/media/gedxxe/DATA/ardupilot"
WEBOTS_EXAMPLE_HOME="${REPO_ROOT}/webots"
WEBOTS_PARAM_FILE="params/iris.parm"
```

If the tree looks incomplete, re-sync from the official ArduPilot
`libraries/SITL/examples/Webots_Python` tree. See `docs/webots-source-sync.md`.

## The `webots/` folder timestamp did not change

Do not use the folder timestamp as the audit signal. A copy from another Git
checkout can preserve old file timestamps, and Git can also check out files with
timestamps that do not obviously reflect the latest repo operation.

Use these checks instead:

```bash
git ls-files webots | wc -l
```

Expected:

```text
36
```

For the official upstream compare command, see `docs/webots-source-sync.md`.

## Webots complains about `StraightRoadSegment`

The upstream ArduPilot example worlds reference Cyberbotics road PROTOs through
`EXTERNPROTO` URLs, for example:

```text
https://raw.githubusercontent.com/cyberbotics/webots/R2023a/projects/objects/road/protos/StraightRoadSegment.proto
```

This is not an Iris asset corruption. It means Webots needs internet access or a
cached copy of that external Cyberbotics PROTO. If this blocks startup in
Ubuntu, first confirm the machine can reach GitHub/raw.githubusercontent.com,
then reopen the world.

## Tests skip MAVLink adapter tests

The tests are skipped when `pymavlink` is not installed in the test environment.

Install dependencies:

```bash
pip install -e ".[dev]"
python -m pytest
```

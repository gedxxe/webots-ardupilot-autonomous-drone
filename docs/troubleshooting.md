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
version first. The script now falls back to running `python -m
drone_autonomy.cli` from `src/` when `drone-autonomy` is not in `PATH`.

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

## `No heartbeat from udp:127.0.0.1:14550`

Check these in order:

1. SITL is running.
2. SITL was launched with `--out=udp:127.0.0.1:14550`.
3. You are using the same endpoint in `drone-autonomy`.
4. No firewall or network namespace is blocking UDP.

Run:

```bash
drone-autonomy --mode listen --connection udp:127.0.0.1:14550 --count 5
```

If no messages print, the issue is before the autonomy code.

## Runtime prints `waiting for LOCAL_POSITION_NED telemetry`

The runtime received heartbeat but has not received local position.

Check:

- ArduPilot SITL has finished booting.
- Vehicle is not stuck before EKF/local-position initialization.
- Telemetry stream request was accepted.
- `LOCAL_POSITION_NED` appears in listen mode.

Inspect raw messages:

```bash
drone-autonomy --mode listen --connection udp:127.0.0.1:14550 --count 50
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
- `MAV_CMD_NAV_TAKEOFF` is accepted for the current vehicle/mode.

Current code does not yet parse `COMMAND_ACK`, so MAVProxy output is the best immediate clue.

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

## Gate pass or final exit distance feels wrong

Check `COURSE_FORWARD_X` and `COURSE_FORWARD_Y`.

The mission measures forward distance by projecting `LOCAL_POSITION_NED` onto the configured course direction. If course direction is wrong, gate pass distance and final forward exit distance are wrong.

## Synthetic detector completes too easily

That is expected. Synthetic detector is not perception. It returns centered boxes to test mission and MAVLink wiring only.

Use `--detector webots-yolo` with a trained gate model before evaluating gate behavior.

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
YOLO_MODEL_PATH="/media/gedxxe/DATA/models/gate_yolov8n.pt"
```

The repo does not include a trained gate model.

## `Ultralytics YOLO is required`

Install the optional vision dependencies in the active Python environment:

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
source .venv/bin/activate
pip install -e ".[dev,vision]"
```

If you use `venv-ardupilot`, activate that environment first and run the same
install command from this repo.

## `webots-yolo` stays in `seeking gate`

Check these in order:

1. Webots opened `webots/worlds/iris_camera.wbt`, not `iris.wbt`.
2. Webots console printed `Camera stream started at 127.0.0.1:5599`.
3. `WEBOTS_CAMERA_PORT="5599"` in `configs/autonomy_runtime.env`.
4. `YOLO_MODEL_PATH` points to a real `.pt` model.
5. The model class filter matches your training labels.

Default class filter:

```text
YOLO_GATE_CLASS_NAMES="gate"
YOLO_GATE_CLASS_IDS=""
```

If your gate class is id `0` but not named `gate`, use:

```text
YOLO_GATE_CLASS_NAMES=""
YOLO_GATE_CLASS_IDS="0"
```

Do not clear both filters during motion tests unless the model detects only
gates. Otherwise the mission may center on the wrong object.

If runtime prints:

```text
webots-yolo waiting for camera frame tcp://127.0.0.1:5599
```

the camera TCP stream is not connected yet. Fix Webots world/port first before
tuning YOLO.

## YOLO detections look worse than expected in Webots

The upstream ArduPilot `iris_camera.wbt` stream is grayscale. The adapter expands
gray frames to three channels before YOLO so the pipeline can run, but this is
not true RGB validation.

For final camera behavior, use a future RGB Webots stream or the real C920/OpenCV
source while keeping the same `YoloGateDetector -> GateDetection` contract.

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

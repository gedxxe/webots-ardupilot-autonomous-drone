# Simulation Runbook

Runbook ini fokus ke workflow yang dipakai sekarang: Webots `iris_camera.wbt`,
ArduPilot SITL, YOLO gate detector, dan OpenCV diagnostics window.

Untuk Raspberry Pi/Pixhawk hardware scaffold, gunakan
`docs/deployment-raspi.md`. Jangan campur `configs/raspi_runtime.env` dengan
tuning simulasi `iris_camera.wbt`.

## Terminal Layout

Gunakan empat terminal. Mission Planner opsional, tapi berguna untuk monitoring.

### Terminal 1: Mission Planner

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot/MissionPlanner
mono MissionPlanner.exe
```

Mission Planner memakai MAVLink `udp:127.0.0.1:14550`.

### Terminal 2: Webots

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
webots webots/worlds/iris_camera.wbt
```

Expected:

- Iris muncul di world.
- World memakai `Camera { name "camera" ... }`.
- Webots console menunjukkan camera stream di port `5599`.

### Terminal 3: ArduPilot SITL

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
source .venv/bin/activate
scripts/run_sitl_webots.sh
```

Expected:

- SITL boot ArduCopter.
- Webots connect ke SITL.
- MAVLink tersedia di `14550` untuk Mission Planner dan `14551` untuk autonomy.

### Terminal 4: YOLO Autonomy

Dry-run dulu. Diagnostics window default aktif untuk script ini, tapi env tetap
ditulis eksplisit supaya tidak ambigu.

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
source .venv/bin/activate
WEBOTS_DIAGNOSTICS_WINDOW=1 SEND_COMMANDS=0 bash scripts/run_iris_camera_yolo.sh
```

Kalau visual target sudah benar dan hanya di SITL:

```bash
WEBOTS_DIAGNOSTICS_WINDOW=1 SEND_COMMANDS=1 bash scripts/run_iris_camera_yolo.sh
```

## Required Setup

Install dependencies once:

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e ".[dev,vision]"
```

Create SITL config once:

```bash
cp configs/sitl_webots.env.example configs/sitl_webots.env
nano configs/sitl_webots.env
```

Minimum values:

```text
ARDUPILOT_HOME="/media/gedxxe/DATA/ardupilot"
WEBOTS_EXAMPLE_HOME="${REPO_ROOT}/webots"
WEBOTS_WORLD="worlds/iris_camera.wbt"
WEBOTS_PARAM_FILE="params/iris.parm"
MAVLINK_OUT="udp:127.0.0.1:14550"
MAVLINK_OUT_EXTRA="udp:127.0.0.1:14551"
```

Create local autonomy config only when you want persistent tuning:

```bash
cp configs/autonomy_runtime.env.example configs/autonomy_runtime.env
nano configs/autonomy_runtime.env
```

Do not tune `configs/autonomy_runtime.env.example` directly. It is the tracked
template.

## Runtime Precedence

Untuk autonomy runtime, urutan nilai yang dipakai adalah:

```text
inline env before command
-> iris-camera-yolo profile-owned defaults
-> configs/autonomy_runtime.env
-> Python runtime defaults in src/drone_autonomy/runtime/config.py
```

`scripts/run_iris_camera_yolo.sh` hanya memilih profile
`AUTONOMY_PROFILE="iris-camera-yolo"`. Profile ini mengunci hal yang memang
milik workflow iris-camera, yaitu `DETECTOR="webots-yolo"` dan diagnostics
default aktif. Nilai eksperimen seperti model path, class filter, area
threshold, clearance, speed, dan gain tetap diatur dari
`configs/autonomy_runtime.env` atau inline env.

Contoh inline env tetap menang:

```bash
YOLO_GATE_CLASS_IDS="" SEND_COMMANDS=0 bash scripts/run_iris_camera_yolo.sh
```

## Current YOLO Profile

`scripts/run_iris_camera_yolo.sh` selects the current profile:

```text
AUTONOMY_PROFILE="iris-camera-yolo"
DETECTOR="webots-yolo"                 # profile-owned unless set inline
WEBOTS_DIAGNOSTICS_WINDOW="1"          # profile-owned unless set inline
MAVLINK_CONNECTION="udp:127.0.0.1:14551"
MAVLINK_BAUD="115200"                  # ignored by UDP SITL
YOLO_MODEL_PATH="models/gate_yolov8n_best.pt"
YOLO_GATE_CLASS_NAMES="Goals-Detection"
YOLO_GATE_CLASS_IDS="3"
```

`MAVLINK_CONNECTION`, `YOLO_MODEL_PATH`, and the class filters are still normal
tuning/config values. Keep them in `configs/autonomy_runtime.env` unless you are
doing a one-shot inline test.

Verified current model metadata:

```text
0=AdvertisementBox
1=Dog
2=Forklift
3=Goals-Detection
4=Table
```

Dog/Forklift/Table must not become targets. If startup does not print this
filter, stop and fix config before enabling `SEND_COMMANDS=1`:

```text
webots-yolo class_filter names=Goals-Detection ids=3
```

## Main Tunables

Tune these in `configs/autonomy_runtime.env` or inline before the bash command.
For the technical meaning and safe tuning order, read `docs/tuning-guide.md`.

Detection and target selection:

```text
YOLO_CONFIDENCE
GATE_SELECTOR_MIN_SEEK_CONFIDENCE
GATE_SELECTOR_MIN_TRACK_CONFIDENCE
GATE_SELECTOR_MIN_AREA_RATIO
GATE_SELECTOR_STABLE_WINDOW
GATE_SELECTOR_REQUIRED_STABLE
WEBOTS_DETECTION_STALE
```

`GATE_SELECTOR_MIN_APPEARANCE_SCORE` and
`GATE_SELECTOR_APPEARANCE_WEIGHT` are optional guards, disabled by default. Do
not use them to hide a camera/config mismatch. First confirm diagnostics show
`frame 640x480 rgb8` and that the `raw=...` line says the model itself is
emitting `3:Goals-Detection` for the false object.

Gate pass and gate-2 acquire:

```text
MISSION_CENTER_DWELL
MISSION_CENTER_CLEARANCE_REQUIRED
MISSION_CENTER_LOST_GRACE_TICKS
MISSION_REQUIRED_DETECTION_TICKS
MISSION_MAX_DETECTION_AGE
MISSION_SEEK_YAW_RATE
MISSION_GATE_PASS_DISTANCE
MISSION_GATE_PASS_SPEED
MISSION_NEXT_GATE_ACQUIRE_SPEED
MISSION_NEXT_GATE_CLEAR_DISTANCE
MISSION_NEXT_GATE_MIN_AREA
MISSION_GATE_READY_AREA
MISSION_NEXT_GATE_MAX_DISTANCE
MISSION_NEXT_GATE_TIMEOUT
MISSION_BRAKE_SETTLE
MISSION_BRAKE_RAMP
MISSION_BRAKE_ALTITUDE_HOLD
MISSION_FINAL_EXIT_DISTANCE
MISSION_FINAL_EXIT_SPEED
```

Visual servo smoothness and clearance:

```text
VISUAL_FILTER_ALPHA
VISUAL_COMMAND_FILTER_ALPHA
VISUAL_CENTER_DEADBAND_X
VISUAL_CENTER_DEADBAND_Y
VISUAL_ALIGNED_ERROR_X
VISUAL_ALIGNED_ERROR_Y
VISUAL_PASS_TARGET_OFFSET_X
VISUAL_PASS_TARGET_OFFSET_Y
VISUAL_PASS_CLEARANCE_LEFT
VISUAL_PASS_CLEARANCE_RIGHT
VISUAL_PASS_CLEARANCE_UP
VISUAL_PASS_CLEARANCE_DOWN
VISUAL_MAX_ERROR_FOR_FORWARD
VISUAL_MAX_FORWARD_SPEED
VISUAL_LATERAL_KP
VISUAL_VERTICAL_KP
VISUAL_YAW_KP
VISUAL_MAX_LATERAL_SPEED
VISUAL_MAX_VERTICAL_SPEED
VISUAL_MAX_YAW_RATE
```

Example one-shot tuning:

```bash
MISSION_GATE_READY_AREA=0.050 \
MISSION_NEXT_GATE_MIN_AREA=0.012 \
WEBOTS_DIAGNOSTICS_WINDOW=1 \
SEND_COMMANDS=0 \
bash scripts/run_iris_camera_yolo.sh
```

## Diagnostics Checklist

Before `SEND_COMMANDS=1`, verify:

- Webots camera stream is alive.
- OpenCV diagnostics window appears.
- Frame line shows `rgb8`, not `rgb8_from_gray8`.
- Candidate labels show `cls=3:Goals-Detection`.
- Candidate labels show a plausible `g=...` hollow-gate appearance score for
  the real gate only if the optional appearance filter is enabled.
- Dog/Forklift/Table do not appear as selected targets.
- Selected gate bbox is stable before `CENTER_GATE`.
- Far/ready area boxes match the expected gate-2 acquire behavior.
- Clearance rectangle represents the safe pass center for the drone body.
- Console uses `udp:127.0.0.1:14551` for autonomy, not `14550`.

Optional camera-only probe:

```bash
python scripts/probe_webots_camera.py --host 127.0.0.1 --port 5599
```

Expected:

```text
camera frame ok source=tcp://127.0.0.1:5599 size=640x480 encoding=rgb8
```

## Expected Mission Behavior

With `SEND_COMMANDS=1` in SITL:

```text
takeoff -> seek gate 1 -> center dwell -> pass gate 1
-> clear forward/acquire gate 2 -> brake -> center dwell -> pass gate 2
-> final forward exit -> brake -> land
```

Takeoff is ArduPilot-managed through `MAV_CMD_NAV_TAKEOFF`. The companion code
does not run a body-z takeoff bootstrap controller.

During committed `PASS_GATE`, the command is forward-only body velocity plus
altitude hold. Lateral/yaw visual corrections are not kept active while crossing
the gate.

## Troubleshooting Pointers

- No camera frame: check Webots world, `--camera camera`, and port `5599`.
- Mission Planner disconnects: keep Mission Planner on `14550`; autonomy uses
  `14551`.
- Wrong class target: check startup `webots-yolo class_filter ...`, the OpenCV
  `raw=...` line, and the `cls=id:name` labels. Non-gate raw classes should be
  filtered out; `raw=3:Goals-Detection` on a non-gate is a model/input issue.
- Gate 2 easy to lose: tune `MISSION_NEXT_GATE_MIN_AREA`,
  `MISSION_GATE_READY_AREA`, selector stability, and detection stale time after
  class filtering is verified.
- Drone moves in dry-run: it should not. Stop and confirm `SEND_COMMANDS=0`.

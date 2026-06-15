# Simulation Runbook

Runbook ini menjelaskan cara menjalankan kode simulasi dari nol sampai autonomy loop tersambung ke ArduPilot SITL.

Status saat ini:

- Webots baseline world sudah tersedia dari vendored ArduPilot Webots example.
- YOLO model belum ada.
- Camera adapter belum ada.
- Runtime tetap bisa diuji dengan ArduPilot Webots example dan synthetic gate detector.

Baseline rule:

- This repo now contains a full vendored copy of ArduPilot's Webots Python
  example in `webots/`.
- For baseline testing, open `webots/worlds/iris.wbt` from this repo.
- Do not replace it with partial copies. Partial copies can make the Iris
  vehicle disappear because `.wbt`, `.proto`, meshes, textures, controllers,
  scripts, and params reference each other.

Synthetic detector hanya untuk membuktikan wiring `MAVLink -> telemetry -> mission -> command`. Jangan pakai synthetic detector untuk menilai performa gate detection.

## Mental Model

Ada tiga proses yang berjalan terpisah:

```text
Terminal A: Webots GUI
Terminal B: ArduPilot SITL
Terminal C: Python autonomy runtime
```

Aliran datanya:

```text
Webots physics
  -> ArduPilot SITL
  -> MAVLink UDP 127.0.0.1:14550
  -> drone-autonomy runtime
  -> optional MAVLink guided commands back to ArduPilot
```

## Before You Start

Pastikan:

- Ubuntu 24.04 environment tersedia untuk Webots dan ArduPilot SITL.
- Webots bisa dibuka.
- ArduPilot sudah cloned di luar repo ini, default `~/ardupilot`.
- ArduPilot submodules sudah initialized.
- `./waf configure --board sitl` dan `./waf copter` sudah sukses.
- Python environment repo ini sudah install dependencies.
- `configs/sitl_webots.env` sudah dibuat dari example.

Expected folder layout when using your DATA partition:

```text
/media/gedxxe/DATA/ardupilot
/media/gedxxe/DATA/venv-ardupilot
/media/gedxxe/DATA/WeBots_Ardupilot
```

This repo does not need to live inside the ArduPilot checkout. ArduPilot stays
external, while this repo owns the companion autonomy code and vendored Webots
baseline assets.

Install repo dependencies:

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e ".[dev]"
```

Create simulator env:

```bash
cp configs/sitl_webots.env.example configs/sitl_webots.env
nano configs/sitl_webots.env
```

Minimal isi yang harus benar:

```text
ARDUPILOT_HOME="<absolute path to your ArduPilot checkout>"
WEBOTS_EXAMPLE_HOME="${REPO_ROOT}/webots"
MAVLINK_OUT="udp:127.0.0.1:14550"
```

Leave `WEBOTS_EXAMPLE_RELATIVE` commented unless you intentionally want to use
the Webots example directly from the ArduPilot checkout instead of this repo's
vendored `webots/` tree.

Examples:

```text
# Native Ubuntu home install
ARDUPILOT_HOME="$HOME/ardupilot"

# DATA partition mounted by Ubuntu
ARDUPILOT_HOME="/media/gedxxe/DATA/ardupilot"

# WSL-style mount
ARDUPILOT_HOME="/mnt/d/ardupilot"
```

Do not point `WEBOTS_EXAMPLE_HOME` to a half-copied Webots folder. The default
below is the safest baseline because this repo already vendors the complete
ArduPilot example:

```text
WEBOTS_EXAMPLE_HOME="${REPO_ROOT}/webots"
WEBOTS_WORLD="worlds/iris.wbt"
WEBOTS_PARAM_FILE="params/iris.parm"
```

## Step 1: Open Webots

Terminal A:

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
webots webots/worlds/iris.wbt
```

Alternative if you prefer opening from the Webots GUI:

```text
<repo>/webots/worlds/iris.wbt
```

Press Run in Webots.

Expected:

- Webots GUI terbuka.
- World `iris.wbt` loaded.
- Iris model appears in the world.
- Simulation is running.

## Step 2: Start ArduPilot SITL

Terminal B:

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
source .venv/bin/activate
scripts/run_sitl_webots.sh
```

If the DATA partition is mounted with `noexec`, run the script through Bash:

```bash
bash scripts/run_sitl_webots.sh
```

Expected:

- MAVProxy starts.
- SITL boots ArduCopter.
- Webots console eventually reports connection to ArduPilot SITL.
- MAVLink output is available on `udp:127.0.0.1:14550`.

Do not continue until SITL is alive.

## Step 3: Check MAVLink Heartbeat

Terminal C:

```bash
cd /media/gedxxe/DATA/WeBots_Ardupilot
source .venv/bin/activate
drone-autonomy --mode heartbeat --connection udp:127.0.0.1:14550
```

Expected output shape:

```text
heartbeat system=... component=... type=... autopilot=...
```

If this fails, fix MAVLink connection before testing autonomy.

## Step 4: Dry-Run Autonomy

Dry-run reads telemetry and runs mission decisions, but sends no movement command.

Terminal C:

```bash
drone-autonomy \
  --mode autonomy \
  --connection udp:127.0.0.1:14550 \
  --detector synthetic
```

Expected output shape:

```text
autonomy connection=udp:127.0.0.1:14550 detector=synthetic command_mode=dry-run loop_hz=20.0
dry-run phase=init gate=1 cmd=set_mode detail=waiting for guided mode
```

Important:

- Dry-run is for wiring visibility.
- It will not arm, take off, or move the vehicle.
- If SITL is still not in `GUIDED` and armed, the mission will stay around `SET_MODE` or `ARM` decisions because commands are not actually sent.

## Step 5: Commanded Synthetic Motion Test

Only run this in SITL. Do not run this on real hardware.

Terminal C:

```bash
drone-autonomy \
  --mode autonomy \
  --connection udp:127.0.0.1:14550 \
  --detector synthetic \
  --send-commands
```

Expected phase progression:

```text
sent phase=init ... cmd=set_mode
sent phase=init ... cmd=arm
sent phase=takeoff ... cmd=takeoff
sent phase=seek_gate ... cmd=body_velocity
sent phase=center_gate ... cmd=body_velocity
sent phase=pass_gate ... cmd=body_velocity
sent phase=next_gate_acquire ... cmd=body_velocity
sent phase=brake ... cmd=body_velocity
sent phase=center_gate ... cmd=body_velocity
sent phase=pass_gate ... cmd=body_velocity
sent phase=final_exit ... cmd=body_velocity
sent phase=land ... cmd=land
```

The exact timing depends on SITL position updates and vehicle response.

## Step 6: Script-Based Run

Create runtime config:

```bash
cp configs/autonomy_runtime.env.example configs/autonomy_runtime.env
nano configs/autonomy_runtime.env
```

Dry-run:

```bash
scripts/run_autonomy_sitl.sh
```

The script prefers the installed `drone-autonomy` command. If that command is
not in `PATH`, it falls back to a Python module launch from `src/`. Make sure the
selected Python environment still has `pymavlink` installed through:

```bash
pip install -e ".[dev]"
```

If you reuse the ArduPilot virtualenv instead of this repo's `.venv`, set this
in `configs/autonomy_runtime.env`:

```text
PYTHON_BIN="/media/gedxxe/DATA/venv-ardupilot/bin/python"
```

Commanded SITL motion:

```bash
SEND_COMMANDS=1 scripts/run_autonomy_sitl.sh
```

Or edit:

```text
SEND_COMMANDS="1"
```

inside `configs/autonomy_runtime.env`.

## Course Direction

`forward_position_m` is computed from ArduPilot `LOCAL_POSITION_NED`.

Default:

```text
COURSE_FORWARD_X=1.0
COURSE_FORWARD_Y=0.0
```

This means course-forward is local NED x/North.

If the Webots gate line points along local y/East:

```text
COURSE_FORWARD_X=0.0
COURSE_FORWARD_Y=1.0
```

When the custom Webots world exists, this must be verified before trusting gate pass distance, adaptive acquire distance, or final forward exit distance.

## Runtime Modes

`drone-autonomy --mode heartbeat`

- Blocks until heartbeat is received.
- Use this first to validate MAVLink endpoint.

`drone-autonomy --mode listen`

- Prints raw MAVLink messages.
- Useful to confirm `LOCAL_POSITION_NED` is being received.

`drone-autonomy --mode autonomy --detector synthetic`

- Runs mission logic with synthetic gate detections.
- Good for SITL wiring before YOLO exists.

`--send-commands`

- Required before runtime sends MAVLink commands.
- Omit this flag for dry-run.

## What This Does Not Test Yet

- Real YOLO gate detection.
- Webots camera frame capture.
- Real gate geometry or lighting.
- Real obstacle avoidance.
- Hardware safety behavior.

Those come after the custom Webots world and detector adapter exist.

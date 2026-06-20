# Webots ArduPilot Autonomy Sandbox

Repository ini disiapkan sebagai workspace simulasi autonomous drone dengan Webots, ArduPilot SITL, dan Python companion application.

ArduPilot sebaiknya tetap berada di luar repo ini, misalnya di `~/ardupilot`. Repo ini hanya menyimpan kode autonomy, konfigurasi simulasi, skrip launch, dokumentasi, dan aset Webots custom.

## Architecture

```text
Webots
  -> ArduPilot SITL
  -> MAVLink UDP
  -> Python companion application
  -> gate perception
  -> visual centering + mission state machine
```

Current simulation perception options are:

- `synthetic`: fake centered detections for wiring tests.
- `webots-yolo`: `iris_camera.wbt` TCP camera stream, YOLO raw candidates, and
  `GateTargetSelector` validation/tracking before `GateDetection`. The repo
  includes the trained gate model at `models/gate_yolov8n_best.pt`.

This repo's `iris_camera.wbt` requests `rgb24` from the vendored Webots
controller so simulation inference is closer to normal RGB video. If diagnostics
show `rgb8_from_gray8`, the run is still using the old grayscale path.

## Repository Layout

```text
.
+-- AGENTS.md                # Notes for future AI agents and maintainers
+-- configs/                 # Environment and simulator configuration
+-- docs/                    # Setup notes, strategy, and contracts
+-- models/                  # Trained local perception model artifacts
+-- scripts/                 # Launch and smoke-test scripts
+-- src/drone_autonomy/      # Python companion/autonomy package
+-- tests/                   # Unit tests for local autonomy code
+-- webots/                  # Vendored ArduPilot Webots_Python baseline assets
```

## Quick Start

Ubuntu 24.04 is the intended runtime for Webots and ArduPilot SITL.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e ".[dev]"
```

Copy and adjust the simulator environment file:

```bash
cp configs/sitl_webots.env.example configs/sitl_webots.env
nano configs/sitl_webots.env
```

Launch Webots first, then start ArduPilot SITL:

```bash
scripts/run_sitl_webots.sh
```

Check MAVLink telemetry from the companion app:

```bash
drone-autonomy --connection udp:127.0.0.1:14551 --mode heartbeat
```

Run the iris-camera YOLO profile in dry-run mode:

```bash
WEBOTS_DIAGNOSTICS_WINDOW=1 SEND_COMMANDS=0 bash scripts/run_iris_camera_yolo.sh
```

Use `SEND_COMMANDS=1` only after Webots, SITL, class filtering, and the OpenCV
diagnostics overlay show the correct gate target. Synthetic detection still
exists for plumbing tests, but it is not the default workflow.

## Development Stages

1. Done: validate Webots plus ArduPilot SITL baseline assets.
2. Done: validate Python MAVLink heartbeat/listen/autonomy wiring.
3. Done: translate `VehicleCommand` outputs into MAVLink guided commands.
4. Done: add Webots TCP camera plus YOLO-to-`GateDetection` pipeline.
5. Done: add the trained YOLOv8n gate model for `iris_camera.wbt` tests.
6. In progress: validate/tune gate target selection, centering, adaptive
   acquire, final exit, and landing in simulation.
7. Later: prepare hardware-facing companion deployment.

## Current Autonomy Scope

The repository now contains simulator/hardware-neutral mission logic for a two-gate task:

```text
takeoff -> seek gate 1 -> center dwell + clearance -> pass
        -> clear forward -> acquire gate 2 -> brake
        -> center dwell + clearance -> pass -> forward exit 2 m -> brake -> land

fallback: adaptive acquire gate 2 -> slow seek gate 2 if detection times out
```

Gate pass tuning is exposed through `configs/autonomy_runtime.env`:
`MISSION_CENTER_DWELL`, `VISUAL_PASS_CLEARANCE_*`,
`MISSION_GATE_READY_AREA`, `MISSION_GATE_PASS_DISTANCE`, and
`MISSION_NEXT_GATE_CLEAR_DISTANCE`.

The `webots/` directory contains a full vendored copy of ArduPilot's Webots
Python example. Use it for baseline SITL tests, then add custom gate worlds
after the baseline works.

Read these before running motion tests:

- [docs/project-status.md](docs/project-status.md): implementation status and anti-hallucination ground truth.
- [docs/run-simulation.md](docs/run-simulation.md): exact step-by-step runbook.
- [docs/tuning-guide.md](docs/tuning-guide.md): technical tuning reference for camera geometry, YOLO inference size, OpenCV boxes, gate acquire, and visual servo gains.
- [docs/webots-yolo-pipeline.md](docs/webots-yolo-pipeline.md): Webots camera stream to YOLO to `GateDetection`.
- [docs/main-logic-map.md](docs/main-logic-map.md): map of runtime, mission, MAVLink, and detector code.
- [docs/webots-source-sync.md](docs/webots-source-sync.md): how the vendored Webots tree is sourced and refreshed.
- [docs/troubleshooting.md](docs/troubleshooting.md): common failures and fixes.
- [docs/simulation-audit.md](docs/simulation-audit.md): readiness and known risk list.

## Ownership Rule

Do not modify ArduPilot source unless there is a clear simulator integration requirement. Prefer local config files, parameter overrides, launch scripts, and companion-app code in this repository.

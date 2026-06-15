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
- `webots-yolo`: `iris_camera.wbt` TCP camera stream plus YOLO-to-`GateDetection`.

The upstream ArduPilot `iris_camera.wbt` stream is grayscale. It is expanded to
three channels for YOLO in simulation; true RGB behavior remains a future
C920/OpenCV or RGB Webots adapter concern.

## Repository Layout

```text
.
+-- AGENTS.md                # Notes for future AI agents and maintainers
+-- configs/                 # Environment and simulator configuration
+-- docs/                    # Setup notes, strategy, and contracts
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
drone-autonomy --connection udp:127.0.0.1:14550 --mode heartbeat
```

Run the autonomy wiring in dry-run mode with synthetic gate detections:

```bash
drone-autonomy --mode autonomy --connection udp:127.0.0.1:14550 --detector synthetic
```

Send commands only when connected to SITL:

```bash
drone-autonomy --mode autonomy --connection udp:127.0.0.1:14550 --detector synthetic --send-commands
```

## Development Stages

1. Done: validate Webots plus ArduPilot SITL baseline assets.
2. Done: validate Python MAVLink heartbeat/listen/autonomy wiring.
3. Done: translate `VehicleCommand` outputs into MAVLink guided commands.
4. Done: add Webots TCP camera plus YOLO-to-`GateDetection` pipeline.
5. Next: provide/train the gate model and add the custom two-gate Webots world.
6. Next: tune centering, adaptive acquire, final exit, and landing in simulation.
7. Later: prepare hardware-facing companion deployment.

## Current Autonomy Scope

The repository now contains simulator/hardware-neutral mission logic for a two-gate task:

```text
takeoff -> seek gate 1 -> center -> pass -> adaptive acquire gate 2
        -> center gate 2 -> pass -> forward exit 2 m -> land

fallback: adaptive acquire gate 2 -> slow seek gate 2 if detection times out
```

The `webots/` directory contains a full vendored copy of ArduPilot's Webots
Python example. Use it for baseline SITL tests, then add custom gate worlds
after the baseline works.

Read these before running motion tests:

- [docs/project-status.md](docs/project-status.md): implementation status and anti-hallucination ground truth.
- [docs/run-simulation.md](docs/run-simulation.md): exact step-by-step runbook.
- [docs/webots-yolo-pipeline.md](docs/webots-yolo-pipeline.md): Webots camera stream to YOLO to `GateDetection`.
- [docs/main-logic-map.md](docs/main-logic-map.md): map of runtime, mission, MAVLink, and detector code.
- [docs/webots-source-sync.md](docs/webots-source-sync.md): how the vendored Webots tree is sourced and refreshed.
- [docs/troubleshooting.md](docs/troubleshooting.md): common failures and fixes.
- [docs/simulation-audit.md](docs/simulation-audit.md): readiness and known risk list.

## Ownership Rule

Do not modify ArduPilot source unless there is a clear simulator integration requirement. Prefer local config files, parameter overrides, launch scripts, and companion-app code in this repository.

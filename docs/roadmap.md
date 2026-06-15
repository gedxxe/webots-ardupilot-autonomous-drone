# Development Roadmap

## Stage 1: Simulator Baseline

- Webots launches and opens the Iris world.
- ArduPilot SITL connects to Webots.
- MAVProxy can arm, take off, and land the vehicle.
- MAVLink output is available on UDP port `14550`.

## Stage 2: Companion Telemetry

- Python companion app receives heartbeat.
- Telemetry loop can read attitude, position, mode, GPS, and battery messages.
- Connection settings are centralized in config/env files.

## Stage 3: Basic Autonomy Commands

- Guided takeoff command.
- Land command.
- Velocity/body-frame command helpers.
- Command acknowledgements are logged and validated.

## Stage 4: Navigation Logic

- Mission state machine.
- Waypoint or local-position navigation.
- Failsafe behavior for heartbeat loss, mode mismatch, and command timeout.

## Stage 5: Sensors and Perception

- Add Webots obstacle scenarios.
- Add simulated range/camera feeds.
- Integrate perception output into navigation constraints.

## Stage 6: Hardware Readiness

- Separate simulator-only adapters from hardware-facing adapters.
- Document Raspberry Pi deployment.
- Preserve the same high-level autonomy API across SITL and hardware.

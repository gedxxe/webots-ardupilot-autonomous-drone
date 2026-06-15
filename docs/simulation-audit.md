# Simulation Readiness Audit

## Pre-Run Checklist

Before any autonomy command test:

- Webots is installed and can open the target world.
- ArduPilot checkout exists outside this repo.
- ArduPilot SITL has been built for copter.
- `configs/sitl_webots.env` points to the correct ArduPilot checkout.
- Python virtual environment is active.
- `pip install -e ".[dev]"` has completed.
- `drone-autonomy --mode heartbeat --connection udp:127.0.0.1:14550` succeeds.
- `LOCAL_POSITION_NED` appears in MAVLink listen output.

Before `--send-commands`:

- You are connected to SITL, not hardware.
- Webots simulation is running.
- Course direction `COURSE_FORWARD_X/Y` is understood or intentionally left at default for smoke testing.
- You are ready to stop SITL/Webots if the vehicle moves incorrectly.

## Ready in Code

- Mission state machine for two gates.
- Adaptive next-gate acquire instead of blind sprint.
- Brake-before-center after the next gate is detected during adaptive acquire.
- Altitude-hold velocity bias from fused `LOCAL_POSITION_NED` altitude.
- MAVLink telemetry adapter for heartbeat, arm state, landed state, altitude, and forward position.
- MAVLink command adapter for mode, arm, takeoff, land, and body-frame velocity.
- Runtime loop that can run dry or send commands.
- Synthetic gate detector for SITL wiring tests before YOLO/Webots camera integration.

## Still Required Outside Code

- Custom Webots world.
- Real or trained YOLOv8n gate model.
- Camera adapter from Webots frames to the detector.
- ArduPilot SITL running and publishing MAVLink on the configured UDP endpoint.
- Correct Webots course alignment relative to `LOCAL_POSITION_NED`.

## Not Yet Implemented in Code

- `COMMAND_ACK` parsing and retry policy.
- Lost-heartbeat failsafe in the runtime loop.
- Webots camera adapter.
- YOLOv8n detector wrapper.
- User-editable mission tuning file.
- Automatic course-frame calibration.
- Hardware launch profile.

## High-Risk Items to Verify in SITL

- Body-frame velocity signs: forward, right, down, and yaw-rate.
- `COURSE_FORWARD_X/Y` projection matches the actual gate-line direction.
- `LOCAL_POSITION_NED.z` behaves as expected with Webots.
- ArduPilot accepts `MAV_CMD_NAV_TAKEOFF` and body velocity commands in GUIDED mode.
- Gate pass distance and final forward exit distance are realistic for the world scale.
- Synthetic detector should be replaced by real detector before judging gate behavior.

## Known Design Choices

- Sensor fusion belongs to ArduPilot EKF or simulator-equivalent fused telemetry, not raw Python blending.
- RGB detection controls alignment, not metric distance.
- Bounding-box area is only a rough pass-readiness signal until gate dimensions/camera calibration are known.
- Runtime defaults to dry-run so accidental hardware motion does not happen.

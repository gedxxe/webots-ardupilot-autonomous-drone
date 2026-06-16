# Sensor Fusion and Altitude Policy

The real hardware target includes Pixhawk 6C Mini, Raspberry Pi 5, Logitech C920 Pro, GPS, rangefinder, and MTF01 optical flow. The simulator should represent the same information flow before hardware tests.

## Recommended Policy

Use ArduPilot EKF as the primary sensor-fusion owner. The companion app should consume fused telemetry and issue high-level/body-frame commands.

Do not raw-blend GPS, rangefinder, and optical-flow data in the Python mission layer unless there is a separately designed estimator with validation tests.

For TAKEOFF specifically, prefer ArduPilot's guided takeoff controller over a
companion-side vertical velocity loop. The mission should send the `1.0 m`
takeoff target, then wait for fused telemetry to settle.

## Why

- ArduPilot already owns attitude, altitude, local position, failsafe, and guided-mode control loops.
- GPS is weak for low-altitude gate racing and indoor/near-obstacle work.
- Rangefinder is strong for low-altitude height above ground but can fail over bad surfaces or outside range.
- Optical flow is useful for local motion when texture and range data are good.
- RGB gate detection gives alignment, not reliable metric distance without calibration.

The MicoAir MTF-01 is a two-in-one optical-flow plus short-range lidar sensor.
When configured correctly in ArduPilot, the rangefinder/flow data should improve
the autopilot's own altitude and local-position control. The companion should
verify that telemetry/logs expose valid range/flow data, but it should not fuse
those raw samples inside `GateAutonomyMission`.

Reference docs:

- ArduPilot rangefinder overview: `https://ardupilot.org/copter/docs/common-rangefinder-landingpage.html`
- ArduPilot MicoAir MTF-01 setup: `https://ardupilot.org/copter/docs/common-mtf-01.html`
- ArduPilot Guided mode: `https://ardupilot.org/copter/docs/ac2_guidedmode.html`

## Companion-App Contract

`MissionTelemetry.altitude_m` should represent fused altitude above the local takeoff/landing reference.

`MissionTelemetry.forward_position_m` should represent fused forward travel along the course. The final `2.0 m` exit is measured from this field, not from altitude.

## Simulation Contract

Before hardware, Webots should provide equivalent signals through ArduPilot SITL or an adapter:

- RGB camera frames for gate detection.
- Local altitude estimate.
- Local forward position or velocity integration.
- Optional rangefinder and optical-flow sensors if the Webots world supports them.

The mission code should not need to know whether telemetry came from Webots or hardware.

## Practical Hardware Direction

For the real drone, prefer this split:

- Pixhawk/ArduPilot: EKF fusion, altitude hold, attitude stabilization, mode/failsafe enforcement.
- Raspberry Pi companion: YOLO inference, mission state machine, visual centering commands.
- MAVLink adapter: convert `VehicleCommand` into ArduPilot guided commands and read fused telemetry back.

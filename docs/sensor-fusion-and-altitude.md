# Sensor Fusion and Altitude Policy

The real hardware target includes Pixhawk 6C Mini, Raspberry Pi 5, Logitech C920 Pro, GPS, rangefinder, and MTF01 optical flow. The simulator should represent the same information flow before hardware tests.

## Recommended Policy

Use ArduPilot EKF as the primary sensor-fusion owner. The companion app should consume fused telemetry and issue high-level/body-frame commands.

Do not raw-blend GPS, rangefinder, and optical-flow data in the Python mission layer unless there is a separately designed estimator with validation tests.

## Why

- ArduPilot already owns attitude, altitude, local position, failsafe, and guided-mode control loops.
- GPS is weak for low-altitude gate racing and indoor/near-obstacle work.
- Rangefinder is strong for low-altitude height above ground but can fail over bad surfaces or outside range.
- Optical flow is useful for local motion when texture and range data are good.
- RGB gate detection gives alignment, not reliable metric distance without calibration.

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

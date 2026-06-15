# Two-Gate Autonomy Strategy

This document describes the autonomy strategy for the Webots/ArduPilot sandbox without depending on a specific Webots world or camera implementation.

## Mission

The competition-style task is:

1. Take off to `1.0 m`.
2. Search for a hollow gate in front of the drone.
3. Center on the gate using RGB camera detections.
4. Approach and pass through the gate.
5. Move forward while actively acquiring the second gate.
6. Fall back to slow seek if the second gate is not found within distance/time limits.
7. Center and pass through the second gate.
8. Continue forward about `2.0 m` after the second gate.
9. Land.

## Core Design

The repository separates the autonomy stack into four layers:

```text
perception -> control -> mission -> vehicle adapter
```

- `perception` converts camera frames into gate detections.
- `control` converts detection error into body-frame velocity commands.
- `mission` owns the state machine and decides what behavior is active.
- `vehicle adapter` sends commands to ArduPilot through MAVLink.

The current code implements `perception`, `control`, and `mission` contracts. The MAVLink command adapter is intentionally separate and can be added after SITL telemetry is validated.

`GateAutonomyMission.update()` is non-blocking. It expects the runtime loop to provide the latest telemetry and gate detection, and it returns exactly one abstract command for that tick.

## Centering Control

The centering controller uses image-based visual servoing. A YOLO detector later provides the gate bounding box. The controller computes:

- normalized horizontal error from image center,
- normalized vertical error from image center,
- normalized bounding-box area as a rough closeness signal.

The command output uses body-frame velocity:

- Move right if the gate appears to the right.
- Move down if the gate appears below center.
- Yaw toward the gate to keep the camera aligned.
- Slow forward speed when the gate is not centered.

This is deliberately conservative. It should prefer a clean gate pass over maximum speed while still allowing a faster adaptive acquire phase between gate 1 and gate 2.

## Adaptive Next-Gate Acquire

After passing gate 1, the mission no longer performs a blind fixed-distance sprint. It moves forward at a capped acquire speed while the detector keeps searching for gate 2.

If gate 2 is detected for the required number of ticks, the mission brakes briefly before centering. If gate 2 is not detected before the configured maximum forward distance or timeout, the mission falls back to slow seek for the next gate.

## Altitude Control

The mission maintains altitude during search, pass, adaptive acquire, brake, and final forward exit by adding a small vertical velocity correction toward the takeoff altitude. During visual centering, vertical image correction is allowed but bounded by altitude guards.

The altitude input should be fused local telemetry from ArduPilot or a simulator adapter. The mission must not consume raw GPS, raw rangefinder, or raw optical-flow samples directly.

## Robustness Rules

- A single detection must not trigger a phase change unless config allows it.
- A gate pass requires both alignment and an apparent-size threshold.
- Loss of detection during centering returns the mission to search instead of continuing blindly.
- The inter-gate phase is not blind: gate detections can interrupt forward acquire and trigger centering immediately.
- Gate 2 acquisition includes a short brake-before-center phase to reduce overshoot risk.
- Centering commands are altitude-guarded so visual servoing cannot keep descending below the configured floor.
- Gate clearance and final exit use forward distance from telemetry.
- Landing is only commanded after the final forward exit distance is reached.

## Future YOLOv8n Integration

YOLOv8n should be wrapped behind the `GateDetector` protocol. The detector should return a `GateDetection`, not call ArduPilot or mutate mission state.

Keep detector-specific code in `src/drone_autonomy/perception/`. Keep model files outside source control unless the user explicitly wants them versioned.

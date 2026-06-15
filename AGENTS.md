# Agent Notes

These notes are for AI agents and future maintainers working on this repository.

## Project Intent

Build a simple but robust autonomous drone pipeline for a two-gate task:

1. Take off to about 1 meter.
2. Search for the first hollow gate using a normal RGB camera.
3. Center on the gate while approaching.
4. Pass through the first gate.
5. Move forward while actively acquiring the second gate.
6. Center and pass through the second gate.
7. Fly forward about 2 meters after the last gate.
8. Land.

The same high-level code should work in Webots/ArduPilot SITL and later on a real Pixhawk 6C Mini, Raspberry Pi 5, Logitech C920 Pro, and hexacopter frame.

## Non-Negotiable Boundaries

- Keep `webots/` empty except placeholders until the user adds worlds and assets.
- Do not modify ArduPilot source by default.
- Keep hardware, simulator, perception, control, and mission logic separated.
- Do not hard-wire YOLO, OpenCV, Webots, or MAVLink details into the mission state machine.
- Use body-frame velocity commands as the mission output contract.
- Prefer deterministic state machines over hidden loops with global state.
- Every control sign convention must be documented before it is used.

## Control Frame

Internal velocity commands use ArduPilot-friendly body axes:

- `body_vx_m_s`: positive forward.
- `body_vy_m_s`: positive right.
- `body_vz_m_s`: positive down.
- `yaw_rate_rad_s`: positive right/clockwise from the vehicle perspective.

Adapters must translate this contract correctly when sending MAVLink commands.

## Perception Contract

The gate detector should return one `GateDetection` per frame:

- Bounding box around the hollow gate structure.
- Confidence score.
- Observation timestamp.
- Optional track id.

The detector should not decide flight phases. It only reports perception.

## Current Strategy

The first implementation uses image-based visual servoing:

- Horizontal image error drives right/left body velocity and yaw rate.
- Vertical image error drives climb/descent through body z velocity.
- Forward speed is reduced when centering error is large.
- Alignment requires consecutive stable ticks before the mission commits to passing a gate.
- After gate 1, do not blind sprint. Use adaptive next-gate acquire: move forward while the detector keeps looking for gate 2.
- Distance after gates uses local forward position from telemetry, not frame count.
- Final exit distance is forward travel after the last gate, not altitude.
- Altitude correction uses fused telemetry from ArduPilot/simulator adapters; do not raw-blend GPS, rangefinder, and optical-flow samples in mission code.

## Anti-Hallucination Rules

- If a module is a placeholder, say it is a placeholder in code or docs.
- If telemetry is required but not wired yet, model it explicitly as input.
- If camera calibration is unknown, avoid pretending to know metric distance from RGB alone.
- If sensor fusion is required, document whether it is ArduPilot EKF fusion or a local estimator before adding code.
- If Webots worlds are absent, do not invent world file names.
- If real-hardware behavior is discussed, mark it as a future adapter unless implemented.

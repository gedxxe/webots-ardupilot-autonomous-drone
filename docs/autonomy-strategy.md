# Two-Gate Autonomy Strategy

This document describes the autonomy strategy for the Webots/ArduPilot sandbox without depending on a specific Webots world or camera implementation.

## Mission

The competition-style task is:

1. Take off to `1.0 m`.
2. Search for a hollow gate in front of the drone.
3. Center on the gate using RGB camera detections.
4. Dwell while centering, validate image-space clearance, then pass through the gate.
5. Move forward far enough to clear the first obstacle before acquiring the second gate.
6. Fall back to slow seek if the second gate is not found within distance/time limits.
7. Dwell, validate clearance, and pass through the second gate.
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

The current code implements `perception`, `control`, `mission`, and MAVLink
adapter contracts. Runtime wiring decides whether commands are dry-run only or
sent to ArduPilot.

`GateAutonomyMission.update()` is non-blocking. It expects the runtime loop to provide the latest telemetry and gate detection, and it returns exactly one abstract command for that tick.

## Centering Control

The centering controller uses image-based visual servoing. A detector provides
the gate bounding box. The controller computes:

- normalized horizontal error from image center,
- normalized vertical error from image center,
- normalized bounding-box area as an image-space readiness guard and diagnostic,
  not as metric distance.

The command output uses body-frame velocity:

- Move right if the gate appears to the right.
- Move down if the gate appears below center.
- Apply only small yaw correction during centering; aggressive yaw shakes the
  rigid Webots camera and destabilizes the detector.
- Use slow forward creep during centering. The actual gate crossing is the
  forward-only `PASS_GATE` phase.
- Low-pass filter both image errors and commanded centering velocities.

This is deliberately conservative. It should prefer a clean gate pass over maximum speed while still allowing a faster adaptive acquire phase between gate 1 and gate 2.

## Gate Pass Commit

The mission no longer commits to a pass only because one alignment tick looks
good. The current pass rule is:

```text
stable gate selected
-> CENTER_GATE visual servo continues
-> dwell for MISSION_CENTER_DWELL seconds
-> clearance validator stays true for MISSION_CENTER_CLEARANCE_REQUIRED seconds
-> bbox area reaches MISSION_GATE_READY_AREA
-> PASS_GATE forward-only command for MISSION_GATE_PASS_DISTANCE
```

The default dwell is `5.0 s`. That is deliberate: it gives ArduPilot and the
vehicle enough time to settle while the camera controller keeps reducing image
error. It is a tuning parameter, not a hidden delay.

Clearance is image-space validation around the selected bounding-box center. It
is not metric obstacle avoidance and it does not infer gate depth from RGB. Tune
these values for camera mounting and physical vehicle margins:

```text
VISUAL_PASS_TARGET_OFFSET_X/Y
VISUAL_PASS_CLEARANCE_LEFT/RIGHT/UP/DOWN
```

`MISSION_GATE_READY_AREA` is also image-space. It prevents a far but centered
gate from triggering the pass sequence too early. Tune it from diagnostics; do
not treat it as a calibrated distance unless camera intrinsics and gate physical
dimensions are added later.

The four clearance margins are asymmetric so the engineer can reserve extra
space for protrusions such as GPS mounts or landing gear without changing code.
Once committed to `PASS_GATE`, the mission commands stable forward body
velocity plus altitude hold; it does not keep lateral/yaw visual corrections
active inside the gate.

## Next-Gate Acquire

After passing gate 1, the mission first moves forward for
`MISSION_NEXT_GATE_CLEAR_DISTANCE`. Gate 2 detections are ignored during this
clear segment so the vehicle does not immediately re-center while it is still
near the first gate.

After the clear distance, the mission moves forward at a capped acquire speed
while the detector searches for gate 2. It only treats a next-gate detection as
actionable after two area gates:

```text
area < MISSION_NEXT_GATE_MIN_AREA
  -> ignore as too small/noisy
MISSION_NEXT_GATE_MIN_AREA <= area < MISSION_GATE_READY_AREA
  -> keep flying forward while tracking that a far gate exists
area >= MISSION_GATE_READY_AREA
  -> count stable detections, then brake before centering
```

If gate 2 is ready for the required number of ticks, the mission brakes briefly
before centering. If gate 2 is not ready before the configured maximum forward
distance or timeout, the mission falls back to slow seek for the next gate.

## Altitude Control

Takeoff uses ArduPilot's own guided takeoff controller. The mission sends
`MAV_CMD_NAV_TAKEOFF` directly to the configured `1.0 m` target and waits for
fused telemetry to settle before entering gate search. It does not mix a short
bootstrap target with companion-side body-z velocity, because that hybrid
profile overshot in SITL.

```text
GUIDED + armed -> MAV_CMD_NAV_TAKEOFF target 1.0 m
not settled    -> keep waiting in TAKEOFF
stable enough  -> enter SEEK_GATE
```

Current takeoff defaults:

- Target altitude: `1.0 m`.
- Settle band: `+/-0.06 m` for `8` consecutive non-landed ticks.

This is intentionally not a PID or velocity controller in the companion
process. ArduPilot owns the attitude, motor, altitude, and takeoff inner loops;
the mission only gates phase progression based on telemetry.

The mission maintains altitude during search, pass, adaptive acquire, brake, and final forward exit by adding a small vertical velocity correction toward the takeoff altitude. During visual centering, vertical image correction is allowed but bounded by altitude guards.

The altitude input should be fused local telemetry from ArduPilot or a simulator adapter. The mission must not consume raw GPS, raw rangefinder, or raw optical-flow samples directly.

## Robustness Rules

- A single detection must not trigger a phase change unless config allows it.
- A gate pass requires dwell, current clearance validator, and the configurable
  image-space ready-area guard. Area alone must not trigger a pass.
- Loss of detection during centering uses a short grace window before returning
  to search.
- The inter-gate phase first clears the previous gate by forward distance, then
  can use detections to trigger brake-before-center.
- Gate 2 acquisition includes area-window gating and a short brake-before-center
  phase to reduce overshoot risk.
- Centering commands are altitude-guarded so visual servoing cannot keep descending below the configured floor.
- Gate clearance and final exit use forward distance from telemetry.
- Landing is only commanded after the final forward exit distance and a short
  brake/settle phase are complete.

## YOLO and Camera Integration

YOLO is implemented behind the `GateDetection` contract:

```text
Webots TCP camera frame
-> YoloGateDetector
-> GateTargetSelector
-> GateDetection
-> MissionTelemetry.gate_detection
```

Detector-specific code stays in `src/drone_autonomy/perception/`. The current
simulation model is versioned at `models/gate_yolov8n_best.pt` and loaded only
by runtime detector profiles, not by the mission state machine.
The current model-safe filter is class name `Goals-Detection` plus YOLO class
id `3`; do not assume class id `0` is a gate after retraining with extra
labels.

Current simulation profile: this repo's `iris_camera.wbt` requests `rgb24`
from the vendored Webots controller. `gray8` remains supported only for
upstream-compatible fallback worlds; seeing `rgb8_from_gray8` in diagnostics
means the run is not using the current RGB simulation path.

Future camera work should replace only the frame source, for example with a
C920/OpenCV adapter, while preserving the same `YoloGateDetector ->
GateDetection -> mission` boundary.

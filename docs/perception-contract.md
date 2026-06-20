# Perception Contract

The autonomy code expects a gate detector, not a specific neural-network runtime.

## Input

The detector may consume:

- Logitech C920 Pro RGB frames on real hardware.
- Webots `iris_camera.wbt` TCP camera frames in simulation.
- Recorded frames for tests or tuning.

Current implemented simulation path:

```text
webots/worlds/iris_camera.wbt
-> TCP camera stream on 127.0.0.1:5599
-> WebotsTcpCameraClient
-> YoloGateDetector
-> GateTargetSelector
-> GateDetection
```

This repo's `iris_camera.wbt` profile requests `rgb24` from the vendored Webots
controller so the simulation input is closer to ordinary RGB camera video.
`gray8` decoding remains available for upstream-compatible fallback worlds; if
diagnostics show `rgb8_from_gray8` during this repo's normal simulation run,
the world/controller/config is stale or mismatched.

## Output

YOLO produces raw `GateCandidate` objects. The target selector validates,
tracks, smooths, and reduces them to either `None` or one `GateDetection`.

```python
GateDetection(
    bbox=BoundingBox(x_min=..., y_min=..., x_max=..., y_max=...),
    confidence=0.0_to_1.0,
    class_name="Goals-Detection",
    observed_at_s=timestamp,
)
```

The current bundled model is `models/gate_yolov8n_best.pt` and should be
accepted by class name `Goals-Detection` plus class id `3`.
Numeric class ids can change after retraining; the current multi-class model
maps `Goals-Detection` to id `3`, not id `0`. Class id and/or class name
filtering should be explicit for whichever model is active, and both filters
must not be empty during motion tests.

## Bounding Box Semantics

The bounding box should cover the visible gate frame, not the empty hole inside it. The controller uses the bounding-box center for centering and area as a rough approach/proximity signal.

## Detector Responsibilities

- Filter by confidence.
- Filter by gate class name and/or class id.
- Produce raw gate candidates with timestamps.
- Avoid changing vehicle state.

## Target Selector Responsibilities

- Validate candidate geometry such as area, aspect ratio, and ROI.
- Prefer the nearer/larger gate when two gate boxes are visible.
- Maintain target continuity with a light lock/IoU score.
- Smooth the selected bounding box before visual servoing.
- Require stable hits across a frame window before publishing a target.
- Fill timestamps from the camera or monotonic clock.
- Avoid changing vehicle state.

## Mission Responsibilities

- Decide whether detection is stable enough.
- Decide when to pass, acquire the next gate, brake, or land.
- Command ArduPilot through an adapter.

## Open Calibration Items

RGB-only distance is not metric without assumptions. For robust real-world use, add one of:

- known gate physical dimensions plus camera intrinsics,
- rangefinder or depth sensor,
- local position/optical flow estimate,
- visual tracking calibrated in simulation and verified on hardware.

# Perception Contract

The autonomy code expects a gate detector, not a specific neural-network runtime.

## Input

The detector may consume:

- Logitech C920 Pro RGB frames on real hardware.
- Webots RGB camera frames in simulation.
- Recorded frames for tests or tuning.

Current implemented simulation path:

```text
webots/worlds/iris_camera.wbt
-> TCP camera stream on 127.0.0.1:5599
-> WebotsTcpCameraClient
-> YoloGateDetector
-> GateDetection
```

The upstream ArduPilot Webots camera stream is grayscale. The current adapter
expands it to three channels before YOLO. Treat this as simulation wiring and
shape detection, not as final RGB-camera validation.

## Output

Each processed frame should return either `None` or one `GateDetection`.

```python
GateDetection(
    bbox=BoundingBox(x_min=..., y_min=..., x_max=..., y_max=...),
    confidence=0.0_to_1.0,
    class_name="gate",
    observed_at_s=timestamp,
)
```

## Bounding Box Semantics

The bounding box should cover the visible gate frame, not the empty hole inside it. The controller uses the bounding-box center for centering and area as a rough approach/proximity signal.

## Detector Responsibilities

- Pick the best gate candidate in the current frame.
- Filter by confidence.
- Filter by gate class name or class id.
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

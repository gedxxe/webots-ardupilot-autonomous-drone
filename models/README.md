# Model Artifacts

This folder stores local perception model artifacts used by simulation profiles.

Current gate model:

- `gate_yolov8n_best.pt`
- YOLO family: YOLOv8n
- Accepted class id for this repo profile: `0`
- Observed class name: `Goals-Detection`

The mission state machine must not import this model directly. Runtime detector
profiles load it through `YoloGateDetector -> GateDetection`.

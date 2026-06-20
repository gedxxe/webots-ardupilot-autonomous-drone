# Model Artifacts

This folder stores local perception model artifacts used by simulation profiles.

Current gate model:

- `gate_yolov8n_best.pt`
- YOLO family: YOLOv8n
- Accepted class filter for this repo profile: class name `Goals-Detection`
  plus class id `3`
- Current observed class id after the multi-class retrain: `3`

Do not assume class id `0` is a gate. The current metadata has id `0` as an
unrelated class in the retrained model. Runtime defaults therefore use
`YOLO_GATE_CLASS_NAMES="Goals-Detection"` with `YOLO_GATE_CLASS_IDS="3"`. If
retraining changes the class order, inspect the model metadata or `data.yaml`
before changing those filters.

The mission state machine must not import this model directly. Runtime detector
profiles load it through `YoloGateDetector -> GateDetection`.

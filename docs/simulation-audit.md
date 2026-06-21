# Simulation Readiness Audit

## Pre-Run Checklist

Repository baseline:

- `webots/` has been compared against the official ArduPilot
  `libraries/SITL/examples/Webots_Python` tracked tree.
- Expected tracked Webots file count: 36.
- Current compare result: 0 missing upstream files, 0 extra local tracked files.
- Folder timestamp is not used as evidence because copied files can retain
  source modification times.

Before any autonomy command test:

- Webots is installed and can open the target world.
- ArduPilot checkout exists outside this repo.
- ArduPilot SITL has been built for copter.
- `configs/sitl_webots.env` points to the correct ArduPilot checkout.
- Python virtual environment is active.
- `pip install -e ".[dev]"` has completed.
- `drone-autonomy --mode heartbeat --connection udp:127.0.0.1:14551` succeeds
  when Mission Planner is using `14550`.
- `LOCAL_POSITION_NED` appears in MAVLink listen output.

Before `--send-commands`:

- You are connected to SITL, not hardware.
- Webots simulation is running.
- Course direction `COURSE_FORWARD_X/Y` is understood or intentionally left at default for smoke testing.
- You are ready to stop SITL/Webots if the vehicle moves incorrectly.

## Ready in Code

- Mission state machine for two gates.
- ArduPilot-managed 1 m takeoff profile using `MAV_CMD_NAV_TAKEOFF`, followed by
  telemetry settle gating (`+/-0.06 m` non-landed settle band).
- Center-dwell plus configurable image-space clearance before committed gate pass.
- Forward-only committed pass segment after clearance validation.
- Configurable post-gate clear distance before gate 2 acquisition instead of a
  blind timed sprint.
- Brake-before-center after the next gate is detected during adaptive acquire.
- Altitude-hold velocity bias from fused `LOCAL_POSITION_NED` altitude.
- MAVLink telemetry adapter for heartbeat, arm state, landed state, altitude, and forward position.
- MAVLink command adapter for mode, arm, takeoff, land, and body-frame velocity.
- Runtime loop that can run dry or send commands.
- Synthetic gate detector for SITL wiring tests that bypass real perception.
- Webots TCP camera adapter for `iris_camera.wbt`.
- YOLO gate detector wrapper that converts model boxes to `GateDetection`.
- Trained YOLOv8n gate model at `models/gate_yolov8n_best.pt`.
- Iris camera YOLO launcher profile at `scripts/run_iris_camera_yolo.sh`.
- Serial MAVLink baud support for future Raspberry Pi USB Pixhawk tests.
- Raspberry Pi dry-run deployment scaffold and paper-oriented math docs.

## Still Required Outside Code

- `pip install -e ".[vision]"` before using `--detector webots-yolo`.
- ArduPilot SITL running and publishing MAVLink on the configured UDP endpoint.
- Correct Webots course alignment relative to `LOCAL_POSITION_NED`.
- SITL validation that the trained model recognizes the experimental goal asset
  from the actual `iris_camera.wbt` camera viewpoint.

## Available Baseline Webots Assets

- Vendored ArduPilot Webots Python example in `webots/`.
- Baseline Iris world at `webots/worlds/iris.wbt`.
- Camera-capable examples at `webots/worlds/iris_camera.wbt` and
  `webots/worlds/iris_depth_camera.wbt`.
- Params at `webots/params/iris.parm`.
- Experimental gate asset at `webots/protos/RobotstadiumGoal.proto`.
- `webots/worlds/iris_camera.wbt` includes two experimental
  `RobotstadiumGoal` instances for early perception/world iteration.

The baseline Webots tree is complete for ArduPilot's upstream examples. The
current `RobotstadiumGoal` additions are experimental and still need detector
validation from the Iris camera viewpoint plus course-direction verification
before they are treated as a competition-grade world.

## Not Yet Implemented in Code

- `COMMAND_ACK` parsing and retry policy.
- Lost-heartbeat failsafe in the runtime loop.
- Real-hardware C920/OpenCV camera source.
- Dedicated YAML/TOML mission tuning file. Current tuning is available through
  CLI flags and `configs/autonomy_runtime.env`.
- Automatic course-frame calibration.
- Validated hardware flight launch profile. The current Raspberry Pi launcher is
  dry-run scaffold only.

## High-Risk Items to Verify in SITL

- Body-frame velocity signs: forward, right, down, and yaw-rate.
- `COURSE_FORWARD_X/Y` projection matches the actual gate-line direction.
- `LOCAL_POSITION_NED.z` behaves as expected with Webots.
- Webots can resolve external Cyberbotics `EXTERNPROTO` dependencies used by the
  upstream example worlds, such as `StraightRoadSegment`.
- ArduPilot accepts `MAV_CMD_NAV_TAKEOFF` to `1.0 m` in GUIDED mode and settles
  without companion-side body-z velocity during TAKEOFF.
- Gate pass distance and final forward exit distance are realistic for the world scale.
- `MISSION_CENTER_DWELL`, `VISUAL_PASS_CLEARANCE_*`, and
  `MISSION_NEXT_GATE_CLEAR_DISTANCE` are tuned for the actual drone dimensions,
  camera mount, and gate opening.
- `MISSION_NEXT_GATE_MIN_AREA` and `MISSION_GATE_READY_AREA` are tuned from the
  OpenCV diagnostics view so gate 2 is not accepted while it is still too far
  away in image space.
- `YOLO_IMGSZ` is treated as inference/letterbox size only. Camera geometry for
  visual servo and diagnostics remains `VISUAL_FRAME_WIDTH/HEIGHT`, currently
  640x480 for `iris_camera.wbt`.
- `webots-yolo` detections only occur for class name `Goals-Detection` plus
  the verified gate class id `3`, and not unrelated objects such as
  Dog/Forklift/Table.
- Synthetic detector should not be used for judging gate behavior.

## Known Design Choices

- Sensor fusion belongs to ArduPilot EKF or simulator-equivalent fused telemetry, not raw Python blending.
- RGB detection controls alignment and image-space clearance, not metric distance.
- Bounding-box area is an image-space readiness guard only until gate dimensions
  and camera calibration are known; it is not metric distance.
- Runtime defaults to dry-run so accidental hardware motion does not happen.

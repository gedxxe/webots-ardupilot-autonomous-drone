# Main Logic Map

Dokumen ini memetakan jalur utama kode simulasi agar runtime tidak terlihat seperti black box.

## Entry Point

CLI entry point:

```text
drone-autonomy
```

Source:

```text
src/drone_autonomy/cli.py
```

Mode yang tersedia:

- `heartbeat`: cek MAVLink heartbeat.
- `listen`: print raw MAVLink messages.
- `autonomy`: jalankan runtime loop.

## Runtime Loop

Source:

```text
src/drone_autonomy/runtime/autonomy_loop.py
```

Loop utama melakukan ini setiap tick:

```text
drain MAVLink messages
-> update telemetry adapter
-> get latest gate detection
-> build MissionTelemetry
-> mission.update(telemetry)
-> optionally send VehicleCommand to ArduPilot
-> print phase/status
```

Runtime boleh blocking karena runtime adalah process loop. Mission state machine tidak boleh blocking.

## Mission State Machine

Source:

```text
src/drone_autonomy/autonomy/mission.py
```

Mission input:

```text
MissionTelemetry(
  now_s,
  altitude_m,
  forward_position_m,
  mode,
  armed,
  landed,
  gate_detection,
)
```

Mission output:

```text
MissionOutput(
  phase,
  command,
  gate_index,
  detail,
  servo,
)
```

Mission phases:

```text
INIT
TAKEOFF
SEEK_GATE
CENTER_GATE
PASS_GATE
NEXT_GATE_ACQUIRE
BRAKE
FINAL_EXIT
LAND
COMPLETE
FAILSAFE
```

Important:

- `TAKEOFF` sends `MAV_CMD_NAV_TAKEOFF` to `1.0 m` and waits for telemetry
  settle; it does not command companion-side body-z velocity.
- `NEXT_GATE_ACQUIRE` is not blind sprint.
- `BRAKE` after gate 2 detection reduces overshoot before centering.
- `FINAL_EXIT` measures forward distance, not altitude.

## MAVLink Telemetry Adapter

Source:

```text
src/drone_autonomy/mavlink/telemetry.py
```

Responsibilities:

- Convert `HEARTBEAT` to mode and armed state.
- Convert `LOCAL_POSITION_NED` to altitude and forward position.
- Convert `EXTENDED_SYS_STATE` to landed state.

It does not fuse raw sensors. Fusion belongs to ArduPilot EKF or simulator-equivalent telemetry.

## MAVLink Command Adapter

Source:

```text
src/drone_autonomy/mavlink/commands.py
```

Responsibilities:

- Convert `VehicleCommand.set_mode()` to MAVLink mode command.
- Convert `VehicleCommand.arm_vehicle()` to arm/disarm command.
- Convert `VehicleCommand.takeoff()` to takeoff command.
- Convert `VehicleCommand.land()` to land command.
- Convert body velocity command to `SET_POSITION_TARGET_LOCAL_NED`.

Internal velocity convention:

```text
body_vx_m_s > 0: forward
body_vy_m_s > 0: right
body_vz_m_s > 0: down
yaw_rate_rad_s > 0: yaw right/clockwise
```

## Detector Layer

Current simulator wiring:

```text
src/drone_autonomy/perception/synthetic.py
```

Synthetic detector is only for wiring tests. It returns centered gate boxes based on mission phase.
It must keep a fake detection continuous across `SEEK_GATE -> CENTER_GATE`; if
it keys detections by phase, the mission will oscillate between seek and center.

Implemented Webots plus YOLO detector path:

```text
webots/worlds/iris_camera.wbt
-> TCP camera stream 127.0.0.1:5599
-> src/drone_autonomy/perception/webots_camera.py
-> src/drone_autonomy/perception/yolo.py
-> GateDetection
-> MissionTelemetry.gate_detection
```

Current profile launcher:

```text
scripts/run_iris_camera_yolo.sh
-> models/gate_yolov8n_best.pt
-> YOLO class id 0
```

The current upstream Webots stream is `gray8`, expanded to three channels for
YOLO. A future real-hardware path should replace only the frame source with a
C920/OpenCV adapter.

The detector must not command the drone.

## Where Configuration Currently Lives

Runtime CLI flags:

```text
src/drone_autonomy/cli.py
```

Runtime shell env example:

```text
configs/autonomy_runtime.env.example
```

Mission tuning defaults:

```text
GateMissionConfig in src/drone_autonomy/autonomy/mission.py
```

Later, mission tuning should be moved to a user-editable config file before serious tuning.

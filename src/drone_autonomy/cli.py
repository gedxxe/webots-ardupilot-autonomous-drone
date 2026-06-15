from __future__ import annotations

import argparse
from collections.abc import Sequence

from drone_autonomy.mavlink.connection import MavlinkClient
from drone_autonomy.runtime.autonomy_loop import AutonomyRuntime, AutonomyRuntimeConfig


def _csv_strings(value: str) -> tuple[str, ...]:
    """Parse comma-separated CLI/env values into a stable tuple."""

    return tuple(item.strip() for item in value.split(",") if item.strip())


def _csv_ints(value: str) -> tuple[int, ...]:
    """Parse comma-separated integer values used for YOLO class ids."""

    if not value.strip():
        return ()
    return tuple(int(item.strip()) for item in value.split(",") if item.strip())


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI used for both MAVLink smoke tests and autonomy runtime."""

    parser = argparse.ArgumentParser(
        prog="drone-autonomy",
        description="Python companion entry point for Webots + ArduPilot SITL.",
    )
    parser.add_argument(
        "--connection",
        default="udp:127.0.0.1:14550",
        help="MAVLink connection string.",
    )
    parser.add_argument(
        "--mode",
        choices=["heartbeat", "listen", "autonomy"],
        default="heartbeat",
        help="heartbeat/listen are MAVLink checks; autonomy runs the mission loop.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=10,
        help="Number of messages to print in listen mode.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Heartbeat wait timeout in seconds.",
    )
    parser.add_argument(
        "--detector",
        choices=["none", "synthetic", "webots-yolo"],
        default="none",
        help="Detection source for autonomy mode.",
    )
    parser.add_argument(
        "--send-commands",
        action="store_true",
        help="Actually send MAVLink motion commands in autonomy mode.",
    )
    parser.add_argument(
        "--loop-hz",
        type=float,
        default=20.0,
        help="Autonomy loop rate in Hz.",
    )
    parser.add_argument(
        "--max-runtime",
        type=float,
        default=180.0,
        help="Maximum autonomy runtime in seconds.",
    )
    parser.add_argument(
        "--course-forward-x",
        type=float,
        default=1.0,
        help="LOCAL_POSITION_NED x component of course-forward direction.",
    )
    parser.add_argument(
        "--course-forward-y",
        type=float,
        default=0.0,
        help="LOCAL_POSITION_NED y component of course-forward direction.",
    )
    parser.add_argument(
        "--webots-camera-host",
        default="127.0.0.1",
        help="Host for ArduPilot Webots TCP camera stream.",
    )
    parser.add_argument(
        "--webots-camera-port",
        type=int,
        default=5599,
        help="Port for ArduPilot Webots TCP camera stream.",
    )
    parser.add_argument(
        "--webots-camera-encoding",
        choices=["gray8", "rgb24"],
        default="gray8",
        help="Camera stream payload format. Upstream iris_camera.wbt uses gray8.",
    )
    parser.add_argument(
        "--yolo-model",
        default="",
        help="Path to YOLO gate model, required for --detector webots-yolo.",
    )
    parser.add_argument(
        "--yolo-confidence",
        type=float,
        default=0.35,
        help="Minimum YOLO confidence used before GateDetection conversion.",
    )
    parser.add_argument(
        "--yolo-imgsz",
        type=int,
        default=640,
        help="YOLO inference image size.",
    )
    parser.add_argument(
        "--yolo-device",
        default="",
        help="Optional YOLO device string such as cpu, cuda, or cuda:0.",
    )
    parser.add_argument(
        "--gate-class-names",
        default="gate",
        help="Comma-separated YOLO class names accepted as gates. Empty accepts all.",
    )
    parser.add_argument(
        "--gate-class-ids",
        default="",
        help="Comma-separated YOLO class ids accepted as gates.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the requested CLI mode.

    `heartbeat` and `listen` are diagnostics. `autonomy` starts the blocking
    runtime loop that wires telemetry, detector, mission, and command adapter.
    """

    args = build_parser().parse_args(argv)

    if args.mode == "autonomy":
        result = AutonomyRuntime(
            AutonomyRuntimeConfig(
                connection=args.connection,
                loop_hz=args.loop_hz,
                max_runtime_s=args.max_runtime,
                heartbeat_timeout_s=args.timeout,
                detector=args.detector,
                send_commands=args.send_commands,
                course_forward_x=args.course_forward_x,
                course_forward_y=args.course_forward_y,
                webots_camera_host=args.webots_camera_host,
                webots_camera_port=args.webots_camera_port,
                webots_camera_encoding=args.webots_camera_encoding,
                yolo_model_path=args.yolo_model,
                yolo_confidence=args.yolo_confidence,
                yolo_image_size_px=args.yolo_imgsz,
                yolo_device=args.yolo_device,
                yolo_gate_class_names=_csv_strings(args.gate_class_names),
                yolo_gate_class_ids=_csv_ints(args.gate_class_ids),
            )
        ).run()
        return 0 if result.completed else 2

    client = MavlinkClient(args.connection)
    heartbeat = client.wait_heartbeat(timeout=args.timeout)
    print(
        "heartbeat "
        f"system={heartbeat.get_srcSystem()} "
        f"component={heartbeat.get_srcComponent()} "
        f"type={heartbeat.type} "
        f"autopilot={heartbeat.autopilot}"
    )

    if args.mode == "listen":
        for message in client.iter_messages(count=args.count):
            print(message)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

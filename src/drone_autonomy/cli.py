from __future__ import annotations

import argparse
from collections.abc import Sequence

from drone_autonomy.mavlink.connection import MavlinkClient
from drone_autonomy.runtime.autonomy_loop import AutonomyRuntime, AutonomyRuntimeConfig


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
        choices=["none", "synthetic"],
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

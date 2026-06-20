#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys
from time import monotonic, sleep


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from drone_autonomy.perception.webots_camera import (  # noqa: E402
    WebotsCameraConfig,
    WebotsTcpCameraClient,
)


def build_parser() -> argparse.ArgumentParser:
    """Build the Webots camera stream probe CLI."""

    parser = argparse.ArgumentParser(
        description="Probe ArduPilot Webots TCP camera stream without YOLO.",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5599)
    parser.add_argument("--encoding", choices=["gray8", "rgb24"], default="rgb24")
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--connect-timeout", type=float, default=1.0)
    parser.add_argument("--read-timeout", type=float, default=0.05)
    parser.add_argument("--idle-reconnect", type=float, default=2.0)
    parser.add_argument("--poll-interval", type=float, default=0.05)
    return parser


def main() -> int:
    """Read one Webots camera frame and print precise stream status."""

    args = build_parser().parse_args()
    client = WebotsTcpCameraClient(
        WebotsCameraConfig(
            host=args.host,
            port=args.port,
            encoding=args.encoding,
            connect_timeout_s=args.connect_timeout,
            read_timeout_s=args.read_timeout,
            idle_reconnect_s=args.idle_reconnect,
        )
    )

    deadline_s = monotonic() + args.timeout
    next_status_s = 0.0
    try:
        while monotonic() <= deadline_s:
            now_s = monotonic()
            frame = client.read_latest(observed_at_s=now_s)
            if frame is not None:
                print(
                    "camera frame ok "
                    f"source={frame.source} "
                    f"size={frame.width_px}x{frame.height_px} "
                    f"encoding={frame.encoding}"
                )
                return 0

            if now_s >= next_status_s:
                status = client.last_status
                print(
                    "waiting for camera frame "
                    f"status={status.stage} "
                    f"connected={status.connected} "
                    f"buffered={status.buffered_bytes} "
                    f"detail={status.detail}"
                )
                next_status_s = now_s + 1.0

            sleep(args.poll_interval)
    finally:
        client.close()

    status = client.last_status
    print(
        "camera frame probe failed "
        f"status={status.stage} "
        f"connected={status.connected} "
        f"buffered={status.buffered_bytes} "
        f"detail={status.detail}"
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

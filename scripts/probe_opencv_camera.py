#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys
from time import monotonic, sleep
from typing import Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from drone_autonomy.perception.opencv_camera import (  # noqa: E402
    OpenCvCameraConfig,
    OpenCvCameraSource,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Probe an OpenCV camera source without MAVLink or YOLO."
    )
    parser.add_argument("--source", default="0", help="Camera index or device path.")
    parser.add_argument(
        "--backend",
        choices=["default", "v4l2"],
        default="default",
        help="OpenCV capture backend.",
    )
    parser.add_argument("--width", type=int, default=640, help="Requested width.")
    parser.add_argument("--height", type=int, default=480, help="Requested height.")
    parser.add_argument("--fps", type=float, default=30.0, help="Requested FPS.")
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for the first frame.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.timeout <= 0.0:
        raise SystemExit("--timeout must be positive")

    source = OpenCvCameraSource(
        OpenCvCameraConfig(
            source=args.source,
            backend=args.backend,
            width_px=args.width,
            height_px=args.height,
            fps=args.fps,
        )
    )
    deadline_s = monotonic() + args.timeout
    try:
        while monotonic() < deadline_s:
            frame = source.read_latest(monotonic())
            if frame is not None:
                print(
                    "opencv_camera_ok "
                    f"source={frame.source} "
                    f"size={frame.width_px}x{frame.height_px} "
                    f"encoding={frame.encoding}"
                )
                return 0
            sleep(0.05)

        status = source.last_status
        print(
            "opencv_camera_failed "
            f"stage={status.stage} detail={status.detail}"
        )
        return 2
    finally:
        source.close()


if __name__ == "__main__":
    raise SystemExit(main())

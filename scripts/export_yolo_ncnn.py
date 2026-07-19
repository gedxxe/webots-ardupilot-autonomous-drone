#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Export the gate YOLO model to NCNN for Raspberry Pi deployment. "
            "The exported directory is a generated artifact and is ignored by git."
        )
    )
    parser.add_argument(
        "--model",
        default="models/gate_yolov8n_best.pt",
        help="Input PyTorch .pt model path.",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Ultralytics export image size. This is not camera resolution.",
    )
    parser.add_argument(
        "--device",
        default="cpu",
        help="Ultralytics export device, for example cpu or cuda:0.",
    )
    parser.add_argument(
        "--half",
        action="store_true",
        help="Request half precision export when supported by the backend.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    model_path = Path(args.model)
    if not model_path.exists():
        raise SystemExit(f"model file not found: {model_path}")
    if args.imgsz <= 0:
        raise SystemExit("--imgsz must be positive")

    try:
        from ultralytics import YOLO
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Ultralytics is required for NCNN export. "
            "Install vision/export dependencies first: "
            "pip install -e '.[vision]' ultralytics[export]"
        ) from exc

    model = YOLO(str(model_path), task="detect")
    export_kwargs: dict[str, object] = {
        "format": "ncnn",
        "imgsz": args.imgsz,
        "device": args.device,
    }
    if args.half:
        export_kwargs["half"] = True

    exported = model.export(**export_kwargs)
    exported_path = Path(str(exported))
    if not exported_path.exists():
        raise SystemExit(f"NCNN export finished but output was not found: {exported}")
    if not exported_path.is_dir():
        raise SystemExit(f"NCNN export output is not a directory: {exported_path}")

    print(f"ncnn_export_ok path={exported_path}")
    print("Use this directory as YOLO_MODEL_PATH for opencv-yolo dry-run validation.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

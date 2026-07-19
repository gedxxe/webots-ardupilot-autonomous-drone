#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from time import monotonic, sleep

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from drone_autonomy.perception.opencv_camera import (
    OpenCvCameraConfig,
    OpenCvCameraSource,
)
from drone_autonomy.perception.target_selector import GateTargetSelectorConfig
from drone_autonomy.perception.webots_camera import WebotsCameraConfig
from drone_autonomy.perception.webots_yolo import WebotsYoloConfig, WebotsYoloGateProvider
from drone_autonomy.perception.yolo import YoloGateConfig
from drone_autonomy.perception.yolo_profile import (
    DEFAULT_GATE_CLASS_IDS,
    DEFAULT_GATE_CLASS_NAMES,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="OpenCV camera + YOLO diagnostics probe. No MAVLink is used.",
    )
    parser.add_argument("--model", required=True, help="YOLO .pt or exported model path.")
    parser.add_argument("--source", default="0", help="OpenCV camera source.")
    parser.add_argument("--backend", choices=["default", "v4l2"], default="default")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="cpu")
    parser.add_argument(
        "--gate-class-names",
        default=_csv_names(DEFAULT_GATE_CLASS_NAMES),
        help="Comma-separated class names accepted as gates.",
    )
    parser.add_argument(
        "--gate-class-ids",
        default=_csv_ids(DEFAULT_GATE_CLASS_IDS),
        help="Comma-separated class ids accepted as gates.",
    )
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--diagnostics-window", action="store_true")
    args = parser.parse_args()

    camera = OpenCvCameraSource(
        OpenCvCameraConfig(
            source=args.source,
            backend=args.backend,
            width_px=args.width,
            height_px=args.height,
            fps=args.fps,
        )
    )
    selected_count = 0
    start_s = monotonic()
    next_status_s = start_s
    provider = WebotsYoloGateProvider(
        WebotsYoloConfig(
            camera=WebotsCameraConfig(read_timeout_s=0.05, idle_reconnect_s=1.0),
            yolo=YoloGateConfig(
                model_path=args.model,
                confidence=args.confidence,
                image_size_px=args.imgsz,
                device=args.device,
                gate_class_names=_parse_csv_names(args.gate_class_names),
                gate_class_ids=_parse_csv_ids(args.gate_class_ids),
            ),
            selector=GateTargetSelectorConfig(),
            detection_stale_s=0.75,
            pipeline_name="opencv-yolo-probe",
            camera_wait_source=f"opencv:{args.source}",
            camera_read_timeout_s=0.05,
            diagnostics_window=args.diagnostics_window,
            diagnostics_window_name="OpenCV YOLO Probe",
        ),
        camera=camera,
    )

    try:
        print(
            "opencv_yolo_probe start "
            f"source={args.source} backend={args.backend} "
            f"model={args.model} names={args.gate_class_names} "
            f"ids={args.gate_class_ids}"
        )
        while monotonic() - start_s <= args.timeout:
            now_s = monotonic()
            provider.process_events()
            provider.update_context(phase="probe", gate_index=0)
            detection = provider.detect(now_s)
            if detection is not None:
                selected_count += 1
            if now_s >= next_status_s:
                status = provider.camera.last_status
                if detection is None:
                    print(
                        "opencv_yolo_probe waiting "
                        f"camera_status={status.stage} detail={status.detail}"
                    )
                else:
                    bbox = detection.bbox
                    print(
                        "opencv_yolo_probe selected "
                        f"conf={detection.confidence:0.2f} "
                        f"class={detection.class_name} "
                        f"bbox=({bbox.x_min:0.0f},{bbox.y_min:0.0f},"
                        f"{bbox.x_max:0.0f},{bbox.y_max:0.0f})"
                    )
                next_status_s = now_s + 1.0
            sleep(0.05)
    finally:
        provider.close()

    print(f"opencv_yolo_probe done selected_frames={selected_count}")
    return 0


def _csv_names(values: tuple[str, ...]) -> str:
    return ",".join(values)


def _csv_ids(values: tuple[int, ...]) -> str:
    return ",".join(str(value) for value in values)


def _parse_csv_names(value: str) -> tuple[str, ...]:
    return tuple(item.strip() for item in value.split(",") if item.strip())


def _parse_csv_ids(value: str) -> tuple[int, ...]:
    if not value.strip():
        return ()
    return tuple(int(item.strip()) for item in value.split(",") if item.strip())


if __name__ == "__main__":
    raise SystemExit(main())

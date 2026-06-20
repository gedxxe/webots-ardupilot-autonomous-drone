#!/usr/bin/env python3

from pathlib import Path
import time
import cv2
from ultralytics import YOLO


def draw_debug(frame, result, model_names, conf_thres=0.25):
    """
    Draw YOLO detections manually for clearer debugging.
    """
    if result.boxes is None:
        return frame

    boxes = result.boxes

    for box in boxes:
        conf = float(box.conf[0])
        if conf < conf_thres:
            continue

        cls_id = int(box.cls[0])
        class_name = model_names.get(cls_id, str(cls_id))

        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

        label = f"{cls_id}:{class_name} {conf:.2f}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        text_y = max(y1 - 8, 20)
        cv2.putText(
            frame,
            label,
            (x1, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        # Center point debug
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        cv2.putText(
            frame,
            f"({cx},{cy})",
            (cx + 6, cy - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )

    return frame


def main():
    # Folder tempat script ini berada:
    script_dir = Path(__file__).resolve().parent

    # Mundur 1 langkah ke folder "models":
    models_dir = script_dir.parent

    model_path = models_dir / "gate_yolov8n_best.pt"
    video_path = script_dir / "webots_test.mp4"

    if not model_path.exists():
        raise FileNotFoundError(f"Model tidak ditemukan: {model_path}")

    if not video_path.exists():
        raise FileNotFoundError(f"Video test tidak ditemukan: {video_path}")

    print("[INFO] Loading YOLO model...")
    print(f"[INFO] Model path : {model_path}")
    print(f"[INFO] Video path : {video_path}")

    model = YOLO(str(model_path))

    print("\n[INFO] Model class names:")
    for cls_id, name in model.names.items():
        print(f"  {cls_id}: {name}")

    cap = cv2.VideoCapture(str(video_path))

    if not cap.isOpened():
        raise RuntimeError(f"Gagal membuka video: {video_path}")

    video_fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print("\n[INFO] Video info:")
    print(f"  Resolution  : {width}x{height}")
    print(f"  FPS         : {video_fps:.2f}")
    print(f"  Frames      : {total_frames}")
    print("\n[INFO] Controls:")
    print("  q     : quit")
    print("  space : pause/resume")
    print("  s     : save current frame")
    print()

    conf_thres = 0.25
    paused = False
    frame_idx = 0
    last_frame = None

    while True:
        if not paused:
            ret, frame = cap.read()

            if not ret:
                print("[INFO] Video selesai.")
                break

            frame_idx += 1
            last_frame = frame.copy()

        else:
            if last_frame is None:
                continue
            frame = last_frame.copy()

        start_time = time.time()

        results = model.predict(
            source=frame,
            conf=conf_thres,
            imgsz=640,
            verbose=False,
            device="0",  # ganti ke 0 kalau mau pakai GPU CUDA
        )

        inference_time = (time.time() - start_time) * 1000.0
        result = results[0]

        debug_frame = frame.copy()
        debug_frame = draw_debug(
            debug_frame,
            result,
            model.names,
            conf_thres=conf_thres,
        )

        detected_count = 0
        if result.boxes is not None:
            detected_count = len(result.boxes)

        overlay_lines = [
            f"Frame: {frame_idx}/{total_frames}",
            f"Detections: {detected_count}",
            f"Inference: {inference_time:.1f} ms",
            f"Conf thres: {conf_thres:.2f}",
            "q: quit | space: pause | s: save",
        ]

        y = 25
        for line in overlay_lines:
            cv2.putText(
                debug_frame,
                line,
                (15, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            y += 25

        cv2.imshow("YOLOv8n Gate Model Debug", debug_frame)

        key = cv2.waitKey(1 if not paused else 0) & 0xFF

        if key == ord("q"):
            print("[INFO] Quit.")
            break

        elif key == ord(" "):
            paused = not paused
            print(f"[INFO] Paused: {paused}")

        elif key == ord("s"):
            save_path = script_dir / f"debug_frame_{frame_idx:06d}.jpg"
            cv2.imwrite(str(save_path), debug_frame)
            print(f"[INFO] Saved frame: {save_path}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

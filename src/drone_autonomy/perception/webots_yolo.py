from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from threading import Event, Lock, Thread, get_ident
from time import monotonic
from typing import Protocol

from drone_autonomy.perception.detections import GateDetection
from drone_autonomy.perception.frames import CameraFrame, CameraSourceStatus
from drone_autonomy.perception.target_selector import (
    CandidateEvaluation,
    GateCandidate,
    GateSelectionResult,
    GateTargetContext,
    GateTargetSelector,
    GateTargetSelectorConfig,
)
from drone_autonomy.perception.webots_camera import (
    WebotsCameraConfig,
    WebotsTcpCameraClient,
)
from drone_autonomy.perception.yolo import YoloGateConfig, YoloGateDetector


@dataclass(frozen=True)
class WebotsYoloConfig:
    """Config for a camera-source plus YOLO perception pipeline.

    The class name is kept for compatibility with the original Webots path, but
    the provider can also use an injected OpenCV camera source.
    """

    camera: WebotsCameraConfig
    yolo: YoloGateConfig
    selector: GateTargetSelectorConfig = field(default_factory=GateTargetSelectorConfig)
    detection_stale_s: float = 0.25
    pipeline_name: str = "webots-yolo"
    camera_wait_source: str = ""
    camera_read_timeout_s: float = 0.0
    diagnostics_window: bool = False
    diagnostics_window_name: str = "Webots YOLO Gate Diagnostics"
    diagnostics_pass_target_offset_x: float = 0.0
    diagnostics_pass_target_offset_y: float = 0.0
    diagnostics_pass_clearance_left_error: float = 0.090
    diagnostics_pass_clearance_right_error: float = 0.090
    diagnostics_pass_clearance_up_error: float = 0.130
    diagnostics_pass_clearance_down_error: float = 0.130
    diagnostics_next_gate_min_area_ratio: float = 0.015
    diagnostics_gate_ready_area_ratio: float = 0.060

    def __post_init__(self) -> None:
        if self.detection_stale_s <= 0.0:
            raise ValueError("detection_stale_s must be positive")
        if not self.pipeline_name:
            raise ValueError("pipeline_name must not be empty")
        if self.camera_read_timeout_s < 0.0:
            raise ValueError("camera_read_timeout_s must be non-negative")
        if not self.diagnostics_window_name:
            raise ValueError("diagnostics_window_name must not be empty")
        clearance_values = (
            self.diagnostics_pass_clearance_left_error,
            self.diagnostics_pass_clearance_right_error,
            self.diagnostics_pass_clearance_up_error,
            self.diagnostics_pass_clearance_down_error,
        )
        if any(value < 0.0 for value in clearance_values):
            raise ValueError("diagnostics pass clearance errors must be non-negative")
        if self.diagnostics_next_gate_min_area_ratio < 0.0:
            raise ValueError("diagnostics next-gate min area must be non-negative")
        if self.diagnostics_gate_ready_area_ratio < 0.0:
            raise ValueError("diagnostics gate ready area must be non-negative")


class CameraFrameSource(Protocol):
    """Thread-owned source that reads camera frames from simulator/hardware."""

    last_status: CameraSourceStatus

    def read_latest(self, observed_at_s: float) -> CameraFrame | None:
        """Return a frame, or `None` while the stream is not ready."""

    def close(self) -> None:
        """Release frame-source resources."""


class FrameGateDetector(Protocol):
    """Detector used by the background perception worker."""

    def detect_candidates(
        self,
        frame: CameraFrame,
        now_s: float,
    ) -> tuple[GateCandidate, ...]:
        """Return raw gate candidates for one frame."""


class WebotsYoloGateProvider:
    """Run camera ingestion and YOLO inference in background workers.

    The mission loop must not block on camera reads or YOLO inference. This
    provider therefore owns two bounded-latest workers:

    - camera worker: reads frames and publishes only the newest frame.
    - detector worker: consumes the newest frame and publishes the newest
      `GateDetection | None`.

    No queue grows without bound. If YOLO is slower than the camera, older frames
    are dropped and the detector catches up to the latest frame.
    """

    def __init__(
        self,
        config: WebotsYoloConfig,
        *,
        camera: CameraFrameSource | None = None,
        detector: FrameGateDetector | None = None,
    ) -> None:
        self.config = config
        self.camera = camera or WebotsTcpCameraClient(config.camera)
        self.detector = detector or YoloGateDetector(config.yolo)
        self.selector = GateTargetSelector(config.selector)

        self._lock = Lock()
        self._stop_event = Event()
        self._frame_ready_event = Event()
        self._latest_frame: CameraFrame | None = None
        self._latest_frame_seq = 0
        self._latest_detection: GateDetection | None = None
        self._latest_detection_frame_s = -999.0
        self._processed_frame_seq = 0
        self._target_context = GateTargetContext()
        self._latest_diagnostics_canvas: object | None = None
        self._latest_diagnostics_seq = 0

        self._last_camera_warning_s = -999.0
        self._last_detector_error_s = -999.0
        self._camera_ready_announced = False
        self._diagnostics_disabled = False
        self._diagnostics_displayed_seq = 0
        self._diagnostics_window_open = False
        self._diagnostics_owner_thread_id: int | None = None
        self._closed = False

        self._camera_thread = Thread(
            target=self._camera_worker,
            name=f"{config.pipeline_name}-camera-reader",
            daemon=True,
        )
        self._detector_thread = Thread(
            target=self._detector_worker,
            name=f"{config.pipeline_name}-detector",
            daemon=True,
        )
        try:
            self._camera_thread.start()
            self._detector_thread.start()
        except BaseException:
            self._stop_event.set()
            self._frame_ready_event.set()
            if self._camera_thread.is_alive():
                self._camera_thread.join()
            if self._detector_thread.is_alive():
                self._detector_thread.join()
            self.camera.close()
            raise

    def update_context(self, *, phase: str, gate_index: int) -> None:
        """Update mission context used by target validation and tracking."""

        with self._lock:
            previous = self._target_context
            self._target_context = GateTargetContext(phase=phase, gate_index=gate_index)
            should_reset = gate_index != previous.gate_index
        if should_reset:
            self.selector.reset()

    def detect(self, now_s: float) -> GateDetection | None:
        """Return the latest fresh detection without blocking the mission loop."""

        self._print_camera_waiting_warning(now_s)

        with self._lock:
            detection = self._latest_detection
            detection_frame_s = self._latest_detection_frame_s

        if detection is None:
            return None
        if now_s - detection_frame_s > self.config.detection_stale_s:
            return None
        return detection

    def stale_reason(self, now_s: float) -> str | None:
        """Return a fail-closed reason when frame/inference progress has stalled."""

        if not self._camera_thread.is_alive():
            return f"{self.config.pipeline_name} camera worker stopped"
        if not self._detector_thread.is_alive():
            return f"{self.config.pipeline_name} detector worker stopped"

        with self._lock:
            frame = self._latest_frame
            processed_frame_seq = self._processed_frame_seq
            processed_frame_s = self._latest_detection_frame_s

        if frame is None:
            return f"{self.config.pipeline_name} camera has not produced a frame"
        frame_age_s = now_s - frame.observed_at_s
        if frame_age_s > self.config.detection_stale_s:
            return (
                f"{self.config.pipeline_name} camera frame stale "
                f"age={frame_age_s:0.2f}s limit={self.config.detection_stale_s:0.2f}s"
            )
        if processed_frame_seq == 0:
            return f"{self.config.pipeline_name} detector has not completed inference"
        inference_age_s = now_s - processed_frame_s
        if inference_age_s > self.config.detection_stale_s:
            return (
                f"{self.config.pipeline_name} inference stale "
                f"age={inference_age_s:0.2f}s "
                f"limit={self.config.detection_stale_s:0.2f}s"
            )
        return None

    def process_events(self) -> None:
        """Pump optional diagnostics UI events from the runtime/main thread."""

        self._pump_diagnostics_window()

    def close(self) -> None:
        """Stop workers and release camera/GUI resources deterministically."""

        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        self._frame_ready_event.set()
        try:
            # A detector invocation cannot be cancelled safely. Wait for its
            # current frame to finish so Python does not tear down Ultralytics,
            # OpenCV, or Qt while the worker still owns native resources.
            self._camera_thread.join()
            self._detector_thread.join()
        finally:
            try:
                self.camera.close()
            finally:
                self._close_diagnostics_window()

    def _camera_worker(self) -> None:
        """Continuously read camera frames and publish only the newest one."""

        while not self._stop_event.is_set():
            frame = self.camera.read_latest(observed_at_s=monotonic())
            if frame is None:
                self._stop_event.wait(self._camera_read_timeout_s())
                continue

            with self._lock:
                self._latest_frame = frame
                self._latest_frame_seq += 1

            self._frame_ready_event.set()
            if not self._camera_ready_announced:
                prefix = f"{self.config.pipeline_name} camera frame ready "
                print(
                    f"{prefix}{frame.width_px}x{frame.height_px} "
                    f"encoding={frame.encoding}"
                )
                self._camera_ready_announced = True

    def _detector_worker(self) -> None:
        """Run YOLO on the newest frame and publish the newest detection."""

        last_seen_seq = 0
        while not self._stop_event.is_set():
            if not self._frame_ready_event.wait(timeout=0.1):
                continue

            with self._lock:
                frame = self._latest_frame
                seq = self._latest_frame_seq
                if frame is None or seq == last_seen_seq:
                    self._frame_ready_event.clear()
                    continue

            last_seen_seq = seq
            try:
                candidates = self.detector.detect_candidates(frame, monotonic())
            except Exception as exc:  # pragma: no cover - defensive runtime path
                candidates = ()
                now_s = monotonic()
                if now_s - self._last_detector_error_s >= 2.0:
                    print(f"{self.config.pipeline_name} detector error: {exc}")
                    self._last_detector_error_s = now_s
            raw_prediction_summary = _format_raw_prediction_summary(
                getattr(self.detector, "last_raw_predictions", ())
            )

            with self._lock:
                context = self._target_context
            selection = self.selector.update(candidates, context=context)
            detection = selection.detection
            diagnostics_canvas = self._build_diagnostics_canvas(
                frame,
                selection,
                raw_prediction_summary,
            )

            with self._lock:
                if seq >= self._processed_frame_seq:
                    self._latest_detection = detection
                    self._latest_detection_frame_s = (
                        detection.observed_at_s
                        if detection is not None
                        else frame.observed_at_s
                    )
                    self._processed_frame_seq = seq
                    self._latest_diagnostics_canvas = diagnostics_canvas
                    self._latest_diagnostics_seq = seq
                if self._latest_frame_seq == seq:
                    self._frame_ready_event.clear()

    def _print_camera_waiting_warning(self, now_s: float) -> None:
        if self._camera_ready_announced:
            return
        if now_s - self._last_camera_warning_s < 2.0:
            return

        status = self.camera.last_status
        source = self.config.camera_wait_source
        if not source:
            source = f"tcp://{self.config.camera.host}:{self.config.camera.port}"
        print(
            f"{self.config.pipeline_name} waiting for camera frame "
            f"{source} "
            f"status={status.stage} detail={status.detail}"
        )
        self._last_camera_warning_s = now_s

    def _camera_read_timeout_s(self) -> float:
        if self.config.camera_read_timeout_s > 0.0:
            return self.config.camera_read_timeout_s
        return self.config.camera.read_timeout_s

    def _build_diagnostics_canvas(
        self,
        frame: CameraFrame,
        selection: GateSelectionResult,
        raw_prediction_summary: str,
    ) -> object | None:
        """Build an overlay in the detector worker without invoking HighGUI."""

        with self._lock:
            diagnostics_disabled = self._diagnostics_disabled
        if not self.config.diagnostics_window or diagnostics_disabled:
            return None

        try:
            import cv2
            import numpy as np
        except ModuleNotFoundError as exc:  # pragma: no cover - environment guard
            self._disable_diagnostics(exc)
            return None

        image = getattr(frame, "image", None)
        shape = getattr(image, "shape", None)
        if shape is None:
            return None

        try:
            canvas = np.ascontiguousarray(image.copy())
            if len(shape) == 2:
                canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
            elif len(shape) == 3 and shape[2] >= 3:
                canvas = cv2.cvtColor(canvas[:, :, :3], cv2.COLOR_RGB2BGR)
            else:
                return None

            self._draw_validator_roi(cv2, canvas, selection)
            self._draw_crosshair(cv2, canvas)
            self._draw_pass_clearance(cv2, canvas)
            self._draw_area_guides(cv2, canvas, selection)

            for evaluation in selection.evaluations:
                bbox = evaluation.candidate.detection.bbox
                is_selected = evaluation is selection.selected
                if is_selected and selection.detection is not None:
                    color = (0, 255, 0)
                elif evaluation.accepted:
                    color = (0, 220, 255)
                else:
                    color = (0, 0, 255)
                cv2.rectangle(
                    canvas,
                    (int(bbox.x_min), int(bbox.y_min)),
                    (int(bbox.x_max), int(bbox.y_max)),
                    color,
                    3 if is_selected else 2,
                )
                self._draw_candidate_label(
                    cv2,
                    canvas,
                    evaluation,
                    color,
                    y=max(20, int(bbox.y_min) - 8),
                )

            lines = [
                f"frame {frame.width_px}x{frame.height_px} {frame.encoding}",
                (
                    f"phase={selection.context.phase} "
                    f"gate={selection.context.gate_index + 1}"
                ),
                (
                    f"candidates={len(selection.evaluations)} "
                    f"stable={selection.stable_hits}/"
                    f"{self.config.selector.required_stable_frames} "
                    f"lost={selection.lost_frames}"
                ),
                f"stale_limit={self.config.detection_stale_s:0.2f}s",
                raw_prediction_summary,
                "cyan=center magenta=pass-clearance target",
                "blue=far-area orange=ready-area",
            ]
            if selection.detection is not None:
                lines.append(
                    f"selected conf={selection.detection.confidence:0.2f} "
                    f"class={selection.detection.class_name}"
                )
            elif selection.selected is not None:
                lines.append("selected warming up")
            else:
                lines.append("selected=None")

            for index, line in enumerate(lines):
                y = 22 + (index * 22)
                cv2.putText(
                    canvas,
                    line,
                    (12, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 0, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    canvas,
                    line,
                    (12, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

            return canvas
        except Exception as exc:  # pragma: no cover - GUI/runtime dependent
            self._disable_diagnostics(exc)
            return None

    def _pump_diagnostics_window(self) -> None:
        """Display the newest overlay and process Qt events on one thread."""

        if not self.config.diagnostics_window:
            return

        current_thread_id = get_ident()
        if self._diagnostics_owner_thread_id is None:
            self._diagnostics_owner_thread_id = current_thread_id
        elif self._diagnostics_owner_thread_id != current_thread_id:
            self._disable_diagnostics(
                "window event loop was called from more than one thread"
            )
            return

        with self._lock:
            disabled = self._diagnostics_disabled
            canvas = self._latest_diagnostics_canvas
            canvas_seq = self._latest_diagnostics_seq

        if disabled:
            self._close_diagnostics_window()
            return
        if canvas is None and not self._diagnostics_window_open:
            return

        try:
            import cv2
        except ModuleNotFoundError as exc:  # pragma: no cover - environment guard
            self._disable_diagnostics(exc)
            return
        _remove_invalid_qt_font_override()

        try:
            if canvas is not None and canvas_seq != self._diagnostics_displayed_seq:
                cv2.imshow(self.config.diagnostics_window_name, canvas)
                self._diagnostics_displayed_seq = canvas_seq
                self._diagnostics_window_open = True

            key = cv2.pollKey() if hasattr(cv2, "pollKey") else cv2.waitKey(1)
            if key in (27, ord("q"), ord("Q")):
                self._disable_diagnostics("window closed by operator")
                self._close_diagnostics_window()
                return

            visible_property = getattr(cv2, "WND_PROP_VISIBLE", None)
            if (
                self._diagnostics_window_open
                and hasattr(cv2, "getWindowProperty")
                and visible_property is not None
            ):
                visible = cv2.getWindowProperty(
                    self.config.diagnostics_window_name,
                    visible_property,
                )
                if visible < 1.0:
                    self._disable_diagnostics("window closed by operator")
                    self._diagnostics_window_open = False
        except Exception as exc:  # pragma: no cover - GUI/runtime dependent
            self._disable_diagnostics(exc)
            self._close_diagnostics_window()

    def _close_diagnostics_window(self) -> None:
        """Destroy this provider's HighGUI window on its owning thread."""

        if not self._diagnostics_window_open:
            return
        if (
            self._diagnostics_owner_thread_id is not None
            and self._diagnostics_owner_thread_id != get_ident()
        ):
            print(
                f"{self.config.pipeline_name} diagnostics close skipped: "
                "called from a non-owner thread"
            )
            return

        self._diagnostics_window_open = False
        try:
            import cv2

            cv2.destroyWindow(self.config.diagnostics_window_name)
            if hasattr(cv2, "pollKey"):
                cv2.pollKey()
            else:
                cv2.waitKey(1)
        except Exception as exc:  # pragma: no cover - GUI/runtime dependent
            print(f"{self.config.pipeline_name} diagnostics close warning: {exc}")

    def _disable_diagnostics(self, reason: object) -> None:
        """Disable only diagnostics; perception and mission behavior continue."""

        with self._lock:
            if self._diagnostics_disabled:
                return
            self._diagnostics_disabled = True
        print(f"{self.config.pipeline_name} diagnostics disabled: {reason}")

    def _draw_validator_roi(
        self,
        cv2: object,
        canvas: object,
        selection: GateSelectionResult,
    ) -> None:
        roi = selection.validation_roi
        if roi is None:
            return
        cv2.rectangle(
            canvas,
            (int(roi.x_min), int(roi.y_min)),
            (int(roi.x_max), int(roi.y_max)),
            (255, 120, 0),
            1,
        )

    def _draw_crosshair(self, cv2: object, canvas: object) -> None:
        height, width = canvas.shape[:2]
        center_x = width // 2
        center_y = height // 2
        cv2.line(
            canvas,
            (center_x - 18, center_y),
            (center_x + 18, center_y),
            (255, 255, 0),
            1,
        )
        cv2.line(
            canvas,
            (center_x, center_y - 18),
            (center_x, center_y + 18),
            (255, 255, 0),
            1,
        )

    def _draw_pass_clearance(self, cv2: object, canvas: object) -> None:
        """Draw the visual-servo pass target and clearance margins."""

        height, width = canvas.shape[:2]
        half_width = width / 2.0
        half_height = height / 2.0
        target_x = half_width + (
            self.config.diagnostics_pass_target_offset_x * half_width
        )
        target_y = half_height + (
            self.config.diagnostics_pass_target_offset_y * half_height
        )
        x_min = target_x - (
            self.config.diagnostics_pass_clearance_left_error * half_width
        )
        x_max = target_x + (
            self.config.diagnostics_pass_clearance_right_error * half_width
        )
        y_min = target_y - (
            self.config.diagnostics_pass_clearance_up_error * half_height
        )
        y_max = target_y + (
            self.config.diagnostics_pass_clearance_down_error * half_height
        )

        cv2.rectangle(
            canvas,
            (int(x_min), int(y_min)),
            (int(x_max), int(y_max)),
            (255, 0, 255),
            1,
        )
        cv2.line(
            canvas,
            (int(target_x) - 12, int(target_y)),
            (int(target_x) + 12, int(target_y)),
            (255, 0, 255),
            1,
        )
        cv2.line(
            canvas,
            (int(target_x), int(target_y) - 12),
            (int(target_x), int(target_y) + 12),
            (255, 0, 255),
            1,
        )

    def _draw_area_guides(
        self,
        cv2: object,
        canvas: object,
        selection: GateSelectionResult,
    ) -> None:
        """Draw centered bbox-area references used by mission area gating."""

        if (
            self.config.diagnostics_next_gate_min_area_ratio <= 0.0
            and self.config.diagnostics_gate_ready_area_ratio <= 0.0
        ):
            return

        aspect_ratio = 1.0
        if selection.selected is not None and selection.selected.aspect_ratio > 0.0:
            aspect_ratio = selection.selected.aspect_ratio

        self._draw_area_box(
            cv2,
            canvas,
            area_ratio=self.config.diagnostics_next_gate_min_area_ratio,
            aspect_ratio=aspect_ratio,
            color=(255, 120, 0),
            label="far",
        )
        self._draw_area_box(
            cv2,
            canvas,
            area_ratio=self.config.diagnostics_gate_ready_area_ratio,
            aspect_ratio=aspect_ratio,
            color=(0, 165, 255),
            label="ready",
        )

    def _draw_area_box(
        self,
        cv2: object,
        canvas: object,
        *,
        area_ratio: float,
        aspect_ratio: float,
        color: tuple[int, int, int],
        label: str,
    ) -> None:
        if area_ratio <= 0.0:
            return

        height, width = canvas.shape[:2]
        area_px = area_ratio * float(width * height)
        box_width = (area_px * aspect_ratio) ** 0.5
        box_height = area_px / box_width if box_width > 0.0 else 0.0
        center_x = width / 2.0
        center_y = height / 2.0
        x_min = center_x - (box_width / 2.0)
        x_max = center_x + (box_width / 2.0)
        y_min = center_y - (box_height / 2.0)
        y_max = center_y + (box_height / 2.0)

        cv2.rectangle(
            canvas,
            (int(x_min), int(y_min)),
            (int(x_max), int(y_max)),
            color,
            1,
        )
        cv2.putText(
            canvas,
            f"{label} a={area_ratio:0.3f}",
            (int(x_min), max(18, int(y_min) - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            color,
            1,
            cv2.LINE_AA,
        )

    def _draw_candidate_label(
        self,
        cv2: object,
        canvas: object,
        evaluation: CandidateEvaluation,
        color: tuple[int, int, int],
        *,
        y: int,
    ) -> None:
        bbox = evaluation.candidate.detection.bbox
        reasons = ",".join(evaluation.reasons) if evaluation.reasons else "ok"
        class_name = evaluation.candidate.detection.class_name
        class_id = evaluation.candidate.class_id
        class_label = (
            f"cls={class_id}:{class_name}"
            if class_id is not None
            else f"cls={class_name}"
        )
        appearance_label = f"g={evaluation.appearance_score:0.2f}"
        label = (
            f"{class_label} "
            f"s={evaluation.score:0.2f} "
            f"{appearance_label} "
            f"a={evaluation.area_ratio:0.2f} "
            f"ar={evaluation.aspect_ratio:0.2f} "
            f"e=({evaluation.center_error_x:+0.2f},{evaluation.center_error_y:+0.2f}) "
            f"{reasons}"
        )
        cv2.putText(
            canvas,
            label,
            (int(bbox.x_min), y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            color,
            1,
            cv2.LINE_AA,
        )


def _format_raw_prediction_summary(raw_predictions: object) -> str:
    """Summarize YOLO raw classes before this pipeline applies class filtering."""

    try:
        predictions = tuple(raw_predictions)
    except TypeError:
        predictions = ()
    if not predictions:
        return "raw=<none>"

    counts: dict[str, int] = {}
    for prediction in predictions:
        class_id = getattr(prediction, "class_id", None)
        class_name = getattr(prediction, "class_name", "unknown")
        key = (
            f"{class_id}:{class_name}"
            if class_id is not None
            else str(class_name)
        )
        counts[key] = counts.get(key, 0) + 1

    ordered = sorted(counts.items(), key=lambda item: item[0])
    return "raw=" + " ".join(f"{label}x{count}" for label, count in ordered)


def _remove_invalid_qt_font_override() -> None:
    """Let Qt use system fontconfig when OpenCV points to missing wheel fonts."""

    font_dir = os.environ.get("QT_QPA_FONTDIR")
    if font_dir and not Path(font_dir).is_dir():
        os.environ.pop("QT_QPA_FONTDIR", None)

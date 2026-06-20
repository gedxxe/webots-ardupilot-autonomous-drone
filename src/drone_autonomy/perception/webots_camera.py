from __future__ import annotations

from dataclasses import dataclass
import socket
import struct
from time import monotonic

from drone_autonomy.perception.frames import CameraFrame


@dataclass(frozen=True)
class WebotsCameraConfig:
    """TCP camera stream settings for ArduPilot's Webots Python controller.

    The vendored `iris_camera.wbt` uses `--camera-port 5599` and this repo's
    profile requests `rgb24`. The dataclass default remains upstream-compatible
    `gray8` so camera-only tests and external ArduPilot worlds can opt in
    without the repo-specific world argument.
    """

    host: str = "127.0.0.1"
    port: int = 5599
    encoding: str = "gray8"
    connect_timeout_s: float = 1.0
    read_timeout_s: float = 0.05
    idle_reconnect_s: float = 2.0
    max_frame_bytes: int = 10_000_000

    def __post_init__(self) -> None:
        if self.port <= 0 or self.port > 65535:
            raise ValueError("camera port must be in the range 1..65535")
        if self.encoding not in {"gray8", "rgb24"}:
            raise ValueError("camera encoding must be 'gray8' or 'rgb24'")
        if self.connect_timeout_s <= 0.0:
            raise ValueError("connect_timeout_s must be positive")
        if self.read_timeout_s <= 0.0:
            raise ValueError("read_timeout_s must be positive")
        if self.idle_reconnect_s <= self.read_timeout_s:
            raise ValueError("idle_reconnect_s must be greater than read_timeout_s")
        if self.max_frame_bytes <= 0:
            raise ValueError("max_frame_bytes must be positive")


@dataclass(frozen=True)
class WebotsCameraStatus:
    """Last observed TCP camera stream state for runtime diagnostics.

    Detection code should not branch mission behavior on this status. It exists
    so SITL runs can distinguish a missing Webots listener from a slow/partial
    frame or an invalid stream payload.
    """

    stage: str = "not_started"
    detail: str = "camera client has not attempted a read yet"
    connected: bool = False
    buffered_bytes: int = 0


def decode_webots_camera_payload(
    *,
    width_px: int,
    height_px: int,
    payload: bytes,
    encoding: str,
    observed_at_s: float,
    source: str = "webots-tcp",
) -> CameraFrame:
    """Decode one Webots camera payload into a YOLO-ready frame.

    `gray8` is expanded to three identical channels for compatibility with the
    upstream ArduPilot camera stream. `rgb24` is preserved as true RGB and is
    the expected format for this repo's `iris_camera.wbt` simulation profile.
    """

    try:
        import numpy as np
    except ModuleNotFoundError as exc:  # pragma: no cover - environment guard
        raise RuntimeError(
            "NumPy is required for Webots camera frames. Install vision extras: "
            "pip install -e '.[vision]'"
        ) from exc

    if width_px <= 0 or height_px <= 0:
        raise ValueError("frame dimensions must be positive")

    if encoding == "gray8":
        expected_len = width_px * height_px
        if len(payload) != expected_len:
            raise ValueError(
                f"gray8 payload has {len(payload)} bytes, expected {expected_len}"
            )
        gray = np.frombuffer(payload, dtype=np.uint8).reshape((height_px, width_px))
        image = np.repeat(gray[:, :, None], 3, axis=2)
        frame_encoding = "rgb8_from_gray8"
    elif encoding == "rgb24":
        expected_len = width_px * height_px * 3
        if len(payload) != expected_len:
            raise ValueError(
                f"rgb24 payload has {len(payload)} bytes, expected {expected_len}"
            )
        image = np.frombuffer(payload, dtype=np.uint8).reshape((height_px, width_px, 3))
        frame_encoding = "rgb8"
    else:
        raise ValueError("camera encoding must be 'gray8' or 'rgb24'")

    return CameraFrame(
        image=image,
        observed_at_s=observed_at_s,
        width_px=width_px,
        height_px=height_px,
        encoding=frame_encoding,
        source=source,
    )


class WebotsTcpCameraClient:
    """Pull camera frames from ArduPilot's Webots TCP image stream.

    This class does not perform detection. It only converts the simulator image
    stream into `CameraFrame` objects. Keeping this separate makes the later
    hardware path straightforward: replace this source with a C920/OpenCV source
    while keeping the YOLO detector and mission contract intact.
    """

    _HEADER_FORMAT = "=HH"
    _HEADER_SIZE = struct.calcsize(_HEADER_FORMAT)

    def __init__(self, config: WebotsCameraConfig | None = None) -> None:
        self.config = config or WebotsCameraConfig()
        self._socket: socket.socket | None = None
        self._buffer = bytearray()
        self._last_byte_s: float | None = None
        self.last_status = WebotsCameraStatus()

    def read_latest(self, observed_at_s: float) -> CameraFrame | None:
        """Return one frame or `None` when the stream is unavailable.

        The call is bounded by `read_timeout_s`. Partial reads are preserved so
        the next tick can continue the same frame; corrupt frames close the
        socket so the next tick reconnects at a frame boundary.
        """

        sock = self._ensure_socket()
        if sock is None:
            return None

        # A normal timeout only means Webots has not emitted enough bytes for
        # the next frame yet. Keep the socket and partial buffer so the Webots
        # server does not see a connect/disconnect loop between camera ticks.
        if not self._fill_buffer(sock, self._HEADER_SIZE, stage="header"):
            return None

        header = bytes(self._buffer[: self._HEADER_SIZE])
        width_px, height_px = struct.unpack(self._HEADER_FORMAT, header)
        channels = 1 if self.config.encoding == "gray8" else 3
        payload_len = width_px * height_px * channels
        frame_len = self._HEADER_SIZE + payload_len

        if payload_len <= 0 or payload_len > self.config.max_frame_bytes:
            self._set_status(
                "invalid_header",
                (
                    f"width={width_px} height={height_px} "
                    f"payload_bytes={payload_len}"
                ),
                connected=True,
            )
            self.close()
            return None

        if not self._fill_buffer(sock, frame_len, stage="payload"):
            return None

        payload = bytes(self._buffer[self._HEADER_SIZE : frame_len])
        del self._buffer[:frame_len]

        try:
            frame = decode_webots_camera_payload(
                width_px=width_px,
                height_px=height_px,
                payload=payload,
                encoding=self.config.encoding,
                observed_at_s=observed_at_s,
                source=f"tcp://{self.config.host}:{self.config.port}",
            )
        except ValueError as exc:
            self._set_status("decode_error", str(exc), connected=True)
            self.close()
            return None

        self._set_status(
            "frame_ready",
            f"{width_px}x{height_px} {self.config.encoding}",
            connected=True,
        )
        return frame

    def close(self) -> None:
        """Close the TCP connection if it is open."""

        self._buffer.clear()
        self._last_byte_s = None
        if self._socket is None:
            return
        try:
            self._socket.close()
        finally:
            self._socket = None

    def _ensure_socket(self) -> socket.socket | None:
        if self._socket is not None:
            return self._socket

        try:
            sock = socket.create_connection(
                (self.config.host, self.config.port),
                timeout=self.config.connect_timeout_s,
            )
        except OSError as exc:
            self._set_status(
                "connect_failed",
                f"{self.config.host}:{self.config.port} {exc}",
                connected=False,
            )
            return None

        sock.settimeout(self.config.read_timeout_s)
        self._socket = sock
        self._last_byte_s = monotonic()
        self._set_status(
            "connected",
            f"tcp://{self.config.host}:{self.config.port}",
            connected=True,
        )
        return sock

    def _fill_buffer(self, sock: socket.socket, byte_count: int, *, stage: str) -> bool:
        """Read until at least `byte_count` buffered bytes exist.

        Socket timeouts are not treated as fatal. The Webots controller sends at
        camera FPS, while the autonomy loop may poll faster than that. Closing
        on every timeout makes Webots print repeated camera client disconnects
        and prevents a full frame from accumulating.
        """

        while len(self._buffer) < byte_count:
            try:
                data = sock.recv(byte_count - len(self._buffer))
            except (BlockingIOError, TimeoutError, socket.timeout):
                idle_s = self._idle_duration_s()
                if idle_s >= self.config.idle_reconnect_s:
                    self._set_status(
                        f"{stage}_idle_reconnect",
                        (
                            f"no bytes for {idle_s:0.2f}s "
                            f"buffered={len(self._buffer)} required={byte_count}"
                        ),
                        connected=False,
                    )
                    self.close()
                    return False
                self._set_status(
                    f"waiting_for_{stage}",
                    f"buffered={len(self._buffer)} required={byte_count}",
                    connected=True,
                )
                return False
            except OSError as exc:
                self._set_status(f"{stage}_read_error", str(exc), connected=False)
                self.close()
                return False
            if not data:
                self._set_status(
                    "stream_closed",
                    f"server closed while reading {stage}",
                    connected=False,
                )
                self.close()
                return False
            self._last_byte_s = monotonic()
            self._buffer.extend(data)
        return True

    def _set_status(self, stage: str, detail: str, *, connected: bool) -> None:
        self.last_status = WebotsCameraStatus(
            stage=stage,
            detail=detail,
            connected=connected,
            buffered_bytes=len(self._buffer),
        )

    def _idle_duration_s(self) -> float:
        if self._last_byte_s is None:
            return 0.0
        return monotonic() - self._last_byte_s

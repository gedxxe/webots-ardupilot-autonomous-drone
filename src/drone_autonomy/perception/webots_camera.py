from __future__ import annotations

from dataclasses import dataclass
import socket
import struct

from drone_autonomy.perception.frames import CameraFrame


@dataclass(frozen=True)
class WebotsCameraConfig:
    """TCP camera stream settings for ArduPilot's Webots Python controller.

    The vendored `iris_camera.wbt` uses `--camera-port 5599`. Upstream ArduPilot
    currently streams `gray8` frames as `uint16 width`, `uint16 height`, then
    `width * height` bytes. `rgb24` is included for future/local controller
    variants but is not emitted by the upstream controller today.
    """

    host: str = "127.0.0.1"
    port: int = 5599
    encoding: str = "gray8"
    connect_timeout_s: float = 1.0
    read_timeout_s: float = 0.05
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
        if self.max_frame_bytes <= 0:
            raise ValueError("max_frame_bytes must be positive")


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

    The current upstream Webots stream is grayscale. For YOLO, grayscale is
    expanded to three identical channels. This preserves geometry for detection
    smoke tests but does not pretend to be a true RGB camera feed.
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

    def read_latest(self, observed_at_s: float) -> CameraFrame | None:
        """Return one frame or `None` when the stream is unavailable.

        The call is bounded by `read_timeout_s`. On partial/corrupt reads the
        socket is closed so the next tick reconnects at a frame boundary.
        """

        sock = self._ensure_socket()
        if sock is None:
            return None

        # A normal timeout only means Webots has not emitted enough bytes for
        # the next frame yet. Keep the socket and partial buffer so the Webots
        # server does not see a connect/disconnect loop between camera ticks.
        if not self._fill_buffer(sock, self._HEADER_SIZE):
            return None

        header = bytes(self._buffer[: self._HEADER_SIZE])
        width_px, height_px = struct.unpack(self._HEADER_FORMAT, header)
        channels = 1 if self.config.encoding == "gray8" else 3
        payload_len = width_px * height_px * channels
        frame_len = self._HEADER_SIZE + payload_len

        if payload_len <= 0 or payload_len > self.config.max_frame_bytes:
            self.close()
            return None

        if not self._fill_buffer(sock, frame_len):
            return None

        payload = bytes(self._buffer[self._HEADER_SIZE : frame_len])
        del self._buffer[:frame_len]

        try:
            return decode_webots_camera_payload(
                width_px=width_px,
                height_px=height_px,
                payload=payload,
                encoding=self.config.encoding,
                observed_at_s=observed_at_s,
                source=f"tcp://{self.config.host}:{self.config.port}",
            )
        except ValueError:
            self.close()
            return None

    def close(self) -> None:
        """Close the TCP connection if it is open."""

        self._buffer.clear()
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
        except OSError:
            return None

        sock.settimeout(self.config.read_timeout_s)
        self._socket = sock
        return sock

    def _fill_buffer(self, sock: socket.socket, byte_count: int) -> bool:
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
                return False
            except OSError:
                self.close()
                return False
            if not data:
                self.close()
                return False
            self._buffer.extend(data)
        return True

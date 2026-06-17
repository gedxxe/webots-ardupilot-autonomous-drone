import socket
import struct

import pytest

np = pytest.importorskip("numpy")

from drone_autonomy.perception.webots_camera import (
    WebotsCameraConfig,
    WebotsTcpCameraClient,
    decode_webots_camera_payload,
)


class _ChunkedSocket:
    """Minimal socket fake for partial Webots TCP frame tests."""

    def __init__(self, data: bytes = b"") -> None:
        self._data = bytearray(data)
        self.closed = False

    def append(self, data: bytes) -> None:
        self._data.extend(data)

    def recv(self, byte_count: int) -> bytes:
        if not self._data:
            raise socket.timeout()
        chunk = bytes(self._data[:byte_count])
        del self._data[:byte_count]
        return chunk

    def close(self) -> None:
        self.closed = True


def test_decode_gray8_payload_expands_to_three_channels() -> None:
    payload = bytes([0, 10, 20, 30, 40, 50])

    frame = decode_webots_camera_payload(
        width_px=3,
        height_px=2,
        payload=payload,
        encoding="gray8",
        observed_at_s=12.5,
    )

    assert frame.width_px == 3
    assert frame.height_px == 2
    assert frame.observed_at_s == 12.5
    assert frame.encoding == "rgb8_from_gray8"
    assert frame.image.shape == (2, 3, 3)
    assert np.all(frame.image[:, :, 0] == frame.image[:, :, 1])
    assert np.all(frame.image[:, :, 1] == frame.image[:, :, 2])


def test_decode_payload_rejects_wrong_size() -> None:
    with pytest.raises(ValueError):
        decode_webots_camera_payload(
            width_px=3,
            height_px=2,
            payload=b"short",
            encoding="gray8",
            observed_at_s=0.0,
        )


def test_tcp_client_keeps_partial_frame_across_timeout() -> None:
    width_px = 3
    height_px = 2
    payload = bytes([1, 2, 3, 4, 5, 6])
    header = struct.pack("=HH", width_px, height_px)

    fake_socket = _ChunkedSocket(header + payload[:3])
    client = WebotsTcpCameraClient(WebotsCameraConfig())
    client._socket = fake_socket

    assert client.read_latest(observed_at_s=1.0) is None
    assert fake_socket.closed is False

    fake_socket.append(payload[3:])
    frame = client.read_latest(observed_at_s=1.1)

    assert frame is not None
    assert frame.width_px == width_px
    assert frame.height_px == height_px
    assert frame.image.shape == (height_px, width_px, 3)

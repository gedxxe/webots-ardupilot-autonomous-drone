import pytest

np = pytest.importorskip("numpy")

from drone_autonomy.perception.webots_camera import decode_webots_camera_payload


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

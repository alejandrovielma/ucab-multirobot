import cv2
import numpy as np

from mobile_base.vision import VisionParams, detect_cubes


def _board_with_cubes():
    img = np.full((480, 640, 3), 255, dtype=np.uint8)  # white board (BGR)
    cv2.line(img, (0, 240), (639, 240), (0, 0, 0), 3)  # thin black line
    cv2.rectangle(img, (80, 300), (140, 360), (20, 20, 200), -1)  # red cube, left
    cv2.rectangle(img, (460, 100), (510, 150), (180, 60, 20), -1)  # blue cube, right
    return img


def test_detects_exactly_red_and_blue():
    detections = detect_cubes(_board_with_cubes(), VisionParams())
    assert sorted(d.color for d in detections) == ["blue", "red"]


def test_normalized_coordinates():
    detections = {d.color: d for d in detect_cubes(_board_with_cubes(), VisionParams())}
    red, blue = detections["red"], detections["blue"]
    assert -1.0 <= red.cx_norm < 0.0  # left half
    assert 0.0 < blue.cx_norm <= 1.0  # right half
    assert blue.cy_norm < red.cy_norm  # blue is higher in the image
    assert 0.0 <= blue.cy_norm <= 1.0
    assert red.area_px > blue.area_px


def test_elongated_blob_is_rejected():
    img = np.full((480, 640, 3), 255, dtype=np.uint8)
    cv2.rectangle(img, (100, 200), (400, 230), (20, 20, 200), -1)  # 10:1 red bar
    assert detect_cubes(img, VisionParams()) == []


def test_tiny_blob_is_rejected():
    img = np.full((480, 640, 3), 255, dtype=np.uint8)
    cv2.rectangle(img, (100, 200), (104, 204), (20, 20, 200), -1)
    assert detect_cubes(img, VisionParams()) == []


def test_blank_board_has_no_detections():
    img = np.full((480, 640, 3), 255, dtype=np.uint8)
    assert detect_cubes(img, VisionParams()) == []


def _raw(img, channels):
    h, w = img.shape[:2]
    padded = np.zeros((h, w * channels + 8), dtype=np.uint8)  # row padding exercises `step`
    padded[:, : w * channels] = img.reshape(h, w * channels)
    return h, w, w * channels + 8, padded.tobytes()


def test_image_to_bgr_handles_encodings_and_step():
    from mobile_base.vision import image_to_bgr

    bgr = np.zeros((4, 6, 3), dtype=np.uint8)
    bgr[..., 0], bgr[..., 1], bgr[..., 2] = 10, 20, 30
    h, w, step, data = _raw(bgr, 3)
    assert np.array_equal(image_to_bgr(h, w, "bgr8", step, data), bgr)

    rgb = bgr[..., ::-1].copy()
    h, w, step, data = _raw(rgb, 3)
    assert np.array_equal(image_to_bgr(h, w, "rgb8", step, data), bgr)

    bgra = np.dstack([bgr, np.full((4, 6), 255, np.uint8)])
    h, w, step, data = _raw(bgra, 4)
    assert np.array_equal(image_to_bgr(h, w, "bgra8", step, data), bgr)


def test_image_to_bgr_rejects_unknown_encoding():
    import pytest

    from mobile_base.vision import image_to_bgr

    with pytest.raises(ValueError):
        image_to_bgr(1, 1, "mono16", 2, b"\x00\x00")

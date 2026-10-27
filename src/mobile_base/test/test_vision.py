import math
import cv2
import numpy as np
import pytest

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


# ---- blue delivery square: never a cube, but the robot sees it


def _board_with_square_and_cube():
    img = np.full((480, 640, 3), 255, dtype=np.uint8)
    cv2.rectangle(img, (120, 250), (420, 420), (180, 60, 20), -1)  # blue square (300 x 170 px)
    cv2.rectangle(img, (500, 300), (540, 340), (180, 60, 20), -1)  # blue cube beside it
    return img


def test_blue_square_is_not_a_cube_but_a_blue_cube_next_to_it_is():
    from mobile_base.vision import detect_square

    img = _board_with_square_and_cube()
    cubes = detect_cubes(img, VisionParams())
    assert [d.color for d in cubes] == ["blue"]
    assert cubes[0].cx_norm > 0.5  # the small one on the right, not the square
    square = detect_square(img, VisionParams())
    assert square.visible
    assert square.cx_norm == pytest.approx((270 / 640) * 2.0 - 1.0, abs=0.01)
    assert square.cy_norm == pytest.approx(335 / 480, abs=0.01)
    assert square.area_px > VisionParams().square_min_area


def test_square_shaped_like_a_cube_is_still_the_square():
    # Seen straight from the entry the square can pass the cube shape filters: its size decides.
    from mobile_base.vision import detect_square

    img = np.full((480, 640, 3), 255, dtype=np.uint8)
    cv2.rectangle(img, (200, 200), (380, 380), (180, 60, 20), -1)
    assert detect_cubes(img, VisionParams()) == []
    assert detect_square(img, VisionParams()).visible


def test_no_square_without_a_big_blue_region():
    from mobile_base.vision import detect_square

    img = _board_with_cubes()  # only a small blue cube
    assert not detect_square(img, VisionParams()).visible
    big_red = np.full((480, 640, 3), 255, dtype=np.uint8)
    cv2.rectangle(big_red, (120, 250), (420, 420), (20, 20, 200), -1)
    assert not detect_square(big_red, VisionParams()).visible


def _marker_image(marker_id, side_px, center_x, width=640, height=480):
    aruco = cv2.aruco
    marker = aruco.generateImageMarker(aruco.getPredefinedDictionary(aruco.DICT_4X4_50), marker_id, side_px)
    img = np.full((height, width, 3), 255, np.uint8)
    x0, y0 = center_x - side_px // 2, height // 2 - side_px // 2
    img[y0:y0 + side_px, x0:x0 + side_px] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return img


def test_the_other_robots_marker_gives_its_bearing_and_distance():
    # No overhead camera: the robot camera sees the marker on the other robot (2026-10-27).
    from mobile_base.vision import VisionParams, detect_peer

    params = VisionParams(peer_marker_id=2, marker_size=0.05, camera_hfov=1.60)
    focal = 320.0 / math.tan(0.8)
    bearing, distance = detect_peer(_marker_image(2, 80, 320), params)
    assert bearing == pytest.approx(0.0, abs=0.02)
    assert distance == pytest.approx(focal * 0.05 / 80.0, rel=0.05)  # ~0.19 m
    right, _ = detect_peer(_marker_image(2, 60, 500), params)
    assert right < 0.0  # to the right of the image: negative bearing (robot convention, left positive)
    assert detect_peer(_marker_image(1, 80, 320), params) is None  # its own marker id is not the peer
    assert detect_peer(_marker_image(2, 80, 320), VisionParams()) is None  # no peer configured

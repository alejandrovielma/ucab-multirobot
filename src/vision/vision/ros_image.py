"""
Conversion between sensor_msgs/Image and OpenCV arrays, without cv_bridge.

Only handles the encodings we use: bgr8, rgb8, bgra8, rgba8 and mono8.
"""

import numpy as np
from sensor_msgs.msg import Image

BGR_ENCODING = "bgr8"
BGR_CHANNELS = 3

# Number of interleaved channels of each supported encoding.
ENCODING_CHANNELS = {
    "bgr8": 3,
    "rgb8": 3,
    "bgra8": 4,
    "rgba8": 4,
    "mono8": 1,
}


def image_to_bgr(msg: Image) -> np.ndarray:
    """
    sensor_msgs/Image -> BGR uint8 array (height, width, 3).
    """
    encoding = msg.encoding.lower()
    if encoding not in ENCODING_CHANNELS:
        raise ValueError(f"unsupported encoding: {msg.encoding}")
    channels = ENCODING_CHANNELS[encoding]
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
        msg.height,
        msg.step,
    )
    pixels = rows[:, : msg.width * channels].reshape(
        msg.height,
        msg.width,
        channels,
    )
    if encoding == "bgr8":
        return pixels.copy()
    if encoding == "rgb8":
        return pixels[:, :, ::-1].copy()
    if encoding == "bgra8":
        return pixels[:, :, :BGR_CHANNELS].copy()
    if encoding == "rgba8":
        # Channels 2, 1, 0 of RGBA are B, G, R.
        return pixels[:, :, 2::-1].copy()
    return np.repeat(pixels, BGR_CHANNELS, axis=2)


def bgr_to_image(bgr: np.ndarray, header=None) -> Image:
    """
    BGR uint8 array -> sensor_msgs/Image (bgr8).
    """
    msg = Image()
    if header is not None:
        msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = BGR_ENCODING
    msg.is_bigendian = 0
    msg.step = msg.width * BGR_CHANNELS
    msg.data = np.ascontiguousarray(bgr, dtype=np.uint8).tobytes()
    return msg

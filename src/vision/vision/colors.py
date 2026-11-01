"""
Colour names used across the vision package. Pure data, no rclpy.

Single source of truth for the list of cube colours and for the colour of the
boxes drawn on the debug image.
"""

# Every colour a cube (or the delivery line) can have on the track.
COLOR_NAMES = ("red", "orange", "yellow", "green", "blue", "purple")

# Box colour for each class in the debug image (BGR).
DRAW_COLORS_BGR = {
    "red": (40, 40, 230),
    "orange": (0, 140, 255),
    "yellow": (0, 230, 255),
    "green": (60, 200, 60),
    "blue": (230, 120, 30),
    "purple": (160, 40, 140),
}

# Used for a class that has no entry in DRAW_COLORS_BGR.
FALLBACK_DRAW_COLOR_BGR = (255, 255, 255)

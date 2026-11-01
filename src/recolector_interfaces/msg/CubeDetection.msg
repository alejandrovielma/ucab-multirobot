# A cube seen by the camera: colour, angle and approximate distance.
# Angle and distance are computed on the floor, assuming a camera at a fixed height
# tilted downwards (see vision/color_detection.py).

string color          # colour name: red, orange, yellow, green, blue, purple
float32 bearing_rad   # angle relative to the camera's forward axis; positive = to the left
float32 distance_m    # distance on the floor from the camera to the bottom edge of the cube
int32 u               # image column of the cube centre (px)
int32 v               # image row of the cube's bottom edge (px)
int32 width_px        # box width in the image (px)
int32 height_px       # box height in the image (px)

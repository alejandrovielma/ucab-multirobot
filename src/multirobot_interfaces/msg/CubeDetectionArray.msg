builtin_interfaces/Time stamp
CubeDetection[] cubes
# Cube inside the fixed scoop: color filling the window between the scoop walls ("" = none).
string scoop_color
float32 scoop_fill
# Blue delivery square seen by the camera (never listed in cubes): largest blue region bigger than
# square_min_area px. Centroid normalized like CubeDetection (cx in [-1, 1], left negative; cy in
# [0, 1], 0 = top); area in px. Values are 0 when square_visible is false.
bool square_visible
float32 square_cx
float32 square_cy
float32 square_area

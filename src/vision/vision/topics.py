"""
Topic and frame names used by the vision nodes.

Relative names resolve under the robot namespace. Keep this in sync with
docs/architecture.md.
"""

TOPIC_IMAGE_RAW = "image_raw"
TOPIC_CUBES = "cubes"
TOPIC_CUBES_DEBUG_IMAGE = "cubes/debug_image"
TOPIC_ZONE_SEEN = "zone/seen"
TOPIC_BEHAVIOR_STATE = "behavior/state"
TOPIC_CMD_VEL_TELEOP = "cmd_vel_teleop"

DEFAULT_QOS_DEPTH = 10

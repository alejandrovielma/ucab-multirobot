"""
Topic, service and frame names used by the mobile_base nodes.

Relative names resolve under the robot namespace (for example
/ferb/cmd_vel). Absolute names belong to the shared fleet (coordinator).
Keep this in sync with docs/architecture.md.
"""

# --- Own topics (relative) ---------------------------------------------------
TOPIC_CMD_VEL_RAW = "cmd_vel_raw"
TOPIC_CMD_VEL_TELEOP = "cmd_vel_teleop"
TOPIC_CMD_VEL = "cmd_vel"
TOPIC_RANGE = "range"
TOPIC_CUBES = "cubes"
TOPIC_BEHAVIOR_STATE = "behavior/state"
TOPIC_SCOOP_HAS_CUBE = "scoop/has_cube"
TOPIC_SCOOP_LOWER = "scoop/lower"
TOPIC_SCOOP_COLOR = "scoop/color"
TOPIC_LINE = "line"
TOPIC_ZONE_SEEN = "zone/seen"
TOPIC_COUNT = "count"
TOPIC_ZONE_WANT = "zone/want"
TOPIC_ZONE_GRANTED = "zone/granted"
TOPIC_ZONE_DELIVERED = "zone/delivered"
TOPIC_FLEET_OK = "fleet/ok"
TOPIC_POSE = "pose"
TOPIC_ROBOT_STATE = "state"

# --- Shared fleet endpoints (absolute) ---------------------------------------
TOPIC_ZONE_OCCUPANT = "/zone/occupant"
# Overhead pose of the OTHER collector (Phineas); we are Ferb.
TOPIC_PEER_POSE = "/phineas/pose"
SERVICE_ZONE_REQUEST = "/zone/request"
SERVICE_ZONE_RELEASE = "/zone/release"

# --- Common QoS --------------------------------------------------------------
DEFAULT_QOS_DEPTH = 10
OCCUPANT_QOS_DEPTH = 1

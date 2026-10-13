# High-level behavior state of one robot.
builtin_interfaces/Time stamp
string robot
string state
bool carrying
uint32 delivered
# Scoop gate commanded closed (cube retained: spins and reverse allowed while carrying).
bool gate_closed

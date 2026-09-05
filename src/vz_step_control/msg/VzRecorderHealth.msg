# Timestamped health of the vz_step OFFBOARD bag recorder, published at ~2 Hz
# (latched). The step controller treats the recorder as required: while a test
# is arming or running it demands a fresh healthy=true, so a step can never
# execute unrecorded (e.g. after the recorder died, or respawned mid-OFFBOARD
# and correctly refused to open a half-log).
std_msgs/Header header
bool healthy       # alive and able to record this or the next session
bool recording     # a bag is currently open for the active OFFBOARD session
string bag_path    # current bag path, empty when no bag is open

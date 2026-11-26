# Timestamped health of the vz_chirp OFFBOARD bag recorder, published at ~2 Hz
# (latched). The chirp controller treats the recorder as required: while a
# sweep session is arming or running it demands a fresh healthy=true (and
# recording=true during STABILIZE/SWEEP/POST_HOLD), so a chirp can never run
# unrecorded (e.g. after the recorder died, or respawned mid-OFFBOARD and
# correctly refused to open a half-log).
std_msgs/Header header
bool healthy       # alive and able to record this or the next session
bool recording     # a bag is currently open for the active OFFBOARD session
string bag_path    # current bag path, empty when no bag is open

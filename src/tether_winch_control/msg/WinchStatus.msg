std_msgs/Header header
# DISABLED / LOW / HIGH / FAULT. FAULT remains latched until explicit enable.
string mode
bool enabled
bool input_valid
string reason
# NaN means unavailable. A geometric distance, NOT measured paid-out length.
float64 distance_m
float64 radial_velocity_mps
# Valid only when command_valid is true; disabled/fault is NOT a 0% command.
bool command_valid
float64 target_duty
string transport
# This version has NO hardware feedback. Always false, applied_duty is NaN.
bool hardware_feedback_valid
float64 applied_duty

std_msgs/Header header
# DISABLED / LOW / HIGH / FAULT. FAULT remains latched until explicit enable.
string mode
bool enabled
bool input_valid
string reason
# NaN means unavailable. A geometric distance, NOT measured paid-out length.
float64 distance_m
float64 radial_velocity_mps
# Valid only when command_valid is true; raw integer units 0..500. Disabled/fault uses transport fallback.
bool command_valid
float64 target_duty
string transport
# No device acknowledgement protocol is available. Always false, applied_duty is NaN.
bool hardware_feedback_valid
float64 applied_duty

# Last successful TCP socket write, NOT hardware acknowledgement; NaN before success.
float64 last_sent_duty
# Unix wall timestamp (independent of ROS /clock), zero before successful write.
time last_send_time
# Latest TCP error; cleared on successful socket write, FAULT remains latched.
string communication_error

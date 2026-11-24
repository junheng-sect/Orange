# Source image timestamp; frame_id names the parent coordinate frame.
std_msgs/Header header

# Marker selected for this image.
int32 marker_id

# Row-major 4x4 matrix with semantics ^header.frame_id T_marker.
float64[16] matrix

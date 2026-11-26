# Result of one ~decode call, published on ~result.
std_msgs/Header header   # stamp/frame of the decoded image
bool success             # all expected_codes reached (or any code when all expected are 0)
bool partial             # some codes decoded but expected_codes not all reached
# "" on success; no_image/no_camera_info/no_tag/busy/disabled/no_formats/decode_fail/partial/cancelled
string error
float32 elapsed_s
bool used_tag_roi        # decode used the label ROI projected from /front_marker/pose
string report            # e.g. "QR=1/1 DM=1/1 C128=2/2"
Barcode[] codes

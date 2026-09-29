# One serial reader attempt, published after every ~read call.
std_msgs/Header header
uint32 request_seq
bool success
# Decoded text (UTF-8, delimiters and surrounding whitespace removed); empty on failure.
string code
# Empty on success; otherwise serial_unavailable/busy/not_configured/timeout/
# empty_frame/cancelled/write_failed/io_error.
string error
float32 elapsed_s
# Raw frame bytes as hex, for debugging.
string raw_hex

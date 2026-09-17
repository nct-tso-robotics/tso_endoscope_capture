# One message per successfully published combined image, with the same timestamp
# and frame ID. No messages are generated during missing input. The separate
# diagnostics topic supplies the node heartbeat during missing input.
std_msgs/Header header

# Configured output frequency. The input can be slower.
float64 configured_publish_hz

# Recent output frequency from steady-clock publication intervals. The estimate
# restarts after missing input/stalls. Zero while measured_rate_valid is false.
float64 measured_publish_hz
bool measured_rate_valid
uint32 rate_window_samples
float64 rate_window_seconds

# Sequence of the accepted SDK callback frame, independent of ROS publication.
uint64 source_sequence

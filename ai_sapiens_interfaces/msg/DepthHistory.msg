# Header stamp is the newest actual acquisition, never the publication time.
# Data is normalized [0,1], oldest frame first, row-major within each frame.
std_msgs/Header header
uint32 width
uint32 height
uint32 frames
float64[8] frame_stamps
float32[4608] data

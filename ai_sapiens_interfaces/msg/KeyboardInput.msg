# Device-neutral command state published by the interactive keyboard teleop node.

std_msgs/Header header
uint32 sequence

bool api_mode
uint16 input_code
uint16 selector_code

# Normalized velocity commands in [-1, 1].
float32 linear_x
float32 linear_y
float32 angular_z

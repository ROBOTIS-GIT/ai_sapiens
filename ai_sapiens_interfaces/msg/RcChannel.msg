# One RC input channel and its derived values.

uint8 rc_channel
string name

# True when the source state-interface sample is finite and within the accepted PWM range.
bool valid
uint16 rc_us
float32 axis
int32 button

bool axis_in_deadzone
bool button_active

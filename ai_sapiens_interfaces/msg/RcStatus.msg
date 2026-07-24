# Runtime RC input and safety status published by the K1 HAT RC broadcaster.

std_msgs/Header header

string sensor_name
string input_topic_name
string status_topic_name

RcChannel[] channels

# 0 means no HAT hardware error.
uint8 hardware_error_status
# HAT tick used to detect stale hardware data.
uint32 realtime_tick
# True while the emergency stop is active.
bool estop_active
# CRSF Last Frame Age, unit ms.
uint16 crsf_last_frame_age_ms
# 0 means CRSF failsafe is inactive.
uint8 crsf_failsafe
# CRSF Link Quality, unit percent.
uint8 crsf_link_quality
# Raw CRSF RSSI 1 value.
uint8 crsf_rssi_1

# Availability and health derived exclusively from the seven HAT RC status interfaces.
bool status_data_valid
bool realtime_tick_fresh
bool all_channels_valid
bool hardware_ok
bool estop_released
bool rc_link_ok
bool is_control_input_safe

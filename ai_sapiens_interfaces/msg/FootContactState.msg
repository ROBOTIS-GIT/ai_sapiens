std_msgs/Header header

# Foot identifiers, e.g. "left"/"right".
string[] foot_names

geometry_msgs/Vector3[] force
float64[] normal_force
float64[] contact_probability

# contact measurement noise.
float64[] covariance
bool[] in_contact

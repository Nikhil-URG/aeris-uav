builtin_interfaces/Time stamp

string target_id
string class_name
float32 confidence

uint32 bbox_x
uint32 bbox_y
uint32 bbox_width
uint32 bbox_height

float64 bearing_rad
float64 range_m

# Estimated target position in PX4 local NED, when the ground projection is
# available. NaN otherwise.
float64 local_x
float64 local_y
float64 local_z

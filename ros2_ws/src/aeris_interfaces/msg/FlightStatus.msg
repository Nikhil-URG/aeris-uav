builtin_interfaces/Time stamp

# Offboard controller state machine name
string state

bool armed
bool offboard

uint32 goal_sequence
bool goal_active
bool goal_reached
bool goal_failed

# Current position in PX4 local NED (metres)
float64 x
float64 y
float64 z
float64 yaw

# Active target in PX4 local NED (metres)
float64 target_x
float64 target_y
float64 target_z

float64 position_error
float64 altitude_m

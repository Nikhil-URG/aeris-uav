builtin_interfaces/Time stamp

string grid_name
string state
bool active
bool success
string message

int32 start_x
int32 start_y
int32 goal_x
int32 goal_y
int32 current_x
int32 current_y

# Grid cells for the active path, in order.
int32[] path_x
int32[] path_y
uint32 current_index
float64 progress
float64 path_length_m
float64 remaining_m

# Actual vehicle position in PX4 local NED.
float64 drone_x
float64 drone_y
float64 drone_z

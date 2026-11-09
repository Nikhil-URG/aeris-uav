builtin_interfaces/Time stamp

# Overall safety verdict. False means at least one fault is active.
bool safe
bool armed
bool offboard

# Safe action selected by the supervisor: none, hold, rtl, land.
string action
# Fault that selected the action, or "none".
string trigger

bool pose_valid
float64 pose_age_s

bool battery_ok
float64 battery_remaining

bool geofence_ok
float64 distance_from_home_m

bool mission_heartbeat_ok
float64 mission_heartbeat_age_s

string[] active_faults

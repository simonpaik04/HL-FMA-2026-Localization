std_msgs/Header header
# Incremented when route, mode or requested direction changes. Planners echo it.
uint64 decision_id
string route_name
uint8 section
string mission
string phase
string selected_branch
string path_mode
# Requested body travel direction, +1 forward / -1 reverse.
int8 direction
float64 progress
float64 distance_m
float64 speed_limit_mps
# Distance from the relevant body reference to the required stop; -1 if none.
float64 remaining_stop_m
bool stop_requested
# Reserved for an actual emergency-stop command. Normal traffic, hill and
# parking holds use stop_requested without asserting the vehicle E-Stop line.
bool emergency_stop_requested
bool valid
bool finished
string reason
# Accepted parking leg. -1 means no calibrated planner leg has been accepted.
int32 parking_leg_index
string parking_leg_phase
float64 parking_leg_target_s

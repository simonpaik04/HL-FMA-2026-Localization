std_msgs/Header header
# Echo the current mission request. Old requests cannot authorize another leg.
uint64 decision_id
string route_name
uint32 leg_index
# FORWARD_APPROACH, REVERSE_ENTRY or FORWARD_EXIT. No implicit completion flag.
string phase
int8 direction
# Distances in the named RDDF's increasing traversal order, measured in metres.
float64 start_s
float64 target_s
bool final_leg

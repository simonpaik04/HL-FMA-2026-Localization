# Planner contract. Require a fresh, valid message matching MissionState's
# route_name / decision_id in traffic sections. Stale/invalid is NOT permission.
std_msgs/Header header
uint64 decision_id
string route_name
bool valid
# True prohibits planning through the stop line; approach remains allowed.
bool active
# Physical line, in RDDF arc length. -1 if geometry is unavailable.
float64 stop_line_s
# Maximum rear-axle/base_link route s, including front bumper + stop buffer.
float64 target_s
# map pose: position is line centre, orientation points along travel direction.
# The transverse line is a route restriction, not a real LaserScan obstacle.
geometry_msgs/Pose stop_line_pose
string required_signal
string reason

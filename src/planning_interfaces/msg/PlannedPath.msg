std_msgs/Header header
uint64 decision_id
string route_name
int8 direction
# Poses contain body heading. A single response represents one travel direction;
# a mixed forward/reverse parking maneuver needs a separate leg handshake.
nav_msgs/Path path

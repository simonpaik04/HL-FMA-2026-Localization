std_msgs/Header header
# The observed junction belongs to this active route, preventing signal reuse.
string route_name
string junction_id
# GREEN, RED, YELLOW, LEFT_ARROW, UNKNOWN. GREEN never implies LEFT_ARROW.
string value
float64 confidence

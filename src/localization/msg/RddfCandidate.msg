# One geometric projection. route_name is the public group; source is the CSV.
string route_name
string source_route_name
# CSV index of the start point of the matched segment (not the route number).
int32 segment_index
geometry_msgs/Point projected_point
float64 distance_m
float64 heading_rad
float64 segment_fraction

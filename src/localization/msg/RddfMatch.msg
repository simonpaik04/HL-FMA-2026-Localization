# Publication time and RDDF map frame; pose_stamp is the Global input time.
std_msgs/Header header
time pose_stamp
bool matched
string reason
# Public group and nearest original CSV, populated only for matched=true.
string route_name
string source_route_name
# Ordered tracking window: current source RDDF followed by connected routes in
# the next numbered section. The first entry is always source_route_name.
string[] active_source_route_names
# Index in source_route_name; -1 when unmatched.
int32 segment_index
# nearest remains diagnostic-only when matched=false; ignore if has_nearest=false.
bool has_nearest
RddfCandidate nearest
# Nearby physical branches; may contain several segments from one route.
RddfCandidate[] candidates

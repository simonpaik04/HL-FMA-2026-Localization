# Publication time and RDDF map frame; pose_stamp is the Global input time.
std_msgs/Header header
time pose_stamp
bool matched
string reason
# Public group and nearest original CSV, populated only for matched=true.
string route_name
string source_route_name
# Index in source_route_name; -1 when unmatched.
int32 segment_index
# nearest remains diagnostic-only when matched=false; ignore if has_nearest=false.
bool has_nearest
RddfCandidate nearest
# Nearby physical branches; may contain several segments from one route.
RddfCandidate[] candidates

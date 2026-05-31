std_msgs/Header header

# Evidence source. For this message, normally:
# "plan_intersection_monitor"
string trigger_source

# Responsible object evidence, if available.
# Empty responsible_object_key means no object match.
string responsible_object_key
string match_type              # "verified" | "inferred" | "unknown"

# Map-frame blockage geometry.
geometry_msgs/Point blockage_centroid
uint32 blocked_plan_index_lo
uint32 blocked_plan_index_hi
float32 blockage_extent_m

# Debounce/arbitration metadata.
string debounce_key
string note
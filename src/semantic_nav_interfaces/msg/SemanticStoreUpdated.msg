string semantic_map_uri          # local file path or future provider URI
string semantic_map_version      # version string derived from the file/provider

uint32 db_version
builtin_interfaces/Time db_stamp

string displaced_object_key
string reason

# Region that motivated the update. Consumers may ignore this and reload full map.
geometry_msgs/Point update_center
float32 update_radius_m

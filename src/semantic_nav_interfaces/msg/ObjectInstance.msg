string object_key
string source_key
string object_tag
string object_caption
string object_state
string safety_class
bool openable
bool clearable
geometry_msgs/Point bbox_center
geometry_msgs/Vector3 bbox_extent
float32 bbox_volume

# Runtime metadata — empty/zero for persistent-map objects (M5B)
string source                          # persistent_map | dynamic_overlay
float32 confidence                     # 0.0–1.0; 0.0 = not reported
builtin_interfaces/Time observation_stamp
float32 ttl_sec                        # 0.0 = not time-limited
string state_detail                    # visible | inferred | ""
string traversability                  # passable | blocked | unknown | ""

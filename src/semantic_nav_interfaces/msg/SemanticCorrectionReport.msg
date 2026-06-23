std_msgs/Header header

string environment_id
string semantic_map_id
string base_map_version

string object_key
string object_tag
string previous_object_state

string correction_type           # suspected_displacement | confirmed_displacement | door_state_observed
string responsible_match_type    # inferred | verified
float32 confidence               # 0.0–1.0

geometry_msgs/Point evidence_center
float32 evidence_radius_m
geometry_msgs/PoseStamped robot_pose
string frame_id

string recovery_event_id
string directive_action
string reason

builtin_interfaces/Time observed_at
string evidence_source           # bt_recovery | operator | perception

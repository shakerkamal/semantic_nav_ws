std_msgs/Header header

# Must match a persistent map object_key where object_tag contains "door".
string object_key

# open | closed | unknown
string door_state

# passable | blocked | unknown
string traversability

# Whether the robot has a capability to open this door.
bool robot_openable

# Detection/state-estimation confidence in [0, 1].
float32 confidence

# If <= 0.0, LocalObjectQueryNode uses default_door_state_ttl_sec.
float32 ttl_sec

string source

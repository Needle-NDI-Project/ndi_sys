# ndi_msgs/msg/RigidArray.msg
# Time of sensor data acquisition, coordinate frame ID.
std_msgs/Header header

# Array of tracker IDs
int32[] ids

# Array of tracker names
string[] frames

# If tracker is within measurement pyramid
bool[] inbound

# Array of 3D poses in the frame given in header
geometry_msgs/Pose[] poses

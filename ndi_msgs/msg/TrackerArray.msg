# ndi_msgs/msg/TrackerArray.msg

# Time of sensor data acquisition, coordinate frame ID.
std_msgs/Header header

# Array of point IDs.
int32[] ids

# Array of 3d points in the frame given in header
geometry_msgs/Point32[] points

# Array of estimation residuals
float32[] residuals
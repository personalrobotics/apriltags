# Message storing raw 2D and 3D information about a single AprilTag detection.
Header header
uint32 id  # id of AprilTag (not necessarily unique)
geometry_msgs/Point32[4] corners2d  # AprilTag corners in image
float32 tag_size  # tag size in m
geometry_msgs/Pose pose  # pose of the marker in the camera frame

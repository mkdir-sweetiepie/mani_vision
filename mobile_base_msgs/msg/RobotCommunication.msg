########################################
# Messages
########################################
# Robot UDP Communication msgs.

Header Header

bool connection 

sensor_msgs/Image[10] image_data # Image data.
float32[10] image_fps # Image Fps.
float32[10] image_bps # Image bps.

geometry_msgs/Point velodyne # velodyne pointcloud.
geometry_msgs/Point slam # SLAM pointcloud data.

geometry_msgs/Point[4] realsense_pointcloud # realsense raw pointcloud.
float32[4] realsense_fps # realsense Fps.
float32[4] realsense_bps # Image bps.

bool robot_error_state
string robot_error

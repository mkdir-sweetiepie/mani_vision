########################################
# Messages
########################################
# Robot Control msgs.

Header  Header

bool init_trigger

Flipper flipper_control
Manipulator manipulator_control

float32 L_RPM
float32 R_RPM

bool[10] image_transport_status # Set status for image transport.

string camera_request # msg for request

bool[4] realsense_transport_status # Set status for realsense pointcloud transport.


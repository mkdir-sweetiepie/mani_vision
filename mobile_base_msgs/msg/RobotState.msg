########################################
# Messages
########################################
# Robot State msgs.

Header  Header

bool initTrigger
bool init_done

bool stm_connection

Flipper flipper_state
Manipulator manipulator_state

float32 L_RPM
float32 R_RPM

float32[5] battery
bool SSR

int32[5] debug

float32 linear_vel
float32 angular_vel

bool[10] image_transport_status # Set status for image transport.
bool[4] realsense_transport_status # Set status for realsense pointcloud transport.

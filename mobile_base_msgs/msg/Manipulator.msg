########################################
# Messages
########################################
# Robot Manipulator msgs.

float32[7] dynamixel_target_position
float32[7] dynamixel_current_position
float32[7] dynamixel_target_speed
float32[7] dynamixel_current_speed

float32[7] dynamixel_current

bool[7] dynamixel_torque_status
bool[7] dynamixel_torque_enable 

string[7] dynamixel_err
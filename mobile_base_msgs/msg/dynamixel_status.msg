########################################
# Messages
########################################
# dynamixel_status msgs.

bool torque                 # on : 1 , off : 0
uint8 hardware_error_status # https://emanual.robotis.com/docs/kr/dxl/mx/mx-106-2/#shutdown63
bool moving                 # moving : 1 , stop : 0
bool in_position            # in position : 1 , out position : 0
float64 present_current     # A
float64 present_velocity    # rps
float64 present_position    # rad
########################################
# Messages
########################################
# Robot mani_vision msgs.

Header  Header

bool in_cam # detecting one is in cam. 

bool in_center 
float32[3] difference

bool turn # left true , right false
float32 gradient
########################################
# Messages
########################################
# Robot mani_vision msgs.

Header  Header

bool in_cam # detecting one is in cam. 

bool in_center 
float32[3] difference

float32 center_depth  
float32 high_depth  
float32 low_depth  

float32[3] gradient
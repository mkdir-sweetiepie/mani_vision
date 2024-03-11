cmake_minimum_required(VERSION 3.0.2)
project(mobile_base_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

# Add your message files with the correct directory
add_message_files(
  DIRECTORY msg 
  FILES 
  RobotControl.msg
  RobotState.msg
  RobotCommunication.msg
  Flipper.msg
  Manipulator.msg
  STMtx.msg
  dynamixel_control.msg
  dynamixel_status.msg
  manipulator_control.msg
  manipulator_status.msg
  mani_vision.msg
)

# Generate messages
generate_messages(
  DEPENDENCIES 
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS 
    message_runtime  
    geometry_msgs
    sensor_msgs
    std_msgs
)

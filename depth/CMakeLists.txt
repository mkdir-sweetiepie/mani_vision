cmake_minimum_required(VERSION 3.0.2)
project(depth)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
sensor_msgs
image_transport
cv_bridge
geometry_msgs
mobile_base_msgs
)

find_package(OpenCV 4.5.2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES depth
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
set (OpenCV_INClUDE_DIRS /usr/local/include /usr/local/include/opencv2)
set(OpenCV_LIB_DIR /usr/local/lib)

add_library(depth src/depth.cpp)
add_executable(${PROJECT_NAME}_node src/depth.cpp)
target_link_libraries(${PROJECT_NAME}_node
${catkin_LIBRARIES}
realsense2
)
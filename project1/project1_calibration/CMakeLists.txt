cmake_minimum_required(VERSION 3.0.2)
project(project1_calibration)

## Find catkin and any catkin packages
find_package(catkin  REQUIRED COMPONENTS 
roscpp 
rospy
std_msgs 
tf2
message_generation
message_filters
)

# Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  LIBRARIES project1_calibration
  CATKIN_DEPENDS roscpp rospy std_msgs message_filters
  DEPENDS system_lib
)

add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration ${catkin_LIBRARIES})
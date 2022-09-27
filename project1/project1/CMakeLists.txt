cmake_minimum_required(VERSION 3.0.2)
project(project1)

## Find catkin and any catkin packages
find_package(catkin  REQUIRED COMPONENTS 
roscpp 
rospy
std_msgs 
tf2 
tf2_ros
message_generation
dynamic_reconfigure
)

# Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  WheelSpeed.msg
)

add_service_files(
  FILES
  SetPose.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/dynamic_rec.cfg
)

catkin_package(
  LIBRARIES project1
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS system_lib
)

add_executable(fw_omnidirectional_robot_odometry src/fw_omnidirectional_robot_odometry.cpp)
target_link_libraries(fw_omnidirectional_robot_odometry ${catkin_LIBRARIES})
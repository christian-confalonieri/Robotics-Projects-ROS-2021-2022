cmake_minimum_required(VERSION 3.0.2)
project(project2)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  std_msgs
  sensor_msgs
  roslib
  roscpp
  tf
  message_generation
)

add_service_files(
  FILES
  PrintTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  nav_msgs
  std_msgs
  sensor_msgs
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${OpenCV_INCLUDE_DIRS}
)

add_executable(odom_tf
  src/odom_tf.cpp
)
add_dependencies(odom_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  odom_tf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)


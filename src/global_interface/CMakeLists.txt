cmake_minimum_required(VERSION 3.16.0)
project(global_interface)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs message_generation roscpp rospy sensor_msgs std_msgs
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(remote_script_executor src/remote_script_executor.cpp)
target_link_libraries(remote_script_executor ${catkin_LIBRARIES})
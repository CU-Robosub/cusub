cmake_minimum_required(VERSION 3.0.2)
project(ros_dvl)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  geometry_msgs
  nav_msgs
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

cmake_minimum_required(VERSION 2.8.3)
project(detection_tree)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  darknet_ros_msgs
  geometry_msgs
  roscpp
  std_msgs
)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

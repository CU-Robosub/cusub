cmake_minimum_required(VERSION 2.8.3)
project(downcam_respawner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  sensor_msgs
)

catkin_package(
  LIBRARIES downcam_respawner_node
  CATKIN_DEPENDS nodelet roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library( downcam_respawner_node
src/respawner.cpp
)
target_link_libraries(downcam_respawner_node ${catkin_LIBRARIES})
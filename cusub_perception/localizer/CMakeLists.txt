cmake_minimum_required(VERSION 2.8.3)
project(localizer)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  darknet_ros_msgs
)

catkin_package(
  LIBRARIES localizer
  CATKIN_DEPENDS cv_bridge image_transport nodelet roscpp sensor_msgs std_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(localizer
  src/localizer.cpp
  src/start_gate_watershed.cpp
)
target_link_libraries(localizer ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(localizer ${catkin_EXPORTED_LIBRARIES})
endif()

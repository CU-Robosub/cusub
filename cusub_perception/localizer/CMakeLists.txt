cmake_minimum_required(VERSION 2.8.3)
project(localizer)

set(OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  darknet_ros_msgs
  OpenCV
  cv_bridge
  image_transport
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

add_executable(sg_2019 src/start_gate_watershed.cpp)
target_link_libraries(sg_2019 ${catkin_LIBRARIES})

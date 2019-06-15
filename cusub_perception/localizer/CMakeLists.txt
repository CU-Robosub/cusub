cmake_minimum_required(VERSION 2.8.3)
project(localizer)

set(OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  OpenCV
  darknet_ros_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
  geometry_msgs
)

add_message_files(
  FILES
  Detection.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  LIBRARIES localizer
  DEPENDS message_runtime geometry_msgs
  CATKIN_DEPENDS cv_bridge image_transport nodelet roscpp sensor_msgs std_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(localizer
  src/localizer.cpp
  src/start_gate_hough.cpp
)
target_link_libraries(localizer ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(localizer ${catkin_EXPORTED_LIBRARIES})
endif()

add_executable(sg_2019 src/start_gate_hough.cpp)
target_link_libraries(sg_2019 ${catkin_LIBRARIES})

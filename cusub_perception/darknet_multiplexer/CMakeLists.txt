cmake_minimum_required(VERSION 2.8.3)
project(darknet_multiplexer)

set(OpenCV_DIR /usr/share/OpenCV)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  sensor_msgs
  message_generation
  std_msgs
  OpenCV
  cv_bridge
  darknet_ros_msgs
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   DarknetCameras.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES darknet_multiplexer
   CATKIN_DEPENDS nodelet roscpp sensor_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library( multiplexer
  src/multiplexer.cpp
)
target_link_libraries(multiplexer ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(multiplexer ${catkin_EXPORTED_LIBRARIES})
endif()
add_dependencies(multiplexer darknet_multiplexer_generate_messages_cpp)

add_library( drawer
  src/drawer.cpp
)
target_link_libraries(drawer ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(drawer ${catkin_EXPORTED_LIBRARIES})
endif()

# Multiplexer Node (no nodelet)
add_executable( multiplexer_node
  src/multiplexer.cpp
)
add_dependencies(multiplexer_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(multiplexer_node ${catkin_LIBRARIES})
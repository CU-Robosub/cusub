cmake_minimum_required(VERSION 2.8.3)
project(detection_tree)

add_compile_options(-std=c++11)
set(OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  darknet_ros_msgs
  geometry_msgs
  roscpp
  std_msgs
  nodelet
  tf
  sensor_msgs
  OpenCV
  cv_bridge
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Dvector.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  LIBRARIES detection_tree_node
  CATKIN_DEPENDS nodelet roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library( detection_tree_node 
src/detection_tree_node.cpp )
add_dependencies(detection_tree_node detection_tree_generate_messages_cpp)
target_link_libraries(detection_tree_node ${catkin_LIBRARIES})

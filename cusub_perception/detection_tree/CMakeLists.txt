cmake_minimum_required(VERSION 2.8.3)
project(detection_tree)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  darknet_ros_msgs
  geometry_msgs
  roscpp
  std_msgs
  nodelet
  tf
)
catkin_package(
  LIBRARIES detection_tree_node
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library( detection_tree_node 
src/detection_tree_node )
target_link_libraries(detection_tree_node ${catkin_LIBRARIES})

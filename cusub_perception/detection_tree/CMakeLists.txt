cmake_minimum_required(VERSION 2.8.3)
project(detection_tree)

find_package(catkin REQUIRED COMPONENTS
  darknet_ros_msgs
  rospy
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

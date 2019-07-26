cmake_minimum_required(VERSION 2.8.3)
project(perception_control)
add_compile_options(-std=c++11)
set(OpenCV_DIR /usr/share/OpenCV)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  OpenCV
  actionlib
  actionlib_msgs
  darknet_ros_msgs
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  nodelet
  waypoint_navigator
  cv_bridge
)

add_action_files(
  DIRECTORY action 
  FILES 
    OrbitBuoy.action 
    VisualServo.action 
    PathOrient.action
)

# ORBIT SERVER
include_directories(include ${catkin_INCLUDE_DIRS})
add_library( orbit
  src/orbit.cpp
)
add_dependencies(orbit perception_control_generate_messages_cpp)
target_link_libraries(orbit ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(orbit ${catkin_EXPORTED_LIBRARIES} ${PROJECT_NAME}_EXPORTED_TARGETS)
endif()


## VISUAL SERVO
add_library( visual_servo
  src/visual_servo.cpp
  src/bb_controller.cpp
  src/bb_proportional.cpp
)
add_dependencies(visual_servo perception_control_generate_messages_cpp)
target_link_libraries(visual_servo ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(visual_servo ${catkin_EXPORTED_LIBRARIES} ${PROJECT_NAME}_EXPORTED_TARGETS)
endif()


# PATH ORIENTATION
add_library( path_orient
  src/path_orient.cpp
)
add_dependencies(path_orient perception_control_generate_messages_cpp)
target_link_libraries(path_orient ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(path_orient ${catkin_EXPORTED_LIBRARIES} ${PROJECT_NAME}_EXPORTED_TARGETS)
endif()

add_executable( path_unit 
  unit_tests/path_orientation_test.cpp
  src/path_orient.cpp
)
add_dependencies(path_unit perception_control_generate_messages_cpp)
target_link_libraries(path_unit ${catkin_LIBRARIES})


generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception_control
  CATKIN_DEPENDS actionlib_msgs darknet_ros_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  # include 
  ${catkin_INCLUDE_DIRS}
)
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

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   darknet_ros_msgs#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

add_action_files(DIRECTORY action FILES OrbitBuoy.action VisualServo.action PathOrient.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

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

# ORBIT SERVER
include_directories(include ${catkin_INCLUDE_DIRS})
add_library( orbit
  src/orbit.cpp
)
target_link_libraries(orbit ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(orbit ${catkin_EXPORTED_LIBRARIES})
endif()


## VISUAL SERVO
add_library( visual_servo
  src/visual_servo.cpp
  src/bb_controller.cpp
  src/bb_proportional.cpp
)
target_link_libraries(visual_servo ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(visual_servo ${catkin_EXPORTED_LIBRARIES})
endif()


# PATH ORIENTATION
add_library( path_orient
  src/path_orient.cpp
)
target_link_libraries(path_orient ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(path_orient ${catkin_EXPORTED_LIBRARIES})
endif()

add_executable( path_unit 
  unit_tests/path_orientation_test.cpp
  src/path_orient.cpp
)
target_link_libraries(path_unit ${catkin_LIBRARIES})
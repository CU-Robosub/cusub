cmake_minimum_required(VERSION 2.8.3)
project(localizer_msgs)

find_package(catkin REQUIRED COMPONENTS
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
  CATKIN_DEPENDS message_runtime geometry_msgs
)
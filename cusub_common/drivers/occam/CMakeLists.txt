cmake_minimum_required(VERSION 2.8.3)
project(ros_indigosdk)
find_package(catkin 
	REQUIRED COMPONENTS 
		image_transport 
		roscpp 
		std_msgs 
		sensor_msgs 
        camera_info_manager)


set(ROS_BUILD_TYPE Release)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_LIBUSB libusb-1.0)
include_directories(${PC_LIBUSB_INCLUDE_DIRS})
link_directories(${PC_LIBUSB_LIBDIR})

catkin_package(
	INCLUDE_DIRS
	CATKIN_DEPENDS
		image_transport 
		roscpp 
		std_msgs 
		sensor_msgs
        camera_info_manager
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -Wno-format-extra-args")

set(INDIGOSDK_PATH ${PROJECT_SOURCE_DIR}/sdk)

if (NOT INDIGOSDK_PATH)
  message(FATAL_ERROR "You must specify the location of the corresponding indigosdk using cmake option -DINDIGOSDK_PATH=<path>")
endif()

include_directories(
	${catkin_INCLUDE_DIRS}
	${INDIGOSDK_PATH}/include
)
add_subdirectory(${INDIGOSDK_PATH} indigosdk)

add_executable(ros_indigosdk_node nodes/ros_indigosdk_node.cpp)
target_link_libraries(ros_indigosdk_node indigo ${catkin_LIBRARIES})

install(TARGETS ros_indigosdk_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

cmake_minimum_required(VERSION 2.8)
project(indigosdk)

set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/bin")

if (WIN32)
  set(MSVC_FLAGS "/FC /wd4018 /wd4996 /wd4200 /MP8")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MSVC_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MSVC_FLAGS}")
endif()
if (UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -fPIC")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()

if (WIN32)
  add_subdirectory(third/libusb)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third/libusb/libusb)

  add_subdirectory(third/cyusb)
  set(USE_CYUSB 1)
  add_definitions(-DHAVE_CYUSB)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third/cyusb)
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -DNDEBUG")

  find_package(PkgConfig)
  pkg_check_modules(PC_LIBUSB libusb-1.0)
  include_directories(${PC_LIBUSB_INCLUDE_DIRS})
  link_directories(${PC_LIBUSB_LIBDIR})
endif()

option(OCCAM_SSE2 "SSE2 Enabled" ON)
if (OCCAM_SSE2)
  add_definitions(-DOCCAM_SSE2)
  if (UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
  endif()
endif()

option(USE_OPENGL "Include support for OpenGL backend" OFF)
option(USE_OPENCV "Use OpenCV" OFF)
if (MSVC)
    option(INDIGOSDK_MSVC_STATIC_CRT "Use MSVC runtime library DLL" ON)
endif()

if (USE_OPENGL)
  add_definitions(-DGLEW_STATIC -DOCCAM_OPENGL_SUPPORT)
  set(GLFW_BUILD_EXAMPLES OFF)
  set(GLFW_BUILD_TESTS OFF)
  set(GLFW_BUILD_DOCS OFF)
  set(GLFW_INSTALL OFF)
  if (INDIGOSDK_MSVC_STATIC_CRT)
    set(USE_MSVC_RUNTIME_LIBRARY_DLL OFF)
  endif()
  add_subdirectory(third/glfw)
  add_subdirectory(third/glew)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third/glfw/include)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third/glew)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third/glfw/include/GLFW)
  find_package(OpenGL)
endif()

option(DEVICE_OMNI5U3MT9V022 "Build support for omni5u3mt9v022" ON)
option(DEVICE_OMNIS5U3MT9V022 "Build support for omnis5u3mt9v022" ON)
option(DEVICE_TGMT9V02 "Build support for tgmt9v022" OFF)
option(DEVICE_BU3AR0330 "Build support for bu3ar0330" OFF)

if(USE_OPENCV)
  include(FindOpenCV.cmake)
  message("OpenCV include path: ${OpenCV_INCLUDE_DIRS}")
  include_directories(${OpenCV_INCLUDE_DIRS} ${OpenCV_DIR})
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(indigo_srcs
src/bm_stereo.cc
src/cylinder_blend.cc
src/debayer_filter.cc
src/device_data_cache.cc
src/device_enum.cc
src/device_iface.cc
src/gl_utils.cc
src/image.cc
src/image_collect.cc
src/image_filter.cc
src/indigo.cc
src/libusb_utils.cc
src/markers.cc
src/module.cc
src/module_utils.cc
src/offset_blend.cc
src/omni_libusb.cc
src/planar_rectify.cc
src/point_cloud.cc
src/rate_utils.cc
src/system.cc
src/undistort_filter.cc
src/remap.cc
)

if (DEVICE_OMNI5U3MT9V022)
  message("omni5u3mt9v022 configured")
  add_definitions(-DDEVICE_OMNI5U3MT9V022) # move these to config-written indigo_config.h etc.
  set(indigo_srcs ${indigo_srcs} src/omni5u3mt9v022.cc)
endif()
if (DEVICE_OMNIS5U3MT9V022)
  message("omnis5u3mt9v022 configured")
  add_definitions(-DDEVICE_OMNIS5U3MT9V022)
  set(indigo_srcs ${indigo_srcs} src/omnis5u3mt9v022.cc)
endif()
if (DEVICE_TGMT9V02)
  message("tgmt9v022 configured")
  add_definitions(-DDEVICE_TGMT9V02)
  set(indigo_srcs ${indigo_srcs} src/tgmt9v022.cc)
endif()
if (DEVICE_BU3AR0330)
  message("bu3ar0330 configured")
  add_definitions(-DDEVICE_BU3AR0330)
  set(indigo_srcs ${indigo_srcs} src/bu3ar0330.cc)
endif()

add_library(indigo SHARED ${indigo_srcs})
set_target_properties(indigo PROPERTIES COMPILE_FLAGS -DOCCAM_SDK_BUILD=1)

if (UNIX)
  target_link_libraries(indigo ${PC_LIBUSB_LIBRARIES})
endif()
if (WIN32)
  target_link_libraries(indigo libusb109)
  if (INDIGOSDK_MSVC_STATIC_CRT)
    target_compile_options(indigo PRIVATE "/MT$<$<CONFIG:Debug>:d>")
  endif()
endif()
if (USE_OPENGL)
  target_link_libraries(indigo glfw glew ${OPENGL_LIBRARIES})
  if (UNIX)
    target_link_libraries(indigo Xrandr X11 Xi Xxf86vm)
  endif()
endif()
if(USE_OPENCV)
  target_link_libraries(indigo ${OpenCV_LIBS})
endif()
if(USE_CYUSB)
  target_link_libraries(indigo cyusb)
  target_link_libraries(indigo setupapi)
endif()

add_executable(read_images examples/read_images.c)
target_link_libraries(read_images indigo)

add_executable(read_images_nonblock examples/read_images_nonblock.c)
target_link_libraries(read_images_nonblock indigo)

add_executable(read_images_fps examples/read_images_fps.c)
target_link_libraries(read_images_fps indigo)

add_executable(read_stitched_images examples/read_stitched_images.c)
target_link_libraries(read_stitched_images indigo)

add_executable(read_tiled_images examples/read_tiled_images.c)
target_link_libraries(read_tiled_images indigo)

add_executable(read_raw_images examples/read_raw_images.c)
target_link_libraries(read_raw_images indigo)

if(UNIX)
  add_executable(emit_raw examples/emit_raw.c)
  target_link_libraries(emit_raw indigo)
endif()

add_executable(read_calib examples/read_calib.cc)
target_link_libraries(read_calib indigo)

if(USE_OPENCV)
  add_executable(read_images_opencv examples/read_images_opencv.cc)
  target_link_libraries(read_images_opencv indigo ${OpenCV_LIBS})

  add_executable(record_raw examples/record_raw.cc)
  target_link_libraries(record_raw indigo ${OpenCV_LIBS})

  add_executable(process_raw examples/process_raw.cc)
  target_link_libraries(process_raw indigo ${OpenCV_LIBS})

  add_executable(read_images_periodic examples/read_images_periodic.cc)
  target_link_libraries(read_images_periodic indigo ${OpenCV_LIBS})

  add_executable(offline_stitch examples/offline_stitch.cc)
  target_link_libraries(offline_stitch indigo ${OpenCV_LIBS})

  if (WIN32)
    add_executable(imgseq_video examples/imgseq_video.cc)
    target_link_libraries(imgseq_video indigo ${OpenCV_LIBS})
  endif()
endif()

if (WIN32)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libusb
    ${CMAKE_CURRENT_SOURCE_DIR}/libusb/os
    ${CMAKE_CURRENT_SOURCE_DIR}/msvc)

  #add_definitions(-DENABLE_DEBUG_LOGGING=1) # * debug logic

  add_library(libusb109 STATIC
    libusb/core.c
    libusb/descriptor.c
    libusb/io.c
    libusb/sync.c
    libusb/os/poll_windows.c
    libusb/os/threads_windows.c
    libusb/os/windows_usb.c
    )
  if (INDIGOSDK_MSVC_STATIC_CRT)
    target_compile_options(libusb109 PRIVATE "/MT$<$<CONFIG:Debug>:d>")
  endif()
endif()

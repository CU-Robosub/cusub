add_library(glew STATIC
glew.c
glewinfo.c
)
if (WIN32)
  if (INDIGOSDK_MSVC_STATIC_CRT)
    target_compile_options(glew PRIVATE "/MT$<$<CONFIG:Debug>:d>")
  endif()
endif()

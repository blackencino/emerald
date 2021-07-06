if (EMERALD_BUILD_GUIS)
emerald_define_library(geep_glfw
  HEADERS
    foundation.h
    program.h
    util_gl.h
  SOURCES
    program.cpp
    util_gl.cpp
  DEPENDENCIES
    emerald::util
    OpenEXR::OpenEXR
    glfw::glfw
    OpenGL::GL
    glad::glad
)

if (MSVC)
target_compile_options(geep_glfw PUBLIC /wd4005)
endif()

add_subdirectory(tests)
endif()


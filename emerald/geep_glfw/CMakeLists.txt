emerald_define_library(geep_glfw
  SOURCES
    program.cpp
    util_gl.cpp
  HEADERS
    foundation.h
    program.h
    util_gl.h
  DEPENDENCIES
    emerald::util
    OpenEXR::OpenEXR
    glfw::glfw
    OpenGL::GL
    glad::glad
)

add_subdirectory(tests)


add_library(emerald_geep_glfw
    foundation.h
    program.h
    util_gl.h

    program.cpp
    util_gl.cpp
)

target_include_directories(emerald_geep_glfw PUBLIC "${emerald_SOURCE_DIR}")
target_link_libraries(emerald_geep_glfw PUBLIC
                      emerald_util
                      OpenEXR::OpenEXR
                      glfw::glfw)

add_subdirectory(tests)


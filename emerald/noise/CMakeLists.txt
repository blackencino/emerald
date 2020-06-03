add_library(emerald_noise
    all.h
    cell_noise.h
    fbm.h
    foundation.h
    gritzy_noises.h
    pattern_gradient.h
    simplex_noise.h

    cell_noise.cpp
    simplex_noise.cpp)

target_include_directories(emerald_noise PUBLIC "${emerald_SOURCE_DIR}")
target_link_libraries(emerald_noise PUBLIC emerald_util OpenEXR::OpenEXR)

add_subdirectory(tests)

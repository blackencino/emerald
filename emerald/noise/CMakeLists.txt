emerald_define_library(noise
  SOURCES
    cell_noise.cpp
    simplex_noise.cpp
  HEADERS
    all.h
    cell_noise.h
    fbm.h
    foundation.h
    gritzy_noises.h
    pattern_gradient.h
    simplex_noise.h
  DEPENDENCIES
    emerald::util
    OpenEXR::OpenEXR
  )

add_subdirectory(tests)

emerald_define_library(sph_common
  HEADERS
    cfl.h
    common.h
    density.h
    divergence.h
    dynamics.h
    kernels.h
    neighborhood.h
    pressure.h
    types.h
    volume.h
  SOURCES
    cfl.cpp
    common.cpp
    density.cpp
    divergence.cpp
    dynamics.cpp
    neighborhood.cpp
    pressure.cpp
    volume.cpp
  DEPENDENCIES
    emerald::util
    emerald::z_index
    OpenEXR::OpenEXR
    TBB::TBB
    fmt::fmt
  )


##------------------------------------------------------------------------------
# TESTS
foreach(_test
        common
        dynamics
        neighborhood
        performance)
    emerald_define_gtest(sph_common_test_${_test}
        SOURCES tests/test_${_test}.cpp
        DEPENDENCIES emerald::sph_common)
endforeach()


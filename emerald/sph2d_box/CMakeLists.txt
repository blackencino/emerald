emerald_define_library(sph2d_box
  HEADERS
    colors.h
    config.h
    dfsph_p.h
    dfsph_p_ops.h
    forces.h
    iisph.h
    iisph_ap.h
    iisph_ap_ops.h
    iisph_ops.h
    iisph_pseudo_ap.h
    iisph_pseudo_ap_ops.h
    initial_state.h
    neighborhoods_from_state.h
    parameters.h
    pcisph.h
    pcisph_ops.h
    simulation.h
    solids.h
    state.h
  SOURCES
    colors.cpp
    config.cpp
    dfsph_p.cpp
    dfsph_p_ops.cpp
    forces.cpp
    iisph.cpp
    iisph_ap.cpp
    iisph_ap_ops.cpp
    iisph_ops.cpp
    iisph_pseudo_ap.cpp
    iisph_pseudo_ap_ops.cpp
    initial_state.cpp
    neighborhoods_from_state.cpp
    parameters.cpp
    pcisph.cpp
    pcisph_ops.cpp
    simulation.cpp
    solids.cpp
  DEPENDENCIES
    emerald::sph_common
    emerald::util
    emerald::z_index
    OpenEXR::OpenEXR
    TBB::TBB
    fmt::fmt
    cxxopts::cxxopts
  )

emerald_define_executable(sph2d_box_headless
    SOURCES 
        bin/headless_main.cpp
    DEPENDENCIES 
        emerald::sph2d_box
        fmt::fmt
    )

if (EMERALD_BUILD_GUIS)
    emerald_define_executable(sph2d_box_gui
        SOURCES
            bin/main.cpp
            bin/multi_scale_draw.h
            bin/multi_scale_draw.cpp
        DEPENDENCIES 
            emerald::sph2d_box
            emerald::simple_sim_viewer 
            fmt::fmt
        )
endif()

##------------------------------------------------------------------------------
# TESTS
foreach(_test
        parallel_scan
        repeatability)
    emerald_define_gtest(sph2d_box_test_${_test}
        SOURCES tests/test_${_test}.cpp
        DEPENDENCIES emerald::sph2d_box)
endforeach()




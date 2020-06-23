add_library(emerald_sph2d_box

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
            )
target_include_directories(emerald_sph2d_box PUBLIC "${emerald_SOURCE_DIR}")
target_link_libraries(emerald_sph2d_box
                      PUBLIC
                      emerald_util
                      emerald_sph_common
                      cxxopts::cxxopts
                      OpenEXR::OpenEXR
                      TBB::TBB
                      fmt::fmt)

add_executable(emerald_sph2d_box_headless
               bin/headless_main.cpp)
target_link_libraries(emerald_sph2d_box_headless
                      PUBLIC
                      emerald_sph2d_box
                      fmt::fmt)

if (${EMERALD_BUILD_GUIS})
add_executable(emerald_sph2d_box_main
               bin/main.cpp
               bin/multi_scale_draw.h
               bin/multi_scale_draw.cpp)
target_link_libraries(emerald_sph2d_box_main
                      PUBLIC
                      emerald_sph2d_box
                      emerald_simple_sim_viewer
                      fmt::fmt)
endif()

#------------------------------------------------------------------------------
# TESTS
foreach(_test
        parallel_scan
        repeatability)
    set(_target test_${_test})
    add_executable(${_target} "tests/${_target}.cpp")
    target_link_libraries(${_target}
                          PUBLIC
                          emerald_sph2d_box
                          GTest::GTest)
    add_test(TEST_emerald_sph2d_box_${_test} ${_target})
endforeach()


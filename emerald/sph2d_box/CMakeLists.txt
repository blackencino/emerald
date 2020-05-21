add_library(emerald_sph2d_box
            parameters.h
            sim_ops.h
            simulation.h
            solids.h
            state.h
            tag.h

            parameters.cpp
            sim_ops.cpp
            simulation.cpp
            solids.cpp)
target_include_directories(emerald_sph2d_box PUBLIC "${emerald_SOURCE_DIR}")
target_link_libraries(emerald_sph2d_box
                      PUBLIC
                      emerald_util
                      emerald_sph_common
                      cxxopts::cxxopts
                      openexr::openexr
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

##------------------------------------------------------------------------------
# TESTS
foreach(_test
        parallel_scan
        sim_ops
        simulation)
    set(_target test_${_test})
    add_executable(${_target} "tests/${_target}.cpp")
    target_link_libraries(${_target}
                          PUBLIC
                          emerald_sph2d_box
                          GTest::GTest)
    add_test(TEST_emerald_sph2d_box_${_test} ${_target})
endforeach()

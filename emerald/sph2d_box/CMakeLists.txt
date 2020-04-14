add_library(emerald_sph2d_box
            foundation.h
            sim_ops.h

            sim_ops.cpp)
target_include_directories(emerald_sph2d_box PUBLIC "${emerald_SOURCE_DIR}")
target_link_libraries(emerald_sph2d_box
                      PUBLIC
                      emerald_util
                      emerald_z_index
                      openexr::openexr
                      TBB::TBB
                      fmt::fmt)

##------------------------------------------------------------------------------
# TESTS
foreach(_test
        parallel_scan
        sim_ops)
    set(_target test_${_test})
    add_executable(${_target} "tests/${_target}.cpp")
    target_link_libraries(${_target}
                          PUBLIC
                          emerald_sph2d_box
                          GTest::GTest)
    add_test(TEST_emerald_sph2d_box_${_test} ${_target})
endforeach()

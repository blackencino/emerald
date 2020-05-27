add_library(emerald_sph_common
            common.h
            dynamics.h
            kernels.h
            neighborhood.h
            types.h

            common.cpp
            dynamics.cpp
            neighborhood.cpp)
target_include_directories(emerald_sph_common PUBLIC "${emerald_SOURCE_DIR}")
target_link_libraries(emerald_sph_common
                      PUBLIC
                      emerald_util
                      emerald_z_index
                      openexr::openexr
                      TBB::TBB
                      fmt::fmt)

##------------------------------------------------------------------------------
# TESTS
foreach(_test
        common
        dynamics
        neighborhood
        performance)
    set(_target test_${_test})
    add_executable(${_target} "tests/${_target}.cpp")
    target_link_libraries(${_target}
                          PUBLIC
                          emerald_sph_common
                          GTest::GTest)
    add_test(TEST_emerald_sph_common_${_test} ${_target})
endforeach()

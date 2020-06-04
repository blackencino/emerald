add_library(emerald_sph_common
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

            cfl.cpp
            common.cpp
            density.cpp
            divergence.cpp
            dynamics.cpp
            neighborhood.cpp
            pressure.cpp
            volume.cpp)
target_include_directories(emerald_sph_common PUBLIC "${emerald_SOURCE_DIR}")
target_link_libraries(emerald_sph_common
                      PUBLIC
                      emerald_util
                      emerald_z_index
                      OpenEXR::OpenEXR
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

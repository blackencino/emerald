add_library(emerald_z_index z_index.h z_index.cpp)
target_include_directories(emerald_z_index PUBLIC "${emerald_SOURCE_DIR}")
target_link_libraries(emerald_z_index PUBLIC fmt::fmt)

##------------------------------------------------------------------------------
# TESTS
foreach(_test
        z_index)
    set(_target test_${_test})
    add_executable(${_target} "tests/${_target}.cpp")
    target_link_libraries(${_target}
                          PUBLIC
                          emerald_z_index GTest::GTest)
    add_test(TEST_emerald_z_index_${_test} ${_target})
endforeach()

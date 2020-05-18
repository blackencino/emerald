
##------------------------------------------------------------------------------
# TESTS
foreach(_test
        functions
        safe_divide
        vector_util)
    set(_target test_${_test})
    add_executable(${_target} "${_target}.cpp")
    target_link_libraries(${_target}
                          PUBLIC
                          emerald_util
                          GTest::GTest
                          fmt::fmt)
    add_test(TEST_emerald_util_${_test} ${_target})
endforeach()

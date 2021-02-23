
##------------------------------------------------------------------------------
# TESTS
foreach(_test
        functions
        safe_divide
        vector_util)
    emerald_define_gtest(util_test_${_test}
        SOURCES test_${_test}.cpp
        DEPENDENCIES emerald::util fmt::fmt)
endforeach()

##------------------------------------------------------------------------------
# TESTS
foreach(_test
        noise
        cell_noise)
    emerald_define_gtest(noise_test_${_test}
        SOURCES test_${_test}.cpp
        DEPENDENCIES 
            emerald::noise 
            emerald::util 
            fmt::fmt
        )
endforeach()


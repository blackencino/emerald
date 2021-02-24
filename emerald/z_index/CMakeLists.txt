emerald_define_library(z_index
  HEADERS
    z_index.h
  SOURCES
    z_index.cpp
  DEPENDENCIES
    fmt::fmt
  )

##------------------------------------------------------------------------------
# TESTS
foreach(_test
        z_index)
    emerald_define_gtest(z_index_test_${_test}
        SOURCES tests/test_${_test}.cpp
        DEPENDENCIES emerald::z_index)
endforeach()

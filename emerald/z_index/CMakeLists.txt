add_library(emerald_z_index z_index.h z_index.cpp)
target_include_directories(emerald_z_index PUBLIC "${emerald_SOURCE_DIR}")
target_link_libraries(emerald_z_index PUBLIC fmt::fmt)

emerald_define_library(z_index
  SOURCES
    z_index.cpp
  HEADERS
    z_index.h
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

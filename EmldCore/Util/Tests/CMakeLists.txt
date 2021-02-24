emerald_define_test(EmldCore_Util_ExceptionTest
    SOURCES ExceptionTest.cpp
    DEPENDENCIES emerald::EmldCore_Util)

emerald_define_test(EmldCore_Util_Cpp11Test
    SOURCES test_Cpp11.cpp
    DEPENDENCIES emerald::EmldCore_Util)

emerald_define_test(EmldCore_Util_FunctionsTest
    SOURCES test_Functions.cpp
    DEPENDENCIES emerald::EmldCore_Util)

emerald_define_test(EmldCore_Util_VectorUtilTest
    SOURCES VectorUtilTest.cpp
    DEPENDENCIES emerald::EmldCore_Util)


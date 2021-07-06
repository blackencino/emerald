function(emerald_define_library libname)
    set(options)
    set(one_value_args)
    set(multi_value_args SOURCES HEADERS DEPENDENCIES PRIVATE_DEPS)
    cmake_parse_arguments(EMERALD_CURLIB 
                          "${options}"
                          "${one_value_args}"
                          "${multi_value_args}"
                          ${ARGN})
    add_library(${libname}
        ${EMERALD_CURLIB_HEADERS}
        ${EMERALD_CURLIB_SOURCES})

    target_compile_features(${libname} PUBLIC cxx_std_17)
    # target_compile_options(${libname} PRIVATE
    #     $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
    #     $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -pedantic -Werror>)

    set_target_properties(${libname} PROPERTIES 
        CXX_STANDARD_REQUIRED ON 
        CXX_EXTENSIONS OFF
        POSITION_INDEPENDENT_CODE ON
    )
    set_property(TARGET ${libname} 
        PROPERTY PUBLIC_HEADER
        ${EMERALD_CURLIB_HEADERS})
    if (EMERALD_CURLIB_DEPENDENCIES)
        target_link_libraries(${libname} PUBLIC ${EMERALD_CURLIB_DEPENDENCIES})
    endif()
    if (EMERALD_CURLIB_PRIVATE_DEPS)
        target_link_libraries(${libname} PRIVATE ${EMERALD_CURLIB_PRIVATE_DEPS})
    endif()
    target_include_directories(${libname} 
        PUBLIC
         $<BUILD_INTERFACE:${emerald_SOURCE_DIR}>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )

    add_library(emerald::${libname} ALIAS ${libname})

    install(TARGETS ${libname}
        EXPORT emerald
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include/emerald/${libname}
    )
endfunction()

function(emerald_define_executable binname)
    set(options)
    set(one_value_args)
    set(multi_value_args SOURCES DEPENDENCIES)
    cmake_parse_arguments(EMERALD_CURLIB 
                          "${options}"
                          "${one_value_args}"
                          "${multi_value_args}"
                          ${ARGN})
    add_executable(${binname}
        ${EMERALD_CURLIB_SOURCES})

    target_compile_features(${binname} PUBLIC cxx_std_17)
    # target_compile_options(${binname} PRIVATE
    #     $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
    #     $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -pedantic -Werror>)
    set_target_properties(${binname} PROPERTIES 
        CXX_STANDARD_REQUIRED ON 
        CXX_EXTENSIONS OFF
        POSITION_INDEPENDENT_CODE ON
    )
    if (EMERALD_CURLIB_DEPENDENCIES)
        target_link_libraries(${binname} PUBLIC ${EMERALD_CURLIB_DEPENDENCIES})
    endif()
    target_include_directories(${binname} 
        PUBLIC
         $<BUILD_INTERFACE:${emerald_SOURCE_DIR}>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )

    add_executable(emerald::${binname} ALIAS ${binname})

    install(TARGETS ${binname}
        EXPORT emerald
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include/emerald/${binname}
    )
endfunction()

function(emerald_define_gtest testname)
    if (EMERALD_DO_TESTS)
        message(STATUS "Creating gtest: ${testname}")
        set(options)
        set(one_value_args)
        set(multi_value_args SOURCES DEPENDENCIES)
        cmake_parse_arguments(EMERALD_CURLIB 
                              "${options}"
                              "${one_value_args}"
                              "${multi_value_args}"
                              ${ARGN})
        add_executable(${testname} ${EMERALD_CURLIB_SOURCES})
        target_compile_features(${testname} PUBLIC cxx_std_17)
        # target_compile_options(${testname} PRIVATE
        #     $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
        #     $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -pedantic -Werror>)
        set_target_properties(${testname} PROPERTIES 
            CXX_STANDARD_REQUIRED ON 
            CXX_EXTENSIONS OFF
            POSITION_INDEPENDENT_CODE ON
        )
        target_link_libraries(${testname} PUBLIC GTest::GTest ${EMERALD_CURLIB_DEPENDENCIES})
        target_include_directories(${testname} 
            PUBLIC
             $<BUILD_INTERFACE:${emerald_SOURCE_DIR}>
             $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        )
        gtest_discover_tests(${testname})
    endif()
endfunction()


function(emerald_define_test testname)
    if (EMERALD_DO_TESTS)
        message(STATUS "Creating test: ${testname}")
        set(options)
        set(one_value_args)
        set(multi_value_args SOURCES DEPENDENCIES)
        cmake_parse_arguments(EMERALD_CURLIB 
                              "${options}"
                              "${one_value_args}"
                              "${multi_value_args}"
                              ${ARGN})
        add_executable(${testname} ${EMERALD_CURLIB_SOURCES})
        target_compile_features(${testname} PUBLIC cxx_std_17)
        # target_compile_options(${testname} PRIVATE
        #     $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
        #     $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -pedantic -Werror>)
        set_target_properties(${testname} PROPERTIES 
            CXX_STANDARD_REQUIRED ON 
            CXX_EXTENSIONS OFF
            POSITION_INDEPENDENT_CODE ON
        )
        if (EMERALD_CURLIB_DEPENDENCIES)
            target_link_libraries(${testname} PUBLIC ${EMERALD_CURLIB_DEPENDENCIES})
        endif()
        target_include_directories(${testname} 
            PUBLIC
             $<BUILD_INTERFACE:${emerald_SOURCE_DIR}>
             $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        )
        add_test(NAME TEST_${testname} 
            COMMAND ${testname}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    endif()
endfunction()

function(emldcore_legacy_define_library input_libname)
    set(libname "EmldCore_${input_libname}")
    set(options)
    set(one_value_args)
    set(multi_value_args SOURCES HEADERS DEPENDENCIES PRIVATE_DEPS)
    cmake_parse_arguments(EMERALD_CURLIB 
                          "${options}"
                          "${one_value_args}"
                          "${multi_value_args}"
                          ${ARGN})
    add_library(${libname}
        ${EMERALD_CURLIB_HEADERS}
        ${EMERALD_CURLIB_SOURCES})

    target_compile_features(${libname} PUBLIC cxx_std_17)
    # target_compile_options(${libname} PRIVATE
    #     $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
    #     $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -pedantic -Werror>)
    set_target_properties(${libname} PROPERTIES 
        CXX_STANDARD_REQUIRED ON 
        CXX_EXTENSIONS OFF
        POSITION_INDEPENDENT_CODE ON
    )
    set_property(TARGET ${libname} 
        PROPERTY PUBLIC_HEADER
        ${EMERALD_CURLIB_HEADERS})
    if (EMERALD_CURLIB_DEPENDENCIES)
        target_link_libraries(${libname} PUBLIC ${EMERALD_CURLIB_DEPENDENCIES})
    endif()
    if (EMERALD_CURLIB_PRIVATE_DEPS)
        target_link_libraries(${libname} PRIVATE ${EMERALD_CURLIB_PRIVATE_DEPS})
    endif()
    target_include_directories(${libname} 
        PUBLIC
         $<BUILD_INTERFACE:${emerald_SOURCE_DIR}>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )

    add_library(emerald::${libname} ALIAS ${libname})

    install(TARGETS ${libname}
        EXPORT emerald
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include/EmldCore/${input_libname}
    )
endfunction()

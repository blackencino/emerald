# CMakeLists files in this project can
# refer to the root source directory of the project as ${FURY_SOURCE_DIR} and
# to the root binary directory of the project as ${FURY_BINARY_DIR}.
cmake_minimum_required(VERSION 3.15)

cmake_policy(SET CMP0042 NEW)

project(emerald 
        LANGUAGES CXX
        VERSION 0.0.7)

option(EMERALD_BUILD_GUIS "Build Emerald Graphical UIs" ON)
option(EMERALD_DO_TESTS "Do unit tests" ON)
option(EMERALD_ENABLE_RPATH "Build with RPATH information" ON)

if (${EMERALD_BUILD_GUIS})
    add_definitions( "-DGL_SILENCE_DEPRECATION=1" )
    set(EMLD_CONAN_OPTIONS "enable_guis=True")
else()
    set(EMLD_CONAN_OPTIONS "enable_guis=False")
endif()

mark_as_advanced(EMERALD_ENABLE_RPATH)


if(EMERALD_ENABLE_RPATH)
  # Configure rpath for installation base on the following:
  # https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/RPATH-handling
  set(CMAKE_SKIP_BUILD_RPATH FALSE)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  # @todo make relocatable?
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()


message(STATUS "DO TESTS?: ${EMERALD_DO_TESTS}")

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    if (EMERALD_DO_TESTS)
        message(STATUS "I'm enabling testing")
        include(GoogleTest)
        enable_testing()
    endif()

    message(STATUS "about to do conan cmake run")
    include(cmake/conan.cmake)
    conan_cmake_run(CONANFILE conanfile.py
                    BASIC_SETUP
                    BUILD missing
                    OPTIONS ${EMLD_CONAN_OPTIONS})
    include(${CMAKE_BINARY_DIR}/conan_paths.cmake)
endif()

# Platform
set( WINDOWS FALSE )
set( DARWIN FALSE )
set( LINUX FALSE )
if( "${CMAKE_SYSTEM_NAME}" MATCHES "Windows" )
    set(WINDOWS TRUE)
    add_definitions("-DPLATFORM=WINDOWS -DPLATFORM_WINDOWS")
elseif( "${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
  set(DARWIN TRUE)
  add_definitions("-DPLATFORM_DARWIN -DPLATFORM=DARWIN")
elseif( "${CMAKE_SYSTEM_NAME}" MATCHES "Linux" )
  set( LINUX TRUE )
  add_definitions( "-DPLATFORM_LINUX -DPLATFORM=LINUX" )
endif()

# Packages
find_package(Alembic REQUIRED)
find_package(OpenEXR REQUIRED)
find_package(fmt REQUIRED)
find_package(TBB REQUIRED)
find_package(Boost REQUIRED)
find_package(gsl-lite REQUIRED)
find_package(cxxopts REQUIRED)
find_package(GTest REQUIRED)

set(EMERALD_GL_LIBS "")
if (${EMERALD_BUILD_GUIS})
    find_package(glfw3 REQUIRED)
    find_package(OpenGL REQUIRED)
    find_package(glad REQUIRED)
endif()

# This is necessary because of weird skew in the gsl-lite repo.
if(NOT TARGET gsl-lite::gsl-lite)
    add_library(gsl-lite::gsl-lite INTERFACE IMPORTED)
    set_target_properties(gsl-lite::gsl-lite PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                          "${gsl_gsl-lite_INCLUDE_DIRS}")
    set_target_properties(gsl-lite::gsl-lite PROPERTIES INTERFACE_LINK_DIRECTORIES
                          "${gsl_gsl-lite_LIB_DIRS}")
    set_target_properties(gsl-lite::gsl-lite PROPERTIES INTERFACE_LINK_LIBRARIES
                          "${gsl_gsl-lite_LINK_LIBS};${gsl_gsl-lite_LINKER_FLAGS_LIST}")
    set_target_properties(gsl-lite::gsl-lite PROPERTIES INTERFACE_COMPILE_DEFINITIONS
                          "${gsl_gsl-lite_COMPILE_DEFINITIONS}")
    set_target_properties(gsl-lite::gsl-lite PROPERTIES INTERFACE_COMPILE_OPTIONS
                          "${gsl_gsl-lite_COMPILE_OPTIONS_C};${gsl_gsl-lite_COMPILE_OPTIONS_CXX}")
endif()

include(cmake/emerald_utils.cmake)

add_subdirectory(emerald)
add_subdirectory(EmldCore)


# CMakeLists files in this project can
# refer to the root source directory of the project as ${FURY_SOURCE_DIR} and
# to the root binary directory of the project as ${FURY_BINARY_DIR}.
cmake_minimum_required(VERSION 3.11.2)

cmake_policy(SET CMP0042 NEW)

project(emerald CXX)

enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions( "-DGL_SILENCE_DEPRECATION=1" )

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
find_package(Alembic REQUIRED CONFIG)
find_package(openexr REQUIRED)
find_package(fmt REQUIRED)
find_package(TBB REQUIRED)
find_package(boost REQUIRED)
find_package(glfw REQUIRED)
find_package(fftw REQUIRED)
find_package(gsl-lite REQUIRED)
find_package(cxxopts REQUIRED)
#find_package(glm REQUIRED)

add_subdirectory(emerald)


#!/bin/bash

set -e

##
OS=`uname`
if [[ $OS == "Darwin" ]]; then
    BUILD_SUBDIR="Darwin"
else
    BUILD_SUBDIR="Linux"
fi

## Get directory this script was run from
CWD=`pwd`

## Make sure the depot dir variable is set
if [ -z "${DEPOT_DIR}" ]; then
    echo "ERROR: DEPOT_DIR is unset or set to the empty string"
    exit -1
fi
echo "DEPOT_DIR: ${DEPOT_DIR}"

## Make sure the build root variable is set
if [ -z "${DEPOT_BUILD_DIR}" ]; then
    if [ -d "${DEPOT_DIR}/build/${BUILD_SUBDIR}" ]; then
        echo "NOTICE!!! USING DEFAULT DEPOT_BUILD_DIR"
        DEPOT_BUILD_DIR="${DEPOT_DIR}/build/${BUILD_SUBDIR}"
    else
        echo "ERROR: DEPOT_BUILD_DIR is unset or set to the empty string"
        exit -1
    fi
fi
echo "DEPOT_BUILD_DIR: ${DEPOT_BUILD_DIR}"

## Make sure the install root variable is set
if [ -z "${DEPOT_INSTALL_DIR}" ]; then
    if [ -d "${DEPOT_DIR}/install/${BUILD_SUBDIR}" ]; then
        echo "NOTICE!!! USING DEFAULT DEPOT_INSTALL_DIR"
        DEPOT_INSTALL_DIR="${DEPOT_DIR}/install/${BUILD_SUBDIR}"
    else
        echo "ERROR: DEPOT_INSTALL_DIR is unset or set to the empty string"
        exit -1
    fi
fi
echo "DEPOT_INSTALL_DIR: ${DEPOT_INSTALL_DIR}"

## Make sure the module paths directory exists
DEPOT_CMAKE_PATHS_DIR="${DEPOT_BUILD_DIR}/cmake_paths"
if [ ! -d "${DEPOT_CMAKE_PATHS_DIR}" ]; then
    echo "ERROR: cmake_paths directory doesn't exist in ${DEPOT_BUILD_DIR}"
    exit -1
else
    echo "DEPOT_CMAKE_PATHS_DIR: ${DEPOT_CMAKE_PATHS_DIR}"
fi

## Make sure the conan paths file exists
DEPOT_CONAN_PATHS_CMAKE_FILE="${DEPOT_CMAKE_PATHS_DIR}/conan_paths.cmake"
if [ ! -f "${DEPOT_CONAN_PATHS_CMAKE_FILE}" ]; then
    echo "${DEPOT_CONAN_PATHS_CMAKE_FILE} does not exist"
else
    echo "DEPOT_CONAN_PATHS_CMAKE_FILE: ${DEPOT_CONAN_PATHS_CMAKE_FILE}"
fi

## Get the SRC dir no matter where it is
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  # if $SOURCE was a relative symlink, we need to resolve it relative to the path where
  # the symlink file was located
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
SRC_DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
if [[ $SRC_DIR == $CWD ]]; then
    echo "ERROR: CANNOT RUN SCRIPT FROM INSIDE SRC ROOT. CD TO BUILD ROOT"
    exit -1
fi

################################
# Set the build root
DEBUG_BUILD_DIR="${CWD}/${BUILD_SUBDIR}/Debug"
mkdir -p "${DEBUG_BUILD_DIR}"
echo "DEBUG_BUILD_DIR = ${DEBUG_BUILD_DIR}"

# We don't need to install, but just to prevent it from going into /usr/local,
DEBUG_INSTALL_DIR="${CWD}/install/${BUILD_SUBDIR}/Debug"

# Config
cd "${DEBUG_BUILD_DIR}"
cmake -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE="${DEPOT_CONAN_PATHS_CMAKE_FILE}" \
  -DCMAKE_PREFIX_PATH="${DEPOT_INSTALL_DIR}" \
  -DCMAKE_INSTALL_PREFIX="${DEBUG_INSTALL_DIR}" \
  -GNinja \
  "${SRC_DIR}"


################################
# Set the build root
RELEASE_BUILD_DIR="${CWD}/${BUILD_SUBDIR}/Release"
mkdir -p "${RELEASE_BUILD_DIR}"
echo "RELEASE_BUILD_DIR = ${RELEASE_BUILD_DIR}"

# We don't need to install, but just to prevent it from going into /usr/local,
RELEASE_INSTALL_DIR="${CWD}/install/${BUILD_SUBDIR}/Release"

# Config
cd "${RELEASE_BUILD_DIR}"
cmake -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="${DEPOT_CONAN_PATHS_CMAKE_FILE}" \
  -DCMAKE_PREFIX_PATH="${DEPOT_INSTALL_DIR}" \
  -DCMAKE_INSTALL_PREFIX="${RELEASE_INSTALL_DIR}" \
  -GNinja \
  "${SRC_DIR}"












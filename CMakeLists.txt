cmake_minimum_required(VERSION 3.24)
project(MAVMOOS VERSION 0.17.0 LANGUAGES CXX)

if(NOT UNIX)
  message(FATAL_ERROR "MAVMOOS UDP components require POSIX IPv4 sockets.")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(MOOS_IVP_ROOT "$ENV{MOOS_IVP_ROOT}" CACHE PATH "Built stock MOOS-IvP checkout")
if(NOT MOOS_IVP_ROOT)
  find_program(MAVMOOS_HELM pHelmIvP PATHS ENV PATH NO_DEFAULT_PATH NO_CACHE)
  if(MAVMOOS_HELM)
    file(REAL_PATH "${MAVMOOS_HELM}" helm_executable)
    get_filename_component(helm_bin "${helm_executable}" DIRECTORY)
    get_filename_component(helm_checkout "${helm_bin}" DIRECTORY)
  endif()
  unset(MOOS_IVP_ROOT CACHE)
  find_path(MOOS_IVP_ROOT NAMES build-ivp.sh
    PATHS "${helm_checkout}" "${CMAKE_CURRENT_SOURCE_DIR}/../moos-ivp"
    NO_DEFAULT_PATH DOC "Built stock MOOS-IvP checkout")
endif()
if(NOT EXISTS "${MOOS_IVP_ROOT}/ivp/src")
  message(FATAL_ERROR "Add your built MOOS-IvP checkout's bin directory to PATH, or set MOOS_IVP_ROOT to the checkout")
endif()
find_package(MOOS 10.0 REQUIRED CONFIG PATHS
  "${MOOS_IVP_ROOT}/build/MOOS/MOOSCore"
  "${MOOS_IVP_ROOT}/build/MOOS"
  NO_DEFAULT_PATH)
find_library(MAVMOOS_APPUTIL apputil PATHS "${MOOS_IVP_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(MAVMOOS_MBUTIL mbutil PATHS "${MOOS_IVP_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)

include(FetchContent)
set(MAVMOOS_MAVLINK_ARCHIVE "" CACHE FILEPATH "Optional offline copy of the pinned MAVLink tar.gz (hash checked)")
set(mavlink_url "https://codeload.github.com/mavlink/c_library_v2/tar.gz/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29")
if(MAVMOOS_MAVLINK_ARCHIVE)
  set(mavlink_url "${MAVMOOS_MAVLINK_ARCHIVE}")
endif()
FetchContent_Declare(mavlink
  URL "${mavlink_url}"
  URL_HASH SHA256=3fc41a3cd46717b1aa226b958aed0829fbe7b7c9793bc8ffdcca76345a9dfde3
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(mavlink)
add_library(mavlink_headers INTERFACE)
target_include_directories(mavlink_headers SYSTEM INTERFACE "${mavlink_SOURCE_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
add_subdirectory(src)
include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

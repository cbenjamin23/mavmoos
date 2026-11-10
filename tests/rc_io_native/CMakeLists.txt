cmake_minimum_required(VERSION 3.24)
project(MavmoosRcNativeProbe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(MOOS_IVP_ROOT "$ENV{MOOS_IVP_ROOT}" CACHE PATH "Built stock MOOS-IvP dependency")
find_package(MOOS 10.0 REQUIRED CONFIG PATHS
  "${MOOS_IVP_ROOT}/build/MOOS/MOOSCore" "${MOOS_IVP_ROOT}/build/MOOS" NO_DEFAULT_PATH)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
add_executable(rc_io_moos_probe RcIoMoosProbe.cpp)
target_link_libraries(rc_io_moos_probe PRIVATE ${MOOS_LIBRARIES})

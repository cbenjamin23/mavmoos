cmake_minimum_required(VERSION 3.24)
project(MAVMOOSSendBoundary LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
enable_testing()
find_package(Threads REQUIRED)
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
if(NOT MAVLINK_DIR)
  message(FATAL_ERROR "Supply MAVLINK_DIR containing generated ardupilotmega/mavlink.h")
endif()
add_library(boundary_rx STATIC
  "${REPO_ROOT}/src/libmavmoos_rx/Config.cpp"
  "${REPO_ROOT}/src/libmavmoos_rx/Telemetry.cpp"
  "${REPO_ROOT}/src/libmavmoos_rx/UdpReceiver.cpp"
  "${REPO_ROOT}/src/libmavmoos_rx/SimulationSendGate.cpp")
target_include_directories(boundary_rx PUBLIC "${REPO_ROOT}/src/libmavmoos_rx")
target_include_directories(boundary_rx SYSTEM PUBLIC "${MAVLINK_DIR}")
target_compile_options(boundary_rx PRIVATE -Wall -Wextra -Wpedantic)
add_executable(send_boundary_tests SendBoundaryTests.cpp)
target_link_libraries(send_boundary_tests PRIVATE boundary_rx Threads::Threads)
target_compile_options(send_boundary_tests PRIVATE -Wall -Wextra -Wpedantic)
add_executable(send_boundary_inert_fixture InertSocketOwner.cpp)
set_target_properties(send_boundary_inert_fixture PROPERTIES OUTPUT_NAME arducopter
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/test-fixtures/copter")
target_compile_options(send_boundary_inert_fixture PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME send_boundary_fake COMMAND send_boundary_tests fake)
add_test(NAME send_boundary_loopback COMMAND send_boundary_tests loopback $<TARGET_FILE:send_boundary_inert_fixture>)
set_tests_properties(send_boundary_fake send_boundary_loopback PROPERTIES TIMEOUT 15 RUN_SERIAL TRUE)

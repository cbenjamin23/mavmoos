cmake_minimum_required(VERSION 3.24)
project(MAVMOOSObserverReview LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(REVIEW_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
if(NOT MAVLINK_DIR)
  message(FATAL_ERROR "Supply MAVLINK_DIR with the immutable generated MAVLink headers")
endif()
add_library(observer_review_runtime STATIC
  "${REVIEW_ROOT}/src/libmavmoos_runtime/Runtime.cpp"
  "${REVIEW_ROOT}/src/libmavmoos_runtime/RuntimeMission.cpp"
  "${REVIEW_ROOT}/src/libmavmoos_runtime/PluginHost.cpp"
  "${REVIEW_ROOT}/src/libmavmoos_runtime/ModuleSelection.cpp")
target_include_directories(observer_review_runtime PUBLIC "${REVIEW_ROOT}/src/libmavmoos_runtime")
target_include_directories(observer_review_runtime SYSTEM PUBLIC "${MAVLINK_DIR}")
target_link_libraries(observer_review_runtime PRIVATE ${CMAKE_DL_LIBS})
target_compile_options(observer_review_runtime PRIVATE -Wall -Wextra -Wpedantic)
add_executable(observer_host_driver ObserverHostDriver.cpp)
add_library(observer_review_ingress STATIC
  "${REVIEW_ROOT}/src/libmavmoos_rx/Config.cpp"
  "${REVIEW_ROOT}/src/libmavmoos_rx/Telemetry.cpp")
target_include_directories(observer_review_ingress PUBLIC "${REVIEW_ROOT}/src/libmavmoos_rx")
target_include_directories(observer_review_ingress SYSTEM PUBLIC "${MAVLINK_DIR}")
target_compile_options(observer_review_ingress PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(observer_host_driver PRIVATE observer_review_runtime observer_review_ingress)
target_compile_options(observer_host_driver PRIVATE -Wall -Wextra -Wpedantic)
add_library(observer_interval_fixture MODULE IntervalRequestFixture.cpp)
target_include_directories(observer_interval_fixture PRIVATE "${REVIEW_ROOT}/src/libmavmoos_runtime")
target_include_directories(observer_interval_fixture SYSTEM PRIVATE "${MAVLINK_DIR}")

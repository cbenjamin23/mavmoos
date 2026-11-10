cmake_minimum_required(VERSION 3.16)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(mavmoos_fault_coordination_controls LANGUAGES CXX)
  enable_testing()
  set(MOOS_IVP_ROOT "$ENV{MOOS_IVP_ROOT}" CACHE PATH "Built stock MOOS-IvP checkout")
  find_package(MOOS 10.0 REQUIRED CONFIG PATHS
    "${MOOS_IVP_ROOT}/build/MOOS/MOOSCore"
    "${MOOS_IVP_ROOT}/build/MOOS"
    NO_DEFAULT_PATH)
endif()
add_executable(marker_wait_tests MarkerWaitTests.cpp)
target_compile_features(marker_wait_tests PRIVATE cxx_std_17)
target_link_libraries(marker_wait_tests PRIVATE ${MOOS_LIBRARIES})
add_test(NAME fault_marker_wait COMMAND marker_wait_tests "${CMAKE_CURRENT_BINARY_DIR}/marker-wait-files")

set_tests_properties(fault_marker_wait PROPERTIES TIMEOUT 10)

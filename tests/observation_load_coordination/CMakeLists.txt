cmake_minimum_required(VERSION 3.16)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(mavmoos_observation_finish_controls LANGUAGES CXX)
  enable_testing()
  set(MOOS_IVP_ROOT "$ENV{MOOS_IVP_ROOT}" CACHE PATH "Built stock MOOS-IvP checkout")
  find_package(MOOS 10.0 REQUIRED CONFIG PATHS
    "${MOOS_IVP_ROOT}/build/MOOS/MOOSCore"
    "${MOOS_IVP_ROOT}/build/MOOS"
    NO_DEFAULT_PATH)
endif()
add_executable(observation_finish_tests FinishBoundaryTests.cpp)
target_compile_features(observation_finish_tests PRIVATE cxx_std_17)
target_link_libraries(observation_finish_tests PRIVATE ${MOOS_LIBRARIES})
target_compile_options(observation_finish_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME observation_finish_boundary COMMAND observation_finish_tests "${CMAKE_CURRENT_BINARY_DIR}/finish-boundary-files")
set_tests_properties(observation_finish_boundary PROPERTIES TIMEOUT 10)

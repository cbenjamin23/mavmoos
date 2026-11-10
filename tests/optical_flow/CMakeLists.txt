add_executable(optical_flow_scenario_driver OpticalFlowScenarioDriver.cpp)
target_include_directories(optical_flow_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(optical_flow_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(optical_flow_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME optical_flow_observations COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/OpticalFlowTests.py"
  --driver $<TARGET_FILE:optical_flow_scenario_driver> --module $<TARGET_FILE:mavmoos_optical_flow>)
set_tests_properties(optical_flow_observations PROPERTIES TIMEOUT 15)

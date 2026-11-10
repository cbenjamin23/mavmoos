add_executable(debug_values_scenario_driver DebugScenarioDriver.cpp)
target_include_directories(debug_values_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(debug_values_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(debug_values_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME debug_values_observations COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/DebugValuesTests.py"
  --driver $<TARGET_FILE:debug_values_scenario_driver> --module $<TARGET_FILE:mavmoos_debug_values>)
set_tests_properties(debug_values_observations PROPERTIES TIMEOUT 15)

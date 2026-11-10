add_executable(esc_status_scenario_driver EscStatusScenarioDriver.cpp)
target_include_directories(esc_status_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(esc_status_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(esc_status_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME esc_status_observations
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/EscStatusProviderTests.py"
  --driver $<TARGET_FILE:esc_status_scenario_driver> --module $<TARGET_FILE:mavmoos_esc_status>)
set_tests_properties(esc_status_observations PROPERTIES TIMEOUT 20)

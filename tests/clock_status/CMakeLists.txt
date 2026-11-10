add_executable(clock_status_scenario_driver ClockStatusScenarioDriver.cpp)
target_include_directories(clock_status_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(clock_status_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(clock_status_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME clock_status_observations COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/ClockStatusTests.py"
  --driver $<TARGET_FILE:clock_status_scenario_driver> --module $<TARGET_FILE:mavmoos_clock_status>)
set_tests_properties(clock_status_observations PROPERTIES TIMEOUT 15)

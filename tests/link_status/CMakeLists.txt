add_executable(link_status_scenario_driver LinkStatusScenarioDriver.cpp)
target_include_directories(link_status_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(link_status_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(link_status_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME link_status_observations COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/LinkStatusTests.py"
  --driver $<TARGET_FILE:link_status_scenario_driver> --module $<TARGET_FILE:mavmoos_link_status>)
set_tests_properties(link_status_observations PROPERTIES TIMEOUT 15)

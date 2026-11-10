add_executable(event_timeline_scenario_driver EventScenarioDriver.cpp)
target_include_directories(event_timeline_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(event_timeline_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(event_timeline_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME event_timeline_observations COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/EventTimelineTests.py"
  --driver $<TARGET_FILE:event_timeline_scenario_driver> --module $<TARGET_FILE:mavmoos_event_timeline>)
set_tests_properties(event_timeline_observations PROPERTIES TIMEOUT 15)

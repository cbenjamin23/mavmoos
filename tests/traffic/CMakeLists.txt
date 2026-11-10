add_executable(traffic_scenario_driver TrafficScenarioDriver.cpp)
target_include_directories(traffic_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(traffic_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(traffic_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME traffic_observations COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/TrafficTests.py"
  --driver $<TARGET_FILE:traffic_scenario_driver> --module $<TARGET_FILE:mavmoos_traffic>)
set_tests_properties(traffic_observations PROPERTIES TIMEOUT 15)

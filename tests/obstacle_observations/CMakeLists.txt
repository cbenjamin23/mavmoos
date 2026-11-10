add_executable(obstacle_scenario_driver ObstacleScenarioDriver.cpp)
target_include_directories(obstacle_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(obstacle_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(obstacle_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME obstacle_observations COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/ObstacleTests.py"
  --driver $<TARGET_FILE:obstacle_scenario_driver> --module $<TARGET_FILE:mavmoos_distance_sensor>)
set_tests_properties(obstacle_observations PROPERTIES TIMEOUT 15)

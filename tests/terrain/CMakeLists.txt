add_executable(terrain_scenario_driver TerrainScenarioDriver.cpp)
target_include_directories(terrain_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(terrain_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(terrain_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME terrain_observations COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/TerrainTests.py"
  --driver $<TARGET_FILE:terrain_scenario_driver> --module $<TARGET_FILE:mavmoos_terrain>)
set_tests_properties(terrain_observations PROPERTIES TIMEOUT 15)

add_executable(water_depth_scenario_driver WaterDepthScenarioDriver.cpp)
target_include_directories(water_depth_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(water_depth_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(water_depth_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME water_depth_observations COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/WaterDepthTests.py"
  --driver $<TARGET_FILE:water_depth_scenario_driver> --module $<TARGET_FILE:mavmoos_water_depth>)
set_tests_properties(water_depth_observations PROPERTIES TIMEOUT 15)

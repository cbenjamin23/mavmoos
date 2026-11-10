add_executable(odometry_scenario_driver OdometryScenarioDriver.cpp)
target_include_directories(odometry_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(odometry_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(odometry_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME odometry_observations COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/OdometryProviderTests.py"
  --driver $<TARGET_FILE:odometry_scenario_driver> --plugin $<TARGET_FILE:mavmoos_odometry>)
set_tests_properties(odometry_observations PROPERTIES TIMEOUT 15)

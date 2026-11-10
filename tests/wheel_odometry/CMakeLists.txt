add_executable(wheel_scenario_driver WheelScenarioDriver.cpp)
target_include_directories(wheel_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(wheel_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(wheel_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME wheel_observations COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/WheelTests.py"
  --driver $<TARGET_FILE:wheel_scenario_driver> --module $<TARGET_FILE:mavmoos_odometry>)
set_tests_properties(wheel_observations PROPERTIES TIMEOUT 15)

add_executable(landing_target_scenario_driver LandingTargetScenarioDriver.cpp)
target_include_directories(landing_target_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(landing_target_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(landing_target_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME landing_target_observations COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/LandingTargetTests.py"
  --driver $<TARGET_FILE:landing_target_scenario_driver> --module $<TARGET_FILE:mavmoos_landing_target>)
set_tests_properties(landing_target_observations PROPERTIES TIMEOUT 15)

add_executable(air_data_scenario_driver AirDataScenarioDriver.cpp)
target_include_directories(air_data_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(air_data_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(air_data_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME air_data_observations COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/AirDataTests.py"
  --driver $<TARGET_FILE:air_data_scenario_driver> --module $<TARGET_FILE:mavmoos_air_data>)
set_tests_properties(air_data_observations PROPERTIES TIMEOUT 15)

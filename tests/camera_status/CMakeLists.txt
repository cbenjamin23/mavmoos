add_executable(camera_scenario_driver CameraScenarioDriver.cpp)
target_include_directories(camera_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(camera_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(camera_scenario_driver PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME camera_status_observations COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/CameraTests.py"
  --driver $<TARGET_FILE:camera_scenario_driver> --module $<TARGET_FILE:mavmoos_camera_status>)
set_tests_properties(camera_status_observations PROPERTIES TIMEOUT 15)

add_executable(camera_status_frames CameraLoopbackFrames.cpp)
target_link_libraries(camera_status_frames PRIVATE mavlink_headers)

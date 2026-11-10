add_executable(payload_runtime_tests PayloadRuntimeTests.cpp)
target_link_libraries(payload_runtime_tests PRIVATE mavmoos_runtime)
add_test(NAME payload_runtime COMMAND payload_runtime_tests $<TARGET_FILE:mavmoos_telemetry>
  $<TARGET_FILE:mavmoos_camera_status> $<TARGET_FILE:mavmoos_gimbal_status>)
set_tests_properties(payload_runtime PROPERTIES TIMEOUT 15)

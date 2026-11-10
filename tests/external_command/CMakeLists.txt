add_executable(external_disarm_runtime RuntimeTests.cpp)
target_link_libraries(external_disarm_runtime PRIVATE mavmoos_runtime)
target_compile_options(external_disarm_runtime PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME external_disarm_runtime COMMAND external_disarm_runtime
  $<TARGET_FILE:mavmoos_arducopter_control> $<TARGET_FILE:mavmoos_example_disarmed_command>)
set_tests_properties(external_disarm_runtime PROPERTIES TIMEOUT 20)

add_executable(disarm_sitl_probe DisarmSitlProbe.cpp)
target_link_libraries(disarm_sitl_probe PRIVATE ${MOOS_LIBRARIES})
target_compile_options(disarm_sitl_probe PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME external_disarm_evidence COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/EvidenceTests.py")
set_tests_properties(external_disarm_evidence PROPERTIES TIMEOUT 20)

if(MAVMOOS_ARDUPILOT_ROOT AND MAVMOOS_SITL_PYTHON)
  add_test(NAME external_disarm_sitl COMMAND "${MAVMOOS_SITL_PYTHON}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/run_native.py"
    --source-root "${PROJECT_SOURCE_DIR}" --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}"
    --moos-root "${MOOS_IVP_ROOT}" --build "${CMAKE_BINARY_DIR}"
    --probe $<TARGET_FILE:disarm_sitl_probe> --module $<TARGET_FILE:mavmoos_example_disarmed_command>
    --output "${CMAKE_BINARY_DIR}/evidence/external-disarm")
  set_tests_properties(external_disarm_sitl PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE LABELS "sitl;commands")
endif()

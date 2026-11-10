add_executable(auto_recovery_core_tests RecoveryTests.cpp)
target_link_libraries(auto_recovery_core_tests PRIVATE mavmoos_runtime)
target_compile_options(auto_recovery_core_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME auto_recovery_core COMMAND auto_recovery_core_tests
  $<TARGET_FILE:mavmoos_arducopter_control> $<TARGET_FILE:mavmoos_missions>
  $<TARGET_FILE:mavmoos_parameters>)
set_tests_properties(auto_recovery_core PROPERTIES TIMEOUT 25)

add_executable(auto_recovery_probe RecoveryProbe.cpp)
target_link_libraries(auto_recovery_probe PRIVATE mavmoos_runtime ${MOOS_LIBRARIES})
target_compile_options(auto_recovery_probe PRIVATE -Wall -Wextra -Wpedantic)

if(MAVMOOS_ARDUPILOT_ROOT AND MAVMOOS_SITL_PYTHON)
  add_test(NAME auto_restart_recovery_sitl COMMAND "${MAVMOOS_SITL_PYTHON}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/run_native.py"
    --source-root "${PROJECT_SOURCE_DIR}" --build "${CMAKE_BINARY_DIR}"
    --moos-root "${MOOS_IVP_ROOT}" --behavior-dir "${MAVMOOS_BEHAVIOR_DIR}"
    --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}"
    --probe $<TARGET_FILE:auto_recovery_probe> --starter-probe $<TARGET_FILE:mission_execution_probe>
    --output "${CMAKE_BINARY_DIR}/evidence/auto-restart-recovery")
  set_tests_properties(auto_restart_recovery_sitl PROPERTIES TIMEOUT 380 RUN_SERIAL TRUE
    LABELS "sitl;missions;faults")
endif()

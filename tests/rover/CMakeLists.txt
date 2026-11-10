add_executable(rover_backend_tests RoverBackendTests.cpp)
target_link_libraries(rover_backend_tests PRIVATE mavmoos_rover_components)
add_test(NAME rover_backend COMMAND rover_backend_tests)

add_executable(rover_control_tests RoverControlTests.cpp)
target_link_libraries(rover_control_tests PRIVATE mavmoos_runtime mavmoos_rover_components ${CMAKE_DL_LIBS})
add_test(NAME rover_control COMMAND rover_control_tests $<TARGET_FILE:mavmoos_ardurover_control>)

add_executable(rover_operation_tests RoverOperationTests.cpp)
target_link_libraries(rover_operation_tests PRIVATE mavmoos_rover_components)
add_test(NAME rover_operations COMMAND rover_operation_tests)
foreach(profile rover copter)
  add_executable(${profile}_guard_fixture RoverGuardFixture.cpp)
  set_target_properties(${profile}_guard_fixture PROPERTIES OUTPUT_NAME "ardu${profile}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/test-fixtures/${profile}")
endforeach()
add_executable(rover_profile_guard_tests RoverGuardTests.cpp)
target_link_libraries(rover_profile_guard_tests PRIVATE mavmoos_rx)
add_test(NAME rover_profile_guards COMMAND rover_profile_guard_tests
  $<TARGET_FILE:rover_guard_fixture> $<TARGET_FILE:copter_guard_fixture>)
set_tests_properties(rover_profile_guards PROPERTIES TIMEOUT 15 RUN_SERIAL TRUE SKIP_RETURN_CODE 77)

add_executable(rover_workflow_probe RoverWorkflowProbe.cpp)
target_link_libraries(rover_workflow_probe PRIVATE ${MOOS_LIBRARIES})
add_executable(rover_expanded_probe ../rover_expanded/RoverExpandedProbe.cpp)
target_link_libraries(rover_expanded_probe PRIVATE ${MOOS_LIBRARIES})

# A separate source/build path avoids implying that a Copter binary is a Rover.
set(MAVMOOS_ROVER_ARDUPILOT_ROOT "${MAVMOOS_ARDUPILOT_ROOT}" CACHE PATH
    "Pinned source export containing build/sitl/bin/ardurover")
if(MAVMOOS_SITL_PYTHON)
  add_test(NAME rover_harness_evidence COMMAND "${MAVMOOS_SITL_PYTHON}"
    "${CMAKE_CURRENT_SOURCE_DIR}/HarnessEvidenceTests.py")
endif()
set(rover_codec_python "${Python3_EXECUTABLE}")
if(MAVMOOS_SITL_PYTHON)
  set(rover_codec_python "${MAVMOOS_SITL_PYTHON}")
endif()
execute_process(COMMAND "${rover_codec_python}" -B -c "import pymavlink"
  RESULT_VARIABLE rover_codec_result OUTPUT_QUIET ERROR_QUIET)
if(rover_codec_result EQUAL 0)
  add_test(NAME rover_passive_profiles COMMAND "${rover_codec_python}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/../rover_expanded/run_passive.py"
    --moos-root "${MOOS_IVP_ROOT}" --build "${CMAKE_BINARY_DIR}"
    --output-parent "${CMAKE_BINARY_DIR}/evidence/rover-passive")
  set_tests_properties(rover_passive_profiles PROPERTIES TIMEOUT 140 RUN_SERIAL TRUE)
endif()
if(EXISTS "${MAVMOOS_ROVER_ARDUPILOT_ROOT}/build/sitl/bin/ardurover" AND MAVMOOS_SITL_PYTHON)
  add_test(NAME native_rover COMMAND "${MAVMOOS_SITL_PYTHON}"
    "${CMAKE_CURRENT_SOURCE_DIR}/run_rover.py"
    --ardupilot-root "${MAVMOOS_ROVER_ARDUPILOT_ROOT}"
    --moos-root "${MOOS_IVP_ROOT}" --build "${CMAKE_BINARY_DIR}"
    --output "${CMAKE_BINARY_DIR}/evidence/native-rover")
  # GPS/EKF convergence and movement/return are observed at real-time speed.
  # The wrapper also takes the common native lock and owns every process/port.
  set_tests_properties(native_rover PROPERTIES TIMEOUT 240 RUN_SERIAL TRUE LABELS "sitl;rover")
  foreach(profile ground boat)
    add_test(NAME native_rover_${profile} COMMAND "${MAVMOOS_SITL_PYTHON}" -B
      "${CMAKE_CURRENT_SOURCE_DIR}/../rover_expanded/run_native.py"
      --profile "${profile}" --ardupilot-root "${MAVMOOS_ROVER_ARDUPILOT_ROOT}"
      --moos-root "${MOOS_IVP_ROOT}" --build "${CMAKE_BINARY_DIR}"
      --output "${CMAKE_BINARY_DIR}/evidence/native-rover-${profile}")
    set_tests_properties(native_rover_${profile} PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE LABELS "sitl;rover")
  endforeach()
endif()

add_executable(rover_mission_probe ../rover_mission/RoverMissionProbe.cpp)
target_link_libraries(rover_mission_probe PRIVATE ${MOOS_LIBRARIES})

add_executable(autonomy_mission_probe ../rover_mission/RoverMissionProbe.cpp)
target_link_libraries(autonomy_mission_probe PRIVATE ${MOOS_LIBRARIES})

add_executable(ardu_rover_mission_runtime_tests ../rover_mission/ArduRoverMissionRuntimeTests.cpp)
target_link_libraries(ardu_rover_mission_runtime_tests PRIVATE mavmoos_runtime)
add_test(NAME ardu_rover_mission_runtime COMMAND ardu_rover_mission_runtime_tests
  $<TARGET_FILE:mavmoos_ardurover_control> $<TARGET_FILE:mavmoos_missions>)
set_tests_properties(ardu_rover_mission_runtime PROPERTIES TIMEOUT 30)

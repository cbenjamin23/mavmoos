find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Threads REQUIRED)
add_test(NAME release_evidence COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/release_evidence/ReleaseEvidenceTests.py")
set_tests_properties(release_evidence PROPERTIES TIMEOUT 15)
add_test(NAME release_profiles COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/release_profiles/test_profiles.py")
set_tests_properties(release_profiles PROPERTIES TIMEOUT 15)
add_subdirectory(optical_flow)
add_subdirectory(terrain)
add_subdirectory(link_status)
add_subdirectory(clock_status)
add_subdirectory(navigation_monitor)
add_subdirectory(obstacle_observations)
add_subdirectory(wheel_odometry)
add_subdirectory(camera_status)
add_subdirectory(gimbal_status)
add_subdirectory(fence_operations)
add_subdirectory(payload_gimbal)
add_subdirectory(payload_operations)
add_subdirectory(payload_runtime)
add_executable(camera_capture_tests payload_camera/CameraCaptureTests.cpp)
target_include_directories(camera_capture_tests PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime"
  "${CMAKE_CURRENT_SOURCE_DIR}/camera_status")
target_link_libraries(camera_capture_tests PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
add_test(NAME camera_capture_workflow COMMAND camera_capture_tests $<TARGET_FILE:mavmoos_camera_status>)
set_tests_properties(camera_capture_workflow PROPERTIES TIMEOUT 15)
add_subdirectory(observer_extensions_native)
add_subdirectory(odometry)
add_subdirectory(air_data)
add_subdirectory(plane_native)
add_subdirectory(quadplane_native)
add_subdirectory(plane_provider)
add_subdirectory(plane_faults)
add_subdirectory(px4_runtime)
add_subdirectory(px4_offboard)
add_subdirectory(px4_rover)
add_executable(plane_runtime_tests PlaneRuntimeTests.cpp)
target_link_libraries(plane_runtime_tests PRIVATE mavmoos_runtime mavmoos_plane_components)
add_test(NAME plane_runtime COMMAND plane_runtime_tests $<TARGET_FILE:mavmoos_arduplane_control>)
set_tests_properties(plane_runtime PROPERTIES TIMEOUT 15)
add_subdirectory(esc_status)
add_subdirectory(landing_target)
add_subdirectory(observer_matrix)
add_subdirectory(ardusub)
add_subdirectory(component_metadata)
add_subdirectory(external_navigation)
add_subdirectory(mode_catalog)
add_subdirectory(remote_id)
add_subdirectory(traffic)
add_executable(flight_log_runtime_tests FlightLogRuntimeTests.cpp)
target_link_libraries(flight_log_runtime_tests PRIVATE mavmoos_runtime)
add_test(NAME flight_log_runtime COMMAND flight_log_runtime_tests $<TARGET_FILE:mavmoos_arducopter_control>)
set_tests_properties(flight_log_runtime PROPERTIES TIMEOUT 15)
add_executable(mission_execution_runtime_tests MissionExecutionRuntimeTests.cpp)
target_link_libraries(mission_execution_runtime_tests PRIVATE mavmoos_runtime)
add_test(NAME mission_execution_runtime COMMAND mission_execution_runtime_tests $<TARGET_FILE:mavmoos_arducopter_control> $<TARGET_FILE:mavmoos_missions>)
set_tests_properties(mission_execution_runtime PROPERTIES TIMEOUT 15)
add_executable(copter_hold_boundary_wait copter_scenarios/HoldBoundaryTests.cpp)
target_link_libraries(copter_hold_boundary_wait PRIVATE ${MOOS_LIBRARIES})
add_test(NAME copter_hold_boundary_wait COMMAND copter_hold_boundary_wait)
set_tests_properties(copter_hold_boundary_wait PROPERTIES TIMEOUT 10)
add_executable(copter_mission_execution_tests mission_execution/MissionExecutionTests.cpp)
target_link_libraries(copter_mission_execution_tests PRIVATE mavmoos_control_components)
add_test(NAME copter_mission_execution COMMAND copter_mission_execution_tests)
set_tests_properties(copter_mission_execution PROPERTIES TIMEOUT 15)
add_test(NAME firmware_scenario_fixtures COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/firmware/ScenarioFixtureTests.py")
set_tests_properties(firmware_scenario_fixtures PROPERTIES TIMEOUT 15)
add_executable(firmware_mission_fixture_shapes_tests firmware/MissionFixtureShapeTests.cpp)
target_include_directories(firmware_mission_fixture_shapes_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/firmware")
add_test(NAME firmware_mission_fixture_shapes COMMAND firmware_mission_fixture_shapes_tests)
set_tests_properties(firmware_mission_fixture_shapes PROPERTIES TIMEOUT 10)
add_test(NAME concurrent_mission_fixture_evidence COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/concurrent/MissionReadEvidenceTests.py")
set_tests_properties(concurrent_mission_fixture_evidence PROPERTIES TIMEOUT 15)
add_subdirectory("${PROJECT_SOURCE_DIR}/examples/heartbeat_observer" "${CMAKE_BINARY_DIR}/external-example")
add_subdirectory("${PROJECT_SOURCE_DIR}/examples/disarmed_command" "${CMAKE_BINARY_DIR}/external-command-example")
add_subdirectory(plugin_configuration)
add_executable(external_observer_probe external_observer/ExternalObserverProbe.cpp)
target_link_libraries(external_observer_probe PRIVATE ${MOOS_LIBRARIES})
add_test(NAME external_observer_evidence COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/external_observer/EvidenceTests.py")
set_tests_properties(external_observer_evidence PROPERTIES TIMEOUT 20)
set(MAVMOOS_EXTERNAL_OBSERVER_EVIDENCE_DIR "${CMAKE_BINARY_DIR}/external-observer-evidence"
    CACHE PATH "Retained raw external observer app-case evidence")
foreach(scenario normal defaults invalid_config wrong_abi unknown_key duplicate_key missing_owner reserved_policy noncanonical)
  add_test(NAME external_observer_moos_${scenario} COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/external_observer/CTestCase.py"
    --scenario "${scenario}" --source-root "${PROJECT_SOURCE_DIR}"
    --build "${CMAKE_BINARY_DIR}" --module $<TARGET_FILE:mavmoos_example_heartbeat_observer>
    --wrong-abi $<TARGET_FILE:example_wrong_abi> --moos-root "${MOOS_IVP_ROOT}"
    --probe $<TARGET_FILE:external_observer_probe>
    --output-parent "${MAVMOOS_EXTERNAL_OBSERVER_EVIDENCE_DIR}")
  set_tests_properties(external_observer_moos_${scenario} PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE LABELS "loopback")
endforeach()
add_executable(mission_execution_probe mission_execution/MissionExecutionProbe.cpp)
target_link_libraries(mission_execution_probe PRIVATE ${MOOS_LIBRARIES})
add_test(NAME mission_execution_evidence COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/mission_execution/ExecutionEvidenceTests.py")
set_tests_properties(mission_execution_evidence PROPERTIES TIMEOUT 15)
add_executable(mission_execution_boundary_wait mission_execution/MissionExecutionBoundaryTests.cpp)
target_link_libraries(mission_execution_boundary_wait PRIVATE ${MOOS_LIBRARIES})
add_test(NAME mission_execution_boundary_wait COMMAND mission_execution_boundary_wait)
set_tests_properties(mission_execution_boundary_wait PROPERTIES TIMEOUT 20)
add_executable(imu_scenario_driver imu/ImuScenarioDriver.cpp)
target_include_directories(imu_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(imu_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
add_test(NAME imu_provider_observations COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/imu/ImuProviderTests.py"
  --driver $<TARGET_FILE:imu_scenario_driver> --plugin $<TARGET_FILE:mavmoos_imu>)
set_tests_properties(imu_provider_observations PROPERTIES TIMEOUT 30)
add_executable(imu_moos_probe imu/ImuMoosProbe.cpp)
target_link_libraries(imu_moos_probe PRIVATE ${MOOS_LIBRARIES})
add_test(NAME imu_native_evidence COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/imu/NativeEvidenceTests.py")
set_tests_properties(imu_native_evidence PROPERTIES TIMEOUT 15)
set(mavmoos_imu_codec_python "${Python3_EXECUTABLE}")
if(MAVMOOS_SITL_PYTHON AND EXISTS "${MAVMOOS_SITL_PYTHON}")
  set(mavmoos_imu_codec_python "${MAVMOOS_SITL_PYTHON}")
endif()
execute_process(COMMAND "${mavmoos_imu_codec_python}" -B -c "import pymavlink"
  RESULT_VARIABLE mavmoos_imu_codec_available OUTPUT_QUIET ERROR_QUIET TIMEOUT 10)
if(mavmoos_imu_codec_available EQUAL 0)
  add_test(NAME imu_wire_codec COMMAND "${mavmoos_imu_codec_python}" -B "${CMAKE_CURRENT_SOURCE_DIR}/imu/WireCodecTests.py")
  set_tests_properties(imu_wire_codec PROPERTIES TIMEOUT 15)
endif()
add_executable(execution_concurrency_tests mission_execution/ExecutionConcurrencyTests.cpp)
target_link_libraries(execution_concurrency_tests PRIVATE mavmoos_runtime)
add_test(NAME mission_execution_concurrency COMMAND execution_concurrency_tests
  $<TARGET_FILE:mavmoos_arducopter_control> $<TARGET_FILE:mavmoos_missions> $<TARGET_FILE:mavmoos_parameters>)
set_tests_properties(mission_execution_concurrency PROPERTIES TIMEOUT 20)
add_executable(firmware_runtime_tests firmware_runtime/FirmwareRuntimeTests.cpp)
target_link_libraries(firmware_runtime_tests PRIVATE mavmoos_runtime)
add_test(NAME firmware_runtime COMMAND firmware_runtime_tests $<TARGET_FILE:mavmoos_telemetry> $<TARGET_FILE:mavmoos_arducopter_control>)
set_tests_properties(firmware_runtime PROPERTIES TIMEOUT 15)
add_test(NAME firmware_selector COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/firmware_runtime/SelectorTests.py")
set_tests_properties(firmware_selector PROPERTIES TIMEOUT 10)
add_executable(firmware_discovery_tests firmware/FirmwareDiscoveryTests.cpp)
target_link_libraries(firmware_discovery_tests PRIVATE mavmoos_runtime)
add_test(NAME firmware_discovery COMMAND firmware_discovery_tests)
set_tests_properties(firmware_discovery PROPERTIES TIMEOUT 15)
add_test(NAME firmware_manifests COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/firmware/ManifestTests.py")
set_tests_properties(firmware_manifests PROPERTIES TIMEOUT 45)
add_test(NAME ci_runner COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/ci/run_tests.py")
set_tests_properties(ci_runner PROPERTIES TIMEOUT 120)
add_subdirectory(copter_scenarios)
add_subdirectory(faults_coordination)
add_subdirectory(observation_load_coordination)
add_test(NAME capability_registry COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/copter_scenarios/RegistryTests.py"
  --source-root "${PROJECT_SOURCE_DIR}")
set_tests_properties(capability_registry PROPERTIES TIMEOUT 30)
add_test(NAME copter_scenario_audit COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/copter_scenarios/ScenarioEvidenceTests.py"
  --checker "${PROJECT_SOURCE_DIR}/tests/copter_scenarios/scenario_evidence.py")
set_tests_properties(copter_scenario_audit PROPERTIES TIMEOUT 30)
add_test(NAME distance_native_audit COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/distance_native/DistanceEvidenceTests.py"
  --checker "${CMAKE_CURRENT_SOURCE_DIR}/distance_native/distance_evidence.py")
set_tests_properties(distance_native_audit PROPERTIES TIMEOUT 15)
add_executable(protocol_fault_tests protocol_faults/ProtocolFaultTests.cpp)
target_link_libraries(protocol_fault_tests PRIVATE mavmoos_runtime)
add_test(NAME protocol_faults COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/protocol_faults/run_fake.py"
  --driver $<TARGET_FILE:protocol_fault_tests> --parameters $<TARGET_FILE:mavmoos_parameters>
  --missions $<TARGET_FILE:mavmoos_missions> --control $<TARGET_FILE:mavmoos_arducopter_control>)
set_tests_properties(protocol_faults PROPERTIES TIMEOUT 20)
add_test(NAME protocol_native_audit COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/protocol_faults/test_native_audit.py")
set_tests_properties(protocol_native_audit PROPERTIES TIMEOUT 10)
add_executable(protocol_native_driver protocol_faults/ProtocolNativeDriver.cpp)
target_link_libraries(protocol_native_driver PRIVATE mavmoos_runtime mavmoos_rx)
add_executable(distance_sensor_scenario_driver distance_sensor/DistanceSensorScenarioDriver.cpp)
target_include_directories(distance_sensor_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(distance_sensor_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
add_test(NAME distance_sensor_provider COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/distance_sensor/DistanceSensorProviderTests.py"
  --driver $<TARGET_FILE:distance_sensor_scenario_driver> --module $<TARGET_FILE:mavmoos_distance_sensor>)
set_tests_properties(distance_sensor_provider PROPERTIES TIMEOUT 20)
add_executable(observer_runtime_clock_tests observer_hardening/ObserverRuntimeClockTests.cpp)
target_link_libraries(observer_runtime_clock_tests PRIVATE mavmoos_runtime)
add_test(NAME observer_runtime_clock COMMAND observer_runtime_clock_tests $<TARGET_FILE:mavmoos_attitude> $<TARGET_FILE:mavmoos_system_status> $<TARGET_FILE:mavmoos_rc_io>)
set_tests_properties(observer_runtime_clock PROPERTIES TIMEOUT 10)
add_executable(distance_moos_probe distance_native/DistanceMoosProbe.cpp)
target_link_libraries(distance_moos_probe PRIVATE ${MOOS_LIBRARIES})
add_executable(runtime_initial_clock_tests runtime_initial_clock/RuntimeInitialClockTests.cpp)
target_link_libraries(runtime_initial_clock_tests PRIVATE mavmoos_runtime)
add_test(NAME runtime_initial_clock COMMAND runtime_initial_clock_tests $<TARGET_FILE:mavmoos_telemetry> current)
set_tests_properties(runtime_initial_clock PROPERTIES TIMEOUT 10)
add_executable(stream_request_state_tests stream_requests/StreamRequestTests.cpp)
target_link_libraries(stream_request_state_tests PRIVATE mavmoos_runtime)
add_test(NAME stream_request_state COMMAND stream_request_state_tests)
set_tests_properties(stream_request_state PROPERTIES TIMEOUT 10)
add_executable(rc_io_moos_probe rc_io_native/RcIoMoosProbe.cpp)
target_link_libraries(rc_io_moos_probe PRIVATE ${MOOS_LIBRARIES})
add_test(NAME rc_native_audit COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/rc_io_native/test_rc_native_audit.py")
set_tests_properties(rc_native_audit PROPERTIES TIMEOUT 10)
add_executable(rc_io_scenario_driver rc_io/RcIoScenarioDriver.cpp)
target_include_directories(rc_io_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(rc_io_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
add_test(NAME rc_io_provider COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/rc_io/RcIoProviderTests.py"
  --driver $<TARGET_FILE:rc_io_scenario_driver> --module $<TARGET_FILE:mavmoos_rc_io>)
set_tests_properties(rc_io_provider PROPERTIES TIMEOUT 20)
add_executable(stream_runtime_tests stream_runtime/StreamRuntimeTests.cpp)
target_link_libraries(stream_runtime_tests PRIVATE mavmoos_runtime)
add_test(NAME stream_runtime COMMAND stream_runtime_tests $<TARGET_FILE:mavmoos_telemetry> $<TARGET_FILE:mavmoos_arducopter_control> current)
set_tests_properties(stream_runtime PROPERTIES TIMEOUT 10)
add_executable(observation_load_probe observation_load/ObservationLoadProbe.cpp)
target_link_libraries(observation_load_probe PRIVATE ${MOOS_LIBRARIES})
add_executable(attitude_scenario_driver attitude/AttitudeScenarioDriver.cpp)
target_include_directories(attitude_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(attitude_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
add_test(NAME attitude_provider COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/attitude/AttitudeProviderTests.py"
  --driver $<TARGET_FILE:attitude_scenario_driver> --module $<TARGET_FILE:mavmoos_attitude>)
set_tests_properties(attitude_provider PROPERTIES TIMEOUT 20)
add_test(NAME observation_load_evidence COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/observation_load/ObservationLoadEvidenceTests.py")
set_tests_properties(observation_load_evidence PROPERTIES TIMEOUT 15)
add_library(observer_interval_fixture MODULE observer_review/IntervalRequestFixture.cpp)
target_include_directories(observer_interval_fixture PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(observer_interval_fixture PRIVATE mavlink_headers)
add_executable(observer_host_driver observer_review/ObserverHostDriver.cpp)
target_link_libraries(observer_host_driver PRIVATE mavmoos_runtime mavmoos_rx)
add_test(NAME observer_host COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/observer_review/ObserverHostTests.py"
  --driver $<TARGET_FILE:observer_host_driver> --telemetry $<TARGET_FILE:mavmoos_telemetry>
  --status $<TARGET_FILE:mavmoos_system_status> --attitude $<TARGET_FILE:mavmoos_attitude>
  --control $<TARGET_FILE:mavmoos_arducopter_control> --parameters $<TARGET_FILE:mavmoos_parameters>
  --rate-fixture $<TARGET_FILE:observer_interval_fixture>)
set_tests_properties(observer_host PROPERTIES TIMEOUT 20)
add_executable(send_boundary_tests send_boundary/SendBoundaryTests.cpp)
target_link_libraries(send_boundary_tests PRIVATE mavmoos_rx Threads::Threads)
add_executable(send_boundary_inert_fixture send_boundary/InertSocketOwner.cpp)
set_target_properties(send_boundary_inert_fixture PROPERTIES OUTPUT_NAME arducopter
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/test-fixtures/send-boundary/copter")
add_test(NAME send_boundary_fake COMMAND send_boundary_tests fake)
add_test(NAME send_boundary_loopback COMMAND send_boundary_tests loopback $<TARGET_FILE:send_boundary_inert_fixture>)
set_tests_properties(send_boundary_fake send_boundary_loopback PROPERTIES TIMEOUT 15 RUN_SERIAL TRUE)
foreach(kind observer legacy)
  add_library(metadata_${kind}_fixture MODULE overnight/MessageMetadataTests.cpp)
  target_compile_definitions(metadata_${kind}_fixture PRIVATE MAVMOOS_METADATA_FIXTURE)
  target_include_directories(metadata_${kind}_fixture PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
  target_link_libraries(metadata_${kind}_fixture PRIVATE mavlink_headers)
endforeach()
target_compile_definitions(metadata_legacy_fixture PRIVATE MAVMOOS_METADATA_LEGACY)
add_executable(message_metadata_tests overnight/MessageMetadataTests.cpp)
target_link_libraries(message_metadata_tests PRIVATE mavmoos_runtime)
add_test(NAME message_metadata COMMAND message_metadata_tests $<TARGET_FILE:metadata_observer_fixture> $<TARGET_FILE:metadata_legacy_fixture>)
add_executable(bounded_runtime_tests overnight/BoundedRuntimeTests.cpp)
target_link_libraries(bounded_runtime_tests PRIVATE mavmoos_runtime)
add_test(NAME bounded_runtime COMMAND bounded_runtime_tests $<TARGET_FILE:mavmoos_arducopter_control>)
add_executable(status_scenario_driver status/StatusScenarioDriver.cpp)
target_include_directories(status_scenario_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(status_scenario_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
add_test(NAME gnss_observations COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/gnss_observations/GnssProviderTests.py"
  --driver $<TARGET_FILE:status_scenario_driver> --module $<TARGET_FILE:mavmoos_system_status>)
set_tests_properties(gnss_observations PROPERTIES TIMEOUT 15)
add_test(NAME status_provider COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/status/StatusProviderTests.py" --driver $<TARGET_FILE:status_scenario_driver>
  --module $<TARGET_FILE:mavmoos_system_status>)
set_tests_properties(status_provider PROPERTIES TIMEOUT 20)
add_executable(status_moos_probe status/StatusMoosProbe.cpp)
target_link_libraries(status_moos_probe PRIVATE ${MOOS_LIBRARIES})
add_test(NAME status_observation_audit COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/status/StatusObservationAuditTests.py")
set_tests_properties(status_observation_audit PROPERTIES TIMEOUT 10)
add_test(NAME status_native_runner_audit COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/status/NativeRunnerReviewTests.py")
set_tests_properties(status_native_runner_audit PROPERTIES TIMEOUT 10)
add_executable(mission_fault_probe mission_faults/MissionFaultProbe.cpp)
target_link_libraries(mission_fault_probe PRIVATE ${MOOS_LIBRARIES})
add_test(NAME mission_fault_evidence COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/mission_faults/MissionFaultEvidenceTests.py")
set_tests_properties(mission_fault_evidence PROPERTIES TIMEOUT 10)
add_test(NAME owned_process_cleanup COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/OwnedProcessesTests.py")
set_tests_properties(owned_process_cleanup PROPERTIES TIMEOUT 10)
add_test(NAME concurrent_heartbeat_audit COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/concurrent/HeartbeatAuditTests.py")
set_tests_properties(concurrent_heartbeat_audit PROPERTIES TIMEOUT 10)
set(MAVMOOS_BEHAVIOR_DIR "$ENV{HOME}/moos-ivp-uav-base/lib" CACHE PATH "Directory containing the ordinary UAV altitude behavior")
foreach(fixture valid missing null collision resource request invalid companion)
  add_library(plugin_fixture_${fixture} MODULE PluginHostTests.cpp)
  target_compile_definitions(plugin_fixture_${fixture} PRIVATE MAVMOOS_TEST_PLUGIN_FIXTURE)
  target_include_directories(plugin_fixture_${fixture} PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
  target_link_libraries(plugin_fixture_${fixture} PRIVATE mavlink_headers)
endforeach()
target_compile_definitions(plugin_fixture_missing PRIVATE MAVMOOS_TEST_MISSING_FACTORY)
target_compile_definitions(plugin_fixture_null PRIVATE MAVMOOS_TEST_NULL_FACTORY)
target_compile_definitions(plugin_fixture_collision PRIVATE MAVMOOS_TEST_CAP_COLLISION)
target_compile_definitions(plugin_fixture_resource PRIVATE MAVMOOS_TEST_RESOURCE_COLLISION)
target_compile_definitions(plugin_fixture_request PRIVATE MAVMOOS_TEST_REQUEST_COLLISION)
target_compile_definitions(plugin_fixture_invalid PRIVATE MAVMOOS_TEST_INVALID_DESCRIPTOR)
target_compile_definitions(plugin_fixture_companion PRIVATE MAVMOOS_TEST_COMPANION)
add_executable(plugin_host_tests PluginHostTests.cpp)
target_link_libraries(plugin_host_tests PRIVATE mavmoos_runtime ${CMAKE_DL_LIBS})
add_test(NAME plugin_host COMMAND plugin_host_tests $<TARGET_FILE:plugin_fixture_valid> $<TARGET_FILE:plugin_fixture_missing> $<TARGET_FILE:plugin_fixture_null> $<TARGET_FILE:plugin_fixture_collision> $<TARGET_FILE:plugin_fixture_resource> $<TARGET_FILE:plugin_fixture_request> $<TARGET_FILE:plugin_fixture_invalid> $<TARGET_FILE:plugin_fixture_companion>)
add_executable(telemetry_plugin_tests TelemetryPluginTests.cpp)
target_link_libraries(telemetry_plugin_tests PRIVATE mavmoos_runtime ${CMAKE_DL_LIBS})
add_test(NAME telemetry_plugin COMMAND telemetry_plugin_tests $<TARGET_FILE:mavmoos_telemetry>)
add_executable(helm_plugin_tests HelmPluginTests.cpp)
target_link_libraries(helm_plugin_tests PRIVATE mavmoos_control_components)
add_test(NAME helm_plugin COMMAND helm_plugin_tests)
add_executable(command_plugin_tests CommandPluginTests.cpp)
target_link_libraries(command_plugin_tests PRIVATE mavmoos_control_components)
add_test(NAME command_plugin COMMAND command_plugin_tests)
add_executable(control_backend_tests control/ControlBackendTests.cpp)
target_link_libraries(control_backend_tests PRIVATE mavmoos_control_components)
add_test(NAME control_backend COMMAND control_backend_tests)
add_executable(module_selection_tests ModuleSelectionTests.cpp)
target_link_libraries(module_selection_tests PRIVATE mavmoos_runtime)
add_test(NAME module_selection COMMAND module_selection_tests)
add_executable(control_plugin_tests control/ControlPluginTests.cpp)
target_link_libraries(control_plugin_tests PRIVATE mavmoos_runtime ${CMAKE_DL_LIBS})
add_test(NAME control_plugin COMMAND control_plugin_tests $<TARGET_FILE:mavmoos_arducopter_control>)
add_executable(copter_operation_tests operations/CopterOperationTests.cpp)
target_link_libraries(copter_operation_tests PRIVATE mavmoos_control_components ${CMAKE_DL_LIBS})
add_test(NAME copter_operations COMMAND copter_operation_tests $<TARGET_FILE:mavmoos_arducopter_control>)
set_tests_properties(copter_operations PROPERTIES TIMEOUT 20)
add_executable(copter_parity_tests copter_parity/CopterParityTests.cpp)
target_link_libraries(copter_parity_tests PRIVATE mavmoos_runtime)
add_test(NAME copter_parity COMMAND copter_parity_tests $<TARGET_FILE:mavmoos_arducopter_control>)
set_tests_properties(copter_parity PROPERTIES TIMEOUT 15)
add_test(NAME copter_operation_evidence_audit COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/operations/HarnessEvidenceTests.py")
set_tests_properties(copter_operation_evidence_audit PROPERTIES TIMEOUT 10)
add_executable(copter_operation_probe operations/CopterOperationProbe.cpp)
target_link_libraries(copter_operation_probe PRIVATE ${MOOS_LIBRARIES})

foreach(fixture provider observer)
  add_library(${fixture}_lifetime_fixture MODULE control/ProviderLifetimeFixtures.cpp)
  target_include_directories(${fixture}_lifetime_fixture PRIVATE
    "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime" "${PROJECT_SOURCE_DIR}/src/libmavmoos_control")
  target_link_libraries(${fixture}_lifetime_fixture PRIVATE mavlink_headers)
endforeach()
target_compile_definitions(provider_lifetime_fixture PRIVATE MAVMOOS_REPLACEABLE_PROVIDER)
target_link_libraries(provider_lifetime_fixture PRIVATE mavmoos_control_components)
add_executable(provider_lifetime_tests control/ProviderLifetimeTests.cpp)
target_link_libraries(provider_lifetime_tests PRIVATE mavmoos_runtime)
add_test(NAME provider_lifetime COMMAND provider_lifetime_tests $<TARGET_FILE:provider_lifetime_fixture> $<TARGET_FILE:observer_lifetime_fixture>)

add_executable(parameter_plugin_tests ParameterPluginTests.cpp ../src/plugins/ParameterPlugin.cpp)
target_include_directories(parameter_plugin_tests PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(parameter_plugin_tests PRIVATE mavlink_headers)
add_test(NAME parameter_plugin COMMAND parameter_plugin_tests)
add_executable(mission_provider_tests missions/MissionPluginTests.cpp ../src/plugins/MissionPlugin.cpp)
target_include_directories(mission_provider_tests PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(mission_provider_tests PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
add_test(NAME mission_provider COMMAND mission_provider_tests $<TARGET_FILE:mavmoos_missions>)
add_executable(mission_inspection_probe missions/InspectionProbe.cpp)
target_link_libraries(mission_inspection_probe PRIVATE mavlink_headers ${MOOS_LIBRARIES})
add_test(NAME mission_inspection_loopback COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/missions/run_inspection.py" --mode loopback
  --moos-root "${MOOS_IVP_ROOT}" --build "${CMAKE_BINARY_DIR}" --probe $<TARGET_FILE:mission_inspection_probe>
  --output "${CMAKE_BINARY_DIR}/evidence/mission-inspection-loopback")
set_tests_properties(mission_inspection_loopback PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE LABELS "loopback")

add_test(NAME mission_request_helper COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/missions/MissionRequestTests.py")
add_test(NAME mission_evidence_helper COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/missions/MissionEvidenceTests.py")
add_test(NAME mission_read_evidence COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/missions/ConcurrentReadEvidenceTests.py")
set_tests_properties(mission_read_evidence PROPERTIES TIMEOUT 10)
add_executable(mission_moos_probe missions/MissionMoosProbe.cpp)
target_link_libraries(mission_moos_probe PRIVATE ${MOOS_LIBRARIES})
add_executable(parameter_moos_probe parameters/ParameterMoosProbe.cpp)
target_link_libraries(parameter_moos_probe PRIVATE ${MOOS_LIBRARIES})
add_executable(fault_probe faults/FaultProbe.cpp)
target_link_libraries(fault_probe PRIVATE ${MOOS_LIBRARIES})
add_executable(sitl_workflow_probe SitlWorkflowProbe.cpp)
target_link_libraries(sitl_workflow_probe PRIVATE ${MOOS_LIBRARIES})
add_executable(runtime_tests RuntimeTests.cpp)
target_link_libraries(runtime_tests PRIVATE mavmoos_runtime)
add_test(NAME runtime COMMAND runtime_tests $<TARGET_FILE:mavmoos_arducopter_control>)
add_executable(mission_runtime_tests MissionRuntimeTests.cpp)
target_link_libraries(mission_runtime_tests PRIVATE mavmoos_runtime)
add_test(NAME mission_runtime COMMAND mission_runtime_tests $<TARGET_FILE:mavmoos_arducopter_control>)
add_executable(projection_service_tests ProjectionServiceTests.cpp)
target_link_libraries(projection_service_tests PRIVATE mavmoos_runtime)
add_test(NAME projection_service COMMAND projection_service_tests $<TARGET_FILE:mavmoos_telemetry>)
add_executable(ingress_runtime_tests IngressRuntimeTests.cpp)
target_link_libraries(ingress_runtime_tests PRIVATE mavmoos_runtime)
add_test(NAME ingress_runtime COMMAND ingress_runtime_tests $<TARGET_FILE:mavmoos_parameters> $<TARGET_FILE:mavmoos_arducopter_control>)
add_executable(scheduling_tests scheduling/SchedulingTests.cpp)
target_link_libraries(scheduling_tests PRIVATE mavmoos_runtime)
add_test(NAME scheduling COMMAND scheduling_tests $<TARGET_FILE:mavmoos_arducopter_control> $<TARGET_FILE:mavmoos_parameters>)
add_executable(resume_permit_review_tests scheduling/ResumePermitDiagnostic.cpp)
target_link_libraries(resume_permit_review_tests PRIVATE mavmoos_runtime)
add_test(NAME resume_permit_review COMMAND resume_permit_review_tests $<TARGET_FILE:mavmoos_arducopter_control>)
add_executable(authority_review_tests concurrency_review/AuthorityConflictDiagnostic.cpp)
target_link_libraries(authority_review_tests PRIVATE mavmoos_runtime)
add_test(NAME authority_review COMMAND authority_review_tests $<TARGET_FILE:mavmoos_arducopter_control>)
add_executable(write_interlock_tests concurrency_review/WriteInterlockDiagnostic.cpp)
target_link_libraries(write_interlock_tests PRIVATE mavmoos_runtime)
add_test(NAME write_interlock COMMAND write_interlock_tests $<TARGET_FILE:mavmoos_arducopter_control>)
add_executable(concurrent_moos_probe concurrent/ConcurrentMoosProbe.cpp)
target_link_libraries(concurrent_moos_probe PRIVATE ${MOOS_LIBRARIES})
add_executable(ingress_tests ingress/IngressTests.cpp)
target_include_directories(ingress_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(ingress_tests PRIVATE mavmoos_rx)
target_compile_options(ingress_tests PRIVATE -Wall -Wextra -Wpedantic)
foreach(scenario policy loopback matrix)
  add_test(NAME ingress_${scenario} COMMAND ingress_tests ${scenario})
  set_tests_properties(ingress_${scenario} PROPERTIES TIMEOUT 15 RUN_SERIAL TRUE)
endforeach()
add_test(NAME stock_helm COMMAND "${Python3_EXECUTABLE}"
  "${PROJECT_SOURCE_DIR}/missions/copter_sitl/check_stock_helm.py"
  --output "${CMAKE_BINARY_DIR}/evidence/stock-helm" --moos-bin "${MOOS_IVP_ROOT}/bin"
  --behavior-dir "${MAVMOOS_BEHAVIOR_DIR}")
set_tests_properties(stock_helm PROPERTIES TIMEOUT 25 RUN_SERIAL TRUE)
add_test(NAME stock_helm_operations COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/operations/check_stock_operations.py"
  --output "${CMAKE_BINARY_DIR}/evidence/stock-operations" --moos-bin "${MOOS_IVP_ROOT}/bin"
  --behavior-dir "${MAVMOOS_BEHAVIOR_DIR}")
set_tests_properties(stock_helm_operations PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
add_executable(shadow_moos_probe ShadowMoosProbe.cpp)
target_link_libraries(shadow_moos_probe PRIVATE mavmoos_rx ${MOOS_LIBRARIES})
add_test(NAME moos_shadow COMMAND "${Python3_EXECUTABLE}"
  "${PROJECT_SOURCE_DIR}/tests/run_shadow_moos.py" --app "$<TARGET_FILE:iMAVLink>"
  --probe "$<TARGET_FILE:shadow_moos_probe>" --moos-bin "${MOOS_IVP_ROOT}/bin"
  --plugin-dir "${CMAKE_BINARY_DIR}/plugins" --behavior-dir "${MAVMOOS_BEHAVIOR_DIR}"
  --output "${CMAKE_BINARY_DIR}/evidence/shadow")
set_tests_properties(moos_shadow PROPERTIES TIMEOUT 25 RUN_SERIAL TRUE)
set(MAVMOOS_ARDUPILOT_ROOT "" CACHE PATH "Optional existing native ArduPilot SITL checkout")
set(MAVMOOS_SITL_PYTHON "" CACHE FILEPATH "Python with pymavlink for the optional full SITL test")
if(MAVMOOS_ARDUPILOT_ROOT AND MAVMOOS_SITL_PYTHON)
  add_test(NAME mission_native_inspection COMMAND "${MAVMOOS_SITL_PYTHON}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/missions/run_inspection.py" --mode native
    --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}" --moos-root "${MOOS_IVP_ROOT}"
    --build "${CMAKE_BINARY_DIR}" --probe $<TARGET_FILE:mission_inspection_probe>
    --output "${CMAKE_BINARY_DIR}/evidence/mission-inspection-native")
  set_tests_properties(mission_native_inspection PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE LABELS "sitl;missions")
  foreach(profile status attitude)
    add_test(NAME observation_load_${profile} COMMAND "${MAVMOOS_SITL_PYTHON}" -B
      "${CMAKE_CURRENT_SOURCE_DIR}/observation_load/run_observation_load.py"
      --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}" --moos-root "${MOOS_IVP_ROOT}"
      --behavior-dir "${MAVMOOS_BEHAVIOR_DIR}" --build "${CMAKE_BINARY_DIR}"
      --source-root "${PROJECT_SOURCE_DIR}" --probe "$<TARGET_FILE:observation_load_probe>"
      --profile ${profile} --output "${CMAKE_BINARY_DIR}/evidence/observation-load-${profile}")
    # Real-time GPS/EKF convergence, existing flight workflow, original60 s
    # indexed inventory and bounded landing require a longer native window.
    set_tests_properties(observation_load_${profile} PROPERTIES TIMEOUT 280 RUN_SERIAL TRUE LABELS "sitl;observations;concurrent")
  endforeach()
  foreach(scenario cancel reset)
    add_test(NAME mission_native_${scenario} COMMAND "${MAVMOOS_SITL_PYTHON}" -B
      "${CMAKE_CURRENT_SOURCE_DIR}/mission_faults/run_mission_faults.py"
      --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}" --moos-root "${MOOS_IVP_ROOT}"
      --source-root "${PROJECT_SOURCE_DIR}" --build "${CMAKE_BINARY_DIR}"
      --probe "$<TARGET_FILE:mission_fault_probe>" --scenario ${scenario}
      --output "${CMAKE_BINARY_DIR}/evidence/mission-${scenario}")
    # Disarmed HOME initialization, partial transfer, the pinned >8 s remote
    # protocol-settlement wait, readback and cleanup all run at timewarp 1.
    set_tests_properties(mission_native_${scenario} PROPERTIES TIMEOUT 200 RUN_SERIAL TRUE LABELS "sitl;missions;faults")
  endforeach()
  add_test(NAME status_native_observations COMMAND "${MAVMOOS_SITL_PYTHON}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/status/run_status_observations.py"
    --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}" --moos-root "${MOOS_IVP_ROOT}"
    --source-root "${PROJECT_SOURCE_DIR}" --build "${CMAKE_BINARY_DIR}"
    --probe "$<TARGET_FILE:status_moos_probe>"
    --output "${CMAKE_BINARY_DIR}/evidence/status-observations")
  # Bounded20 s discovery +25 s read-only probe + shutdown/independent log audit.
  set_tests_properties(status_native_observations PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE LABELS "sitl;observations")
  add_test(NAME mission_native_transfer COMMAND "${MAVMOOS_SITL_PYTHON}"
    "${CMAKE_CURRENT_SOURCE_DIR}/missions/run_missions.py" --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}"
    --moos-root "${MOOS_IVP_ROOT}" --build "${CMAKE_BINARY_DIR}"
    --output "${CMAKE_BINARY_DIR}/evidence/mission-transfer")
  set_tests_properties(mission_native_transfer PROPERTIES TIMEOUT 180 RUN_SERIAL TRUE LABELS "sitl;missions")
  add_test(NAME copter_native_operations COMMAND "${MAVMOOS_SITL_PYTHON}"
    "${CMAKE_CURRENT_SOURCE_DIR}/operations/run_operations.py" --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}"
    --moos-root "${MOOS_IVP_ROOT}" --build "${CMAKE_BINARY_DIR}" --behavior-dir "${MAVMOOS_BEHAVIOR_DIR}"
    --output "${CMAKE_BINARY_DIR}/evidence/copter-operations")
  # Bounded native GPS/EKF convergence, several real-time motion strategies,
  # and a return/landing trajectory require a longer integration deadline.
  set_tests_properties(copter_native_operations PROPERTIES TIMEOUT 360 RUN_SERIAL TRUE LABELS "sitl;operations")
  add_test(NAME copter_sitl_workflow COMMAND "${MAVMOOS_SITL_PYTHON}"
    "${PROJECT_SOURCE_DIR}/tests/run_sitl.py" --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}"
    --moos-root "${MOOS_IVP_ROOT}" --build "${CMAKE_BINARY_DIR}" --behavior-dir "${MAVMOOS_BEHAVIOR_DIR}"
    --output "${CMAKE_BINARY_DIR}/evidence/copter-sitl")
  set_tests_properties(copter_sitl_workflow PROPERTIES TIMEOUT 240 RUN_SERIAL TRUE LABELS sitl)
  add_test(NAME parameters_sitl COMMAND "${MAVMOOS_SITL_PYTHON}"
    "${PROJECT_SOURCE_DIR}/tests/parameters/run_parameters.py" --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}"
    --moos-root "${MOOS_IVP_ROOT}" --build "${CMAKE_BINARY_DIR}"
    --output "${CMAKE_BINARY_DIR}/evidence/parameters-sitl")
  set_tests_properties(parameters_sitl PROPERTIES TIMEOUT 180 RUN_SERIAL TRUE LABELS sitl)
  foreach(scenario bulk mission_default bridge_pause mission_read)
    add_test(NAME concurrent_${scenario} COMMAND "${MAVMOOS_SITL_PYTHON}"
      "${PROJECT_SOURCE_DIR}/tests/concurrent/run_concurrent.py" --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}"
      --moos-root "${MOOS_IVP_ROOT}" --build "${CMAKE_BINARY_DIR}" --probe "$<TARGET_FILE:concurrent_moos_probe>"
      --behavior-dir "${MAVMOOS_BEHAVIOR_DIR}" --scenario "${scenario}" --output "${CMAKE_BINARY_DIR}/evidence/concurrent-${scenario}")
    set_tests_properties(concurrent_${scenario} PROPERTIES TIMEOUT 240 RUN_SERIAL TRUE LABELS "sitl;concurrent")
  endforeach()
  foreach(scenario helm_pause moosdb_restart simulator_pause simulator_restart command_timeout bridge_pause)
    add_test(NAME fault_${scenario} COMMAND "${MAVMOOS_SITL_PYTHON}"
      "${PROJECT_SOURCE_DIR}/tests/faults/run_faults.py" --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}"
      --moos-root "${MOOS_IVP_ROOT}" --build "${CMAKE_BINARY_DIR}" --probe "$<TARGET_FILE:fault_probe>"
      --behavior-dir "${MAVMOOS_BEHAVIOR_DIR}" --scenario "${scenario}"
      --output "${CMAKE_BINARY_DIR}/evidence/fault-${scenario}")
    set_tests_properties(fault_${scenario} PROPERTIES TIMEOUT 240 RUN_SERIAL TRUE LABELS "sitl;faults")
  endforeach()
endif()
add_executable(egress_fake_tests EgressFakeTests.cpp)
target_link_libraries(egress_fake_tests PRIVATE mavmoos_egress)
target_compile_options(egress_fake_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME egress_fake COMMAND egress_fake_tests "${CMAKE_BINARY_DIR}/evidence/p2-fake.jsonl")
set_tests_properties(egress_fake PROPERTIES TIMEOUT 15)
add_executable(egress_loopback_tests EgressLoopbackTests.cpp)
target_link_libraries(egress_loopback_tests PRIVATE mavmoos_egress)
target_compile_options(egress_loopback_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME egress_loopback COMMAND egress_loopback_tests "${CMAKE_BINARY_DIR}/evidence/p2-loopback.jsonl")
set_tests_properties(egress_loopback PROPERTIES TIMEOUT 10 RUN_SERIAL TRUE)
add_executable(rx_tests RxTests.cpp)
target_link_libraries(rx_tests PRIVATE mavmoos_rx)
target_compile_options(rx_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME rx_core COMMAND rx_tests)
set_tests_properties(rx_core PROPERTIES TIMEOUT 10)
foreach(option help example interface)
  add_test(NAME app_${option} COMMAND iMAVLink --${option})
endforeach()
add_executable(moos_rx_probe MoosRxProbe.cpp)
target_link_libraries(moos_rx_probe PRIVATE mavmoos_rx ${MOOS_LIBRARIES})
target_compile_options(moos_rx_probe PRIVATE -Wall -Wextra -Wpedantic)
foreach(scenario normal invalid_config invalid_streamed invalid_auto_recovery)
  add_test(NAME moos_${scenario} COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/run_moos.py"
    --moos-bin "${MOOS_IVP_ROOT}/bin" --app "$<TARGET_FILE:iMAVLink>"
    --probe "$<TARGET_FILE:moos_rx_probe>" --scenario ${scenario}
    --output "${CMAKE_BINARY_DIR}/evidence/${scenario}")
  set_tests_properties(moos_${scenario} PROPERTIES TIMEOUT 25 RUN_SERIAL TRUE)
endforeach()

add_subdirectory(rover)
add_subdirectory(external_command)
add_subdirectory(auto_recovery)
add_subdirectory(flight_logs)
add_subdirectory(mavlink_files)
add_subdirectory(debug_values)
add_subdirectory(event_timeline)
add_subdirectory(water_depth)

add_subdirectory(plane_tasks)

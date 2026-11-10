add_executable(flight_log_protocol_tests FlightLogProtocolTests.cpp)
target_link_libraries(flight_log_protocol_tests PRIVATE mavmoos_runtime)
add_test(NAME flight_logs_faults COMMAND flight_log_protocol_tests $<TARGET_FILE:mavmoos_flight_logs>)
set_tests_properties(flight_logs_faults PROPERTIES TIMEOUT 15)

add_executable(flight_logs_moos_probe FlightLogsMoosProbe.cpp)
target_link_libraries(flight_logs_moos_probe PRIVATE ${MOOS_LIBRARIES} mavlink_headers)
add_test(NAME flight_logs_rx_moos COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/run.py" --scenario rx --moos-root "${MOOS_IVP_ROOT}"
  --build "${CMAKE_BINARY_DIR}" --probe $<TARGET_FILE:flight_logs_moos_probe>
  --output-parent "${CMAKE_BINARY_DIR}/evidence/flight-logs-rx")
set_tests_properties(flight_logs_rx_moos PROPERTIES TIMEOUT 25 RUN_SERIAL TRUE LABELS "loopback")

if(MAVMOOS_ARDUPILOT_ROOT AND MAVMOOS_SITL_PYTHON)
  add_test(NAME flight_logs_native_copter COMMAND "${MAVMOOS_SITL_PYTHON}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/run.py" --scenario native --moos-root "${MOOS_IVP_ROOT}"
    --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}" --build "${CMAKE_BINARY_DIR}"
    --probe $<TARGET_FILE:flight_logs_moos_probe>
    --output "${CMAKE_BINARY_DIR}/evidence/flight-logs-native")
  # A bounded disarmed seed boot, a second exact native process, and one file workflow.
  set_tests_properties(flight_logs_native_copter PROPERTIES TIMEOUT 150 RUN_SERIAL TRUE LABELS "sitl;flight_logs")
endif()

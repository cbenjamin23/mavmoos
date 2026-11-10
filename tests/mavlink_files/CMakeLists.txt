add_executable(mavlink_ftp_transaction_tests "${PROJECT_SOURCE_DIR}/tests/ftp_protocol/FtpTransactionTests.cpp")
target_link_libraries(mavlink_ftp_transaction_tests PRIVATE mavmoos_ftp_core)
target_compile_options(mavlink_ftp_transaction_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME mavlink_files_transaction COMMAND mavlink_ftp_transaction_tests)
set_tests_properties(mavlink_files_transaction PROPERTIES TIMEOUT 12)

add_executable(mavlink_ftp_runtime_tests FilesRuntimeTests.cpp)
target_link_libraries(mavlink_ftp_runtime_tests PRIVATE mavmoos_runtime)
target_compile_options(mavlink_ftp_runtime_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME mavlink_files_runtime COMMAND mavlink_ftp_runtime_tests
  $<TARGET_FILE:mavmoos_telemetry> $<TARGET_FILE:mavmoos_mavlink_files>)
set_tests_properties(mavlink_files_runtime PROPERTIES TIMEOUT 12)

add_executable(owned_ftp_root_tests OwnedFtpRootTests.cpp "${PROJECT_SOURCE_DIR}/src/iMAVLink/OwnedFtpRoot.cpp")
target_include_directories(owned_ftp_root_tests PRIVATE "${PROJECT_SOURCE_DIR}/src/iMAVLink")
target_link_libraries(owned_ftp_root_tests PRIVATE mavmoos_ftp_core)
target_compile_options(owned_ftp_root_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME mavlink_ftp_owned_root COMMAND owned_ftp_root_tests)
set_tests_properties(mavlink_ftp_owned_root PROPERTIES TIMEOUT 12)

add_executable(mavlink_files_moos_probe FilesMoosProbe.cpp)
target_link_libraries(mavlink_files_moos_probe PRIVATE ${MOOS_LIBRARIES})
target_compile_options(mavlink_files_moos_probe PRIVATE -Wall -Wextra -Wpedantic)
add_library(mavlink_files_wire_tap SHARED OwnedUdpTap.cpp)
set_target_properties(mavlink_files_wire_tap PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(mavlink_files_wire_tap PRIVATE ${CMAKE_DL_LIBS})
target_compile_options(mavlink_files_wire_tap PRIVATE -Wall -Wextra -Wpedantic)
if(MAVMOOS_ARDUPILOT_ROOT AND MAVMOOS_SITL_PYTHON)
  add_test(NAME mavlink_files_native_copter COMMAND "${MAVMOOS_SITL_PYTHON}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/run_native.py" --source-root "${PROJECT_SOURCE_DIR}"
    --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}" --moos-root "${MOOS_IVP_ROOT}" --build "${CMAKE_BINARY_DIR}"
    --probe $<TARGET_FILE:mavlink_files_moos_probe> --tap $<TARGET_FILE:mavlink_files_wire_tap>
    --output "${CMAKE_BINARY_DIR}/evidence/mavlink-files-native")
  set_tests_properties(mavlink_files_native_copter PROPERTIES TIMEOUT 160 RUN_SERIAL TRUE LABELS "sitl;mavlink_files")
endif()

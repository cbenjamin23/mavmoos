add_executable(observer_matrix_probe Probe.cpp)
target_link_libraries(observer_matrix_probe PRIVATE ${MOOS_LIBRARIES})
add_executable(observer_matrix_frames Frames.cpp)
target_include_directories(observer_matrix_frames PRIVATE "${PROJECT_SOURCE_DIR}/tests")
target_link_libraries(observer_matrix_frames PRIVATE mavlink_headers)
add_executable(observer_extensions_frames ExtensionFrames.cpp)
target_include_directories(observer_extensions_frames PRIVATE "${PROJECT_SOURCE_DIR}/tests"
  "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(observer_extensions_frames PRIVATE mavlink_headers)
add_test(NAME observer_matrix_native_audit COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/NativeAuditTests.py")
set_tests_properties(observer_matrix_native_audit PROPERTIES TIMEOUT 15)

foreach(identity ardupilot px4)
  add_test(NAME observer_matrix_${identity} COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/run_loopback.py" --identity ${identity}
    --moos-bin "${MOOS_IVP_ROOT}/bin" --build "${CMAKE_BINARY_DIR}"
    --frames $<TARGET_FILE:observer_matrix_frames> --probe $<TARGET_FILE:observer_matrix_probe>
    --output "${CMAKE_BINARY_DIR}/evidence/observer-matrix-${identity}")
  set_tests_properties(observer_matrix_${identity} PROPERTIES TIMEOUT 25 RUN_SERIAL TRUE LABELS "loopback")
endforeach()

foreach(identity ardupilot px4 plane boat)
  add_test(NAME observer_extensions_${identity} COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/run_extensions.py" --identity ${identity}
    --moos-bin "${MOOS_IVP_ROOT}/bin" --build "${CMAKE_BINARY_DIR}"
    --frames $<TARGET_FILE:observer_extensions_frames> --probe $<TARGET_FILE:observer_matrix_probe>
    --output "${CMAKE_BINARY_DIR}/evidence/observer-extensions-${identity}")
  set_tests_properties(observer_extensions_${identity} PROPERTIES TIMEOUT 27 RUN_SERIAL TRUE LABELS "loopback")
endforeach()

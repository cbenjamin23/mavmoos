add_executable(gimbal_status_driver GimbalScenarioDriver.cpp)
target_include_directories(gimbal_status_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(gimbal_status_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(gimbal_status_driver PRIVATE -Wall -Wextra -Wpedantic)
add_executable(gimbal_status_moos_probe GimbalMoosProbe.cpp)
target_link_libraries(gimbal_status_moos_probe PRIVATE ${MOOS_LIBRARIES})
add_test(NAME gimbal_status_observations COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/GimbalProviderTests.py"
  --driver $<TARGET_FILE:gimbal_status_driver> --module $<TARGET_FILE:mavmoos_gimbal_status>)
set_tests_properties(gimbal_status_observations PROPERTIES TIMEOUT 15)

add_test(NAME payload_observers_loopback COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/run_loopback.py"
  --app $<TARGET_FILE:iMAVLink> --probe $<TARGET_FILE:gimbal_status_moos_probe>
  --driver $<TARGET_FILE:gimbal_status_driver> --camera-frames $<TARGET_FILE:camera_status_frames>
  --moos-root "${MOOS_IVP_ROOT}" --plugin-directory "${CMAKE_BINARY_DIR}/plugins"
  --output-parent "${CMAKE_BINARY_DIR}/evidence/payload-observers")
set_tests_properties(payload_observers_loopback PROPERTIES TIMEOUT 28 RUN_SERIAL TRUE LABELS "loopback")

add_executable(observer_extensions_native_probe ObserverExtensionsProbe.cpp)
target_link_libraries(observer_extensions_native_probe PRIVATE ${MOOS_LIBRARIES})

if(EXISTS "${MAVMOOS_ARDUPILOT_ROOT}/build/sitl/bin/arducopter" AND MAVMOOS_SITL_PYTHON)
  add_test(NAME observer_extensions_native COMMAND "${MAVMOOS_SITL_PYTHON}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/run_native.py" --source-root "${PROJECT_SOURCE_DIR}"
    --ardupilot-root "${MAVMOOS_ARDUPILOT_ROOT}" --moos-root "${MOOS_IVP_ROOT}"
    --build "${CMAKE_BINARY_DIR}" --probe $<TARGET_FILE:observer_extensions_native_probe>
    --output "${CMAKE_BINARY_DIR}/evidence/observer-extensions-native")
  set_tests_properties(observer_extensions_native PROPERTIES TIMEOUT 170 RUN_SERIAL TRUE LABELS "sitl;observations")
endif()

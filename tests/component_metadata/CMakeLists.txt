add_executable(component_metadata_tests ComponentMetadataTests.cpp)
target_include_directories(component_metadata_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime" "${PROJECT_SOURCE_DIR}/tests/mode_catalog")
target_link_libraries(component_metadata_tests PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(component_metadata_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME component_metadata_observations COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/ComponentMetadataTests.py"
  $<TARGET_FILE:component_metadata_tests> $<TARGET_FILE:mavmoos_component_metadata>)
set_tests_properties(component_metadata_observations PROPERTIES TIMEOUT 15)
add_test(NAME component_metadata_loopback COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/run_metadata.py" --build "${CMAKE_BINARY_DIR}"
  --moos-bin "${MOOS_IVP_ROOT}/bin" --probe $<TARGET_FILE:observer_matrix_probe>
  --output "${CMAKE_BINARY_DIR}/evidence/component-metadata")
set_tests_properties(component_metadata_loopback PROPERTIES TIMEOUT 32 RUN_SERIAL TRUE LABELS loopback)

if(APPLE OR CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_library(mavmoos_plane_fault_tap SHARED OwnedPlaneFaultTap.cpp)
  set_target_properties(mavmoos_plane_fault_tap PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
  target_link_libraries(mavmoos_plane_fault_tap PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
  target_compile_options(mavmoos_plane_fault_tap PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(mavmoos_plane_fault_probe FaultTapProbe.cpp)
  target_link_libraries(mavmoos_plane_fault_probe PRIVATE mavlink_headers)
  target_compile_options(mavmoos_plane_fault_probe PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME plane_fault_tap_loopback COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/FaultTapTests.py"
    $<TARGET_FILE:mavmoos_plane_fault_probe> $<TARGET_FILE:mavmoos_plane_fault_tap>)
  set_tests_properties(plane_fault_tap_loopback PROPERTIES TIMEOUT 10)
endif()

add_test(NAME plane_firmware_manifest COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/PlaneFirmwareTests.py")
set_tests_properties(plane_firmware_manifest PROPERTIES TIMEOUT 10)

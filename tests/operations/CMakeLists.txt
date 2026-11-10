cmake_minimum_required(VERSION 3.24)
project(MAVMOOSCopterOperations LANGUAGES CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
if(NOT MAVLINK_DIR)
  message(FATAL_ERROR "Supply MAVLINK_DIR containing generated ardupilotmega/mavlink.h")
endif()
add_library(operation_components STATIC
  "${REPO_ROOT}/src/libmavmoos_control/CopterOperations.cpp"
  "${REPO_ROOT}/src/libmavmoos_control/CopterCommands.cpp"
  "${REPO_ROOT}/src/libmavmoos_control/HelmAdapter.cpp"
  "${REPO_ROOT}/src/libmavmoos_control/ArduCopterBackend.cpp"
  "${REPO_ROOT}/src/libmavmoos_egress/CopterModel.cpp")
set_target_properties(operation_components PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(operation_components SYSTEM PUBLIC "${MAVLINK_DIR}")
target_include_directories(operation_components PUBLIC
  "${REPO_ROOT}/src/libmavmoos_runtime" "${REPO_ROOT}/src/libmavmoos_control" "${REPO_ROOT}/src/libmavmoos_egress")
target_compile_options(operation_components PRIVATE -Wall -Wextra -Wpedantic)
add_library(operation_control MODULE "${REPO_ROOT}/src/plugins/ArduCopterControlPlugin.cpp")
target_link_libraries(operation_control PRIVATE operation_components)
target_compile_options(operation_control PRIVATE -Wall -Wextra -Wpedantic)
add_executable(copter_operation_tests CopterOperationTests.cpp)
target_link_libraries(copter_operation_tests PRIVATE operation_components ${CMAKE_DL_LIBS})
target_compile_options(copter_operation_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME copter_operations COMMAND copter_operation_tests $<TARGET_FILE:operation_control>)
set_tests_properties(copter_operations PROPERTIES TIMEOUT 20)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME copter_operation_evidence_audit COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/HarnessEvidenceTests.py")
set_tests_properties(copter_operation_evidence_audit PROPERTIES TIMEOUT 10)
if(MOOS_IVP_ROOT)
  find_package(MOOS 10.0 REQUIRED CONFIG PATHS
    "${MOOS_IVP_ROOT}/build/MOOS/MOOSCore" "${MOOS_IVP_ROOT}/build/MOOS" NO_DEFAULT_PATH)
  add_executable(copter_operation_probe CopterOperationProbe.cpp)
  target_link_libraries(copter_operation_probe PRIVATE ${MOOS_LIBRARIES})
  target_compile_options(copter_operation_probe PRIVATE -Wall -Wextra -Wpedantic)
endif()

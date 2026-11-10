cmake_minimum_required(VERSION 3.24)
project(MAVMOOSConcurrentCore LANGUAGES CXX)
enable_testing()
include("${REPO_ROOT}/tests/ingress_review/LinuxCore.cmake")

if(EXISTS "${REPO_ROOT}/tests/concurrent/HeartbeatAuditTests.py")
  add_test(NAME concurrent_heartbeat_audit COMMAND "${Python3_EXECUTABLE}" "${REPO_ROOT}/tests/concurrent/HeartbeatAuditTests.py")
  set_tests_properties(concurrent_heartbeat_audit PROPERTIES TIMEOUT 10)
endif()

add_executable(authority_review_tests "${REPO_ROOT}/tests/concurrency_review/AuthorityConflictDiagnostic.cpp")
target_link_libraries(authority_review_tests PRIVATE mavmoos_runtime)
add_test(NAME authority_review COMMAND authority_review_tests $<TARGET_FILE:mavmoos_arducopter_control>)

add_executable(scheduling_tests "${REPO_ROOT}/tests/scheduling/SchedulingTests.cpp")
target_link_libraries(scheduling_tests PRIVATE mavmoos_runtime)
add_test(NAME scheduling COMMAND scheduling_tests $<TARGET_FILE:mavmoos_arducopter_control> $<TARGET_FILE:mavmoos_parameters>)

add_executable(write_interlock_review "${REPO_ROOT}/tests/concurrency_review/WriteInterlockDiagnostic.cpp")
target_link_libraries(write_interlock_review PRIVATE mavmoos_runtime)
add_test(NAME write_interlock_review COMMAND write_interlock_review $<TARGET_FILE:mavmoos_arducopter_control>)

add_executable(resume_permit_review "${REPO_ROOT}/tests/scheduling/ResumePermitDiagnostic.cpp")
target_link_libraries(resume_permit_review PRIVATE mavmoos_runtime)
add_test(NAME resume_permit_review COMMAND resume_permit_review $<TARGET_FILE:mavmoos_arducopter_control>)
set_tests_properties(authority_review scheduling write_interlock_review resume_permit_review PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)

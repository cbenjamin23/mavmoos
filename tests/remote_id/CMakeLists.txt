add_executable(remote_id_tests RemoteIdTests.cpp)
target_include_directories(remote_id_tests PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(remote_id_tests PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(remote_id_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME remote_id_observations COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/RemoteIdTests.py"
  $<TARGET_FILE:remote_id_tests> $<TARGET_FILE:mavmoos_remote_id>)
set_tests_properties(remote_id_observations PROPERTIES TIMEOUT 12)

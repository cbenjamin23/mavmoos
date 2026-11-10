add_executable(gimbal_operations_driver GimbalOperationsDriver.cpp)
target_include_directories(gimbal_operations_driver PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(gimbal_operations_driver PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(gimbal_operations_driver PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME gimbal_point_operations COMMAND $<TARGET_FILE:gimbal_operations_driver>
  $<TARGET_FILE:mavmoos_gimbal_status>)
set_tests_properties(gimbal_point_operations PROPERTIES TIMEOUT 10)

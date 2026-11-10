add_executable(fence_operations_tests FenceOperationsTests.cpp)
target_link_libraries(fence_operations_tests PRIVATE mavmoos_runtime)
add_test(NAME fence_operations COMMAND fence_operations_tests $<TARGET_FILE:mavmoos_missions>)
set_tests_properties(fence_operations PROPERTIES TIMEOUT 15)

if(MOOS_LIBRARIES)
  add_executable(fence_moos_probe FenceMoosProbe.cpp)
  target_link_libraries(fence_moos_probe PRIVATE ${MOOS_LIBRARIES})
endif()

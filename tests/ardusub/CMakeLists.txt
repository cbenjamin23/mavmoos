add_executable(ardusub_control_tests ArduSubControlTests.cpp)
target_link_libraries(ardusub_control_tests PRIVATE mavmoos_runtime)
add_test(NAME ardusub_control COMMAND ardusub_control_tests $<TARGET_FILE:mavmoos_ardusub_control>)
set_tests_properties(ardusub_control PROPERTIES TIMEOUT 15)

add_executable(ardusub_moos_probe ArduSubMoosProbe.cpp)
target_link_libraries(ardusub_moos_probe PRIVATE ${MOOS_LIBRARIES})
target_compile_options(ardusub_moos_probe PRIVATE -Wall -Wextra -Wpedantic)

add_executable(depth_autonomy_probe ArduSubMoosProbe.cpp)
target_link_libraries(depth_autonomy_probe PRIVATE ${MOOS_LIBRARIES})

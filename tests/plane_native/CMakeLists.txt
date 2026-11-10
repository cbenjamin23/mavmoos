add_executable(plane_native_probe PlaneMoosProbe.cpp)
target_link_libraries(plane_native_probe PRIVATE ${MOOS_LIBRARIES})

add_executable(plane_autonomy_probe PlaneMoosProbe.cpp)
target_link_libraries(plane_autonomy_probe PRIVATE ${MOOS_LIBRARIES})

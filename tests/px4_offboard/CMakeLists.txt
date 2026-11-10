add_executable(px4_offboard_moos_probe Px4MoosProbe.cpp)
target_link_libraries(px4_offboard_moos_probe PRIVATE mavlink_headers ${MOOS_LIBRARIES})
target_compile_options(px4_offboard_moos_probe PRIVATE -Wall -Wextra -Wpedantic)

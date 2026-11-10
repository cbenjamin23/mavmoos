# P2 lab component. Deliberately NOT linked into iMAVLink.
add_library(mavmoos_egress CopterModel.cpp ManagedEgress.cpp UdpLoopbackIo.cpp)
target_include_directories(mavmoos_egress PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(mavmoos_egress PUBLIC mavlink_headers Threads::Threads)
target_compile_options(mavmoos_egress PRIVATE -Wall -Wextra -Wpedantic)

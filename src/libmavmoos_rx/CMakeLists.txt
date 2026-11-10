add_library(mavmoos_rx Config.cpp UdpReceiver.cpp Telemetry.cpp SimulationSendGate.cpp)
target_include_directories(mavmoos_rx PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(mavmoos_rx PUBLIC mavlink_headers)
target_compile_options(mavmoos_rx PRIVATE -Wall -Wextra -Wpedantic)

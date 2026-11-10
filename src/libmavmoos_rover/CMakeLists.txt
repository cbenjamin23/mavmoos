add_library(mavmoos_rover_components STATIC ArduRoverBackend.cpp RoverCommands.cpp RoverMissionEvidence.cpp ArduRoverMission.cpp)
set_target_properties(mavmoos_rover_components PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mavmoos_rover_components PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(mavmoos_rover_components PUBLIC mavlink_headers PRIVATE mavmoos_replay)
target_compile_options(mavmoos_rover_components PRIVATE -Wall -Wextra -Wpedantic)

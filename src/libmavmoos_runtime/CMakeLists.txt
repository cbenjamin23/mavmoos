add_library(mavmoos_replay STATIC ControlReplayJournal.cpp)
set_target_properties(mavmoos_replay PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(mavmoos_replay PRIVATE -Wall -Wextra -Wpedantic)
add_library(mavmoos_runtime PluginHost.cpp Runtime.cpp RuntimeControlReplay.cpp RuntimeMission.cpp RuntimeMissionExecution.cpp RuntimeArduRoverMission.cpp
  RuntimeAutoRecovery.cpp RuntimePlane.cpp RuntimeQuadPlane.cpp RuntimePx4.cpp RuntimePx4Rover.cpp RuntimeFlightLog.cpp RuntimeFileTransfer.cpp RuntimePayload.cpp RuntimeExternalNavigation.cpp RuntimeStream.cpp RuntimeFirmware.cpp FirmwareDiscovery.cpp ObservationRequests.cpp ModuleSelection.cpp)
target_include_directories(mavmoos_runtime PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(mavmoos_runtime PUBLIC mavlink_headers PRIVATE mavmoos_replay ${CMAKE_DL_LIBS})
target_compile_options(mavmoos_runtime PRIVATE -Wall -Wextra -Wpedantic)

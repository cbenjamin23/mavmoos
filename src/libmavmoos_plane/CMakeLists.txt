add_library(mavmoos_plane_components STATIC ArduPlaneBackend.cpp PlaneHelmTasks.cpp)
set_target_properties(mavmoos_plane_components PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mavmoos_plane_components PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(mavmoos_plane_components PUBLIC mavlink_headers)
target_compile_options(mavmoos_plane_components PRIVATE -Wall -Wextra -Wpedantic)

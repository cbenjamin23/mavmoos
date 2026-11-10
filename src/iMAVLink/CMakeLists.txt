#--------------------------------------------------------
# The CMakeLists.txt for:                       iMAVLink
# Author(s):                              Charles Benjamin
#--------------------------------------------------------

add_executable(iMAVLink MAVLink.cpp MAVLink_Info.cpp OwnedFtpRoot.cpp main.cpp)
target_include_directories(iMAVLink PRIVATE "${MOOS_IVP_ROOT}/include/ivp")
target_link_libraries(iMAVLink PRIVATE mavmoos_rx mavmoos_runtime mavmoos_ftp_core ${MOOS_LIBRARIES}
  "${MAVMOOS_APPUTIL}" "${MAVMOOS_MBUTIL}")
target_compile_options(iMAVLink PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(iMAVLink PRIVATE MAVMOOS_PLUGIN_DIRECTORY="${CMAKE_BINARY_DIR}/plugins"
  MAVMOOS_PLUGIN_SUFFIX="${CMAKE_SHARED_MODULE_SUFFIX}")
add_dependencies(iMAVLink mavmoos_standard_plugins)

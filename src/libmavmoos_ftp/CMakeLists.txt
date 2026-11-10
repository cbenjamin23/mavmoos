add_library(mavmoos_ftp_core STATIC FtpTransaction.cpp)
set_target_properties(mavmoos_ftp_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mavmoos_ftp_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(mavmoos_ftp_core PRIVATE -Wall -Wextra -Wpedantic)

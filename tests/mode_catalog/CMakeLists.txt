add_executable(mode_catalog_tests ModeCatalogTests.cpp)
target_include_directories(mode_catalog_tests PRIVATE "${PROJECT_SOURCE_DIR}/src/libmavmoos_runtime")
target_link_libraries(mode_catalog_tests PRIVATE mavlink_headers ${CMAKE_DL_LIBS})
target_compile_options(mode_catalog_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME mode_catalog_observations COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/ModeCatalogTests.py"
  $<TARGET_FILE:mode_catalog_tests> $<TARGET_FILE:mavmoos_mode_catalog>)
set_tests_properties(mode_catalog_observations PROPERTIES TIMEOUT 12)

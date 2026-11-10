#pragma once
#include <cstdlib>
#include <unistd.h>
#include <filesystem>
#include <stdexcept>
#include <string>

struct NativeReplayDirectory {
  std::filesystem::path path;
  NativeReplayDirectory() {
    auto name=(std::filesystem::temp_directory_path()/"mavmoos-rover-replay.XXXXXX").string();
    if(!mkdtemp(name.data()))throw std::runtime_error("could not create private Rover replay fixture");
    path=name;
  }
  ~NativeReplayDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path,error);
  }
  NativeReplayDirectory(const NativeReplayDirectory&)=delete;
  NativeReplayDirectory& operator=(const NativeReplayDirectory&)=delete;
};

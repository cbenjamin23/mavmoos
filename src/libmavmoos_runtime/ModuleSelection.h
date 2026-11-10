#pragma once
#include <optional>
#include <string>
#include <vector>

namespace mavmoos::runtime {
struct ModuleSelection {
  std::vector<std::string> modules;
  bool native_actions=false,helm=false;
  std::vector<std::string> notices;
};
// Startup-only normalization; old names preserve their original request surfaces.
ModuleSelection selectModules(const std::vector<std::string>& requested,bool simulation,
                             bool helm_enabled,bool helm_explicit=false,
                             std::optional<bool> native_override={},std::optional<bool> helm_override={},
                             const std::string& vehicle_profile="arducopter");
} // namespace mavmoos::runtime

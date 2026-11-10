#include "ModuleSelection.h"
#include "Px4Offboard.h"
#include <algorithm>
#include <cctype>
#include <set>
#include <stdexcept>

namespace mavmoos::runtime {
ModuleSelection selectModules(const std::vector<std::string>& requested,bool simulation,
                             bool helm_enabled,bool helm_explicit,
                             std::optional<bool> native_override,std::optional<bool> helm_override,
                             const std::string& vehicle_profile)
{
  ModuleSelection result;
  const bool copter=vehicle_profile=="arducopter";
  const bool plane=vehicle_profile=="arduplane";
  const bool sub=vehicle_profile=="ardusub";
  const bool px4=mavmoos::runtime::px4::isProfile(vehicle_profile);
  const bool px4_rover=vehicle_profile==mavmoos::runtime::px4::rover_profile;
  const bool quadplane=vehicle_profile=="arduplane_quadplane";
  if(!copter && !plane && !quadplane && !px4 && !sub && vehicle_profile!="ardurover" && vehicle_profile!="ardurover_boat")
    throw std::invalid_argument("unsupported vehicle_profile");
  const std::string canonical=copter?"arducopter_control":plane?"arduplane_control":quadplane?"arduquadplane_control":px4_rover?"px4_rover_control":
    px4?"px4_control":sub?"ardusub_control":"ardurover_control";
  const auto contains=[&](const std::string& name) {
    return std::find(requested.begin(),requested.end(),name)!=requested.end();
  };
  const bool legacy_commands=contains("commands"),legacy_helm=contains("helm");
  for(const std::string provider:{"arducopter_control","ardurover_control","arduplane_control","px4_control",
                                 "px4_rover_control","ardusub_control","arduquadplane_control"})
    if(provider!=canonical && contains(provider))
      throw std::invalid_argument("control provider does not match vehicle_profile");
  if(!copter && (legacy_commands||legacy_helm))
    throw std::invalid_argument("legacy commands/helm aliases require the arducopter profile");
  if(contains("arducopter_control")&&(legacy_commands||legacy_helm))
    throw std::invalid_argument("use arducopter_control or legacy commands/helm aliases, not both");
  std::set<std::string> seen;
  for(const auto& name:requested) {
    if(name.empty()||!std::all_of(name.begin(),name.end(),[](unsigned char c){return std::isalnum(c)||c=='_';})||
       !seen.insert(name).second) throw std::invalid_argument("invalid or duplicate module selection");
  }
  if(requested.empty()) {
    result.modules={"telemetry"};
    result.native_actions=simulation&&(!px4||px4_rover);result.helm=helm_enabled;
    if(simulation||helm_enabled) result.modules.push_back(canonical);
    if(simulation&&!px4&&!sub&&!quadplane) result.modules.push_back("parameters");
  } else {
    for(const auto& name:requested) {
      const auto actual=(name=="commands"||name=="helm")?"arducopter_control":name;
      if(std::find(result.modules.begin(),result.modules.end(),actual)==result.modules.end())
        result.modules.push_back(actual);
    }
    if(legacy_commands||legacy_helm) {
      result.native_actions=legacy_commands;result.helm=legacy_helm;
      result.notices.push_back("commands/helm plugin names are legacy aliases for arducopter_control; original enabled request surfaces are preserved");
    } else if(contains(canonical)) {
      result.native_actions=!px4||px4_rover;result.helm=helm_explicit?helm_enabled:!quadplane;
    }
  }
  const bool control=std::find(result.modules.begin(),result.modules.end(),canonical)!=result.modules.end();
  if(!control&&(native_override||helm_override))
    throw std::invalid_argument("control facet settings require the selected control provider");
  if(native_override) result.native_actions=*native_override;
  if(helm_override) result.helm=*helm_override;
  return result;
}
} // namespace mavmoos::runtime

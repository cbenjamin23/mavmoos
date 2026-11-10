/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ArduRoverControlPlugin.cpp                      */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "RoverComponents.h"
#include "ArduRoverMissionPolicy.h"
#include "ControlComponents.h"
#include <algorithm>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
bool enabled(const std::string& key,const std::string& value)
{
  if(value!="true"&&value!="false")throw std::invalid_argument(key+" must be true/false");
  return value=="true";
}
class ArduRoverControl final:public Plugin {
public:
  explicit ArduRoverControl(Services& services)
    :m_services(services) {selectProfile(false);}
  std::string name() const override {return "ardurover_control";}
  const VehicleControlBackend* controlBackend() const override {return m_backend.get();}
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor d;d.profile=m_backend->profile();d.exclusive_resources={"vehicle.control"};
    d.request_owners={"MAVMOOS_COMMAND","MAVMOOS_NAV_CONTROL","MAVMOOS_CONTROL_REQUEST"};
    d.observation_streams={{33,50000.f},{193,500000.f},{242,500000.f}};
    if(m_native) {
      for(const auto& pair:std::vector<std::pair<std::string,std::string>>{
        {"control.arm","arm"},{"control.disarm","disarm"},{"control.guided","guided"},
        {"control.hold.native_stop","hold"},{"control.return.native_rtl","rtl"}})
        d.capabilities.push_back({pair.first,"MAVMOOS_COMMAND","action="+pair.second,"request"});
      if(m_backend->profile()=="ardurover_boat")
        d.capabilities.push_back({"control.hold.native_loiter","MAVMOOS_COMMAND","action=loiter","request"});
      const auto commands=m_commands->descriptor();
      d.capabilities.insert(d.capabilities.end(),commands.capabilities.begin(),commands.capabilities.end());
      if(m_mission_enabled) {
        const auto mission=m_mission->descriptor();
        d.capabilities.insert(d.capabilities.end(),mission.capabilities.begin(),mission.capabilities.end());
        d.request_owners.insert(d.request_owners.end(),mission.request_owners.begin(),mission.request_owners.end());
        d.observation_streams.insert(d.observation_streams.end(),mission.observation_streams.begin(),mission.observation_streams.end());
      }
    }
    if(m_helm_enabled) {
      d.capabilities.push_back({"control.navigation.helm","MAVMOOS_NAV_CONTROL","acquire|release","request"});
      d.capabilities.push_back({"control.hold.guided_zero_velocity","","","internal"});
    }
    return d;
  }
  std::vector<std::string> subscriptions() const override
  {
    std::vector<std::string> result;
    if(m_native)result=m_commands->subscriptions();
    if(m_native&&m_mission_enabled) {
      const auto mission=m_mission->subscriptions();
      result.insert(result.end(),mission.begin(),mission.end());
    }
    if(m_helm_enabled)for(const auto& key:m_helm->subscriptions())
      if(std::find(result.begin(),result.end(),key)==result.end())result.push_back(key);
    return result;
  }
  void configure(const std::string& key,const std::string& value) override
  {
    if(key=="vehicle_profile") {
      if(value!="ardurover"&&value!="ardurover_boat")throw std::invalid_argument("invalid ArduRover control profile");
      if(m_backend->profile()!=value) selectProfile(value=="ardurover_boat");
      return;
    }
    if(key=="control_native_actions"){m_native=enabled(key,value);return;}
    if(key=="control_helm"){m_helm_enabled=enabled(key,value);return;}
    if(key=="control_mission_execution") {
      m_mission_enabled=enabled(key,value);
      m_mission->configure(key,value);
      return;
    }
    if(key=="facing_policy"&&value!="course")throw std::invalid_argument("Rover profile supports facing_policy=course only");
    m_commands->configure(key,value);m_helm->configure(key,value);
  }
  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override
  {
    for(const auto& setting:settings) {
      if(setting.key!="native_replay_directory")
        return {ModuleConfigStatus::UnknownKey,setting.key,"unknown Rover setting"};
      try {m_commands->configure(setting.key,setting.value);}
      catch(const std::invalid_argument& error) {
        return {ModuleConfigStatus::Invalid,setting.key,error.what()};
      }
    }
    return {};
  }
  void onMail(const Mail& x) override
  {
    if(x.key=="MAVMOOS_NAV_CONTROL") {
      if(m_helm_enabled) (m_native?m_commands:m_helm)->onMail(x);
      return;
    }
    if(m_native&&m_mission_enabled)m_mission->onMail(x);
    if(m_native)m_commands->onMail(x);
    if(m_helm_enabled)m_helm->onMail(x);
  }
  void onMessage(const mavlink_message_t& x,double at) override
  {onMessageWithMetadata(x,{at,at,"READ_ACQUISITION"});}
  void onMessageWithMetadata(const mavlink_message_t& x,const MessageTime& time) override
  {
    if(m_native&&m_mission_enabled) {
      missionPolicy().observe(x,time,m_services.vehicle(),m_services.now());
      m_mission->onMessageWithMetadata(x,time);
    }
    if(m_native)m_commands->onMessageWithMetadata(x,time);
    if(m_helm_enabled)m_helm->onMessageWithMetadata(x,time);
  }
  void onCommandSent(const CommandRequest& x,double at,bool accepted) override
  {
    if(m_native&&m_mission_enabled)m_mission->onCommandSent(x,at,accepted);
    if(m_native)m_commands->onCommandSent(x,at,accepted);
  }
  void onParameterSent(const ParameterRequest& x,double at,bool accepted) override
  {if(m_native&&m_mission_enabled)missionPolicy().parameterSent(x,at,accepted);}
  void onReset(const std::string& why) override
  {
    if(m_native&&m_mission_enabled){missionPolicy().reset(m_services.now());m_mission->onReset(why);}
    if(m_native)m_commands->onReset(why);
    if(m_helm_enabled)m_helm->onReset(why);
  }
  void onInputBoundary() override
  {
    if(m_native&&m_mission_enabled)m_mission->onInputBoundary();
    if(m_native)m_commands->onInputBoundary();
    if(m_helm_enabled)m_helm->onInputBoundary();
  }
  void iterate() override
  {
    if(m_native&&m_mission_enabled){missionPolicy().readConfiguration(m_services);m_mission->iterate();}
    if(m_native)m_commands->iterate();
    if(m_helm_enabled)m_helm->iterate();
  }
private:
  ArduRoverMissionPolicy& missionPolicy() {return dynamic_cast<ArduRoverMissionPolicy&>(*m_backend);}
  void selectProfile(bool boat)
  {
    auto backend=makeArduRoverBackend(boat);
    auto helm=makeHelmAdapter(m_services,*backend);
    auto commands=makeRoverCommands(m_services,*backend,helm.get());
    auto mission=makeArduRoverMission(m_services,*backend);
    if(m_mission_enabled) mission->configure("control_mission_execution","true");
    m_mission=std::move(mission);
    // The commands hold references to the helm and backend. Retire them first.
    m_commands=std::move(commands);m_helm=std::move(helm);m_backend=std::move(backend);
  }
  Services& m_services;
  std::unique_ptr<VehicleControlBackend> m_backend;
  std::unique_ptr<Plugin> m_helm,m_commands,m_mission;
  bool m_native=true,m_helm_enabled=true,m_mission_enabled=false;
};
} // namespace
} // namespace mavmoos::runtime
extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{return abi==mavmoos::runtime::plugin_abi&&services?new mavmoos::runtime::ArduRoverControl(*services):nullptr;}
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin){delete plugin;}

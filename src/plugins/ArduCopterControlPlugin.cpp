/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ArduCopterControlPlugin.cpp                      */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "ControlComponents.h"
#include "ControlReplayService.h"
#include "ArduPilotObservations.h"
#include "CopterFrame.h"
#include "CopterOperations.h"
#include "CopterMissionExecution.h"
#include <algorithm>
#include <functional>
#include <stdexcept>
#include <utility>

namespace mavmoos::runtime {
namespace {
const VehicleControlBackend& requireBackend(const VehicleControlBackend* backend)
{
  if(!backend) throw std::runtime_error("ArduCopter control backend unavailable");
  return *backend;
}
bool enabled(const std::string& key,const std::string& value)
{
  if(value!="true" && value!="false") throw std::invalid_argument(key+" must be true/false");
  return value=="true";
}
class ComponentServices final:public Services,public ControlReplayService {
public:
  explicit ComponentServices(Services& services):m_services(services) {}
  CopterOperations* operations=nullptr;
  bool controlReplayEnabled() const override {
    const auto* replay=dynamic_cast<const ControlReplayService*>(&m_services);
    return replay&&replay->controlReplayEnabled();
  }
  bool controlRequestSeen(const std::string& id) const override {
    return mavmoos::runtime::controlRequestSeen(m_services,id);
  }
  ReplayResult reserveControlRequest(const std::string& id) override {
    return mavmoos::runtime::reserveControlRequest(m_services,id);
  }
  ReplayResult reserveControlEvent() override {return mavmoos::runtime::reserveControlEvent(m_services);}
  double now() const override {return m_services.now();}
  const Vehicle& vehicle() const override {return m_services.vehicle();}
  void publish(const std::string& key,const std::string& value) override {
    m_services.publish(key,value);
    if(operations)operations->observePublication(key,value);
  }
  void publish(const std::string& key,double value) override {m_services.publish(key,value);}
  bool sendCommand(const CommandRequest& request) override {
    auto bounded=request;
    if(operations&&!operations->boundNativeCommand(bounded))return false;
    const bool accepted=m_services.sendCommand(bounded);
    if(operations)operations->nativeSubmitted(bounded,accepted);
    return accepted;
  }
  bool acquireNavigation(const std::string& id) override {return m_services.acquireNavigation(id);}
  void revokeNavigation(const std::string& reason) override {m_services.revokeNavigation(reason);}
  bool navigation(const NavigationTarget& target) override {
    auto bounded=target;
    if(operations&&!operations->boundHelmTaskTarget(bounded)) return false;
    return m_services.navigation(bounded);
  }
  bool parameter(const ParameterRequest& request) override {return m_services.parameter(request);}
  bool mission(const MissionRequest& request) override {return m_services.mission(request);}
  void missionFinished(const std::string& id) override {m_services.missionFinished(id);}
  bool navigationGranted() const override {return m_services.navigationGranted();}
  bool controlBusy() const override {return m_services.controlBusy();}
  bool projectGlobal(double latitude,double longitude,double& x,double& y) const override {
    return m_services.projectGlobal(latitude,longitude,x,y);
  }
  void cancelCommand(const std::string& id) override {m_services.cancelCommand(id);}
  bool suspendNavigation(double until) override {return m_services.suspendNavigation(until);}
  bool resumeNavigation(const std::string& id) override {return m_services.resumeNavigation(id);}
  void commandFinished(const std::string& id) override {
    if(!operations||operations->nativeFinished(id))m_services.commandFinished(id);
  }
private:
  Services& m_services;
};
class ArduCopterControl final: public Plugin {
public:
  explicit ArduCopterControl(Services& services)
    :m_backend(makeArduCopterBackend()),m_services(services),m_commands(makeCopterCommands(m_services)),
     m_helm(makeHelmAdapter(m_services,requireBackend(m_backend.get())))
  {
    if(!m_commands || !m_helm) throw std::runtime_error("ArduCopter control component unavailable");
    m_operations=makeCopterOperations(services,*m_commands,*m_helm,*m_backend);
    m_mission_execution=makeCopterMissionExecution(services);
    if(!m_mission_execution)throw std::runtime_error("Copter mission execution component unavailable");
    m_services.operations=m_operations.get();
  }
  ~ArduCopterControl() override {m_services.operations=nullptr;}
  std::string name() const override { return "arducopter_control"; }
  const VehicleControlBackend* controlBackend() const override { return m_backend.get(); }
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile="arducopter";
    result.exclusive_resources={"vehicle.control"};
    // Ownership stays reserved when a path is disabled; another module cannot
    // become a competing consumer of the same control request channel.
    result.request_owners={"MAVMOOS_COMMAND","MAVMOOS_NAV_CONTROL","MAVMOOS_CONTROL_REQUEST","MAVMOOS_MISSION_CONTROL"};
    result.observation_streams=arduPilotObservationStreams();
    if(m_native_enabled) {
      for(const auto& item:std::vector<std::pair<std::string,std::string>>{
          {"control.arm","arm"},{"control.disarm","disarm"},{"control.takeoff","takeoff"},
          {"control.hold.native_loiter","hold"},{"control.land","land"},
          {"control.return.native_rtl","rtl"},{"control.guided","guided"}})
        result.capabilities.push_back({item.first,"MAVMOOS_COMMAND","action="+item.second,"request"});
      const auto additions=m_commands->descriptor();
      result.capabilities.insert(result.capabilities.end(),additions.capabilities.begin(),additions.capabilities.end());
      result.request_owners.insert(result.request_owners.end(),additions.request_owners.begin(),additions.request_owners.end());
    }
    if(m_helm_enabled) {
      result.capabilities.push_back({"control.navigation.helm","MAVMOOS_NAV_CONTROL","acquire|release","request"});
      result.capabilities.push_back({"control.hold.guided_recovery","","","internal"});
    }
    const auto operation_capabilities=m_operations->capabilities();
    result.capabilities.insert(result.capabilities.end(),operation_capabilities.begin(),operation_capabilities.end());
    if(m_native_enabled) {
      const auto mission=m_mission_execution->descriptor();
      result.capabilities.insert(result.capabilities.end(),mission.capabilities.begin(),mission.capabilities.end());
      result.observation_streams.insert(result.observation_streams.end(),mission.observation_streams.begin(),mission.observation_streams.end());
    }
    return result;
  }
  std::vector<std::string> subscriptions() const override
  {
    std::vector<std::string> result;
    if(m_native_enabled) result=m_commands->subscriptions();
    if(m_helm_enabled) {
      const auto values=m_helm->subscriptions();
      for(const auto& value:values)
        if(std::find(result.begin(),result.end(),value)==result.end()) result.push_back(value);
    }
    const auto operations=m_operations->subscriptions();
    result.insert(result.end(),operations.begin(),operations.end());
    if(m_native_enabled) {
      const auto mission=m_mission_execution->subscriptions();
      result.insert(result.end(),mission.begin(),mission.end());
    }
    return result;
  }
  void configure(const std::string& key,const std::string& value) override
  {
    // Startup selection, before the host starts dispatching operational input.
    if(key=="copter_frame") {
      const auto type=copterFrameType(value);
      if(type==MAV_TYPE_GENERIC) throw std::invalid_argument("unsupported Copter frame");
      m_backend->bindFrameType(type);
      m_mission_execution->configure(key,value);
      return;
    }
    if(key=="control_native_actions") { m_native_enabled=enabled(key,value);m_operations->setFacets(m_native_enabled,m_helm_enabled);return; }
    if(key=="control_helm") { m_helm_enabled=enabled(key,value);m_operations->setFacets(m_native_enabled,m_helm_enabled);return; }
    m_commands->configure(key,value);
    m_helm->configure(key,value);
    m_operations->configure(key,value);
    m_mission_execution->configure(key,value);
  }
  void onMail(const Mail& mail) override
  {
    if(m_native_enabled)m_mission_execution->onMail(mail);
    m_operations->onMail(mail);
    if(m_native_enabled) m_commands->onMail(mail);
    if(m_helm_enabled) m_helm->onMail(mail);
  }
  void onMessage(const mavlink_message_t& message,double acquired) override
  {
    onMessageWithMetadata(message,{acquired,acquired,"READ_ACQUISITION"});
  }
  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time) override
  {
    if(m_native_enabled) {
      m_mission_execution->onMessageWithMetadata(message,time);
      m_commands->onMessageWithMetadata(message,time);
    }
    if(m_helm_enabled)m_helm->onMessageWithMetadata(message,time);
  }
  void onCommandSent(const CommandRequest& request,double at,bool accepted) override
  {
    if(m_native_enabled) {
      m_mission_execution->onCommandSent(request,at,accepted);
      m_commands->onCommandSent(request,at,accepted);
    }
    if(m_helm_enabled)m_helm->onCommandSent(request,at,accepted);
    m_operations->onCommandSent(request,at,accepted);
  }
  void onReset(const std::string& reason) override
  {
    if(m_native_enabled) m_commands->onReset(reason);
    if(m_helm_enabled) m_helm->onReset(reason);
    m_operations->onReset(reason);
    m_mission_execution->onReset(reason);
  }
  void onParameterSent(const ParameterRequest& request,double sent_at,bool accepted) override
  {
    if(m_native_enabled) m_commands->onParameterSent(request,sent_at,accepted);
    if(m_helm_enabled) m_helm->onParameterSent(request,sent_at,accepted);
  }
  void onInputBoundary() override
  {
    if(m_native_enabled)m_mission_execution->onInputBoundary();
    m_operations->onInputBoundary();
    if(m_native_enabled) m_commands->onInputBoundary();
    if(m_helm_enabled) m_helm->onInputBoundary();
  }
  void iterate() override
  {
    if(m_native_enabled)m_mission_execution->iterate();
    m_operations->iterate();
    if(m_native_enabled) m_commands->iterate();
    if(m_helm_enabled) m_helm->iterate();
    m_operations->afterComponents();
  }
private:
  // Reverse destruction order retires both components before their backend.
  std::unique_ptr<ArduCopterControlBackend> m_backend;
  ComponentServices m_services;
  std::unique_ptr<Plugin> m_commands,m_helm;
  std::unique_ptr<CopterOperations> m_operations;
  std::unique_ptr<Plugin> m_mission_execution;
  bool m_native_enabled=true,m_helm_enabled=true;
};
} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{ return abi==mavmoos::runtime::plugin_abi && services ? new mavmoos::runtime::ArduCopterControl(*services) : nullptr; }
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin) { delete plugin; }

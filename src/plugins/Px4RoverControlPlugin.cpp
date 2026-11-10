/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: Px4RoverControlPlugin.cpp                        */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "ControlComponents.h"
#include "Px4Offboard.h"
#include "Px4RoverBackend.h"
#include "Px4RoverMission.h"
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
class Px4RoverControl final : public Plugin {
public:
  explicit Px4RoverControl(Services& host):m_host(host),
    m_helm(makeHelmAdapter(host,m_backend)),m_mission(makePx4RoverMission(host,m_backend)) {}

  std::string name() const override { return "px4_rover_control"; }
  const VehicleControlBackend* controlBackend() const override { return &m_backend; }
  PluginDescriptor descriptor() const override
  {
    auto d=m_mission->descriptor();
    d.profile=px4::rover_profile;
    d.exclusive_resources={"vehicle.control"};
    d.observation_streams={{MAVLINK_MSG_ID_CURRENT_MODE,200000.f},{MAVLINK_MSG_ID_GPS_RAW_INT,125000.f},
      {MAVLINK_MSG_ID_ESTIMATOR_STATUS,200000.f},{MAVLINK_MSG_ID_HOME_POSITION,500000.f}};
    if(m_helm_enabled) {
      d.request_owners.push_back("MAVMOOS_NAV_CONTROL");
      d.capabilities.push_back({"control.navigation.helm","MAVMOOS_NAV_CONTROL","acquire|release"});
      d.capabilities.push_back({"control.hold.offboard_prestream","","","internal"});
    }
    d.capabilities.push_back({"control.rover.stop","MAVMOOS_PX4_ROVER_STATUS","","observation"});
    return d;
  }
  std::vector<std::string> subscriptions() const override
  {
    auto subscriptions=m_mission->subscriptions();
    if(m_helm_enabled) {
      const auto helm=m_helm->subscriptions();
      subscriptions.insert(subscriptions.end(),helm.begin(),helm.end());
    }
    return subscriptions;
  }
  void configure(const std::string& key,const std::string& value) override
  {
    if(key=="vehicle_profile" && value!=px4::rover_profile)
      throw std::invalid_argument("invalid PX4 Rover profile");
    if(key=="facing_policy" && value!="course")
      throw std::invalid_argument("PX4 Ackermann uses course guidance");
    if(key=="control_helm") {
      if(value!="true" && value!="false") throw std::invalid_argument("invalid PX4 Rover helm setting");
      m_helm_enabled=value=="true";
    }
    m_helm->configure(key,value);
    m_mission->configure(key,value);
  }
  void onMail(const Mail& mail) override
  {
    if(m_helm_enabled) m_helm->onMail(mail);
    m_mission->onMail(mail);
  }
  void onMessageWithMetadata(const mavlink_message_t& msg,const MessageTime& time) override
  {
    m_backend.observe(msg,time,m_host.vehicle(),m_host.now());
    m_mission->onMessageWithMetadata(msg,time);
  }
  void onParameterSent(const ParameterRequest& request,double at,bool accepted) override
  { m_backend.parameterSent(request,at,accepted); }
  void onCommandSent(const CommandRequest& request,double at,bool accepted) override
  { m_mission->onCommandSent(request,at,accepted); }
  void onInputBoundary() override
  {
    if(m_helm_enabled) m_helm->onInputBoundary();
    m_mission->onInputBoundary();
  }
  void iterate() override
  {
    m_backend.readConfiguration(m_host);
    if(m_helm_enabled) m_helm->iterate();
    m_mission->iterate();
    const auto now=m_host.now();
    const auto& v=m_host.vehicle();
    const bool configured=m_backend.verifiedConfig(now);
    const bool stopped=m_backend.stopped(now,-1);
    const bool hold=v.heartbeat_valid && px4::fresh(v.heartbeat_at,now,px4::rover_heartbeat_age) &&
      v.mode==px4::hold && m_backend.holdObserved(now,-1);
    const auto boolean=[](bool value){return value?"true":"false";};
    m_host.publish("MAVMOOS_PX4_ROVER_STATUS",std::string("schema=1,config_verified=")+boolean(configured)+
      ",stop_confirmed="+boolean(stopped)+",hold_ready="+boolean(configured&&stopped&&hold)+
      ",active_intended_hold="+boolean(hold));
  }
  void onReset(const std::string& reason) override
  {
    m_backend.reset(m_host.now());
    if(m_helm_enabled) m_helm->onReset(reason);
    m_mission->onReset(reason);
  }
private:
  Services& m_host;
  Px4RoverBackend m_backend;
  std::unique_ptr<Plugin> m_helm,m_mission;
  bool m_helm_enabled=true;
};
}
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{ return abi==mavmoos::runtime::plugin_abi && services?new mavmoos::runtime::Px4RoverControl(*services):nullptr; }

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin) { delete plugin; }

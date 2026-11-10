// Trusted test modules exercising provider object lifetime, never raw I/O.
#include "ControlComponents.h"
#include <stdexcept>

using namespace mavmoos::runtime;
namespace {
#ifdef MAVMOOS_REPLACEABLE_PROVIDER
// A deliberately synthetic policy proves that Runtime delegates vehicle meaning.
// It reuses a bounded encoder; this is not a second vehicle implementation.
class Mode77Backend final:public VehicleControlBackend {
  std::unique_ptr<VehicleControlBackend> m_base=makeArduCopterBackend();
  static Vehicle mapped(Vehicle value) {if(value.mode==77)value.mode=4;return value;}
public:
  std::string profile() const override {return "test_mode_77";}
  bool identityCompatible(unsigned type,unsigned autopilot) const override {return m_base->identityCompatible(type,autopilot);}
  bool flightReady(const Vehicle& v,double now,bool motion) const override
  {return (!motion||v.mode==77)&&m_base->flightReady(mapped(v),now,motion);}
  bool acquisitionAllowed(const Vehicle& v,double now) const override
  {return (v.mode==5||v.mode==77)&&v.armed&&v.in_air&&m_base->flightReady(v,now,false);}
  bool activeModeAllowed(const Vehicle& v,bool grace) const override {return v.mode==77||(v.mode==5&&grace);}
  std::optional<CommandRequest> acquisitionCommand(const Vehicle& v,const std::string& id,double now) const override
  {
    if(v.mode!=5)return std::nullopt;
    CommandRequest request;request.id="synthetic-acquire-"+id;request.command=MAV_CMD_DO_SET_MODE;
    request.params[0]=1;request.params[1]=77;request.expires=now+5;return request;
  }
  bool commandAllowed(const CommandRequest& request,const Vehicle& v,double now) const override
  {
    if(request.command==MAV_CMD_DO_SET_MODE)return request.params[0]==1&&request.params[1]==77;
    return m_base->commandAllowed(request,mapped(v),now);
  }
  bool navigationInputAllowed(const NavigationTarget& target) const override {return m_base->navigationInputAllowed(target);}
  bool helmFieldAllowed(unsigned index,double value) const override {return m_base->helmFieldAllowed(index,value);}
  bool helmVehicleReady(const Vehicle& v,double now,double timeout,bool shadow,bool acquiring) const override
  {return (shadow||activeModeAllowed(v,acquiring))&&m_base->helmVehicleReady(mapped(v),now,timeout,shadow,acquiring);}
  std::optional<ControlFrame> navigationFrame(const NavigationTarget& target,const Vehicle& v,double now,
    uint64_t revision,mavlink_status_t& codec) const override
  {return v.mode==77?m_base->navigationFrame(target,mapped(v),now,revision,codec):std::nullopt;}
  std::optional<ControlFrame> stopFrame(const Vehicle& v,double now,uint64_t revision,mavlink_status_t& codec) const override
  {return v.mode==77?m_base->stopFrame(mapped(v),now,revision,codec):std::nullopt;}
};
class Fixture final:public Plugin {
public:
  std::string name() const override {return "replaceable_provider";}
  const VehicleControlBackend* controlBackend() const override {return m_backend.get();}
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;result.profile=m_backend->profile();
    result.exclusive_resources={"vehicle.control"};
    if(m_collision)result.capabilities.push_back({"review.occupied","","","observation"});
    return result;
  }
  void configure(const std::string& key,const std::string&) override
  {
    if(key=="synthetic_mode_77") {
      std::unique_ptr<VehicleControlBackend> next=std::make_unique<Mode77Backend>();m_backend.swap(next);return;
    }
    if(key!="replace_backend"&&key!="reject_reconfigure")return;
    // Allocate before retiring the old object, making address reuse unable to
    // conceal a stale borrowed pointer. This is valid startup configuration.
    std::unique_ptr<VehicleControlBackend> next=makeArduCopterBackend();m_backend.swap(next);
    m_collision=key=="reject_reconfigure";
  }
  void iterate() override {}
private:
  std::unique_ptr<VehicleControlBackend> m_backend=makeArduCopterBackend();
  bool m_collision=false;
};
#else
class Fixture final:public Plugin {
public:
  explicit Fixture(Services& services):m_services(services) {}
  ~Fixture() override
  {
    m_services.revokeNavigation("observer_teardown");
    m_services.publish("MAVMOOS_TEST_PROVIDER_LIFETIME","observer_destroyed");
  }
  std::string name() const override {return "teardown_observer";}
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;result.capabilities.push_back({"review.occupied","","","observation"});return result;
  }
  void iterate() override {}
private:
  Services& m_services;
};
#endif
}
extern "C" Plugin* mavmoos_create_plugin(unsigned abi,Services* services)
{
  if(abi!=plugin_abi||!services)return nullptr;
#ifdef MAVMOOS_REPLACEABLE_PROVIDER
  return new Fixture();
#else
  return new Fixture(*services);
#endif
}
extern "C" void mavmoos_destroy_plugin(Plugin* plugin) {delete plugin;}

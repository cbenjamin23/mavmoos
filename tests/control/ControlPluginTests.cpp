// Real combined DSO + fake services. No socket, MOOS process or vehicle is opened.
#include "PluginHost.h"
#include "ControlBackend.h"
#include <algorithm>
#include <dlfcn.h>
#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void require(bool value,const std::string& why) { if(!value)throw std::runtime_error(why); }
struct ServicesFixture final:Services {
  double time=100;
  Vehicle state;
  unsigned acquisitions=0;
  std::vector<CommandRequest> commands;
  std::vector<NavigationTarget> targets;
  std::vector<std::string> revocations,finished;
  std::vector<std::pair<std::string,std::string>> publications;
  ServicesFixture() { fresh(false); }
  void fresh(bool airborne=true) {
    state.heartbeat_valid=state.position_valid=state.home_valid=state.landed_valid=state.estimator_valid=true;
    state.heartbeat_at=state.position_at=state.landed_at=state.estimator_at=time;
    state.type=2;state.autopilot=3;state.mode=4;state.armed=state.in_air=airborne;
    state.landed_state=airborne?2:1;state.relative_height=5;
  }
  double now() const override { return time; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key,const std::string& text) override { publications.emplace_back(key,text); }
  void publish(const std::string&,double) override {}
  bool sendCommand(const CommandRequest& request) override { commands.push_back(request);return true; }
  bool acquireNavigation(const std::string&) override { ++acquisitions;return true; }
  void revokeNavigation(const std::string& reason) override { revocations.push_back(reason); }
  bool navigation(const NavigationTarget& target) override { targets.push_back(target);return true; }
  void commandFinished(const std::string& id) override { finished.push_back(id); }
  bool published(const std::string& key,const std::string& value) const {
    return std::any_of(publications.begin(),publications.end(),[&](const auto& p){return p.first==key&&p.second.find(value)!=std::string::npos;});
  }
};
struct Rig {
  ServicesFixture services;
  PluginHost host;
  explicit Rig(const std::string& path) { require(host.load(path,services),"load combined control: "+host.lastError()); }
  Plugin& plugin() { require(host.plugins().size()==1,"expected one provider object");return *host.plugins().front(); }
  void text(const std::string& key,const std::string& value,double at,const std::string& source="operator") {
    services.time=at;Mail mail;mail.key=key;mail.text=value;mail.source=source;mail.received_at=mail.published_at=at;host.dispatchMail(mail);
  }
  void fields(double at) {
    services.time=at;services.fresh();text("IVPHELM_STATE","DRIVE",at,"pHelmIvP");
    for(const auto& value:std::vector<std::pair<std::string,double>>{{"DESIRED_HEADING",90},{"DESIRED_SPEED",1},{"DESIRED_ALTITUDE",5}}) {
      Mail mail;mail.key=value.first;mail.value=value.second;mail.numeric=true;mail.source="pHelmIvP";mail.received_at=mail.published_at=at;host.dispatchMail(mail);
    }
  }
  void acquire(double at=100.01) {
    services.time=at;services.fresh();text("IVPHELM_ALLSTOP","clear",at,"pHelmIvP");
    text("IVPHELM_STATE","DRIVE",at+.01,"pHelmIvP");text("MAVMOOS_NAV_CONTROL","acquire",at+.02);fields(at+.03);host.iterate();
  }
};
std::set<std::string> capabilityNames(const PluginDescriptor& descriptor) {
  std::set<std::string> names;for(const auto& capability:descriptor.capabilities)require(names.insert(capability.name).second,"duplicate capability");return names;
}
bool subscribed(Plugin& plugin,const std::string& key) {
  const auto keys=plugin.subscriptions();return std::find(keys.begin(),keys.end(),key)!=keys.end();
}
const std::set<std::string> native={"control.arm","control.disarm","control.takeoff","control.hold.native_loiter","control.land","control.return.native_rtl","control.guided","control.operations.native","control.operations.guided","control.operations.cancel"};
const std::set<std::string> helm={"control.navigation.helm","control.hold.guided_recovery"};
}
int main(int argc,char** argv) {
  try {
    require(argc==2,"usage: control_plugin_tests CONTROL_MODULE");const std::string path=argv[1];unsigned passed=0;
    auto test=[&](const char* name,const std::function<void()>& run){run();++passed;std::cout<<"PASS "<<name<<'\n';};
    test("matching development ABI is enforced at the single public factory",[&]{
      void* handle=dlopen(path.c_str(),RTLD_NOW|RTLD_LOCAL|RTLD_NODELETE);require(handle,"dlopen control");
      const auto create=reinterpret_cast<CreatePlugin>(dlsym(handle,"mavmoos_create_plugin"));
      const auto destroy=reinterpret_cast<DestroyPlugin>(dlsym(handle,"mavmoos_destroy_plugin"));require(create&&destroy,"public exports missing");
      ServicesFixture services;require(!create(plugin_abi-1,&services)&&!create(plugin_abi+1,&services)&&!create(plugin_abi,nullptr),"incompatible ABI/context accepted");
      auto* plugin=create(plugin_abi,&services);require(plugin&&plugin->name()=="arducopter_control","combined identity");destroy(plugin);dlclose(handle);
      require(services.commands.empty()&&services.targets.empty()&&services.acquisitions==0,"factory caused control effects");
    });
    test("one coherent provider declares profile, owners, streams and implemented variants",[&]{
      Rig rig(path);auto& plugin=rig.plugin();const auto descriptor=plugin.descriptor();auto expected=native;expected.insert(helm.begin(),helm.end());
      require(descriptor.profile=="arducopter"&&capabilityNames(descriptor)==expected,"wrong advertised variants");
      require(descriptor.exclusive_resources==std::vector<std::string>{"vehicle.control"},"missing exclusive control owner");
      require(std::set<std::string>(descriptor.request_owners.begin(),descriptor.request_owners.end())==std::set<std::string>{"MAVMOOS_COMMAND","MAVMOOS_NAV_CONTROL","MAVMOOS_CONTROL_REQUEST","MAVMOOS_MISSION_CONTROL"},"request ownership mismatch");
      require(!descriptor.streamed_parameter_bench_compatible&&!rig.host.streamedParameterBenchCompatible(),"control admitted to streamed ground bench");
      require(plugin.controlBackend()&&plugin.controlBackend()->profile()=="arducopter","backend missing");
      std::map<unsigned,float> streams;for(const auto& stream:descriptor.observation_streams)require(streams.emplace(stream.message_id,stream.interval_us).second,"duplicate observation stream");
      require(streams==std::map<unsigned,float>{{33,50000},{245,250000},{193,500000},{242,500000}},"observation profile changed");
      const auto recovery=std::find_if(descriptor.capabilities.begin(),descriptor.capabilities.end(),[](const auto& c){return c.name=="control.hold.guided_recovery";});
      require(recovery!=descriptor.capabilities.end()&&recovery->access=="internal"&&recovery->request_variable.empty(),"recovery falsely exposed as a public hold API");
      require(rig.services.commands.empty()&&rig.services.targets.empty()&&rig.services.acquisitions==0&&rig.services.revocations.empty(),"discovery mutated authority");
      require(!rig.host.load(path,rig.services)&&rig.host.plugins().size()==1,"duplicate provider accepted or first lost");
    });
    test("AUTO opt-in adds progress demand through the shared descriptor without effects",[&]{
      Rig rig(path);rig.host.configure("control_mission_execution","true");
      const auto descriptor=rig.plugin().descriptor();auto expected=native;expected.insert(helm.begin(),helm.end());
      for(const auto& action:{"start","pause","resume","abort","release","progress"})expected.insert(std::string("control.mission.")+action);
      require(capabilityNames(descriptor)==expected&&subscribed(rig.plugin(),"MAVMOOS_MISSION_CONTROL"),"enabled AUTO surface missing");
      std::map<unsigned,float> streams;for(const auto& stream:descriptor.observation_streams)require(streams.emplace(stream.message_id,stream.interval_us).second,"duplicate enabled observation stream");
      require(streams==std::map<unsigned,float>{{33,50000},{245,250000},{193,500000},{242,500000},{42,500000}},"AUTO progress stream demand missing");
      require(rig.services.commands.empty()&&rig.services.targets.empty()&&rig.services.acquisitions==0&&rig.services.revocations.empty(),"AUTO discovery caused control effects");
      rig.host.configure("control_native_actions","false");
      require(capabilityNames(rig.plugin().descriptor())==helm&&!subscribed(rig.plugin(),"MAVMOOS_MISSION_CONTROL"),"disabled native facet retained AUTO interface");
      for(const auto& stream:rig.plugin().descriptor().observation_streams)require(stream.message_id!=42,"disabled native facet retained AUTO demand");
      rig.host.configure("control_native_actions","true");rig.host.configure("control_mission_execution","false");
      require(!subscribed(rig.plugin(),"MAVMOOS_MISSION_CONTROL"),"disabled AUTO remained subscribed");
      for(const auto& stream:rig.plugin().descriptor().observation_streams)require(stream.message_id!=42,"disabled AUTO retained progress demand");
    });
    test("native-only facet has one request consumer and no helm acquisition",[&]{
      Rig rig(path);rig.host.configure("control_helm","false");require(capabilityNames(rig.plugin().descriptor())==native,"native-only catalog");
      require(subscribed(rig.plugin(),"MAVMOOS_COMMAND")&&!subscribed(rig.plugin(),"MAVMOOS_NAV_CONTROL"),"native-only subscriptions");
      rig.text("MAVMOOS_NAV_CONTROL","acquire",100.01);rig.host.iterate();require(rig.services.acquisitions==0,"disabled helm acquired");
      rig.text("MAVMOOS_COMMAND","id=one-arm,action=arm",100.02);rig.host.iterate();
      require(rig.services.commands.size()==1&&rig.services.commands.front().command==MAV_CMD_COMPONENT_ARM_DISARM,"native request not handled exactly once");
    });
    test("helm-only facet keeps stock inputs while native requests are disabled",[&]{
      Rig rig(path);rig.host.configure("control_native_actions","false");require(capabilityNames(rig.plugin().descriptor())==helm,"helm-only catalog");
      require(!subscribed(rig.plugin(),"MAVMOOS_COMMAND")&&subscribed(rig.plugin(),"MAVMOOS_NAV_CONTROL"),"helm-only subscriptions");
      rig.text("MAVMOOS_COMMAND","id=disabled-arm,action=arm",100.001);rig.host.iterate();require(rig.services.commands.empty(),"disabled native action sent");
      rig.acquire();require(rig.services.acquisitions==1&&rig.services.targets.size()==1,"stock-helm strategy failed");
    });
    test("disabled public facets retain backend and exclusive ownership without effects",[&]{
      Rig rig(path);rig.host.configure("control_native_actions","false");rig.host.configure("control_helm","false");
      const auto descriptor=rig.plugin().descriptor();require(descriptor.capabilities.empty()&&rig.plugin().subscriptions().empty(),"disabled public surface exposed");
      require(rig.host.controlBackend()!=nullptr&&descriptor.exclusive_resources==std::vector<std::string>{"vehicle.control"}&&descriptor.request_owners.size()==4,"disabled facets lost backend/ownership");
      rig.text("MAVMOOS_COMMAND","id=disabled,action=arm",100.01);rig.text("MAVMOOS_NAV_CONTROL","acquire",100.02);rig.host.inputBoundary();rig.host.iterate();
      require(rig.services.commands.empty()&&rig.services.targets.empty()&&rig.services.acquisitions==0&&rig.services.publications.empty(),"disabled callbacks ran");
    });
    test("facet flags validate booleans without silently changing descriptor",[&]{
      Rig rig(path);const auto before=capabilityNames(rig.plugin().descriptor());
      for(const auto& key:{"control_native_actions","control_helm"}) {bool rejected=false;try {rig.host.configure(key,"yes");}catch(const std::invalid_argument&){rejected=true;}require(rejected,"nonboolean facet accepted");}
      require(capabilityNames(rig.plugin().descriptor())==before,"bad configuration partially enabled/disabled capability");
    });
    test("legacy native grammar remains strict alongside the versioned operation interface",[&]{
      Rig rig(path);rig.text("MAVMOOS_COMMAND","id=new-return,action=return",100.01);rig.text("MAVMOOS_COMMAND","id=new-hold,action=hold,backend=helm",100.02);rig.host.iterate();
      require(rig.services.commands.empty()&&rig.services.published("MAVMOOS_COMMAND_RESULT","reason=invalid_request"),"unqualified workflow unexpectedly accepted");
    });
    test("reset retires native pending operation without retry",[&]{
      Rig rig(path);rig.text("MAVMOOS_COMMAND","id=pending-arm,action=arm",100.01);rig.host.iterate();require(rig.services.commands.size()==1,"initial native submission");
      rig.services.time=100.1;rig.host.reset("vehicle_connection_lost");rig.host.inputBoundary();rig.host.iterate();
      require(rig.services.published("MAVMOOS_COMMAND_RESULT","status=CANCELLED,reason=connection_reset")&&rig.services.finished==std::vector<std::string>{"pending-arm"},"native reset not forwarded");
      require(rig.services.commands.size()==1,"old operation retried after reset");
    });
    test("reset latches helm until fresh explicit acquisition",[&]{
      Rig rig(path);rig.acquire();require(rig.services.targets.size()==1,"initial helm target");rig.services.time=100.2;rig.host.reset("vehicle_connection_lost");
      rig.fields(100.3);rig.host.iterate();require(rig.services.targets.size()==1&&rig.services.published("MAVMOOS_HELM_STATUS","state=LATCHED"),"fresh inputs auto-revived reset helm");
      rig.text("MAVMOOS_NAV_CONTROL","acquire",100.4);rig.fields(100.41);rig.host.iterate();require(rig.services.acquisitions==2&&rig.services.targets.size()==2,"fresh explicit recovery failed");
    });
    std::cout<<"PASS "<<passed<<" combined control packaging groups; real DSO, fake services only\n";return 0;
  }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}

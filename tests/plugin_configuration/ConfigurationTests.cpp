// Real loader/Runtime, trusted adversarial modules, fake clock and sender only.
#include "FixtureAudit.h"
#include "PluginHost.h"
#include "Runtime.h"
#include <algorithm>
#include <dlfcn.h>
#include <functional>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

using namespace mavmoos::runtime;
using namespace configuration_fixture;
namespace {
using Batches=std::map<std::string,std::vector<ModuleSetting>>;
void require(bool condition,const std::string& why) {if(!condition)throw std::runtime_error(why);}
template<class F> void rejects(F operation,const std::string& why)
{bool failed=false;try {operation();}catch(...) {failed=true;}require(failed,why);}
class Image {
  void* m_image=nullptr;
public:
  std::string path;
  Read read=nullptr;Reset reset=nullptr;Control control=nullptr;SetName setName=nullptr;
  explicit Image(const char* file):path(file)
  {
    m_image=dlopen(file,RTLD_NOW|RTLD_LOCAL|RTLD_NODELETE);require(m_image,"fixture dlopen: "+path);
    read=reinterpret_cast<Read>(dlsym(m_image,"configuration_fixture_read"));
    reset=reinterpret_cast<Reset>(dlsym(m_image,"configuration_fixture_reset"));
    control=reinterpret_cast<Control>(dlsym(m_image,"configuration_fixture_control"));
    setName=reinterpret_cast<SetName>(dlsym(m_image,"configuration_fixture_name"));
    require(read&&reset&&control&&setName,"fixture audit exports absent");
  }
  ~Image(){if(m_image)dlclose(m_image);}
  void attack(Hook at,Effect effect) {control(static_cast<unsigned>(at),static_cast<unsigned>(effect));}
  const Audit& audit() const{return *read();}
};
class FakeServices final:public Services {
public:
  unsigned effects=0,diagnostics=0;Vehicle state;
  double now() const override{return 100;}
  const Vehicle& vehicle() const override{return state;}
  void publish(const std::string&,const std::string&) override{++diagnostics;}
  void publish(const std::string&,double) override{++diagnostics;}
  bool effect(){++effects;throw std::runtime_error("unexpected host effect");}
  bool sendCommand(const CommandRequest&) override{return effect();}
  bool acquireNavigation(const std::string&) override{return effect();}
  void revokeNavigation(const std::string&) override{effect();}
  bool navigation(const NavigationTarget&) override{return effect();}
  bool parameter(const ParameterRequest&) override{return effect();}
  bool mission(const MissionRequest&) override{return effect();}
  void missionFinished(const std::string&) override{effect();}
  bool recordStoredMissionProof(const std::string&,const std::vector<MissionItem>&) override{return effect();}
  void invalidateStoredMissionProof(const std::string&) override{effect();}
  bool beginMissionExecution(const std::string&,uint64_t,const std::string&,double) override{return effect();}
  bool beginMissionExecutionAbort(const std::string&,const std::string&,double) override{return effect();}
  bool finishMissionExecution(const std::string&,const std::string&) override{return effect();}
  bool suspendNavigation(double) override{return effect();}
  bool resumeNavigation(const std::string&) override{return effect();}
  void commandFinished(const std::string&) override{effect();}
  void cancelCommand(const std::string&) override{effect();}
  bool flightLog(const FlightLogRequest&) override{return effect();}
  void flightLogFinished(const std::string&) override{effect();}
};
struct Rig {
  unsigned sends=0,publications=0;double clock=100;
  Runtime runtime;
  Rig(bool simulation=true,bool shadow=false,bool deferred=false)
    :runtime([this](const std::string&,const std::string&,double,bool){++publications;},
    [this](const uint8_t*,std::size_t){++sends;return true;},simulation,shadow,[this]{return clock;},
    MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,deferred){}
  void heartbeat()
  {
    mavlink_message_t message{};mavlink_msg_heartbeat_pack(1,1,&message,2,3,0,4,4);
    runtime.message(message,clock);
  }
};
std::vector<ModuleSetting> padding(unsigned count,std::size_t bytes=0)
{
  std::vector<ModuleSetting> settings;
  for(unsigned i=0;i<count;++i)settings.push_back({"pad"+std::to_string(i),std::string(bytes,'x')});
  return settings;
}
void inertAfterFailure(Rig& rig,Image& fixture)
{
  const auto previous=fixture.audit();
  require(rig.runtime.moduleNames().empty(),"failed startup retained modules");
  require(rig.runtime.subscriptions().empty(),"failed startup retained dispatch subscriptions");
  require(!rig.runtime.navigationGranted()&&!rig.runtime.controlBusy(),"failed startup retained authority/lease");
  require(!rig.runtime.storedMissionProof().valid,"failed startup retained stored mission proof");
  require(!rig.runtime.load(fixture.path),"failed startup allowed another module load");
  rejects([&]{rig.runtime.configureModules({});},"failed startup allowed retry without restart");
  rig.heartbeat();Mail mail;mail.key="CONFIGURATION_INPUT";mail.received_at=rig.clock;rig.runtime.mail(mail);
  rig.runtime.tick(rig.clock);
  require(fixture.audit().mail==previous.mail&&fixture.audit().message==previous.message&&
    fixture.audit().iterations==previous.iterations,"failed startup continued feature dispatch");
  require(rig.sends==0,"failed startup emitted MAVLink");
}
}

int main(int argc,char** argv)
{
  unsigned passed=0,failed=0;
  try {
    require(argc==6,"expected alpha bravo default provider ABI9 fixture paths");
    Image alpha(argv[1]),bravo(argv[2]),defaults(argv[3]),provider(argv[4]),abi9(argv[5]);
    auto group=[&](const char* name,const std::function<void()>& test){
      try {
        alpha.reset();bravo.reset();defaults.reset();provider.reset();abi9.reset();test();
        ++passed;std::cout<<"PASS "<<name<<'\n';
      }catch(const std::exception& error){++failed;std::cout<<"FAIL "<<name<<": "<<error.what()<<'\n';}
      catch(...){++failed;std::cout<<"FAIL "<<name<<": non-standard exception\n";}
    };
    group("checked owner isolation and complete empty batches",[&]{
      FakeServices services;PluginHost host;
      require(host.load(alpha.path,services,"configuration_alpha")&&host.load(bravo.path,services,"configuration_bravo"),host.lastError());
      require(host.configureModules({{"configuration_alpha",{{"value","alpha-only"}}}}),host.lastError());
      require(alpha.audit().checked==1&&alpha.audit().last_batch_size==1&&std::string(alpha.audit().value)=="alpha-only","owner batch missing");
      require(bravo.audit().checked==1&&bravo.audit().last_batch_size==0&&std::string(bravo.audit().value)=="default","unconfigured owner did not get one empty batch");
      require(alpha.audit().legacy==0&&bravo.audit().legacy==0&&services.effects==0,"checked settings leaked through flat route");
    });
    group("missing owner denied before any module callback",[&]{
      FakeServices services;PluginHost host;require(host.load(alpha.path,services),host.lastError());
      require(!host.configureModules({{"not_loaded",{{"value","x"}}}}),"unloaded owner accepted");
      require(!host.lastError().empty()&&alpha.audit().checked==0,"missing-owner validation happened after callback");
    });
    group("selected identity mismatch rejects actual instance",[&]{
      FakeServices services;PluginHost host;
      require(!host.load(alpha.path,services,"configuration_bravo")&&host.plugins().empty(),"wrong selected ID accepted");
      require(alpha.audit().constructed==1&&alpha.audit().destroyed==1&&!host.lastError().empty(),"identity rejection leaked instance/diagnostic");
    });
    group("default empty acceptance and nonempty unknown rejection",[&]{
      FakeServices services;PluginHost host;require(host.load(defaults.path,services),host.lastError());
      require(host.configureModules({}),host.lastError());
      require(!host.configureModules({{"configuration_default",{{"value","x"}}}}),"default callback silently accepted settings");
      require(host.lastError().find("value")!=std::string::npos,"default rejection did not identify key");
    });
    group("per-module candidate is atomic on unknown and invalid key",[&]{
      FakeServices services;PluginHost host;require(host.load(alpha.path,services),host.lastError());
      auto* module=host.plugins().front();
      require(module->configureModule({{"value","original"}}).status==ModuleConfigStatus::Accepted,"fixture setup");
      require(module->configureModule({{"value","partial"},{"unknown","bad"}}).status==ModuleConfigStatus::UnknownKey,"unknown key accepted");
      require(std::string(alpha.audit().value)=="original"&&alpha.audit().committed==1,"unknown batch partially committed");
      require(module->configureModule({{"value","reject"}}).status==ModuleConfigStatus::Invalid,"invalid candidate accepted");
      require(std::string(alpha.audit().value)=="original"&&alpha.audit().committed==1,"invalid candidate partially committed");
    });
    group("all owner/key/duplicate structure prevalidated",[&]{
      const std::vector<Batches> bad={
        {{"Configuration_alpha",{{"value","x"}}}},{{"1alpha",{{"value","x"}}}},
        {{"configuration_alpha",{{"Value","x"}}}},{{"configuration_alpha",{{"bad.key","x"}}}},
        {{"configuration_alpha",{{"bad-key","x"}}}},{{"configuration_alpha",{{"","x"}}}},
        {{"configuration_alpha",{{"\xC3\xA9","x"}}}},{{"configuration_alpha",{{"value","one"},{"value","two"}}}},
        {{"configuration_alpha",{{std::string(97,'a'),"x"}}}},{{std::string(97,'a'),{}}}
      };
      for(const auto& batch:bad) {
        alpha.reset();bravo.reset();FakeServices services;PluginHost host;
        require(host.load(alpha.path,services)&&host.load(bravo.path,services),host.lastError());
        require(!host.configureModules(batch),"noncanonical/duplicate setting accepted");
        require(alpha.audit().checked==0&&bravo.audit().checked==0&&!host.lastError().empty(),"structure rejected after callback");
      }
    });
    group("reserved host policy cannot be routed to module",[&]{
      const std::vector<std::string> keys={"simulation_only","shadow_mode","plugins","plugin_directory","sitl_pid","vehicle_profile","copter_frame",
        "target_system","target_component","camera_component","gimbal_component","expected_type","expected_autopilot",
        "bind_address","bind_port","peer_address","peer_port",
        "control_native_actions","control_helm","control_mission_execution","auto_recovery_directory",
        "helm_enabled","firmware_requests","parameter_list_mode","apptick","commstick"};
      for(const auto& key:keys) {
        alpha.reset();FakeServices services;PluginHost host;require(host.load(alpha.path,services),host.lastError());
        require(!host.configureModules({{"configuration_alpha",{{key,"true"}}}}),"reserved key accepted: "+key);
        require(alpha.audit().checked==0,"reserved key delivered to module: "+key);
      }
    });
    group("per-owner setting count bound",[&]{
      FakeServices services;PluginHost host;require(host.load(alpha.path,services),host.lastError());
      require(host.configureModules({{"configuration_alpha",padding(32)}}),host.lastError());
      const auto calls=alpha.audit().checked;
      require(!host.configureModules({{"configuration_alpha",padding(33)}})&&alpha.audit().checked==calls,"33 settings not rejected before callback");
    });
    group("value byte bound",[&]{
      FakeServices services;PluginHost host;require(host.load(alpha.path,services),host.lastError());
      require(host.configureModules({{"configuration_alpha",{{"value",std::string(4096,'x')}}}}),host.lastError());
      require(alpha.audit().value_size==4096,"max value was truncated");const auto calls=alpha.audit().checked;
      require(!host.configureModules({{"configuration_alpha",{{"value",std::string(4097,'x')}}}})&&alpha.audit().checked==calls,"oversized value delivered");
    });
    group("values cannot conceal line or NUL separators",[&]{
      for(const std::string& value:std::vector<std::string>{"one\ntwo","one\rtwo",std::string("one\0two",7)}) {
        alpha.reset();FakeServices services;PluginHost host;require(host.load(alpha.path,services),host.lastError());
        require(!host.configureModules({{"configuration_alpha",{{"value",value}}}})&&alpha.audit().checked==0,"control separator reached callback");
      }
    });
    group("canonical 96-byte key accepted",[&]{
      FakeServices services;PluginHost host;require(host.load(alpha.path,services),host.lastError());
      require(host.configureModules({{"configuration_alpha",{{"pad"+std::string(93,'x'),""}}}}),host.lastError());
      require(alpha.audit().checked==1,"inclusive key limit rejected");
    });
    group("total setting count bound",[&]{
      FakeServices services;PluginHost host;Batches batch;
      for(unsigned i=0;i<5;++i) {
        const auto name="owner"+std::to_string(i);alpha.setName(name.c_str());
        require(host.load(alpha.path,services,name),host.lastError());batch[name]=padding(i<4?32:0);
      }
      require(host.configureModules(batch),host.lastError());const auto calls=alpha.audit().checked;
      batch["owner4"].push_back({"pad0",""});
      require(!host.configureModules(batch)&&alpha.audit().checked==calls,"129 total settings delivered");
    });
    group("owner batch count bound",[&]{
      FakeServices services;PluginHost host;Batches batch;
      for(unsigned i=0;i<32;++i) {
        const auto name="owner"+std::to_string(i);alpha.setName(name.c_str());
        require(host.load(alpha.path,services,name),host.lastError());batch[name]={};
      }
      require(host.configureModules(batch),host.lastError());require(alpha.audit().checked==32,"32-owner complete batch did not reach every owner once");
      batch["owner32"]={};require(!host.configureModules(batch)&&alpha.audit().checked==32,"33-owner batch reached callbacks");
    });
    group("aggregate value bytes rejected before callbacks",[&]{
      FakeServices services;PluginHost host;Batches batch;
      for(unsigned i=0;i<4;++i) {
        const auto name="owner"+std::to_string(i);alpha.setName(name.c_str());
        require(host.load(alpha.path,services,name),host.lastError());batch[name]=padding(32,600);
      }
      require(!host.configureModules(batch)&&alpha.audit().checked==0,"aggregate byte overflow reached module");
    });
    group("inclusive aggregate envelope bytes bound",[&]{
      FakeServices services;PluginHost host;require(host.load(alpha.path,services),host.lastError());
      auto settings=padding(17);std::size_t used=std::string("configuration_alpha").size();
      for(const auto& item:settings)used+=item.key.size();
      auto remaining=65536-used;
      for(auto& item:settings){const auto take=std::min<std::size_t>(4096,remaining);item.value.assign(take,'x');remaining-=take;}
      require(remaining==0,"fixture cannot fill envelope");
      require(host.configureModules({{"configuration_alpha",settings}}),host.lastError());const auto calls=alpha.audit().checked;
      settings.back().value.push_back('x');
      require(!host.configureModules({{"configuration_alpha",settings}})&&alpha.audit().checked==calls,"65537 envelope bytes delivered");
    });
    for(const auto& behavior:{"throw","throw_nonstandard","invalid_status","bad_descriptor","collision","rename",
                             "accepted_with_key","accepted_with_reason","claim_recovery"}) {
      group((std::string("Runtime rejects and latches ")+behavior).c_str(),[&]{
        Rig rig;require(rig.runtime.load(alpha.path)&&rig.runtime.load(bravo.path),"Runtime fixture load");
        rejects([&]{rig.runtime.configureModules({{"configuration_alpha",{{"behavior",behavior}}}});},"bad callback/composition accepted");
        require(alpha.audit().destroyed==1&&bravo.audit().destroyed==1,"failed configuration retained candidate object");
        inertAfterFailure(rig,alpha);
      });
    }
    group("Runtime failed second owner cannot operate first accepted owner",[&]{
      Rig rig;require(rig.runtime.load(alpha.path)&&rig.runtime.load(bravo.path),"Runtime fixture load");
      rejects([&]{rig.runtime.configureModules({{"configuration_alpha",{{"value","accepted-first"}}},{"configuration_bravo",{{"value","reject"}}}});},"second-owner rejection accepted");
      require(alpha.audit().checked==1&&alpha.audit().committed==1,"control did not reach accepted first callback");
      inertAfterFailure(rig,alpha);
    });
    group("provider backend replacement refreshed before first input",[&]{
      Rig rig;require(rig.runtime.load(provider.path),"provider load");
      rig.runtime.configureModules({{"configuration_provider",{{"behavior","replace_backend"}}}});
      require(provider.audit().backend_replacements==1,"replacement control absent");rig.heartbeat();
      require(rig.runtime.vehicle().heartbeat_valid,"Runtime retained pre-configuration backend pointer/policy");
    });
    group("provider descriptor is revalidated after module configuration",[&]{
      Rig rig;require(rig.runtime.load(provider.path),"provider load");
      rejects([&]{rig.runtime.configureModules({{"configuration_provider",{{"behavior","bad_profile"}}}});},"provider profile mismatch accepted");
      inertAfterFailure(rig,provider);
    });
    group("final control profile follows replaced provider policy",[&]{
      Rig rig;require(rig.runtime.load(provider.path),"provider load");
      require(rig.runtime.controlProfile()=="arducopter","initial provider profile missing");
      rig.runtime.configureModules({{"configuration_provider",{{"behavior","replace_rover"}}}});
      require(rig.runtime.controlProfile()=="ardurover","final provider policy getter retained old backend");
    });
    group("deferred startup denies effect from final subscription getter",[&]{
      Rig rig(false,true,true);require(rig.runtime.load(provider.path,"configuration_provider"),"deferred provider load");
      rig.runtime.configureModules({{"configuration_provider",{{"behavior","subscription_effect"}}}});
      require(provider.audit().attempted==0&&!rig.runtime.navigationGranted(),"control did not isolate post-batch query");
      require(rig.runtime.subscriptions()==std::vector<std::string>{"CONFIGURATION_INPUT"},"final startup subscription query did not run");
      require(provider.audit().attempted==1&&provider.audit().accepted==0&&!rig.runtime.navigationGranted(),"final query acquired shadow authority");
      (void)rig.runtime.capabilities(); // Unrelated read cannot erase the sticky attempt.
      rejects([&]{rig.runtime.finishStartup();},"final startup accepted a deferred effect attempt");
      require(rig.runtime.lastConfigurationError().find("effect during startup validation")!=std::string::npos,"deferred failure diagnostic missing");
      require(provider.audit().destroyed==1,"failed final validation retained provider");
      inertAfterFailure(rig,provider);
    });
    group("deferred effect remains sticky across a later legacy callback",[&]{
      Rig rig(false,true,true);require(rig.runtime.load(provider.path),"deferred provider load");
      rig.runtime.configureModules({{"configuration_provider",{{"behavior","subscription_effect"}}}});
      (void)rig.runtime.subscriptions();
      require(provider.audit().attempted==1&&provider.audit().accepted==0,"sticky attempt control did not run");
      rejects([&]{rig.runtime.configure("legacy_setting","x");},"later legacy callback erased deferred effect attempt");
      inertAfterFailure(rig,provider);
    });
    group("deferred startup blocks input and dispatch until explicit finish",[&]{
      Rig rig(false,false,true);require(rig.runtime.load(alpha.path),"deferred observer load");
      rig.runtime.configureModules({{"configuration_alpha",{{"value","configured"}}}});
      require(rig.runtime.subscriptions()==std::vector<std::string>{"CONFIGURATION_INPUT"},"read-only final startup validation denied");
      Mail mail;mail.key="CONFIGURATION_INPUT";mail.received_at=rig.clock;
      rig.heartbeat();rig.runtime.mail(mail);rig.runtime.tick(rig.clock);
      require(!rig.runtime.vehicle().heartbeat_valid&&alpha.audit().message==0&&alpha.audit().mail==0&&alpha.audit().iterations==0&&rig.sends==0,"input/dispatch escaped before startup finish");
      rig.runtime.finishStartup();rig.heartbeat();rig.runtime.mail(mail);rig.runtime.tick(rig.clock);
      require(rig.runtime.vehicle().heartbeat_valid&&alpha.audit().message==1&&alpha.audit().mail==1&&alpha.audit().iterations==1&&rig.sends==0,"clean finish failed to enable ordinary receive-only dispatch");
      rejects([&]{rig.runtime.finishStartup();},"second finish accepted");
      require(rig.runtime.moduleNames().size()==1,"rejected second finish discarded operational composition");
    });
    group("deferred finish requires checked complete batch",[&]{
      Rig rig(true,false,true);require(rig.runtime.load(alpha.path),"deferred fixture load");
      rejects([&]{rig.runtime.finishStartup();},"startup unlocked before checked batch");
      rig.runtime.tick(rig.clock);require(rig.sends==0&&alpha.audit().iterations==0,"early finish released guarded startup");
      rig.runtime.configureModules({});rig.runtime.finishStartup();rig.runtime.tick(rig.clock);
      require(alpha.audit().checked==1&&alpha.audit().iterations==1,"valid startup could not finish after denied early request");
    });
    group("checked successful configuration is startup-only",[&]{
      Rig rig;require(rig.runtime.load(alpha.path),"Runtime load");rig.runtime.configureModules({});
      rejects([&]{rig.runtime.configureModules({{"configuration_alpha",{{"value","again"}}}});},"checked startup configured twice");
      require(!rig.runtime.load(bravo.path),"checked startup allowed a later module load");
      require(alpha.audit().checked==1&&alpha.audit().destroyed==0,"repeated configuration mutated/discarded current startup");
      rig.runtime.tick(rig.clock);
      require(alpha.audit().iterations==1&&rig.runtime.moduleNames().size()==1&&
        rig.runtime.status().find("configuration_valid=true")!=std::string::npos,"rejected repeat interrupted current composition");
    });
    for(const std::string boundary:{"message","mail","tick","checkpoint"}) {
      group(("checked configuration rejected after "+boundary).c_str(),[&]{
        Rig rig;require(rig.runtime.load(alpha.path),"Runtime load");
        if(boundary=="message")rig.heartbeat();
        if(boundary=="mail"){Mail mail;mail.received_at=rig.clock;rig.runtime.mail(mail);}
        if(boundary=="tick")rig.runtime.tick(rig.clock);
        if(boundary=="checkpoint")rig.runtime.checkpoint(rig.clock);
        rejects([&]{rig.runtime.configureModules({});},"late configuration accepted after "+boundary);
        require(alpha.audit().checked==0&&alpha.audit().destroyed==0,"late configuration changed the active module");
        const auto iterations=alpha.audit().iterations;rig.runtime.tick(rig.clock);
        require(alpha.audit().iterations==iterations+1&&rig.runtime.moduleNames().size()==1,"denied late call interrupted current composition");
      });
    }
    group("legacy flat route still dispatches separately",[&]{
      Rig rig;require(rig.runtime.load(alpha.path),"Runtime load");rig.runtime.configure("legacy_setting","value");
      require(alpha.audit().legacy==1&&alpha.audit().checked==0,"legacy route changed");
      rig.runtime.configureModules({});require(alpha.audit().checked==1,"legacy route prevented checked startup");
    });
    group("legacy configuration cannot change selected module identity",[&]{
      Rig rig;require(rig.runtime.load(alpha.path,"configuration_alpha"),"Runtime selected load");
      rejects([&]{rig.runtime.configure("mutate_name","true");},"legacy configuration changed owner identity");
      inertAfterFailure(rig,alpha);
    });
    group("factory ABI9 rejects host ABI10 before object construction",[&]{
      FakeServices services;PluginHost host;
      require(plugin_abi==10,"this ABI transition test must be reviewed when ABI changes");
      require(!host.load(abi9.path,services)&&host.plugins().empty(),"ABI9 factory accepted ABI10");
      require(abi9.audit().constructed==0&&host.lastError().find("ABI")!=std::string::npos,"ABI rejection did not precede object construction");
    });
    for(const auto hook:{Hook::Factory,Hook::Checked,Hook::Legacy,Hook::Destroy}) {
      for(unsigned selected=1;selected<static_cast<unsigned>(Effect::Count);++selected) {
        const auto label="effect guard hook="+std::to_string(static_cast<unsigned>(hook))+" service="+std::to_string(selected);
        group(label.c_str(),[&]{
          Rig rig;alpha.attack(hook,static_cast<Effect>(selected));
          if(hook==Hook::Factory) {
            require(!rig.runtime.load(alpha.path),"effectful factory accepted");
          } else {
            require(rig.runtime.load(alpha.path),"effect fixture load");
            if(hook==Hook::Legacy)rejects([&]{rig.runtime.configure("legacy_setting","x");},"effectful legacy configure accepted");
            else if(hook==Hook::Checked)rejects([&]{rig.runtime.configureModules({});},"effectful checked callback accepted");
            else rejects([&]{rig.runtime.configureModules({{"configuration_alpha",{{"unknown","x"}}}});},"discard fixture rejection absent");
          }
          require(alpha.audit().attempted==1&&alpha.audit().accepted==0,"effect attempt missing or was accepted");
          inertAfterFailure(rig,alpha);
        });
      }
    }
    std::cout<<(failed?"FAIL":"PASS")<<" plugin configuration "<<passed<<" passed / "<<failed
             <<" failed groups; real PluginHost/Runtime and DSOs; fake I/O only\n";
    return failed?1:0;
  }catch(const std::exception& error){std::cerr<<"FAIL after "<<passed<<" groups: "<<error.what()<<'\n';return 1;}
}

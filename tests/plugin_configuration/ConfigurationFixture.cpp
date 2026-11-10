#include "FixtureAudit.h"
#include "ControlBackend.h"
#include <algorithm>
#include <cstring>
#include <memory>
#include <stdexcept>

using namespace mavmoos::runtime;
using namespace configuration_fixture;
#ifndef CONFIGURATION_FIXTURE_NAME
#define CONFIGURATION_FIXTURE_NAME "configuration_alpha"
#endif
namespace {
Audit audit;
Hook hook=Hook::None;
Effect selected=Effect::None;
std::string next_name=CONFIGURATION_FIXTURE_NAME;
[[maybe_unused]] void copy(char* out,std::size_t capacity,const std::string& value)
{
  const auto count=std::min(capacity-1,value.size());
  std::memcpy(out,value.data(),count);out[count]=0;
}
void attempt(Services& services,Effect effect=selected)
{
  ++audit.attempted;
  try {
    bool accepted=false;
    CommandRequest command;command.id="configuration-command";command.command=MAV_CMD_DO_SET_MODE;
    command.params[0]=1;command.params[1]=4;command.expires=services.now()+1;
    ParameterRequest parameter;parameter.id="configuration-parameter";parameter.name="FRAME_CLASS";
    parameter.expires=services.now()+1;
    MissionRequest mission;mission.id="configuration-mission";mission.operation_id=mission.id;
    mission.expires=services.now()+1;mission.operation_expires=services.now()+2;
    switch(effect) {
    case Effect::Command: accepted=services.sendCommand(command);break;
    case Effect::Acquire: accepted=services.acquireNavigation("configuration-acquire");break;
    case Effect::Revoke: services.revokeNavigation("configuration-revoke");break;
    case Effect::Navigation: accepted=services.navigation({90,1,5,services.now()+1});break;
    case Effect::Parameter: accepted=services.parameter(parameter);break;
    case Effect::Mission: accepted=services.mission(mission);break;
    case Effect::MissionFinished: services.missionFinished(mission.id);break;
    case Effect::RecordProof: accepted=services.recordStoredMissionProof("configuration-proof",{});break;
    case Effect::InvalidateProof: services.invalidateStoredMissionProof("configuration-proof");break;
    case Effect::BeginExecution: accepted=services.beginMissionExecution("configuration-execution",1,"content",services.now()+1);break;
    case Effect::BeginAbort: accepted=services.beginMissionExecutionAbort("configuration-execution","configuration-abort",services.now()+1);break;
    case Effect::FinishExecution: accepted=services.finishMissionExecution("configuration-execution","RELEASED");break;
    case Effect::Suspend: accepted=services.suspendNavigation(services.now()+1);break;
    case Effect::Resume: accepted=services.resumeNavigation("configuration-resume");break;
    case Effect::CommandFinished: services.commandFinished(command.id);break;
    case Effect::CancelCommand: services.cancelCommand(command.id);break;
    case Effect::FlightLog: accepted=services.flightLog({});break;
    case Effect::FlightLogFinished: services.flightLogFinished("configuration-log");break;
    default: break;
    }
    if(accepted)++audit.accepted;
  } catch(...) {++audit.caught;} // A hostile module cannot erase the host's violation latch.
}
#ifdef CONFIGURATION_PROVIDER
class Backend final:public VehicleControlBackend {
  bool m_accept;
  std::string m_profile;
public:
  explicit Backend(bool accept,std::string profile="arducopter"):m_accept(accept),m_profile(std::move(profile)) {}
  std::string profile() const override {return m_profile;}
  bool identityCompatible(unsigned type,unsigned autopilot) const override {return m_accept&&type==2&&autopilot==3;}
  bool flightReady(const Vehicle&,double,bool) const override {return false;}
  bool acquisitionAllowed(const Vehicle&,double) const override {return false;}
  bool activeModeAllowed(const Vehicle&,bool) const override {return false;}
  std::optional<CommandRequest> acquisitionCommand(const Vehicle&,const std::string&,double) const override {return {};}
  bool commandAllowed(const CommandRequest&,const Vehicle&,double) const override {return false;}
  bool navigationInputAllowed(const NavigationTarget&) const override {return false;}
  bool helmFieldAllowed(unsigned,double) const override {return false;}
  bool helmVehicleReady(const Vehicle&,double,double,bool,bool) const override {return false;}
  std::optional<ControlFrame> navigationFrame(const NavigationTarget&,const Vehicle&,double,uint64_t,mavlink_status_t&) const override {return {};}
  std::optional<ControlFrame> stopFrame(const Vehicle&,double,uint64_t,mavlink_status_t&) const override {return {};}
};
#endif
class Fixture final:public Plugin {
  Services& m_services;
  std::string m_name=next_name,m_value="default",m_behavior;
#ifdef CONFIGURATION_PROVIDER
  std::unique_ptr<Backend> m_backend=std::make_unique<Backend>(false);
#endif
public:
  explicit Fixture(Services& services):m_services(services)
  {++audit.constructed;if(hook==Hook::Factory)attempt(m_services);}
  ~Fixture() override {++audit.destroyed;if(hook==Hook::Destroy)attempt(m_services);}
  std::string name() const override {return m_name;}
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor d;d.profile="configuration_profile";d.streamed_parameter_bench_compatible=true;
    d.capabilities={{"configuration."+m_name,"","","observation"}};
#ifdef CONFIGURATION_PROVIDER
    d.profile=m_backend->profile();
    d.exclusive_resources={"vehicle.control"};
#endif
    if(m_behavior=="bad_descriptor")d.capabilities={{"configuration.request","CONFIGURATION_REQUEST","send","request"}};
    if(m_behavior=="collision")d.capabilities={{"configuration.configuration_bravo","","","observation"}};
    if(m_behavior=="bad_profile")d.profile="wrong_profile";
    if(m_behavior=="bench_incompatible")d.streamed_parameter_bench_compatible=false;
    if(m_behavior=="claim_mission")d.capabilities.push_back({"control.mission.start","","","internal"});
    if(m_behavior=="claim_recovery")d.request_owners.push_back("MAVMOOS_AUTO_RECOVERY_REQUEST");
    return d;
  }
  const VehicleControlBackend* controlBackend() const override {
#ifdef CONFIGURATION_PROVIDER
    return m_backend.get();
#else
    return nullptr;
#endif
  }
  std::vector<std::string> subscriptions() const override
  {
    // Composition validation never calls this getter. The app first uses it
    // for initial registration after the checked callback has committed.
    if(m_behavior=="subscription_effect")attempt(m_services,Effect::Acquire);
    return {"CONFIGURATION_INPUT"};
  }
  void configure(const std::string& key,const std::string&) override
  {
    ++audit.legacy;if(hook==Hook::Legacy)attempt(m_services);
    if(key=="mutate_name")m_name="renamed_owner";
  }
#ifndef CONFIGURATION_DEFAULT_ONLY
  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override
  {
    if(audit.checked<64)copy(audit.callbacks[audit.checked],97,m_name);
    ++audit.checked;audit.last_batch_size=static_cast<unsigned>(settings.size());
    if(hook==Hook::Checked)attempt(m_services);
    // Diagnostics and read-only queries are expressly allowed at startup.
    m_services.publish("CONFIGURATION_DIAGNOSTIC","checked");
    if(m_services.navigationGranted()||m_services.missionExecutionGranted("configuration-execution"))++audit.query_grants;
    (void)m_services.now();(void)m_services.vehicle();(void)m_services.controlBusy();
    (void)m_services.storedMissionProof();double x=0,y=0;(void)m_services.projectGlobal(0,0,x,y);
    std::string candidate="default",behavior;
    for(const auto& setting:settings) {
      if(setting.key=="value")candidate=setting.value;
      else if(setting.key=="behavior")behavior=setting.value;
      else if(setting.key.rfind("pad",0)==0) {} // Opaque padding only for host-boundary tests.
      else return {ModuleConfigStatus::UnknownKey,setting.key,"fixture unknown key"};
    }
    if(candidate=="reject")return {ModuleConfigStatus::Invalid,"value","fixture rejected candidate"};
    if(behavior=="throw")throw std::runtime_error("fixture checked exception");
    if(behavior=="throw_nonstandard")throw 17;
    if(behavior=="invalid_status")return {static_cast<ModuleConfigStatus>(99),"behavior","invalid enum"};
    if(behavior=="accepted_with_key")return {ModuleConfigStatus::Accepted,"value",""};
    if(behavior=="accepted_with_reason")return {ModuleConfigStatus::Accepted,"","ambiguous acceptance"};
    m_value=candidate;m_behavior=behavior;
    if(behavior=="rename")m_name="renamed_owner";
#ifdef CONFIGURATION_PROVIDER
    if(behavior=="replace_backend") {
      auto replacement=std::make_unique<Backend>(true);m_backend.swap(replacement);++audit.backend_replacements;
    }
    if(behavior=="replace_rover") {
      auto replacement=std::make_unique<Backend>(true,"ardurover");m_backend.swap(replacement);++audit.backend_replacements;
    }
#endif
    ++audit.committed;audit.value_size=m_value.size();copy(audit.value,sizeof(audit.value),m_value);
    return {};
  }
#endif
  void onMail(const Mail&) override {++audit.mail;}
  void onMessage(const mavlink_message_t&,double) override {++audit.message;}
  void iterate() override {++audit.iterations;}
  void onInputBoundary() override {++audit.input_boundaries;}
  void onReset(const std::string&) override {++audit.resets;}
};
}
extern "C" Plugin* mavmoos_create_plugin(unsigned abi,Services* services)
{
#ifdef CONFIGURATION_ABI9
  if(abi!=9||!services)return nullptr;
#else
  if(abi!=plugin_abi||!services)return nullptr;
#endif
  return new Fixture(*services);
}
extern "C" void mavmoos_destroy_plugin(Plugin* plugin) {delete plugin;}
extern "C" const Audit* configuration_fixture_read() {return &audit;}
extern "C" void configuration_fixture_reset() {audit={};hook=Hook::None;selected=Effect::None;next_name=CONFIGURATION_FIXTURE_NAME;}
extern "C" void configuration_fixture_control(unsigned at,unsigned effect) {hook=static_cast<Hook>(at);selected=static_cast<Effect>(effect);}
extern "C" void configuration_fixture_name(const char* value) {next_name=value;}

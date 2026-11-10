#include "PluginHost.h"
#include "ControlBackend.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <dlfcn.h>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
bool localName(const std::string& name)
{
  if(name.empty()||name.size()>96||name.front()<'a'||name.front()>'z') return false;
  return std::all_of(name.begin(),name.end(),[](unsigned char c) {
    return (c>='a'&&c<='z')||(c>='0'&&c<='9')||c=='_';
  });
}
bool reservedPolicyKey(const std::string& key)
{
  static const std::set<std::string> reserved={
    "simulation_only","shadow_mode","plugins","plugin_directory","sitl_pid","vehicle_profile","copter_frame",
    "bind_address","bind_port","peer_address","peer_port","target_system","target_component",
    "camera_component","gimbal_component","remote_id_component","component_metadata_component",
    "expected_type","expected_autopilot","helm_enabled","firmware_requests","parameter_list_mode",
    "control_native_actions","control_helm","control_mission_execution","auto_recovery_directory","control_replay_directory","apptick","commstick"};
  return reserved.count(key)!=0;
}
bool token(const std::string& text,bool empty=false)
{
  return (empty||!text.empty()) && text.size()<=96 &&
    std::all_of(text.begin(),text.end(),[](unsigned char c) {
      return std::isalnum(c)||c=='_'||c=='.'||c=='-';
    });
}
bool printable(const std::string& text)
{
  return text.size()<=256 && std::all_of(text.begin(),text.end(),[](unsigned char c){return c>=32&&c<=126;});
}
void validateComposition(const std::vector<Plugin*>& plugins)
{
  if(plugins.size()>32) throw std::runtime_error("too many plugins");
  std::set<std::string> names,capabilities,resources,requests;
  unsigned backends=0;
  for(const auto* plugin:plugins) {
    const auto name=plugin->name();
    if(!token(name)||!names.insert(name).second) throw std::runtime_error("invalid or duplicate plugin name: "+name);
    const auto d=plugin->descriptor();
    if(!token(d.profile,true)||d.capabilities.size()>64||d.exclusive_resources.size()>32||
       d.request_owners.size()>32||d.observation_streams.size()>32)
      throw std::runtime_error("invalid descriptor: "+name);
    for(const auto& capability:d.capabilities) {
      if(!token(capability.name)||!token(capability.request_variable,true)||!printable(capability.request_value)||
         (capability.access!="request"&&capability.access!="internal"&&capability.access!="observation"&&capability.access!="mission_behavior")||
         (capability.access=="request"&&capability.request_variable.empty()))
        throw std::runtime_error("invalid capability descriptor: "+name);
      if(!capabilities.insert(capability.name).second)
        throw std::runtime_error("duplicate capability: "+capability.name);
      if(capability.access=="request"&&
         std::find(d.request_owners.begin(),d.request_owners.end(),capability.request_variable)==d.request_owners.end())
        throw std::runtime_error("request capability lacks declared owner: "+capability.request_variable);
    }
    for(const auto& resource:d.exclusive_resources)
      if(!token(resource)||!resources.insert(resource).second)
        throw std::runtime_error("invalid or duplicate exclusive resource: "+resource);
    for(const auto& request:d.request_owners)
      if(!token(request)||request=="MAVMOOS_AUTO_RECOVERY_REQUEST"||!requests.insert(request).second)
        throw std::runtime_error("invalid or duplicate request owner: "+request);
    std::set<uint32_t> stream_ids;
    for(const auto& stream:d.observation_streams)
      if(stream.message_id>0xFFFFFF||!std::isfinite(stream.interval_us)||stream.interval_us<10000||
         stream.interval_us>60000000||!stream_ids.insert(stream.message_id).second)
        throw std::runtime_error("invalid observation stream: "+name);
    if(const auto* backend=plugin->controlBackend()) {
      if(++backends>1) throw std::runtime_error("duplicate vehicle-control provider");
      if(d.profile.empty()||backend->profile()!=d.profile||
         std::find(d.exclusive_resources.begin(),d.exclusive_resources.end(),"vehicle.control")==d.exclusive_resources.end())
        throw std::runtime_error("vehicle-control provider descriptor mismatch: "+name);
    }
  }
}
void jsonStrings(std::ostream& out,const std::vector<std::string>& values)
{
  out<<'[';
  for(std::size_t i=0;i<values.size();++i) { if(i)out<<',';out<<std::quoted(values[i]); }
  out<<']';
}
}
struct PluginHost::Entry {
  void* library=nullptr;
  Plugin* plugin=nullptr;
  DestroyPlugin destroy=nullptr;
  ~Entry() {
    // Objects are owned normally; NODELETE keeps startup feature code resident
    // after logical handle release. Live unloading/replacement is unsupported.
    if(plugin && destroy) destroy(plugin);
    if(library) dlclose(library);
  }
};

PluginHost::PluginHost()=default;
PluginHost::~PluginHost() { clear(); }

bool PluginHost::load(const std::string& path,Services& services,const std::string& expected_name)
{
  m_error.clear();
  if(!expected_name.empty()&&!localName(expected_name)) {
    m_error="invalid selected module ID: "+expected_name;return false;
  }
  auto entry=std::make_unique<Entry>();
  entry->library=dlopen(path.c_str(),RTLD_NOW|RTLD_LOCAL|RTLD_NODELETE);
  if(!entry->library) {
    const char* error=dlerror();
    m_error=error ? error : "dlopen failed";
    return false;
  }
  dlerror();
  const auto create=reinterpret_cast<CreatePlugin>(dlsym(entry->library,"mavmoos_create_plugin"));
  const char* create_error=dlerror();
  if(create_error || !create) {
    m_error="missing mavmoos_create_plugin export";
    return false;
  }
  dlerror();
  entry->destroy=reinterpret_cast<DestroyPlugin>(dlsym(entry->library,"mavmoos_destroy_plugin"));
  const char* destroy_error=dlerror();
  if(destroy_error || !entry->destroy) {
    m_error="missing mavmoos_destroy_plugin export";
    return false;
  }
  try {
    entry->plugin=create(plugin_abi,&services);
    if(!entry->plugin) throw std::runtime_error("plugin rejected ABI or service context");
    if(!expected_name.empty()&&entry->plugin->name()!=expected_name)
      throw std::runtime_error("selected module ID does not match Plugin::name(): "+expected_name);
    auto proposed=plugins(); proposed.push_back(entry->plugin);
    validateComposition(proposed);
    m_entries.push_back(std::move(entry));
  } catch(const std::exception& error) {
    m_error=error.what();
    return false;
  } catch(...) {
    m_error="plugin factory threw an unknown exception";
    return false;
  }
  return true;
}

std::vector<Plugin*> PluginHost::plugins() const
{
  std::vector<Plugin*> out;
  for(const auto& entry:m_entries) out.push_back(entry->plugin);
  return out;
}

bool PluginHost::hasCapability(const std::string& name) const
{
  for(const auto* plugin:plugins()) for(const auto& capability:plugin->descriptor().capabilities)
    if(capability.name==name) return true;
  return false;
}
bool PluginHost::streamedParameterBenchCompatible() const
{
  for(const auto* plugin:plugins())
    if(plugin->controlBackend()||!plugin->descriptor().streamed_parameter_bench_compatible) return false;
  return true;
}
const VehicleControlBackend* PluginHost::controlBackend() const
{
  for(const auto* plugin:plugins()) if(const auto* backend=plugin->controlBackend()) return backend;
  return nullptr;
}
bool PluginHost::projectGlobal(double latitude,double longitude,double& x,double& y) const
{
  for(const auto* plugin:plugins()) {
    const auto resources=plugin->descriptor().exclusive_resources;
    if(std::find(resources.begin(),resources.end(),"projection.navigation")!=resources.end())
      return plugin->projectGlobal(latitude,longitude,x,y);
  }
  return false;
}
std::vector<ObservationStream> PluginHost::observationStreams() const
{
  std::vector<ObservationStream> result;
  for(const auto* plugin:plugins()) for(const auto& request:plugin->descriptor().observation_streams) {
    auto current=std::find_if(result.begin(),result.end(),[&](const auto& item){return item.message_id==request.message_id;});
    if(current==result.end()) result.push_back(request);
    else current->interval_us=std::min(current->interval_us,request.interval_us);
  }
  return result;
}
std::string PluginHost::catalog() const
{
  std::ostringstream out;
  out<<"{\"plugin_abi\":"<<plugin_abi<<",\"modules\":[";
  bool first=true;
  for(const auto* plugin:plugins()) {
    if(!first) out<<',';
    first=false;
    const auto d=plugin->descriptor();
    out<<"{\"name\":"<<std::quoted(plugin->name())<<",\"profile\":"<<std::quoted(d.profile)
       <<",\"control_provider\":"<<(plugin->controlBackend()?"true":"false")<<",\"exclusive_resources\":";
    jsonStrings(out,d.exclusive_resources);out<<",\"request_owners\":";jsonStrings(out,d.request_owners);
    out<<",\"streamed_parameter_bench_compatible\":"<<(d.streamed_parameter_bench_compatible?"true":"false")<<",\"capabilities\":[";
    for(std::size_t i=0;i<d.capabilities.size();++i) {
      if(i) out<<',';
      const auto& c=d.capabilities[i];
      out<<"{\"name\":"<<std::quoted(c.name)<<",\"request_variable\":"<<std::quoted(c.request_variable)
         <<",\"request_value\":"<<std::quoted(c.request_value)<<",\"access\":"<<std::quoted(c.access)<<'}';
    }
    out<<"]}";
  }
  out<<"]}";return out.str();
}

std::vector<std::string> PluginHost::subscriptions() const
{
  std::set<std::string> names;
  for(const auto& entry:m_entries)
    for(const auto& key:entry->plugin->subscriptions()) names.insert(key);
  return {names.begin(),names.end()};
}

void PluginHost::configure(const std::string& key,const std::string& value)
{
  std::vector<std::string> names;
  for(const auto& entry:m_entries) names.push_back(entry->plugin->name());
  for(const auto& entry:m_entries) entry->plugin->configure(key,value);
  for(std::size_t i=0;i<m_entries.size();++i)
    if(m_entries[i]->plugin->name()!=names[i]) throw std::invalid_argument("module identity changed during configuration: "+names[i]);
  validateComposition(plugins());
}

bool PluginHost::configureModules(const std::map<std::string,std::vector<ModuleSetting>>& batches)
{
  m_error.clear();
  try {
    const auto selected=plugins();
    validateComposition(selected);
    if(batches.size()>32) throw std::invalid_argument("too many module configuration owners");
    std::set<std::string> owners;std::vector<std::string> names;
    for(const auto* plugin:selected) {owners.insert(plugin->name());names.push_back(plugin->name());}
    std::size_t count=0,bytes=0;
    // Reject the complete host envelope before invoking any module callback.
    for(const auto& [owner,settings]:batches) {
      if(!localName(owner)||!owners.count(owner))
        throw std::invalid_argument("unknown or invalid module configuration owner: "+owner);
      if(settings.size()>32) throw std::invalid_argument(owner+": too many local settings");
      std::set<std::string> keys;
      bytes+=owner.size();
      for(const auto& setting:settings) {
        if(!localName(setting.key)||!keys.insert(setting.key).second)
          throw std::invalid_argument(owner+": invalid or duplicate local key: "+setting.key);
        if(reservedPolicyKey(setting.key))
          throw std::invalid_argument(owner+": reserved host policy key: "+setting.key);
        if(setting.value.size()>4096||setting.value.find_first_of("\r\n")!=std::string::npos||
           setting.value.find('\0')!=std::string::npos)
          throw std::invalid_argument(owner+"."+setting.key+": invalid or oversized value");
        bytes+=setting.key.size()+setting.value.size();
        if(++count>128||bytes>65536) throw std::invalid_argument("module configuration batch exceeds limits");
      }
    }
    const std::vector<ModuleSetting> empty;
    for(std::size_t i=0;i<selected.size();++i) {
      auto* plugin=selected[i];const auto& name=names[i];
      const auto found=batches.find(name);
      const auto result=plugin->configureModule(found==batches.end()?empty:found->second);
      if(result.status==ModuleConfigStatus::Accepted&&(!result.key.empty()||!result.reason.empty()))
        throw std::invalid_argument(name+": Accepted configuration result must not contain error fields");
      if(result.status!=ModuleConfigStatus::Accepted) {
        const auto kind=result.status==ModuleConfigStatus::UnknownKey?"unknown local setting":"invalid local configuration";
        throw std::invalid_argument(plugin->name()+": "+kind+" "+result.key.substr(0,96)+": "+result.reason.substr(0,512));
      }
    }
    for(std::size_t i=0;i<selected.size();++i)
      if(selected[i]->name()!=names[i]) throw std::invalid_argument("module identity changed during configuration: "+names[i]);
    validateComposition(plugins());
    return true;
  } catch(const std::exception& error) {
    m_error=error.what();
  } catch(...) { m_error="module configuration threw an unknown exception"; }
  return false;
}

void PluginHost::dispatchMail(const Mail& mail)
{
  for(const auto& entry:m_entries) {
    const auto keys=entry->plugin->subscriptions();
    if(std::find(keys.begin(),keys.end(),mail.key)!=keys.end()) entry->plugin->onMail(mail);
  }
}

void PluginHost::dispatchMessage(const mavlink_message_t& message,double received_at)
{
  dispatchMessage(message,MessageTime{received_at,received_at,"READ_ACQUISITION"});
}

void PluginHost::dispatchMessage(const mavlink_message_t& message,const MessageTime& time)
{
  for(const auto& entry:m_entries) entry->plugin->onMessageWithMetadata(message,time);
}

void PluginHost::parameterSent(const ParameterRequest& request,double sent_at,bool accepted)
{
  for(const auto& entry:m_entries) entry->plugin->onParameterSent(request,sent_at,accepted);
}
void PluginHost::missionSent(const MissionRequest& request,double sent_at,bool accepted)
{
  for(const auto& entry:m_entries) entry->plugin->onMissionSent(request,sent_at,accepted);
}
void PluginHost::flightLogSent(const FlightLogRequest& request,double sent_at,bool accepted)
{
  for(const auto& entry:m_entries) entry->plugin->onFlightLogSent(request,sent_at,accepted);
}
void PluginHost::fileTransferSent(const FileTransferRequest& request,double sent_at,bool accepted)
{
  for(const auto& entry:m_entries)
    if(auto* listener=dynamic_cast<FileTransferListener*>(entry->plugin))
      listener->onFileTransferSent(request,sent_at,accepted);
}
void PluginHost::externalNavigationSent(const ExternalNavigationSample& sample,double at,bool accepted)
{
  for(const auto& entry:m_entries)
    if(auto* listener=dynamic_cast<ExternalNavigationListener*>(entry->plugin))
      listener->onExternalNavigationSent(sample,at,accepted);
}

void PluginHost::payloadSent(const PayloadRequest& request,double sent_at,bool accepted)
{
  for(const auto& entry:m_entries)
    if(auto* listener=dynamic_cast<PayloadListener*>(entry->plugin))
      listener->onPayloadSent(request,sent_at,accepted);
}
void PluginHost::commandSent(const CommandRequest& request,double sent_at,bool accepted)
{
  for(const auto& entry:m_entries) entry->plugin->onCommandSent(request,sent_at,accepted);
}

void PluginHost::iterate()
{
  for(const auto& entry:m_entries) entry->plugin->iterate();
}

void PluginHost::inputBoundary()
{
  for(const auto& entry:m_entries) entry->plugin->onInputBoundary();
}

void PluginHost::reset(const std::string& reason)
{
  for(const auto& entry:m_entries) entry->plugin->onReset(reason);
}

void PluginHost::clear()
{
  while(!m_entries.empty()) m_entries.pop_back();
}
} // namespace mavmoos::runtime

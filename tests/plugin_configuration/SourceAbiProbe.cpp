// Manual fresh-consumer proof: a source-built frozen ABI9 DSO must reject SDK10.
// The old factory is never invoked with ABI9 in this SDK10 process.
#include "PluginHost.h"
#include <dlfcn.h>
#include <iostream>
#include <stdexcept>
using namespace mavmoos::runtime;
namespace {
void require(bool good,const std::string& why){if(!good)throw std::runtime_error(why);}
class Guard final:public Services {
public:
  Vehicle state;unsigned effects=0,publications=0;
  double now() const override{return 100;}
  const Vehicle& vehicle() const override{return state;}
  void publish(const std::string&,const std::string&) override{++publications;}
  void publish(const std::string&,double) override{++publications;}
  bool effect(){++effects;throw std::runtime_error("consumer observer attempted an effect");}
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
}
int main(int argc,char** argv)
{
  try {
    require(argc==3,"usage: plugin_configuration_source_abi_probe actual-source9.so actual-source10.so");
    require(plugin_abi==10,"review this transition proof when the development ABI changes");
    Guard services;
    // A valid native image and both factory exports are required. A Mach-O/ELF
    // format error is not an ABI rejection and cannot satisfy this control.
    void* image=dlopen(argv[1],RTLD_NOW|RTLD_LOCAL|RTLD_NODELETE);
    require(image!=nullptr,"frozen ABI9 module is not a loadable native image");
    require(dlsym(image,"mavmoos_create_plugin")&&dlsym(image,"mavmoos_destroy_plugin"),"frozen ABI9 exports missing");
    PluginHost old;
    require(!old.load(argv[1],services,"example_heartbeat_observer")&&old.plugins().empty(),"source9 module accepted by SDK10 host");
    require(old.lastError().find("ABI")!=std::string::npos,"source9 rejected for a reason other than ABI");
    PluginHost current;
    require(current.load(argv[2],services,"example_heartbeat_observer"),"matching source10 module rejected: "+current.lastError());
    require(current.configureModules({{"example_heartbeat_observer",{{"sample_timeout","0.5"},{"publish_interval","0.1"}}}}),current.lastError());
    current.iterate();current.clear();dlclose(image);
    require(services.effects==0&&services.publications>0,"matching observer failed to publish or attempted effects");
    std::cout<<"{\"result\":\"PASS\",\"host_abi\":"<<plugin_abi
             <<",\"source9_rejected_for_abi\":true,\"source10_loaded_and_configured\":true,\"effects\":0}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<"FAIL source ABI consumer: "<<error.what()<<'\n';return 1;}
}

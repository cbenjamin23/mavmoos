#include "PluginHost.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

using namespace mavmoos::runtime;

#ifdef MAVMOOS_TEST_PLUGIN_FIXTURE
namespace {
class Fixture final:public Plugin {
public:
  explicit Fixture(Services& services):m_services(services) {}
  ~Fixture() override { m_services.publish("FIXTURE_DESTROY",1.0); }
  std::string name() const override {
#if defined(MAVMOOS_TEST_CAP_COLLISION)||defined(MAVMOOS_TEST_RESOURCE_COLLISION)||defined(MAVMOOS_TEST_REQUEST_COLLISION)||defined(MAVMOOS_TEST_INVALID_DESCRIPTOR)
    return "other";
#elif defined(MAVMOOS_TEST_COMPANION)
    return "companion";
#else
    return "fixture";
#endif
  }
  PluginDescriptor descriptor() const override {
    PluginDescriptor d;d.profile="fixture";d.streamed_parameter_bench_compatible=true;
#ifdef MAVMOOS_TEST_INVALID_DESCRIPTOR
    d.capabilities={{"bad\nname","FIXTURE_IN","value"}};
#else
    std::string cap="fixture",resource="fixture.resource",owner="FIXTURE_IN";
#ifdef MAVMOOS_TEST_CAP_COLLISION
    resource="other.resource";owner="OTHER_IN";
#elif defined(MAVMOOS_TEST_RESOURCE_COLLISION)
    cap="other";owner="OTHER_IN";
#elif defined(MAVMOOS_TEST_REQUEST_COLLISION)
    cap="other";resource="other.resource";
#elif defined(MAVMOOS_TEST_COMPANION)
    cap="companion";resource="companion.resource";owner="COMPANION_IN";
#endif
    d.capabilities={{cap,owner,"value"}};d.exclusive_resources={resource};d.request_owners={owner};
#endif
#ifdef MAVMOOS_TEST_COMPANION
    d.observation_streams={{33,50000},{245,250000}};
#else
    d.observation_streams={{33,100000}};
#endif
    if(m_omit_owner) d.request_owners.clear();
    return d;
  }
  std::vector<std::string> subscriptions() const override { return {"FIXTURE_IN","FIXTURE_IN"}; }
  void configure(const std::string& key,const std::string& value) override {
    if(key=="omit_request_owner") m_omit_owner=value=="true";
    m_services.publish("FIXTURE_CONFIG",value);
  }
  void onMail(const Mail&) override { m_services.publish("FIXTURE_MAIL",1.0); }
  void onMessage(const mavlink_message_t&,double) override { m_services.publish("FIXTURE_MESSAGE",1.0); }
  void onReset(const std::string& reason) override { m_services.publish("FIXTURE_RESET",reason); }
  void iterate() override { m_services.publish("FIXTURE_ITERATE",1.0); }
private:
  Services& m_services;
  bool m_omit_owner=false;
};
}
#ifndef MAVMOOS_TEST_MISSING_FACTORY
extern "C" Plugin* mavmoos_create_plugin(unsigned abi,Services* services)
{
#ifdef MAVMOOS_TEST_NULL_FACTORY
  (void)abi; (void)services;
  return nullptr;
#else
  return abi==plugin_abi && services ? new Fixture(*services) : nullptr;
#endif
}
#endif
extern "C" void mavmoos_destroy_plugin(Plugin* plugin) { delete plugin; }
#else
namespace {
void require(bool ok,const std::string& message) { if(!ok) throw std::runtime_error(message); }
class FakeServices final:public Services {
public:
  double now() const override { return 1; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key,const std::string& value) override { ++counts[key]; texts[key]=value; }
  void publish(const std::string& key,double) override { ++counts[key]; }
  bool sendCommand(const CommandRequest&) override { ++effects; return false; }
  bool acquireNavigation(const std::string&) override { ++effects; return false; }
  void revokeNavigation(const std::string&) override { ++effects; }
  bool navigation(const NavigationTarget&) override { ++effects; return false; }
  Vehicle state;
  std::unordered_map<std::string,unsigned> counts;
  std::unordered_map<std::string,std::string> texts;
  unsigned effects=0;
};
}
int main(int argc,char** argv)
{
  try {
    require(argc==9,"usage: plugin_host_tests valid missing null capability_collision resource_collision request_collision invalid_descriptor companion");
    FakeServices services;
    PluginHost host;
    require(!host.load("/mavmoos/nonexistent/plugin",services),"missing library accepted");
    require(!host.lastError().empty(),"missing library has no diagnostic");
    require(!host.load(argv[2],services),"missing factory accepted");
    require(host.lastError().find("create_plugin")!=std::string::npos,"wrong missing-factory diagnostic");
    require(!host.load(argv[3],services),"null factory accepted");
    require(host.lastError().find("ABI")!=std::string::npos,"null factory has no ABI diagnostic");
    require(host.plugins().empty(),"failed load retained plugin");
    require(host.load(argv[1],services),"valid plugin rejected: "+host.lastError());
    require(host.plugins().size()==1,"valid plugin absent");
    require(host.hasCapability("fixture")&&!host.hasCapability("other"),"capability lookup incorrect");
    require(host.catalog().find("\"profile\":\"fixture\"")!=std::string::npos,"catalog omits actual profile");
    require(host.streamedParameterBenchCompatible(),"compatible fixture rejected from bench composition");
    require(host.subscriptions()==std::vector<std::string>{"FIXTURE_IN"},"subscriptions not deduplicated");
    host.configure("value","hello");
    require(services.texts["FIXTURE_CONFIG"]=="hello","configuration not dispatched");
    Mail mail; mail.key="UNSUBSCRIBED"; host.dispatchMail(mail);
    require(services.counts["FIXTURE_MAIL"]==0,"unsubscribed mail dispatched");
    mail.key="FIXTURE_IN"; host.dispatchMail(mail);
    mavlink_message_t message{}; host.dispatchMessage(message,1.0);
    host.iterate();
    host.reset("test_restart");
    require(services.texts["FIXTURE_RESET"]=="test_restart","reset not dispatched");
    require(services.counts["FIXTURE_MAIL"]==1 && services.counts["FIXTURE_MESSAGE"]==1 &&
      services.counts["FIXTURE_ITERATE"]==1,"callback routing failed");
    require(!host.load(argv[1],services),"duplicate name accepted");
    require(host.plugins().size()==1 && services.counts["FIXTURE_DESTROY"]==1,
      "duplicate instance not destroyed while original remains alive");
    for(int i=4;i<=7;++i) {
      require(!host.load(argv[i],services),"conflicting or malformed descriptor accepted");
      require(host.plugins().size()==1,"rejected descriptor damaged original composition");
    }
    require(services.counts["FIXTURE_DESTROY"]==5,"rejected capability/resource/request/metadata instances leaked");
    host.iterate();
    require(services.counts["FIXTURE_ITERATE"]==2,"duplicate rejection broke original module");
    require(host.load(argv[8],services),"independent companion rejected: "+host.lastError());
    const auto streams=host.observationStreams();
    require(streams.size()==2&&streams[0].message_id==33&&streams[0].interval_us==50000&&
      streams[1].message_id==245,"observation stream union did not choose fastest requirement");
    require(host.catalog().find("\"name\":\"companion\"")!=std::string::npos,"catalog omits second provider");
    bool missing_owner_rejected=false;
    try { host.configure("omit_request_owner","true"); }
    catch(const std::runtime_error&) { missing_owner_rejected=true; }
    require(missing_owner_rejected,"advertised request capability omitted exclusive ownership");
    host.configure("omit_request_owner","false");
    host.clear();
    require(host.plugins().empty() && services.counts["FIXTURE_DESTROY"]==7,"module destruction before library close failed");
    host.clear();
    require(services.counts["FIXTURE_DESTROY"]==7,"clear is not idempotent");
    require(services.effects==0,"loader unexpectedly requested effects");
    std::cout<<"PASS plugin loader: ABI/export failures, duplicate names/capabilities/resources/requests, invalid metadata, catalog, stream union and teardown\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"FAIL plugin loader: "<<error.what()<<'\n';
    return 1;
  }
}
#endif

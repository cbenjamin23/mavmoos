#include "Runtime.h"
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
using namespace mavmoos::runtime;
#ifdef MAVMOOS_METADATA_FIXTURE
namespace {
class Fixture final:public Plugin {
public:
 explicit Fixture(Services& services):s(services){}
 std::string name() const override {
#ifdef MAVMOOS_METADATA_LEGACY
  return "legacy_metadata_fixture";
#else
  return "observation_metadata_fixture";
#endif
 }
 PluginDescriptor descriptor() const override {PluginDescriptor d;d.profile=name();return d;}
 void onMessage(const mavlink_message_t&,double at) override {s.publish("LEGACY_READ",at);}
#ifndef MAVMOOS_METADATA_LEGACY
 void onMessageWithMetadata(const mavlink_message_t&,const MessageTime& t) override {
  s.publish("META_EFFECTIVE",t.effective_at);s.publish("META_READ",t.read_at);s.publish("META_BASIS",t.age_basis);
 }
#endif
 void iterate() override {}
 Services& s;
};
}
extern "C" Plugin* mavmoos_create_plugin(unsigned abi,Services* s){return abi==plugin_abi&&s?new Fixture(*s):nullptr;}
extern "C" void mavmoos_destroy_plugin(Plugin* p){delete p;}
#else
namespace {
void require(bool b,const char* why){if(!b)throw std::runtime_error(why);}
struct Fake:Services {
 Vehicle v;std::map<std::string,double> numbers;std::map<std::string,std::string> strings;
 double now()const override{return 100;}
 const Vehicle& vehicle()const override{return v;}
 void publish(const std::string& k,double n)override{numbers[k]=n;}
 void publish(const std::string& k,const std::string& s)override{strings[k]=s;}
 bool sendCommand(const CommandRequest&)override{return false;}
 bool acquireNavigation(const std::string&)override{return false;}
 void revokeNavigation(const std::string&)override{}
 bool navigation(const NavigationTarget&)override{return false;}
};
}
int main(int argc,char** argv){try{
 require(argc==3,"usage: message_metadata_tests METADATA_MODULE LEGACY_MODULE");
 Fake f;PluginHost host;require(host.load(argv[1],f)&&host.load(argv[2],f),"load metadata fixtures");mavlink_message_t m{};
 host.dispatchMessage(m,MessageTime{99.75,100,"QUEUE_EMPTY_FLOOR"});
 require(f.numbers["META_EFFECTIVE"]==99.75&&f.numbers["META_READ"]==100&&f.numbers["LEGACY_READ"]==100,"metadata/read-time dispatch differs");
 require(f.strings["META_BASIS"]=="QUEUE_EMPTY_FLOOR","metadata basis lost");
 host.dispatchMessage(m,100.25);
 require(f.numbers["META_EFFECTIVE"]==100.25&&f.numbers["META_READ"]==100.25&&f.numbers["LEGACY_READ"]==100.25,"legacy direct dispatch no longer exact read acquisition");
 require(f.strings["META_BASIS"]=="READ_ACQUISITION","legacy direct dispatch basis changed");
 unsigned sends=0;std::map<std::string,double> n;std::map<std::string,std::string> text;
 Runtime runtime([&](const auto& k,const auto& s,double d,bool numeric){if(numeric)n[k]=d;else text[k]=s;},[&](const uint8_t*,std::size_t){++sends;return true;},false,false);
 require(runtime.load(argv[1])&&runtime.load(argv[2]),"runtime metadata modules");
 mavlink_msg_heartbeat_pack(1,1,&m,2,3,0,4,4);runtime.message(m,99.8,100,"QUEUE_EMPTY_FLOOR");
 require(n["META_EFFECTIVE"]==99.8&&n["META_READ"]==100&&n["LEGACY_READ"]==100,"runtime replaced floor with actual read");
 require(text["META_BASIS"]=="QUEUE_EMPTY_FLOOR"&&!runtime.navigationGranted()&&sends==0,"metadata granted effects or lost basis");
 const auto before=n;runtime.message(m,101,100.1,"QUEUE_EMPTY_FLOOR");require(n==before,"reversed metadata dispatched");
 std::cout<<"PASS observation floor/read metadata, legacy read correlation, Runtime integration and invalid-time rejection; no I/O\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
#endif

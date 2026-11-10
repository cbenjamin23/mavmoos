#include "PluginHost.h"
#include <cmath>
#include <dlfcn.h>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>

using namespace mavmoos::runtime;
namespace {
void require(bool ok,const std::string& message) { if(!ok) throw std::runtime_error(message); }
void near(double got,double expected,double tolerance,const std::string& message)
{ require(std::abs(got-expected)<=tolerance,message+": "+std::to_string(got)); }
class FakeServices final:public Services {
public:
  double now() const override { return time; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key,const std::string& value) override { ++counts[key]; texts[key]=value; }
  void publish(const std::string& key,double value) override { ++counts[key]; numbers[key]=value; }
  bool sendCommand(const CommandRequest&) override { ++effects; return false; }
  bool acquireNavigation(const std::string&) override { ++effects; return false; }
  void revokeNavigation(const std::string&) override { ++effects; }
  bool navigation(const NavigationTarget&) override { ++effects; return false; }
  double time=10;
  Vehicle state;
  std::unordered_map<std::string,unsigned> counts;
  std::unordered_map<std::string,std::string> texts;
  std::unordered_map<std::string,double> numbers;
  unsigned effects=0;
};
}
int main(int argc,char** argv)
{
  try {
    require(argc==2,"usage: telemetry_plugin_tests module_path");
    FakeServices services;
    PluginHost host;
    require(host.load(argv[1],services),"load telemetry failed: "+host.lastError());
    require(host.plugins().size()==1 && host.plugins()[0]->name()=="telemetry","wrong module name");
    void* library=dlopen(argv[1],RTLD_NOW|RTLD_LOCAL);
    require(library,"dlopen failed for ABI test");
    auto create=reinterpret_cast<CreatePlugin>(dlsym(library,"mavmoos_create_plugin"));
    require(create && !create(plugin_abi+1,&services) && !create(plugin_abi,nullptr),"factory accepted incompatible ABI/context");
    dlclose(library);
    host.iterate();
    require(services.numbers["NAV_VALID"]==0 && services.counts["NAV_LAT"]==0,"absent values became zero observations");

    auto& v=services.state;
    v.position_valid=true; v.position_at=10; v.home_valid=true;
    v.latitude=42; v.longitude=-71; v.relative_height=8; v.msl_height=122;
    v.north_velocity=3; v.east_velocity=4; v.down_velocity=-0.5; v.heading=359;
    host.configure("lat_origin","42"); host.configure("long_origin","-71");
    host.iterate();
    near(services.numbers["NAV_X"],0,0.0001,"origin X");
    near(services.numbers["NAV_Y"],0,0.0001,"origin Y");
    near(services.numbers["NAV_SPEED"],5,1e-9,"ground speed");
    near(services.numbers["NAV_COURSE"],53.130102354,1e-6,"course");
    near(services.numbers["NAV_HEADING"],359,0,"yaw must stay separate from course");
    near(services.numbers["NAV_ALTITUDE"],8,0,"home-relative height");
    near(services.numbers["NAV_ALTITUDE_MSL"],122,0,"MSL height");
    near(services.numbers["NAV_VERTICAL_SPEED"],0.5,0,"vertical sign");
    require(services.numbers["NAV_VALID"]==1 && services.numbers["NAV_XY_VALID"]==1,"fresh sample invalid");
    const auto sample_count=services.counts["NAV_X"];
    services.time=10.2; host.iterate();
    require(services.counts["NAV_X"]==sample_count,"cached NAV republished with new apparent age");
    services.time=10.6; v.heartbeat_valid=true; v.heartbeat_at=10.6; host.iterate();
    require(services.numbers["NAV_VALID"]==0 && services.counts["NAV_X"]==sample_count,"heartbeat refreshed stale position");
    require(services.texts["MAVMOOS_NAV_STATUS"].find("received_at=10,")!=std::string::npos,"diagnostics changed original receipt");

    services.time=10.7; v.position_at=10.7; v.latitude+=0.0001; v.longitude+=0.0001;
    host.iterate();
    require(services.numbers["NAV_Y"]>10 && services.numbers["NAV_Y"]<12 &&
      services.numbers["NAV_X"]>7 && services.numbers["NAV_X"]<10,"MOOS local-grid axes/distance incorrect");
    require(services.counts["NAV_X"]==sample_count+1,"new sample not published");
    host.reset("connection_changed"); host.iterate();
    require(services.numbers["NAV_VALID"]==0 && services.counts["NAV_X"]==sample_count+1,
      "reset republished a pre-boundary position sample");
    const auto heading_count=services.counts["NAV_HEADING"];
    const auto course_count=services.counts["NAV_COURSE"];
    const auto altitude_count=services.counts["NAV_ALTITUDE"];
    services.time=10.8; v.position_at=10.8; v.heading=std::numeric_limits<double>::quiet_NaN();
    v.north_velocity=0; v.east_velocity=0; v.home_valid=false; host.iterate();
    require(services.numbers["NAV_VALID"]==1 && services.numbers["NAV_HEADING_VALID"]==0 &&
      services.numbers["NAV_COURSE_VALID"]==0 && services.numbers["NAV_ALTITUDE_VALID"]==0,
      "per-quantity validity not independent");
    require(services.counts["NAV_HEADING"]==heading_count && services.counts["NAV_COURSE"]==course_count &&
      services.counts["NAV_ALTITUDE"]==altitude_count,"unknown quantities published as valid zeros");
    services.time=10.9; v.position_at=10.9; v.latitude=std::numeric_limits<double>::quiet_NaN(); host.iterate();
    require(services.numbers["NAV_VALID"]==0,"nonfinite latitude accepted");
    v.latitude=0; v.longitude=0; v.position_at=11.1; host.iterate();
    require(services.numbers["NAV_VALID"]==0,"future acquisition accepted");
    services.time=11.1; host.configure("lat_origin","0"); host.configure("long_origin","0"); host.iterate();
    require(services.numbers["NAV_VALID"]==1 && services.numbers["NAV_LAT"]==0 && services.numbers["NAV_LONG"]==0,
      "valid zero coordinates treated as absent");
    bool invalid=false;
    try { host.configure("lat_origin","nan"); } catch(const std::exception&) { invalid=true; }
    require(invalid,"nonfinite origin accepted");
    invalid=false;
    try { host.configure("nav_position_timeout","0.5seconds"); } catch(const std::exception&) { invalid=true; }
    require(invalid,"partial numeric config token accepted");
    services.time=11.2;v.position_at=11.1;v.position_read_at=11.2;v.position_time_basis="QUEUE_EMPTY_FLOOR";
    v.latitude=.0001;host.iterate();const auto floor_count=services.counts["NAV_LAT"];
    services.time=11.3;v.position_read_at=11.3;v.latitude=.0002;host.iterate();
    require(services.counts["NAV_LAT"]==floor_count+1,"new packet in same queue epoch suppressed");
    require(services.texts["MAVMOOS_NAV_STATUS"].find("age_basis=QUEUE_EMPTY_FLOOR,age_is_upper_bound=true")!=std::string::npos,"NAV conservative age metadata missing");
    services.time=11.7;host.iterate();require(services.numbers["NAV_VALID"]==0,"read timestamp renewed conservative position age");

    services.time=11.8;v.position_at=v.position_read_at=11.8;v.home_valid=false;v.relative_height=9;
    host.iterate();
    require(services.numbers["NAV_ALTITUDE_VALID"]==0 && services.counts["NAV_ALTITUDE"]==altitude_count,
      "position without home published an altitude");
    services.time=11.85;v.home_valid=true;v.home_at=11.85;host.iterate();
    require(services.numbers["NAV_ALTITUDE_VALID"]==0 && services.counts["NAV_ALTITUDE"]==altitude_count,
      "new home made a previously unpublished altitude valid");
    services.time=11.9;v.position_at=v.position_read_at=11.9;host.iterate();
    require(services.numbers["NAV_ALTITUDE_VALID"]==1 && services.counts["NAV_ALTITUDE"]==altitude_count+1,
      "first position after home did not publish a valid altitude");
    near(services.numbers["NAV_ALTITUDE"],9,0,"first altitude after home");
    services.time=11.95;v.home_at=11.95;host.iterate();
    require(services.numbers["NAV_ALTITUDE_VALID"]==1 && services.counts["NAV_ALTITUDE"]==altitude_count+1,
      "repeat of the same home invalidated or renewed the published altitude");
    services.time=11.96;v.home_at=11.96;v.home_msl+=1;host.iterate();
    require(services.numbers["NAV_ALTITUDE_VALID"]==0 && services.counts["NAV_ALTITUDE"]==altitude_count+1,
      "changed home left an older relative altitude valid");
    require(services.effects==0,"telemetry requested control effects");
    std::cout<<"PASS telemetry plugin: ABI, origins/axes, units/references, independent validity, freshness, zero/NaN/future data\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"FAIL telemetry plugin: "<<error.what()<<'\n';
    return 1;
  }
}

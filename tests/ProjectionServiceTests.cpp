#include "Runtime.h"
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
using namespace mavmoos::runtime;
void check(bool value,const char* reason) {if(!value)throw std::runtime_error(reason);}
int main(int argc,char** argv) {
  try {
    check(argc==2,"usage: projection_service_tests TELEMETRY_MODULE");
    std::map<std::string,double> numeric;unsigned publications=0,sends=0;
    Runtime host([&](const auto& key,const auto&,double value,bool number){++publications;if(number)numeric[key]=value;},
      [&](const uint8_t*,std::size_t){++sends;return true;},false,false);
    host.checkpoint(100);double x=123,y=456;
    check(!host.projectGlobal(42,-71,x,y)&&x==123&&y==456,"missing projector mutated output");
    check(host.load(argv[1]),"load telemetry");check(!host.projectGlobal(42,-71,x,y),"unconfigured origin projected");
    host.configure("lat_origin","42");check(!host.projectGlobal(42,-71,x,y),"partial origin projected");host.configure("long_origin","-71");
    const auto before=publications;check(host.projectGlobal(42,-71,x,y)&&std::hypot(x,y)<1e-5,"origin is not MOOS XY zero");
    check(host.projectGlobal(42.0001,-70.9999,x,y)&&x>0&&y>0,"east/north projection swapped");
    check(publications==before&&sends==0&&!host.navigationGranted(),"projection produced effects or authority");
    mavlink_message_t message{};mavlink_status_t codec{};
    mavlink_msg_global_position_int_pack_status(1,1,&codec,&message,0,420001000,-709999000,15000,5000,0,0,0,9000);host.message(message,100.1);host.tick(100.1);
    check(std::abs(numeric.at("NAV_X")-x)<1e-5&&std::abs(numeric.at("NAV_Y")-y)<1e-5,"target projection differs from telemetry frame");
    const double original=host.vehicle().position_at;host.tick(101);
    check(host.projectGlobal(42.0001,-70.9999,x,y)&&host.vehicle().position_at==original&&!host.vehicle().position_valid,"projection renewed stale observation");
    check(!host.projectGlobal(NAN,-71,x,y)&&!host.projectGlobal(85,-71,x,y),"invalid projection coordinates");
    check(sends==0,"read-only projection sent MAVLink");
    std::cout<<"PASS projection ownership, initialization, frame consistency and observational isolation\n";return 0;
  }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}

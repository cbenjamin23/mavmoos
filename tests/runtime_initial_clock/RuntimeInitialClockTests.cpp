/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: RuntimeInitialClockTests.cpp                    */
/*    DATE: November 4, 2026                                */
/************************************************************/
// Actual Runtime/modules, synchronous fake I/O; no socket or simulator.
#include "Runtime.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace mavmoos::runtime;
namespace {
void require(bool value,const char* why){if(!value)throw std::runtime_error(why);}
void heartbeat(Runtime& runtime,double at)
{
  mavlink_message_t message{};mavlink_status_t codec{};
  mavlink_msg_heartbeat_pack_status(1,1,&codec,&message,2,3,0,4,4);
  runtime.message(message,at);
}
}
int main(int argc,char** argv)
{
  try {
    require(argc==3,"usage: runtime_initial_clock_tests TELEMETRY baseline|current");
    const double invalid[]={-1,std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};
    if(std::string(argv[2])=="baseline") {
      unsigned admitted=0,recovered=0;
      for(double initial:invalid) {
        double clock=initial;unsigned sends=0;
        Runtime runtime([](const auto&,const auto&,double,bool){},[&](const uint8_t*,std::size_t){++sends;return true;},true,false,[&]{return clock;});
        require(runtime.status().find("configuration_valid=true")!=std::string::npos,"legacy startup no longer config-valid");
        require(runtime.load(argv[1]),"legacy invalid clock did not admit module");
        ++admitted;
        clock=100;heartbeat(runtime,clock);runtime.tick(clock);
        if(sends>0)++recovered;
      }
      require(admitted==4&&recovered==3,"legacy invalid-initial-clock behavior changed");
      std::cout<<"REPRODUCED 4 invalid-initial-clock cases admitted modules; 3 later sent after a good sample, positive infinity remained unusable; fake I/O only\n";
      return 0;
    }
    unsigned groups=0;
    for(double initial:invalid)for(bool simulation:{false,true})for(bool shadow:{false,true}) {
      double clock=initial;unsigned sends=0;
      Runtime runtime([](const auto&,const auto&,double,bool){},[&](const uint8_t*,std::size_t){++sends;return true;},simulation,shadow,[&]{return clock;});
      require(std::isfinite(runtime.now())&&runtime.now()>=0,"invalid startup sample escaped through Services.now");
      require(runtime.status().find("configuration_valid=false")!=std::string::npos&&
              runtime.status().find("reason=invalid_initial_clock")!=std::string::npos,"invalid clock did not fail startup visibly");
      require(!runtime.load(argv[1]),"invalid startup clock admitted a module");
      bool rejected=false;try{runtime.configure("lat_origin","42");}catch(const std::runtime_error&){rejected=true;}
      require(rejected,"configuration revived invalid startup");
      clock=100;heartbeat(runtime,clock);runtime.tick(clock);
      require(sends==0&&!runtime.navigationGranted()&&!runtime.acquireNavigation("after-clock-fault"),"later good time revived invalid startup effects");
      require(runtime.status().find("configuration_valid=false")!=std::string::npos,"startup failure was not latched");++groups;
    }
    for(double initial:{0.,100.}) {
      double clock=initial;unsigned sends=0;
      Runtime runtime([](const auto&,const auto&,double,bool){},[&](const uint8_t*,std::size_t){++sends;return true;},true,false,[&]{return clock;});
      require(runtime.load(argv[1]),"valid initial time denied module load");
      clock=100;heartbeat(runtime,clock);runtime.tick(clock);
      require(sends>0&&!runtime.navigationGranted(),"valid startup changed housekeeping or granted navigation");++groups;
    }
    {
      unsigned sends=0;Runtime runtime([](const auto&,const auto&,double,bool){},[&](const uint8_t*,std::size_t){++sends;return true;},true,false);
      require(runtime.load(argv[1]),"no-clock fake host load changed");heartbeat(runtime,100);runtime.tick(100);
      require(sends>0&&!runtime.navigationGranted(),"checkpoint-driven fake clock changed");++groups;
    }
    std::cout<<"PASS "<<groups<<" initial-clock groups; actual Runtime/module, fake I/O only\n";return 0;
  }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}

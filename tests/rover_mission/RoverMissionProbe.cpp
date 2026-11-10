// Test-only MOOS operator. The supervisor owns the independent FC receiver.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <cerrno>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

namespace {
double monotonic()
{
#ifdef __APPLE__
  static const auto timebase=[] {
    mach_timebase_info_data_t info{};
    mach_timebase_info(&info);
    return info;
  }();
  return static_cast<double>(mach_absolute_time())*timebase.numer/timebase.denom/1e9;
#else
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
#endif
}

std::string quote(const std::string& text)
{
  constexpr char hex[]="0123456789abcdef";
  std::string out="\"";
  for(const unsigned char c:text) {
    if(c=='"' || c=='\\') out+='\\';
    if(c<32 || c>=127) {
      out+="\\u00";
      out+=hex[c>>4];out+=hex[c&15];
    } else out+=char(c);
  }
  return out+'"';
}

void require(bool ok,const char* why)
{ if(!ok) throw std::runtime_error(why); }
}

int main(int argc,char** argv)
{
  try {
    const bool autonomy=argc==4 && std::string(argv[3])=="--autonomy";
    require(argc==3 || autonomy,"expected MOOS port, trace path and optional --autonomy");
    const int port=std::stoi(argv[1]);
    require(port>0 && port<=65535,"invalid MOOS port");
    std::ofstream trace(argv[2]);
    require(bool(trace),"unable to open trace");
    trace<<std::setprecision(17);
    CMOOSCommClient client;
    client.SetQuiet(true);
    require(client.Run("127.0.0.1",port,"rover_mission_operator",30),"MOOS client start failed");
    require(fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK)==0,"cannot set nonblocking operator input");
    constexpr const char* subscriptions[]={
      "MAVMOOS_RUNTIME_STATUS","MAVMOOS_HOME_POSITION","MAVMOOS_MISSION_RESULT",
      "MAVMOOS_MISSION_SNAPSHOT","MAVMOOS_STORED_MISSION_PROOF","MAVMOOS_MISSION_EXECUTION_STATUS",
      "MAVMOOS_MISSION_EXECUTION_RESULT","MAVMOOS_MISSION_EXECUTION_PROGRESS",
      "MAVMOOS_MISSION_EXECUTION_LEASE","MAVMOOS_AUTO_RECOVERY_STATUS","MAVMOOS_AUTO_RECOVERY_RESULT",
      "MAVMOOS_COMMAND_RESULT","MAVMOOS_CONTROL_RESULT","MAVMOOS_HELM_STATUS","MAVMOOS_TX_RESULT","DESIRED_HEADING","DESIRED_SPEED",
      "DESIRED_ALTITUDE"
    };
    std::set<std::string> publish={
      "MAVMOOS_MISSION_REQUEST","MAVMOOS_MISSION_CONTROL","MAVMOOS_AUTO_RECOVERY_REQUEST",
      "MAVMOOS_COMMAND","MAVMOOS_CONTROL_REQUEST","MAVMOOS_NAV_CONTROL","DEPLOY","MOOS_MANUAL_OVERRIDE"
    };
    const std::set<std::string> missionEvents={
      "SURVEY_INDEX","SURVEY_DONE","SURVEY_RETURNING","CROSSING_ACTIVE","SITL_ROUTE_FINISHED",
      "MISSION_EVALUATED","BHV_WARNING","BHV_ERROR","IVPHELM_STATE"
    };
    if(autonomy) publish.insert({"SURVEY_UPDATE","STRESS_RESUME","NODE_REPORT","SITL_INDEPENDENT_DONE"});
    bool connected=false;
    std::string input;
    for(;;) {
      if(client.IsConnected()) {
        if(!connected) {
          for(auto key:subscriptions) require(client.Register(key,0),"register failed");
          if(autonomy) for(const auto& key:missionEvents) require(client.Register(key,0),"register mission event failed");
          trace<<"{\"kind\":\"connected\",\"at\":"<<monotonic()<<"}\n"<<std::flush;
          connected=true;
        }
        MOOSMSG_LIST mail;
        client.Fetch(mail);
        for(const auto& item:mail) {
          if(autonomy && missionEvents.count(item.GetKey())) {
            trace<<"{\"kind\":\"event\",\"at\":"<<monotonic()<<",\"key\":"<<quote(item.GetKey())
                 <<",\"source\":"<<quote(item.GetSource())<<",\"value\":";
            if(item.IsString()) trace<<quote(item.GetString());
            else if(item.IsDouble() && std::isfinite(item.GetDouble())) trace<<item.GetDouble();
            else trace<<"null";
            trace<<"}\n";
          }
          if(item.IsString() && item.GetSource()=="iMAVLink")
            trace<<"{\"kind\":\"mail\",\"at\":"<<monotonic()<<",\"key\":"<<quote(item.GetKey())
                 <<",\"value\":"<<quote(item.GetString())<<"}\n";
          if(item.IsDouble() && item.GetSource()=="pHelmIvP" && std::isfinite(item.GetDouble()))
            trace<<"{\"kind\":\"helm\",\"at\":"<<monotonic()<<",\"key\":"<<quote(item.GetKey())
                 <<",\"value\":"<<item.GetDouble()<<"}\n";
        }
        trace.flush();
        char buffer[4096];
        const auto size=read(STDIN_FILENO,buffer,sizeof(buffer));
        if(size==0) break;
        if(size<0) require(errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR,"operator read failed");
        if(size>0) input.append(buffer,static_cast<std::size_t>(size));
        for(auto end=input.find('\n');end!=std::string::npos;end=input.find('\n')) {
          const auto row=input.substr(0,end);
          input.erase(0,end+1);
          const auto tab=row.find('\t');
          require(tab!=std::string::npos && row.size()<=2048,"malformed operator command");
          const auto key=row.substr(0,tab),value=row.substr(tab+1);
          require(publish.count(key) && client.Notify(key,value),"operator send failed");
          trace<<"{\"kind\":\"request\",\"at\":"<<monotonic()<<",\"key\":"<<quote(key)
               <<",\"value\":"<<quote(value)<<"}\n"<<std::flush;
        }
        require(input.size()<=2048,"operator input exceeds bound");
      } else require(!connected,"lost stock MOOS community");
      std::this_thread::sleep_for(std::chrono::milliseconds(12));
    }
    client.Close();
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';
    return 1;
  }
}

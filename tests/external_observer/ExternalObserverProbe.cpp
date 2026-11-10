// Read-only MOOS subscriber. No Notify, MAVLink socket or authority request.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>
namespace {
double now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
std::string quote(const std::string& text) {
  static constexpr char hex[]="0123456789abcdef";std::string result="\"";
  for(unsigned char c:text) {
    if(c=='"'||c=='\\') { result+='\\';result+=char(c); }
    else if(c<32) { result+="\\u00";result+=hex[c>>4];result+=hex[c&15]; }
    else result+=char(c);
  }
  return result+'"';
}
}
int main(int argc,char** argv) {
  CMOOSCommClient client;
  try {
    if(argc!=3)throw std::runtime_error("usage: external_observer_probe MOOS_PORT DURATION8..30");
    std::size_t used=0;const int port=std::stoi(argv[1],&used);
    if(used!=std::string(argv[1]).size()||port<1||port>65535)throw std::runtime_error("invalid MOOS port");
    const double duration=std::stod(argv[2],&used);
    if(used!=std::string(argv[2]).size()||!std::isfinite(duration)||duration<8||duration>30)throw std::runtime_error("invalid duration");
    const auto started=now();bool registered=false;unsigned count=0;
    client.SetQuiet(true);
    if(!client.Run("127.0.0.1",port,"external_observer_probe",50))throw std::runtime_error("MOOS client startup failed");
    while(now()-started<duration) {
      if(client.IsConnected()&&!registered) {
        for(const auto* key:{"EXAMPLE_HEARTBEAT_OBSERVATION","MAVMOOS_CAPABILITIES","MAVMOOS_STATUS","MAVMOOS_RUNTIME_STATUS","APPCAST"})client.Register(key,0);
        registered=true;
      }
      if(!client.IsConnected())registered=false;
      MOOSMSG_LIST mail;client.Fetch(mail);
      for(const auto& message:mail) {
        if(message.GetSource()!="iMAVLink"||!message.IsString())continue;
        if(++count>10000)throw std::runtime_error("MOOS observation bound exceeded");
        std::cout<<"{\"mono\":"<<std::setprecision(17)<<now()<<",\"moos_time\":"<<message.GetTime()
                 <<",\"key\":"<<quote(message.GetKey())<<",\"source\":"<<quote(message.GetSource())
                 <<",\"value\":"<<quote(message.GetString())<<"}\n";
      }
      std::cout.flush();std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    client.Close();if(!count)throw std::runtime_error("no iMAVLink observations");return 0;
  } catch(const std::exception& error) {
    client.Close();std::cerr<<"external observer probe: "<<error.what()<<'\n';return 1;
  }
}

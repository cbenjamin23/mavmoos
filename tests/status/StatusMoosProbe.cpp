// Read-only MOOS subscriber. No Notify, FC socket, request or simulator launcher.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
namespace {
double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
std::string jsonString(const std::string& input)
{
  static constexpr char hex[]="0123456789abcdef";std::string out="\"";
  for(unsigned char c:input){if(c=='"'||c=='\\'){out+='\\';out+=char(c);}else if(c<32){out+="\\u00";out+=hex[c>>4];out+=hex[c&15];}else out+=char(c);}
  return out+'"';
}
}
int main(int argc,char** argv)
{
  CMOOSCommClient client;
  try {
    if(argc!=3)throw std::runtime_error("usage: status_moos_probe DB_PORT DURATION_SECONDS (JSONL stdout)");
    std::size_t used=0;const int port=std::stoi(argv[1],&used);
    if(used!=std::string(argv[1]).size()||port<1||port>65535)throw std::runtime_error("invalid MOOS port");
    const double duration=std::stod(argv[2],&used);
    if(used!=std::string(argv[2]).size()||!std::isfinite(duration)||duration<5||duration>60)throw std::runtime_error("duration outside 5..60 seconds");
    const double start=now();bool registered=false;unsigned records=0;
    client.SetQuiet(true);
    if(!client.Run("127.0.0.1",port,"mavmoos_system_status_probe",50))throw std::runtime_error("MOOS startup failed");
    while(now()-start<duration) {
      if(client.IsConnected()&&!registered){for(const auto* key:{"MAVMOOS_SYS_STATUS","MAVMOOS_BATTERY_STATUS","MAVMOOS_GPS_STATUS","MAVMOOS_FC_TEXT","MAVMOOS_SYSTEM_STATUS","MAVMOOS_RUNTIME_STATUS","MAVMOOS_CAPABILITIES"})client.Register(key,0);registered=true;}
      if(!client.IsConnected())registered=false;
      MOOSMSG_LIST mail;client.Fetch(mail);
      for(const auto& message:mail) {
        if(message.GetSource()!="iMAVLink"||!message.IsString())continue;
        if(++records>10000)throw std::runtime_error("observation trace record bound exceeded");
        std::cout<<"{\"elapsed\":"<<std::fixed<<std::setprecision(6)<<now()-start<<",\"key\":"<<jsonString(message.GetKey())
                 <<",\"source\":"<<jsonString(message.GetSource())<<",\"value\":"<<jsonString(message.GetString())<<"}\n";
      }
      std::cout.flush();std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    client.Close();if(!records)throw std::runtime_error("no iMAVLink observation records");return 0;
  } catch(const std::exception& error){std::cerr<<"status probe: "<<error.what()<<'\n';client.Close();return 1;}
}

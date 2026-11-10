/*****************************************************************/
/* NAME: MAVMOOS contributors                                     */
/* ORGN: MAVMOOS                                                  */
/* FILE: RcIoMoosProbe.cpp                                         */
/* DATE: November 4, 2026                                          */
/*****************************************************************/
// Read-only subscriber. A local marker labels evidence windows; no Notify,
// MAVLink socket, command, simulator launcher or RC input belongs here.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
namespace {
double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
std::string quoted(const std::string& value)
{
  static constexpr char hex[]="0123456789abcdef";std::string result="\"";
  for(unsigned char c:value){if(c=='"'||c=='\\'){result+='\\';result+=char(c);}else if(c<32){result+="\\u00";result+=hex[c>>4];result+=hex[c&15];}else result+=char(c);}
  return result+'"';
}
std::string marker(const std::filesystem::path& path)
{
  std::ifstream stream(path);std::string value;std::getline(stream,value);
  if(value.size()>32)throw std::runtime_error("phase marker exceeds bound");return value;
}
}
int main(int argc,char** argv)
{
  CMOOSCommClient client;
  try {
    if(argc!=4)throw std::runtime_error("usage: rc_io_moos_probe DB_PORT DURATION_SECONDS CONTROL_DIRECTORY (JSONL stdout)");
    std::size_t used=0;const int port=std::stoi(argv[1],&used);
    if(used!=std::string(argv[1]).size()||port<1||port>65535)throw std::runtime_error("invalid MOOS port");
    const double duration=std::stod(argv[2],&used);
    if(used!=std::string(argv[2]).size()||!std::isfinite(duration)||duration<5||duration>220)throw std::runtime_error("duration outside5..220 seconds");
    const std::filesystem::path control=argv[3];const double start=now();bool registered=false;unsigned records=0;std::string previous;
    client.SetQuiet(true);
    if(!client.Run("127.0.0.1",port,"mavmoos_rc_io_probe",50))throw std::runtime_error("MOOS startup failed");
    while(now()-start<duration&&!std::filesystem::exists(control/"observer.stop")) {
      const auto phase=marker(control/"rc.phase");
      if(!phase.empty()&&phase!=previous){std::cout<<"{\"type\":\"phase\",\"elapsed\":"<<std::fixed<<std::setprecision(6)<<now()-start<<",\"phase\":"<<quoted(phase)<<"}\n";previous=phase;}
      if(client.IsConnected()&&!registered){for(const auto* key:{"MAVMOOS_RC_INPUT","MAVMOOS_SERVO_OUTPUT","MAVMOOS_RC_IO_STATUS","MAVMOOS_RUNTIME_STATUS","MAVMOOS_CAPABILITIES"})client.Register(key,0);registered=true;}
      if(!client.IsConnected())registered=false;
      MOOSMSG_LIST mail;client.Fetch(mail);
      for(const auto& message:mail) {
        if(message.GetSource()!="iMAVLink"||!message.IsString())continue;
        if(++records>30000)throw std::runtime_error("RC observation record bound exceeded");
        std::cout<<"{\"type\":\"observation\",\"elapsed\":"<<std::fixed<<std::setprecision(6)<<now()-start<<",\"phase\":"<<quoted(previous)
                 <<",\"key\":"<<quoted(message.GetKey())<<",\"source\":"<<quoted(message.GetSource())<<",\"value\":"<<quoted(message.GetString())<<"}\n";
      }
      std::cout.flush();std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout<<"{\"type\":\"end\",\"elapsed\":"<<std::fixed<<std::setprecision(6)<<now()-start<<",\"records\":"<<records<<"}\n";
    client.Close();if(!records)throw std::runtime_error("no iMAVLink observations");return 0;
  }catch(const std::exception& error){std::cerr<<"RC observation probe: "<<error.what()<<'\n';client.Close();return 1;}
}

// Operator and stock helm mail witness for the owned ArduSub test community.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace {
double now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void need(bool condition,const char* why)
{
  if(!condition)throw std::runtime_error(why);
}

void report(const std::string& kind,const std::string& key,const std::string& source,
            double sent,const std::string& value)
{
  need(value.find_first_of("\t\r\n")==std::string::npos,"probe row is not one value");
  std::cout<<kind<<'\t'<<key<<'\t'<<now()<<'\t'<<sent<<'\t'<<source<<'\t'<<value<<'\n';
}

bool allowed(const std::string& key,const std::string& value,bool quadplane)
{
  if(key=="AUTONOMY_GRADE_READY") return value=="true";
  if(quadplane)
    return (key=="DEPLOY" && (value=="true" || value=="false")) ||
      (key=="AUTONOMY_ROUTE" && value.size()>7 && value.size()<120 &&
       value.compare(0,7,"points=")==0 && value.find_first_not_of("0123456789.,:-",7)==std::string::npos) ||
      (key=="MAVMOOS_NAV_CONTROL" && (value=="acquire" || value=="release")) ||
      (key=="MAVMOOS_QUADPLANE_REQUEST" && value.find("schema=1,id=")==0) ||
      (key=="MAVMOOS_MISSION_REQUEST" && value.find("id=quad-plan,action=upload,")==0) ||
      (key=="MAVMOOS_AUTO_RECOVERY_REQUEST" && value.find("schema=1,id=")==0 &&
       value.find(",action=handback,session=")!=std::string::npos);
  return (key=="DEPLOY" && (value=="true" || value=="false")) ||
    (key=="MAVMOOS_NAV_CONTROL" && (value=="acquire" || value=="release")) ||
    (key=="MAVMOOS_SUB_HANDBACK" && (value=="hold" || value=="surface" ||
     value=="schema=1,id=operator-hold,action=hold" || value=="schema=1,id=interrupted-surface,action=surface"));
}
}

int main(int argc,char** argv)
{
  CMOOSCommClient client;
  try {
    const bool quadplane=argc==3 && std::string(argv[2])=="quadplane";
    need(argc==2 || quadplane,"usage: ardusub_moos_probe DB_PORT [quadplane]");
    const std::size_t max_request=quadplane?512:128;
    std::size_t used=0;
    const std::string input=argv[1];
    const auto port=std::stoi(input,&used);
    need(used==input.size() && port>0 && port<=65535,"invalid private DB port");
    need(fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK)==0,"probe stdin setup failed");
    client.SetQuiet(true);
    need(client.Run("127.0.0.1",port,"ardusub_operator_probe",40),"private MOOS connection failed");
    std::cout<<std::setprecision(17);
    const auto deadline=now()+(quadplane?320:150);
    bool registered=false;
    std::string pending;
    while(now()<deadline) {
      if(registered)need(client.IsConnected(),"private MOOS connection lost");
      if(!registered && client.IsConnected()) {
        if(quadplane)
          for(const auto* key:{"MAVMOOS_STATUS","MAVMOOS_QUADPLANE_RESULT","MAVMOOS_MISSION_RESULT","MAVMOOS_STORED_MISSION_PROOF",
                              "MAVMOOS_MISSION_EXECUTION_LEASE","MAVMOOS_AUTO_RECOVERY_STATUS","MAVMOOS_AUTO_RECOVERY_RESULT",
                              "MAVMOOS_STREAM_REQUEST_RESULT","MAVMOOS_TX_RESULT","DESIRED_ALTITUDE"})
            need(client.Register(key,0),"QuadPlane probe registration failed");
        for(const auto* key:{"MAVMOOS_CAPABILITIES","MAVMOOS_RUNTIME_STATUS","MAVMOOS_HELM_STATUS",
                            "MAVMOOS_SUB_STATUS","MAVMOOS_SUB_HANDBACK_RESULT","NAV_DEPTH","NAV_DEPTH_VALID",
                            "DESIRED_HEADING","DESIRED_SPEED","DESIRED_DEPTH","IVPHELM_ALLSTOP","IVPHELM_STATE","AUTONOMY_STAGE"})
          need(client.Register(key,0),"ArduSub probe registration failed");
        registered=true;
      }
      if(registered) {
        char buffer[512];
        const auto size=read(STDIN_FILENO,buffer,sizeof(buffer));
        if(size==0) {client.Close();return 0;}
        if(size<0)need(errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR,"probe pipe failed");
        else pending.append(buffer,static_cast<std::size_t>(size));
        for(auto end=pending.find('\n');end!=std::string::npos;end=pending.find('\n')) {
          const auto request=pending.substr(0,end);
          pending.erase(0,end+1);
          const auto tab=request.find('\t');
          need(tab!=std::string::npos && end<=max_request,"invalid Sub probe request");
          const auto key=request.substr(0,tab),value=request.substr(tab+1);
          need(allowed(key,value,quadplane),"request outside the owned fixture policy");
          need(client.Notify(key,value),"probe publication failed");
          report("request",key,"ardusub_operator_probe",0,value);
        }
        need(pending.size()<=max_request,"Sub probe request exceeds bound");
        MOOSMSG_LIST mail;
        client.Fetch(mail);
        for(const auto& message:mail) {
          if(message.GetSource()!="iMAVLink" && message.GetSource()!="pHelmIvP")continue;
          if(message.IsString())report("string",message.GetKey(),message.GetSource(),message.GetTime(),message.GetString());
          else if(message.IsDouble())report("number",message.GetKey(),message.GetSource(),message.GetTime(),
                                            std::to_string(message.GetDouble()));
        }
      }
      std::cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("Sub probe deadline elapsed");
  } catch(const std::exception& error) {
    client.Close();
    std::cerr<<error.what()<<'\n';
    return 1;
  }
}

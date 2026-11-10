// Exact same-host ExternalNav producer relay for the disarmed Copter fixture.
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
constexpr char request_key[]="MAVMOOS_EXTERNAL_NAV_REQUEST";
constexpr char input_key[]="MAVMOOS_EXTERNAL_NAV_INPUT";
constexpr char source[]="external_navigation_fixture";

double now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void need(bool condition,const char* reason)
{
  if(!condition) throw std::runtime_error(reason);
}

void record(const char* kind,const std::string& key,const std::string& value)
{
  need(value.find_first_of("\t\r\n")==std::string::npos,"external navigation probe row is not one field");
  std::cout << kind << '\t' << key << '\t' << now() << '\t' << value << '\n';
}

void publish(CMOOSCommClient& client,const std::string& row)
{
  const auto tab=row.find('\t');
  need(tab!=std::string::npos && row.size()<=1024,"invalid probe row");
  const auto key=row.substr(0,tab);
  auto value=row.substr(tab+1);
  need((key==request_key || key==input_key) && value.compare(0,9,"schema=1,")==0,
       "probe publication outside fixture policy");
  if(key==input_key) {
    const std::string capture="capture_us=now,";
    const auto position=value.find(capture);
    if(position!=std::string::npos) {
      const auto stamp=std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      value.replace(position,capture.size(),"capture_us="+std::to_string(stamp)+',');
    }
  }
  need(client.Notify(key,value),"external navigation probe publication failed");
  record("request",key,value);
}
}

int main(int argc,char** argv)
{
  CMOOSCommClient client;
  try {
    need(argc==2,"usage: external_navigation_moos_probe DB_PORT");
    std::size_t used=0;
    const std::string input=argv[1];
    const auto port=std::stoi(input,&used);
    need(used==input.size() && port>0 && port<=65535,"invalid private DB port");
    need(fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK)==0,"probe stdin setup failed");
    client.SetQuiet(true);
    need(client.Run("127.0.0.1",port,source,50),"private MOOS connection failed");
    std::cout << std::setprecision(17);
    const auto deadline=now()+60;
    bool registered=false;
    std::string pending;
    while(now()<deadline) {
      if(registered) need(client.IsConnected(),"private MOOS connection lost");
      if(!registered && client.IsConnected()) {
        for(const auto* key:{"MAVMOOS_CAPABILITIES","MAVMOOS_RUNTIME_STATUS","MAVMOOS_EXTERNAL_NAV_STATUS"})
          need(client.Register(key,0),"private ExternalNav registration failed");
        registered=true;
      }
      if(registered) {
        char input_buffer[1024];
        const auto size=read(STDIN_FILENO,input_buffer,sizeof(input_buffer));
        if(size==0) { client.Close();return 0; }
        if(size<0) need(errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR,"probe pipe failed");
        else pending.append(input_buffer,static_cast<std::size_t>(size));
        for(auto end=pending.find('\n');end!=std::string::npos;end=pending.find('\n')) {
          publish(client,pending.substr(0,end));
          pending.erase(0,end+1);
        }
        need(pending.size()<=1024,"probe row exceeds bound");
        MOOSMSG_LIST mail;
        client.Fetch(mail);
        for(const auto& message:mail)
          if(message.GetSource()=="iMAVLink" && message.IsString())
            record("app",message.GetKey(),message.GetString());
      }
      std::cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("external navigation probe deadline elapsed");
  } catch(const std::exception& error) {
    client.Close();
    std::cerr << error.what() << '\n';
    return 1;
  }
}

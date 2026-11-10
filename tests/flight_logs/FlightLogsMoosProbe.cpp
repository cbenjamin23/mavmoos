// Independent MOOS operator/observer. Generated peer only in the receive-only fixture.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <ardupilotmega/mavlink.h>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace {
constexpr char request_key[]="MAVMOOS_FLIGHT_LOG_REQUEST";
constexpr char result_key[]="MAVMOOS_FLIGHT_LOG_RESULT";
constexpr char catalog_key[]="MAVMOOS_FLIGHT_LOG_CATALOG";
constexpr char tx_key[]="MAVMOOS_FLIGHT_LOG_TX_RESULT";
constexpr char runtime_key[]="MAVMOOS_RUNTIME_STATUS";
constexpr char capabilities_key[]="MAVMOOS_CAPABILITIES";
constexpr char app_key[]="MAVMOOS_STATUS";
double now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void require(bool value,const char* error) { if(!value) throw std::runtime_error(error); }
std::string quote(const std::string& text)
{
  constexpr char hex[]="0123456789abcdef";
  std::string quoted="\"";
  for(unsigned char c:text) {
    if(c=='"' || c=='\\') { quoted+='\\'; quoted+=static_cast<char>(c); }
    else if(c<32) { quoted+="\\u00";quoted+=hex[c>>4];quoted+=hex[c&15]; }
    else quoted+=static_cast<char>(c);
  }
  return quoted+'"';
}
uint16_t port(const std::string& value)
{
  std::size_t used=0;
  const int result=std::stoi(value,&used);
  require(used==value.size() && result>0 && result<=65535,"invalid loopback port");
  return static_cast<uint16_t>(result);
}
sockaddr_in endpoint(uint16_t port_number)
{
  sockaddr_in address{};
  address.sin_family=AF_INET;address.sin_port=htons(port_number);
  address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  return address;
}
struct Peer {
  int fd=-1;
  sockaddr_in remote{};
  mavlink_status_t status{};
  double next=0;
  unsigned sent=0,received=0;
  ~Peer() { if(fd>=0) close(fd); }
  void open(uint16_t local_port,uint16_t remote_port)
  {
    fd=socket(AF_INET,SOCK_DGRAM,0);
    const auto local=endpoint(local_port);
    remote=endpoint(remote_port);
    require(fd>=0 && fcntl(fd,F_SETFL,O_NONBLOCK)==0 &&
      bind(fd,reinterpret_cast<const sockaddr*>(&local),sizeof(local))==0,"generated peer bind failed");
  }
  void send(const mavlink_message_t& frame)
  {
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
    const auto count=mavlink_msg_to_send_buffer(bytes,&frame);
    require(sendto(fd,bytes,count,0,reinterpret_cast<const sockaddr*>(&remote),sizeof(remote))==count,
      "generated peer send failed");
    ++sent;
  }
  void pump(double current)
  {
    if(fd<0) return;
    if(current>=next) {
      next=current+.1;
      mavlink_message_t frame{};
      mavlink_msg_heartbeat_pack_status(1,1,&status,&frame,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,0,MAV_STATE_STANDBY);
      send(frame);
      mavlink_msg_extended_sys_state_pack_status(1,1,&status,&frame,0,MAV_LANDED_STATE_ON_GROUND);
      send(frame);
      mavlink_msg_log_entry_pack_status(1,1,&status,&frame,1,1,1,123,200);
      send(frame);
    }
    uint8_t packet[2048]{};
    for(unsigned attempt=0;attempt<128;++attempt) {
      if(recv(fd,packet,sizeof(packet),0)<0) {
        require(errno==EAGAIN || errno==EWOULDBLOCK,"generated peer receive failed");
        return;
      }
      ++received;
    }
    throw std::runtime_error("unexpected unbounded bridge egress");
  }
};
bool has(const std::string& text,const char* fragment) { return text.find(fragment)!=std::string::npos; }
bool ready(const std::string& status)
{
  return has(status,"heartbeat_valid=1") && has(status,"landed_valid=1") &&
    has(status,"armed=0") && has(status,"in_air=0") && has(status,"permission=false");
}
}

int main(int argc,char** argv)
{
  CMOOSCommClient client;
  try {
    require(argc==3 || argc==5,"usage: flight_logs_moos_probe MOOS_PORT native|rx [PEER_PORT BRIDGE_PORT]");
    const std::string scenario=argv[2];
    const bool rx=scenario=="rx";
    require((rx && argc==5) || (scenario=="native" && argc==3),"invalid probe scenario");
    Peer peer;
    if(rx) peer.open(port(argv[3]),port(argv[4]));
    client.SetQuiet(true);
    require(client.Run("127.0.0.1",port(argv[1]),"flight_logs_probe",50),"MOOS client startup failed");
    bool registered=false,composition=false,ground=false,list_done=false,catalog_done=false,download_done=false;
    unsigned stage=0;
    double finish=0,deadline=now()+(rx?12:82);
    while(now()<deadline) {
      const auto current=now();
      peer.pump(current);
      if(client.IsConnected() && !registered) {
        for(const auto* key:{result_key,catalog_key,tx_key,runtime_key,capabilities_key,app_key,"APPCAST"})
          client.Register(key,0);
        client.Notify("APPCAST_REQ",std::string("node=all,app=all,duration=6,key=flight-logs-probe"));
        registered=true;
      }
      MOOSMSG_LIST mail;
      client.Fetch(mail);
      for(const auto& message:mail) {
        if(message.GetSource()!="iMAVLink" || !message.IsString()) continue;
        const auto key=message.GetKey(),value=message.GetString();
        if(key==runtime_key) ground=ready(value);
        if(key==capabilities_key)
          composition=has(value,"\"name\":\"flight_logs\"") &&
            has(value,rx?"\"execution_mode\":\"RX_ONLY\"":"\"execution_mode\":\"SITL_CONTROL\"");
        if(key==result_key && has(value,"\"id\":\"initial\"")) {
          list_done=has(value,rx?"\"status\":\"rejected\"":"\"status\":\"complete\"");
        } else if(key==catalog_key && has(value,"\"id\":\"initial\"") && has(value,"\"status\":\"complete\"")) {
          catalog_done=true;
        } else if(key==result_key && has(value,"\"id\":\"retrieve\"") && has(value,"\"status\":\"complete\"")) {
          download_done=true;
        } else if(key==result_key && has(value,"\"id\":\"after-end\"") && has(value,"\"status\":\"complete\"")) {
          if(stage==3) finish=current+.3;
        }
        std::cout<<"{\"mono\":"<<std::setprecision(17)<<current<<",\"key\":"<<quote(key)
          <<",\"moos_time\":"<<message.GetTime()<<",\"value\":"<<quote(value)<<"}\n";
      }
      if(!stage && composition && ground) {
        require(client.Notify(request_key,std::string("schema=1,id=initial,action=list")),"initial request failed");
        stage=1;
      } else if(stage==1 && list_done) {
        if(rx) { finish=current+.4;stage=4; }
        else if(catalog_done) {
          require(client.Notify(request_key,std::string("schema=1,id=retrieve,action=download,catalog=initial,log_id=1")),
            "download request failed");
          stage=2;
        }
      } else if(stage==2 && download_done) {
        require(client.Notify(request_key,std::string("schema=1,id=after-end,action=list")),"post-END request failed");
        stage=3;
      }
      std::cout.flush();
      if(finish && current>=finish) {
        if(rx) {
          require(peer.sent>=3 && peer.received==0,"receive-only bridge emitted MAVLink");
          std::cout<<"{\"fixture\":\"generated_receive_only\",\"sent\":"<<peer.sent
            <<",\"received_from_app\":"<<peer.received<<"}\n";
        }
        client.Close();return 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("operator workflow did not complete inside the probe deadline");
  } catch(const std::exception& error) {
    client.Close();std::cerr<<"flight_logs probe: "<<error.what()<<'\n';return 1;
  }
}

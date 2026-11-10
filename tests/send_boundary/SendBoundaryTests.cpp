// Pure final-handoff tests and exact bytes on an owned localhost-only peer.
// The named process fixture is inert, not ArduPilot or a native simulator.
#include "SimulationSendGate.h"
#include "UdpReceiver.h"
#include <ardupilotmega/mavlink.h>
#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <limits>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

using mavmoos::detail::attemptSimulationSend;
using Reason=mavmoos::detail::SimulationSendReason;
using Outcome=mavmoos::detail::SimulationSendOutcome;
namespace {
void require(bool value,const std::string& message)
{if(!value)throw std::runtime_error(message);}
struct Fake {
  double time=100;
  double expires=101;
  unsigned proof_calls=0,eligible_calls=0,writes=0,clock_calls=0;
  std::function<bool()> proof_action;
  std::function<bool(double)> eligible_action;
  std::function<double(unsigned)> clock_action;
  std::function<bool()> write_action;
  Outcome run() {
    return attemptSimulationSend(expires,
      [&]{++proof_calls;return proof_action?proof_action():true;},
      [&]{++clock_calls;return clock_action?clock_action(clock_calls):time;},
      [&](double at){++eligible_calls;require(at==time,"eligibility received incorrect local clock");return eligible_action?eligible_action(at):true;},
      [&]{++writes;return write_action?write_action():true;});
  }
};
void noWrite(const Fake& fixture,const Outcome& result,Reason reason)
{require(!result.accepted()&&!result.write_attempted&&fixture.writes==0&&result.reason==reason,"unexpected final-gate outcome/write");}

void fakeTests()
{
  unsigned passed=0;
  auto test=[&](const char* name,const auto& run){run();++passed;std::cout<<"PASS "<<name<<'\n';};
  test("valid permit proves endpoint then rechecks owner and writes exactly once",[]{
    Fake f;std::vector<std::string> order;
    f.clock_action=[&](unsigned){order.push_back("clock");return f.time;};
    f.proof_action=[&]{order.push_back("proof");return true;};
    f.eligible_action=[&](double){order.push_back("eligible");return true;};
    f.write_action=[&]{order.push_back("write");return true;};
    const auto result=f.run();
    require(result.accepted()&&result.write_attempted&&result.checked_at==100&&f.writes==1,"valid send missing");
    require(order==std::vector<std::string>({"clock","proof","clock","eligible","clock","write"}),"incorrect boundary order");
  });
  test("nonfinite zero and negative original expiry reject before proof",[]{
    for(double value:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),0.,-1.}) {Fake f;f.expires=value;noWrite(f,f.run(),Reason::InvalidPermit);require(f.clock_calls==0&&f.proof_calls==0,"invalid permit performed preflight");}
  });
  test("every callback is mandatory and missing callback cannot write",[]{
    unsigned writes=0;
    const std::function<bool()> proof=[]{return true;},write=[&]{++writes;return true;};
    const std::function<double()> clock=[]{return 100.;};const std::function<bool(double)> eligible=[](double){return true;};
    for(unsigned missing=0;missing<4;++missing) {
      const auto result=attemptSimulationSend(101,missing==0?std::function<bool()>{}:proof,
        missing==1?std::function<double()>{}:clock,missing==2?std::function<bool(double)>{}:eligible,
        missing==3?std::function<bool()>{}:write);
      require(result.reason==Reason::InvalidPermit&&!result.write_attempted,"missing callback admitted");
    }require(writes==0,"missing callback wrote");
  });
  test("invalid initial monotonic clock rejects before endpoint proof",[]{
    for(double value:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),-1.}) {Fake f;f.time=value;noWrite(f,f.run(),Reason::ClockInvalid);require(f.proof_calls==0,"bad clock invoked proof");}
  });
  test("expiry is exclusive and already expired work does not prove an endpoint",[]{
    for(double value:{99.,100.}) {Fake f;f.expires=value;noWrite(f,f.run(),Reason::Expired);require(f.proof_calls==0,"expired work invoked proof");}
  });
  test("endpoint rejection prevents eligibility and I/O",[]{
    Fake f;f.proof_action=[]{return false;};noWrite(f,f.run(),Reason::EndpointRejected);require(f.eligible_calls==0,"denied endpoint reached eligibility");
  });
  test("expiry consumed during endpoint proof is never renewed",[]{
    for(double end:{101.,101.01}) {Fake f;f.proof_action=[&]{f.time=end;return true;};noWrite(f,f.run(),Reason::Expired);require(f.eligible_calls==0,"expired proof reached eligibility");}
  });
  test("endpoint proof may finish inside original lifetime without adding time",[]{
    Fake f;f.proof_action=[&]{f.time=100.9;return true;};const auto result=f.run();
    require(result.accepted()&&result.checked_at==100.9&&f.expires==101,"proof changed expiry");
  });
  test("endpoint proof cannot conceal a backwards or invalid clock",[]{
    for(double end:{99.9,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()}) {Fake f;f.proof_action=[&]{f.time=end;return true;};noWrite(f,f.run(),Reason::ClockInvalid);}
  });
  test("cancellation arriving during proof stops the waiting frame",[]{
    Fake f;bool current=true;f.proof_action=[&]{current=false;return true;};f.eligible_action=[&](double){return current;};
    noWrite(f,f.run(),Reason::Ineligible);require(f.eligible_calls==1,"no current-owner check");
  });
  test("owner revalidation sees time after proof rather than queued time",[]{
    Fake f;f.proof_action=[&]{f.time=100.7;return true;};f.eligible_action=[](double at){return at<100.5;};noWrite(f,f.run(),Reason::Ineligible);
  });
  test("expiry consumed by eligibility itself stops before write",[]{
    for(double end:{101.,105.}) {Fake f;f.eligible_action=[&](double){f.time=end;return true;};noWrite(f,f.run(),Reason::Expired);}
  });
  test("eligibility cannot conceal a backwards or invalid clock",[]{
    for(double end:{99.9,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()}) {Fake f;f.eligible_action=[&](double){f.time=end;return true;};noWrite(f,f.run(),Reason::ClockInvalid);}
  });
  test("preflight callback exceptions have no write attempt",[]{
    for(unsigned stage=0;stage<3;++stage) {Fake f;
      if(stage==0)f.clock_action=[](unsigned)->double{throw std::runtime_error("clock");};
      if(stage==1)f.proof_action=[]()->bool{throw std::runtime_error("proof");};
      if(stage==2)f.eligible_action=[](double)->bool{throw std::runtime_error("eligible");};
      noWrite(f,f.run(),Reason::PreflightException);
    }
  });
  test("failed writer is attempted once without retry",[]{
    Fake f;f.write_action=[]{return false;};const auto result=f.run();
    require(result.reason==Reason::WriteFailed&&result.write_attempted&&!result.accepted()&&f.writes==1,"writer failure lost distinction");
  });
  test("throwing writer retains possible-effect distinction",[]{
    Fake f;f.write_action=[]()->bool{throw std::runtime_error("writer outcome unknown");};const auto result=f.run();
    require(result.reason==Reason::WriteException&&result.write_attempted&&!result.accepted()&&f.writes==1,"writer exception claimed no attempt");
  });
  test("no valid permit assumes a particular process-uptime epoch",[]{
    for(double start:{0.,5000000.,1000000000.}) {Fake f;f.time=start;f.expires=start+1;const auto result=f.run();require(result.accepted()&&result.checked_at==start,"valid epoch denied");}
  });
  std::cout<<"PASS "<<passed<<" pure send-boundary groups; no socket or simulator\n";
}

class Peer {
public:
  explicit Peer(const std::string& bind_address="127.0.0.1") {
    m_fd=socket(AF_INET,SOCK_DGRAM,0);require(m_fd>=0,"peer socket failed");
    sockaddr_in address{};address.sin_family=AF_INET;
    require(inet_pton(AF_INET,bind_address.c_str(),&address.sin_addr)==1,"peer address invalid");
    if(bind(m_fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))!=0) {close(m_fd);m_fd=-1;throw std::runtime_error("peer bind failed");}
    socklen_t length=sizeof(address);require(getsockname(m_fd,reinterpret_cast<sockaddr*>(&address),&length)==0,"peer name failed");
    m_port=ntohs(address.sin_port);require(fcntl(m_fd,F_SETFD,FD_CLOEXEC)==0,"peer cloexec failed");
    const int flags=fcntl(m_fd,F_GETFL,0);require(flags>=0&&fcntl(m_fd,F_SETFL,flags|O_NONBLOCK)==0,"peer nonblocking failed");
  }
  ~Peer(){if(m_fd>=0)close(m_fd);}
  Peer(const Peer&)=delete;Peer& operator=(const Peer&)=delete;
  int fd()const{return m_fd;}uint16_t port()const{return m_port;}
  std::vector<uint8_t> receive() const {
    std::array<uint8_t,MAVLINK_MAX_PACKET_LEN+1> bytes{};
    const auto size=recv(m_fd,bytes.data(),bytes.size(),0);
    if(size<0) {require(errno==EAGAIN||errno==EWOULDBLOCK,"peer receive error");return {};}
    return {bytes.begin(),bytes.begin()+size};
  }
  std::vector<uint8_t> waitReceive(int milliseconds) const {
    pollfd event{m_fd,POLLIN,0};int status=0;
    do {status=poll(&event,1,milliseconds);} while(status<0&&errno==EINTR);
    require(status>=0,"peer poll error");return status?receive():std::vector<uint8_t>{};
  }
private:
  int m_fd=-1;uint16_t m_port=0;
};
class InertOwner {
public:
  InertOwner(const std::string& executable,const Peer& peer) {
    int input[2],output[2];require(pipe(input)==0,"owner input pipe failed");
    if(pipe(output)!=0){close(input[0]);close(input[1]);throw std::runtime_error("owner output pipe failed");}
    m_pid=fork();
    if(m_pid<0){for(int fd:{input[0],input[1],output[0],output[1]})close(fd);throw std::runtime_error("owner fork failed");}
    if(m_pid==0) {
      close(input[1]);close(output[0]);
      if(dup2(input[0],STDIN_FILENO)<0||dup2(output[1],STDOUT_FILENO)<0||fcntl(peer.fd(),F_SETFD,0)<0)_exit(126);
      close(input[0]);close(output[1]);
      const auto number=std::to_string(peer.fd());
      char* const args[]={const_cast<char*>(executable.c_str()),const_cast<char*>("--inert-send-boundary-fixture"),const_cast<char*>(number.c_str()),nullptr};
      char locale[]="LC_ALL=C";char* const env[]={locale,nullptr};
      execve(executable.c_str(),args,env);_exit(127);
    }
    close(input[0]);close(output[1]);m_input=input[1];
    try {
      require(fcntl(m_input,F_SETFD,FD_CLOEXEC)==0,"owner keepalive cloexec failed");
      std::string ready;const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
      while(ready.find('\n')==std::string::npos&&ready.size()<64&&std::chrono::steady_clock::now()<deadline) {
        pollfd event{output[0],POLLIN,0};const int status=poll(&event,1,50);
        if(status<0&&errno==EINTR)continue;
        require(status>=0,"owner ready poll failed");if(status==0)continue;
        char buffer[32];const auto size=read(output[0],buffer,sizeof(buffer));require(size>0,"owner exited before ready");ready.append(buffer,static_cast<std::size_t>(size));
      }
      std::istringstream data(ready);std::string magic,extra;unsigned port=0;data>>magic>>port;
      require(magic=="INERT_SEND_BOUNDARY_V1"&&port==peer.port()&&!(data>>extra),"owner handshake failed");close(output[0]);
    }catch(...){close(output[0]);stop();throw;}
  }
  ~InertOwner(){stop();}
  InertOwner(const InertOwner&)=delete;InertOwner& operator=(const InertOwner&)=delete;
  int pid()const{return m_pid;}
  void stop() {
    if(m_input>=0){close(m_input);m_input=-1;}
    if(m_pid<=0)return;
    for(unsigned attempt=0;attempt<20;++attempt) {
      const auto result=waitpid(m_pid,nullptr,WNOHANG);
      if(result==m_pid||(result<0&&errno==ECHILD)){m_pid=-1;return;}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    kill(m_pid,SIGTERM);while(waitpid(m_pid,nullptr,0)<0&&errno==EINTR){}m_pid=-1;
  }
private:
  pid_t m_pid=-1;int m_input=-1;
};
std::vector<uint8_t> frame() {
  mavlink_set_position_target_local_ned_t payload{};payload.time_boot_ms=1234;payload.target_system=1;payload.target_component=1;
  payload.coordinate_frame=MAV_FRAME_LOCAL_NED;payload.type_mask=0x09C7;payload.vx=1.25f;payload.vy=-.5f;payload.vz=.25f;
  mavlink_message_t message{};mavlink_status_t codec{};
  mavlink_msg_set_position_target_local_ned_encode_status(245,191,&codec,&message,&payload);
  std::vector<uint8_t> bytes(MAVLINK_MAX_PACKET_LEN);bytes.resize(mavlink_msg_to_send_buffer(bytes.data(),&message));return bytes;
}
mavmoos::RxConfig config(uint16_t peer,const std::string& local_address="127.0.0.1") {
  // Reserve against the same address family/range the receiver will bind.
  // A port free only on127.0.0.1 need not be free for a wildcard socket.
  Peer temporary(local_address);
  mavmoos::RxConfig config;config.bind_address=local_address;config.peer_address="127.0.0.1";config.bind_port=temporary.port();config.peer_port=peer;
  config.target_system=config.target_component=1;config.expected_type=MAV_TYPE_QUADROTOR;config.expected_autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA;
  return config;
}
void loopbackTests(const std::string& executable)
{
#if !defined(__APPLE__) && !defined(__linux__)
  throw std::runtime_error("exact process/socket guard tests require Darwin or Linux");
#endif
  unsigned passed=0;auto test=[&](const char* name,const auto& run){run();++passed;std::cout<<"PASS "<<name<<'\n';};
  Peer peer;InertOwner owner(executable,peer);const auto bytes=frame();
  mavmoos::UdpReceiver receiver;require(receiver.open(config(peer.port())),"receiver open failed");
  test("real receiver retains exact native-name PID and socket guard",[&]{
    require(mavmoos::UdpReceiver::isSitlEndpoint(owner.pid(),peer.port(),"arducopter"),"inert exact owner not recognized");
    require(!mavmoos::UdpReceiver::isSitlEndpoint(owner.pid(),peer.port(),"ardurover"),"cross profile accepted");
  });
  test("fresh permit sends exactly one unchanged standard MAVLink datagram",[&]{
    unsigned calls=0;const double expires=mavmoos::monotonicSeconds()+1;
    require(receiver.sendSimulationFrame(bytes.data(),bytes.size(),owner.pid(),expires,[&](double at){++calls;return std::isfinite(at)&&at<expires;}),"fresh datagram denied");
    const auto actual=peer.waitReceive(200);
    require(actual==bytes&&calls==1,"datagram bytes or gate count changed: expected="+std::to_string(bytes.size())+",actual="+std::to_string(actual.size())+",gates="+std::to_string(calls));
    mavlink_message_t buffer{},message{};mavlink_status_t state{},result{};bool decoded=false;
    for(const auto byte:actual)if(mavlink_frame_char_buffer(&buffer,&state,byte,&message,&result)==MAVLINK_FRAMING_OK)decoded=true;
    mavlink_set_position_target_local_ned_t payload{};mavlink_msg_set_position_target_local_ned_decode(&message,&payload);
    require(decoded&&message.sysid==245&&message.compid==191&&message.msgid==84&&payload.target_system==1&&payload.target_component==1&&payload.vx==1.25f&&payload.vy==-.5f,"standard frame decode mismatch");
    require(peer.waitReceive(20).empty(),"fresh permit wrote more than one datagram");
  });
  test("expired invalid and absent permit emit zero datagrams",[&]{
    unsigned calls=0;
    for(double expiry:{mavmoos::monotonicSeconds()-.1,double(std::numeric_limits<double>::quiet_NaN()),double(std::numeric_limits<double>::infinity()),0.})
      require(!receiver.sendSimulationFrame(bytes.data(),bytes.size(),owner.pid(),expiry,[&](double){++calls;return true;}),"invalid expiry admitted");
    require(!receiver.sendSimulationFrame(bytes.data(),bytes.size(),owner.pid(),mavmoos::monotonicSeconds()+1,{}),"missing eligibility admitted");
    require(calls==0&&peer.waitReceive(20).empty(),"invalid permit reached owner or emitted bytes");
  });
  test("revoked owner or disconnect predicate emits zero datagrams",[&]{
    require(!receiver.sendSimulationFrame(bytes.data(),bytes.size(),owner.pid(),mavmoos::monotonicSeconds()+1,[](double){return false;}),"revoked predicate accepted");
    require(peer.waitReceive(20).empty(),"revoked predicate emitted bytes");
  });
  test("expiry consumed inside revalidation emits zero datagrams",[&]{
    bool called=false;const double expiry=mavmoos::monotonicSeconds()+1;
    require(!receiver.sendSimulationFrame(bytes.data(),bytes.size(),owner.pid(),expiry,[&](double){
      called=true;while(mavmoos::monotonicSeconds()<=expiry)std::this_thread::sleep_for(std::chrono::milliseconds(2));return true;
    }),"expiry consumed by validator still sent");
    require(called&&peer.waitReceive(20).empty(),"delayed revalidation fixture did not prove zero datagrams");
  });
  test("wrong PID empty frame oversized frame and null data cannot write",[&]{
    unsigned calls=0;const auto eligible=[&](double){++calls;return true;};const double expires=mavmoos::monotonicSeconds()+1;
    require(!receiver.sendSimulationFrame(bytes.data(),bytes.size(),getpid(),expires,eligible),"ordinary process substituted for owner");
    require(!receiver.sendSimulationFrame(bytes.data(),0,owner.pid(),expires,eligible),"empty frame accepted");
    require(!receiver.sendSimulationFrame(bytes.data(),MAVLINK_MAX_PACKET_LEN+1,owner.pid(),expires,eligible),"oversized frame accepted");
    require(!receiver.sendSimulationFrame(nullptr,bytes.size(),owner.pid(),expires,eligible),"null buffer accepted");
    require(calls==0&&peer.waitReceive(20).empty(),"bad input reached predicate/wire");
  });
  test("wrong configured profile and wildcard local binding remain prohibited",[&]{
    auto other=config(peer.port());other.simulator_profile="ardurover";mavmoos::UdpReceiver rover;require(rover.open(other),"Rover receiver open failed");
    require(!rover.sendSimulationFrame(bytes.data(),bytes.size(),owner.pid(),mavmoos::monotonicSeconds()+1,[](double){return true;}),"profile widened send guard");
    rover.close();other=config(peer.port(),"0.0.0.0");mavmoos::UdpReceiver wildcard;
    const bool opened=wildcard.open(other);require(opened,"wildcard receive-only open failed, errno="+std::to_string(wildcard.stats().last_errno)+",port="+std::to_string(other.bind_port));
    require(!wildcard.sendSimulationFrame(bytes.data(),bytes.size(),owner.pid(),mavmoos::monotonicSeconds()+1,[](double){return true;}),"wildcard send guard widened");
    require(peer.waitReceive(20).empty(),"wrong profile/route emitted bytes");
  });
  test("closed sender and exited socket owner cannot retain send eligibility",[&]{
    receiver.close();require(!receiver.sendSimulationFrame(bytes.data(),bytes.size(),owner.pid(),mavmoos::monotonicSeconds()+1,[](double){return true;}),"closed sender wrote");
    require(receiver.open(config(peer.port())),"reopen receiver failed");const auto pid=owner.pid();owner.stop();
    require(!receiver.sendSimulationFrame(bytes.data(),bytes.size(),pid,mavmoos::monotonicSeconds()+1,[](double){return true;}),"exited owner wrote");require(peer.waitReceive(20).empty(),"closed/exited case emitted bytes");
  });
  receiver.close();
  std::cout<<"PASS "<<passed<<" owned-loopback/inert-owner send groups; no ArduPilot or real vehicle\n";
}
} // namespace
int main(int argc,char** argv)
{
  try {
    require(argc>=2,"usage: send_boundary_tests fake | loopback INERT_FIXTURE");
    const std::string mode=argv[1];
    if(mode=="fake"&&argc==2)fakeTests();
    else if(mode=="loopback"&&argc==3)loopbackTests(argv[2]);
    else throw std::runtime_error("invalid send-boundary test arguments");
    return 0;
  }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}

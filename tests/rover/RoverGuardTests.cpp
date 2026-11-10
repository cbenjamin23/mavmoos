// Software guard predicates against inert child sockets only. No protocol sends.
#include "UdpReceiver.h"
#include <chrono>
#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace {
void require(bool value,const std::string& why){if(!value)throw std::runtime_error(why);}
class SocketFixture {
public:
  SocketFixture(const std::string& executable,const std::string& address)
  {
    int input[2]{-1,-1},output[2]{-1,-1};
    if(pipe(input)!=0)throw std::runtime_error("fixture input pipe failed");
    if(pipe(output)!=0){close(input[0]);close(input[1]);throw std::runtime_error("fixture output pipe failed");}
    m_pid=fork();
    if(m_pid<0){for(int fd:{input[0],input[1],output[0],output[1]})close(fd);throw std::runtime_error("fixture fork failed");}
    if(m_pid==0) {
      close(input[1]);close(output[0]);
      if(dup2(input[0],STDIN_FILENO)<0||dup2(output[1],STDOUT_FILENO)<0)_exit(126);
      close(input[0]);close(output[1]);
      char* const args[]={const_cast<char*>(executable.c_str()),const_cast<char*>("--inert-guard-fixture"),const_cast<char*>(address.c_str()),nullptr};
      // No inherited environment can select a route, preload a library or alter
      // the fixture. execve uses the exact CMake-supplied dedicated fixture path.
      char locale[]="LC_ALL=C";char* const env[]={locale,nullptr};
      execve(executable.c_str(),args,env);_exit(127);
    }
    close(input[0]);close(output[1]);m_input=input[1];
    try {
      require(fcntl(m_input,F_SETFD,FD_CLOEXEC)==0,"fixture keepalive close-on-exec failed");
      std::string ready;const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
      while(ready.find('\n')==std::string::npos&&ready.size()<64&&std::chrono::steady_clock::now()<deadline) {
        pollfd event{output[0],POLLIN,0};const int status=poll(&event,1,50);
        if(status<0&&errno==EINTR)continue;
        require(status>=0,"fixture ready poll failed");
        if(status==0)continue;
        char buffer[32];const auto count=read(output[0],buffer,sizeof(buffer));
        require(count>0,"fixture exited before ready");ready.append(buffer,static_cast<std::size_t>(count));
      }
      std::string magic,extra;unsigned port=0;std::istringstream parsed(ready);parsed>>magic>>port;
      require(magic=="INERT_GUARD_FIXTURE_V1"&&port>0&&port<=65535&&!(parsed>>extra),"invalid or missing inert fixture handshake");
      m_port=static_cast<uint16_t>(port);close(output[0]);
    }catch(...){close(output[0]);stop();throw;}
  }
  SocketFixture(const SocketFixture&)=delete;SocketFixture& operator=(const SocketFixture&)=delete;
  ~SocketFixture(){stop();}
  int pid()const{return m_pid;}uint16_t port()const{return m_port;}
  void stop()
  {
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
  pid_t m_pid=-1;int m_input=-1;uint16_t m_port=0;
};
}
int main(int argc,char** argv)
{
#if !defined(__APPLE__) && !defined(__linux__)
  std::cout<<"SKIP guard inspection is implemented for Darwin and Linux only\n";return 77;
#endif
  try {
    require(argc==3,"usage: rover_profile_guard_tests INERT_ARDUROVER_FIXTURE INERT_ARDUCOPTER_FIXTURE");
    using mavmoos::UdpReceiver;unsigned count=0;
    auto test=[&](const char* name,const auto& run){run();++count;std::cout<<"PASS "<<name<<'\n';};
    test("invalid PID profile and zero endpoint deny without spawning native",[]{
      for(int pid:{-1,0})for(const auto& profile:{"ardurover","arducopter"})
        require(!UdpReceiver::isSitlProcess(pid,profile)&&!UdpReceiver::isSitlEndpoint(pid,12345,profile),"invalid owner accepted");
      for(const auto& profile:{"","ArduRover","boat","ardurover;echo_bad"})
        require(!UdpReceiver::isSitlProcess(getpid(),profile)&&!UdpReceiver::isSitlEndpoint(getpid(),12345,profile),"unsupported profile accepted");
      require(!UdpReceiver::isSitlProcess(getpid(),"ardurover")&&!UdpReceiver::isSitlEndpoint(getpid(),0,"ardurover"),"ordinary self/zero endpoint accepted");
    });
    SocketFixture rover(argv[1],"127.0.0.1"),copter(argv[2],"127.0.0.1");
    test("exact basename owned loopback socket passes predicate only",[&]{
      require(UdpReceiver::isSitlProcess(rover.pid(),"ardurover")&&UdpReceiver::isSitlEndpoint(rover.pid(),rover.port(),"ardurover"),"inert Rover owner not recognized");
      require(UdpReceiver::isSitlProcess(copter.pid(),"arducopter")&&UdpReceiver::isSitlEndpoint(copter.pid(),copter.port(),"arducopter"),"inert Copter owner not recognized");
    });
    test("Copter and Rover executable profiles cannot substitute",[&]{
      require(!UdpReceiver::isSitlProcess(rover.pid(),"arducopter")&&!UdpReceiver::isSitlEndpoint(rover.pid(),rover.port(),"arducopter"),"Rover widened Copter guard");
      require(!UdpReceiver::isSitlProcess(copter.pid(),"ardurover")&&!UdpReceiver::isSitlEndpoint(copter.pid(),copter.port(),"ardurover"),"Copter widened Rover guard");
    });
    test("a different process owning the port does not satisfy nominated PID",[&]{
      require(rover.port()!=copter.port(),"fixture ports must be distinct");
      require(!UdpReceiver::isSitlEndpoint(rover.pid(),copter.port(),"ardurover")&&!UdpReceiver::isSitlEndpoint(copter.pid(),rover.port(),"arducopter"),"foreign process endpoint accepted");
    });
    test("wildcard MAVLink socket remains ineligible",[&]{
      SocketFixture wildcard(argv[1],"0.0.0.0");
      require(UdpReceiver::isSitlProcess(wildcard.pid(),"ardurover")&&!UdpReceiver::isSitlEndpoint(wildcard.pid(),wildcard.port(),"ardurover"),"wildcard endpoint widened effectful guard");
    });
    test("exited nominated owner cannot retain endpoint eligibility",[&]{
      const auto pid=rover.pid();const auto port=rover.port();rover.stop();
      require(!UdpReceiver::isSitlProcess(pid,"ardurover")&&!UdpReceiver::isSitlEndpoint(pid,port,"ardurover"),"exited owner accepted");
    });
    std::cout<<"PASS "<<count<<" inert fake process/socket guard groups; no MAVLink sends or native simulator\n";return 0;
  }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}

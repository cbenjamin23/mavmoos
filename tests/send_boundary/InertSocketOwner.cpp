// Inert inherited socket owner for send-boundary tests, NOT ArduPilot/SITL.
// No MAVLink parser, send, receive loop, device access or environment route.
#include <arpa/inet.h>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

int main(int argc,char** argv)
{
  if(argc!=3 || std::string(argv[1])!="--inert-send-boundary-fixture") return 2;
  char* end=nullptr;errno=0;const long parsed=std::strtol(argv[2],&end,10);
  if(errno || !end || *end || parsed<3 || parsed>INT_MAX) return 2;
  const int descriptor=static_cast<int>(parsed);
  sockaddr_in address{};socklen_t length=sizeof(address);
  int type=0;socklen_t type_length=sizeof(type);
  if(getsockname(descriptor,reinterpret_cast<sockaddr*>(&address),&length)!=0 ||
     length!=sizeof(address) || address.sin_family!=AF_INET ||
     address.sin_addr.s_addr!=htonl(INADDR_LOOPBACK) || address.sin_port==0 ||
     getsockopt(descriptor,SOL_SOCKET,SO_TYPE,&type,&type_length)!=0 || type!=SOCK_DGRAM) return 3;
  std::cout<<"INERT_SEND_BOUNDARY_V1 "<<ntohs(address.sin_port)<<'\n'<<std::flush;
  char value=0;
  while(read(STDIN_FILENO,&value,1)<0 && errno==EINTR) {}
  close(descriptor);return 0;
}

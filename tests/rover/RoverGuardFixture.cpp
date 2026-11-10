// Inert socket holder for guard predicates. This is NOT ArduPilot or a simulator.
// No MAVLink, socket sends, receive handling, environment configuration or control.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <iostream>
#include <string>

int main(int argc,char** argv)
{
  if(argc!=3||std::string(argv[1])!="--inert-guard-fixture")return 2;
  const std::string address=argv[2];
  if(address!="127.0.0.1"&&address!="0.0.0.0")return 2;
  const int fd=socket(AF_INET,SOCK_DGRAM,0);if(fd<0)return 3;
  sockaddr_in local{};local.sin_family=AF_INET;local.sin_port=0;
  if(inet_pton(AF_INET,address.c_str(),&local.sin_addr)!=1||bind(fd,reinterpret_cast<sockaddr*>(&local),sizeof(local))!=0){close(fd);return 4;}
  socklen_t size=sizeof(local);
  if(getsockname(fd,reinterpret_cast<sockaddr*>(&local),&size)!=0){close(fd);return 5;}
  std::cout<<"INERT_GUARD_FIXTURE_V1 "<<ntohs(local.sin_port)<<'\n'<<std::flush;
  // Parent holds stdin open; EOF or process termination releases the socket.
  char byte=0;while(read(STDIN_FILENO,&byte,1)<0&&errno==EINTR){}
  close(fd);return 0;
}

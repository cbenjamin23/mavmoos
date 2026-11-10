// Fake-service diagnostic: delayed observations/ACKs must not rescue a deadline.
#include "ControlComponents.h"
#include <iostream>
#include <memory>
#include <stdexcept>
using namespace mavmoos::runtime;
struct Host final:Services {
  double clock=100;
  Vehicle state;
  std::string output;
  Host(){state.mode=5;state.heartbeat_valid=true;state.heartbeat_at=clock;}
  double now()const override{return clock;}
  const Vehicle& vehicle()const override{return state;}
  void publish(const std::string&,const std::string& text)override{output+=text+'\n';}
  void publish(const std::string&,double)override{}
  bool sendCommand(const CommandRequest&)override{return true;}
  bool acquireNavigation(const std::string&)override{return false;}
  void revokeNavigation(const std::string&)override{}
  bool navigation(const NavigationTarget&)override{return false;}
};
int main(int argc,char**)
{
  const bool expect_fixed=argc>1;
  for(int scenario=0;scenario<3;++scenario){
    Host h;auto p=makeCopterCommands(h);
    h.clock=100.01;Mail m;m.key="MAVMOOS_COMMAND";m.text="id=deadline,action=guided";
    m.received_at=m.published_at=h.clock;p->onMail(m);p->iterate();
    auto ack=[&](){mavlink_message_t wire{};mavlink_msg_command_ack_pack(1,1,&wire,MAV_CMD_DO_SET_MODE,MAV_RESULT_ACCEPTED,0,0,245,191);p->onMessage(wire,h.clock);};
    if(scenario==0){h.clock=100.02;ack();h.clock=108.02;}
    else {h.clock=104.02;if(scenario==1)ack();}
    h.state.mode=4;h.state.heartbeat_at=h.clock;p->iterate();
    const bool completed=h.output.find("status=COMPLETED")!=std::string::npos;
    const bool timed_out=h.output.find("status=TIMED_OUT")!=std::string::npos;
    std::cout<<"scenario="<<scenario<<",completed="<<completed<<",timed_out="<<timed_out<<'\n'<<h.output;
    if(expect_fixed?(!timed_out||completed):(!completed||timed_out))return 1;
  }
  std::cout<<(expect_fixed?"FIXED all3 deadline-first cases":"REPRODUCED all3 delayed-input deadline rescues")<<'\n';
  return 0;
}

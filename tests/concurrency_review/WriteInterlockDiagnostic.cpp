#include "Runtime.h"
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
int main(int argc,char** argv) {
  try {
    if(argc!=2&&argc!=3)throw std::runtime_error("usage: write_interlock_tests CONTROL_MODULE [--expect-preinterlock]");
    const bool expect_old=argc==3&&std::string(argv[2])=="--expect-preinterlock";
    double clock=100;std::vector<unsigned> wire;
    Runtime runtime([](const auto&,const auto&,double,bool){},[&](const uint8_t* bytes,std::size_t size) {
      mavlink_message_t m{},buffer{};mavlink_status_t parser{},status{};
      for(std::size_t i=0;i<size;++i)if(mavlink_frame_char_buffer(&buffer,&parser,bytes[i],&m,&status)==MAVLINK_FRAMING_OK)wire.push_back(m.msgid);
      return true;
    },true,false,[&]{return clock;});
    if(!runtime.load(argv[1]))throw std::runtime_error("load Copter control provider");
    runtime.configure("control_native_actions","false");runtime.configure("control_helm","false");
    mavlink_message_t m{};mavlink_msg_heartbeat_pack(1,1,&m,2,3,0,4,4);runtime.message(m,clock);
    mavlink_msg_extended_sys_state_pack(1,1,&m,0,MAV_LANDED_STATE_ON_GROUND);runtime.message(m,clock);
    mavlink_ekf_status_report_t ekf{};ekf.flags=51;mavlink_msg_ekf_status_report_encode(1,1,&m,&ekf);runtime.message(m,clock);
    runtime.tick(clock);wire.clear();
    ParameterRequest write;write.id="queued-before-arm";write.action=ParameterAction::Write;write.name="SIM_WIND_SPD";write.type=MAV_PARAM_TYPE_REAL32;write.value=.25f;write.expires=101;
    if(!runtime.parameter(write))throw std::runtime_error("initial disarmed-ground write not admitted");
    CommandRequest arm;arm.id="owned-arm";arm.command=MAV_CMD_COMPONENT_ARM_DISARM;arm.params[0]=1;arm.expires=101;
    if(!runtime.sendCommand(arm))throw std::runtime_error("arming request not admitted");
    runtime.tick(clock);
    unsigned sets=0;for(auto id:wire)sets+=id==MAVLINK_MSG_ID_PARAM_SET;
    std::cout<<"{\"observed_disarmed_ground\":true,\"set_queued_before_arm\":true,\"param_set_frames\":"<<sets<<",\"wire_ids\":[";
    for(std::size_t i=0;i<wire.size();++i)std::cout<<(i?",":"")<<wire[i];
    std::cout<<"]}\n";
    if(expect_old?sets!=1:sets!=0)throw std::runtime_error("parameter-write interlock expectation failed");
    std::cout<<"PASS "<<(expect_old?"preinterlock ordering reproduced":"owned command lease inhibited waiting write")<<'\n';
    return 0;
  } catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}

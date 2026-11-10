// Deterministic integration of the real feature modules; fake I/O never opens a socket.
#include "Runtime.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void require(bool value,const std::string& message) { if(!value)throw std::runtime_error(message); }
std::map<std::string,std::string> fields(const std::string& text) {
  std::map<std::string,std::string> result;
  for(std::size_t start=0;start<text.size();) {
    const auto end=text.find(',',start),split=text.find('=',start);
    if(split!=std::string::npos&&(end==std::string::npos||split<end))
      result[text.substr(start,split-start)]=text.substr(split+1,end==std::string::npos?end:end-split-1);
    if(end==std::string::npos)break;
    start=end+1;
  }
  return result;
}
struct Frame { mavlink_message_t message{};double at=0;bool accepted=false; };
struct Rig {
  double time=100;
  bool fail_parameter_io=false;
  double parameter_io_delay=0;
  std::vector<Frame> frames;
  std::vector<std::pair<std::string,std::string>> publications;
  std::map<std::string,std::string> latest;
  Runtime runtime;
  Rig(const std::vector<std::string>& modules):runtime(
    [this](const auto& key,const auto& text,double,bool numeric) {
      if(!numeric){latest[key]=text;publications.emplace_back(key,text);}
    },
    [this](const uint8_t* bytes,std::size_t size) {
      mavlink_message_t m{},buffer{};mavlink_status_t parse{},status{};bool accepted=true;
      for(std::size_t i=0;i<size;++i)if(mavlink_frame_char_buffer(&buffer,&parse,bytes[i],&m,&status)==MAVLINK_FRAMING_OK) {
        const bool parameter=m.msgid==20||m.msgid==21||m.msgid==23;
        accepted=!(parameter&&fail_parameter_io);frames.push_back({m,time,accepted});
        if(parameter)time+=parameter_io_delay;
      }
      return accepted;
    },true,false,[this]{return time;}) {
    // Load the coherent control provider once; its native/helm strategies share host authority.
    for(const auto& module:modules)require(runtime.load(module),"module load failed: "+module);
    fresh();
  }
  void text(const std::string& key,const std::string& value,const std::string& source="operator") {
    Mail mail;mail.key=key;mail.text=value;mail.source=source;
    mail.received_at=mail.published_at=time;runtime.mail(mail);
  }
  void scalar(const std::string& key,double value) {
    Mail mail;mail.key=key;mail.value=value;mail.numeric=true;mail.source="pHelmIvP";
    mail.received_at=mail.published_at=time;runtime.mail(mail);
  }
  void fresh(unsigned mode=4,bool armed=true,bool position=true) {
    const double effective=time-.005;mavlink_message_t m{};
    mavlink_msg_heartbeat_pack(1,1,&m,2,3,armed?128:0,mode,4);runtime.message(m,effective,time);
    if(position) {
      mavlink_msg_global_position_int_pack(1,1,&m,1000,423584560,-710875890,15000,5000,100,0,0,9000);runtime.message(m,effective,time);
    }
    mavlink_msg_extended_sys_state_pack(1,1,&m,0,armed?2:1);runtime.message(m,effective,time);
    mavlink_home_position_t home{};home.latitude=423584560;home.longitude=-710875890;home.altitude=10000;
    mavlink_msg_home_position_encode(1,1,&m,&home);runtime.message(m,effective,time);
    mavlink_ekf_status_report_t ekf{};ekf.flags=51;mavlink_msg_ekf_status_report_encode(1,1,&m,&ekf);runtime.message(m,effective,time);
  }
  void helmFields() {
    text("IVPHELM_STATE","DRIVE","pHelmIvP");
    scalar("DESIRED_HEADING",90);scalar("DESIRED_SPEED",1);scalar("DESIRED_ALTITUDE",5);
  }
  void active() {
    time=100.01;text("IVPHELM_ALLSTOP","clear","pHelmIvP");
    time=100.02;text("IVPHELM_STATE","DRIVE","pHelmIvP");
    time=100.03;text("MAVMOOS_NAV_CONTROL","acquire");
    time=100.04;fresh();helmFields();runtime.tick(time);
    require(latest["MAVMOOS_HELM_STATUS"].find("state=ACTIVE")!=std::string::npos,"helm did not activate");
  }
  void step(double at) { time=at;fresh();helmFields();runtime.tick(time); }
  void parameter(const std::string& id,const std::string& action="list",const std::string& extra="") {
    text("MAVMOOS_PARAM_REQUEST","id="+id+",action="+action+extra);
  }
  void value(unsigned index,unsigned count,const std::string& supplied="",double effective=-1) {
    const std::string name=supplied.empty()?"P"+std::to_string(index):supplied;
    mavlink_param_value_t v{};std::copy(name.begin(),name.end(),v.param_id);
    v.param_value=index==UINT16_MAX?1.f:static_cast<float>(index);v.param_type=MAV_PARAM_TYPE_REAL32;v.param_index=index;v.param_count=count;
    mavlink_message_t m{};mavlink_msg_param_value_encode(1,1,&m,&v);
    runtime.message(m,effective<0?time-.005:effective,time);
  }
  bool outcome(const std::string& id,const std::string& status) const {
    for(const auto& publication:publications)if(publication.first=="MAVMOOS_PARAM_RESULT"||publication.first=="MAVMOOS_COMMAND_RESULT") {
      const auto f=fields(publication.second);
      if(f.count("id")&&f.at("id")==id&&f.at("status")==status)return true;
    }
    return false;
  }
  unsigned count(unsigned id,bool accepted_only=false) const {
    return std::count_if(frames.begin(),frames.end(),[=](const auto& frame){return frame.message.msgid==id&&(!accepted_only||frame.accepted);});
  }
  unsigned navAccepted()const {return static_cast<unsigned>(std::stoul(fields(runtime.status()).at("nav_tx_accepted")));}
  std::vector<double> navTimes() const {
    std::vector<double> result;for(const auto& frame:frames)if(frame.message.msgid==84&&frame.accepted)result.push_back(frame.at);return result;
  }
};

std::vector<double> profile(const std::vector<std::string>& modules,bool transfer,bool drop_indices) {
  Rig rig(modules);rig.active();
  if(transfer){rig.time=100.05;rig.parameter("concurrent-list");}
  std::size_t inspected=rig.frames.size();std::vector<unsigned> responses;std::set<unsigned> dropped;
  // 1364 request/reply pairs at 50 ticks/s require at least 27.28 fake seconds.
  // These are deterministic scheduling observations, not wall-clock or FC QoS guarantees.
  for(unsigned step=0;step<1500;++step) {
    rig.time=100.06+step*.02;rig.fresh();rig.helmFields();
    for(unsigned index:responses)rig.value(index,1364);
    responses.clear();
    const auto before=rig.frames.size();rig.runtime.tick(rig.time);
    unsigned param_reads=0;bool parameter_seen=false;
    for(std::size_t i=before;i<rig.frames.size();++i) {
      const auto id=rig.frames[i].message.msgid;
      if(id==20||id==21||id==23){++param_reads;parameter_seen=true;}
      if(id==84)require(!parameter_seen,"parameter dispatch preceded NAV in the same tick");
    }
    require(param_reads<=1,"parameter lane emitted more than one frame per tick");
    for(;inspected<rig.frames.size();++inspected)if(rig.frames[inspected].message.msgid==20) {
      mavlink_param_request_read_t p{};mavlink_msg_param_request_read_decode(&rig.frames[inspected].message,&p);
      require(p.param_index>=0,"paced list sent a named rather than indexed READ");
      const auto index=static_cast<unsigned>(p.param_index);
      if(drop_indices&&(index==2||index==9||index==1024)&&dropped.insert(index).second)continue;
      responses.push_back(index);
    }
    const auto result=fields(rig.latest["MAVMOOS_PARAM_RESULT"]);
    if(result.count("outstanding"))require(std::stoul(result.at("outstanding"))<=4,"paced window exceeded four logical indices");
  }
  require(rig.navAccepted()>240&&rig.latest["MAVMOOS_HELM_STATUS"].find("state=ACTIVE")!=std::string::npos,"NAV lost under transfer");
  require(rig.count(23)==0&&rig.count(21)==0,"paced list emitted PARAM_SET or PARAM_REQUEST_LIST");
  if(transfer) {
    require(rig.outcome("concurrent-list","COMPLETED"),"indexed table never completed");
    require(rig.latest["MAVMOOS_PARAM_SNAPSHOT"].find("\"count\":1364")!=std::string::npos,"complete table snapshot missing");
    require(rig.count(20)==(drop_indices?1367u:1364u),"paced requests/retries not bounded to requested indices");
  }
  std::cout<<"METRIC profile="<<(transfer?(drop_indices?"paced-list-recovery":"paced-list"):"baseline")
           <<" fake_seconds=30 nav_accepted="<<rig.navAccepted()<<" list_requests="<<rig.count(21)<<" index_reads="<<rig.count(20)<<'\n';
  return rig.navTimes();
}
}

int main(int argc,char** argv) {
  try {
    require(argc==3,"usage: scheduling_tests CONTROL_MODULE PARAMETERS_MODULE");
    const std::vector<std::string> modules={argv[1],argv[2]};
    unsigned passed=0;
    auto test=[&](const char* name,const std::function<void()>& fn){fn();++passed;std::cout<<"PASS "<<name<<'\n';};
    test("paced full list and bounded recovery leave deterministic NAV schedule unchanged",[&]{
      const auto baseline=profile(modules,false,false);
      require(profile(modules,true,false)==baseline,"full parameter list changed NAV send times");
      require(profile(modules,true,true)==baseline,"missing-index recovery changed NAV send times");
    });
    test("named read uses actual protocol receipt and leaves NAV active",[&]{
      Rig r(modules);r.active();r.time=100.05;r.parameter("read","read",",name=FRAME_CLASS");r.step(100.06);
      r.time=100.08;r.fresh();r.value(UINT16_MAX,1364,"FRAME_CLASS",100.01);r.helmFields();r.runtime.tick(r.time);
      require(r.outcome("read","COMPLETED"),"conservative floor predating send prevented valid read");
      require(r.latest["MAVMOOS_HELM_STATUS"].find("state=ACTIVE")!=std::string::npos,"read disturbed helm");
    });
    test("parameter I/O failure does not become NAV retry or failure",[&]{
      Rig r(modules);r.active();r.fail_parameter_io=true;r.time=100.05;r.parameter("failed","read",",name=FRAME_CLASS");
      for(unsigned n=0;n<60;++n)r.step(100.06+n*.02);
      require(r.outcome("failed","FAILED")&&r.count(20)==1&&r.count(20,true)==0,"failed read retried or misreported");
      require(r.navAccepted()>9&&r.latest["MAVMOOS_HELM_STATUS"].find("state=ACTIVE")!=std::string::npos,"parameter I/O failure starved NAV");
    });
    test("full parameter lane rejects extra work while NAV continues",[&]{
      Rig r(modules);r.active();r.time=100.05;r.runtime.checkpoint(r.time);
      ParameterRequest request;request.name="FRAME_CLASS";request.expires=101;
      for(unsigned n=0;n<4;++n){request.id="other-"+std::to_string(n);require(r.runtime.parameter(request),"bounded fixture admission");}
      request.id="overflow";require(!r.runtime.parameter(request),"parameter queue exceeded four");
      r.parameter("busy-read","read",",name=FRAME_CLASS");
      for(unsigned n=0;n<30;++n)r.step(100.06+n*.02);
      require(r.outcome("busy-read","REJECTED")&&r.count(20)==4,"backpressure did not remain local to parameter operation");
      require(r.navAccepted()>4&&r.latest["MAVMOOS_HELM_STATUS"].find("state=ACTIVE")!=std::string::npos,"queue pressure inhibited NAV");
    });
    test("original parameter deadline expires while fresh NAV remains independent",[&]{
      Rig r(modules);r.active();r.runtime.configure("parameter_list_timeout","1");r.time=100.05;r.parameter("silent-list");
      for(unsigned n=0;n<100;++n)r.step(100.06+n*.02);
      require(r.outcome("silent-list","INCOMPLETE")&&!r.outcome("silent-list","COMPLETED"),"silent list exceeded original deadline");
      for(const auto& f:r.frames)if(f.message.msgid==20||f.message.msgid==21)require(f.at<101.05,"parameter request sent after original expiry");
      // The one-second original operation deadline retires this seed before
      // its one-second response timeout can admit any retry.
      require(r.count(21)==0&&r.count(20)==1&&r.navAccepted()>15,"unbounded recovery or NAV starvation");
    });
    test("in-flight parameter SET is denied at module and host",[&]{
      Rig r(modules);r.active();r.time=100.05;r.parameter("flight-write","write",",name=SIM_WIND_SPD,value=0.25,type=REAL32");r.step(100.06);
      ParameterRequest p;p.id="direct-flight-write";p.name="SIM_WIND_SPD";p.action=ParameterAction::Write;p.value=.25f;p.expires=101;
      require(!r.runtime.parameter(p),"host admitted airborne SET");
      require(r.outcome("flight-write","REJECTED")&&r.count(23)==0,"module emitted airborne SET");
      r.step(100.2);require(r.navAccepted()>1,"rejected write inhibited NAV");
    });
    test("pose-only continuity loss preserves independent parameter read",[&]{
      Rig r(modules);r.active();r.time=100.05;r.parameter("pose-read","read",",name=FRAME_CLASS");r.step(100.06);
      const auto nav=r.navAccepted();r.time=100.7;r.runtime.checkpoint(r.time);r.fresh();r.helmFields();r.value(UINT16_MAX,1364,"FRAME_CLASS");r.runtime.tick(r.time);
      require(r.outcome("pose-read","COMPLETED"),"pose loss cancelled independent read");
      require(r.navAccepted()==nav&&r.latest["MAVMOOS_HELM_STATUS"].find("state=LATCHED")!=std::string::npos,"returning input auto-restored NAV");
    });
    test("heartbeat session loss cancels table and queued work",[&]{
      Rig r(modules);r.active();r.time=100.05;r.parameter("lost-list");r.step(100.06);r.time=102.2;r.runtime.checkpoint(r.time);
      r.fresh();r.value(0,1);r.helmFields();r.runtime.tick(r.time);
      require(r.outcome("lost-list","CANCELLED")&&!r.outcome("lost-list","COMPLETED"),"old session completed a table");
      require(fields(r.runtime.status()).at("permission")=="false", "heartbeat recovery restored old permission");
    });
    test("explicit HOLD retires NAV while parameter table can finish",[&]{
      Rig r(modules);r.active();r.time=100.05;r.parameter("hold-list");r.step(100.06);
      r.time=100.08;r.value(0,4);r.time=100.2;r.fresh();r.helmFields();r.text("MAVMOOS_COMMAND","id=hold,action=hold");
      const auto nav=r.navAccepted();r.runtime.tick(r.time);require(r.navAccepted()==nav,"HOLD allowed another NAV target");
      r.time=100.22;r.fresh(5);mavlink_command_ack_t ack{};ack.command=MAV_CMD_DO_SET_MODE;ack.result=MAV_RESULT_ACCEPTED;ack.target_system=245;ack.target_component=191;
      mavlink_message_t m{};mavlink_msg_command_ack_encode(1,1,&m,&ack);r.runtime.message(m,r.time-.005,r.time);
      // Reply to actual accepted index requests. A paced observation request
      // can share these ticks, so guessed per-tick replies can predate a read.
      std::size_t inspected=0;std::set<unsigned> replied={0};
      for(unsigned step=0;step<30&&!r.outcome("hold-list","COMPLETED");++step) {
        r.time+=.02;r.fresh(5);r.runtime.tick(r.time);
        while(inspected<r.frames.size()) {
          const auto frame=r.frames[inspected++];if(frame.message.msgid!=20||!frame.accepted)continue;
          mavlink_param_request_read_t read{};mavlink_msg_param_request_read_decode(&frame.message,&read);
          if(read.param_index>=0&&read.param_index<4&&replied.insert(read.param_index).second) {
            r.time+=.006;r.value(read.param_index,4);
          }
        }
      }
      require(r.outcome("hold","COMPLETED")&&r.outcome("hold-list","COMPLETED"),"hold/parameter outcomes interfered");
      require(r.navAccepted()==nav&&fields(r.runtime.status()).at("permission")=="false","HOLD automatically recaptured NAV");
    });
    test("slow parameter send return cannot conceal an observation gap",[&]{
      Rig r(modules);r.active();r.time=100.05;r.parameter("slow-read","read",",name=FRAME_CLASS");r.parameter_io_delay=.7;r.step(100.06);
      require(r.time>100.75,"slow I/O fixture did not advance fake clock");const auto nav=r.navAccepted();
      r.parameter_io_delay=0;r.time+=.01;r.fresh();r.helmFields();r.value(UINT16_MAX,1364,"FRAME_CLASS");r.runtime.tick(r.time);
      require(r.outcome("slow-read","COMPLETED")&&r.navAccepted()==nav,"slow protocol send concealed gap or cancelled independent read");
      require(r.latest["MAVMOOS_HELM_STATUS"].find("state=LATCHED")!=std::string::npos,"slow I/O gap auto-resumed NAV");
    });
    test("ordinary command origin is independent of acquire-looking ID",[&]{
      Rig r(modules);r.active();r.time=100.05;r.parameter("origin-read","read",",name=FRAME_CLASS");r.step(100.06);
      CommandRequest command;command.id="acquire-user-visible-id";command.command=MAV_CMD_DO_SET_MODE;
      command.params[0]=1;command.params[1]=5;command.expires=101;
      require(r.runtime.sendCommand(command),"ordinary command fixture admission failed");
      r.runtime.revokeNavigation("operator_release");r.time=100.2;r.fresh();r.runtime.tick(r.time);
      unsigned loiter=0;
      for(const auto& frame:r.frames)if(frame.message.msgid==MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t wire{};mavlink_msg_command_long_decode(&frame.message,&wire);
        if(wire.command==MAV_CMD_DO_SET_MODE&&wire.param2==5)++loiter;
      }
      require(loiter==1,"ordinary command was cancelled by its ID prefix");
      r.time=100.21;r.value(UINT16_MAX,1364,"FRAME_CLASS");
      require(r.outcome("origin-read","COMPLETED"),"ordinary command barrier cancelled independent parameter read");
    });
    test("parameter reply traffic cannot renew expired helm inputs",[&]{
      Rig r(modules);r.active();r.time=100.05;r.parameter("long-list");
      unsigned stopped=0;
      for(unsigned n=0;n<150;++n) {
        r.time=100.06+n*.02;r.fresh();
        if(n)r.value(n-1,1364);
        r.runtime.tick(r.time);
        if(n==110)stopped=r.navAccepted();
      }
      require(r.latest["MAVMOOS_HELM_STATUS"].find("state=LATCHED")!=std::string::npos,"parameter traffic refreshed helm eligibility");
      require(r.navAccepted()==stopped&&!r.outcome("long-list","COMPLETED")&&!r.outcome("long-list","CANCELLED"),"NAV/input expiry and parameter operation became coupled");
      r.time+=.01;r.parameter("busy-after-helm-expiry","read",",name=FRAME_CLASS");
      require(r.outcome("busy-after-helm-expiry","REJECTED"),"independent table unexpectedly stopped on helm expiry");
    });
    std::cout<<"PASS "<<passed<<" scheduling groups; all sends use fake I/O, not a vehicle or socket\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}

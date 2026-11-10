#include "../observation_load/FinishBoundary.h"
#define main observation_load_unused_program_main
#include "../observation_load/ObservationLoadProbe.cpp"
#undef main
#include <vector>

namespace {
void require(bool value,const std::string& why){if(!value)throw std::runtime_error(why);}
struct Schedule {
  double at=0,ack_at=100,pump_delay=0,read_delay=0,coordinator_boundary=-1;
  unsigned pumps=0,reads=0;
  std::string token="opaque-finish-token\n",response=token;
  bool pump_error=false,read_error=false;
  std::vector<double> pauses,guided;
  auto wait(double deadline=3) {
    return mavmoos::test::waitForFinishAck(token,deadline,[&]() -> std::optional<std::string> {
      ++reads;if(read_error)throw std::runtime_error("inert read failure");at+=read_delay;
      if(at<ack_at)return std::nullopt;
      coordinator_boundary=at;return response;
    },[&]{++pumps;if(pump_error)throw std::runtime_error("inert pump failure");guided.push_back(at);at+=pump_delay;},
    [&]{return at;},[&](double seconds){require(seconds>0&&seconds<=.0100000001,"unbounded pause");pauses.push_back(seconds);at+=seconds;});
  }
};
void immediate_ack_still_pumps(){Schedule c;c.ack_at=0;require(c.wait().value_or(-1)==0&&c.pumps==1,"available ack bypassed pump");}
void delayed_coordinator_keeps_guided_until_boundary(){
  Schedule c;c.ack_at=2.4;const auto ack=c.wait();require(ack&&c.pumps>200,"delayed coordinator was not pumped");
  require(c.coordinator_boundary==*ack,"coordinator boundary was fabricated");
  for(const auto at:c.guided)require(at<=c.coordinator_boundary,"Guided sample incorrectly crossed acknowledgement");
  const double permitted_land=*ack+.1;require(permitted_land>c.coordinator_boundary,"landing allowed before recorded boundary");
}
void missing_ack(){Schedule c;require(!c.wait()&&std::abs(c.at-3)<1e-9&&c.pumps<303,"missing ack renewed deadline");}
void foreign_ack(){Schedule c;c.ack_at=.1;c.response="foreign\n";bool rejected=false;try{c.wait();}catch(const std::runtime_error& e){rejected=std::string(e.what()).find("token mismatch")!=std::string::npos;}require(rejected,"foreign ack accepted");}
void pump_cannot_accept_after_deadline(){Schedule c;c.ack_at=0;c.pump_delay=3.01;require(!c.wait()&&c.reads==0,"late pump accepted ack");}
void read_cannot_accept_after_deadline(){Schedule c;c.ack_at=0;c.read_delay=3.01;require(!c.wait(),"late read accepted ack");}
void exact_deadline_is_expired(){Schedule c;c.ack_at=3;require(!c.wait(),"ack at deadline accepted");}
void completed_timeout_cannot_resume(){Schedule c;c.ack_at=3.1;const auto result=c.wait();c.at=3.2;require(!result&&c.coordinator_boundary<0,"later ack changed completed timeout");}
void pump_failure_propagates(){Schedule c;c.pump_error=true;bool rejected=false;try{c.wait();}catch(const std::runtime_error& e){rejected=std::string(e.what())=="inert pump failure";}require(rejected,"pump failure swallowed");}
void read_failure_propagates(){Schedule c;c.read_error=true;bool rejected=false;try{c.wait();}catch(const std::runtime_error& e){rejected=std::string(e.what())=="inert read failure";}require(rejected,"read failure swallowed");}
void last_pause_is_clipped(){Schedule c;require(!c.wait(.023)&&c.pauses.size()==3&&std::abs(c.pauses.back()-.003)<1e-9,"last pause renewed deadline");}
void actual_probe_wrappers(const std::filesystem::path& root){
  require(!std::filesystem::exists(root),"refusing to reuse wrapper fixture");std::filesystem::create_directories(root);
  Probe p;p.client.SetQuiet(true);p.control=root;require(!p.client.IsConnected(),"inert client connected");
  std::ostringstream trace;auto* previous=std::cout.rdbuf(trace.rdbuf());
  try {
    std::ofstream(root/"load.finished.ack")<<"stale\n";bool rejected=false;
    try{p.finishMarker();}catch(const std::runtime_error& e){rejected=std::string(e.what()).find("preexisting")!=std::string::npos;}
    require(rejected&&!std::filesystem::exists(root/"load.finished"),"preexisting ack allowed phase publication");
    std::filesystem::remove(root/"load.finished.ack");const auto token=p.finishMarker();
    std::ifstream input(root/"load.finished");const std::string stored((std::istreambuf_iterator<char>(input)),{});
    require(stored==token&&!token.empty(),"actual marker token differs from published bytes");
    std::ofstream(root/"load.finished.ack")<<token;p.monitor=true;rejected=false;
    try{p.finishAck(token,mono());}catch(const std::runtime_error& e){rejected=std::string(e.what()).find("MOOS disconnected")!=std::string::npos;}
    require(rejected&&trace.str().find("finish_boundary_ack")==std::string::npos,"disconnected wrapper accepted cached acknowledgement");
    require(trace.str().find("\"type\":\"request\"")==std::string::npos,"inert wrapper emitted effect request");
  }catch(...){std::cout.rdbuf(previous);throw;}
  std::cout.rdbuf(previous);std::filesystem::remove_all(root);
}
}
int main(int argc,char** argv){
  unsigned completed=0;
  try {
    require(argc==2,"usage: observation_finish_tests LOCAL_FIXTURE_DIRECTORY");
    for(const auto test:{immediate_ack_still_pumps,delayed_coordinator_keeps_guided_until_boundary,missing_ack,foreign_ack,
        pump_cannot_accept_after_deadline,read_cannot_accept_after_deadline,exact_deadline_is_expired,
        completed_timeout_cannot_resume,pump_failure_propagates,read_failure_propagates,last_pause_is_clipped}){test();++completed;}
    actual_probe_wrappers(argv[1]);++completed;
    std::cout<<"{\"result\":\"PASS\",\"controls\":"<<completed<<",\"scope\":\"deterministic finish scheduling and actual inert Probe wrapper; no MOOS Run, socket, native or vehicle\"}\n";return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';std::cout<<"{\"result\":\"FAIL\",\"completed\":"<<completed<<"}\n";return 1;}
}

#include "MarkerWait.h"
#define main fault_probe_unused_program_main
#include "../faults/FaultProbe.cpp"
#undef main
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void require(bool value,const std::string& why) {if(!value)throw std::runtime_error(why);}
bool near(double a,double b) {return std::abs(a-b)<1e-9;}
struct Clock {
  double at=0,ack=100,oversleep=0;
  unsigned samples=0;
  std::vector<double> pauses;
  bool ready(){++samples;return at>=ack;}
  void pause(double seconds){require(seconds>0&&seconds<=.0200000001,"pause exceeded its bounded remaining time");pauses.push_back(seconds);at+=seconds;if(at>=3-1e-9)at+=oversleep;}
  bool wait(double timeout){return mavmoos::test::waitForMarker([&]{return ready();},timeout,[&]{return at;},[&](double seconds){pause(seconds);});}
};
void initial_marker() {
  Clock c;c.ack=0;require(c.wait(3),"present marker missed");
  require(c.pauses.empty()&&c.samples==1&&c.at==0,"initial marker unnecessarily waited");
}
void ordinary_ack() {
  Clock c;c.ack=.069;require(c.wait(3)&&c.at>=.069&&c.at<.1,"ordinary coordinator ACK missed");
}
void timely_edge_ack() {
  Clock c;c.ack=2.983782114;require(c.wait(3),"ACK before deadline missed by final sample");
  require(near(c.at,3)&&c.samples<=152,"deadline renewed or unbounded polling");
}
void legacy_ordering_misses_edge() {
  Clock c;c.ack=2.983782114;
  // Deterministic model of the old wait ordering: its loop condition exits
  // before sampling the marker after the final twenty-millisecond pause.
  bool observed=false;const auto deadline=c.at+3;
  while(c.at<deadline){if(c.ready()){observed=true;break;}c.pause(.02);}
  require(!observed&&c.ready(),"legacy edge counterexample did not reproduce");
}
void missing_ack() {
  Clock c;require(!c.wait(3),"missing ACK accepted");
  require(near(c.at,3)&&c.samples<=152,"missing ACK extended original deadline");
}
void arrives_only_after_final_sample() {
  Clock c;c.ack=3.1;const bool result=c.wait(3);
  require(!result&&near(c.at,3),"late ACK accepted before a final absence decision");
  c.at=3.1;require(c.ready()&&!result,"later marker changed a completed wait result");
}
void clipped_last_pause() {
  Clock c;require(!c.wait(.053),"missing short-window ACK accepted");
  require(c.pauses.size()==3&&near(c.pauses.back(),.013)&&near(c.at,.053),"last pause overshot remaining deadline by design");
}
void oversleep_scope_is_explicit() {
  Clock c;c.ack=3.002;c.oversleep=.007;
  require(c.wait(3)&&near(c.at,3.007),"final sample semantics changed under simulated OS oversleep");
  // This is deliberately accepted at the final sample: there is no mapping
  // from producer/Python/file creation time to the probe's steady clock.
}
void disconnected_stock_client_wrapper(const std::filesystem::path& root) {
  // Compile and call the actual Probe wrapper, not a second implementation.
  // A stock client is constructed but Run() is NEVER called: no network/thread
  // session, database, app or vehicle exists in this test.
  Probe p;p.client.SetQuiet(true);require(!p.client.IsConnected(),"inert client unexpectedly connected");
  p.control=root/"inert-control";
  require(!std::filesystem::exists(p.control),"refusing to reuse an old wrapper fixture");
  std::filesystem::create_directories(p.control);
  std::ofstream(p.control/"injected")<<"inert test marker\n";
  p.connected=true;p.registered=true;p.runtime_at=1;p.runtime["permission"]="false";
  std::ostringstream trace;auto* previous=std::cout.rdbuf(trace.rdbuf());
  try {p.waitMarker("injected",3,"inert marker");}
  catch(...){std::cout.rdbuf(previous);throw;}
  std::cout.rdbuf(previous);
  require(p.connected&&p.registered&&p.runtime_at==1&&p.runtime.at("permission")=="false",
          "marker wrapper pumped or changed cached MOOS/vehicle state");
  require(trace.str().find("coordinator_ack")!=std::string::npos,"marker observation diagnostic absent");
  bool rejected=false;
  try {p.waitMarker("missing",0,"missing inert marker");}
  catch(const std::runtime_error& error){rejected=std::string(error.what()).find("marker=missing")!=std::string::npos;}
  require(rejected&&p.connected&&p.registered&&p.runtime_at==1,"missing marker used or changed MOOS state");
  std::filesystem::remove_all(p.control);
}
}
int main(int argc,char** argv) {
  unsigned completed=0;
  try {
    require(argc==2,"usage: marker_wait_tests LOCAL_FIXTURE_DIRECTORY");
    for(const auto test:{initial_marker,ordinary_ack,timely_edge_ack,legacy_ordering_misses_edge,missing_ack,
        arrives_only_after_final_sample,clipped_last_pause,oversleep_scope_is_explicit}) {
      test();++completed;
    }
    disconnected_stock_client_wrapper(argv[1]);++completed;
    std::cout<<"{\"result\":\"PASS\",\"controls\":"<<completed
      <<",\"scope\":\"deterministic scheduling and inert stock-client wrapper; no MOOS Run/process, socket, native simulator or vehicle control\"}\n";
    return 0;
  } catch(const std::exception& error) {
    std::cout<<"{\"result\":\"FAIL\",\"controls_completed\":"<<completed<<"}\n";
    std::cerr<<error.what()<<'\n';return 1;
  }
}

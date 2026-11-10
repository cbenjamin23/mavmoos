// Actual private Probe boundary wrapper, unconnected MOOS client, files only.
#define main mavmoos_copter_scenario_probe_main
#include "CopterScenarioProbe.cpp"
#undef main
#include <unistd.h>

namespace {
void require(bool condition,const char* why){if(!condition)throw std::runtime_error(why);}
struct Folder {
  std::filesystem::path path;
  explicit Folder(unsigned n):path(std::filesystem::temp_directory_path()/("mavmoos-hold-boundary-"+std::to_string(getpid())+"-"+std::to_string(n))){
    if(!std::filesystem::create_directory(path))throw std::runtime_error("fresh inert folder required");
  }
  ~Folder(){std::filesystem::remove_all(path);}
};
struct Writer {
  std::thread worker;
  Writer(const std::filesystem::path& path,const std::string& text,double delay):worker([=]{
    std::this_thread::sleep_for(std::chrono::duration<double>(delay));
    const auto temporary=path.string()+".inert.tmp";{std::ofstream file(temporary);file<<text;}
    std::filesystem::rename(temporary,path);
  }){}
  ~Writer(){worker.join();}
};
}
int main()
{
  unsigned passed=0;
  try {
    const std::string key="cycle1.hold_helm.started";
    const auto test=[&](const char* name,const std::function<void(Probe&)>& run){
      Folder folder(++passed);Probe probe;probe.control=folder.path;probe.runtime["nav_tx_accepted"]="9";
      run(probe);std::cout<<"PASS "<<name<<'\n';
    };
    test("matching delayed ack pumps before success",[&](Probe& p){const auto token=p.hold_marker(key);unsigned pumped=0;Writer w(p.control/(key+".ack"),token,.02);p.hold_ack(key,token,mono(),mono()+.15,[&]{++pumped;});require(pumped>=2,"wait did not pump while ack delayed");});
    test("ready ack still pumps original state guard",[&](Probe& p){const auto token=p.hold_marker(key);std::ofstream(p.control/(key+".ack"))<<token;unsigned pumped=0;p.hold_ack(key,token,mono(),mono()+.1,[&]{++pumped;});require(pumped>0,"ready ack bypassed pump/guard");});
    test("preexisting ack rejected before marker publication",[&](Probe& p){std::ofstream(p.control/(key+".ack"))<<"old";bool rejected=false;try{p.hold_marker(key);}catch(const std::runtime_error&){rejected=true;}require(rejected&&!std::filesystem::exists(p.control/key),"preexisting ack accepted");});
    test("mismatched token rejected",[&](Probe& p){const auto token=p.hold_marker(key);std::ofstream(p.control/(key+".ack"))<<"foreign";bool rejected=false;try{p.hold_ack(key,token,mono(),mono()+.1,[]{});}catch(const std::runtime_error&){rejected=true;}require(rejected,"foreign acknowledgement accepted");});
    test("missing ack ends at original fixed deadline",[&](Probe& p){const auto token=p.hold_marker(key);const auto began=mono();bool rejected=false;try{p.hold_ack(key,token,began,began+.04,[]{});}catch(const std::runtime_error&){rejected=true;}require(rejected&&mono()-began<.25,"missing ack wait unbounded");});
    test("ack arriving after expiry cannot succeed",[&](Probe& p){const auto token=p.hold_marker(key);Writer w(p.control/(key+".ack"),token,.07);bool rejected=false;try{p.hold_ack(key,token,mono(),mono()+.03,[]{});}catch(const std::runtime_error&){rejected=true;}require(rejected,"late ack accepted");});
    test("slow pump past deadline cannot accept queued ack",[&](Probe& p){const auto token=p.hold_marker(key);std::ofstream(p.control/(key+".ack"))<<token;bool rejected=false;try{p.hold_ack(key,token,mono(),mono()+.02,[]{std::this_thread::sleep_for(std::chrono::milliseconds(35));});}catch(const std::runtime_error&){rejected=true;}require(rejected,"slow pump renewed original deadline");});
    test("operation failure remains fatal during ack wait",[&](Probe& p){const auto token=p.hold_marker(key);std::ofstream(p.control/(key+".ack"))<<token;p.terminal["hold"]="TIMED_OUT";bool rejected=false;try{p.hold_ack(key,token,mono(),mono()+.1,[&]{p.check("hold");});}catch(const std::runtime_error&){rejected=true;}require(rejected,"expired operation bypassed by ack");});
    test("oversized ack rejected",[&](Probe& p){const auto token=p.hold_marker(key);std::ofstream(p.control/(key+".ack"))<<std::string(129,'x');bool rejected=false;try{p.hold_ack(key,token,mono(),mono()+.1,[]{});}catch(const std::runtime_error&){rejected=true;}require(rejected,"oversized ack accepted");});
    std::cout<<"{\"result\":\"PASS\",\"controls\":"<<passed<<",\"scope\":\"actual Probe boundary wrapper; no MOOS Run, Notify, socket or native flight\"}\n";return 0;
  } catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}

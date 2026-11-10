// Actual Runtime, startup-loaded provider, fake owner clock and packet handoff.
#include "../protocol_faults/ProtocolSupport.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sys/stat.h>
#include <unistd.h>

using namespace protocol_faults;
namespace fs=std::filesystem;
namespace {
constexpr char request_key[]="MAVMOOS_FLIGHT_LOG_REQUEST";
constexpr char result_key[]="MAVMOOS_FLIGHT_LOG_RESULT";
constexpr char catalog_key[]="MAVMOOS_FLIGHT_LOG_CATALOG";

struct PrivateDirectory {
  fs::path path;
  PrivateDirectory()
  {
    auto pattern=(fs::canonical(fs::temp_directory_path())/"mavmoos-log-test-XXXXXX").string();
    const char* made=mkdtemp(pattern.data());
    require(made!=nullptr,"private temporary directory");
    path=made;
  }
  ~PrivateDirectory() { std::error_code error; fs::remove_all(path,error); }
};
bool stringField(const std::string& text,const char* key,const std::string& value)
{
  return text.find(std::string("\"")+key+"\":"+quote(value))!=std::string::npos;
}
std::vector<uint8_t> fixture(unsigned size)
{
  std::vector<uint8_t> data(size);
  for(unsigned i=0;i<size;++i) data[i]=i%90>=87?0:uint8_t(i*37+9);
  return data;
}
bool hasNoPartial(const fs::path& directory)
{
  return std::none_of(fs::directory_iterator(directory),fs::directory_iterator(),[](const auto& entry) {
    return entry.path().extension()==".partial";
  });
}

struct Rig {
  PrivateDirectory directory;
  double now=100;
  bool airborne=false,deny_end=false;
  std::vector<Publication> publications;
  std::vector<Frame> frames;
  mavlink_status_t peer{};
  Runtime host;

  Rig(const char* plugin,unsigned timeout=8,unsigned max_bytes=4096):host(
    [&](const auto& key,const auto& text,double,bool numeric) {
      if(!numeric && (key==result_key || key==catalog_key || key=="MAVMOOS_FLIGHT_LOG_TX_RESULT"))
        publications.push_back({key,text,now});
    },[](const uint8_t*,std::size_t) { throw std::runtime_error("unbounded sender used"); return false; },
    true,false,[&] { return now; })
  {
    require(host.load(plugin,"flight_logs"),"load actual flight_logs module");
    host.configureModules({{"flight_logs",{{"directory",directory.path.string()},
      {"timeout",std::to_string(timeout)},{"max_bytes",std::to_string(max_bytes)}}}});
    host.setBoundedSend([&](const uint8_t* bytes,std::size_t size,double expires,const Runtime::SendEligibility& live) {
      const auto frame=decode(bytes,size);
      const bool allowed=live(now) && now<expires && !(deny_end && frame.msgid==MAVLINK_MSG_ID_LOG_REQUEST_END);
      if(frame.msgid>=MAVLINK_MSG_ID_LOG_REQUEST_LIST && frame.msgid<=MAVLINK_MSG_ID_LOG_REQUEST_END)
        frames.push_back({frame,now,expires,allowed});
      require(frame.msgid!=MAVLINK_MSG_ID_LOG_ERASE,"erase unexpectedly encoded");
      return allowed;
    });
    tick();
  }
  void fresh()
  {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack_status(1,1,&peer,&message,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,
      airborne?MAV_MODE_FLAG_SAFETY_ARMED:0,0,MAV_STATE_STANDBY);
    host.message(message,now);
    mavlink_msg_extended_sys_state_pack_status(1,1,&peer,&message,0,
      airborne?MAV_LANDED_STATE_IN_AIR:MAV_LANDED_STATE_ON_GROUND);
    host.message(message,now);
  }
  void tick(double delta=.02) { now+=delta; fresh(); host.tick(now); }
  void until(const std::function<bool()>& done,double seconds=3)
  {
    const auto end=now+seconds;
    while(!done() && now<end) tick();
    require(done(),"fixture waited beyond bound");
  }
  void mail(const std::string& request,double age=0)
  {
    now+=.001;
    fresh();
    Mail mail;mail.key=request_key;mail.text=request;mail.source="fixture";mail.published_at=now-age;mail.received_at=now;
    host.mail(mail);
  }
  void entry(unsigned id,unsigned total,unsigned last,unsigned bytes,unsigned utc=15,uint8_t system=1,uint8_t component=1)
  {
    mavlink_message_t message{};
    mavlink_msg_log_entry_pack_status(system,component,&peer,&message,id,total,last,utc,bytes);
    receive(message);
  }
  void chunk(unsigned id,unsigned offset,unsigned count,const std::vector<uint8_t>& bytes,
             uint8_t system=1,uint8_t component=1)
  {
    uint8_t payload[MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN]{};
    if(offset<bytes.size())
      std::copy_n(bytes.begin()+offset,std::min<std::size_t>(sizeof(payload),bytes.size()-offset),payload);
    mavlink_message_t message{};
    mavlink_msg_log_data_pack_status(system,component,&peer,&message,id,offset,count,payload);
    receive(message);
  }
  void receive(const mavlink_message_t& message)
  {
    uint8_t wire_bytes[MAVLINK_MAX_PACKET_LEN]{};
    const auto size=mavlink_msg_to_send_buffer(wire_bytes,&message);
    now+=.001;
    host.message(decode(wire_bytes,size),now);
  }
  std::string result(const std::string& id) const
  {
    for(auto at=publications.rbegin();at!=publications.rend();++at)
      if(at->key==result_key && stringField(at->text,"id",id)) return at->text;
    return {};
  }
  bool status(const std::string& id,const std::string& name) const { return stringField(result(id),"status",name); }
  void expect(const std::string& id,const std::string& status_name,const std::string& reason="")
  {
    until([&] { return status(id,status_name); });
    if(!reason.empty()) require(stringField(result(id),"reason",reason),result(id));
  }
  std::size_t count(uint32_t msgid,bool accepted=true) const
  {
    return std::count_if(frames.begin(),frames.end(),[&](const auto& frame) {
      return frame.message.msgid==msgid && (!accepted || frame.accepted);
    });
  }
  mavlink_message_t next(uint32_t id,std::size_t index)
  {
    until([&] { return count(id)>index; });
    for(const auto& frame:frames)
      if(frame.accepted && frame.message.msgid==id && index--==0) return frame.message;
    throw std::runtime_error("unreachable frame");
  }
  void listWire(std::size_t index,unsigned first,unsigned last)
  {
    const auto message=next(MAVLINK_MSG_ID_LOG_REQUEST_LIST,index);
    mavlink_log_request_list_t request{};mavlink_msg_log_request_list_decode(&message,&request);
    require(message.sysid==245 && message.compid==191 && request.target_system==1 && request.target_component==1 &&
      request.start==first && request.end==last,"wrong list wire scope");
  }
  void dataWire(std::size_t index,unsigned offset,unsigned bytes)
  {
    const auto message=next(MAVLINK_MSG_ID_LOG_REQUEST_DATA,index);
    mavlink_log_request_data_t request{};mavlink_msg_log_request_data_decode(&message,&request);
    require(message.sysid==245 && message.compid==191 && request.target_system==1 && request.target_component==1 &&
      request.id==1 && request.ofs==offset && request.count==bytes,"wrong data wire window");
  }
  void seed(unsigned size)
  {
    mail("schema=1,id=catalog,action=list");
    listWire(0,0,65535);
    entry(1,1,1,size);
    expect("catalog","complete","catalog_verified");
  }
  void download(unsigned size)
  {
    mail("schema=1,id=download,action=download,catalog=catalog,log_id=1");
    listWire(1,1,1);
    entry(1,1,1,size);
    dataWire(0,0,size);
  }
  void trace(const char* name) const
  {
    std::cout<<"{\"case\":"<<quote(name)<<",\"scope\":\"actual_runtime_dso_fake_clock_and_io\",\"frames\":[";
    bool comma=false;
    for(const auto& frame:frames) {
      if(comma) std::cout<<',';
      comma=true;std::cout<<frameJson(frame);
    }
    std::cout<<"],\"publications\":[";
    comma=false;
    for(const auto& publication:publications) {
      if(comma) std::cout<<',';
      comma=true;std::cout<<publicationJson(publication);
    }
    std::cout<<"]}\n";
  }
};

void reorderedRepairAndFile(Rig& r)
{
  constexpr unsigned size=3000;
  const auto bytes=fixture(size);
  r.mail("schema=1,id=catalog,action=list");
  r.entry(1,1,1,42); // Before local request handoff.
  r.listWire(0,0,65535);
  r.entry(1,1,1,42,15,2,1);
  r.entry(1,1,1,42,15,1,2);
  r.entry(2,2,2,103);
  r.entry(2,2,2,103);
  r.listWire(1,1,1);
  r.entry(1,2,2,size); // Prior request range, delivered late.
  require(r.status("catalog","queued"),"retired list response completed inventory");
  r.entry(1,2,1,size);
  r.expect("catalog","complete","catalog_verified");
  r.mail("schema=1,id=download,action=download,catalog=catalog,log_id=1");
  r.listWire(2,1,1);
  r.entry(1,2,1,size);
  r.dataWire(0,0,2880);
  r.chunk(1,2880,90,bytes); // Outside current window.
  r.chunk(1,90,1,bytes,1,7);
  r.chunk(2,90,1,bytes);
  for(unsigned index=32;index>0;--index)
    if(index!=2) r.chunk(1,(index-1)*90,90,bytes);
  r.chunk(1,0,90,bytes);
  r.dataWire(1,90,90);
  r.chunk(1,90,90,bytes);
  r.dataWire(2,2880,120);
  r.chunk(1,2970,30,bytes);
  r.chunk(1,2880,90,bytes);
  r.dataWire(3,size,1);
  r.chunk(1,size,0,bytes);
  r.listWire(3,1,1);
  r.entry(1,2,1,size);
  require(fs::is_empty(r.directory.path),"final file released before local END");
  r.expect("download","complete","byte_coverage_and_catalog_verified");
  require(stringField(r.result("download"),"cleanup","local_end_accepted_unacknowledged"),"wrong cleanup witness");
  const auto file=r.directory.path/"log-1-download.bin";
  std::ifstream in(file,std::ios::binary);
  const std::vector<uint8_t> saved{std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>()};
  require(saved==bytes,"released file does not match received fixture");
  struct stat info{};
  require(stat(file.c_str(),&info)==0 && (info.st_mode&077)==0 && info.st_nlink==1 &&
    hasNoPartial(r.directory.path) && r.count(MAVLINK_MSG_ID_LOG_REQUEST_END)==2,"file ownership or END bounds");
}
}

int main(int argc,char** argv)
{
  try {
    require(argc==2,"usage: flight_log_protocol_tests FLIGHT_LOGS_MODULE");
    const auto test=[&](const char* name,const auto& body,unsigned timeout=8) {
      Rig r(argv[1],timeout);
      try { body(r); } catch(...) { r.trace(name); throw; }
      r.trace(name);
    };
    test("reordered_repair_and_byte_exact_no_early_release",reorderedRepairAndFile);
    test("empty_zero_row_is_complete",[](Rig& r) {
      r.mail("schema=1,id=empty,action=list");r.listWire(0,0,65535);
      r.entry(0,0,0,0,0);r.expect("empty","complete","empty_catalog");
      r.mail("schema=1,id=no-item,action=download,catalog=empty,log_id=1");
      r.expect("no-item","rejected","missing_empty_or_oversized_log");
      require(r.count(MAVLINK_MSG_ID_LOG_REQUEST_DATA)==0,"download sent for empty log inventory");
    });
    test("short_chunk_never_releases_partial",[](Rig& r) {
      r.seed(100);r.download(100);r.chunk(1,0,90,fixture(100));r.chunk(1,90,0,fixture(100));
      r.expect("download","incomplete","invalid_or_short_log_chunk");
      require(fs::is_empty(r.directory.path) && r.count(MAVLINK_MSG_ID_LOG_REQUEST_END)==2,"short download leaked file or remote cleanup");
    });
    test("conflicting_duplicate_rejected",[](Rig& r) {
      r.seed(100);r.download(100);auto bytes=fixture(100);r.chunk(1,0,90,bytes);bytes[0]^=1;r.chunk(1,0,90,bytes);
      r.expect("download","incomplete","conflicting_log_chunk");
      require(fs::is_empty(r.directory.path),"conflicting duplicate released a file");
    });
    test("changed_catalog_and_size_rejected",[](Rig& r) {
      r.seed(100);r.mail("schema=1,id=download,action=download,catalog=catalog,log_id=1");r.listWire(1,1,1);
      r.entry(1,1,1,100,16);r.expect("download","incomplete","selected_entry_changed");
      r.mail("schema=1,id=stale,action=download,catalog=catalog,log_id=1");
      r.expect("stale","rejected","fresh_catalog_required");
      require(!r.count(MAVLINK_MSG_ID_LOG_REQUEST_DATA) && fs::is_empty(r.directory.path),"changed catalog admitted data");
    });
    test("advertised_size_is_not_eof",[](Rig& r) {
      const auto bytes=fixture(101);r.seed(100);r.download(100);r.chunk(1,0,90,bytes);r.chunk(1,90,10,bytes);
      r.dataWire(1,100,1);r.chunk(1,100,1,bytes);
      r.expect("download","incomplete","advertised_size_not_eof");
      require(fs::is_empty(r.directory.path),"grew during transfer and released prefix");
    });
    test("cancel_before_and_after_handoff",[](Rig& r) {
      r.mail("schema=1,id=early,action=list");
      r.mail("schema=1,id=cancel-early,action=cancel,target=early");
      r.expect("early","cancelled","operator_cancel");
      require(r.frames.empty() && stringField(r.result("early"),"cleanup","no_local_send"),"early cancel emitted wire");
      r.seed(100);r.download(100);r.chunk(1,0,90,fixture(100));
      r.mail("schema=1,id=stale-cancel,action=cancel,target=download",3);
      r.expect("stale-cancel","rejected","stale_future_or_pre_session_request");
      r.mail("schema=1,id=cancel,action=cancel,target=download");
      r.expect("download","cancelled","operator_cancel");
      require(r.count(MAVLINK_MSG_ID_LOG_REQUEST_END)==2 && r.count(MAVLINK_MSG_ID_LOG_REQUEST_DATA)==1 &&
        fs::is_empty(r.directory.path),"cancel failed to fence waiting data or cleanup");
    });
    test("reset_and_armed_fence",[](Rig& r) {
      r.seed(100);r.download(100);r.host.disconnect(r.now);
      r.expect("download","cancelled","connection_reset");
      require(stringField(r.result("download"),"cleanup","session_lost_remote_unknown"),"reset lost uncertainty");
      r.chunk(1,0,90,fixture(100));r.tick();
      r.mail("schema=1,id=stale,action=download,catalog=catalog,log_id=1");
      r.expect("stale","rejected","fresh_catalog_required");
      r.airborne=true;r.tick();r.mail("schema=1,id=armed,action=list");r.expect("armed","rejected","disarmed_ground_required");
      require(r.count(MAVLINK_MSG_ID_LOG_REQUEST_END)==1 && fs::is_empty(r.directory.path),"lost session sent END or file");
    });
    test("cancel_after_byte_validation_before_end_retires_release",[](Rig& r) {
      const auto bytes=fixture(100);
      r.seed(100);r.download(100);r.chunk(1,0,90,bytes);r.chunk(1,90,10,bytes);
      r.dataWire(1,100,1);r.chunk(1,100,0,bytes);
      r.listWire(2,1,1);r.entry(1,1,1,100);
      r.mail("schema=1,id=late-cancel,action=cancel,target=download");
      r.expect("late-cancel","accepted","target_cancelled");
      r.expect("download","cancelled","operator_cancel");
      require(r.count(MAVLINK_MSG_ID_LOG_REQUEST_END)==2 && fs::is_empty(r.directory.path),
        "post-verification cancel emitted a file or lost its END");
    });
    test("end_handoff_failure_is_explicit",[](Rig& r) {
      r.deny_end=true;r.mail("schema=1,id=list,action=list");r.listWire(0,0,65535);
      r.entry(1,1,1,100);r.expect("list","incomplete","log_cleanup_or_ground_not_confirmed_locally");
      require(r.count(MAVLINK_MSG_ID_LOG_REQUEST_END,false)==2 && r.count(MAVLINK_MSG_ID_LOG_REQUEST_END)==0 &&
        stringField(r.result("list"),"cleanup","end_not_accepted_remote_unknown"),"cleanup denial not bounded/observable");
    });
    test("no_overwrite_and_request_path_rejected",[](Rig& r) {
      const auto bytes=fixture(100);
      r.seed(bytes.size());
      r.mail("schema=1,id=outside,action=download,catalog=catalog,log_id=1,path=../escape");
      r.expect("outside","rejected","invalid_request");
      r.download(bytes.size());r.chunk(1,0,90,bytes);r.chunk(1,90,10,bytes);
      r.dataWire(1,100,1);r.chunk(1,100,0,bytes);
      r.listWire(2,1,1);r.entry(1,1,1,bytes.size());
      const auto existing=r.directory.path/"log-1-download.bin";
      std::ofstream(existing)<<"prior operator file";
      r.expect("download","incomplete","local_destination_unavailable");
      std::ifstream in(existing);
      const std::string preserved{std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>()};
      require(preserved=="prior operator file" && hasNoPartial(r.directory.path),"pre-existing file changed or staging leaked");
    });
    test("duplicate_noise_cannot_extend_original_deadline",[](Rig& r) {
      const auto sent=r.now+.001;
      r.mail("schema=1,id=deadline,action=list");
      r.listWire(0,0,65535);r.entry(1,2,2,100);
      while(r.now<sent+2.1) { r.entry(1,2,2,100);r.tick(.05); }
      r.expect("deadline","timed_out","original_deadline");
      require(r.count(MAVLINK_MSG_ID_LOG_REQUEST_LIST)>=2 && r.count(MAVLINK_MSG_ID_LOG_REQUEST_END)==1,
        "retry or bounded END missing");
      for(const auto& frame:r.frames)
        if(frame.message.msgid!=MAVLINK_MSG_ID_LOG_REQUEST_END) require(frame.at<sent+2,"old work transmitted past bound");
    },2);
    PrivateDirectory directory;
    const auto validate=[&](const fs::path& path) {
      Runtime host([](const auto&,const auto&,double,bool) {},[](const uint8_t*,std::size_t){return false;},false,false);
      require(host.load(argv[1],"flight_logs"),"load module for config test");
      try { host.configureModules({{"flight_logs",{{"directory",path.string()}}}}); }
      catch(const std::invalid_argument&) { return; }
      throw std::runtime_error("invalid directory accepted");
    };
    validate("relative");
    fs::create_directory_symlink(directory.path,directory.path/"redirect");
    validate(directory.path/"redirect");
    validate(directory.path/".."/directory.path.filename());
    require(chmod(directory.path.c_str(),0755)==0,"set insecure directory mode");
    validate(directory.path);
    std::cout<<"{\"case\":\"reject_relative_symlink_dotdot_and_public_directories\",\"result\":\"PASS\"}\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';return 1;
  }
}

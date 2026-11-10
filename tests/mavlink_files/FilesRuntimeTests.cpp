#include "Runtime.h"
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

using namespace mavmoos::runtime;
namespace {
void check(bool okay,const char* reason) { if(!okay) throw std::runtime_error(reason); }
class PrivateDirectory {
public:
  PrivateDirectory() {
    auto path=std::filesystem::canonical(std::filesystem::temp_directory_path())/"mavmoos-files-host-XXXXXX";
    auto value=path.string();
    const auto* out=mkdtemp(value.data());
    if(!out) throw std::runtime_error("cannot create owned temporary directory");
    m_path=out;
  }
  ~PrivateDirectory() { std::error_code ec;std::filesystem::remove(m_path,ec); }
  const std::string& path() const { return m_path; }
private:
  std::string m_path;
};
struct Rig {
  std::string telemetry, files;
  PrivateDirectory directory;
  double now=100;
  std::optional<double> final_time;
  bool allowed_root=true;
  std::vector<mavlink_message_t> sent;
  Runtime host;
  Rig(const char* telemetry_path,const char* files_path,bool simulation=true,bool shadow=false,bool load_telemetry=true)
    :telemetry(telemetry_path),files(files_path),
     host([](auto&&,auto&&,double,bool){},[](auto,auto){return false;},simulation,shadow,[this]{return now;},2,3,true)
  {
    if(load_telemetry) check(host.load(telemetry,"telemetry"),"load telemetry");
    check(host.load(files,"mavlink_files"),"load files");
    host.configureModules({{"mavlink_files",{{"directory",directory.path()}}}});
    if(simulation&&!shadow) host.enableFileTransferBench([this](auto,const auto&){return allowed_root;});
    host.setBoundedSend([this](const uint8_t* bytes,std::size_t size,double,const auto& eligible){
      mavlink_message_t m{}, buffer{};mavlink_status_t status{}, accepted{};
      for(size_t i=0;i<size;++i) mavlink_frame_char_buffer(&buffer,&status,bytes[i],&m,&accepted);
      const bool ftp=m.msgid==MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL;
      if(ftp && final_time) now=*final_time;
      if(!eligible(now)) return false;
      sent.push_back(m);return true;
    });
    host.finishStartup(); refresh(); host.tick(now); sent.clear();
  }
  void refresh(bool armed=false,uint8_t landed=MAV_LANDED_STATE_ON_GROUND)
  {
    mavlink_message_t m{};
    mavlink_msg_heartbeat_pack(1,1,&m,2,3,armed?MAV_MODE_FLAG_SAFETY_ARMED:0,0,MAV_STATE_STANDBY);
    host.message(m,now);
    mavlink_msg_extended_sys_state_pack(1,1,&m,0,landed);host.message(m,now);
  }
  std::vector<mavlink_file_transfer_protocol_t> transfers()
  {
    std::vector<mavlink_file_transfer_protocol_t> out;
    for(const auto& m:sent) if(m.msgid==MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) {
      check(m.sysid==245 && m.compid==191,"FTP sender mismatch");
      out.emplace_back();mavlink_msg_file_transfer_protocol_decode(&m,&out.back());
      check(out.back().target_network==0 && out.back().target_system==1 && out.back().target_component==1,"FTP destination mismatch");
    }
    return out;
  }
  FileTransferRequest list() const { return {"list.1","list","mavmoos-ftp",FileTransferAction::List,66,1,0,0,now+.2,now+4}; }
};
}

int main(int argc,char** argv)
{
  try {
    check(argc==3,"usage: files_runtime_tests TELEMETRY_MODULE FILES_MODULE");
    for(const auto& [simulation,shadow]:{std::pair{false,false},std::pair{true,true}}) {
      Rig passive(argv[1],argv[2],simulation,shadow);
      check(!passive.host.fileTransferReady() && !passive.host.fileTransfer(passive.list()),"passive mode admitted FTP");
      passive.host.tick(passive.now);
      check(passive.sent.empty(),"passive mode emitted egress");
    }
    {
      bool rejected=false;
      try { Rig incomplete(argv[1],argv[2],true,false,false); }
      catch(const std::invalid_argument&) { rejected=true; }
      check(rejected,"active file bench accepted another roster");
    }
    {
      Rig r(argv[1],argv[2]);const auto good=r.list();
      auto bad=good;bad.action=static_cast<FileTransferAction>(6);
      check(!r.host.fileTransfer(bad),"modifying FTP opcode admitted");
      bad=good;bad.path="/mavmoos-ftp";
      check(!r.host.fileTransfer(bad),"absolute LIST path admitted");
      bad=good;bad.session=2;
      check(!r.host.fileTransfer(bad),"wrong FTP session admitted");
      bad=good;bad.sequence=67;
      check(!r.host.fileTransfer(bad),"non-pinned FTP sequence parity admitted");
      bad=good;bad.offset=128;
      check(!r.host.fileTransfer(bad),"unbounded LIST offset admitted");
      bad=good;bad.action=FileTransferAction::OpenReadOnly;bad.path="mavmoos-ftp/../sample";
      check(!r.host.fileTransfer(bad),"traversal path admitted");
      bad=good;bad.action=FileTransferAction::Read;bad.path.clear();bad.count=1;
      check(!r.host.fileTransfer(bad),"read before open admitted");
      check(!r.host.parameter({"blocked","SIM_WIND_SPD",ParameterAction::Write,-1,1,MAV_PARAM_TYPE_REAL32,101}),
            "file-only bench admitted parameter effect");
      check(r.host.fileTransfer(good) && !r.host.fileTransfer(good),"file pending budget exceeded");
      r.host.retireFileTransferPacket("list","other");
      check(!r.host.fileTransfer(good),"wrong token retired the queued LIST");
      r.host.retireFileTransferPacket("list",good.id);
      check(r.host.fileTransfer(good),"obsolete retry retirement cancelled the logical LIST lease");
      r.host.cancelFileTransfer("list");r.host.tick(r.now);
      check(r.transfers().empty(),"cancelled LIST reached final send");
      check(r.host.fileTransfer(good),"valid LIST refused after cleanup");
      r.host.tick(r.now);
      const auto out=r.transfers();
      check(out.size()==1 && out[0].payload[0]==66 && out[0].payload[2]==1 && out[0].payload[3]==3 &&
        out[0].payload[4]==11 && std::string(reinterpret_cast<const char*>(&out[0].payload[12]),11)=="mavmoos-ftp",
        "typed LIST packing differs from wire contract");
    }
    {
      Rig r(argv[1],argv[2]);check(r.host.fileTransfer(r.list()),"private-root setup failed");
      r.allowed_root=false;r.host.tick(r.now);
      check(r.transfers().empty(),"source-root loss crossed final FTP send");
    }
    {
      Rig r(argv[1],argv[2]);const auto good=r.list();
      check(r.host.fileTransfer(good),"final-deadline setup failed");
      r.final_time=good.expires;r.host.tick(r.now);
      check(r.transfers().empty(),"file packet crossed socket cutoff");
    }
    {
      Rig r(argv[1],argv[2]);auto open=r.list();open.id="get.1";open.operation_id="get";
      open.action=FileTransferAction::OpenReadOnly;open.path="mavmoos-ftp/a.bin";
      check(r.host.fileTransfer(open),"OPEN denied");
      r.host.tick(r.now);check(r.transfers().size()==1,"OPEN missing");
      auto too_much=open;too_much.id="get.2";too_much.action=FileTransferAction::Read;too_much.path.clear();
      too_much.offset=65536;too_much.count=239;
      check(!r.host.fileTransfer(too_much),"past-max READ length admitted");
      r.host.cancelFileTransfer("get");
      auto read=open;read.id="get.2";read.action=FileTransferAction::Read;read.path.clear();read.count=10;
      check(!r.host.fileTransfer(read),"cancelled read admitted");
      auto close=read;close.action=FileTransferAction::Terminate;close.count=0;
      check(r.host.fileTransfer(close),"known session cleanup after cancellation refused");
      r.host.tick(r.now);const auto out=r.transfers();
      check(out.size()==2 && out.back().payload[3]==1,"wrong operation survived cleanup fence");
      r.host.finishFileTransfer("get");
      check(!r.host.fileTransfer(close),"cleanup after release admitted");
    }
    {
      Rig r(argv[1],argv[2]);check(r.host.fileTransfer(r.list()),"ground-state setup");
      r.now+=.1;r.refresh(true);r.host.tick(r.now);
      check(r.transfers().empty() && !r.host.fileTransfer(r.list()),"arming failed to fence waiting FTP");
      r.now+=.1;r.refresh(false,MAV_LANDED_STATE_IN_AIR);
      check(!r.host.fileTransfer(r.list()),"airborne file transfer admitted");
    }
    std::cout<<"MAVLink FTP host isolation, queue, socket deadline, cancel and cleanup passed\n";
    return 0;
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n';return 1; }
}

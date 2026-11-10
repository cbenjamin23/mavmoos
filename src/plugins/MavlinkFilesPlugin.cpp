#include "Plugin.h"
#include "FileTransferService.h"
#include "../libmavmoos_ftp/FtpTransaction.h"
#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <fcntl.h>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

namespace mavmoos::runtime {
namespace {
constexpr char request_key[]="MAVMOOS_FILE_REQUEST";
constexpr char result_key[]="MAVMOOS_FILE_RESULT";
constexpr char catalog_key[]="MAVMOOS_FILE_CATALOG";
constexpr double catalog_lifetime=30, mail_lifetime=2;
constexpr size_t max_requests=128;

class Fd {
public:
  explicit Fd(int fd=-1):m_fd(fd) {}
  ~Fd() { reset(); }
  Fd(const Fd&)=delete;
  Fd& operator=(const Fd&)=delete;
  int get() const { return m_fd; }
  int release() { const auto fd=m_fd; m_fd=-1; return fd; }
  void reset(int fd=-1) { if(m_fd>=0) close(m_fd);m_fd=fd; }
private:
  int m_fd;
};

bool privateDir(int fd,struct stat& s)
{
  return fd>=0 && fstat(fd,&s)==0 && S_ISDIR(s.st_mode) && s.st_uid==geteuid() && (s.st_mode&077)==0;
}
int openDirectory(const std::string& path)
{
  if(path.size()<2 || path.size()>4096 || path.front()!='/' || path.back()=='/' ||
     path.find('\0')!=std::string::npos) return -1;
  constexpr int directory_flags=O_DIRECTORY|O_NOFOLLOW|O_CLOEXEC;
#ifdef O_SEARCH
  constexpr int traverse=O_SEARCH|directory_flags;
#else
  constexpr int traverse=O_RDONLY|directory_flags;
#endif
  Fd fd(open("/",traverse));
  std::istringstream parts(path.substr(1));
  for(std::string part;std::getline(parts,part,'/');) {
    if(fd.get()<0 || part.empty() || part=="." || part=="..") return -1;
    fd.reset(openat(fd.get(),part.c_str(),traverse));
  }
  struct stat s{};
  return privateDir(fd.get(),s)?openat(fd.get(),".",O_RDONLY|directory_flags):-1;
}
bool sameDirectory(int fd,const std::string& path)
{
  Fd named(openDirectory(path));
  struct stat current{}, original{};
  return privateDir(fd,original) && privateDir(named.get(),current) &&
    current.st_dev==original.st_dev && current.st_ino==original.st_ino;
}
bool syncFile(int fd)
{
  int code=0;
  do { code=fsync(fd); } while(code<0 && errno==EINTR);
  return code==0;
}
bool writeBytes(int fd,const std::vector<uint8_t>& bytes)
{
  for(std::size_t offset=0;offset<bytes.size();) {
    const auto written=write(fd,bytes.data()+offset,bytes.size()-offset);
    if(written<0 && errno==EINTR) continue;
    if(written<=0) return false;
    offset+=static_cast<std::size_t>(written);
  }
  return true;
}
std::optional<std::map<std::string,std::string>> parse(const std::string& text)
{
  if(text.empty() || text.size()>256 || text.back()==',') return {};
  std::map<std::string,std::string> args;
  std::istringstream fields(text);
  for(std::string field;std::getline(fields,field,',');) {
    const auto at=field.find('=');
    if(at==std::string::npos || at==0 || at+1==field.size() ||
       !args.emplace(field.substr(0,at),field.substr(at+1)).second) return {};
  }
  return args;
}
bool identifier(const std::string& s) { return s.size()<=48 && ftp::safeBasename(s); }
const char* sessionName(ftp::RemoteSession s)
{
  switch(s) {
  case ftp::RemoteSession::None: return "none";
  case ftp::RemoteSession::Possible: return "possible";
  case ftp::RemoteSession::Open: return "open";
  case ftp::RemoteSession::ConfirmedClosed: return "confirmed_closed";
  case ftp::RemoteSession::Uncertain: return "uncertain";
  }
  return "uncertain";
}

struct Operation { std::string id, action; };
struct Pending { std::string host_id; ftp::Intent intent; };
struct Catalog { std::string id; double at; };
class MavlinkFiles final:public Plugin,public FileTransferListener {
public:
  MavlinkFiles(Services& host,FileTransferService& files)
    :m_host(host),m_files(files),m_transfer(static_cast<uint16_t>(std::random_device{}())),m_boundary(host.now()) {}
  std::string name() const override { return "mavlink_files"; }
  PluginDescriptor descriptor() const override {
    PluginDescriptor result;
    result.profile="arducopter_disarmed_read_only_files";
    result.capabilities={{"mavlink_files.list",request_key,"action=list"},
      {"mavlink_files.get",request_key,"action=get"},{"mavlink_files.cancel",request_key,"action=cancel"}};
    result.request_owners={request_key};
    result.exclusive_resources={"protocol.mavlink_ftp"};
    result.observation_streams={{MAVLINK_MSG_ID_EXTENDED_SYS_STATE,250000.f}};
    return result;
  }
  std::vector<std::string> subscriptions() const override { return {request_key}; }
  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override
  {
    std::string path;
    unsigned timeout=45;
    for(const auto& setting:settings) {
      if(setting.key=="directory") path=setting.value;
      else if(setting.key=="timeout") {
        const auto first=setting.value.data(),last=first+setting.value.size();
        const auto parsed=std::from_chars(first,last,timeout);
        if(parsed.ec!=std::errc() || parsed.ptr!=last || timeout<2 || timeout>60)
          return {ModuleConfigStatus::Invalid,setting.key,"expected whole seconds from 2 to 60"};
      } else return {ModuleConfigStatus::UnknownKey,setting.key,"unknown file transfer setting"};
    }
    Fd directory(openDirectory(path));
    if(directory.get()<0)
      return {ModuleConfigStatus::Invalid,"directory","expected an existing absolute owner-private directory with no symlink components"};
    m_directory.reset(directory.release());m_directory_path=std::move(path);m_timeout=timeout;
    return {};
  }
  void onMail(const Mail& mail) override
  {
    if(mail.key!=request_key) return;
    auto decoded=mail.numeric?std::nullopt:parse(mail.text);
    auto args=decoded.value_or(std::map<std::string,std::string>{});
    const auto size=args.size();
    const auto id=args["id"],action=args["action"];
    const bool known=action=="list"||action=="get"||action=="cancel";
    const Operation op{identifier(id)?id:"invalid",known?action:"invalid"};
    const bool shape=args["schema"]=="1" && ((action=="list" && size==3) ||
      (action=="get" && size==5 && identifier(args["catalog"]) && ftp::safeBasename(args["name"])) ||
      (action=="cancel" && size==4 && identifier(args["operation_id"])));
    if(!identifier(id) || !shape) { report(op,"REJECTED","invalid_request");return; }
    const auto now=m_host.now();
    const auto age=mail.received_at-mail.published_at;
    if(!std::isfinite(age) || age<-.05 || age>mail_lifetime || !std::isfinite(now) ||
       mail.published_at<=m_boundary || mail.received_at>now || now-mail.received_at>mail_lifetime) {
      report(op,"REJECTED","stale_future_or_pre_session_request");return;
    }
    if(m_seen.count(op.id) || m_seen.size()>=max_requests+(action=="cancel"?1:0)) {
      report(op,"REJECTED","duplicate_or_history_full");return;
    }
    if(action=="cancel") {
      if(!m_operation || m_operation->id!=args["operation_id"] || !m_transfer.cancel(now)) {
        report(op,"REJECTED","no_matching_active_operation");return;
      }
      m_seen.insert(id);
      m_files.cancelFileTransfer(m_operation->id);
      if(m_pending && m_pending->intent.opcode!=ftp::Opcode::TerminateSession) m_pending.reset();
      report(op,"CANCEL_ACCEPTED","operator_cancel");
      settle();
      return;
    }
    if(m_operation) { report(op,"REJECTED","busy");return; }
    if(!m_files.fileTransferReady()) { report(op,"REJECTED","disarmed_owned_bench_required");return; }
    if(!sameDirectory(m_directory.get(),m_directory_path)) { report(op,"REJECTED","local_directory_changed");return; }
    const double deadline=mail.published_at+m_timeout;
    if(action=="list") {
      if(!m_transfer.beginList(now,deadline)) { report(op,"REJECTED","list_not_admitted");return; }
      clearCatalog();
    } else {
      if(!m_catalog || m_catalog->id!=args["catalog"] || now-m_catalog->at>catalog_lifetime) {
        report(op,"REJECTED","fresh_catalog_required");return;
      }
      struct stat existing{};
      const auto name=op.id+".bin";
      if(fstatat(m_directory.get(),name.c_str(),&existing,AT_SYMLINK_NOFOLLOW)==0 || errno!=ENOENT) {
        report(op,"REJECTED","local_destination_unavailable");return;
      }
      if(!m_transfer.beginGet(args["name"],now,deadline)) { report(op,"REJECTED","listed_file_required");return; }
      clearCatalog();
    }
    m_seen.insert(id);m_operation=op;
    report(op,"QUEUED","original_deadline_bound");
  }
  void onMessageWithMetadata(const mavlink_message_t& msg,const MessageTime& time) override
  {
    if(!m_operation || msg.msgid!=MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL ||
       (msg.magic!=MAVLINK_STX && msg.magic!=MAVLINK_STX_MAVLINK1) || msg.len<8 ||
       (msg.magic==MAVLINK_STX_MAVLINK1 && msg.len!=MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN)) return;
    mavlink_file_transfer_protocol_t packet{};
    mavlink_msg_file_transfer_protocol_decode(&msg,&packet);
    ftp::Reply reply{static_cast<uint8_t>(msg.sysid),static_cast<uint8_t>(msg.compid),
      packet.target_network,packet.target_system,packet.target_component,
      {},time.effective_at,time.read_at};
    std::copy_n(packet.payload,reply.payload.size(),reply.payload.begin());
    if(!m_transfer.receive(reply,m_host.now())) return;
    retireObsoletePacket();
    const auto& b=reply.payload;
    const auto sequence=unsigned(b[0])|(unsigned(b[1])<<8);
    const auto offset=uint32_t(b[8])|(uint32_t(b[9])<<8)|(uint32_t(b[10])<<16)|(uint32_t(b[11])<<24);
    m_host.publish("MAVMOOS_FILE_RX_RESULT","operation_id="+m_operation->id+",reply="+std::to_string(b[3])+
      ",opcode="+std::to_string(b[5])+",sequence="+std::to_string(sequence)+
      ",session="+std::to_string(b[2])+",offset="+std::to_string(offset)+",count="+std::to_string(b[4]));
    settle();
  }
  void onFileTransferSent(const FileTransferRequest& req,double at,bool accepted) override
  {
    if(!m_pending || !m_operation || req.operation_id!=m_operation->id || req.id!=m_pending->host_id) return;
    const auto intent=m_pending->intent;
    m_pending.reset();
    m_transfer.sent(intent,at,accepted);
    settle();
  }
  void onReset(const std::string&) override { lose("connection_reset"); }
  void onInputBoundary() override
  {
    const auto now=m_host.now();
    if(!m_files.fileTransferReady()) { if(m_operation || m_catalog) lose("disarmed_ground_evidence_lost");return; }
    if(m_catalog && now-m_catalog->at>catalog_lifetime) clearCatalog();
    if(m_operation) {
      const auto before=m_transfer.status().phase;
      m_transfer.advance(now);
      retireObsoletePacket();
      if(before!=ftp::Phase::Closing && m_transfer.status().phase==ftp::Phase::Closing) {
        m_files.cancelFileTransfer(m_operation->id);m_pending.reset();
      }
      settle();
    }
  }
  void iterate() override
  {
    onInputBoundary();
    if(!m_operation || m_pending) return;
    const auto now=m_host.now();
    auto intent=m_transfer.take(now);
    if(!intent) { settle();return; }
    const auto id=m_operation->id+"."+std::to_string(intent->token);
    const FileTransferRequest request{id,m_operation->id,intent->path,
      static_cast<FileTransferAction>(intent->opcode),intent->sequence,intent->session,
      static_cast<uint8_t>(intent->path.empty()?intent->size:0),
      intent->offset,intent->expires,intent->operation_expires};
    if(m_files.fileTransfer(request)) m_pending=Pending{id,std::move(*intent)};
    else { m_transfer.sent(*intent,now,false);settle(); }
  }
private:
  void retireObsoletePacket()
  {
    if(m_operation && m_pending && !m_transfer.eligible(m_pending->intent,m_host.now())) {
      m_files.retireFileTransferPacket(m_operation->id,m_pending->host_id);
      m_pending.reset();
    }
  }
  void report(const Operation& op,const char* status,std::string_view reason,
              ftp::RemoteSession remote=ftp::RemoteSession::None,std::size_t bytes=0,const std::string& file={})
  {
    // IDs and released names are validated ASCII tokens, and reasons are
    // fixed local strings. No server path or bytes are inserted here.
    m_host.publish(result_key,"{\"schema\":1,\"id\":\""+op.id+"\",\"action\":\""+op.action+
      "\",\"status\":\""+status+"\",\"reason\":\""+std::string(reason)+"\",\"remote_session\":\""+
      sessionName(remote)+"\",\"bytes\":"+std::to_string(bytes)+",\"file\":"+
      (file.empty()?"null":"\""+file+"\"")+"}");
  }
  void clearCatalog()
  {
    if(!m_catalog) return;
    m_catalog.reset();
    m_host.publish(catalog_key,"{\"schema\":1,\"id\":null,\"entries\":[]}");
  }
  void publishCatalog(const std::string& id)
  {
    m_catalog=Catalog{id,m_host.now()};
    std::string data="{\"schema\":1,\"id\":\""+id+"\",\"entries\":[";
    bool first=true;
    for(const auto& entry:m_transfer.listing()) {
      if(!ftp::safeBasename(entry.name)) continue;
      if(!first) data+=',';
      first=false;
      data+="{\"name\":\""+entry.name+"\",\"kind\":\""+
        (entry.kind==ftp::EntryKind::File?"file":"directory")+"\",\"bytes\":"+std::to_string(entry.size)+'}';
    }
    m_host.publish(catalog_key,data+"]}");
  }
  void lose(const char* reason)
  {
    m_boundary=m_host.now();
    const auto prior=m_transfer.status().remote_session;
    m_transfer.reset(m_host.now());
    if(m_operation) {
      const auto remote=prior==ftp::RemoteSession::ConfirmedClosed?prior:m_transfer.status().remote_session;
      report(*m_operation,"CANCELLED",reason,remote);
      m_files.finishFileTransfer(m_operation->id);
    }
    m_operation.reset();m_pending.reset();clearCatalog();
  }
  const char* releaseFile(const std::string& id,const std::vector<uint8_t>& bytes)
  {
    const auto fd=m_directory.get();
    if(bytes.size()>ftp::max_file_size || !sameDirectory(fd,m_directory_path)) return "local_directory_changed";
    const auto final=id+".bin",stage=id+".partial";
    Fd temp(openat(fd,stage.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW|O_CLOEXEC,0600));
    if(temp.get()<0) return "local_stage_unavailable";
    struct stat opened{}, named{};
    const bool ready=writeBytes(temp.get(),bytes) && syncFile(temp.get()) &&
      sameDirectory(fd,m_directory_path) && fstat(temp.get(),&opened)==0 &&
      fstatat(fd,stage.c_str(),&named,AT_SYMLINK_NOFOLLOW)==0 &&
      S_ISREG(named.st_mode) && named.st_dev==opened.st_dev && named.st_ino==opened.st_ino;
    const bool linked=ready && linkat(fd,stage.c_str(),fd,final.c_str(),0)==0;
    const bool removed=unlinkat(fd,stage.c_str(),0)==0;
    if(linked && removed && syncFile(fd) && sameDirectory(fd,m_directory_path)) return nullptr;
    const bool rolled_back=!linked || (unlinkat(fd,final.c_str(),0)==0 && syncFile(fd));
    return removed && rolled_back?"local_release_failed":"local_release_failed_residue_possible";
  }
  void settle()
  {
    if(!m_operation) return;
    const auto s=m_transfer.status();
    if(ftp::active(s.phase)) return;
    const auto op=*m_operation;
    m_operation.reset();m_pending.reset();m_files.finishFileTransfer(op.id);
    switch(s.phase) {
    case ftp::Phase::Listed:
      publishCatalog(op.id);report(op,"LISTED",s.reason,s.remote_session);break;
    case ftp::Phase::Complete: {
      const auto completed=m_transfer.takeFile();
      const auto* error=completed && s.remote_session==ftp::RemoteSession::ConfirmedClosed
        ?releaseFile(op.id,completed->bytes):"remote_completion_unproven";
      if(!error) report(op,"COMPLETE",s.reason,s.remote_session,completed->bytes.size(),op.id+".bin");
      else report(op,"FAILED",error,s.remote_session);
      break;
    }
    case ftp::Phase::Cancelled: report(op,"CANCELLED",s.reason,s.remote_session);break;
    case ftp::Phase::Expired: report(op,"EXPIRED",s.reason,s.remote_session);break;
    default: report(op,"FAILED",s.reason,s.remote_session);break;
    }
  }
  Services& m_host;
  FileTransferService& m_files;
  ftp::Transaction m_transfer;
  double m_boundary;
  unsigned m_timeout=45;
  Fd m_directory;
  std::string m_directory_path;
  std::set<std::string> m_seen;
  std::optional<Operation> m_operation;
  std::optional<Pending> m_pending;
  std::optional<Catalog> m_catalog;
};
}
}

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* host)
{
  if(abi!=mavmoos::runtime::plugin_abi || !host) return nullptr;
  auto* files=dynamic_cast<mavmoos::runtime::FileTransferService*>(host);
  return files?new mavmoos::runtime::MavlinkFiles(*host,*files):nullptr;
}
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin) { delete plugin; }

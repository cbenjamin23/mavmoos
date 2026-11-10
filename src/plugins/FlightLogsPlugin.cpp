/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                  */
/*    FILE: FlightLogsPlugin.cpp                             */
/*    DATE: November 6, 2026                                 */
/************************************************************/
#include "Plugin.h"
#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <fcntl.h>
#include <iomanip>
#include <locale>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

namespace mavmoos::runtime {
namespace {
constexpr char request_key[]="MAVMOOS_FLIGHT_LOG_REQUEST";
constexpr char result_key[]="MAVMOOS_FLIGHT_LOG_RESULT";
constexpr char catalog_key[]="MAVMOOS_FLIGHT_LOG_CATALOG";
constexpr unsigned max_entries=128, max_file_bytes=4*1024*1024;
constexpr unsigned chunk_bytes=MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN, window_chunks=32;
constexpr unsigned max_retries=32, max_idle_attempts=4;
constexpr double idle_timeout=.65, send_timeout=1, cleanup_grace=1.5;
constexpr double catalog_age=60, mail_age=2;

class FileDescriptor {
public:
  explicit FileDescriptor(int fd=-1):m_fd(fd) {}
  ~FileDescriptor() { reset(); }
  FileDescriptor(const FileDescriptor&)=delete;
  FileDescriptor& operator=(const FileDescriptor&)=delete;
  int get() const { return m_fd; }
  int release() { const int fd=m_fd; m_fd=-1; return fd; }
  void reset(int fd=-1) { if(m_fd>=0) close(m_fd); m_fd=fd; }
private:
  int m_fd;
};

bool privateDirectory(int fd,struct stat& info)
{
  return fd>=0 && fstat(fd,&info)==0 && S_ISDIR(info.st_mode) &&
    info.st_uid==geteuid() && (info.st_mode&(S_IRWXG|S_IRWXO))==0;
}
int openPrivateDirectory(const std::string& path)
{
  if(path.size()<2 || path.front()!='/' || path.back()=='/' ||
     path.find('\0')!=std::string::npos) return -1;
  constexpr int safe=O_DIRECTORY|O_NOFOLLOW|O_CLOEXEC;
#ifdef O_SEARCH
  // Search ancestors without enumerating them. Some local policies permit
  // the configured descendant while denying read access to an ancestor.
  constexpr int traverse=O_SEARCH|safe;
#else
  constexpr int traverse=O_RDONLY|safe;
#endif
  FileDescriptor fd(open("/",traverse));
  std::istringstream parts(path.substr(1));
  for(std::string part;std::getline(parts,part,'/');) {
    if(fd.get()<0 || part.empty() || part=="." || part=="..") return -1;
    fd.reset(openat(fd.get(),part.c_str(),traverse));
  }
  struct stat info{};
  return privateDirectory(fd.get(),info)?openat(fd.get(),".",O_RDONLY|safe):-1;
}
bool sameDirectory(int fd,const std::string& path)
{
  FileDescriptor named(openPrivateDirectory(path));
  struct stat original{}, current{};
  return privateDirectory(fd,original) && privateDirectory(named.get(),current) &&
    original.st_dev==current.st_dev && original.st_ino==current.st_ino;
}
bool syncFile(int fd)
{
  int result;
  do { result=fsync(fd); } while(result<0 && errno==EINTR);
  return result==0;
}
bool writeAll(int fd,const std::vector<uint8_t>& bytes)
{
  std::size_t offset=0;
  while(offset<bytes.size()) {
    const auto wrote=write(fd,bytes.data()+offset,bytes.size()-offset);
    if(wrote<0 && errno==EINTR) continue;
    if(wrote<=0) return false;
    offset+=static_cast<std::size_t>(wrote);
  }
  return true;
}
std::string digest(const std::vector<uint8_t>& bytes)
{
  // Local checksum for the released bytes. The FC does not supply a digest.
  uint64_t value=UINT64_C(14695981039346656037);
  for(const uint8_t byte:bytes) { value^=byte; value*=UINT64_C(1099511628211); }
  std::ostringstream out;
  out<<std::hex<<std::setw(16)<<std::setfill('0')<<value;
  return out.str();
}
bool alphanumeric(unsigned char c)
{
  return (c>='A' && c<='Z') || (c>='a' && c<='z') || (c>='0' && c<='9');
}
bool token(const std::string& value)
{
  return !value.empty() && value.size()<=48 && alphanumeric(value.front()) &&
    std::all_of(value.begin(),value.end(),[](unsigned char c) {
      return alphanumeric(c) || c=='-' || c=='_' || c=='.';
    });
}
bool number(const std::string& text,unsigned& value)
{
  const auto result=std::from_chars(text.data(),text.data()+text.size(),value);
  return result.ec==std::errc() && result.ptr==text.data()+text.size();
}
std::map<std::string,std::string> fields(const std::string& text)
{
  if(text.empty() || text.size()>256 || text.back()==',') return {};
  std::map<std::string,std::string> result;
  std::istringstream input(text);
  for(std::string field;std::getline(input,field,',');) {
    const auto equal=field.find('=');
    if(equal==std::string::npos || !result.emplace(field.substr(0,equal),field.substr(equal+1)).second) return {};
  }
  return result;
}

struct Entry {
  uint32_t bytes=0, time_utc=0;
  bool matches(const Entry& other) const { return bytes==other.bytes && time_utc==other.time_utc; }
};
struct Catalog {
  std::string id;
  double acquired=0;
  std::map<uint16_t,Entry> rows;
};
enum class Phase { List, VerifyBefore, Data, Eof, VerifyAfter, Cleanup };
struct Operation {
  std::string id, action, catalog, file;
  double expires=0, issued_at=0, sent_at=0, progress_at=0, cleanup_until=0, cleanup_retry=0;
  unsigned sequence=0, retries=0, idle_attempts=0, end_attempts=0;
  Phase phase=Phase::List;
  bool remote_possible=false;
  std::optional<FlightLogRequest> pending, range;
  std::optional<uint16_t> total;
  std::map<uint16_t,Entry> rows;
  uint16_t selected=0;
  Entry entry;
  std::vector<uint8_t> bytes;
  std::vector<bool> covered;
  unsigned chunks_received=0;
  std::string outcome, reason;
};
struct LocalFile {
  bool released=false, staging_removed=true, directory_synced=false;
  const char* reason="local_storage_unavailable";
};

class FlightLogs final:public Plugin {
public:
  explicit FlightLogs(Services& services):m_services(services),m_boundary(services.now()) {}
  std::string name() const override { return "flight_logs"; }
  PluginDescriptor descriptor() const override {
    PluginDescriptor result;
    result.profile="arducopter_disarmed_logs";
    result.capabilities={{"flight_logs.list",request_key,"action=list"},
      {"flight_logs.download",request_key,"action=download"},
      {"flight_logs.cancel",request_key,"action=cancel"}};
    result.request_owners={request_key};
    result.exclusive_resources={"protocol.flight_logs"};
    result.observation_streams={{MAVLINK_MSG_ID_EXTENDED_SYS_STATE,250000.f}};
    return result;
  }
  std::vector<std::string> subscriptions() const override { return {request_key}; }
  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override
  {
    std::string directory;
    unsigned timeout=60, limit=max_file_bytes;
    for(const auto& setting:settings) {
      if(setting.key=="directory") directory=setting.value;
      else if(setting.key=="timeout") {
        if(!number(setting.value,timeout) || timeout<2 || timeout>120)
          return {ModuleConfigStatus::Invalid,setting.key,"expected an integer from 2 to 120 seconds"};
      } else if(setting.key=="max_bytes") {
        if(!number(setting.value,limit) || limit<1 || limit>max_file_bytes)
          return {ModuleConfigStatus::Invalid,setting.key,"expected an integer from 1 to 4194304 bytes"};
      } else return {ModuleConfigStatus::UnknownKey,setting.key,"unrecognized flight log setting"};
    }
    FileDescriptor opened(openPrivateDirectory(directory));
    if(opened.get()<0)
      return {ModuleConfigStatus::Invalid,"directory","expected an existing absolute owner-private directory with no symlink components"};
    m_directory.reset(opened.release());
    m_path=std::move(directory);
    m_timeout=timeout;
    m_max_bytes=limit;
    return {};
  }
  void onMail(const Mail& mail) override
  {
    if(mail.key!=request_key) return;
    auto args=mail.numeric?std::map<std::string,std::string>{}:fields(mail.text);
    const auto field_count=args.size();
    Operation op;
    const bool valid_id=token(args["id"]);
    op.id=valid_id?args["id"]:"invalid";
    const auto& action=args["action"];
    op.action=action=="list" || action=="download" || action=="cancel"?action:"invalid";
    unsigned log_id=0;
    const bool shape=(op.action=="list" && field_count==3) ||
      (op.action=="download" && field_count==5 && token(args["catalog"]) &&
        number(args["log_id"],log_id) && log_id>=1 && log_id<=max_entries) ||
      (op.action=="cancel" && field_count==4 && token(args["target"]));
    if(!shape || !valid_id || args["schema"]!="1") { report(op,"rejected","invalid_request"); return; }
    const double now=m_services.now();
    const double age=mail.received_at-mail.published_at;
    if(!std::isfinite(now) || !std::isfinite(age) || age<-.1 || age>mail_age ||
       mail.published_at<=m_boundary || mail.received_at>now+.1 || now-mail.received_at>mail_age) {
      report(op,"rejected","stale_future_or_pre_session_request"); return;
    }
    if(m_seen.count(op.id)) { report(op,"duplicate","id_already_seen"); return; }
    // Cancellation of accepted work remains available when the normal ID budget is exhausted.
    if(m_seen.size()>=128 && op.action!="cancel") { report(op,"rejected","request_history_full"); return; }
    if(op.action=="cancel") {
      if(!m_operation || m_operation->id!=args["target"] ||
         (m_operation->phase==Phase::Cleanup && m_operation->outcome!="complete")) {
        report(op,"rejected","no_matching_active_operation"); return;
      }
      if(m_seen.size()<129) m_seen.insert(op.id);
      if(m_operation->phase==Phase::Cleanup) {
        // END is already pending or due. Retire the local success without
        // replacing its cleanup request or extending its original window.
        m_operation->outcome="cancelled";
        m_operation->reason="operator_cancel";
      } else beginFinish("cancelled","operator_cancel");
      report(op,"accepted","target_cancelled");
      return;
    }
    if(m_operation) { report(op,"rejected","busy"); return; }
    if(!groundReady()) { report(op,"rejected","disarmed_ground_required"); return; }
    if(!sameDirectory(m_directory.get(),m_path)) { report(op,"rejected","local_directory_changed"); return; }
    op.expires=mail.published_at+m_timeout;
    if(op.expires<=now || op.expires-now>120) { report(op,"rejected","expired_or_future_request"); return; }
    if(op.action=="download") {
      op.catalog=args["catalog"];
      if(!m_catalog || m_catalog->id!=op.catalog || now-m_catalog->acquired>catalog_age) {
        report(op,"rejected","fresh_catalog_required"); return;
      }
      op.selected=static_cast<uint16_t>(log_id);
      const auto row=m_catalog->rows.find(op.selected);
      if(row==m_catalog->rows.end() || row->second.bytes==0 || row->second.bytes>m_max_bytes) {
        report(op,"rejected","missing_empty_or_oversized_log"); return;
      }
      op.entry=row->second;
      op.total=static_cast<uint16_t>(m_catalog->rows.size());
      op.file="log-"+std::to_string(op.selected)+"-"+op.id+".bin";
      struct stat existing{};
      if(fstatat(m_directory.get(),op.file.c_str(),&existing,AT_SYMLINK_NOFOLLOW)==0 || errno!=ENOENT) {
        report(op,"rejected","local_destination_unavailable"); return;
      }
      op.phase=Phase::VerifyBefore;
    }
    if(op.action=="list") invalidateCatalog("new_list");
    m_seen.insert(op.id);
    m_operation=std::move(op);
    report(*m_operation,"queued","original_deadline_bound");
  }
  void onReset(const std::string&) override
  {
    if(m_operation) {
      auto& op=*m_operation;
      const bool possible=op.remote_possible || op.pending.has_value();
      report(op,"cancelled","connection_reset",possible?"session_lost_remote_unknown":"no_local_send");
      m_operation.reset();
    }
    invalidateCatalog("connection_reset");
    m_boundary=std::max(m_boundary,m_services.now());
  }
  void onInputBoundary() override
  {
    if(m_catalog) {
      if(!groundReady()) invalidateCatalog("disarmed_ground_evidence_lost");
      else if(m_services.now()-m_catalog->acquired>catalog_age) invalidateCatalog("catalog_expired");
    }
    if(!m_operation || m_operation->phase==Phase::Cleanup) return;
    if(m_services.now()>=m_operation->expires) beginFinish("timed_out","original_deadline");
    else if(!groundReady()) beginFinish("cancelled","disarmed_ground_evidence_lost");
  }
  void onMessage(const mavlink_message_t& message,double read_at) override
  {
    onInputBoundary();
    if(!m_operation || !m_operation->range || message.sysid!=1 || message.compid!=1) return;
    auto& op=*m_operation;
    if(!std::isfinite(read_at) || read_at<=op.sent_at || read_at>=op.expires) return;
    if(message.msgid==MAVLINK_MSG_ID_LOG_ENTRY &&
       (op.phase==Phase::List || op.phase==Phase::VerifyBefore || op.phase==Phase::VerifyAfter)) {
      if(!validWireLength(message,MAVLINK_MSG_ID_LOG_ENTRY_LEN)) return;
      mavlink_log_entry_t entry{};
      mavlink_msg_log_entry_decode(&message,&entry);
      receiveEntry(op,entry,read_at);
    } else if(message.msgid==MAVLINK_MSG_ID_LOG_DATA && (op.phase==Phase::Data || op.phase==Phase::Eof)) {
      if(!validWireLength(message,MAVLINK_MSG_ID_LOG_DATA_LEN)) return;
      mavlink_log_data_t data{};
      mavlink_msg_log_data_decode(&message,&data);
      receiveData(op,data,read_at);
    }
  }
  void onFlightLogSent(const FlightLogRequest& request,double sent_at,bool accepted) override
  {
    if(!m_operation || !m_operation->pending || request.id!=m_operation->pending->id ||
       request.operation_id!=m_operation->id) return;
    auto& op=*m_operation;
    const auto pending=*op.pending;
    if(request.action!=pending.action || request.first!=pending.first || request.last!=pending.last ||
       request.log_id!=pending.log_id || request.offset!=pending.offset || request.count!=pending.count ||
       request.expires!=pending.expires || request.operation_expires!=pending.operation_expires) return;
    op.pending.reset();
    const bool timely=accepted && std::isfinite(sent_at) && sent_at>=op.issued_at && sent_at<request.expires;
    if(op.phase==Phase::Cleanup) {
      if(timely) settle("local_end_accepted_unacknowledged");
      else op.cleanup_retry=m_services.now()+.1;
      return;
    }
    op.remote_possible=op.remote_possible || accepted;
    if(!timely) {
      beginFinish("incomplete","request_send_failed_or_late");
      return;
    }
    op.range=request;
    op.sent_at=op.progress_at=sent_at;
  }
  void iterate() override
  {
    onInputBoundary();
    if(!m_operation) return;
    auto& op=*m_operation;
    const double now=m_services.now();
    if(op.phase==Phase::Cleanup) {
      if(!groundReady() || now>=op.cleanup_until || (!op.pending && op.end_attempts>=2)) {
        m_services.flightLogFinished(op.id);
        settle("end_not_accepted_remote_unknown");
      } else if(!op.pending && now>=op.cleanup_retry) {
        ++op.end_attempts;
        FlightLogRequest request;
        request.action=FlightLogAction::End;
        if(!submit(op,request,op.cleanup_until)) settle("end_not_accepted_remote_unknown");
      }
      return;
    }
    if(op.pending) return;
    if(op.range) {
      if(now-op.progress_at<idle_timeout) return;
      if(++op.idle_attempts>=max_idle_attempts || ++op.retries>max_retries) {
        beginFinish("incomplete","response_retry_limit"); return;
      }
      op.range.reset();
    }
    const auto request=nextRequest(op);
    if(!submit(op,request,op.expires)) {
      beginFinish(op.remote_possible?"incomplete":"rejected","local_request_rejected");
    }
  }
private:
  static bool validWireLength(const mavlink_message_t& message,uint8_t length)
  {
    // MAVLink 2 drops trailing zero bytes, including an empty LOG_ENTRY row
    // and zero-padded LOG_DATA. The generated decoder restores those zeros.
    return (message.magic==MAVLINK_STX && message.len>0 && message.len<=length) ||
      (message.magic==MAVLINK_STX_MAVLINK1 && message.len==length);
  }
  bool groundReady() const
  {
    const auto& vehicle=m_services.vehicle();
    const double now=m_services.now();
    const auto fresh=[now](double at) { return at>0 && now>=at && now-at<=2; };
    return vehicle.heartbeat_valid && fresh(vehicle.heartbeat_at) &&
      vehicle.type==MAV_TYPE_QUADROTOR && vehicle.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA &&
      vehicle.landed_valid && fresh(vehicle.landed_at) &&
      vehicle.landed_state==MAV_LANDED_STATE_ON_GROUND && !vehicle.armed;
  }
  void report(const Operation& op,const std::string& status,const std::string& reason,
              const char* cleanup="not_applicable",const LocalFile& file={})
  {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out<<std::setprecision(17)<<"{\"schema\":1,\"id\":\""<<op.id<<"\",\"action\":\""<<op.action
       <<"\",\"status\":\""<<status<<"\",\"reason\":\""<<reason<<"\",\"cleanup\":\""<<cleanup
       <<"\",\"at\":"<<m_services.now()<<",\"expires\":"<<op.expires<<",\"retries\":"<<op.retries;
    if(op.selected) out<<",\"log_id\":"<<op.selected<<",\"catalog\":\""<<op.catalog<<"\"";
    if(file.released) {
      out<<",\"file\":\""<<op.file<<"\",\"bytes\":"<<op.bytes.size()
         <<",\"digest_algorithm\":\"fnv1a64\",\"digest\":\""<<digest(op.bytes)
         <<"\",\"staging_removed\":"<<(file.staging_removed?"true":"false")
         <<",\"directory_synced\":"<<(file.directory_synced?"true":"false");
    }
    out<<'}';
    m_services.publish(result_key,out.str());
  }
  void invalidateCatalog(const char* reason)
  {
    if(!m_catalog) return;
    m_services.publish(catalog_key,"{\"schema\":1,\"status\":\"invalidated\",\"id\":\""+m_catalog->id+
      "\",\"reason\":\""+reason+"\",\"count\":0,\"logs\":[]}");
    m_catalog.reset();
  }
  void publishCatalog(Operation& op)
  {
    Catalog catalog{op.id,op.progress_at,std::move(op.rows)};
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out<<std::setprecision(17)<<"{\"schema\":1,\"status\":\"complete\",\"id\":\""<<catalog.id
       <<"\",\"source_system\":1,\"source_component\":1,\"acquired_at\":"<<catalog.acquired
       <<",\"count\":"<<catalog.rows.size()<<",\"logs\":[";
    bool comma=false;
    for(const auto& [id,entry]:catalog.rows) {
      if(comma) out<<',';
      out<<"{\"log_id\":"<<id<<",\"bytes_reported\":"<<entry.bytes<<",\"time_utc\":";
      if(entry.time_utc) out<<entry.time_utc;
      else out<<"null";
      out<<'}';
      comma=true;
    }
    out<<"]}";
    m_catalog=std::move(catalog);
    m_services.publish(catalog_key,out.str());
  }
  bool submit(Operation& op,FlightLogRequest request,double deadline)
  {
    op.issued_at=m_services.now();
    request.id=op.id+"/"+std::to_string(++op.sequence);
    request.operation_id=op.id;
    request.expires=std::min(op.issued_at+send_timeout,deadline);
    request.operation_expires=op.expires;
    if(!m_services.flightLog(request)) return false;
    op.pending=request;
    return true;
  }
  FlightLogRequest nextRequest(const Operation& op) const
  {
    FlightLogRequest request;
    if(op.phase==Phase::List) {
      if(op.total) {
        request.first=1;
        while(op.rows.count(request.first)) ++request.first;
        request.last=request.first;
        while(request.last<*op.total && !op.rows.count(request.last+1)) ++request.last;
      }
    } else if(op.phase==Phase::VerifyBefore || op.phase==Phase::VerifyAfter) {
      request.first=request.last=op.selected;
    } else {
      request.action=FlightLogAction::Data;
      request.log_id=op.selected;
      if(op.phase==Phase::Eof) {
        request.offset=op.entry.bytes;
        request.count=1;
      } else {
        const auto missing=std::find(op.covered.begin(),op.covered.end(),false);
        const auto first=static_cast<unsigned>(missing-op.covered.begin());
        unsigned last=first;
        while(last<op.covered.size() && last-first<window_chunks && !op.covered[last]) ++last;
        request.offset=first*chunk_bytes;
        request.count=std::min(last*chunk_bytes,op.entry.bytes)-request.offset;
      }
    }
    return request;
  }
  static void advance(Operation& op,Phase phase)
  {
    op.phase=phase;
    op.range.reset();
    op.idle_attempts=0;
  }
  void receiveEntry(Operation& op,const mavlink_log_entry_t& value,double read_at)
  {
    const auto& range=*op.range;
    const Entry entry{value.size,value.time_utc};
    const bool whole=range.first==0 && range.last==UINT16_MAX;
    if(op.phase==Phase::List && whole && !op.total &&
       !value.id && !value.num_logs && !value.last_log_num && !entry.bytes && !entry.time_utc) {
      op.progress_at=read_at;
      beginFinish("complete","empty_catalog");
      return;
    }
    if(!value.id || value.id<range.first || value.id>range.last) return;
    if(value.last_log_num!=std::min(value.num_logs,range.last)) return; // A retired request's range.
    if(!value.num_logs || value.id>value.last_log_num || (op.total && value.num_logs!=*op.total)) {
      if(op.phase!=Phase::List) invalidateCatalog("remote_catalog_changed");
      beginFinish("incomplete","remote_catalog_changed");
      return;
    }
    if(op.phase!=Phase::List) {
      if(!entry.matches(op.entry)) {
        invalidateCatalog("remote_catalog_changed");
        beginFinish("incomplete","selected_entry_changed");
        return;
      }
      op.progress_at=read_at;
      if(op.phase==Phase::VerifyAfter) { beginFinish("complete","byte_coverage_and_catalog_verified"); return; }
      op.bytes.resize(op.entry.bytes);
      op.covered.resize((op.entry.bytes+chunk_bytes-1)/chunk_bytes);
      advance(op,Phase::Data);
      return;
    }
    if(value.num_logs>max_entries) { beginFinish("incomplete","catalog_size_limit"); return; }
    op.total=value.num_logs;
    const auto [row, inserted]=op.rows.emplace(value.id,entry);
    if(!row->second.matches(entry)) { beginFinish("incomplete","conflicting_log_entry"); return; }
    if(!inserted) return;
    op.progress_at=read_at;
    op.idle_attempts=0;
    if(op.rows.size()==*op.total) { beginFinish("complete","catalog_verified"); return; }
    if(!whole && op.rows.size()>=range.last-range.first+1u) {
      for(unsigned id=range.first;id<=range.last;++id)
        if(!op.rows.count(static_cast<uint16_t>(id))) return;
      advance(op,Phase::List);
    }
  }
  void receiveData(Operation& op,const mavlink_log_data_t& data,double read_at)
  {
    if(data.id!=op.selected) return;
    if(op.phase==Phase::Eof && data.ofs==op.entry.bytes) {
      if(data.count) {
        invalidateCatalog("remote_size_changed");
        beginFinish("incomplete","advertised_size_not_eof");
      } else advance(op,Phase::VerifyAfter);
      return;
    }
    if(data.ofs>=op.entry.bytes || data.ofs%chunk_bytes) return;
    const auto chunk=data.ofs/chunk_bytes;
    const bool duplicate=op.covered[chunk];
    const auto& range=*op.range;
    if(!duplicate && (op.phase!=Phase::Data || data.ofs<range.offset || data.ofs>=range.offset+range.count)) return;
    const auto expected=std::min(chunk_bytes,op.entry.bytes-data.ofs);
    if(data.count!=expected) {
      beginFinish("incomplete","invalid_or_short_log_chunk");
      return;
    }
    auto dest=op.bytes.begin()+data.ofs;
    if(duplicate) {
      if(!std::equal(data.data,data.data+expected,dest)) beginFinish("incomplete","conflicting_log_chunk");
      return;
    }
    std::copy_n(data.data,expected,dest);
    op.covered[chunk]=true;
    ++op.chunks_received;
    op.progress_at=read_at;
    op.idle_attempts=0;
    if(op.chunks_received==op.covered.size()) { advance(op,Phase::Eof); return; }
    for(uint32_t offset=range.offset;offset<range.offset+range.count;offset+=chunk_bytes)
      if(!op.covered[offset/chunk_bytes]) return;
    advance(op,Phase::Data);
  }
  void beginFinish(const char* status,const char* reason)
  {
    auto& op=*m_operation;
    m_services.flightLogFinished(op.id);
    op.outcome=status;
    op.reason=reason;
    op.pending.reset();
    advance(op,Phase::Cleanup);
    if(!op.remote_possible) { settle("no_local_send"); return; }
    op.cleanup_until=std::min(m_services.now()+cleanup_grace,op.expires+2);
  }
  LocalFile releaseFile(const Operation& op) const
  {
    LocalFile result;
    const int dir=m_directory.get();
    if(!sameDirectory(dir,m_path)) { result.reason="local_directory_changed"; return result; }
    const auto stage="."+op.file+".partial";
    FileDescriptor staged(openat(dir,stage.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW|O_CLOEXEC,0600));
    if(staged.get()<0) { result.reason="local_stage_unavailable"; return result; }
    struct stat held{}, named{};
    if(writeAll(staged.get(),op.bytes) && syncFile(staged.get()) &&
       fstat(staged.get(),&held)==0 && fstatat(dir,stage.c_str(),&named,AT_SYMLINK_NOFOLLOW)==0 &&
       held.st_dev==named.st_dev && held.st_ino==named.st_ino && held.st_nlink==1 &&
       held.st_size==static_cast<off_t>(op.bytes.size()) && sameDirectory(dir,m_path)) {
      result.released=linkat(dir,stage.c_str(),dir,op.file.c_str(),0)==0;
      if(!result.released) result.reason="local_destination_unavailable";
    }
    result.staging_removed=unlinkat(dir,stage.c_str(),0)==0;
    if(result.released) result.directory_synced=syncFile(dir);
    return result;
  }
  void settle(const char* cleanup)
  {
    auto& op=*m_operation;
    LocalFile file;
    if(op.outcome=="complete") {
      if(std::string(cleanup)!="local_end_accepted_unacknowledged" || !groundReady()) {
        op.outcome="incomplete";
        op.reason="log_cleanup_or_ground_not_confirmed_locally";
      } else if(op.action=="list") publishCatalog(op);
      else {
        file=releaseFile(op);
        if(!file.released) { op.outcome="incomplete"; op.reason=file.reason; }
      }
    }
    report(op,op.outcome,op.reason,cleanup,file);
    m_operation.reset();
  }
  Services& m_services;
  FileDescriptor m_directory;
  std::string m_path;
  double m_boundary;
  unsigned m_timeout=60, m_max_bytes=max_file_bytes;
  std::set<std::string> m_seen;
  std::optional<Catalog> m_catalog;
  std::optional<Operation> m_operation;
};
}

extern "C" Plugin* mavmoos_create_plugin(unsigned abi,Services* services)
{
  return abi==plugin_abi && services?new FlightLogs(*services):nullptr;
}
extern "C" void mavmoos_destroy_plugin(Plugin* plugin) { delete plugin; }
} // namespace mavmoos::runtime

#include "Runtime.h"
#include "CopterFrame.h"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace mavmoos::runtime {
namespace {
constexpr char marker_name[]="auto-may-continue";
constexpr double heartbeat_age=2, request_age=2;
enum class Marker { Missing, Present, Unavailable };

bool token(const std::string& value,std::size_t limit)
{
  return !value.empty() && value.size()<=limit &&
    std::all_of(value.begin(),value.end(),[](unsigned char c) {
      return (c>='a'&&c<='z') || (c>='A'&&c<='Z') || (c>='0'&&c<='9') ||
        c=='_' || c=='-' || c=='.';
    });
}
bool privateOwner(const struct stat& info)
{
  return info.st_uid==geteuid() && !(info.st_mode&(S_IRWXG|S_IRWXO));
}
bool sameDirectory(int fd,const std::string& name)
{
  struct stat held{}, named{};
  return fstat(fd,&held)==0 && lstat(name.c_str(),&named)==0 &&
    S_ISDIR(held.st_mode) && S_ISDIR(named.st_mode) && privateOwner(held) &&
    held.st_dev==named.st_dev && held.st_ino==named.st_ino;
}
Marker marker(int directory)
{
  struct stat info{};
  if(fstatat(directory,marker_name,&info,AT_SYMLINK_NOFOLLOW)!=0)
    return errno==ENOENT ? Marker::Missing : Marker::Unavailable;
  return S_ISREG(info.st_mode) && privateOwner(info) && info.st_nlink==1
    ? Marker::Present : Marker::Unavailable;
}
bool syncFile(int fd)
{
  int result;
  do { result=fsync(fd); } while(result<0 && errno==EINTR);
  return result==0;
}
bool createMarker(int directory)
{
  const int fd=openat(directory,marker_name,O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW|O_CLOEXEC,0600);
  if(fd<0) return false;
  const bool synced=syncFile(fd);
  const bool closed=close(fd)==0;
  // A failed admission keeps any created marker for the next process.
  return synced && closed && syncFile(directory);
}
std::map<std::string,std::string> handbackFields(const std::string& text)
{
  if(text.empty() || text.size()>256 || text.back()==',') return {};
  std::istringstream input(text);
  std::map<std::string,std::string> fields;
  std::string part;
  while(std::getline(input,part,',')) {
    const auto equal=part.find('=');
    if(equal==std::string::npos ||
       !fields.emplace(part.substr(0,equal),part.substr(equal+1)).second) return {};
  }
  if(fields.size()!=4 || fields["schema"]!="1" || fields["action"]!="handback" ||
     !token(fields["id"],96) || !token(fields["session"],40)) return {};
  return fields;
}
}

Runtime::AutoRecovery::~AutoRecovery()
{
  if(directory_fd>=0) close(directory_fd);
}

void Runtime::configureAutoRecovery(const std::string& directory,const std::string& process_session)
{
  const bool copter=m_backend && m_backend->profile()=="arducopter" && supportedCopterType(m_expected_type) &&
    m_backend->identityCompatible(m_expected_type,m_expected_autopilot);
  if(m_auto_recovery || !m_startup_blocked || !m_module_configuration_complete ||
     !m_configuration_valid || !m_clock || !m_simulation || m_shadow ||
     (!copter && !px4RoverPolicy() && !quadPlaneProfile() && !arduRoverMissionPolicy() && !planeProfile()))
    throw std::invalid_argument("mission recovery requires checked startup of an owned mission execution provider");
  if(directory.empty() || directory.front()!='/' || directory.size()>4096 ||
     directory.find('\0')!=std::string::npos || directory.find_first_of("\r\n")!=std::string::npos ||
     directory.back()=='/' || !token(process_session,40))
    throw std::invalid_argument("AUTO recovery requires an absolute private directory and a process session");
  auto recovery=std::make_unique<AutoRecovery>();
  recovery->directory=directory;
  recovery->session=process_session;
  recovery->directory_fd=open(directory.c_str(),O_RDONLY|O_DIRECTORY|O_NOFOLLOW|O_CLOEXEC);
  if(recovery->directory_fd<0 || !sameDirectory(recovery->directory_fd,directory))
    throw std::invalid_argument("AUTO recovery directory must exist and be private to the current user");
  if(flock(recovery->directory_fd,LOCK_EX|LOCK_NB)!=0 ||
     !sameDirectory(recovery->directory_fd,directory))
    throw std::invalid_argument("AUTO recovery directory cannot be exclusively locked");
  const auto current=marker(recovery->directory_fd);
  if(current==Marker::Unavailable)
    throw std::invalid_argument("AUTO recovery uncertainty marker is unavailable or unsafe");
  recovery->marker_present=current==Marker::Present;
  recovery->state=recovery->marker_present ? AutoRecovery::State::Unresolved : AutoRecovery::State::AwaitingHeartbeat;
  recovery->reason=recovery->marker_present ? "restart_uncertainty_marker" : "waiting_for_fresh_target_heartbeat";
  recovery->mail_after=recovery->observation_after=m_now;
  m_auto_recovery=std::move(recovery);
}

bool Runtime::autoRecoveryBlocksEffects() const
{
  return m_auto_recovery && m_auto_recovery->state!=AutoRecovery::State::Ready;
}

const char* Runtime::autoRecoveryState() const
{
  if(!m_auto_recovery) return "DISABLED";
  switch(m_auto_recovery->state) {
  case AutoRecovery::State::AwaitingHeartbeat: return "WAIT_HEARTBEAT";
  case AutoRecovery::State::Ready: return m_execution ? "OWNED" : "READY";
  case AutoRecovery::State::Unresolved: return "UNRESOLVED";
  case AutoRecovery::State::StorageFault: return "STORAGE_FAULT";
  }
  return "STORAGE_FAULT";
}

bool Runtime::freshNonAutoRecoveryEvidence() const
{
  return m_auto_recovery && m_vehicle.heartbeat_valid && !observedExecutionMode() &&
    m_auto_recovery->observed_mode==m_vehicle.mode &&
    m_auto_recovery->observed_at==m_vehicle.heartbeat_at &&
    m_auto_recovery->observed_at>m_auto_recovery->observation_after &&
    fresh(m_vehicle.heartbeat_at,heartbeat_age) &&
    (!quadPlaneProfile() || quadPlaneGroundHandback(m_auto_recovery->observation_after)) &&
    (!px4RoverProfile() || roverHoldStopped(m_auto_recovery->observation_after)) &&
    (!arduRoverMissionPolicy() || arduRoverMissionHandback(m_auto_recovery->observation_after));
}

void Runtime::observeAutoRecoveryHeartbeat(const mavlink_message_t& message)
{
  if(!m_auto_recovery || message.sysid!=1 || message.compid!=1) return;
  auto& recovery=*m_auto_recovery;
  if(observedExecutionMode()) {
    recovery.observation_after=std::max(recovery.observation_after,m_now);
    recovery.observed_at=-1;
    if(m_execution || recovery.state==AutoRecovery::State::StorageFault ||
       recovery.state==AutoRecovery::State::Unresolved) return;
    const bool previous_session_work=recovery.state==AutoRecovery::State::Ready;
    recovery.state=AutoRecovery::State::Unresolved;
    recovery.reason="unowned_auto_observed";
    if(!sameDirectory(recovery.directory_fd,recovery.directory) ||
       !createMarker(recovery.directory_fd)) {
      autoRecoveryStorageFault("unowned_auto_marker_write_failed");
      return;
    }
    recovery.marker_present=true;
    if(previous_session_work) disconnect(m_now,"unowned_auto_observed");
    return;
  }
  if(m_vehicle.heartbeat_at<=recovery.observation_after ||
     m_vehicle.heartbeat_at<=recovery.observed_at ||
     !fresh(m_vehicle.heartbeat_at,heartbeat_age)) return;
  recovery.observed_at=m_vehicle.heartbeat_at;
  recovery.observed_mode=m_vehicle.mode;
  if(recovery.state==AutoRecovery::State::AwaitingHeartbeat) {
    recovery.state=AutoRecovery::State::Ready;
    recovery.reason="fresh_non_auto_startup";
  }
}

void Runtime::autoRecoveryStorageFault(const char* reason)
{
  auto& recovery=*m_auto_recovery;
  recovery.state=AutoRecovery::State::StorageFault;
  recovery.reason=reason;
  revokeNavigation(reason);
  m_hold.reset();
  invalidateStoredMissionProof(reason);
  publishAutoRecoveryStatus();
}

bool Runtime::armAutoRecoveryMarker(double original_deadline)
{
  if(!m_auto_recovery) return true;
  auto& recovery=*m_auto_recovery;
  if(autoRecoveryBlocksEffects()) return false;
  if(!sameDirectory(recovery.directory_fd,recovery.directory) ||
     !createMarker(recovery.directory_fd)) {
    autoRecoveryStorageFault("execution_marker_write_failed");
    return false;
  }
  recovery.marker_present=true;
  // Storage work cannot renew the original operation or observation ages.
  double sample=std::numeric_limits<double>::quiet_NaN();
  try { sample=m_clock(); } catch(...) {} // The marker stays held on a clock failure.
  if(std::isfinite(sample) && sample>=m_now) {
    m_now=sample;
    refreshValidity();
    const auto proof=storedMissionProof();
    if(m_now<original_deadline && proof.valid && missionEntryReady(proof.items)) return true;
  }
  recovery.state=AutoRecovery::State::Unresolved;
  recovery.reason="execution_admission_lost_during_marker_write";
  recovery.observation_after=m_now;
  recovery.observed_at=-1;
  revokeNavigation(recovery.reason);
  m_hold.reset();
  invalidateStoredMissionProof(recovery.reason);
  publishAutoRecoveryStatus();
  return false;
}

bool Runtime::clearAutoRecoveryMarker()
{
  if(!m_auto_recovery) return true;
  auto& recovery=*m_auto_recovery;
  if(!sameDirectory(recovery.directory_fd,recovery.directory) ||
     marker(recovery.directory_fd)!=Marker::Present ||
     unlinkat(recovery.directory_fd,marker_name,0)!=0 ||
     !syncFile(recovery.directory_fd)) {
    autoRecoveryStorageFault("uncertainty_marker_clear_failed");
    return false;
  }
  recovery.marker_present=false;
  return true;
}

void Runtime::handleAutoRecoveryRequest(const Mail& mail)
{
  auto& recovery=*m_auto_recovery;
  const auto fields=mail.numeric ? std::map<std::string,std::string>{} : handbackFields(mail.text);
  const auto id=fields.empty() ? "none" : fields.at("id");
  auto reply=[&](const char* status,const char* reason) {
    publish("MAVMOOS_AUTO_RECOVERY_RESULT","schema=1,id="+id+",session="+recovery.session+
      ",status="+status+",reason="+reason);
  };
  if(fields.empty()) { reply("REJECTED","invalid_request"); return; }
  if(fields.at("session")!=recovery.session) { reply("REJECTED","wrong_process_session"); return; }
  if(!std::isfinite(mail.published_at) || mail.published_at<=recovery.mail_after ||
     mail.published_at>m_now || m_now-mail.published_at>request_age) {
    reply("REJECTED","stale_or_future_request");
    return;
  }
  if(recovery.state!=AutoRecovery::State::Unresolved || m_execution) {
    reply("REJECTED","recovery_handback_unavailable");
    return;
  }
  if(!freshNonAutoRecoveryEvidence() || mail.published_at<=recovery.observed_at) {
    reply("REJECTED","fresh_prior_non_auto_observation_required");
    return;
  }
  if(!clearAutoRecoveryMarker()) { reply("REJECTED","uncertainty_marker_clear_failed"); return; }
  // Keep the fence through plugin reset. New effects require new input and acquisition.
  disconnect(m_now,"auto_recovery_handback");
  recovery.mail_after=m_now;
  recovery.state=AutoRecovery::State::Ready;
  recovery.reason="explicit_handback";
  publishAutoRecoveryStatus();
  reply("RELEASED","explicit_handback_no_grant");
}

void Runtime::publishAutoRecoveryStatus()
{
  if(!m_auto_recovery) return;
  auto& recovery=*m_auto_recovery;
  if(recovery.last_report<0) publishStoredMissionProof("fresh_process_session");
  std::ostringstream out;
  out<<std::setprecision(17)<<"schema=1,session="<<recovery.session<<",state="<<autoRecoveryState()
     <<",reason="<<recovery.reason<<",effects_blocked="<<(autoRecoveryBlocksEffects()?"true":"false")
     <<",marker="<<(recovery.state==AutoRecovery::State::StorageFault?"unknown":recovery.marker_present?"present":"absent")
     <<",ready_for_handback="<<(recovery.state==AutoRecovery::State::Unresolved && freshNonAutoRecoveryEvidence()?"true":"false")
     <<",observed_at="<<recovery.observed_at
     <<",remote_execution_may_continue="<<((recovery.marker_present || autoRecoveryBlocksEffects() || m_execution)?"true":"false");
  const auto status=out.str();
  if(status!=recovery.last_status || m_now-recovery.last_report>=1) {
    publish("MAVMOOS_AUTO_RECOVERY_STATUS",status);
    recovery.last_status=status;
    recovery.last_report=m_now;
  }
}
} // namespace mavmoos::runtime

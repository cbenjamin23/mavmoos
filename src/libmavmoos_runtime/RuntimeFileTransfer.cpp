#include "Runtime.h"
#include "../libmavmoos_ftp/FtpTransaction.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
bool alnum(unsigned char c)
{
  return (c>='a'&&c<='z') || (c>='A'&&c<='Z') || (c>='0'&&c<='9');
}
bool token(const std::string& s)
{
  return !s.empty() && s.size()<=ftp::max_basename && alnum(s.front()) &&
    std::all_of(s.begin(),s.end(),[](unsigned char c){return alnum(c)||c=='_'||c=='-'||c=='.';});
}
bool allowedFile(const std::string& path)
{
  const std::string prefix=std::string(ftp::directory)+"/";
  return path.compare(0,prefix.size(),prefix)==0 && token(path.substr(prefix.size()));
}
}

void Runtime::enableFileTransferBench(FileTransferPathGuard path_guard)
{
  auto names=moduleNames();
  std::sort(names.begin(),names.end());
  if(!m_startup_blocked || !m_module_configuration_complete || m_files_bench || m_backend || !path_guard ||
     !m_configuration_valid || !m_simulation || m_shadow ||
     m_expected_type!=MAV_TYPE_QUADROTOR || m_expected_autopilot!=MAV_AUTOPILOT_ARDUPILOTMEGA ||
     names!=std::vector<std::string>{"mavlink_files","telemetry"})
    throw std::invalid_argument("active mavlink_files requires the isolated Copter SITL bench: plugins=telemetry,mavlink_files");
  m_file_path_guard=std::move(path_guard);m_files_bench=true;
}

bool Runtime::fileTransferGroundSafe() const
{
  return m_files_bench && !m_backend && m_simulation && !m_shadow && m_configuration_valid &&
    !autoRecoveryBlocksEffects() && m_vehicle.heartbeat_valid && fresh(m_vehicle.heartbeat_at,2) &&
    !m_vehicle.armed && m_vehicle.landed_valid && fresh(m_vehicle.landed_at,2) &&
    m_vehicle.landed_state==MAV_LANDED_STATE_ON_GROUND;
}

bool Runtime::fileTransferEligible(const FileTransferRequest& r) const
{
  if(!fileTransferGroundSafe() || !token(r.id) || !token(r.operation_id) ||
     !std::isfinite(r.expires) || !std::isfinite(r.operation_expires) ||
     r.expires<=m_now || r.expires-m_now>2 || r.session!=ftp::client_session || (r.sequence&1)!=0)
    return false;
  if(m_file_transfer && (m_file_transfer->id!=r.operation_id ||
     m_file_transfer->session!=r.session || m_file_transfer->expires!=r.operation_expires))
    return false;
  if(r.action==FileTransferAction::Terminate)
    return m_file_transfer && m_file_transfer->read && m_file_transfer->open_handed_off &&
      r.expires<=r.operation_expires+ftp::cleanup_seconds && r.path.empty() && !r.count && !r.offset;
  if(r.expires>r.operation_expires || r.operation_expires-m_now>ftp::max_operation_seconds ||
     (m_file_transfer && m_file_transfer->cleanup_only)) return false;
  switch(r.action) {
  case FileTransferAction::List:
    return (!m_file_transfer || !m_file_transfer->read) &&
      r.path==ftp::directory && !r.count && r.offset<=ftp::max_list_offset;
  case FileTransferAction::OpenReadOnly:
    return (!m_file_transfer || m_file_transfer->read) && allowedFile(r.path) && !r.count && !r.offset;
  case FileTransferAction::Read:
    return m_file_transfer && m_file_transfer->open_handed_off && r.path.empty() &&
      r.count>0 && r.count<=ftp::data_size && r.offset<=ftp::max_file_size &&
      (r.offset==ftp::max_file_size?r.count==1:r.count<=ftp::max_file_size-r.offset);
  default:
    return false;
  }
}

bool Runtime::fileTransfer(const FileTransferRequest& r)
{
  if(configurationEffectAttempt()) return false;
  refreshFileTransfer();
  if(!fileTransferEligible(r)) return false;
  if(r.action==FileTransferAction::Terminate) {
    cancelFileTransfer(r.operation_id);
    if(m_file_pending || m_file_handoff) return false;
  } else if(m_file_pending || m_file_handoff) return false;
  if(!m_file_transfer)
    m_file_transfer=FileTransferLease{r.operation_id,r.operation_expires,r.session,r.action==FileTransferAction::OpenReadOnly};
  m_file_pending=r;
  return true;
}

void Runtime::retireFileTransferPacket(const std::string& operation,const std::string& packet)
{
  if(configurationEffectAttempt() || !m_file_transfer || m_file_transfer->id!=operation) return;
  if(m_file_pending && m_file_pending->id==packet) m_file_pending.reset();
  if(m_file_handoff && m_file_handoff->id==packet) m_file_handoff.reset();
}

void Runtime::cancelFileTransfer(const std::string& id)
{
  if(configurationEffectAttempt() || !m_file_transfer || m_file_transfer->id!=id) return;
  if(m_file_pending && m_file_pending->action!=FileTransferAction::Terminate) m_file_pending.reset();
  if(m_file_handoff && m_file_handoff->action!=FileTransferAction::Terminate) m_file_handoff.reset();
  if(m_file_transfer->open_handed_off) m_file_transfer->cleanup_only=true;
  else resetFileTransfer();
}

void Runtime::finishFileTransfer(const std::string& id)
{
  if(configurationEffectAttempt() || !m_file_transfer || m_file_transfer->id!=id) return;
  resetFileTransfer();
}

void Runtime::resetFileTransfer()
{
  m_file_transfer.reset();m_file_pending.reset();m_file_handoff.reset();
}

void Runtime::refreshFileTransfer()
{
  if(!m_file_transfer) return;
  if(!fileTransferGroundSafe() || m_now>=m_file_transfer->expires+ftp::cleanup_seconds) resetFileTransfer();
  else if(m_now>=m_file_transfer->expires && !m_file_transfer->cleanup_only) {
    const auto id=m_file_transfer->id;
    cancelFileTransfer(id);
  }
}

void Runtime::flushFileTransfer()
{
  const auto r=*m_file_pending;
  m_file_pending.reset();
  m_file_handoff=r;
  ftp::Payload payload{};
  payload[0]=static_cast<uint8_t>(r.sequence);
  payload[1]=static_cast<uint8_t>(r.sequence>>8);
  payload[2]=r.session;
  payload[3]=static_cast<uint8_t>(r.action);
  payload[4]=r.path.empty()?r.count:static_cast<uint8_t>(r.path.size());
  for(unsigned i=0;i<4;++i) payload[8+i]=static_cast<uint8_t>(r.offset>>(8*i));
  std::copy(r.path.begin(),r.path.end(),payload.begin()+ftp::header_size);
  mavlink_message_t msg{};
  mavlink_msg_file_transfer_protocol_pack_status(245,191,&m_codec,&msg,0,1,1,payload.data());
  // Filesystem proof may take time. Only then sample the clock and invoke the
  // shared owned-endpoint, absolute-deadline and live ground-state checks.
  bool private_source=false;
  if(fileTransferEligible(r)) {
    try { private_source=m_file_path_guard(r.action,r.path); }
    catch(...) { private_source=false; }
  }
  if(!sampleClock()) return;
  const auto live=[this,&r] {
    return m_file_handoff && m_file_handoff->id==r.id && fileTransferEligible(r);
  };
  const bool accepted=private_source && live() && send(msg,r.expires,live);
  m_file_handoff.reset();
  if(accepted && r.action==FileTransferAction::OpenReadOnly && m_file_transfer &&
     m_file_transfer->id==r.operation_id) m_file_transfer->open_handed_off=true;
  const bool clock_ok=sampleClock();
  std::ostringstream at;at<<std::setprecision(17)<<m_now;
  publish("MAVMOOS_FILE_TX_RESULT","id="+r.id+",operation_id="+r.operation_id+
    ",opcode="+std::to_string(static_cast<unsigned>(r.action))+",sequence="+std::to_string(r.sequence)+
    ",session="+std::to_string(r.session)+",offset="+std::to_string(r.offset)+",count="+std::to_string(payload[4])+
    ",at="+at.str()+",status="+(accepted?"LOCAL_ACCEPTED":"NOT_ACCEPTED")+
    ",late="+(m_now>=r.expires?"true":"false")+",clock_valid="+(clock_ok?"true":"false"));
  if(clock_ok) m_host.fileTransferSent(r,m_now,accepted);
}
}

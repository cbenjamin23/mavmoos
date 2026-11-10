#include "Runtime.h"
#include "ControlReplayJournal.h"
#include <stdexcept>

namespace mavmoos::runtime {
void Runtime::configureControlReplay(const std::string& directory,const std::string& route,bool create)
{
  if(m_control_replay_enabled || !m_startup_blocked || !m_module_configuration_complete ||
     !m_configuration_valid || !m_simulation || m_shadow || !m_backend || route.empty())
    throw std::invalid_argument("control replay requires an owned control profile during startup");
  m_control_replay_enabled=true;
  m_control_replay=std::make_unique<ControlReplayJournal>(directory,route,create);
  publish("MAVMOOS_CONTROL_REPLAY_STATUS","state=ready,used="+std::to_string(m_control_replay->size())+
          ",capacity="+std::to_string(ControlReplayJournal::capacity));
}

bool Runtime::controlRequestSeen(const std::string& id) const
{
  return m_control_replay && m_control_replay->seen(id);
}

ReplayResult Runtime::reserveControlRequest(const std::string& id)
{
  if(configurationEffectAttempt()) return ReplayResult::Unavailable;
  if(!m_control_replay_enabled) return ReplayResult::Reserved;
  if(!m_control_replay) return ReplayResult::Unavailable;
  const auto reserved=m_control_replay->reserve(id);
  ReplayResult result=ReplayResult::Unavailable;
  switch(reserved) {
  case ControlReplayJournal::Result::Reserved: result=ReplayResult::Reserved; break;
  case ControlReplayJournal::Result::Seen: result=ReplayResult::Seen; break;
  case ControlReplayJournal::Result::Full: result=ReplayResult::Full; break;
  case ControlReplayJournal::Result::Unavailable: break;
  }
  publish("MAVMOOS_CONTROL_REPLAY_STATUS",std::string("state=")+
    (result==ReplayResult::Full?"full":result==ReplayResult::Unavailable?"unavailable":"ready")+
    ",used="+std::to_string(m_control_replay->size())+",capacity="+std::to_string(ControlReplayJournal::capacity));
  return result;
}

ReplayResult Runtime::reserveControlEvent()
{
  if(!m_control_replay_enabled) return ReplayResult::Reserved;
  return m_replay_event.empty()?ReplayResult::Unavailable:reserveControlRequest(m_replay_event);
}
}

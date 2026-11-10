#pragma once
#include "Plugin.h"

namespace mavmoos::runtime {
enum class ReplayResult { Reserved, Seen, Full, Unavailable };

// Separate from Services so existing observer and command plugin ABIs stay valid.
class ControlReplayService {
public:
  virtual ~ControlReplayService()=default;
  virtual bool controlReplayEnabled() const=0;
  virtual bool controlRequestSeen(const std::string&) const=0;
  virtual ReplayResult reserveControlRequest(const std::string&)=0;
  virtual ReplayResult reserveControlEvent()=0;
};

inline ReplayResult reserveControlRequest(Services& host,const std::string& id)
{
  auto* replay=dynamic_cast<ControlReplayService*>(&host);
  return replay?replay->reserveControlRequest(id):ReplayResult::Reserved;
}
inline ReplayResult reserveControlEvent(Services& host)
{
  auto* replay=dynamic_cast<ControlReplayService*>(&host);
  return replay?replay->reserveControlEvent():ReplayResult::Reserved;
}
inline bool controlRequestSeen(Services& host,const std::string& id)
{
  auto* replay=dynamic_cast<ControlReplayService*>(&host);
  return replay&&replay->controlRequestSeen(id);
}
inline const char* replayReason(ReplayResult result)
{
  switch(result) {
  case ReplayResult::Reserved: return "reserved";
  case ReplayResult::Seen: return "native_id_already_recorded";
  case ReplayResult::Full: return "native_replay_history_full";
  case ReplayResult::Unavailable: return "native_replay_unavailable";
  }
  return "native_replay_unavailable";
}
inline const char* replayRejection(ReplayResult result)
{ return result==ReplayResult::Seen?"DUPLICATE":"REJECTED"; }
}

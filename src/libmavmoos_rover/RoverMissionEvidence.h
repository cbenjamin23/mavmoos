#pragma once

#include "ArduRoverMissionPolicy.h"
#include <array>
#include <deque>
#include <optional>

namespace mavmoos::runtime {

class RoverMissionEvidence {
public:
  explicit RoverMissionEvidence(bool boat):m_boat(boat) {}
  bool configured(double now) const;
  bool settled(double now,double after) const;
  void observe(const mavlink_message_t&,const MessageTime&,double now);
  void readConfiguration(Services&);
  void parameterSent(const ParameterRequest&,double,bool);
  void reset(double);
private:
  struct Sample {
    double at,latitude,longitude;
    uint32_t boot_ms;
  };
  struct Read {
    ParameterRequest request;
    std::size_t index;
    double sent_at=-1;
  };
  void observePosition(const mavlink_message_t&,double);
  void observeParameter(const mavlink_message_t&,const MessageTime&);
  bool m_boat;
  double m_boundary=0,m_last_at=0,m_next_read=0;
  uint64_t m_read_sequence=0;
  std::array<double,6> m_verified{};
  std::optional<uint32_t> m_boot_ms;
  std::optional<Read> m_read;
  std::deque<Sample> m_samples;
  bool m_conflict=false;
};

} // namespace mavmoos::runtime

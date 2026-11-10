#pragma once

#include <algorithm>
#include <optional>
#include <stdexcept>
#include <string>

namespace mavmoos::test {
// One fixed probe-clock deadline. Tokens are opaque marker contents, never a
// conversion between producer and coordinator clock epochs. Pumping remains
// mandatory even when an acknowledgement is already available.
template <typename Read,typename Pump,typename Now,typename Pause>
std::optional<double> waitForFinishAck(const std::string& token,double deadline,
                                     Read read,Pump pump,Now now,Pause pause)
{
  for(;;) {
    if(now()>=deadline)return std::nullopt;
    pump();
    if(now()>=deadline)return std::nullopt;
    const auto acknowledgement=read();
    const double observed=now();
    if(observed>=deadline)return std::nullopt;
    if(acknowledgement) {
      if(*acknowledgement!=token)throw std::runtime_error("finish acknowledgement token mismatch");
      return observed;
    }
    const double remaining=deadline-now();
    if(remaining<=0)return std::nullopt;
    pause(std::min(.01,remaining));
  }
}
} // namespace mavmoos::test

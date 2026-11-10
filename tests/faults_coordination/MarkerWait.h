#pragma once

#include <algorithm>

namespace mavmoos::test {
// Local coordinator markers need no MOOS connection or cached vehicle state.
// Keep one fixed deadline and take a final predicate sample after the last
// bounded pause. This observes a marker at that final poll; it does not infer
// its creation time or make a hard scheduling guarantee after OS oversleep.
template <typename Exists, typename Now, typename Pause>
bool waitForMarker(Exists exists, double timeout, Now now, Pause pause)
{
  const double deadline=now()+timeout;
  for(;;) {
    if(exists())return true;
    const double remaining=deadline-now();
    if(remaining<=0)return false;
    pause(std::min(.02,remaining));
  }
}
} // namespace mavmoos::test

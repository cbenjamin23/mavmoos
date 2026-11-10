#pragma once
#include "Plugin.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mavmoos::runtime {

inline constexpr std::size_t maximum_fence_vertices = 16;
inline constexpr int64_t maximum_fence_span_e7 = 180000;

inline bool fenceVertex(const MissionItem& item, std::size_t count)
{
  return count >= 3 && count <= maximum_fence_vertices && item.seq < count &&
    item.command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION &&
    item.frame == MAV_FRAME_GLOBAL && item.current == 0 && item.autocontinue == 0 &&
    item.params[0] == count && item.params[1] == 0 && item.params[2] == 0 && item.params[3] == 0 &&
    std::abs(int64_t(item.x)) <= 850000000 && std::abs(int64_t(item.y)) <= 1800000000 &&
    item.z == 0;
}

inline bool editableFence(const std::vector<MissionItem>& polygon)
{
  if (polygon.empty())
    return true;
  const auto count = polygon.size();
  for (std::size_t i = 0; i < count; ++i) {
    if (!fenceVertex(polygon[i], count) || polygon[i].seq != i)
      return false;
  }
  const auto latitude = std::minmax_element(polygon.begin(), polygon.end(),
    [](const MissionItem& a, const MissionItem& b) { return a.x < b.x; });
  const auto longitude = std::minmax_element(polygon.begin(), polygon.end(),
    [](const MissionItem& a, const MissionItem& b) { return a.y < b.y; });
  if (int64_t(latitude.second->x) - latitude.first->x > maximum_fence_span_e7 ||
      int64_t(longitude.second->y) - longitude.first->y > maximum_fence_span_e7)
    return false;

  // A strict convex polygon puts every non-endpoint on the same side of
  // every oriented edge. The span bound makes each determinant fit in int64_t.
  int orientation = 0;
  for (std::size_t i = 0; i < count; ++i) {
    const auto next = (i + 1) % count;
    const auto& a = polygon[i];
    const auto& b = polygon[next];
    for (std::size_t j = 0; j < count; ++j) {
      if (j == i || j == next)
        continue;
      const auto& c = polygon[j];
      const int64_t cross = (int64_t(b.x) - a.x) * (int64_t(c.y) - a.y) -
        (int64_t(b.y) - a.y) * (int64_t(c.x) - a.x);
      if (cross == 0)
        return false;
      const int sign = cross > 0 ? 1 : -1;
      if (orientation && orientation != sign)
        return false;
      orientation = sign;
    }
  }
  return true;
}

} // namespace mavmoos::runtime

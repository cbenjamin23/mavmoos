#include "PassiveRig.h"

#include <algorithm>
#include <cstring>

using namespace mavmoos::test;

namespace {

mavlink_message_t mode(PassiveRig& rig, uint8_t count, uint8_t index, uint32_t custom,
                       const char* name, uint8_t sequence, uint8_t component = 1)
{
  mavlink_available_modes_t value{};
  value.number_modes = count;
  value.mode_index = index;
  value.custom_mode = custom;
  value.properties = index == 2 ? MAV_MODE_PROPERTY_ADVANCED : 0;
  std::copy_n(name, std::strlen(name), value.mode_name);
  value.seq = sequence;
  return rig.encode(value, mavlink_msg_available_modes_encode_status, 1, component);
}

mavlink_message_t current(PassiveRig& rig, uint32_t active, uint32_t intended)
{
  mavlink_current_mode_t value{};
  value.custom_mode = active;
  value.intended_custom_mode = intended;
  return rig.encode(value, mavlink_msg_current_mode_encode_status);
}

mavlink_message_t monitor(PassiveRig& rig, uint8_t sequence)
{
  mavlink_available_modes_monitor_t value{};
  value.seq = sequence;
  return rig.encode(value, mavlink_msg_available_modes_monitor_encode_status);
}

void exercise(PassiveRig& rig)
{
  require(rig.plugin->name() == "mode_catalog", "wrong mode module");
  require(rig.plugin->configureModule({{"sample_timeout", "1"}}).status == ModuleConfigStatus::Accepted,
          "sample age configuration rejected");
  rig.receive(mode(rig, 2, 1, 17, "Hold", 254), 100.02);
  rig.capture("prebind", 100.06, 2);
  rig.heartbeat(100.10);
  rig.receive(mode(rig, 2, 2, 93, "Offboard", 254), 100.12, 100.15);
  rig.receive(mode(rig, 2, 1, 17, "Foreign", 254, 8), 100.16);
  rig.receive(current(rig, 17, 93), 100.18);
  rig.capture("partial", 100.20, 2);
  rig.receive(mode(rig, 2, 1, 17, "Hold", 254), 100.22);
  rig.receive(monitor(rig, 254), 100.24);
  rig.capture("complete", 100.31, 2);
  rig.receive(monitor(rig, 255), 100.42);
  rig.receive(mode(rig, 2, 1, 17, "Old hold", 254), 100.43);
  rig.capture("revision_change", 100.45, 2);
  rig.receive(mode(rig, 1, 1, 0, "Legacy", 255), 100.50);
  rig.receive(mode(rig, 1, 1, 0, "Conflicting", 255), 100.52);
  rig.capture("conflict", 100.57, 2);
  rig.receive(monitor(rig, 0), 100.62);
  rig.receive(mode(rig, 1, 0, 17, "Bad index", 0), 100.64);
  rig.receive(mode(rig, 65, 1, 17, "Oversized", 0), 100.66);
  rig.receive(mode(rig, 1, 1, 17, "Bad\nlabel", 0), 100.68);
  rig.receive(mode(rig, 1, 1, 17, "Untimed", 0), 100.70);
  rig.capture("unversioned", 100.72, 2);
  rig.receive(monitor(rig, 255), 100.77);
  rig.receive(mode(rig, 1, 1, 17, "Untimed", 0), 100.78);
  rig.heartbeat(100.80, 55);
  rig.capture("newer_heartbeat", 100.84, 2);
  rig.receive(mode(rig, 1, 1, 17, "Untimed", 0), 101.70);
  rig.heartbeat(101.73, 55);
  rig.capture("expired_snapshot", 101.75, 2);
  rig.capture("heartbeat_loss", 104, 2);
  rig.heartbeat(104.20, 0, 0);
  rig.capture("no_custom_flag", 104.24, 2);
  rig.heartbeat(104.32, 0);
  rig.capture("numeric_zero", 104.38, 2);
  rig.capture("clock_fault", 104.36, 2);
}

} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "pass one mode plugin path");
    PassiveRig rig(argv[1]);
    exercise(rig);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

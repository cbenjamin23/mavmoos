#include "../mode_catalog/PassiveRig.h"

#include <algorithm>
#include <cstring>
#include <limits>

using namespace mavmoos::test;

namespace {

constexpr uint8_t source_component = 236;

mavlink_message_t identity(PassiveRig& rig, const char* identifier, uint8_t component = source_component,
                           bool relayed = false)
{
  mavlink_open_drone_id_basic_id_t value{};
  value.target_system = 1;
  value.target_component = source_component;
  value.id_or_mac[0] = relayed ? 5 : 0;
  value.id_type = MAV_ODID_ID_TYPE_SERIAL_NUMBER;
  value.ua_type = MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR;
  std::copy_n(identifier, std::strlen(identifier), value.uas_id);
  return rig.encode(value, mavlink_msg_open_drone_id_basic_id_encode_status, 1, component);
}

mavlink_open_drone_id_location_t position()
{
  mavlink_open_drone_id_location_t value{};
  value.latitude = 473977436;
  value.longitude = 85455942;
  value.altitude_barometric = -1000;
  value.altitude_geodetic = 489;
  value.height = 17;
  value.height_reference = MAV_ODID_HEIGHT_REF_OVER_GROUND;
  value.direction = 27825;
  value.speed_horizontal = 25500;
  value.speed_vertical = -50;
  value.horizontal_accuracy = MAV_ODID_HOR_ACC_3_METER;
  value.vertical_accuracy = MAV_ODID_VER_ACC_1_METER;
  value.speed_accuracy = MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND;
  value.timestamp = 65535;
  value.status = MAV_ODID_STATUS_AIRBORNE;
  return value;
}

mavlink_message_t health(PassiveRig& rig, bool ready)
{
  mavlink_open_drone_id_arm_status_t value{};
  value.status = ready ? MAV_ODID_ARM_STATUS_GOOD_TO_ARM : MAV_ODID_ARM_STATUS_PRE_ARM_FAIL_GENERIC;
  if (!ready) std::strcpy(value.error, "hidden transmitter text");
  return rig.encode(value, mavlink_msg_open_drone_id_arm_status_encode_status, 1, source_component);
}

void exercise(PassiveRig& rig)
{
  require(rig.plugin->name() == "remote_id", "wrong Remote ID module");
  require(rig.plugin->configureModule({{"sample_timeout", "1"}}).status == ModuleConfigStatus::Accepted,
          "independent age configuration rejected");
  rig.heartbeat(100.02);
  rig.receive(identity(rig, "PRIVATE-TEST-ID"), 100.03);
  rig.capture("not_selected", 100.05, 3);
  try {
    rig.plugin->configure("remote_id_component", "236suffix");
    throw std::runtime_error("invalid component accepted");
  } catch (const std::invalid_argument&) {}
  rig.plugin->configure("remote_id_component", std::to_string(source_component));
  rig.receive(identity(rig, "PRIVATE-TEST-ID", 1), 100.12);
  rig.capture("wrong_component", 100.16, 3);
  rig.receive(identity(rig, "PRIVATE-TEST-ID"), 100.20, 100.22);
  rig.receive(health(rig, false), 100.24);
  auto value = position();
  rig.receive(rig.encode(value, mavlink_msg_open_drone_id_location_encode_status, 1, source_component), 100.26);
  rig.capture("healthy_input", 100.29, 3);

  auto invalid = value;
  invalid.latitude = 900000001;
  rig.receive(rig.encode(invalid, mavlink_msg_open_drone_id_location_encode_status, 1, source_component), 100.32);
  invalid = value;
  invalid.height = std::numeric_limits<float>::quiet_NaN();
  rig.receive(rig.encode(invalid, mavlink_msg_open_drone_id_location_encode_status, 1, source_component), 100.34);
  invalid = value;
  invalid.target_system = 2;
  rig.receive(rig.encode(invalid, mavlink_msg_open_drone_id_location_encode_status, 1, source_component), 100.36);
  rig.receive(identity(rig, "OTHER-VEHICLE", source_component, true), 100.38);
  mavlink_open_drone_id_operator_id_t excluded{};
  rig.receive(rig.encode(excluded, mavlink_msg_open_drone_id_operator_id_encode_status, 1, source_component), 100.40);
  rig.receive(identity(rig, "OLD-ID"), 100.19, 100.42);
  rig.capture("rejected_inputs", 100.44, 3);

  rig.receive(identity(rig, "PRIVATE-TEST-ID"), 100.47);
  rig.capture("same_identity", 100.56, 3);
  rig.receive(identity(rig, "CHANGED-PRIVATE-ID"), 100.70);
  rig.capture("changed_identity", 100.75, 3);
  rig.heartbeat(101.12);
  rig.receive(health(rig, true), 101.22);
  rig.capture("independent_ages", 101.42, 3);
  rig.capture("heartbeat_loss", 104, 3);
  rig.receive(identity(rig, "CHANGED-PRIVATE-ID"), 104.10);
  rig.capture("no_heartbeat", 104.12, 3);
  rig.heartbeat(104.17);
  rig.receive(identity(rig, "CHANGED-PRIVATE-ID"), 104.20);
  value.latitude = value.longitude = 0;
  value.height = -1000;
  value.speed_vertical = 6300;
  value.direction = 36100;
  rig.receive(rig.encode(value, mavlink_msg_open_drone_id_location_encode_status, 1, source_component), 104.21);
  rig.capture("rebound_unknowns", 104.25, 3);
  rig.capture("clock_fault", 104.23, 3);
}

} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "pass one Remote ID plugin path");
    PassiveRig rig(argv[1]);
    exercise(rig);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

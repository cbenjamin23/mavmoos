#include "FencePlan.h"
#include "Runtime.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace mavmoos::runtime;
namespace {
constexpr auto fence = MAV_MISSION_TYPE_FENCE;
constexpr auto request_key = "MAVMOOS_MISSION_REQUEST";
constexpr auto result_key = "MAVMOOS_MISSION_INSPECTION_RESULT";
const std::string triangle = "420000000:-710000000|420001000:-710000000|420000000:-709999000";

void check(bool ok, const std::string& why)
{
  if (!ok)
    throw std::runtime_error(why);
}

std::vector<MissionItem> vertices(int32_t shift = 0)
{
  std::vector<MissionItem> result(3);
  for (uint16_t seq = 0; seq < result.size(); ++seq) {
    auto& row = result[seq];
    row.seq = seq;
    row.command = MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
    row.frame = MAV_FRAME_GLOBAL;
    row.autocontinue = 0;
    row.params[0] = 3;
    row.x = 420000000 + shift + (seq == 1 ? 1000 : 0);
    row.y = -710000000 + (seq == 2 ? 1000 : 0);
  }
  return result;
}

struct Rig {
  double clock = 100;
  Runtime host;
  mavlink_status_t codec{};
  std::vector<mavlink_message_t> protocol;
  std::vector<std::string> results;

  explicit Rig(const std::string& module, bool enable = true, bool simulation = true,
               bool shadow = false, unsigned vehicle = MAV_TYPE_QUADROTOR)
    : host([&](const auto& key, const auto& text, double, bool) {
        if (key == result_key)
          results.push_back(text);
      }, [&](const uint8_t* data, std::size_t size) {
        mavlink_message_t rx{}, message{};
        mavlink_status_t status{}, result{};
        bool parsed = false;
        for (std::size_t i = 0; i < size; ++i)
          if (mavlink_frame_char_buffer(&rx, &status, data[i], &message, &result) == MAVLINK_FRAMING_OK)
            parsed = true;
        check(parsed, "outbound frame did not decode");
        if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
          mavlink_command_long_t command{};
          mavlink_msg_command_long_decode(&message, &command);
          check(command.command == MAV_CMD_SET_MESSAGE_INTERVAL, "unexpected control command");
        } else if (message.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
          protocol.push_back(message);
        }
        return true;
      }, simulation, shadow, {}, vehicle)
  {
    check(host.load(module, "missions"), host.lastConfigurationError());
    host.configureModules({{"missions", {{"fence_edit", enable ? "true" : "false"}}}});
    host.setFirmwareRequestsEnabled(false);
    state(false, vehicle);
  }

  void state(bool armed, unsigned vehicle = MAV_TYPE_QUADROTOR)
  {
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack_status(1, 1, &codec, &msg, vehicle, MAV_AUTOPILOT_ARDUPILOTMEGA,
      armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0, 0, MAV_STATE_STANDBY);
    receive(msg);
    mavlink_msg_extended_sys_state_pack_status(1, 1, &codec, &msg, 0,
      armed ? MAV_LANDED_STATE_IN_AIR : MAV_LANDED_STATE_ON_GROUND);
    receive(msg);
  }

  void receive(const mavlink_message_t& msg)
  {
    clock += .01;
    host.message(msg, clock);
  }

  void mail(const std::string& body)
  {
    clock += .01;
    host.mail({request_key, "schema=1,type=fence," + body, "operator", 0, clock, clock, false});
  }

  void tick()
  {
    clock += .01;
    host.tick(clock);
  }

  const mavlink_message_t& sent(unsigned id)
  {
    const auto previous = protocol.size();
    for (unsigned cycle = 0; cycle < 8 && protocol.size() == previous; ++cycle)
      tick();
    check(protocol.size() == previous + 1 && protocol.back().msgid == id &&
          protocol.back().magic == MAVLINK_STX, "missing or wrong MAVLink2 phase " + std::to_string(id) +
            " latest result " + (results.empty() ? "" : results.back()));
    return protocol.back();
  }

  void expect(std::string status, const std::string& fields = {}) const
  {
    check(!results.empty() && results.back().find("status=" + status + ",") != std::string::npos &&
          results.back().find(fields) != std::string::npos,
          "wrong result. expected " + status + " " + fields + "; got " + (results.empty() ? "" : results.back()));
  }

  void count(std::size_t size, uint8_t plan = fence)
  {
    mavlink_mission_count_t value{};
    value.target_system = 245; value.target_component = 191; value.mission_type = plan; value.count = size;
    mavlink_message_t msg{};
    mavlink_msg_mission_count_encode_status(1, 1, &codec, &msg, &value);
    receive(msg);
  }

  void item(const MissionItem& row)
  {
    mavlink_mission_item_int_t value{};
    value.target_system = 245; value.target_component = 191; value.mission_type = fence;
    value.seq = row.seq; value.command = row.command; value.frame = row.frame; value.current = row.current;
    value.autocontinue = row.autocontinue; value.param1 = row.params[0];
    value.x = row.x; value.y = row.y; value.z = row.z;
    mavlink_message_t msg{};
    mavlink_msg_mission_item_int_encode_status(1, 1, &codec, &msg, &value);
    receive(msg);
  }

  void requested(uint16_t seq, uint8_t plan = fence, bool legacy = false, bool mavlink1 = false)
  {
    mavlink_mission_request_int_t value{};
    value.target_system = 245; value.target_component = 191; value.mission_type = plan; value.seq = seq;
    mavlink_message_t msg{};
    if (legacy) {
      mavlink_mission_request_t request{};
      request.target_system = value.target_system; request.target_component = value.target_component;
      request.mission_type = plan; request.seq = seq;
      auto framing = codec;
      if (mavlink1)
        framing.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
      mavlink_msg_mission_request_encode_status(1, 1, &framing, &msg, &request);
    } else {
      mavlink_msg_mission_request_int_encode_status(1, 1, &codec, &msg, &value);
    }
    receive(msg);
  }

  void accepted()
  {
    mavlink_mission_ack_t value{};
    value.target_system = 245; value.target_component = 191; value.mission_type = fence;
    mavlink_message_t msg{};
    mavlink_msg_mission_ack_encode_status(1, 1, &codec, &msg, &value);
    receive(msg);
  }

  void read(const std::vector<MissionItem>& rows)
  {
    mavlink_mission_request_list_t list{};
    mavlink_msg_mission_request_list_decode(&sent(MAVLINK_MSG_ID_MISSION_REQUEST_LIST), &list);
    check(list.mission_type == fence, "fence download used flight plan type");
    count(rows.size(), MAV_MISSION_TYPE_RALLY);
    expect("SENT");
    count(rows.size());
    for (const auto& row : rows) {
      mavlink_mission_request_int_t request{};
      mavlink_msg_mission_request_int_decode(&sent(MAVLINK_MSG_ID_MISSION_REQUEST_INT), &request);
      check(request.mission_type == fence && request.seq == row.seq, "fence read requested wrong item");
      item(row);
    }
    mavlink_mission_ack_t ack{};
    mavlink_msg_mission_ack_decode(&sent(MAVLINK_MSG_ID_MISSION_ACK), &ack);
    check(ack.mission_type == fence && ack.type == MAV_MISSION_ACCEPTED, "fence read final ACK mismatch");
  }

  void beginWrite(std::size_t size)
  {
    mavlink_mission_count_t count{};
    mavlink_msg_mission_count_decode(&sent(MAVLINK_MSG_ID_MISSION_COUNT), &count);
    check(count.mission_type == fence && count.count == size, "wrong fence write count");
  }

  void write(const std::vector<MissionItem>& rows)
  {
    beginWrite(rows.size());
    const auto before = protocol.size();
    for (const auto type : {MAV_MISSION_TYPE_MISSION, MAV_MISSION_TYPE_RALLY}) {
      requested(0, type, true);
      tick();
    }
    requested(0, fence, true, true);
    tick();
    check(protocol.size() == before, "wrong plan type or MAVLink1 advanced a fence write");
    for (const auto& row : rows) {
      requested(row.seq, fence, row.seq != 1);
      mavlink_mission_item_int_t item{};
      mavlink_msg_mission_item_int_decode(&sent(MAVLINK_MSG_ID_MISSION_ITEM_INT), &item);
      check(item.mission_type == fence && item.seq == row.seq && item.command == row.command &&
        item.frame == row.frame && item.current == 0 && item.autocontinue == 0 && item.param1 == rows.size() &&
        item.x == row.x && item.y == row.y && item.z == 0, "wrong canonical fence vertex");
    }
    accepted();
    expect("ACKNOWLEDGED", "readback_confirmed=false");
  }
};
}

int main(int argc, char** argv)
{
  try {
    check(argc == 2, "usage: fence_operations_tests MISSIONS_MODULE");
    Rig disabled(argv[1], false);
    disabled.mail("id=disabled,action=save");
    disabled.expect("REJECTED", "fence_edit_disabled");
    check(disabled.protocol.empty(), "disabled request transmitted");
    for (bool shadow : {false, true}) {
      Rig inhibited(argv[1], true, shadow, shadow);
      inhibited.mail("id=effects,action=save");
      inhibited.tick();
      inhibited.expect("FAILED", "host_admission_inhibited");
      check(inhibited.protocol.empty(), "non-effectful request transmitted");
    }
    Rig other(argv[1], true, true, false, MAV_TYPE_GROUND_ROVER);
    other.mail("id=rover,action=save");
    other.expect("REJECTED", "fence_edit_requires_disarmed_ground_copter");

    Rig r(argv[1]);
    r.mail("id=missing,action=replace,backup=unseen,items=" + triangle);
    r.expect("REJECTED", "matching_fence_backup_required");
    r.mail("id=invalid,action=replace,backup=unseen,items=1:1|2:2|1:1");
    r.expect("REJECTED", "invalid_inspection_request");
    r.mail("id=crossing,action=replace,backup=unseen,items=1:1|2:2|2:1|1:2");
    r.expect("REJECTED", "invalid_inspection_request");
    r.mail("id=too-wide,action=replace,backup=unseen,items=0:0|180001:0|0:1");
    r.expect("REJECTED", "invalid_inspection_request");
    auto outside = vertices();
    outside[0].autocontinue = 1;
    r.mail("id=outside,action=save");
    r.read(outside);
    r.expect("FAILED", "stored_fence_outside_edit_profile");
    r.mail("id=save,action=save");
    r.read(vertices(300));
    r.expect("COMPLETED", "backup_id=save,restore_required=false");
    r.mail("id=external-change,action=replace,backup=save,items=" + triangle);
    r.read(vertices(600));
    r.expect("FAILED", "fence_changed_since_backup");
    const auto read_only_count = r.protocol.size();
    r.tick();
    check(r.protocol.size() == read_only_count, "compare failure sent a mutation");

    r.mail("id=interrupted,action=replace,backup=save,items=" + triangle);
    r.read(vertices(300));
    r.expect("BASELINE_MATCHED");
    r.beginWrite(3);
    r.requested(0);
    r.mail("id=interrupted,action=cancel");
    r.expect("CANCELLED", "mutation_may_have_applied=true");
    r.expect("CANCELLED", "restore_required=true");
    const auto after_cancel = r.protocol.size();
    r.requested(1);
    r.tick();
    check(r.protocol.size() == after_cancel, "cancelled write leaked a fence vertex");
    r.mail("id=interrupted,action=replace,backup=save,items=" + triangle);
    r.expect("DUPLICATE");
    r.mail("id=overwrite,action=save");
    r.expect("REJECTED", "verified_restore_required");
    r.mail("id=try-again,action=replace,backup=save,items=" + triangle);
    r.expect("REJECTED", "verified_restore_required");
    r.mail("id=restore,action=restore,backup=save");
    r.write(vertices(300));
    r.read(vertices(301));
    r.expect("FAILED", "readback_confirmed=false");
    r.expect("FAILED", "restore_required=true");
    r.mail("id=restore-verified,action=restore,backup=save");
    r.write(vertices(300));
    r.read(vertices(300));
    r.expect("COMPLETED", "readback_confirmed=true");
    r.expect("COMPLETED", "restore_required=false");

    r.mail("id=arm-before-write,action=replace,backup=save,items=" + triangle);
    r.read(vertices(300));
    r.state(true);
    r.expect("FAILED", "mutation_state_no_longer_permitted");
    const auto after_arm = r.protocol.size();
    r.tick();
    check(r.protocol.size() == after_arm, "armed vehicle sent a mutation");
    r.state(false);
    r.mail("id=replace,action=replace,backup=save,items=" + triangle);
    r.read(vertices(300));
    r.write(vertices());
    r.read(vertices());
    r.expect("COMPLETED", "readback_confirmed=true");
    r.expect("COMPLETED", "restore_required=true");
    check(!r.host.storedMissionProof().valid && !r.host.navigationGranted(), "fence edit gave motion authority");

    Rig empty(argv[1]);
    empty.mail("id=empty,action=save");
    empty.read({});
    empty.expect("COMPLETED", "fence_backup_saved");
    empty.mail("id=empty-restore,action=restore,backup=empty");
    mavlink_mission_clear_all_t clear{};
    mavlink_msg_mission_clear_all_decode(&empty.sent(MAVLINK_MSG_ID_MISSION_CLEAR_ALL), &clear);
    check(clear.mission_type == fence, "empty restore attempted broader clear");
    empty.accepted();
    empty.read({});
    empty.expect("COMPLETED", "readback_confirmed=true");
    empty.host.disconnect();
    empty.state(false);
    empty.mail("id=old-restore,action=restore,backup=empty");
    empty.expect("REJECTED", "matching_fence_backup_required");

    Rig host_only(argv[1]);
    MissionRequest effect;
    effect.id = "phase";
    effect.operation_id = "operation";
    effect.expires = host_only.clock + 1;
    effect.operation_expires = host_only.clock + 5;
    effect.action = MissionAction::Clear;
    effect.mission_type = MAV_MISSION_TYPE_RALLY;
    check(!host_only.host.mission(effect), "runtime permitted rally mutation");
    effect.mission_type = fence;
    effect.action = MissionAction::Item;
    effect.item = vertices().front();
    effect.count = 3;
    check(!host_only.host.mission(effect), "unbound typed vertex escaped the runtime");
    effect.action = MissionAction::Count;
    check(host_only.host.mission(effect), "owned disarmed typed count rejected");
    host_only.state(true);
    host_only.tick();
    check(host_only.protocol.empty(), "final handoff sent a typed count after arming");
    std::cout << "PASS fence scope, typed transfer, readback, cancellation, explicit restore and runtime gates\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}

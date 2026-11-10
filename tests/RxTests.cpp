#include "Fixtures.h"
#include "Telemetry.h"
#include "UdpReceiver.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <thread>

using namespace fixture;
using namespace mavmoos;
namespace {
void ingest(Telemetry& telemetry, const Bytes& bytes, double time, uint64_t id = 1)
{
  telemetry.ingest(bytes.data(), bytes.size(), time, id);
}
void expect(const std::string& record, const std::string& key, const std::string& value)
{
  check(fields(record)[key] == value, key + " expected " + value + " in " + record);
}
void numeric(const std::string& record, const std::string& key, double value)
{
  check(std::abs(std::stod(fields(record).at(key)) - value) < 1e-6, key + " numeric conversion");
}
void configuration()
{
  unsigned value = 55;
  check(!parseUnsigned("-1", 1, 255, value) && value == 55, "reject negative");
  for(auto bad : {"", "256", "1e2", "1x", "1.0", "99999999999999999999"})
    check(!parseUnsigned(bad, 1, 255, value), "reject integer token");
  check(parseUnsigned("255", 1, 255, value) && value == 255, "integer upper bound");
  double seconds = 1;
  for(auto bad : {"nan", "inf", "-1", "0", "2x", " 2"})
    check(!parseSeconds(bad, .01, 60, seconds) && seconds == 1, "reject seconds token");
  check(validateConfig(config()).empty(), "valid explicit config");
  auto bad = config(); bad.target_system = 0;
  check(!validateConfig(bad).empty(), "missing target inhibited");
  for(auto address : {"localhost", "0.0.0.0", "224.0.0.1", "255.255.255.255"}) {
    bad = config(); bad.peer_address = address;
    check(!validateConfig(bad).empty(), "reject invalid peer");
  }
  std::cout << "PASS configuration tokens, required target, numeric fixed peer\n";
}
void payloadConfiguration()
{
  auto setup = config();
  setup.camera_component = 2;
  setup.gimbal_component = 255;
  setup.landing_target_component = 158;
  check(validateConfig(setup).empty(), "distinct payload IDs and legacy landing rejected");

  auto bad = setup;
  bad.camera_component = 1;
  check(!validateConfig(bad).empty(), "camera aliased the FC");
  bad = setup;
  bad.gimbal_component = 1;
  check(!validateConfig(bad).empty(), "gimbal aliased the FC");
  bad = setup;
  bad.camera_component = setup.gimbal_component;
  check(!validateConfig(bad).empty(), "camera and gimbal shared a component");
  bad = setup;
  bad.camera_component = setup.landing_target_component;
  check(!validateConfig(bad).empty(), "camera aliased the landing target");
  bad = setup;
  bad.gimbal_component = setup.landing_target_component;
  check(!validateConfig(bad).empty(), "gimbal aliased the landing target");

  setup.landing_target_component = 0;
  setup.expected_autopilot = MAV_AUTOPILOT_PX4;
  check(validateConfig(setup).empty(), "matching PX4 payload route rejected");
  bad = setup;
  bad.expected_autopilot = MAV_AUTOPILOT_GENERIC;
  check(!validateConfig(bad).empty(), "unqualified autopilot enabled payload ingress");
  bad = setup;
  bad.bind_address = "0.0.0.0";
  check(!validateConfig(bad).empty(), "wildcard bind enabled payload ingress");
  bad = setup;
  bad.peer_address = "192.168.1.42";
  check(!validateConfig(bad).empty(), "non-loopback peer enabled payload ingress");
  bad = setup;
  bad.target_system = 2;
  check(!validateConfig(bad).empty(), "non-1 system enabled payload ingress");
  bad = setup;
  bad.target_component = 2;
  check(!validateConfig(bad).empty(), "non-1 FC component enabled payload ingress");
  std::cout << "PASS payload loopback identity, supported autopilots and distinct component IDs\n";
}
void valuesAndAges()
{
  Telemetry telemetry(config(), "fixture");
  Encoder encode;
  expect(telemetry.record(Family::Position, 100), "present", "false");
  check(fields(telemetry.record(Family::Position, 100)).count("lat_deg") == 0, "absent is not zero");
  ingest(telemetry, encode.position(), 99);
  check(telemetry.stats().unbound_reports == 1, "require heartbeat identity");
  ingest(telemetry, encode.heartbeat(), 100);
  ingest(telemetry, encode.position(), 100.1, 7);
  ingest(telemetry, encode.landed(), 100.2);
  ingest(telemetry, encode.home(), 100.3);
  ingest(telemetry, encode.estimator(), 100.4);
  auto position = telemetry.record(Family::Position, 100.5);
  for(auto key : {"lat_deg", "lon_deg"}) numeric(position, key, 0);
  numeric(position, "alt_msl_m", 123.45);
  numeric(position, "alt_relative_home_m", 23.45);
  numeric(position, "vn_mps", 1.23);
  numeric(position, "ve_mps", -4.56);
  numeric(position, "vd_mps", -.75);
  expect(position, "heading_deg", "unknown");
  expect(position, "source_time", "123456");
  expect(position, "datagram_id", "7");
  expect(telemetry.record(Family::Heartbeat, 100.5), "armed", "true");
  expect(telemetry.record(Family::Heartbeat, 100.5), "custom_mode", "4");
  expect(telemetry.record(Family::Landed, 100.5), "landed_name", "landing");
  numeric(telemetry.record(Family::Home, 100.5), "lat_deg", 12.3456789);
  numeric(telemetry.record(Family::Home, 100.5), "lon_deg", -123.456789);
  expect(telemetry.record(Family::Home, 100.5), "source_time", "unknown");
  expect(telemetry.record(Family::Estimator, 100.5), "airspeed_variance", "not_observed");
  const auto receipt = telemetry.observation(Family::Position).receipt_id;
  for(double now : {101.0, 102.0, 104.0}) {
    ingest(telemetry, encode.heartbeat(), now);
    expect(telemetry.record(Family::Position, now), "freshness", "STALE");
    check(telemetry.observation(Family::Position).receipt_id == receipt, "publication does not renew receipt");
  }
  expect(telemetry.record(Family::Estimator, 104), "freshness", "STALE");
  expect(telemetry.record(Family::Landed, 104), "freshness", "STALE");
  expect(telemetry.record(Family::Home, 104), "freshness", "REFERENCE_KNOWN");
  numeric(telemetry.record(Family::Position, 104), "age_s", 3.9);
  numeric(telemetry.record(Family::Position, 104), "rx_mono_s", 100.1);
  expect(telemetry.record(Family::Position, 104), "source_age", "unknown");
  check(telemetry.heartbeatFresh(104), "fresh heartbeat independent of stale pose");
  ingest(telemetry, encode.position(), 104.1);
  expect(telemetry.record(Family::Position, 104.2), "freshness", "FRESH");
  expect(telemetry.record(Family::Position, 103), "freshness", "CLOCK_ERROR");
  std::cout << "PASS wire quantities, zero coordinates, exact state, original receipt and family expiry\n";
}
void queueTimes()
{
  Telemetry t(config(),"queue-times");Encoder e;
  const auto hb=e.heartbeat();
  t.ingest(hb.data(),hb.size(),100,1,100.2,"QUEUE_EMPTY_FLOOR");
  const auto pose=e.position();t.ingest(pose.data(),pose.size(),100,2,100.2,"QUEUE_EMPTY_FLOOR");
  expect(t.record(Family::Position,100.3),"schema","2");
  expect(t.record(Family::Position,100.3),"age_basis","QUEUE_EMPTY_FLOOR");
  expect(t.record(Family::Position,100.3),"age_is_upper_bound","true");
  numeric(t.record(Family::Position,100.3),"age_s",.3);
  numeric(t.record(Family::Position,100.3),"read_mono_s",100.2);
  numeric(t.record(Family::Position,100.3),"queue_age_upper_s",.2);
  expect(t.record(Family::Position,100.6),"freshness","STALE");
  const auto receipt=t.observation(Family::Position).receipt_id;
  for(auto bad: {std::nan(""),100.3})t.ingest(pose.data(),pose.size(),bad,3,100.2,"QUEUE_EMPTY_FLOOR");
  t.ingest(pose.data(),pose.size(),100,3,100.2,"READ_ACQUISITION");
  check(t.stats().invalid_timestamps==3 && t.observation(Family::Position).receipt_id==receipt,"invalid metadata changed observation");
  std::cout<<"PASS effective/read metadata, conservative freshness, schema and invalid timestamps\n";
}
void filtering()
{
  Telemetry telemetry(config(), "fixture");
  Encoder encode;
  auto heartbeat = encode.heartbeat();
  ingest(telemetry, heartbeat, 10);
  ingest(telemetry, heartbeat, 11);
  check(telemetry.stats().duplicates == 1, "exact duplicate");
  numeric(telemetry.record(Family::Heartbeat, 11), "age_s", 1);
  encode.system = 2;
  ingest(telemetry, encode.position(), 11);
  encode.system = 1; encode.component = 42;
  ingest(telemetry, encode.position(), 11);
  check(telemetry.stats().wrong_target == 2, "wrong IDs rejected");
  encode.component = 1;
  auto corrupt = encode.position();
  corrupt[15] ^= 0x40;
  ingest(telemetry, corrupt, 11);
  check(telemetry.stats().bad_frames == 1, "CRC rejected");
  auto split = encode.position();
  ingest(telemetry, Bytes(split.begin(), split.begin() + 6), 11);
  ingest(telemetry, Bytes(split.begin() + 6, split.end()), 11);
  check(!telemetry.observation(Family::Position).present, "no cross-datagram assembly");
  auto combined = encode.position();
  auto landed = encode.landed();
  combined.insert(combined.end(), landed.begin(), landed.end());
  ingest(telemetry, combined, 12, 9);
  check(telemetry.observation(Family::Position).datagram_id == 9 &&
        telemetry.observation(Family::Landed).received_at == 12, "multi-frame receipt provenance");
  ingest(telemetry, encode.heartbeat(MAV_TYPE_GROUND_ROVER), 13);
  check(telemetry.identityState() == "MISMATCH", "unexpected heartbeat identity");
  check(!telemetry.observation(Family::Position).present, "identity mismatch clears old state");
  ingest(telemetry, encode.position(), 13);
  check(!telemetry.observation(Family::Position).present, "mismatched binding rejects pose");
  std::cout << "PASS wrong IDs, CRC, datagram boundaries, duplicate suppression and identity invalidation\n";
}
void payloadRouting()
{
  auto setup = config();
  setup.camera_component = 100;
  setup.gimbal_component = 154;
  setup.landing_target_component = 158;
  Telemetry telemetry(setup, "payload");
  struct Receipt { unsigned id; unsigned component; double effective; double read; };
  std::vector<Receipt> received;
  telemetry.setTimedMessageHandler([&](const auto& message, double effective, double read) {
    received.push_back({message.msgid, message.compid, effective, read});
  });
  Encoder fc, camera, gimbal, landing;
  camera.component = setup.camera_component;
  gimbal.component = setup.gimbal_component;
  landing.component = setup.landing_target_component;
  const std::vector<std::pair<unsigned, Bytes>> approved{
    {MAVLINK_MSG_ID_CAMERA_INFORMATION, camera.encode(mavlink_camera_information_t{}, mavlink_msg_camera_information_encode_status)},
    {MAVLINK_MSG_ID_CAMERA_SETTINGS, camera.encode(mavlink_camera_settings_t{}, mavlink_msg_camera_settings_encode_status)},
    {MAVLINK_MSG_ID_STORAGE_INFORMATION, camera.encode(mavlink_storage_information_t{}, mavlink_msg_storage_information_encode_status)},
    {MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, camera.encode(mavlink_camera_capture_status_t{}, mavlink_msg_camera_capture_status_encode_status)},
    {MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION, gimbal.encode(mavlink_gimbal_device_information_t{}, mavlink_msg_gimbal_device_information_encode_status)},
    {MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, gimbal.encode(mavlink_gimbal_device_attitude_status_t{},
                                                                  mavlink_msg_gimbal_device_attitude_status_encode_status)},
  };
  for(const auto& item : approved) ingest(telemetry, item.second, 10);
  check(received.empty() && telemetry.identityState() == "UNSEEN" &&
        telemetry.stats().unbound_reports == approved.size(), "payload data established FC identity");
  ingest(telemetry, fc.heartbeat(), 10.1);
  ingest(telemetry, fc.position(), 10.12);
  const auto original_hb = telemetry.record(Family::Heartbeat, 10.5);
  const auto original_pos = telemetry.record(Family::Position, 10.5);
  received.clear();
  for(const auto& item : approved)
    telemetry.ingest(item.second.data(), item.second.size(), 10.2, 42, 10.25, "QUEUE_EMPTY_FLOOR");
  check(received.size() == approved.size(), "selected payload observation lost after binding");
  for(std::size_t i = 0; i < approved.size(); ++i)
    check(received[i].id == approved[i].first && received[i].component == (i < 4 ? camera.component : gimbal.component) &&
          received[i].effective == 10.2 && received[i].read == 10.25, "payload route changed source or original times");
  check(telemetry.stats().observations == 2, "payload entered the FC observation cache");

  Encoder foreign_camera;
  foreign_camera.system = 2;
  foreign_camera.component = camera.component;
  const std::vector<Bytes> rejected{
    foreign_camera.encode(mavlink_camera_information_t{}, mavlink_msg_camera_information_encode_status),
    fc.encode(mavlink_camera_settings_t{}, mavlink_msg_camera_settings_encode_status),
    gimbal.encode(mavlink_camera_capture_status_t{}, mavlink_msg_camera_capture_status_encode_status),
    camera.encode(mavlink_gimbal_device_information_t{}, mavlink_msg_gimbal_device_information_encode_status),
    camera.encode(mavlink_camera_image_captured_t{}, mavlink_msg_camera_image_captured_encode_status),
    gimbal.encode(mavlink_gimbal_manager_information_t{}, mavlink_msg_gimbal_manager_information_encode_status),
    camera.heartbeat(MAV_TYPE_CAMERA),
    gimbal.heartbeat(MAV_TYPE_GIMBAL),
    camera.encode(mavlink_command_ack_t{}, mavlink_msg_command_ack_encode_status),
    camera.encode(mavlink_command_long_t{}, mavlink_msg_command_long_encode_status),
    gimbal.encode(mavlink_set_position_target_local_ned_t{}, mavlink_msg_set_position_target_local_ned_encode_status),
    gimbal.position(100),
    camera.encode(mavlink_landing_target_t{}, mavlink_msg_landing_target_encode_status),
  };
  for(const auto& frame : rejected) ingest(telemetry, frame, 10.3);
  check(received.size() == approved.size() && telemetry.stats().wrong_target == rejected.size(),
        "foreign or out-of-scope payload traffic reached the host");
  check(telemetry.identityState() == "EXPECTED" && telemetry.record(Family::Heartbeat, 10.5) == original_hb &&
        telemetry.record(Family::Position, 10.5) == original_pos, "payload traffic changed FC identity or evidence");
  telemetry.ingest(approved.front().second.data(), approved.front().second.size(), 10.09, 43, 10.4,
                   "QUEUE_EMPTY_FLOOR");
  ingest(telemetry, approved.back().second, 12.11);
  check(received.size() == approved.size() && telemetry.stats().unbound_reports == approved.size() + 2,
        "pre-heartbeat or stale payload reached the host");

  ingest(telemetry, fc.heartbeat(MAV_TYPE_GROUND_ROVER), 12.2);
  ingest(telemetry, approved.front().second, 12.21);
  check(telemetry.identityState() == "MISMATCH" && received.size() == approved.size() + 1,
        "payload accepted with mismatched FC heartbeat");
  ingest(telemetry, fc.heartbeat(), 12.3);
  ingest(telemetry, landing.encode(mavlink_landing_target_t{}, mavlink_msg_landing_target_encode_status), 12.31);
  ingest(telemetry, approved.back().second, 12.32);
  check(received.size() == approved.size() + 4 && received[received.size()-2].id == MAVLINK_MSG_ID_LANDING_TARGET &&
        received.back().id == MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, "rebound payload or legacy landing route failed");
  // The additional operation replies require a per-component opt-in after
  // startup authorization. Enabling camera cannot enable gimbal or generic ACKs.
  const auto before_effects=received.size();
  const auto image=camera.encode(mavlink_camera_image_captured_t{},mavlink_msg_camera_image_captured_encode_status);
  const auto owner=gimbal.encode(mavlink_gimbal_manager_status_t{},mavlink_msg_gimbal_manager_status_encode_status);
  mavlink_command_ack_t ack{};
  ack.command=MAV_CMD_IMAGE_START_CAPTURE;ack.target_system=245;ack.target_component=191;
  const auto capture_ack=camera.encode(ack,mavlink_msg_command_ack_encode_status);
  ack.command=MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
  const auto claim_ack=gimbal.encode(ack,mavlink_msg_command_ack_encode_status);
  ack.command=MAV_CMD_COMPONENT_ARM_DISARM;
  const auto unrelated=camera.encode(ack,mavlink_msg_command_ack_encode_status);
  telemetry.enablePayloadResponses(true,false);
  for(const auto& frame : {image,owner,capture_ack,claim_ack,unrelated})ingest(telemetry,frame,12.4);
  check(received.size()==before_effects+2 && received[before_effects].id==MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED &&
        received.back().id==MAVLINK_MSG_ID_COMMAND_ACK && received.back().component==camera.component,
        "camera opt-in enabled unrelated or gimbal replies");
  telemetry.enablePayloadResponses(true,true);
  for(const auto& frame : {owner,claim_ack})ingest(telemetry,frame,12.5);
  check(received.size()==before_effects+4 && received[before_effects+2].id==MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS &&
        received.back().id==MAVLINK_MSG_ID_COMMAND_ACK && received.back().component==gimbal.component,
        "gimbal opt-in did not enable exactly the operation replies");
  std::cout << "PASS exact payload messages, source/heartbeat binding, FC cache isolation and legacy landing route\n";
}

void payloadDuplicates()
{
  auto setup = config();
  setup.camera_component = 100;
  setup.gimbal_component = 154;
  Telemetry telemetry(setup, "payload-dedup");
  Encoder fc, camera, gimbal;
  camera.component = setup.camera_component;
  gimbal.component = setup.gimbal_component;
  const auto heartbeat = fc.heartbeat();
  const auto first_gimbal = gimbal.encode(mavlink_gimbal_device_attitude_status_t{}, mavlink_msg_gimbal_device_attitude_status_encode_status);
  ingest(telemetry, heartbeat, 100);
  ingest(telemetry, first_gimbal, 100.1);
  for(unsigned i = 0; i < 140; ++i) {
    mavlink_camera_settings_t value{};
    value.time_boot_ms = i + 1;
    ingest(telemetry, camera.encode(value, mavlink_msg_camera_settings_encode_status), 100.2 + i * .001);
  }
  ingest(telemetry, heartbeat, 100.4);
  ingest(telemetry, first_gimbal, 100.41);
  check(telemetry.stats().duplicates == 2 && telemetry.observation(Family::Heartbeat).received_at == 100,
        "camera flood evicted a primary or gimbal duplicate");
  const auto last_camera = camera.encode(mavlink_camera_settings_t{}, mavlink_msg_camera_settings_encode_status);
  ingest(telemetry, last_camera, 100.42);
  for(unsigned i = 0; i < 140; ++i) {
    mavlink_gimbal_device_attitude_status_t value{};
    value.time_boot_ms = i + 1;
    ingest(telemetry, gimbal.encode(value, mavlink_msg_gimbal_device_attitude_status_encode_status), 100.5 + i * .001);
  }
  ingest(telemetry, last_camera, 100.7);
  check(telemetry.stats().duplicates == 3, "gimbal flood evicted a camera duplicate");
  std::cout << "PASS primary, camera and gimbal have independent bounded duplicate caches\n";
}
void invalidFieldsAndVersions()
{
  Telemetry telemetry(config(), "fixture");
  Encoder encode;
  ingest(telemetry, encode.heartbeat(), 20);
  ingest(telemetry, encode.position(900000001), 20);
  check(!telemetry.observation(Family::Position).encoding_valid, "latitude range");
  ingest(telemetry, encode.position(0, 40000), 20);
  check(!telemetry.observation(Family::Position).encoding_valid, "heading range");
  for(auto state : {0, 99}) {
    ingest(telemetry, encode.landed(state), 20);
    check(!telemetry.observation(Family::Landed).encoding_valid, "unknown/undefined landed");
  }
  for(auto state : {1, 2, 3, 4}) {
    ingest(telemetry, encode.landed(state), 20);
    check(telemetry.observation(Family::Landed).encoding_valid, "exact landed enums");
    expect(telemetry.record(Family::Landed, 20), "landed_state", std::to_string(state));
  }
  ingest(telemetry, encode.estimator(std::numeric_limits<float>::quiet_NaN()), 20);
  check(!telemetry.observation(Family::Estimator).encoding_valid, "NaN variance");
  ingest(telemetry, encode.estimator(-1), 20);
  check(!telemetry.observation(Family::Estimator).encoding_valid, "negative variance");
  ingest(telemetry, encode.estimator(0.1f, 2.5f), 20);
  numeric(telemetry.record(Family::Estimator, 20), "airspeed_variance", 2.5);
  const auto original_position = telemetry.observation(Family::Position).receipt_id;
  ingest(telemetry, encode.shortV1Position(), 20);
  check(telemetry.observation(Family::Position).receipt_id == original_position,
        "short CRC-valid MAVLink 1 payload must not invent zero measurements");
  Encoder signed_encoder;
  signed_encoder.useSigning();
  ingest(telemetry, signed_encoder.heartbeat(), 21);
  check(telemetry.stats().unsupported_flags == 1, "signed frames explicitly rejected");
  Encoder v1;
  v1.status.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  ingest(telemetry, v1.heartbeat(), 22);
  check(telemetry.observation(Family::Heartbeat).received_at == 22, "MAVLink 1 supported");
  std::cout << "PASS invalid/unknown fields, optional extensions, signed rejection and MAVLink 1/2\n";
}
void malformedSweep()
{
  Telemetry telemetry(config(), "fixture");
  Encoder encode;
  ingest(telemetry, encode.heartbeat(), 300);
  std::mt19937 random(20261029);
  for(unsigned n = 0; n < 1024; ++n) {
    Bytes noise(random() % 513);
    for(auto& byte : noise) byte = static_cast<uint8_t>(random());
    ingest(telemetry, noise, 300 + n / 1000.0);
  }
  check(telemetry.stats().observations == 1, "deterministic malformed corpus cannot invent observations");
  std::cout << "PASS deterministic 1024-datagram malformed corpus\n";
}
void sockets()
{
  Socket peer, wrong;
  auto setup = config();
  setup.peer_port = peer.port(); setup.bind_port = unusedUdpPort();
  setup.max_datagrams_per_iter = 2;
  setup.camera_component = 100;
  setup.gimbal_component = 154;
  UdpReceiver receiver;
  check(receiver.open(setup), "open configured receiver");
  UdpReceiver collision;
  check(!collision.open(setup) && !collision.isOpen(), "bind failure visible");
  Telemetry telemetry(setup, "fixture");
  Encoder encode;
  wrong.send(encode.heartbeat(), setup.bind_port);
  peer.send(Bytes(5000, 0), setup.bind_port);
  peer.send(encode.heartbeat(), setup.bind_port);
  peer.send(encode.position(), setup.bind_port);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  receiver.poll(telemetry);
  check(receiver.stats().datagrams == 2 && receiver.stats().budget_hits == 1, "bounded receive batch");
  check(receiver.stats().wrong_peer == 1 && receiver.stats().truncated == 1, "peer/truncation filter");
  for(unsigned n = 0; receiver.stats().datagrams < 4 && n < 100; ++n) {
    receiver.poll(telemetry);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  check(telemetry.observation(Family::Position).present, "loopback decoded");
  unsigned payloads = 0;
  telemetry.setMessageHandler([&](const auto& message, double) {
    if(message.msgid == MAVLINK_MSG_ID_CAMERA_SETTINGS || message.msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS)
      ++payloads;
  });
  encode.component = setup.camera_component;
  const auto camera = encode.encode(mavlink_camera_settings_t{}, mavlink_msg_camera_settings_encode_status);
  encode.component = setup.gimbal_component;
  const auto gimbal = encode.encode(mavlink_gimbal_device_attitude_status_t{}, mavlink_msg_gimbal_device_attitude_status_encode_status);
  wrong.send(camera, setup.bind_port);
  peer.send(camera, setup.bind_port);
  peer.send(gimbal, setup.bind_port);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  receiver.poll(telemetry);
  check(receiver.stats().datagrams == 6 && receiver.stats().wrong_peer == 2 && payloads == 1,
        "payload peer filter or common receive budget failed");
  receiver.poll(telemetry);
  check(receiver.stats().datagrams == 7 && payloads == 2, "selected gimbal frame lost");
  check(peer.drain() == 0 && wrong.drain() == 0, "zero outbound UDP at both endpoints");
  receiver.close();
  Socket rebound(setup.bind_port);
  std::cout << "PASS real UDP fixed peer, oversized rejection, batch bound, bind failure, zero output and close\n";
}
}
int main()
{
  try {
    configuration(); payloadConfiguration(); valuesAndAges(); queueTimes(); filtering(); payloadRouting();
    payloadDuplicates(); invalidFieldsAndVersions(); malformedSweep(); sockets();
    return 0;
  } catch(const std::exception& error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}

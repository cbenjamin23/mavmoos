#include "EgressTestSupport.h"
#include <algorithm>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <system_error>
#include <unistd.h>

using namespace p2test;
namespace {
struct Captured { Frame frame; uint16_t source = 0; };
class Peer {
public:
  explicit Peer(uint16_t port = 0)
  {
    fd = socket(AF_INET, SOCK_DGRAM, 0); check(fd >= 0, "peer socket");
    sockaddr_in address{}; address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK); address.sin_port = htons(port);
    if(bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
      const int error = errno; ::close(fd); fd = -1;
      throw std::system_error(error, std::generic_category(), "peer bind");
    }
    socklen_t size = sizeof(address);
    check(getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size) == 0, "peer getsockname");
    local = ntohs(address.sin_port);
    check(fcntl(fd, F_SETFL, O_NONBLOCK) == 0, "peer nonblocking");
  }
  ~Peer() { if(fd >= 0) ::close(fd); }
  Peer(const Peer&) = delete;
  Peer& operator=(const Peer&) = delete;
  uint16_t port() const { return local; }
  void send(const Bytes& bytes, uint16_t destination)
  {
    sockaddr_in address{}; address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK); address.sin_port = htons(destination);
    check(sendto(fd, bytes.data(), bytes.size(), 0, reinterpret_cast<sockaddr*>(&address),
                 sizeof(address)) == static_cast<ssize_t>(bytes.size()), "peer send");
  }
  Captured receive(unsigned wait_ms = 250)
  {
    SteadyClock clock;
    const Tick deadline = clock.now() + Tick(wait_ms) * 1000000;
    do {
      Captured packet;
      sockaddr_in from{}; socklen_t length = sizeof(from);
      const auto size = recvfrom(fd, packet.frame.bytes.data(), packet.frame.bytes.size(), 0,
                                reinterpret_cast<sockaddr*>(&from), &length);
      if(size >= 0) {
        packet.frame.size = static_cast<std::size_t>(size);
        packet.source = ntohs(from.sin_port);
        check(from.sin_addr.s_addr == htonl(INADDR_LOOPBACK), "captured source address");
        return packet;
      }
      check(errno == EAGAIN || errno == EWOULDBLOCK, "peer receive failure");
      if(clock.now() >= deadline) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while(true);
    return {};
  }
private:
  int fd = -1;
  uint16_t local = 0;
};
template<class T> Frame packet(const T& value, mavlink_status_t& status,
  uint16_t (*encoder)(uint32_t, uint8_t, mavlink_status_t*, mavlink_message_t*, const T*),
  uint8_t system = 200, uint8_t component = 191)
{
  mavlink_message_t message{};
  encoder(system, component, &status, &message, &value);
  Frame frame; frame.size = mavlink_msg_to_send_buffer(frame.bytes.data(), &message);
  return frame;
}
void sameBytes(const Frame& actual, const Frame& expected)
{
  check(actual.size == expected.size &&
        std::equal(actual.bytes.begin(), actual.bytes.begin() + actual.size, expected.bytes.begin()),
        "independent generated encoder byte oracle");
}
Frame heartbeat()
{
  mavlink_heartbeat_t report{};
  report.type = MAV_TYPE_QUADROTOR; report.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
  report.base_mode = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  report.custom_mode = 4;
  mavlink_status_t status{};
  return packet(report, status, mavlink_msg_heartbeat_encode_status, 1, 1);
}
template<class Predicate> unsigned receiveUntil(ManagedEgress& engine, Predicate complete)
{
  SteadyClock clock;
  const Tick deadline = clock.now() + second / 5;
  unsigned data = 0;
  while(!complete(engine.stats())) {
    const auto before = engine.stats();
    data += engine.receiveBatch().size;
    const auto after = engine.stats();
    const auto consumed = (after.rx_datagrams - before.rx_datagrams) +
      (after.rx_wrong_peer - before.rx_wrong_peer) + (after.rx_truncated - before.rx_truncated) +
      (after.rx_errors - before.rx_errors);
    check(consumed <= 8, "each nonblocking RX batch stays bounded");
    if(complete(after)) return data;
    check(clock.now() < deadline, "bounded fixture wait for asynchronous loopback arrival");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return data;
}
void loopback(Trace& trace)
{
  SteadyClock clock;
  Peer peer, rogue;
  auto io = std::make_unique<UdpLoopbackIo>(peer.port());
  ManagedEgress engine(binding(), envelope(), clock, std::move(io));
  const uint16_t port = engine.localPort();
  check(engine.updateVehicle(state(clock.now())), "loopback observations");
  check(engine.enable({scope(), 1, 2, clock.now() + 5 * second, 0}) == Reason::Ok, "loopback explicit fixture grant");
  mavlink_status_t oracle_status{};
  trace.begin("D1_real_udp_exact_bytes_addresses_and_latest_nav");
  trace.newInstance();
  auto a = engine.submitNavigation(1, scope(), 1, input(clock.now(), 0, 1));
  trace.admission(a); check(a.accepted, "old waiting nav");
  a = engine.submitNavigation(2, scope(), 1, input(clock.now(), 0, 2));
  trace.admission(a); check(a.accepted, "new waiting nav");
  engine.pump();
  auto capture = peer.receive();
  check(capture.source == port && capture.frame.size, "configured destination and actual source");
  trace.packet(capture.frame, capture.source, peer.port());
  auto target = local(capture.frame);
  mavlink_set_position_target_local_ned_t expected{};
  expected.time_boot_ms = target.time_boot_ms; // only this runtime-clock field is nondeterministic
  expected.target_system = expected.target_component = 1;
  expected.coordinate_frame = MAV_FRAME_LOCAL_NED; expected.type_mask = 2503;
  expected.vx = 2; expected.vz = -1;
  sameBytes(capture.frame, packet(expected, oracle_status, mavlink_msg_set_position_target_local_ned_encode_status));
  check(rogue.receive(1).frame.size == 0, "no rogue output");
  auto results = drain(engine, trace);
  check(results.size() == 2 && results[0].outcome == Outcome::Replaced && results[1].locally_accepted,
        "replaced and exact local acceptance");
  check(results[1].accepted_bytes == capture.frame.size && !results[1].late, "full acceptance size/time");
  trace.pass();

  trace.begin("D2_zero_speed_mask_and_no_peer_learning");
  const Frame report = heartbeat();
  const Bytes bytes(report.bytes.begin(), report.bytes.begin() + report.size);
  rogue.send(bytes, port); peer.send(bytes, port); peer.send(Bytes(5000, 0), port);
  const auto valid = receiveUntil(engine, [](const Stats& s) {
    return s.rx_datagrams >= 1 && s.rx_wrong_peer >= 1 && s.rx_truncated >= 1;
  });
  const auto rx = engine.stats();
  check(valid == 1 && rx.rx_wrong_peer == 1 && rx.rx_truncated == 1, "fixed-peer RX and size filters");
  trace.out << "{\"event\":\"rx\",\"case\":\"D2\",\"valid\":" << rx.rx_datagrams
            << ",\"wrong_peer\":" << rx.rx_wrong_peer << ",\"truncated\":" << rx.rx_truncated << "}\n";
  a = engine.submitNavigation(3, scope(), 1, input(clock.now(), 90, 0, 10));
  trace.admission(a); check(a.accepted, "zero horizontal velocity");
  engine.pump();
  capture = peer.receive(); trace.packet(capture.frame, capture.source, peer.port());
  target = local(capture.frame);
  expected = {}; expected.time_boot_ms = target.time_boot_ms;
  expected.target_system = expected.target_component = 1;
  expected.coordinate_frame = MAV_FRAME_LOCAL_NED; expected.type_mask = 3527;
  sameBytes(capture.frame, packet(expected, oracle_status, mavlink_msg_set_position_target_local_ned_encode_status));
  check(rogue.receive(1).frame.size == 0, "incoming sender did not redirect TX");
  drain(engine, trace); trace.pass();

  trace.begin("D3_priority_fence_rx_flood_and_hold_before_probe");
  check(engine.submitProbe(4, scope(), 2, 42, clock.now() + second).accepted, "waiting discrete probe");
  for(unsigned i = 0; i < 20; ++i) rogue.send(bytes, port);
  const Tick trigger = clock.now();
  engine.requestFence(scope()); engine.pump();
  trace.fence(engine.fence());
  check(engine.fence().is_applied, "receive flood cannot starve serialized fence");
  check(receiveUntil(engine, [](const Stats& s) { return s.rx_wrong_peer >= 21; }) == 0,
        "bounded wrong-peer drain");
  check(engine.stats().rx_wrong_peer == 21, "eight-read batch budget drains finite flood");
  check(engine.grantResponse({scope(), 3, trigger, trigger + response_limit}) == Reason::Ok, "conditional hold fixture permit");
  a = engine.submitResponse(5, scope(), 3); trace.admission(a); check(a.accepted, "hold snapshot");
  engine.pump();
  capture = peer.receive(); trace.packet(capture.frame, capture.source, peer.port());
  auto message = decode(capture.frame);
  check(message.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT, "response before probe");
  mavlink_set_position_target_global_int_t global{};
  mavlink_msg_set_position_target_global_int_decode(&message, &global);
  mavlink_set_position_target_global_int_t hold_expected{};
  hold_expected.time_boot_ms = global.time_boot_ms;
  hold_expected.target_system = hold_expected.target_component = 1;
  hold_expected.coordinate_frame = MAV_FRAME_GLOBAL_INT; hold_expected.type_mask = 3576;
  hold_expected.alt = 123.45f;
  sameBytes(capture.frame, packet(hold_expected, oracle_status, mavlink_msg_set_position_target_global_int_encode_status));
  engine.pump();
  capture = peer.receive(); trace.packet(capture.frame, capture.source, peer.port());
  message = decode(capture.frame);
  check(message.msgid == MAVLINK_MSG_ID_PING, "directed PING fixture follows hold");
  mavlink_ping_t ping{}; mavlink_msg_ping_decode(&message, &ping);
  mavlink_ping_t ping_expected{};
  ping_expected.time_usec = ping.time_usec; ping_expected.seq = 42;
  ping_expected.target_system = ping_expected.target_component = 1;
  sameBytes(capture.frame, packet(ping_expected, oracle_status, mavlink_msg_ping_encode_status));
  drain(engine, trace); trace.pass();

  trace.begin("D4_close_cancels_waiting_preserves_rx_until_close");
  const auto received_before = engine.stats().rx_datagrams;
  peer.send(bytes, port);
  check(receiveUntil(engine, [received_before](const Stats& s) {
    return s.rx_datagrams > received_before;
  }) == 1, "routine fence preserved socket reception");
  check(engine.submitProbe(6, scope(), 2, 99, clock.now() + second).accepted, "work waiting at close");
  const auto closed = engine.close(); trace.fence(closed);
  engine.pump();
  check(peer.receive(20).frame.size == 0 && rogue.receive(1).frame.size == 0, "no output after close");
  check(engine.receiveBatch().size == 0, "closed receive lifetime settled");
  results = drain(engine, trace);
  check(results.size() == 1 && results[0].id == 6 && !results[0].attempted, "one terminal waiting-close outcome");
  check(closed.is_quiescent && closed.residual_possible && closed.accepted_before == 4,
        "local quiescence is separate from accepted datagrams");
  { Peer rebound(port); } // actual port release, no raw fd manipulation or socket bypass
  trace.pass();

  trace.begin("D5_new_epoch_no_inherited_readiness_and_bind_error");
  {
    trace.newInstance();
    auto new_io = std::make_unique<UdpLoopbackIo>(peer.port(), port);
    ManagedEgress fresh(binding(2), envelope(), clock, std::move(new_io));
    auto old = fresh.submitNavigation(1, scope(), 1, input(clock.now()));
    trace.admission(old);
    check(old.reason == Reason::ScopeMismatch, "old connection cannot acquire readiness");
    fresh.pump();
    check(peer.receive(1).frame.size == 0, "new socket does not inherit permission");
    bool failed_bind = false;
    try { UdpLoopbackIo collision(peer.port(), port); }
    catch(const std::system_error& error) { failed_bind = error.code().value() == EADDRINUSE; }
    check(failed_bind, "real OS bind error is explicit");
    trace.fence(fresh.close());
  }
  { Peer rebound(port); }
  trace.out << "{\"event\":\"cleanup\",\"bridge_port\":" << port
            << ",\"peer_port\":" << peer.port() << ",\"rogue_port\":" << rogue.port()
            << ",\"bridge_port_rebound\":true,\"background_processes_started\":0}\n";
  trace.pass();
}
}
int main(int argc, char** argv)
{
  try {
    check(argc == 2, "trace path argument");
    Trace trace(argv[1]); loopback(trace); trace.summary("real_loopback_udp_no_vehicle");
    return 0;
  } catch(const std::exception& error) { std::cerr << "FAIL " << error.what() << '\n'; return 1; }
}

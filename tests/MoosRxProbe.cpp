#include "Fixtures.h"
#include "Telemetry.h"
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace fixture;
using namespace mavmoos;
namespace {
void packet(Socket& peer, const Bytes& bytes, uint16_t target, const char* name)
{
  peer.send(bytes, target);
  std::cout << "FIXTURE_PACKET\t" << name << '\t';
  for(uint8_t byte : bytes)
    std::cout << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
  std::cout << std::dec << '\n';
}
void snapshot(const std::string& stage, const std::map<std::string, std::string>& values)
{
  for(const auto& [name, value] : values)
    if(name != "APPCAST")
      std::cout << "OBS\t" << stage << '\t' << name << '\t' << value << '\n';
}
}
int main(int argc, char** argv)
{
  try {
    check(argc == 5, "args: DB port / bridge UDP port / peer UDP port / scenario");
    const auto db_port = std::stoi(argv[1]);
    const auto bridge_port = static_cast<uint16_t>(std::stoi(argv[2]));
    const auto peer_port = static_cast<uint16_t>(std::stoi(argv[3]));
    const std::string scenario = argv[4];
    const bool invalid = scenario != "normal";
    Socket peer(peer_port), wrong;
    Encoder encoder;
    CMOOSCommClient client;
    client.SetQuiet(true);
    check(client.Run("127.0.0.1", db_port, "mavmoos_rx_fixture", 20), "MOOS fixture start");
    bool registered = false, injected = false, saw_fresh = false;
    std::map<std::string, std::string> values, first_fresh;
    double started = 0, last_heartbeat = 0;
    const double deadline = monotonicSeconds() + 13;
    unsigned outbound = 0;
    while(monotonicSeconds() < deadline) {
      const double now = monotonicSeconds();
      if(client.IsConnected() && !registered) {
        for(auto family : Telemetry::families()) client.Register(Telemetry::variable(family), 0);
        client.Register("MAVMOOS_STATUS", 0);
        client.Register("MAVMOOS_CAPABILITIES", 0);
        client.Register("APPCAST", 0);
        registered = true;
      }
      MOOSMSG_LIST mail;
      client.Fetch(mail);
      for(const auto& msg : mail)
        if(msg.IsString() && msg.GetSource() == "iMAVLink")
          values[msg.GetKey()] = msg.GetString();
      if(!injected && values.count("MAVMOOS_STATUS")) {
        const auto state = fields(values["MAVMOOS_STATUS"])["state"];
        check(state == (invalid ? "CONFIG_ERROR" : "RX_ONLY"), "real ProcessConfig state: " + state);
        client.Notify("APPCAST_REQ", "node=all,app=all,duration=6,key=fixture");
        client.Notify("MAVMOOS_NAV_ACQUIRE", "request_id=fixture,enable=true");
        client.Notify("MOOS_MANUAL_OVERRIDE", "false");
        client.Notify("DESIRED_HEADING", 90.0);
        client.Notify("DESIRED_SPEED", 1.0);
        client.Notify("DESIRED_ALTITUDE", 10.0);
        packet(wrong, encoder.heartbeat(), bridge_port, "wrong_peer");
        encoder.system = 2;
        packet(peer, encoder.heartbeat(), bridge_port, "wrong_target");
        encoder.system = 1;
        auto bad_crc = encoder.position();
        bad_crc[15] ^= 0x40;
        packet(peer, bad_crc, bridge_port, "bad_crc");
        Encoder signed_encoder;
        signed_encoder.useSigning();
        packet(peer, signed_encoder.heartbeat(), bridge_port, "signed_rejected");
        auto all = encoder.heartbeat();
        for(const auto& bytes : {encoder.position(), encoder.landed(), encoder.home(), encoder.estimator()})
          all.insert(all.end(), bytes.begin(), bytes.end());
        packet(peer, all, bridge_port, "known_reports");
        started = now;
        last_heartbeat = now;
        injected = true;
        if(invalid) {
          Socket unbound_bridge_port(bridge_port);
          std::cout << "PASS invalid configuration left UDP port unbound\n";
        }
      }
      if(injected && !invalid && now - last_heartbeat >= 0.2) {
        peer.send(encoder.heartbeat(), bridge_port);
        last_heartbeat = now;
      }
      if(!invalid && injected && !saw_fresh &&
         fields(values["MAVMOOS_GLOBAL_POSITION"])["freshness"] == "FRESH" &&
         fields(values["MAVMOOS_ESTIMATOR"])["present"] == "true" &&
         fields(values["MAVMOOS_HOME_POSITION"])["present"] == "true" &&
         fields(values["MAVMOOS_LANDED_STATE"])["present"] == "true") {
        first_fresh = values;
        snapshot("fresh", values);
        saw_fresh = true;
      }
      outbound += peer.drain() + wrong.drain();
      if(injected && now - started >= (invalid ? 1.1 : 3.8)) {
        const auto status = fields(values.at("MAVMOOS_STATUS"));
        check(status.at("mavlink_tx_attempts") == "0" && status.at("mavlink_tx_accepted") == "0",
              "bridge ledger reports zero attempts/acceptance");
        if(invalid) {
          check(status.at("state") == "CONFIG_ERROR", "configuration stays inhibited");
          check(status.at("rx_datagrams") == "0", "invalid config has no UDP reads");
          if(scenario == "invalid_auto_recovery") {
            const std::string diagnostic = "mission execution requires auto_recovery_directory";
            check(values.count("APPCAST") && values.at("APPCAST").find(diagnostic) != std::string::npos,
                  "missing AUTO recovery directory diagnostic");
            std::cout << "CAPTURE\tstartup_rejection\t" << diagnostic << '\n';
          }
          for(auto family : Telemetry::families())
            check(fields(values.at(Telemetry::variable(family)))["present"] == "false", "absent reports");
        } else {
          check(saw_fresh, "received initial fresh MOOS reports");
          const auto pose = fields(values.at("MAVMOOS_GLOBAL_POSITION"));
          const auto initial = fields(first_fresh.at("MAVMOOS_GLOBAL_POSITION"));
          check(pose.at("freshness") == "STALE" && std::stod(pose.at("age_s")) >= 3.3, "pose expires");
          check(pose.at("rx_mono_s") == initial.at("rx_mono_s") &&
                pose.at("receipt_id") == initial.at("receipt_id"), "original evidence retained");
          check(pose.at("source_age") == "unknown", "unknown source age explicit");
          check(fields(values.at("MAVMOOS_ESTIMATOR"))["freshness"] == "STALE", "estimator expires");
          check(fields(values.at("MAVMOOS_LANDED_STATE"))["freshness"] == "STALE", "landed expires");
          check(fields(values.at("MAVMOOS_HOME_POSITION"))["freshness"] == "REFERENCE_KNOWN", "home reference retained");
          check(status.at("heartbeat_fresh") == "true", "heartbeat stayed fresh");
          for(auto key : {"wrong_peer", "wrong_target", "bad_frames", "unsupported_flags"})
            check(std::stoul(status.at(key)) >= 1, std::string("real filter counter ") + key);
          check(values.count("APPCAST") != 0, "AppCast response received");
        }
        check(outbound == 0, "endpoint capture saw outbound UDP");
        snapshot("final", values);
        if(!invalid) {
          check(values.count("MAVMOOS_CAPABILITIES"),"capability catalog missing");
          std::cout << "CAPTURE\tMAVMOOS_CAPABILITIES\t" << values.at("MAVMOOS_CAPABILITIES") << '\n';
        }
        std::cout << "CAPTURE\tpeer_and_wrong_peer\toutbound_datagrams=" << outbound << '\n';
        std::cout << "PASS " << (invalid ? "inhibited configuration" : "MOOS telemetry, independent expiry, AppCast")
                  << " and zero UDP output during MOOS requests\n";
        client.Close();
        return 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("MOOS fixture deadline");
  } catch(const std::exception& error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}

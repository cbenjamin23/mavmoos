// Baseline diagnostic: a queued position must not become new just because it is read late.
#include "Fixtures.h"
#include "Telemetry.h"
#include "UdpReceiver.h"
#include <chrono>
#include <iostream>
#include <thread>

int main()
{
  using namespace mavmoos;
  using namespace fixture;
  try {
    Socket peer;
    auto settings=config();settings.peer_port=peer.port();settings.bind_port=unusedUdpPort();
    Telemetry telemetry(settings,"backlog-baseline");UdpReceiver receiver;
    check(receiver.open(settings),"receiver open");
    Encoder encoder;
    const double sent=monotonicSeconds();
    peer.send(encoder.heartbeat(),settings.bind_port);
    peer.send(encoder.position(),settings.bind_port);
    std::this_thread::sleep_for(std::chrono::milliseconds(650));
    receiver.poll(telemetry);
    const double observed=monotonicSeconds();
    const auto position=telemetry.record(Family::Position,observed);
    const auto values=fields(position);
    const bool present=values.at("present")=="true";
    std::cout << "{\"layer\":\"owned UDP loopback; no app/control\",\"sent_to_observe_s\":"
              << observed-sent << ",\"reported_position_present\":" << (present?"true":"false")
              << ",\"reported_position_age_s\":" << (present?values.at("age_s"):"null")
              << ",\"reported_freshness\":\"" << (present?values.at("freshness"):"ABSENT")
              << "\",\"rx_datagrams\":" << receiver.stats().datagrams << "}\n";
    receiver.close();
    check(peer.drain()==0,"unexpected outbound datagram");
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}

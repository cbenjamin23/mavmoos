// Standalone diagnostic of the archived 943ff39 read-time admission semantics.
// Receiver sockets are owned loopback fixtures; all outbound effects use fake I/O.
#include "Fixtures.h"
#include "Telemetry.h"
#include "UdpReceiver.h"
#include "Runtime.h"
#include <chrono>
#include <iostream>
#include <thread>

int main()
{
  using namespace fixture;
  using namespace mavmoos;
  try {
    Socket peer;
    auto cfg=config();cfg.peer_port=peer.port();cfg.bind_port=unusedUdpPort();
    UdpReceiver receiver;check(receiver.open(cfg),"receiver open");
    unsigned nav_writes=0;
    runtime::Runtime runtime([](const auto&,const auto&,double,bool){},
      [&nav_writes](const uint8_t* bytes,std::size_t size) {
        mavlink_message_t message{},buffer{};mavlink_status_t state{},result{};
        for(std::size_t i=0;i<size;++i)
          if(mavlink_frame_char_buffer(&buffer,&state,bytes[i],&message,&result)==MAVLINK_FRAMING_OK && message.msgid==84)
            ++nav_writes;
        return true;
      },true,false,monotonicSeconds);
    Telemetry telemetry(cfg,"archived-read-time-baseline");
    telemetry.setMessageHandler([&runtime](const mavlink_message_t& m,double at){runtime.message(m,at);});
    Encoder encode;
    const double sent_at=monotonicSeconds();
    peer.send(encode.heartbeat(),cfg.bind_port);peer.send(encode.position(),cfg.bind_port);
    peer.send(encode.landed(MAV_LANDED_STATE_IN_AIR),cfg.bind_port);peer.send(encode.home(),cfg.bind_port);
    mavlink_ekf_status_report_t ekf{};ekf.flags=51;
    peer.send(encode.encode(ekf,mavlink_msg_ekf_status_report_encode_status),cfg.bind_port);
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    receiver.poll(telemetry);
    const double now=monotonicSeconds();
    const bool acquired=runtime.acquireNavigation("archived-backlog");
    const bool target=runtime.navigation({90,1,5,now+1});
    runtime.tick(monotonicSeconds());
    std::cout << "layer=owned_loopback_receiver_and_fake_outbound_io\n"
              << "baseline_commit=943ff39\n"
              << "elapsed_since_fixture_send_s=" << now-sent_at << '\n'
              << "runtime_position_age_s=" << now-runtime.vehicle().position_at << '\n'
              << "acquired=" << acquired << ",target_admitted=" << target << ",fake_nav_writes=" << nav_writes << '\n'
              << "position_record=" << telemetry.record(Family::Position,now) << '\n';
    check(now-sent_at>.5,"fixture did not outlive position budget");
    check(acquired&&target&&nav_writes==1,"archived baseline did not reproduce fresh-stamped backlog");
    receiver.close();
    std::cout << "REPRODUCED old queued data accepted as fresh; no real vehicle or outbound socket used\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}

// Owned loopback and fake-clock queue-age checks. Never linked into the app.
#include "Fixtures.h"
#include "Telemetry.h"
#include "UdpReceiver.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <thread>

namespace {
using namespace mavmoos;
using namespace fixture;
using Clock=std::chrono::steady_clock;
using Decision=QueueEpoch::Decision;

void policy()
{
  QueueEpoch q;q.reset(100);
  check(q.received(100.01,100.02,.25)==Decision::Accept,"initial bind lower bound");
  check(q.floor()==100,"read renewed original queue epoch");
  check(q.empty(100.1,100.11),"empty proof rejected");
  check(q.floor()==100.1,"used post-syscall empty timestamp");
  check(q.received(100.12,100.13,.25)==Decision::Accept,"post-empty packet denied");
  check(q.floor()<=100.105,"arrival between EAGAIN and return is underbounded");

  q.reset(100);
  check(q.received(100.24,100.25,.25)==Decision::Accept,"inclusive age bound changed");
  check(q.received(100.2501,100.251,.25)==Decision::Stale,"old queue accepted");
  check(q.received(100.252,100.253,1)==Decision::Stale,"draining renewed by a fresh read");
  check(q.empty(100.3,100.31)&&!q.draining(),"empty queue did not end draining");
  check(q.received(100.32,100.33,.25)==Decision::Accept,"new queue did not recover");

  for(const auto sample:{std::numeric_limits<double>::quiet_NaN(),99.0}) {
    q.reset(100);check(q.received(sample,100.01,.25)==Decision::ClockInvalid,"bad pre-read clock accepted");
    check(q.draining(),"bad clock did not latch draining");
    check(!q.empty(99,99.1),"backward empty proof accepted");
    check(q.empty(101,101.01),"valid empty proof could not recover");
  }
  q.reset(100);check(q.received(100.1,100.09,.25)==Decision::ClockInvalid,"clock changed during read");
  q.reset(100);check(q.received(100.1,100.11,.25)==Decision::Accept,"clock fixture");
  check(q.received(100.105,100.12,.25)==Decision::ClockInvalid,"inter-read clock regression accepted");
  q.reset(std::numeric_limits<double>::quiet_NaN());
  check(q.received(101,101.01,.25)==Decision::ClockInvalid,"unknown initial floor accepted");
  std::cout<<"{\"mode\":\"policy\",\"result\":\"PASS\",\"groups\":7,\"clock\":\"fake\"}\n";
}

struct Rig {
  Socket peer;
  RxConfig settings;
  Telemetry telemetry;
  UdpReceiver receiver;
  Encoder encoder;
  explicit Rig(unsigned budget):settings(make(budget)),telemetry(settings,"ingress-loopback") {
    check(receiver.open(settings),"receiver open");
  }
  RxConfig make(unsigned budget) {
    auto c=config();c.peer_port=peer.port();c.bind_port=unusedUdpPort();
    c.max_datagrams_per_iter=budget;c.max_queue_age=.25;return c;
  }
  void send(const Bytes& bytes){peer.send(bytes,settings.bind_port);}
  unsigned poll() {
    const auto before=receiver.stats().datagrams;receiver.poll(telemetry);
    const auto count=receiver.stats().datagrams-before;
    check(count<=settings.max_datagrams_per_iter,"receive work exceeded configured bound");
    return static_cast<unsigned>(count);
  }
  void drain() {
    for(unsigned i=0;i<20;++i) {
      const auto empty=receiver.stats().empty_events;
      poll();if(receiver.stats().empty_events>empty)return;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    throw std::runtime_error("bounded quiet drain did not reach EAGAIN");
  }
  void awaitDatagrams(unsigned expected) {
    const auto deadline=Clock::now()+std::chrono::milliseconds(200);
    while(receiver.stats().datagrams<expected&&Clock::now()<deadline) {
      const auto floor=receiver.stats().queue_epoch_at;
      const auto empty=receiver.stats().empty_events;
      const auto read=poll();
      if(read==settings.max_datagrams_per_iter)
        check(receiver.stats().empty_events==empty&&receiver.stats().queue_epoch_at==floor,
              "budget exhaustion renewed queue epoch without an empty proof");
      if(receiver.stats().datagrams<expected)std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(receiver.stats().datagrams==expected,"owned fixture datagrams not observed within bound");
  }
  ~Rig(){receiver.close();}
};

void burst(unsigned budget)
{
  Rig r(budget);
  r.send(r.encoder.heartbeat());
  for(int i=0;i<39;++i)r.send(r.encoder.position(i));
  r.awaitDatagrams(40);
  r.drain();
  const auto& s=r.receiver.stats();
  check(s.datagrams==40&&s.queue_stale_dropped==0,"quiet burst was dropped");
  check(s.empty_events>0&&s.last_effective_at<=s.last_read_at,"missing conservative effective timestamp");
  check(r.peer.drain()==0,"receive test transmitted an outbound datagram");
  std::cout<<"{\"mode\":\"burst\",\"result\":\"PASS\",\"budget\":"<<budget
           <<",\"sent\":40,\"read\":"<<s.datagrams<<",\"budget_hits\":"<<s.budget_hits
           <<",\"empty_events\":"<<s.empty_events<<",\"basis\":\"QUEUE_EMPTY_FLOOR\"}\n";
}

void stall()
{
  Rig r(32);r.poll();
  r.send(r.encoder.heartbeat());r.send(r.encoder.position());
  std::this_thread::sleep_for(std::chrono::milliseconds(350));
  r.poll();
  check(r.receiver.stats().datagrams==2&&r.receiver.stats().queue_stale_dropped==2,"stalled queue not discarded");
  check(!r.telemetry.observation(Family::Position).present,"stalled position became a new observation");
  check(r.receiver.stats().empty_events>=2&&!r.receiver.stats().draining,"stalled queue did not reach empty proof");
  r.send(r.encoder.heartbeat());r.send(r.encoder.position());
  r.awaitDatagrams(4);
  check(r.telemetry.observation(Family::Position).present,"fresh queue did not recover");
  check(r.receiver.stats().queue_stale_dropped==2,"fresh queue incorrectly counted stale");
  check(r.peer.drain()==0,"stall test transmitted");
  std::cout<<"{\"mode\":\"stall\",\"result\":\"PASS\",\"stall_s\":0.35,\"dropped\":2,"
              "\"fresh_recovery\":true,\"outbound_datagrams\":0}\n";
}

void matrix(unsigned budget,unsigned hz)
{
  Rig r(budget);r.send(r.encoder.heartbeat());r.awaitDatagrams(1);r.drain();
  const auto initial_frames=r.telemetry.stats().frames;
  const auto initial_datagrams=r.receiver.stats().datagrams;
  std::atomic<unsigned> sent{0};std::atomic<bool> done{false},sender_failed{false};
  constexpr unsigned count=2400,rate=1500;
  const auto start=Clock::now();
  std::thread sender([&]{
    try {
      Encoder encoder;
      for(unsigned i=0;i<count;++i) {
        std::this_thread::sleep_until(start+std::chrono::microseconds(i*1000000ULL/rate));
        r.send(encoder.position(static_cast<int32_t>(i+1)));++sent;
      }
    } catch(...) {sender_failed=true;}
    done=true;
  });
  unsigned polls=0,max_work=0;
  while(!done) {
    max_work=std::max(max_work,r.poll());++polls;
    std::this_thread::sleep_until(start+std::chrono::microseconds(polls*1000000ULL/hz));
  }
  sender.join();
  check(!sender_failed&&sent==count,"matrix sender failed");
  r.drain();
  // A second quiet empty proof avoids treating an in-flight loopback send as loss.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));r.drain();
  const auto& s=r.receiver.stats();
  const auto reads=s.datagrams-initial_datagrams,accepted=r.telemetry.stats().frames-initial_frames;
  if(reads>count||accepted+s.queue_stale_dropped!=reads)
    std::cerr<<"matrix accounting: budget="<<budget<<" initial_datagrams="<<initial_datagrams
             <<" initial_frames="<<initial_frames<<" reads="<<reads<<" accepted="<<accepted
             <<" stale="<<s.queue_stale_dropped<<" clock="<<s.queue_clock_dropped
             <<" wrong_peer="<<s.wrong_peer<<" truncated="<<s.truncated
             <<" invalid_timestamps="<<r.telemetry.stats().invalid_timestamps<<'\n';
  check(reads<=count&&accepted+s.queue_stale_dropped==reads,"matrix accounting mismatch");
  check(max_work<=budget,"matrix work bound");
  check(!s.draining,"matrix failed to recover after sender quiescence");
  check(r.peer.drain()==0,"matrix transmitted");
  std::cout<<"{\"mode\":\"matrix\",\"result\":\"PASS\",\"budget\":"<<budget<<",\"poll_hz\":"<<hz
           <<",\"scheduled_sender_hz\":"<<rate<<",\"sent\":"<<sent<<",\"read\":"<<reads
           <<",\"accepted_frames\":"<<accepted<<",\"queue_stale_dropped\":"<<s.queue_stale_dropped
           <<",\"unobserved_after_quiet\":"<<count-reads<<",\"polls\":"<<polls
           <<",\"max_read_per_poll\":"<<max_work<<",\"budget_hits\":"<<s.budget_hits
           <<",\"max_poll_gap_s\":"<<s.max_poll_gap_s<<",\"max_queue_age_upper_s\":"<<s.max_queue_age_upper_s
           <<",\"basis\":\"QUEUE_EMPTY_FLOOR\",\"outbound_datagrams\":0}\n";
}
}

int main(int argc,char** argv)
{
  try {
    const std::string mode=argc>1?argv[1]:"all";
    check(mode=="policy"||mode=="loopback"||mode=="matrix"||mode=="all","mode: policy|loopback|matrix|all");
    if(mode=="policy"||mode=="all")policy();
    if(mode=="loopback"||mode=="all"){burst(32);burst(128);stall();}
    if(mode=="matrix"||mode=="all"){matrix(32,30);matrix(128,50);}
    return 0;
  } catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}

#include "Runtime.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace {
void require(bool ok,const char* message)
{
  if(!ok)throw std::runtime_error(message);
}

class Rig {
public:
  Rig(char** paths,bool simulation=true,bool shadow=false,bool mixed=false)
    : runtime([](const auto&,const auto&,double,bool){},[](const auto*,std::size_t){return false;},simulation,shadow,
      [this]{return now;},MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,true)
  {
    require(runtime.load(paths[1],"telemetry") && runtime.load(paths[2],"external_navigation"),"actual modules did not load");
    if(mixed)require(runtime.load(paths[3],"parameters"),"parameter module did not load");
    runtime.configureModules({{"external_navigation",{{"source","fixture"}}}});
    runtime.setFirmwareRequestsEnabled(false);
    if(!simulation || shadow || mixed) {
      bool rejected=false;
      try {runtime.enableExternalNavigationBench();} catch(const std::invalid_argument&) {rejected=true;}
      require(rejected && !runtime.externalNavigationReady(),"unsafe composition opened typed bench");
      return;
    }
    runtime.enableExternalNavigationBench();
    runtime.finishStartup();
    runtime.setBoundedSend([this](const uint8_t* data,std::size_t size,double expires,const Runtime::SendEligibility& eligible) {
      mavlink_message_t buffer{},wire{};
      mavlink_status_t status{},result{};
      bool complete=false;
      for(size_t i=0;i<size;++i)
        complete=mavlink_frame_char_buffer(&buffer,&status,data[i],&wire,&result)==MAVLINK_FRAMING_OK;
      require(complete,"could not decode actual Runtime send");
      if(wire.msgid!=MAVLINK_MSG_ID_ODOMETRY)return eligible(now);
      ++odometry_handoffs;
      switch(fault) {
      case Fault::None: break;
      case Fault::Cancel: runtime.stopExternalNavigation(run); break;
      case Fault::Expiry: now=expires; break;
      case Fault::Armed: ground(true); break;
      }
      now+=verification_delay;
      const bool allowed=eligible(now);
      // The owned sender checks the original deadline again just before its write.
      last_attempt={expires,now,now+write_delay,allowed};
      now=last_attempt.write_at;
      if(!allowed || now>=expires)return false;
      require(wire.sysid==245 && wire.compid==191 && wire.magic==MAVLINK_STX,"incorrect actual source route");
      mavlink_odometry_t observation{};
      mavlink_msg_odometry_decode(&wire,&observation);
      observed.push_back(observation);
      send_times.push_back(now);
      return true;
    });
    ground(false);
  }

  void heartbeat(bool armed)
  {
    mavlink_message_t wire{};
    mavlink_msg_heartbeat_pack_status(1,1,&codec,&wire,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,
      armed?MAV_MODE_FLAG_SAFETY_ARMED:0,0,MAV_STATE_STANDBY);
    now+=.001;
    heartbeat_at=now;
    runtime.message(wire,heartbeat_at);
  }

  void landed(bool in_air)
  {
    mavlink_message_t wire{};
    mavlink_msg_extended_sys_state_pack_status(1,1,&codec,&wire,0,
      in_air?MAV_LANDED_STATE_IN_AIR:MAV_LANDED_STATE_ON_GROUND);
    now+=.001;
    landed_at=now;
    runtime.message(wire,landed_at);
  }

  void ground(bool armed)
  {
    heartbeat(armed);
    landed(armed);
  }

  bool begin(uint64_t id)
  {
    run=id;
    return runtime.beginExternalNavigation(id,now+external_navigation_lease_s);
  }

  ExternalNavigationSample sample(uint64_t sequence,float x=0)
  {
    runtime.checkpoint(now+=.001);
    ExternalNavigationSample value;
    value.run_id=run;value.sequence=sequence;value.capture_us=static_cast<uint64_t>(std::floor(now*1e6));
    value.capture_at=now;
    value.expires=externalNavigationCaptureTime(value.capture_us)+external_navigation_age_s;
    value.position={x,0,0};value.quaternion={1,0,0,0};
    value.pose_variance.fill(.04);value.angle_variance.fill(.01);
    value.velocity_variance.fill(.04);value.rate_variance.fill(.01);
    value.quality=90;
    return value;
  }

  void tick(double delta=.01) { runtime.tick(now+=delta); }

  void pumpUntilOdometry(unsigned count)
  {
    for(unsigned i=0;i<8 && odometry_handoffs<count;++i)tick(.005);
    require(odometry_handoffs==count,"protocol budget did not reach pending ExternalNav work");
  }

  enum class Fault {None,Cancel,Expiry,Armed};
  Fault fault=Fault::None;
  struct Attempt {
    double expires=0;
    double verified_at=0;
    double write_at=0;
    bool allowed=false;
  };
  Attempt last_attempt;
  double verification_delay=0;
  double write_delay=0;
  double heartbeat_at=0;
  double landed_at=0;
  double now=100;
  Runtime runtime;
  mavlink_status_t codec{};
  uint64_t run=1;
  unsigned odometry_handoffs=0;
  std::vector<mavlink_odometry_t> observed;
  std::vector<double> send_times;
};

void nominal(char** paths)
{
  Rig rig(paths);
  require(rig.runtime.externalNavigationReady() && !rig.runtime.externalNavigation(rig.sample(1)),
          "sample passed before explicit lease");
  require(rig.begin(1),"grounded dedicated host refused acquisition");
  auto older=rig.sample(1,1);
  auto newest=rig.sample(2,2);
  require(rig.runtime.externalNavigation(older) && rig.runtime.externalNavigation(newest),
          "valid samples did not queue");
  require(!rig.runtime.externalNavigation(older),"old sample replaced the new sample");
  rig.pumpUntilOdometry(1);
  require(rig.observed.size()==1 && rig.observed.back().x==2 &&
          rig.observed.back().time_usec==newest.capture_us,"latest-wins wire timestamp or position differs");
  require(rig.runtime.externalNavigation(rig.sample(3,3)),"next sequence refused");
  rig.tick();
  require(rig.observed.size()==1,"host exceeded 20 Hz");
  rig.tick(.051);
  rig.pumpUntilOdometry(2);
  require(rig.observed.size()==2 && rig.observed.back().x==3 &&
          rig.send_times[1]-rig.send_times[0]>=.05,"pending sample missed permitted rate window");
  rig.tick(.5);
  require(!rig.runtime.externalNavigation(rig.sample(4,4)) && !rig.begin(1),
          "source dropout preserved old lease");
  require(rig.begin(2) && rig.runtime.externalNavigation(rig.sample(1,5)),"explicit new lease unavailable");
  rig.runtime.disconnect(rig.now,"test_reset");
  rig.ground(false);
  rig.tick();
  require(rig.observed.size()==2 && !rig.begin(2),"connection reset revived pending work or reused run id");
}

void lateChecks(char** paths)
{
  for(auto fault:{Rig::Fault::Cancel,Rig::Fault::Expiry,Rig::Fault::Armed}) {
    Rig rig(paths);
    require(rig.begin(1) && rig.runtime.externalNavigation(rig.sample(1)),
            "late effect test could not queue sample");
    rig.fault=fault;
    rig.pumpUntilOdometry(1);
    require(rig.observed.empty(),"final owned send ignored cancellation, original expiry or arming");
  }
}

void pacingAfterEndpointProof(char** paths)
{
  Rig rig(paths);
  require(rig.begin(1) && rig.runtime.externalNavigation(rig.sample(1)),
          "could not queue first paced sample");
  rig.verification_delay=.045;
  rig.pumpUntilOdometry(1);
  require(rig.observed.size()==1,"slow endpoint proof did not yield the first write");

  rig.verification_delay=0;
  require(rig.runtime.externalNavigation(rig.sample(2)),"could not queue second paced sample");
  for(unsigned i=0;i<7;++i)rig.tick(.005);
  require(rig.observed.size()==1,"fast endpoint proof allowed a write less than 50 ms after the previous write");
  rig.tick(.016);
  rig.pumpUntilOdometry(2);
  require(rig.observed.size()==2 && rig.send_times[1]-rig.send_times[0]>=.05,
          "actual write times violated 20 Hz pacing");
}

void groundDeadlineAtWrite(char** paths,bool heartbeat_first)
{
  Rig rig(paths);
  const double deadline=(heartbeat_first?rig.heartbeat_at:rig.landed_at)+2;
  rig.tick(deadline-rig.now-.060);
  // Refresh just the other observation so each deadline is tested independently.
  if(heartbeat_first)rig.landed(false);
  else rig.heartbeat(false);
  require(rig.begin(1),"ground deadline fixture could not begin");
  const auto sample=rig.sample(1);
  require(rig.runtime.externalNavigation(sample),"could not queue ground deadline sample");
  rig.verification_delay=.003;
  rig.write_delay=.070;
  rig.pumpUntilOdometry(1);
  const auto& attempt=rig.last_attempt;
  require(attempt.allowed && attempt.verified_at<deadline && attempt.write_at>deadline &&
          attempt.write_at<sample.expires,"fixture did not cross only the ground deadline after verification");
  require(std::abs(attempt.expires-deadline)<1e-9,
          heartbeat_first?"transport expiry omitted heartbeat freshness":"transport expiry omitted landed freshness");
  require(rig.observed.empty(),"write passed after the ground observation expired");
}
}

int main(int argc,char** argv)
{
  try {
    require(argc==4,"pass telemetry, external_navigation and parameters modules");
    Rig passive(argv,false),shadow(argv,true,true),mixed(argv,true,false,true);
    nominal(argv);lateChecks(argv);
    pacingAfterEndpointProof(argv);
    groundDeadlineAtWrite(argv,true);
    groundDeadlineAtWrite(argv,false);
    std::cout << "ExternalNav host bench, latest-wins, pacing, session reset and final handoff checks passed\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

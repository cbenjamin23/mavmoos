#include "ExternalNavigationService.h"
#include "Plugin.h"

#include <cmath>
#include <dlfcn.h>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace {
constexpr char request_key[]="MAVMOOS_EXTERNAL_NAV_REQUEST";
constexpr char input_key[]="MAVMOOS_EXTERNAL_NAV_INPUT";
constexpr char status_key[]="MAVMOOS_EXTERNAL_NAV_STATUS";

void require(bool valid,const char* reason)
{
  if(!valid) throw std::runtime_error(reason);
}

struct Host final : Services, ExternalNavigationService {
  double time=100, expires=0;
  bool safe=true, admit=true;
  uint64_t run=0;
  Vehicle state;
  std::vector<ExternalNavigationSample> samples;
  std::vector<uint64_t> stopped;
  std::string status;

  double now() const override {return time;}
  const Vehicle& vehicle() const override {return state;}
  void publish(const std::string& key,const std::string& value) override
  {
    require(key==status_key,"plugin published unrelated MOOS state");
    status=value;
  }
  void publish(const std::string&,double) override { unexpected(); }
  bool sendCommand(const CommandRequest&) override { unexpected(); }
  bool acquireNavigation(const std::string&) override { unexpected(); }
  void revokeNavigation(const std::string&) override { unexpected(); }
  bool navigation(const NavigationTarget&) override { unexpected(); }

  bool externalNavigationReady() const override {return safe;}
  bool beginExternalNavigation(uint64_t id,double deadline) override
  {
    if(run || !safe || !admit) return false;
    run=id;
    expires=deadline;
    return true;
  }
  bool externalNavigation(const ExternalNavigationSample& sample) override
  {
    if(!safe || !admit || run!=sample.run_id || time>=expires) return false;
    samples.push_back(sample);
    return true;
  }
  void stopExternalNavigation(uint64_t id) override
  {
    if(id==run) {
      stopped.push_back(id);
      run=0;
    }
  }
  [[noreturn]] static void unexpected() { throw std::runtime_error("ExternalNav attempted unrelated control"); }
};

struct Rig {
  Host host;
  std::unique_ptr<Plugin,DestroyPlugin> plugin;
  ExternalNavigationListener* listener;

  Rig(CreatePlugin create,DestroyPlugin destroy)
    : plugin(create(plugin_abi,&host),destroy),listener(dynamic_cast<ExternalNavigationListener*>(plugin.get()))
  {
    require(plugin && listener,"actual ExternalNav DSO does not expose the listener");
    require(plugin->subscriptions().empty() && plugin->descriptor().capabilities.empty(),
            "provider enabled effects without configuration");
    require(plugin->configureModule({{"source","bad/source"}}).status==ModuleConfigStatus::Invalid,
            "invalid source accepted");
    require(plugin->configureModule({{"enabled","true"}}).status==ModuleConfigStatus::UnknownKey,
            "unknown option accepted");
    require(plugin->configureModule({{"source","producer"},{"source","producer"}}).status==ModuleConfigStatus::Invalid,
            "ambiguous source accepted");
    require(plugin->configureModule({{"source","producer"}}).status==ModuleConfigStatus::Accepted,
            "provider refused exact source");
    require(plugin->subscriptions()==std::vector<std::string>{request_key,input_key} &&
            plugin->descriptor().capabilities.size()==1 &&
            plugin->descriptor().capabilities.front().name==external_navigation_capability,
            "opt-in capability did not match MOOS inputs");
  }

  void mail(const char* key,const std::string& value,const std::string& source="producer",double age=0)
  {
    plugin->onMail({key,value,source,0,host.time-age,host.time,false});
  }

  void action(const char* action,uint64_t id,const char* source="producer",double age=0)
  {
    mail(request_key,"schema=1,action="+std::string(action)+",run="+std::to_string(id),source,age);
  }

  std::string sample(uint64_t id,uint64_t sequence,uint64_t capture_us) const
  {
    return "schema=1,run="+std::to_string(id)+",seq="+std::to_string(sequence)+",capture_us="+std::to_string(capture_us)+
      ",quality=90,frame=local_frd_body_frd,position=1/2/-3,velocity=.1/-.2/.3,rates=0/.1/-.1,"
      "quaternion=1/0/0/0,position_var=.04/.05/.06,angle_var=.01/.02/.03,velocity_var=.11/.12/.13,rate_var=.21/.22/.23";
  }
};

void wireChecks(const ExternalNavigationSample& sample)
{
  auto packet=makeExternalNavigationOdometry(sample);
  mavlink_message_t wire{};
  mavlink_status_t codec{};
  mavlink_msg_odometry_encode_status(245,191,&codec,&wire,&packet);
  mavlink_odometry_t received{};
  mavlink_msg_odometry_decode(&wire,&received);
  require(wire.msgid==MAVLINK_MSG_ID_ODOMETRY && wire.magic==MAVLINK_STX && wire.sysid==245 && wire.compid==191,
          "wire identity changed");
  require(received.frame_id==MAV_FRAME_LOCAL_FRD && received.child_frame_id==MAV_FRAME_BODY_FRD &&
          received.estimator_type==MAV_ESTIMATOR_TYPE_VISION && received.quality==90 && received.reset_counter==0 &&
          received.time_usec==sample.capture_us,"wire provenance changed");
  require(received.x==1 && received.y==2 && received.z==-3 && received.vx==.1f && received.vy==-.2f &&
          received.vz==.3f && received.pitchspeed==.1f && received.yawspeed==-.1f &&
          received.q[0]==1 && received.q[1]==0,"wire frame vectors changed");
  require(received.pose_covariance[0]==.04f && received.pose_covariance[6]==.05f &&
          received.pose_covariance[11]==.06f && received.pose_covariance[15]==.01f &&
          received.pose_covariance[18]==.02f && received.pose_covariance[20]==.03f &&
          received.velocity_covariance[0]==.11f && received.velocity_covariance[6]==.12f &&
          received.velocity_covariance[11]==.13f && received.velocity_covariance[15]==.21f &&
          received.velocity_covariance[18]==.22f && received.velocity_covariance[20]==.23f &&
          received.pose_covariance[1]==0 && received.velocity_covariance[19]==0,"wire covariance changed");
}

void exercise(CreatePlugin create,DestroyPlugin destroy)
{
  Rig rig(create,destroy);
  rig.action("begin",1,"foreign");
  rig.action("begin",1,"producer",.5);
  require(rig.host.run==0,"foreign or stale begin acquired");
  rig.host.safe=false;
  rig.action("begin",1);
  require(rig.host.run==0,"unavailable bench acquired");
  rig.host.safe=true;
  rig.action("begin",1,"producer",.01);
  require(rig.host.run==1 && std::abs(rig.host.expires-129.99)<.0001,"begin did not keep the original lease deadline");

  rig.host.time=100.1;
  rig.mail(input_key,rig.sample(1,1,99999999));
  rig.mail(input_key,rig.sample(1,1,100500000));
  rig.mail(input_key,rig.sample(1,1,100090000),"foreign");
  rig.mail(input_key,rig.sample(1,1,100090000)+",quality=90");
  rig.mail(input_key,rig.sample(1,1,100090000),"producer",.1);
  auto wrong=rig.sample(1,1,100090000);
  wrong.replace(wrong.find("local_frd_body_frd"),18,"local_ned_body_frd");
  rig.mail(input_key,wrong);
  auto invalid=rig.sample(1,1,100090000);
  invalid.replace(invalid.find("quaternion=1"),12,"quaternion=2");
  rig.mail(input_key,invalid);
  require(rig.host.samples.empty(),"bad source/frame/time/shape fed the service");

  rig.mail(input_key,rig.sample(1,1,100090000),"producer",.02);
  require(rig.host.samples.empty(),"sample appeared to be published before measurement");
  rig.mail(input_key,rig.sample(1,1,100090000));
  require(rig.host.samples.size()==1 && std::abs(rig.host.samples[0].expires-100.34)<.0001,
          "accepted sample expired from receipt instead of capture");
  auto sample=rig.host.samples[0];
  wireChecks(sample);
  require(!validExternalNavigationSample(sample,sample.expires),"sample survived original deadline");
  sample.pose_variance[0]=std::numeric_limits<float>::infinity();
  require(!validExternalNavigationSample(sample,rig.host.time),"non-finite covariance accepted");
  rig.listener->onExternalNavigationSent(rig.host.samples[0],100.12,true);
  rig.listener->onExternalNavigationSent(rig.host.samples[0],100.13,true);
  rig.host.time=100.2;
  rig.mail(input_key,rig.sample(1,1,100190000));
  rig.mail(input_key,rig.sample(1,2,100080000));
  rig.action("begin",2);
  rig.action("stop",1,"foreign");
  rig.plugin->iterate();
  require(rig.host.samples.size()==1 && rig.host.run==1 && rig.host.status.find(",sent=1,")!=std::string::npos,
          "duplicate, rollback, second begin or foreign stop disturbed the run");

  rig.host.time=100.60001;
  rig.mail(input_key,rig.sample(1,2,100599000));
  require(rig.host.run==0 && rig.host.stopped==std::vector<uint64_t>{1} &&
          rig.host.status.find("reason=source_dropout")!=std::string::npos,
          "fresh input revived a run after the old source deadline");
  rig.action("begin",1);
  require(rig.host.run==0,"retired run id was reusable");
  rig.action("begin",2);
  rig.action("stop",2);
  require(rig.host.run==0 && rig.host.stopped==std::vector<uint64_t>{1,2} &&
          rig.host.status.find("reason=explicit_stop")!=std::string::npos,"explicit stop failed");
  rig.action("begin",3);
  rig.host.safe=false;
  rig.plugin->onInputBoundary();
  require(rig.host.run==0 && rig.host.stopped.back()==3,"ground/identity loss retained a run");
  rig.host.safe=true;
  rig.action("begin",4);
  rig.plugin->onReset("fixture_reset");
  require(rig.host.run==0 && rig.host.stopped.back()==4,"connection reset retained a run");
}
}

int main(int argc,char** argv)
{
  try {
    require(argc==2,"usage: external_navigation_provider_tests module");
    void* handle=dlopen(argv[1],RTLD_NOW|RTLD_LOCAL);
    if(!handle) throw std::runtime_error(dlerror());
    auto create=reinterpret_cast<CreatePlugin>(dlsym(handle,"mavmoos_create_plugin"));
    auto destroy=reinterpret_cast<DestroyPlugin>(dlsym(handle,"mavmoos_destroy_plugin"));
    require(create && destroy,"cannot load ExternalNav module factory");
    exercise(create,destroy);
    dlclose(handle);
    std::cout << "ExternalNav producer provenance, wire shape, age, dropout and lease boundaries passed\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

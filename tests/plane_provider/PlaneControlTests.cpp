#include "PlaneFixture.h"
#include "PluginHost.h"
#include <algorithm>
#include <utility>
#include <vector>

using namespace plane_fixture;

namespace {
class TestServices final:public Services {
public:
  double now() const override { return clock; }
  const Vehicle& vehicle() const override { return flight; }
  void publish(const std::string& key,const std::string& value) override
  { publications.emplace_back(key,value); }
  void publish(const std::string&,double) override {}
  bool sendCommand(const CommandRequest& request) override
  {
    commands.push_back(request);
    if(request.command!=MAV_CMD_GUIDED_CHANGE_SPEED)grant=false;
    busy=true;
    return true;
  }
  bool acquireNavigation(const std::string&) override
  { ++acquisitions;grant=true;return true; }
  void revokeNavigation(const std::string&) override { grant=false; }
  bool navigation(const NavigationTarget& target) override
  {
    if(!grant)return false;
    targets.push_back(target);
    return true;
  }
  bool navigationGranted() const override { return grant; }
  bool controlBusy() const override { return busy; }
  bool projectGlobal(double,double,double& x,double& y) const override
  { x=12;y=-7;return projection; }
  void commandFinished(const std::string& id) override
  { finished.push_back(id);busy=false; }

  bool reportContains(const std::string& key,const std::string& term) const
  {
    return std::any_of(publications.begin(),publications.end(),[&](const auto& report) {
      return report.first==key&&report.second.find(term)!=std::string::npos;
    });
  }

  double clock=1;
  Vehicle flight;
  bool grant=false,busy=false;
  bool projection=false;
  unsigned acquisitions=0;
  std::vector<std::pair<std::string,std::string>> publications;
  std::vector<CommandRequest> commands;
  std::vector<std::string> finished;
  std::vector<NavigationTarget> targets;
};

void prepare(PluginHost& host,TestServices& services,double at,unsigned mode,float airspeed=22)
{
  services.clock=at;
  services.flight=airborne(at,mode);
  host.dispatchMessage(state(),at);
  host.dispatchMessage(sensor(airspeed),at);
}

void modeEvidence(PluginHost& host,TestServices& services,unsigned mode,double effective,double read)
{
  services.clock=read;
  services.flight.mode=mode;
  services.flight.heartbeat_at=effective;
  mavlink_message_t heartbeat{};
  mavlink_msg_heartbeat_pack(1,1,&heartbeat,MAV_TYPE_FIXED_WING,MAV_AUTOPILOT_ARDUPILOTMEGA,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED,mode,MAV_STATE_ACTIVE);
  host.dispatchMessage(heartbeat,{effective,read,"QUEUE_EMPTY_FLOOR"});
}

void request(PluginHost& host,const TestServices& services,const std::string& text,double published)
{
  host.dispatchMail({"MAVMOOS_COMMAND",text,"operator",0,published,services.now(),false});
}

bool result(const TestServices& services,const std::string& id,const std::string& action,const std::string& status)
{
  return services.reportContains("MAVMOOS_COMMAND_RESULT","profile=arduplane,id="+id+",action="+action+",status="+status+",");
}

void testNativeOperations(const char* path)
{
  TestServices services;
  PluginHost host;
  require(host.load(path,services,"arduplane_control"),"could not load standard ArduPlane DSO");
  require(host.controlBackend()&&host.controlBackend()->profile()=="arduplane"&&
    host.hasCapability("control.navigation.helm")&&host.hasCapability("control.hold.native_loiter")&&
    host.hasCapability("control.return.native_rtl"),"missing Plane capability ownership");

  prepare(host,services,2,plane::guided_mode);
  host.iterate();
  require(services.reportContains("MAVMOOS_PLANE_STATUS","flight_ready=true"),
    "qualified conventional state did not reach MOOS status");
  request(host,services,"id=unowned,action=rtl",1.9);
  require(result(services,"unowned","rtl","REJECTED")&&services.commands.empty(),
    "operator request escaped fresh Guided ownership");

  services.grant=true;
  request(host,services,"id=future,action=rtl",2.2);
  request(host,services,"id=ground,action=arm",1.9);
  request(host,services,"id=unsupported,action=unsupported",1.9);
  require(result(services,"future","rtl","REJECTED")&&result(services,"invalid","invalid","REJECTED")&&
    result(services,"ground","arm","REJECTED"),"invalid, future or airborne ground request was not rejected");
  request(host,services,"action=loiter,id=handoff",1.9);
  require(result(services,"handoff","loiter","QUEUED")&&services.commands.empty(),
    "operator request should wait for Iterate");
  request(host,services,"id=handoff,action=loiter",1.9);
  require(result(services,"handoff","loiter","DUPLICATE"),"duplicate mode request was admitted");
  host.iterate();
  require(services.commands.size()==1,"queued handback was not submitted once");
  const auto command=services.commands.front();
  require(command.command==MAV_CMD_DO_SET_MODE&&command.params[1]==plane::loiter_mode&&
    command.expires==3.9&&command.operation_expires==5.9,"native command lost original mail bounds");
  services.clock=2.1;
  host.commandSent(command,2.1,true);
  require(result(services,"handoff","loiter","SENT"),"missing local send result");

  mavlink_message_t ack{};
  mavlink_msg_command_ack_pack(1,1,&ack,MAV_CMD_DO_SET_MODE,MAV_RESULT_ACCEPTED,0,0,245,191);
  services.clock=2.15;
  host.dispatchMessage(ack,2.15);
  modeEvidence(host,services,plane::loiter_mode,2.05,2.2);
  require(!result(services,"handoff","loiter","CONFIRMED"),"ACK or old FC observation confirmed the mode");
  modeEvidence(host,services,plane::loiter_mode,2.3,2.3);
  require(result(services,"handoff","loiter","CONFIRMED")&&services.finished.size()==1,
    "missing post-handoff mode confirmation and host lease cleanup");

  prepare(host,services,3,plane::guided_mode);
  services.grant=true;
  request(host,services,"id=return,action=rtl",2.9);
  host.iterate();
  require(services.commands.size()==2&&services.commands.back().params[1]==plane::rtl_mode,
    "native RTL did not issue its distinct typed mode");
  services.clock=3.1;
  host.commandSent(services.commands.back(),3.1,true);
  modeEvidence(host,services,5,3.2,3.2);
  require(result(services,"return","rtl","CANCELLED")&&services.finished.size()==2,
    "an independently observed pilot mode did not cancel pending RTL");

  prepare(host,services,4,plane::guided_mode);
  services.grant=true;
  request(host,services,"id=unconfirmed,action=loiter",3.9);
  host.iterate();
  services.clock=4.1;
  host.commandSent(services.commands.back(),4.1,true);
  services.clock=8.0;
  host.iterate();
  require(services.commands.size()==3&&result(services,"unconfirmed","loiter","TIMED_OUT"),
    "unconfirmed transaction should expire without a retry");
  host.reset("vehicle_connection_lost");
  prepare(host,services,8.2,plane::guided_mode);
  services.grant=true;
  request(host,services,"id=replayed,action=loiter",7.9);
  require(result(services,"replayed","loiter","REJECTED"),"pre-reset request was admitted");
}

void helmInputs(PluginHost& host,const TestServices& services,const std::string& source,double speed)
{
  const auto now=services.now();
  host.dispatchMail({"IVPHELM_STATE","DRIVE",source,0,now,now,false});
  host.dispatchMail({"IVPHELM_ALLSTOP","clear",source,0,now,now,false});
  host.dispatchMail({"DESIRED_HEADING","",source,90,now,now,true});
  host.dispatchMail({"DESIRED_SPEED","",source,speed,now,now,true});
  host.dispatchMail({"DESIRED_ALTITUDE","",source,75,now,now,true});
}

void testSpeedReceiptAndPhysicalEffect(const char* path)
{
  TestServices services;
  PluginHost host;
  require(host.load(path,services,"arduplane_control"),"could not load Plane speed workflow");
  prepare(host,services,2,plane::guided_mode);
  services.grant=true;
  request(host,services,"id=slower,action=change_speed,delta=-4",1.9);
  host.iterate();
  require(services.commands.size()==1&&services.commands.front().params[1]==18,
    "a bounded Plane speed adjustment was not submitted");
  services.clock=2.05;
  host.commandSent(services.commands.front(),2.05,true);
  mavlink_message_t ack{};
  mavlink_msg_command_ack_pack(1,1,&ack,MAV_CMD_GUIDED_CHANGE_SPEED,MAV_RESULT_ACCEPTED,0,0,245,191);
  services.clock=2.08;
  host.dispatchMessage(ack,{2.04,2.08,"QUEUE_EMPTY_FLOOR"});
  host.iterate();
  require(!result(services,"slower","change_speed","COMPLETED"),"ACK alone completed the speed adjustment");
  for(int sample=0;sample<=8;++sample) {
    const double at=2.25+sample*.25;
    prepare(host,services,at,plane::guided_mode,18);
    host.iterate();
  }
  require(result(services,"slower","change_speed","COMPLETED")&&services.grant&&services.finished.size()==1,
    "post-send receipt and measured airspeed did not complete under the existing grant");
}

void testStockHelmFacet(const char* path)
{
  TestServices services;
  PluginHost host;
  require(host.load(path,services,"arduplane_control"),"could not load Plane helm facet");
  host.configure("control_native_actions","false");
  require(!host.hasCapability("control.hold.native_loiter")&&host.hasCapability("control.navigation.helm"),
    "disabling native actions disabled or relabelled the helm facet");
  const auto subscriptions=host.subscriptions();
  require(std::find(subscriptions.begin(),subscriptions.end(),"DESIRED_ALTITUDE")!=subscriptions.end()&&
    std::find(subscriptions.begin(),subscriptions.end(),"MAVMOOS_COMMAND")==subscriptions.end(),
    "fixed-wing helm subscription set is incorrect");

  prepare(host,services,2,plane::loiter_mode);
  host.dispatchMail({"MAVMOOS_NAV_CONTROL","acquire","operator",0,1.9,2,false});
  helmInputs(host,services,"pHelmIvP",22);
  host.iterate();
  require(services.acquisitions==1&&services.targets.empty(),"pre-Guided output escaped the adapter");

  prepare(host,services,2.2,plane::guided_mode);
  helmInputs(host,services,"other-producer",22);
  host.iterate();
  require(services.targets.empty(),"unbound producer supplied the Plane target");
  prepare(host,services,2.3,plane::guided_mode);
  helmInputs(host,services,"pHelmIvP",22);
  host.iterate();
  require(services.targets.size()==1&&services.targets.front().course==90&&
    services.targets.front().speed==22&&services.targets.front().height==75,
    "stock pHelmIvP scalar adapter did not supply the typed Plane intent");

  prepare(host,services,2.4,plane::guided_mode);
  helmInputs(host,services,"pHelmIvP",0);
  host.iterate();
  require(!services.grant&&services.targets.size()==1&&
    services.reportContains("MAVMOOS_HELM_STATUS","state=LATCHED,reason=input_out_of_envelope"),
    "zero-speed producer intent did not fence Plane navigation");
}

void testHomeMarker(const char* path)
{
  TestServices services;
  PluginHost host;
  require(host.load(path,services,"arduplane_control"),"could not load Plane marker facet");
  prepare(host,services,5,plane::loiter_mode);
  const auto request=[&](const std::string& id) {
    host.dispatchMail({"MAVMOOS_CONTROL_REQUEST","schema=1,id="+id+",action=visualize_home",
      "operator",0,services.clock,services.clock,false});
  };
  request("unprojected");
  require(services.reportContains("MAVMOOS_CONTROL_RESULT","reason=fresh_projected_home_required"),
    "home without a projection was accepted");
  services.projection=true;
  services.flight.home_at=1;
  request("old");
  require(!services.reportContains("VIEW_MARKER","label=MAVMOOS_HOME"),"stale home emitted a marker");
  services.flight.home_at=5;
  request("visible");
  require(services.reportContains("VIEW_MARKER","x=12,y=-7,label=MAVMOOS_HOME")&&
    services.reportContains("MAVMOOS_CONTROL_RESULT","id=visible,action=visualize_home,status=COMPLETED")&&
    services.commands.empty()&&services.acquisitions==0,"home marker was missing or exercised vehicle authority");
}
} // namespace

int main(int argc,char* argv[])
{
  try {
    require(argc==2,"expected standard Plane DSO path");
    testNativeOperations(argv[1]);
    testSpeedReceiptAndPhysicalEffect(argv[1]);
    testStockHelmFacet(argv[1]);
    testHomeMarker(argv[1]);
    std::cout<<"Plane provider DSO, observed mode transactions, and stock helm facade passed\n";
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';
    return 1;
  }
}

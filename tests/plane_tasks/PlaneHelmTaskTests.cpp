#include "PlaneHelmTasks.h"
#include "ControlComponents.h"
#include "../plane_provider/PlaneFixture.h"
#include <algorithm>
#include <functional>
#include <set>
#include <vector>

using namespace plane_fixture;
namespace {
struct Host final : Services {
  double clock=10,paused_until=0;
  Vehicle state=airborne(clock,plane::loiter_mode);
  bool grant=false,busy=false,projection=true,acquisition_allowed=true;
  unsigned acquisitions=0;
  std::vector<NavigationTarget> targets;
  std::vector<std::pair<std::string,std::string>> reports;
  double now() const override {return clock;}
  const Vehicle& vehicle() const override {return state;}
  void publish(const std::string& key,const std::string& value) override {reports.emplace_back(key,value);}
  void publish(const std::string&,double) override {}
  bool sendCommand(const CommandRequest&) override {throw std::runtime_error("task requested a native command");}
  bool acquireNavigation(const std::string&) override {++acquisitions;grant=acquisition_allowed;return grant;}
  void revokeNavigation(const std::string&) override {grant=false;paused_until=0;}
  bool navigation(const NavigationTarget& target) override {
    if(!grant||target.expires<=clock)return false;
    targets.push_back(target);return true;
  }
  bool navigationGranted() const override {return grant;}
  bool controlBusy() const override {return busy;}
  bool projectGlobal(double,double,double& x,double& y) const override {x=12;y=-7;return projection;}
  bool suspendNavigation(double until) override {if(!grant)return false;grant=false;paused_until=until;return true;}
  bool resumeNavigation(const std::string&) override {if(clock>=paused_until)return false;grant=true;return true;}
  bool has(const std::string& key,const std::string& value) const {
    return std::any_of(reports.begin(),reports.end(),[&](const auto& row) {return row.first==key&&row.second.find(value)!=std::string::npos;});
  }
};

struct Rig {
  Host host;
  plane::Backend backend{host.clock};
  std::set<std::string> history;
  plane::HelmTasks tasks{host,backend,history};
  std::unique_ptr<Plugin> helm=makeHelmAdapter(tasks.helmServices(),backend);
  Rig(bool enabled=true) {
    tasks.configure("operation_helm_enabled",enabled?"true":"false");
    tasks.configure("operation_helm_tasks","return,survey,voronoi,waypoint");
    fresh(.01);
    stringMail("IVPHELM_STATE","DRIVE");
    stringMail("IVPHELM_ALLSTOP","clear");
  }
  void fresh(double delta=.1,unsigned mode=0xffffffff) {
    host.clock+=delta;
    if(mode==0xffffffff)mode=host.state.mode;
    host.state=airborne(host.clock,mode);
    const MessageTime time{host.clock,host.clock,"READ_ACQUISITION"};
    backend.observe(state(),time,host.state,host.clock);
    backend.observe(sensor(),time,host.state,host.clock);
  }
  Mail mail(const std::string& key,const std::string& text,double published=-1) const {
    Mail result;result.key=key;result.text=text;result.source="operator";
    result.published_at=published<0?host.clock:published;result.received_at=host.clock;return result;
  }
  void stringMail(const std::string& key,const std::string& value) {
    auto message=mail(key,value);message.source="pHelmIvP";helm->onMail(message);
  }
  void demands() {
    stringMail("IVPHELM_STATE","DRIVE");
    for(const auto& [key,value]:std::vector<std::pair<std::string,double>>{
        {"DESIRED_HEADING",45},{"DESIRED_SPEED",22},{"DESIRED_ALTITUDE",75}}) {
      auto message=mail(key,{});message.numeric=true;message.value=value;message.source="pHelmIvP";
      helm->onMail(message);
    }
  }
  void cycle() {tasks.beforeHelm(*helm);helm->onInputBoundary();helm->iterate();tasks.afterHelm(*helm);}
  bool select(const std::string& action="survey",const std::string& id="task",const std::string& extra="",bool busy=false) {
    return tasks.onRequest(mail("MAVMOOS_CONTROL_REQUEST","schema=1,id="+id+",action="+action+",strategy=helm"+extra),*helm,busy);
  }
  void acquire(double stamp=-1) {require(tasks.onNavControl(mail("MAVMOOS_NAV_CONTROL","acquire",stamp),*helm),"task did not consume acquire");}
  void ready(const std::string& action="survey",const std::string& source="pHelmIvP",double stamp=-1) {
    auto message=mail("MAVMOOS_HELM_TASK_READY",action,stamp);message.source=source;
    require(tasks.onReadiness(message),"task did not consume readiness");
  }
  void active(const std::string& action="survey",const std::string& extra="") {
    fresh();require(select(action,"task",extra),"task not consumed");
    fresh();ready(action);acquire();require(host.grant,"explicit acquire failed");
    fresh(.1,plane::guided_mode);demands();cycle();
    require(host.has("MAVMOOS_CONTROL_RESULT","status=ACTIVE")&&!host.targets.empty(),"fresh stock-helm task did not become active");
  }
};

void selectionAndFreshConsent()
{
  Rig r;
  r.demands();
  r.fresh();r.select("return");
  const auto selected=r.host.clock;
  require(r.host.acquisitions==0&&r.host.targets.empty()&&r.tasks.busy()&&
          r.host.has("MAVMOOS_OPERATION_RETURN_UPDATE","points=12,-7")&&
          r.host.has("MAVMOOS_HELM_OPERATION","return"),"selection granted control or omitted stock update");
  r.acquire(selected);require(r.host.acquisitions==0,"same-stamp acquire crossed task boundary");
  r.fresh();r.ready("survey");r.ready("return","other");r.ready("return","pHelmIvP",selected);
  r.acquire();require(r.host.acquisitions==0,"wrong action, foreign source or old readiness enabled acquisition");
  r.ready("return");require(r.host.acquisitions==0,"readiness replayed a previously refused acquire");
  r.fresh();r.acquire();require(r.host.acquisitions==1,"new explicit acquire rejected");
  r.fresh(.1,plane::guided_mode);r.cycle();
  require(r.host.targets.empty(),"pre-selection cached scalars became a task target");
  r.fresh();r.demands();r.cycle();
  require(!r.host.targets.empty()&&r.host.has("MAVMOOS_HELM_STATUS","allstop=clear"),
          "retained clear all-stop was lost during synthetic release");
}

void originalExpiryAndNormalPause()
{
  Rig r;r.active("survey",",timeout=6");const auto expiry=*r.tasks.deadline();
  r.host.clock=expiry-.2;r.fresh(0);r.demands();r.cycle();
  require(r.host.targets.back().expires==expiry,"task deadline was renewed by fresh scalar input");
  r.stringMail("IVPHELM_ALLSTOP","NothingToDo");r.cycle();
  require(r.tasks.busy()&&!r.host.grant&&r.host.paused_until==expiry&&
          r.host.has("MAVMOOS_HELM_STATUS","state=PAUSED"),"normal pause lost task or exceeded original lifetime");
  r.host.clock=expiry-.1;r.fresh(0);r.stringMail("IVPHELM_ALLSTOP","clear");r.demands();r.cycle();
  require(r.tasks.busy()&&r.host.grant&&r.host.targets.back().expires==expiry,"bounded normal pause failed to resume");
  const auto count=r.host.targets.size();
  r.host.clock=expiry+.001;r.fresh(0);r.cycle();r.demands();r.cycle();
  require(!r.tasks.busy()&&!r.host.grant&&r.host.targets.size()==count&&
          r.host.has("MAVMOOS_CONTROL_RESULT","status=TIMED_OUT"),"expiry admitted a target or kept selector ownership");
}

void takeoverAndExplicitRelease()
{
  for(const auto action:{"return","survey","voronoi","waypoint"}) {
    Rig r;r.active(action);const auto count=r.host.targets.size();
    r.fresh(.1,0);r.cycle();
    require(!r.tasks.busy()&&!r.host.grant&&r.host.targets.size()==count&&
            r.host.has("MAVMOOS_HELM_OPERATION","none"),"pilot takeover left task active");
    r.fresh(.1,plane::loiter_mode);r.demands();r.cycle();
    require(!r.host.grant&&r.host.acquisitions==1,"mode return silently reacquired task");
  }
  Rig r;r.active();const auto count=r.host.targets.size();r.fresh();
  require(r.tasks.onNavControl(r.mail("MAVMOOS_NAV_CONTROL","release"),*r.helm),"task did not consume release");
  r.cycle();require(!r.tasks.busy()&&!r.host.grant&&r.host.targets.size()==count,"explicit release retained task demand");
}

void rejectedAdmission()
{
  Rig disabled(false);disabled.fresh();disabled.select();
  require(!disabled.tasks.busy()&&disabled.tasks.capabilities().empty(),"disabled tasks advertised or selected");
  Rig busy;busy.fresh();busy.select("survey","a",{},true);busy.host.busy=true;busy.select("survey","b");
  require(!busy.tasks.busy()&&busy.host.acquisitions==0,"direct/native/mission ownership admitted selection");
  Rig mode;mode.fresh(.1,plane::guided_mode);mode.select();require(!mode.tasks.busy(),"task selected outside native Loiter");
  Rig projection;projection.host.projection=false;projection.fresh();projection.select("return");
  require(!projection.tasks.busy(),"return task ignored missing projection");
  Rig history;history.history.insert("used");history.fresh();history.select("survey","used");
  require(!history.tasks.busy(),"task reused a provider operation ID");
  Rig window;window.fresh();window.select("survey","task",",timeout=6");window.fresh(1.2);window.ready();window.acquire();
  require(window.host.acquisitions==0&&window.host.has("MAVMOOS_CONTROL_RESULT","status=ACQUIRE_REJECTED"),
          "task admitted a mode request with less than its full host window remaining");
}

void cancelResetAndStaleInputs()
{
  Rig r;r.fresh();r.select();const auto deadline=*r.tasks.deadline();r.fresh();
  require(!r.tasks.onRequest(r.mail("MAVMOOS_CONTROL_REQUEST","schema=1,id=wrong,action=cancel,strategy=guided,target=task"),*r.helm,false)&&r.tasks.busy(),
          "foreign strategy cancelled the selected helm task");
  require(r.tasks.onRequest(r.mail("MAVMOOS_CONTROL_REQUEST","schema=1,id=cancel,action=cancel,target=task"),*r.helm,false),
          "matching task cancel was not consumed");
  require(!r.tasks.busy()&&!r.host.grant&&r.host.has("MAVMOOS_CONTROL_RESULT","id=cancel,action=cancel,strategy=helm,status=COMPLETED"),
          "task cancellation failed");
  r.fresh();r.select("voronoi","reset");r.tasks.onReset("test_reset");r.helm->onReset("test_reset");
  r.cycle();require(!r.tasks.busy()&&!r.host.grant&&r.history.count("reset"),"reset restored task or erased history");
  Rig stale;stale.fresh(3);auto mail=stale.mail("MAVMOOS_CONTROL_REQUEST","schema=1,id=stale,action=survey,strategy=helm",stale.host.clock-2.1);
  stale.tasks.onRequest(mail,*stale.helm,false);require(!stale.tasks.busy(),"stale selector admitted");
  Rig delayed;delayed.fresh(1);auto request=delayed.mail("MAVMOOS_CONTROL_REQUEST","schema=1,id=delayed,action=survey,strategy=helm,timeout=9",delayed.host.clock-.7);
  delayed.tasks.onRequest(request,*delayed.helm,false);
  require(delayed.tasks.deadline()&&*delayed.tasks.deadline()==request.published_at+9&&deadline>0,
          "task lifetime started at receipt instead of publication");
}

void observationAndAuthorityLoss()
{
  Rig home;home.active();home.fresh();home.host.state.home_msl+=1;home.cycle();
  require(!home.tasks.busy()&&!home.host.grant,"HOME replacement retained task authority");
  Rig stale;stale.active();stale.host.clock+=3;stale.cycle();
  require(!stale.tasks.busy()&&!stale.host.grant,"stale vehicle observation retained task authority");
  Rig native;native.active();native.host.busy=true;native.fresh();native.cycle();
  require(!native.tasks.busy()&&!native.host.grant,"native command lease did not supersede task");
  Rig allstop;allstop.active();allstop.stringMail("IVPHELM_ALLSTOP","BehaviorError");allstop.cycle();
  require(!allstop.tasks.busy()&&!allstop.host.grant,"invalid helm stop retained selected task");
  Rig denied;denied.fresh();denied.select();denied.host.acquisition_allowed=false;denied.fresh();denied.ready();denied.acquire();denied.cycle();
  require(denied.tasks.busy()&&!denied.host.grant&&denied.host.acquisitions==1&&
          denied.host.has("MAVMOOS_CONTROL_RESULT","status=ACQUIRE_REJECTED"),
          "failed acquisition granted task permission");
  denied.host.acquisition_allowed=true;denied.fresh();denied.acquire();
  require(denied.host.grant&&denied.host.acquisitions==2,"fresh explicit acquire could not retry an unacquired selector");
  Rig expired;expired.fresh();expired.select("survey","task",",timeout=6");
  expired.host.clock=*expired.tasks.deadline();expired.acquire();
  require(!expired.tasks.busy()&&!expired.host.grant&&expired.host.acquisitions==0,
          "expiry at acquire ingress reached the host");
  Rig clock;clock.active();clock.host.clock-=.01;
  clock.tasks.onRequest(clock.mail("MAVMOOS_CONTROL_REQUEST","schema=1,id=next,action=survey,strategy=helm"),*clock.helm,false);
  require(!clock.tasks.busy()&&!clock.host.grant&&clock.host.acquisitions==1,
          "backward clock at selector ingress renewed task authority");
}
} // namespace

int main()
{
  try {
    selectionAndFreshConsent();originalExpiryAndNormalPause();takeoverAndExplicitRelease();
    rejectedAdmission();cancelResetAndStaleInputs();observationAndAuthorityLoss();
    std::cout<<"Plane helm task component and actual adapter 6/6 passed\n";
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}

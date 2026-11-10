#include "ControlComponents.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void require(bool condition,const std::string& message) { if(!condition) throw std::runtime_error(message); }
struct Host final : Services {
  double time=20;
  Vehicle state;
  std::map<std::string,std::string> strings;
  std::vector<NavigationTarget> targets;
  std::vector<std::string> revocations;
  unsigned acquisitions=0;
  double resume_until=0;
  bool allow_acquire=true,allow_nav=true,granted=false;
  Host() { state.heartbeat_valid=state.position_valid=state.home_valid=state.landed_valid=true;
    state.armed=state.in_air=true; state.mode=4; state.relative_height=10; fresh(20); }
  void fresh(double t) { time=t; state.heartbeat_at=state.position_at=state.landed_at=t; }
  double now() const override { return time; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key,const std::string& value) override { strings[key]=value; }
  void publish(const std::string&,double) override {}
  bool sendCommand(const CommandRequest&) override { return false; }
  bool acquireNavigation(const std::string&) override { ++acquisitions; granted=allow_acquire; return granted; }
  bool navigationGranted() const override { return granted; }
  void revokeNavigation(const std::string& reason) override { revocations.push_back(reason); resume_until=0; granted=false; }
  bool suspendNavigation(double until) override { revokeNavigation("normal_handoff");resume_until=until;return true; }
  bool resumeNavigation(const std::string& id) override { if(time>=resume_until)return false;resume_until=0;return acquireNavigation(id); }
  bool navigation(const NavigationTarget& target) override { targets.push_back(target); return allow_nav; }
};
using Owner=std::unique_ptr<Plugin>;
Owner plugin(Host& host) {
  static const auto backend=makeArduCopterBackend();
  return makeHelmAdapter(host,*backend);
}
void text(Plugin& p,Host& h,const std::string& key,const std::string& value,double at,
          const std::string& source="pHelmIvP",double published=-1) {
  h.fresh(at); Mail m; m.key=key;m.text=value;m.source=source;m.received_at=at;
  m.published_at=published<0?at:published; p.onMail(m);
}
void scalar(Plugin& p,Host& h,const std::string& key,double value,double at,
            const std::string& source="pHelmIvP",double published=-1) {
  h.fresh(at); Mail m; m.key=key;m.value=value;m.numeric=true;m.source=source;m.received_at=at;
  m.published_at=published<0?at:published; p.onMail(m);
}
void fields(Plugin& p,Host& h,double at,double course=90,double speed=2,double height=10) {
  scalar(p,h,"DESIRED_HEADING",course,at); scalar(p,h,"DESIRED_SPEED",speed,at);
  scalar(p,h,"DESIRED_ALTITUDE",height,at);
}
void acquire(Plugin& p,Host& h) {
  text(p,h,"IVPHELM_ALLSTOP","clear",20.01);
  text(p,h,"IVPHELM_STATE","DRIVE",20.02);
  text(p,h,"MAVMOOS_NAV_CONTROL","acquire",20.03,"operator");
}
void active(Plugin& p,Host& h) { acquire(p,h); fields(p,h,20.04); p.iterate(); require(!h.targets.empty(),"initial target missing"); }
bool status(const Host& h,const std::string& value) {
  const auto it=h.strings.find("MAVMOOS_HELM_STATUS");
  return it!=h.strings.end() && it->second.find(value)!=std::string::npos;
}
bool statusField(const Host& h,const std::string& key,const std::string& value) {
  const auto it=h.strings.find("MAVMOOS_HELM_STATUS");
  if(it==h.strings.end()) return false;
  const std::string wrapped=','+it->second+',';
  return wrapped.find(','+key+'='+value+',')!=std::string::npos;
}
void sample(const Host& h,const std::string& scenario) {
  std::cout << "STATUS " << scenario << ' ' << h.strings.at("MAVMOOS_HELM_STATUS") << '\n';
}
}
int main() {
  unsigned count=0;
  auto test=[&](const std::string& name,const std::function<void()>& fn) {
    try { fn(); ++count; std::cout << "PASS " << name << '\n'; }
    catch(const std::exception& e) { std::cerr << "FAIL " << name << ": " << e.what() << '\n'; throw; }
  };
  try {
    test("private stock-helm component uses explicit Copter backend",[] { Host h;auto p=plugin(h);require(p&&p->name()=="helm","private component identity");auto b=makeArduCopterBackend();require(b&&b->profile()=="arducopter","wrong vehicle profile"); });
    test("startup disabled with retained scalars",[] { Host h; auto p=plugin(h); fields(*p,h,20.01);p->iterate();require(h.targets.empty(),"unsolicited target"); });
    test("post-acquisition target and immutable deadline",[] { Host h;auto p=plugin(h);active(*p,h);auto first=h.targets.back();h.fresh(20.2);p->iterate();require(h.targets.size()==2,"no renewed send");require(first.expires==h.targets.back().expires,"send renewed deadline");require(first.course==90&&first.speed==2&&first.height==10,"mapping changed");require(status(h,"coherence=ARRIVAL_GROUPED"),"coherence label missing"); });
    test("pre-acquisition fields cannot authorize target",[] { Host h;auto p=plugin(h);text(*p,h,"IVPHELM_ALLSTOP","clear",20.01);fields(*p,h,20.02);text(*p,h,"IVPHELM_STATE","DRIVE",20.025);text(*p,h,"MAVMOOS_NAV_CONTROL","acquire",20.03,"operator");p->iterate();require(h.targets.empty(),"cached fields used"); });
    test("delayed original publication is rejected",[] { Host h;auto p=plugin(h);acquire(*p,h);scalar(*p,h,"DESIRED_HEADING",90,20.04,"pHelmIvP",15);scalar(*p,h,"DESIRED_SPEED",2,20.04);scalar(*p,h,"DESIRED_ALTITUDE",10,20.04);p->iterate();require(h.targets.empty(),"old original publication accepted"); });
    test("wrong producer and future publications rejected",[] { Host h;auto p=plugin(h);acquire(*p,h);scalar(*p,h,"DESIRED_HEADING",90,20.04,"other");scalar(*p,h,"DESIRED_SPEED",2,20.04,"pHelmIvP",30);scalar(*p,h,"DESIRED_ALTITUDE",10,20.04);p->iterate();require(h.targets.empty(),"untrusted values accepted"); });
    test("retained clear does not need recurring publication",[] { Host h;auto p=plugin(h);text(*p,h,"IVPHELM_ALLSTOP","clear",20.01,"pHelmIvP",10);text(*p,h,"IVPHELM_STATE","DRIVE",20.02);text(*p,h,"MAVMOOS_NAV_CONTROL","acquire",20.03,"operator");fields(*p,h,20.04);p->iterate();require(h.targets.size()==1,"old clear inhibited fresh fields"); });
    test("duplicate publication cannot change target",[] { Host h;auto p=plugin(h);active(*p,h);scalar(*p,h,"DESIRED_HEADING",200,20.2,"pHelmIvP",20.04);p->iterate();require(h.targets.back().course==90,"duplicate mutated heading"); });
    test("fresh speed zero retains altitude target",[] { Host h;auto p=plugin(h);acquire(*p,h);fields(*p,h,20.04,90,0,10);p->iterate();require(h.targets.size()==1&&h.targets.back().speed==0&&h.targets.back().height==10,"valid zero speed rejected"); });
    test("normal handoff ignores all-stop zeros and resumes",[] { Host h;auto p=plugin(h);active(*p,h);text(*p,h,"IVPHELM_ALLSTOP","NothingToDo",20.2);fields(*p,h,20.21,0,0,0);p->iterate();require(status(h,"state=PAUSED"),"normal pause latched by zeros");text(*p,h,"IVPHELM_ALLSTOP","clear",20.3);text(*p,h,"IVPHELM_STATE","DRIVE",20.3);fields(*p,h,20.31);p->iterate();require(status(h,"state=ACTIVE")&&h.acquisitions==2&&h.targets.size()==2,"did not resume"); });
    test("normal handoff requires new fields",[] { Host h;auto p=plugin(h);active(*p,h);text(*p,h,"IVPHELM_ALLSTOP","NothingToDo",20.2);text(*p,h,"IVPHELM_ALLSTOP","clear",20.3);p->iterate();require(h.targets.size()==1&&status(h,"state=PAUSED"),"old tuple resumed"); });
    test("handoff deadline cannot be renewed by repeated messages",[] { Host h;auto p=plugin(h);active(*p,h);text(*p,h,"IVPHELM_ALLSTOP","NothingToDo",20.2);text(*p,h,"IVPHELM_ALLSTOP","NothingToDo",20.6);h.fresh(20.71);p->iterate();require(status(h,"handoff_window_expired"),"window renewed");text(*p,h,"IVPHELM_ALLSTOP","clear",20.8);fields(*p,h,20.81);p->iterate();require(h.targets.size()==1,"expired pause resumed"); });
    test("behavior error latches until explicit acquisition",[] { Host h;auto p=plugin(h);active(*p,h);text(*p,h,"IVPHELM_ALLSTOP","BehaviorError",20.2);text(*p,h,"IVPHELM_ALLSTOP","clear",20.3);fields(*p,h,20.31);p->iterate();require(status(h,"state=LATCHED")&&h.targets.size()==1,"fault auto-resumed"); });
    test("duplicate acquire cannot reopen faulted permission",[] { Host h;auto p=plugin(h);active(*p,h);text(*p,h,"IVPHELM_ALLSTOP","BehaviorError",20.2);text(*p,h,"IVPHELM_ALLSTOP","clear",20.3);text(*p,h,"MAVMOOS_NAV_CONTROL","acquire",20.31,"operator",20.03);fields(*p,h,20.32);p->iterate();require(status(h,"state=LATCHED")&&h.acquisitions==1,"replayed acquire reopened authority"); });
    test("pilot mode takeover cannot auto-recapture",[] { Host h;auto p=plugin(h);active(*p,h);h.fresh(20.2);h.state.mode=6;p->iterate();require(status(h,"external_mode_change")&&h.acquisitions==1,"pilot takeover recaptured"); });
    test("stale vehicle blocks navigation",[] { Host h;auto p=plugin(h);active(*p,h);h.time=20.6;p->iterate();require(status(h,"vehicle_observation_stale_or_invalid")&&h.targets.size()==1,"old vehicle state used"); });
    test("old scalar expiry despite fresh state",[] { Host h;auto p=plugin(h);active(*p,h);text(*p,h,"IVPHELM_STATE","DRIVE",22.1);p->iterate();require(status(h,"helm_input_stale_or_incomplete")&&h.targets.size()==1,"expired scalar target sent"); });
    test("release and recent retained acquire rejected after restart",[] { Host h;auto p=plugin(h);active(*p,h);text(*p,h,"MAVMOOS_NAV_CONTROL","release",20.2,"operator");fields(*p,h,20.3);p->iterate();require(status(h,"state=DISABLED")&&h.targets.size()==1,"release ineffective");h.fresh(21);auto restarted=plugin(h);text(*restarted,h,"MAVMOOS_NAV_CONTROL","acquire",21.1,"operator",20.9);restarted->iterate();require(status(h,"state=DISABLED"),"retained acquire reused after restart"); });
    test("disconnect reset requires new explicit acquisition",[] { Host h;auto p=plugin(h);active(*p,h);h.fresh(20.2);p->onReset("moos_disconnect");text(*p,h,"IVPHELM_ALLSTOP","clear",20.3);text(*p,h,"IVPHELM_STATE","DRIVE",20.3);fields(*p,h,20.31);p->iterate();require(status(h,"state=LATCHED")&&h.targets.size()==1,"reset auto-resumed");text(*p,h,"MAVMOOS_NAV_CONTROL","acquire",20.4,"operator",20.1);fields(*p,h,20.41);p->iterate();require(h.targets.size()==1,"pre-reset retained acquire reused");text(*p,h,"MAVMOOS_NAV_CONTROL","acquire",20.5,"operator");fields(*p,h,20.51);p->iterate();require(status(h,"state=ACTIVE")&&h.targets.size()==2,"new acquisition failed"); });
    test("nonfinite out-of-range and bad configuration rejected",[] { Host h;auto p=plugin(h);acquire(*p,h);fields(*p,h,20.04,90,6,10);p->iterate();require(status(h,"input_out_of_envelope"),"speed over limit accepted");bool threw=false;try {p->configure("freshness_timeout","nan");}catch(const std::invalid_argument&){threw=true;}require(threw,"invalid config accepted"); });
    test("new invalid scalar revokes old valid target",[] { Host h;auto p=plugin(h);active(*p,h);scalar(*p,h,"DESIRED_SPEED",std::nan(""),20.2);p->iterate();require(status(h,"invalid_helm_input")&&h.targets.size()==1,"invalid publication retained old authority"); });
    test("field publication spread must be bounded",[] { Host h;auto p=plugin(h);acquire(*p,h);scalar(*p,h,"DESIRED_HEADING",90,20.04);scalar(*p,h,"DESIRED_SPEED",2,20.6);scalar(*p,h,"DESIRED_ALTITUDE",10,20.61);p->iterate();require(h.targets.empty(),"mixed-age fields accepted"); });
    test("mode acquisition waits and expires without retry",[] { Host h;auto p=plugin(h);h.state.mode=5;acquire(*p,h);fields(*p,h,20.04);p->iterate();require(status(h,"state=ACQUIRING")&&h.targets.empty(),"LOITER treated as active");h.fresh(25.04);p->iterate();require(status(h,"acquisition_timeout")&&h.acquisitions==1,"acquisition retried or failed to expire"); });
    test("home reference change and rejected effect latch",[] { Host h;auto p=plugin(h);active(*p,h);h.fresh(20.2);h.state.home_msl=1;p->iterate();require(status(h,"home_reference_changed"),"home change ignored");Host h2;auto p2=plugin(h2);h2.allow_nav=false;acquire(*p2,h2);fields(*p2,h2,20.04);p2->iterate();require(status(h2,"navigation_rejected"),"effect denial ignored"); });
    test("vehicle reset retains allstop state but requires new permission and intent",[] {
      Host h;auto p=plugin(h);active(*p,h);h.fresh(20.2);p->onReset("vehicle_connection_lost");
      text(*p,h,"IVPHELM_STATE","DRIVE",20.3);fields(*p,h,20.31);p->iterate();
      require(h.targets.size()==1&&status(h,"state=LATCHED"),"vehicle recovery auto-resumed");
      text(*p,h,"MAVMOOS_NAV_CONTROL","acquire",20.4,"operator");p->iterate();
      require(h.targets.size()==1,"pre-acquisition intent reused");
      fields(*p,h,20.41);p->iterate();
      require(h.targets.size()==2&&status(h,"state=ACTIVE"),"unchanged allstop blocked explicit recovery");
    });
    test("MOOS reset requires source-qualified producer resynchronization",[] {
      Host h;auto p=plugin(h);active(*p,h);h.fresh(20.2);p->onReset("moos_connection_lost");
      text(*p,h,"IVPHELM_STATE","DRIVE",20.3);
      text(*p,h,"MAVMOOS_NAV_CONTROL","acquire",20.31,"operator");fields(*p,h,20.32);p->iterate();
      require(h.targets.size()==1&&status(h,"producer_resync_required=true"),"database reset retained clear");
      text(*p,h,"IVPHELM_ALLSTOP","clear",20.4,"other");fields(*p,h,20.41);p->iterate();
      require(h.targets.size()==1,"untrusted clear reopened navigation");
      text(*p,h,"IVPHELM_ALLSTOP","ManualOverride",20.5);text(*p,h,"IVPHELM_ALLSTOP","clear",20.6);
      fields(*p,h,20.61);p->iterate();
      require(h.targets.size()==2&&status(h,"producer_resync_required=false"),"producer resync did not restore eligibility");
    });
    test("reset rejects pre-boundary queued state and allstop",[] {
      Host h;auto p=plugin(h);active(*p,h);h.fresh(20.2);p->onReset("moos_connection_lost");
      text(*p,h,"IVPHELM_STATE","DRIVE",20.25,"pHelmIvP",20.1);
      text(*p,h,"IVPHELM_ALLSTOP","clear",20.26,"pHelmIvP",20.01);
      text(*p,h,"MAVMOOS_NAV_CONTROL","acquire",20.3,"operator");fields(*p,h,20.31);p->iterate();
      require(h.targets.size()==1&&status(h,"producer_resync_required=true"),"pre-reset allstop restored state");
      text(*p,h,"IVPHELM_ALLSTOP","ManualOverride",20.4);text(*p,h,"IVPHELM_ALLSTOP","clear",20.5);
      fields(*p,h,20.51);p->iterate();require(h.targets.size()==1,"pre-reset DRIVE satisfied producer readiness");
      text(*p,h,"IVPHELM_STATE","DRIVE",20.6);fields(*p,h,20.61);p->iterate();
      require(h.targets.size()==2,"fresh post-reset state failed to restore eligibility");
    });
    test("startup diagnostics separate unknown producer and intentional inactive cache",[] {
      Host h;auto p=plugin(h);p->iterate();
      require(status(h,"readiness=INACTIVE")&&status(h,"next_action=WAIT_HELM_STATE"),"startup advice misleading");
      require(status(h,"heading_age_s=-1")&&status(h,"drive_age_s=-1"),"unknown age invented");
      require(!status(h,"heading_missing")&&status(h,"explicit_acquire_required"),"inactive cache described as waitable input");
      require(h.acquisitions==0&&h.targets.empty()&&h.revocations.empty(),"diagnostics created a control effect");
      sample(h,"startup");
    });
    test("Loiter diagnostics allow acquisition preparation but never NAV readiness",[] {
      Host h;auto p=plugin(h);h.state.mode=5;
      text(*p,h,"IVPHELM_ALLSTOP","clear",20.01);text(*p,h,"IVPHELM_STATE","DRIVE",20.02);p->iterate();
      require(status(h,"acquire_preparation=READY")&&status(h,"readiness=INACTIVE"),"Loiter conflated acquisition with NAV");
      require(status(h,"next_action=EXPLICIT_ACQUIRE_WITH_NEW_INPUTS")&&status(h,"host_gates=INDEPENDENT"),"Loiter acquisition advice missing");
      require(!status(h,"heading_missing")&&h.acquisitions==0&&h.targets.empty(),"readiness changed control");
      sample(h,"inactive_loiter");
      text(*p,h,"MAVMOOS_NAV_CONTROL","acquire",20.03,"operator");p->iterate();
      require(status(h,"next_action=WAIT_GUIDED_CONFIRMATION")&&status(h,"heading_missing"),"pending acquisition advice wrong");
      sample(h,"acquiring_loiter");
    });
    test("active diagnostics show original input lifetime without renewal",[] {
      Host h;auto p=plugin(h);active(*p,h);h.fresh(20.2);p->iterate();
      require(status(h,"readiness=READY")&&statusField(h,"blocked_by","none")&&status(h,"next_action=NONE"),"eligible target not ready");
      require(status(h,"heading_age_s=0.16")&&status(h,"input_remaining_s=1.84"),"original input ages not reported");
      const auto first=h.targets.back().expires;h.fresh(20.3);p->iterate();
      require(status(h,"input_remaining_s=1.74")&&h.targets.back().expires==first,"diagnostic refreshed target lifetime");
      sample(h,"active");
    });
    test("database recovery advice survives acquire and retained old clear",[] {
      Host h;auto p=plugin(h);active(*p,h);h.fresh(20.2);p->onReset("moos_connection_lost");
      text(*p,h,"IVPHELM_STATE","DRIVE",20.21);p->iterate();
      require(status(h,"next_action=RESYNCHRONIZE_HELM")&&status(h,"allstop_unknown"),"database reset advice missing");
      sample(h,"moos_reset");
      text(*p,h,"MAVMOOS_NAV_CONTROL","acquire",20.3,"operator");
      text(*p,h,"IVPHELM_ALLSTOP","clear",20.31,"pHelmIvP",20.1);p->iterate();
      require(status(h,"next_action=RESYNCHRONIZE_HELM")&&h.targets.size()==1,"old clear concealed resync need");
      text(*p,h,"IVPHELM_ALLSTOP","ManualOverride",20.4);p->iterate();
      require(status(h,"next_action=REVIEW_HELM_ALLSTOP"),"manual override presented as clear");
      text(*p,h,"IVPHELM_ALLSTOP","clear",20.5);p->iterate();
      require(status(h,"next_action=WAIT_FRESH_INPUTS")&&status(h,"heading_missing"),"clear did not wait for new scalars");
    });
    test("vehicle recovery distinguishes new acquisition from bounded normal handoff",[] {
      Host h;auto p=plugin(h);active(*p,h);h.fresh(20.2);p->onReset("vehicle_connection_lost");
      text(*p,h,"IVPHELM_STATE","DRIVE",20.21);p->iterate();
      require(status(h,"producer_resync_required=false")&&status(h,"next_action=EXPLICIT_ACQUIRE_WITH_NEW_INPUTS"),"vehicle loss needlessly requires producer resync");
      sample(h,"vehicle_reset_recovered_observations");
      text(*p,h,"MAVMOOS_NAV_CONTROL","acquire",20.3,"operator");fields(*p,h,20.31);p->iterate();
      text(*p,h,"IVPHELM_ALLSTOP","NothingToDo",20.4);h.fresh(20.6);p->iterate();
      require(status(h,"next_action=WAIT_BOUNDED_HANDOFF")&&status(h,"handoff_remaining_s=0.3")&&status(h,"acquire_required=false"),"normal handoff advice wrong");
      require(h.acquisitions==2&&h.targets.size()==2,"diagnostics acquired or navigated during pause");
      sample(h,"normal_handoff");
    });
    test("pilot and stale vehicle diagnostics identify different operator actions",[] {
      Host h;auto p=plugin(h);active(*p,h);h.fresh(20.2);h.state.mode=6;p->iterate();
      require(status(h,"next_action=OPERATOR_SELECT_GUIDED_OR_LOITER")&&status(h,"acquire_preparation=BLOCKED"),"pilot takeover advised recapture");
      require(h.acquisitions==1&&h.targets.size()==1,"pilot mode recaptured by diagnostics");
      sample(h,"pilot_rtl");
      h.time=23;p->iterate();
      require(status(h,"next_action=WAIT_VEHICLE_READY")&&status(h,"heartbeat_stale_or_future"),"stale vehicle obscured");
      sample(h,"vehicle_stale");
    });
    test("input boundary latches old active pose before fresh ingress can conceal it",[] {
      Host h;auto p=plugin(h);active(*p,h);const auto old_status=h.strings;
      h.time=20.7;p->onInputBoundary();
      require(h.strings==old_status&&h.targets.size()==1&&h.acquisitions==1,"boundary published or sent navigation");
      require(h.revocations.back()=="vehicle_observation_stale_or_invalid","boundary missed old pose expiry");
      text(*p,h,"IVPHELM_STATE","DRIVE",20.71);fields(*p,h,20.72);p->iterate();
      require(status(h,"state=LATCHED")&&h.targets.size()==1&&h.acquisitions==1,"fresh ingress concealed expired active pose");
    });
    test("input boundary latches old active helm fields before replacements",[] {
      Host h;auto p=plugin(h);active(*p,h);h.fresh(22.1);p->onInputBoundary();
      require(h.revocations.back()=="helm_input_stale_or_incomplete","boundary missed old helm expiry");
      text(*p,h,"IVPHELM_STATE","DRIVE",22.11);fields(*p,h,22.12);p->iterate();
      require(status(h,"state=LATCHED")&&h.targets.size()==1,"new fields renewed expired permission");
    });
    test("input boundary prevents handoff resume across stale paused vehicle evidence",[] {
      Host h;auto p=plugin(h);p->configure("nav_position_timeout","0.1");active(*p,h);
      text(*p,h,"IVPHELM_ALLSTOP","NothingToDo",20.05);h.time=20.2;p->onInputBoundary();
      require(h.revocations.back()=="vehicle_observation_stale_or_invalid","paused pose expiry ignored");
      text(*p,h,"IVPHELM_ALLSTOP","clear",20.21);text(*p,h,"IVPHELM_STATE","DRIVE",20.21);
      fields(*p,h,20.22);p->iterate();
      require(status(h,"state=LATCHED")&&h.targets.size()==1&&h.acquisitions==1,"paused handoff recaptured after owner gap");
    });
    test("input boundary expires paused window but never performs normal resume",[] {
      Host h;auto p=plugin(h);active(*p,h);text(*p,h,"IVPHELM_ALLSTOP","NothingToDo",20.2);
      text(*p,h,"IVPHELM_ALLSTOP","clear",20.3);text(*p,h,"IVPHELM_STATE","DRIVE",20.3);fields(*p,h,20.31);
      const auto old_status=h.strings;p->onInputBoundary();
      require(h.acquisitions==1&&h.targets.size()==1&&h.strings==old_status,"boundary resumed or published");
      h.fresh(20.71);p->onInputBoundary();p->iterate();
      require(status(h,"handoff_window_expired")&&h.acquisitions==1&&h.targets.size()==1,"fresh post-gap input revived expired handoff");
    });
    std::cout << "PASS " << count << " helm module cases\n";
  } catch(...) { return 1; }
  return 0;
}

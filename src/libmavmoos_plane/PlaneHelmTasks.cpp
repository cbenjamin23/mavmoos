#include "ControlReplayService.h"
#include "PlaneHelmTasks.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime::plane {
namespace {
constexpr unsigned loiter=12,guided=15;
using Fields=std::map<std::string,std::string>;

bool token(const std::string& text)
{
  return !text.empty()&&text.size()<=48&&std::all_of(text.begin(),text.end(),[](unsigned char ch) {
    return (ch>='a'&&ch<='z')||(ch>='A'&&ch<='Z')||(ch>='0'&&ch<='9')||ch=='_'||ch=='-'||ch=='.';
  });
}
std::optional<Fields> fields(const Mail& mail)
{
  if(mail.numeric||mail.text.empty()||mail.text.size()>768||mail.text.back()==',')return {};
  Fields values;
  std::istringstream input(mail.text);
  for(std::string item;std::getline(input,item,',');) {
    const auto at=item.find('=');
    if(at==0||at==std::string::npos||at+1==item.size()||
       !values.emplace(item.substr(0,at),item.substr(at+1)).second)return {};
  }
  return values;
}
bool boolean(const std::string& key,const std::string& value)
{
  if(value!="true"&&value!="false")throw std::invalid_argument(key+" must be true or false");
  return value=="true";
}
bool knownTask(const std::string& task)
{return task=="return"||task=="survey"||task=="voronoi"||task=="waypoint";}
} // namespace

HelmTasks::HelmTasks(Services& host,const VehicleControlBackend& backend,std::set<std::string>& history)
  :m_host(host),m_backend(backend),m_history(history),m_floor(host.now()),m_last_now(host.now())
{
  if(backend.profile()!="arduplane")throw std::invalid_argument("Plane helm tasks require the conventional Plane backend");
}

bool HelmTasks::enabled() const
{return m_configured_enabled&&m_helm_enabled&&!m_shadow&&!m_clock_fault;}

void HelmTasks::configure(const std::string& key,const std::string& value)
{
  if(key=="operation_helm_enabled")m_configured_enabled=boolean(key,value);
  else if(key=="control_helm")m_helm_enabled=boolean(key,value);
  else if(key=="shadow_mode")m_shadow=boolean(key,value);
  else if(key=="helm_source")m_helm_source=value;
  else if(key=="operation_helm_tasks") {
    std::set<std::string> tasks;
    std::istringstream input(value);
    for(std::string task;std::getline(input,task,',');) {
      const auto first=task.find_first_not_of(" \t"),last=task.find_last_not_of(" \t");
      if(first==std::string::npos)throw std::invalid_argument("empty Plane helm task");
      task=task.substr(first,last-first+1);
      if(!knownTask(task)||!tasks.insert(task).second)throw std::invalid_argument("unknown or duplicate Plane helm task");
    }
    if(tasks.empty()||value.back()==',')throw std::invalid_argument("operation_helm_tasks requires a nonempty task list");
    m_configured=std::move(tasks);
  }
}

std::vector<Capability> HelmTasks::capabilities() const
{
  std::vector<Capability> result;
  if(!enabled())return result;
  for(const auto& task:m_configured)
    result.push_back({"control.operations.helm."+task,"MAVMOOS_CONTROL_REQUEST","schema=1,action="+task+",strategy=helm"});
  if(!m_configured.empty())result.push_back({"control.operations.helm.cancel","MAVMOOS_CONTROL_REQUEST","schema=1,action=cancel,target=<task>"});
  return result;
}

bool HelmTasks::freshMail(const Mail& mail) const
{
  const auto current=now();
  return !m_clock_fault&&std::isfinite(current)&&std::isfinite(mail.published_at)&&std::isfinite(mail.received_at)&&
    mail.published_at>m_floor&&mail.published_at<=mail.received_at+.1&&mail.received_at<=current+.1&&
    current>=mail.published_at&&current-mail.published_at<=2&&current-mail.received_at<=2;
}

bool HelmTasks::onRequest(const Mail& mail,Plugin& helm,bool other_work)
{
  if(mail.key!="MAVMOOS_CONTROL_REQUEST")return false;
  const auto values=fields(mail);
  if(!values)return false;
  const auto get=[&](const char* key) {const auto it=values->find(key);return it==values->end()?std::string{}:it->second;};
  Task task;
  task.id=get("id");task.action=get("action");
  const auto strategy=get("strategy"),target=get("target");
  const bool cancel=task.action=="cancel"&&(strategy=="helm"||
    (strategy.empty()&&m_task&&m_task->id==target));
  if(strategy!="helm"&&!cancel)return false;
  beforeHelm(helm);
  if(!enabled()||get("schema")!="1"||!token(task.id)||!freshMail(mail)||
     m_history.count(task.id)||m_history.size()>=128) {
    report(task,"REJECTED","disabled_stale_or_duplicate_task");return true;
  }
  if(cancel) {
    const std::size_t count=strategy.empty()?4:5;
    if(values->size()!=count||!m_task||target!=m_task->id) {
      report(task,"REJECTED","unknown_task_target");return true;
    }
    m_history.insert(task.id);finish("CANCELLED","explicit_task_cancel",&helm);
    report(task,"COMPLETED","task_cancelled_helm_unacquired");return true;
  }
  const std::size_t count=values->count("timeout")?5:4;
  double timeout=90;
  if(values->count("timeout")) {
    try {
      std::size_t used=0;
      timeout=std::stod(get("timeout"),&used);
      if(used!=get("timeout").size())timeout=0;
    } catch(const std::exception&) {timeout=0;}
  }
  task.expires=mail.published_at+timeout;
  if(values->size()!=count||!knownTask(task.action)||!m_configured.count(task.action)||
     !std::isfinite(timeout)||timeout<5||timeout>120||now()>=task.expires) {
    report(task,"REJECTED","unsupported_task_or_original_deadline");return true;
  }
  const auto& state=vehicle();
  if(other_work||m_task||m_host.controlBusy()||m_host.navigationGranted()||state.mode!=loiter||
     !m_backend.acquisitionAllowed(state,now())) {
    report(task,"REJECTED","idle_fresh_native_loiter_required");return true;
  }
  double x=0,y=0;
  if(task.action=="return"&&(!m_host.projectGlobal(state.home_latitude,state.home_longitude,x,y)||
     !std::isfinite(x)||!std::isfinite(y))) {
    report(task,"REJECTED","projected_home_required");return true;
  }
  const auto replay=reserveControlRequest(m_host,task.id);
  if(replay!=ReplayResult::Reserved) {report(task,replayRejection(replay),replayReason(replay));return true;}
  releaseHelm(helm);
  task.selected_at=now();task.home=vehicle();
  m_history.insert(task.id);m_task=task;m_consent_at=-1;m_initialized=true;
  if(task.action=="return") {
    std::ostringstream update;update<<std::setprecision(12)<<"points="<<x<<','<<y;
    m_host.publish("MAVMOOS_OPERATION_RETURN_UPDATE",update.str());
  }
  m_host.publish("MAVMOOS_HELM_OPERATION",task.action);
  report(task,"SELECTED","configured_behavior_requires_separate_explicit_acquire");
  return true;
}

bool HelmTasks::onNavControl(const Mail& mail,Plugin& helm)
{
  if(mail.key!="MAVMOOS_NAV_CONTROL"||!m_task)return false;
  beforeHelm(helm);
  if(!m_task)return true;
  if(mail.numeric||!freshMail(mail)||mail.published_at<=m_task->selected_at)return true;
  if(mail.text=="release") {
    finish("CANCELLED","explicit_navigation_release",&helm);return true;
  }
  if(mail.text!="acquire")return true;
  if(!readyToAcquire()) {
    report(*m_task,"ACQUIRE_REJECTED","fresh_selected_behavior_required");return true;
  }
  if(m_task->acquired||!acquisitionFitsDeadline()||vehicle().mode!=loiter||
     m_host.controlBusy()||m_host.navigationGranted()||!sameHome()||!m_backend.acquisitionAllowed(vehicle(),now())) {
    report(*m_task,"ACQUIRE_REJECTED","fresh_native_loiter_and_full_acquisition_window_required");return true;
  }
  m_consent_at=now();
  helm.onMail(mail);
  if(m_host.navigationGranted()) {
    m_task->acquired=true;
    report(*m_task,"ACQUIRING","explicit_helm_permission_pending_new_inputs");
  } else report(*m_task,"ACQUIRE_REJECTED","host_acquisition_inhibited");
  m_consent_at=-1;
  return true;
}

bool HelmTasks::onReadiness(const Mail& mail)
{
  if(mail.key!="MAVMOOS_HELM_TASK_READY")return false;
  if(m_task&&!mail.numeric&&mail.source==m_helm_source&&mail.text==m_task->action&&freshMail(mail)&&
     mail.published_at>m_task->selected_at&&mail.published_at>m_task->ready_at)
    m_task->ready_at=mail.published_at;
  return true;
}

bool HelmTasks::readyToAcquire() const
{ return m_task&&m_task->ready_at>m_task->selected_at&&now()-m_task->ready_at<=2; }

bool HelmTasks::sameHome() const
{
  if(!m_task)return true;
  const auto& state=vehicle();const auto& home=m_task->home;
  return state.home_valid&&state.home_latitude==home.home_latitude&&state.home_longitude==home.home_longitude&&
    std::abs(state.home_msl-home.home_msl)<=.05;
}

bool HelmTasks::acquisitionFitsDeadline() const
{
  // Inspect the backend's pure setup description so changes to its bounded
  // mode-entry lifetime also apply here. This does not submit a command.
  const auto setup=m_backend.acquisitionCommand(vehicle(),"task_deadline_check",now());
  return m_task&&setup&&std::isfinite(setup->expires)&&setup->expires<=m_task->expires;
}

void HelmTasks::beforeHelm(Plugin& helm)
{
  const auto current=now();
  if(!std::isfinite(current)||current<m_last_now) {
    m_clock_fault=true;
    finish("FAILED","task_clock_fault",&helm);return;
  }
  m_last_now=current;
  if(enabled()&&!m_initialized) {m_host.publish("MAVMOOS_HELM_OPERATION","none");m_initialized=true;}
  if(!m_task)return;
  if(current>=m_task->expires) {finish("TIMED_OUT","original_task_deadline",&helm);return;}
  if(!sameHome()||!m_backend.flightReady(vehicle(),current,false)||m_host.controlBusy()) {
    finish("CANCELLED","task_observation_or_host_authority_lost",&helm);return;
  }
  const auto mode=vehicle().mode;
  if((!m_task->acquired&&mode!=loiter)||(m_task->acquired&&mode!=loiter&&mode!=guided)||
     (m_task->confirmed&&mode!=guided))finish("CANCELLED","pilot_or_native_mode_takeover",&helm);
}

void HelmTasks::afterHelm(Plugin& helm)
{
  if(!m_configured_enabled)return;
  beforeHelm(helm);
  if(m_task&&m_task->acquired) {
    if(m_helm_phase=="LATCHED"||m_helm_phase=="DISABLED"||
       (m_helm_phase=="ACTIVE"&&!m_host.navigationGranted()))finish("CANCELLED","helm_authority_lost",&helm);
    else if(m_task&&!m_task->confirmed&&m_helm_phase=="ACTIVE"&&vehicle().mode==guided&&m_host.navigationGranted()) {
      m_task->confirmed=true;report(*m_task,"ACTIVE","fresh_stock_helm_guided_control_observed");
    }
  }
  std::ostringstream status;
  status<<"schema=1,selected="<<(m_task?"true":"false")<<",id="<<(m_task?m_task->id:"none")
        <<",action="<<(m_task?m_task->action:"none")<<",helm_phase="<<m_helm_phase
        <<",acquire_ready="<<(readyToAcquire()?"true":"false")
        <<",expires="<<std::setprecision(16)<<(m_task?m_task->expires:0)<<",at="<<now();
  m_host.publish("MAVMOOS_PLANE_HELM_TASK_STATUS",status.str());
}

void HelmTasks::onReset(const std::string& reason)
{
  finish("CANCELLED",reason.c_str(),nullptr);
  m_floor=std::max(m_floor,now());m_last_now=now();m_consent_at=-1;m_helm_phase="LATCHED";
  m_initialized=false;
}

void HelmTasks::releaseHelm(Plugin& helm)
{
  Mail release;release.key="MAVMOOS_NAV_CONTROL";release.text="release";release.source="plane_helm_tasks";
  release.published_at=release.received_at=now();
  helm.onMail(release);
  m_host.revokeNavigation("operator_release");
}

void HelmTasks::finish(const char* state,const char* reason,Plugin* helm)
{
  if(!m_task)return;
  const auto task=*m_task;m_task.reset();m_consent_at=-1;
  if(helm)releaseHelm(*helm);
  else m_host.revokeNavigation("operator_release");
  m_host.publish("MAVMOOS_HELM_OPERATION","none");report(task,state,reason);
}

void HelmTasks::report(const Task& task,const char* state,const char* reason)
{
  std::ostringstream out;
  out<<"schema=1,profile=arduplane,id="<<task.id<<",action="<<task.action<<",strategy=helm,status="<<state
     <<",reason="<<reason<<",expires="<<std::setprecision(16)<<task.expires<<",at="<<now();
  m_host.publish("MAVMOOS_CONTROL_RESULT",out.str());
}

std::optional<double> HelmTasks::deadline() const
{if(m_task)return m_task->expires;return {};}
double HelmTasks::bound(double expires) const
{if(m_task)return std::min(expires,m_task->expires);return expires;}
double HelmTasks::now() const {return m_host.now();}
const Vehicle& HelmTasks::vehicle() const {return m_host.vehicle();}
void HelmTasks::publish(const std::string& key,const std::string& value)
{
  if(key=="MAVMOOS_HELM_STATUS"&&value.rfind("state=",0)==0) {
    const auto end=value.find(',');m_helm_phase=value.substr(6,end==std::string::npos?end:end-6);
  }
  m_host.publish(key,value);
}
void HelmTasks::publish(const std::string& key,double value) {m_host.publish(key,value);}
bool HelmTasks::sendCommand(const CommandRequest& request) {return m_host.sendCommand(request);}
bool HelmTasks::acquireNavigation(const std::string& id)
{
  if(m_task&&(m_consent_at!=now()||!acquisitionFitsDeadline()))return false;
  return m_host.acquireNavigation(id);
}
void HelmTasks::revokeNavigation(const std::string& reason) {m_host.revokeNavigation(reason);}
bool HelmTasks::navigation(const NavigationTarget& target)
{
  auto bounded=target;bounded.expires=bound(target.expires);
  return bounded.expires>now()&&m_host.navigation(bounded);
}
bool HelmTasks::navigationGranted() const {return m_host.navigationGranted();}
bool HelmTasks::controlBusy() const {return m_host.controlBusy();}
bool HelmTasks::projectGlobal(double lat,double lon,double& x,double& y) const {return m_host.projectGlobal(lat,lon,x,y);}
bool HelmTasks::suspendNavigation(double until) {return bound(until)>now()&&m_host.suspendNavigation(bound(until));}
bool HelmTasks::resumeNavigation(const std::string& id) {return bound(now()+1)>now()&&m_host.resumeNavigation(id);}
void HelmTasks::commandFinished(const std::string& id) {m_host.commandFinished(id);}
void HelmTasks::cancelCommand(const std::string& id) {m_host.cancelCommand(id);}
} // namespace mavmoos::runtime::plane

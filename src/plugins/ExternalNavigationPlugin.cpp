// One timestamped ExternalNav producer for the owned disarmed Copter bench.
#include "ExternalNavigationService.h"
#include "Plugin.h"

#include <charconv>
#include <map>
#include <sstream>
#include <string_view>

namespace mavmoos::runtime {
namespace {
constexpr char request_key[]="MAVMOOS_EXTERNAL_NAV_REQUEST";
constexpr char input_key[]="MAVMOOS_EXTERNAL_NAV_INPUT";
constexpr char status_key[]="MAVMOOS_EXTERNAL_NAV_STATUS";

bool validSource(std::string_view value)
{
  return !value.empty() && value.size()<=48 &&
    std::all_of(value.begin(),value.end(),[](unsigned char c) {
      return (c>='A' && c<='Z') || (c>='a' && c<='z') ||
             (c>='0' && c<='9') || c=='_' || c=='-' || c=='.';
    });
}

using Fields=std::map<std::string_view,std::string_view>;

Fields parseFields(std::string_view input)
{
  Fields fields;
  if(input.empty() || input.size()>1024 || input.back()==',') return {};
  while(!input.empty()) {
    const size_t comma=input.find(',');
    const auto field=input.substr(0,comma);
    const size_t equals=field.find('=');
    if(equals==0 || equals==std::string_view::npos || equals+1==field.size() ||
       !fields.emplace(field.substr(0,equals),field.substr(equals+1)).second) return {};
    if(comma==std::string_view::npos) break;
    input.remove_prefix(comma+1);
  }
  return fields;
}

std::string_view field(const Fields& fields,std::string_view key)
{
  const auto it=fields.find(key);
  return it==fields.end()?std::string_view{}:it->second;
}

template<typename Number>
bool number(std::string_view text,Number& value)
{
  if(text.empty()) return false;
  const auto* end=text.data()+text.size();
  const auto result=std::from_chars(text.data(),end,value);
  return result.ec==std::errc{} && result.ptr==end;
}

template<size_t N>
bool vector(std::string_view text,std::array<float,N>& values)
{
  for(size_t i=0;i<N;++i) {
    const size_t slash=text.find('/');
    if((i+1==N)!=(slash==std::string_view::npos) ||
       !number(text.substr(0,slash),values[i])) return false;
    if(slash!=std::string_view::npos) text.remove_prefix(slash+1);
  }
  return true;
}

bool parseSample(const Fields& fields,double posted,ExternalNavigationSample& sample)
{
  unsigned quality=0;
  if(fields.size()!=14 || field(fields,"schema")!="1" ||
     !number(field(fields,"run"),sample.run_id) || !number(field(fields,"seq"),sample.sequence) ||
     !number(field(fields,"capture_us"),sample.capture_us) || !number(field(fields,"quality"),quality) ||
     quality<50 || quality>100 ||
     field(fields,"frame")!="local_frd_body_frd" ||
     !vector(field(fields,"position"),sample.position) || !vector(field(fields,"velocity"),sample.velocity) ||
     !vector(field(fields,"rates"),sample.angular_rate) || !vector(field(fields,"quaternion"),sample.quaternion) ||
     !vector(field(fields,"position_var"),sample.pose_variance) ||
     !vector(field(fields,"angle_var"),sample.angle_variance) ||
     !vector(field(fields,"velocity_var"),sample.velocity_variance) ||
     !vector(field(fields,"rate_var"),sample.rate_variance)) return false;
  sample.quality=static_cast<uint8_t>(quality);
  sample.capture_at=posted;
  sample.expires=std::min(posted,externalNavigationCaptureTime(sample.capture_us))+external_navigation_age_s;
  return true;
}

class ExternalNavigation final : public Plugin, public ExternalNavigationListener {
public:
  explicit ExternalNavigation(Services& services)
    : m_services(services),m_sender(dynamic_cast<ExternalNavigationService*>(&services)) {}

  std::string name() const override { return "external_navigation"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile="arducopter_sitl_external_nav";
    if(!m_source.empty()) {
      result.capabilities={{external_navigation_capability,request_key,"schema=1,action=begin|stop,run=<id>"}};
      result.exclusive_resources={"estimator.external_navigation"};
      result.request_owners={request_key,input_key};
    }
    return result;
  }

  std::vector<std::string> subscriptions() const override
  {
    if(m_source.empty()) return {};
    return {request_key,input_key};
  }

  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override
  {
    if(settings.empty()) return {};
    for(const auto& item:settings)
      if(item.key!="source") return {ModuleConfigStatus::UnknownKey,item.key,"use source"};
    if(settings.size()!=1 || !validSource(settings.front().value))
      return {ModuleConfigStatus::Invalid,"source","one local producer name required"};
    if(!m_sender) return {ModuleConfigStatus::Invalid,"source","host lacks the typed ExternalNav service"};
    m_source=settings.front().value;
    return {};
  }

  void onMail(const Mail& mail) override
  {
    if(m_source.empty() || (mail.key!=request_key && mail.key!=input_key)) return;
    const double now=m_services.now();
    advance(now);
    if(mail.source!=m_source) { reject("wrong_source"); return; }
    if(mail.numeric || !std::isfinite(mail.published_at) || !std::isfinite(mail.received_at) ||
       mail.received_at<=0 || mail.published_at<=0 ||
       mail.received_at>now+external_navigation_clock_slop_s ||
       mail.published_at>mail.received_at+external_navigation_clock_slop_s ||
       now-mail.published_at>=external_navigation_age_s) { reject("invalid_publication"); return; }
    const auto fields=parseFields(mail.text);
    if(mail.key==request_key) {
      handleRequest(fields,mail.published_at,now);
      return;
    }
    handleSample(fields,mail.published_at,now);
  }

  void onInputBoundary() override { advance(m_services.now()); }

  void iterate() override
  {
    if(m_source.empty()) return;
    const double now=m_services.now();
    advance(now);
    if(std::isfinite(now) && now>=m_next_status) {
      publish();
      m_next_status=now+.2;
    }
  }

  void onReset(const std::string&) override
  {
    if(m_run) stop("connection_reset");
  }

  void onExternalNavigationSent(const ExternalNavigationSample& sample,double,bool accepted) override
  {
    if(!m_run || sample.run_id!=m_run || sample.sequence<=m_last_settled ||
       sample.sequence>m_last_sequence) return;
    m_last_settled=sample.sequence;
    if(accepted) ++m_sent;
    else ++m_unsent;
  }

private:
  void advance(double now)
  {
    if(!m_run) return;
    if(!std::isfinite(now) || now<m_started || !m_sender->externalNavigationReady()) stop("bench_unavailable");
    else if(now>=m_run_expires) stop("lease_expired");
    else if(now-m_last_measurement>=external_navigation_silence_s) stop("source_dropout");
  }

  void handleRequest(const Fields& fields,double posted,double now)
  {
    uint64_t id=0;
    if(fields.size()!=3 || field(fields,"schema")!="1" || !number(field(fields,"run"),id) || !id) {
      reject("invalid_request");
      return;
    }
    const auto action=field(fields,"action");
    if(action=="stop" && id==m_run && posted>=m_started) {
      stop("explicit_stop");
      return;
    }
    if(action!="begin" || m_run || id<=m_last_run) { reject("run_unavailable"); return; }
    const double expires=std::min(posted,now)+external_navigation_lease_s;
    if(!m_sender->externalNavigationReady() || !m_sender->beginExternalNavigation(id,expires)) {
      reject("bench_unavailable");
      return;
    }
    m_run=m_last_run=id;
    m_started=m_last_measurement=now;
    m_run_expires=expires;
    m_last_capture_us=m_last_sequence=m_last_settled=0;
    m_reason="begun";
    publish();
  }

  void handleSample(const Fields& fields,double posted,double now)
  {
    ExternalNavigationSample sample;
    if(!parseSample(fields,posted,sample) || !validExternalNavigationSample(sample,now)) {
      reject("invalid_or_stale_sample");
      return;
    }
    if(!m_run || sample.run_id!=m_run || sample.sequence<=m_last_sequence ||
       sample.capture_us<=m_last_capture_us || externalNavigationCaptureTime(sample.capture_us)<=m_started ||
       posted<m_started) { reject("stale_or_inactive_run"); return; }
    sample.expires=std::min(sample.expires,m_run_expires);
    if(!m_sender->externalNavigation(sample)) { reject("host_denied_sample"); return; }
    m_last_sequence=sample.sequence;
    m_last_capture_us=sample.capture_us;
    m_last_measurement=std::min(posted,externalNavigationCaptureTime(sample.capture_us));
    ++m_queued;
    m_reason="fresh_input";
  }

  void stop(const char* reason)
  {
    m_sender->stopExternalNavigation(m_run);
    m_run=0;
    m_reason=reason;
    publish();
  }

  void reject(const char* reason)
  {
    ++m_rejected;
    m_rejection=reason;
  }

  void publish()
  {
    std::ostringstream out;
    out << "schema=1,state=" << (m_run?"STREAMING":m_last_run?"STOPPED":"IDLE")
        << ",reason=" << m_reason << ",run=" << m_run << ",last_run=" << m_last_run
        << ",last_sequence=" << m_last_sequence << ",queued=" << m_queued << ",sent=" << m_sent
        << ",unsent=" << m_unsent << ",rejected=" << m_rejected << ",last_rejection=" << m_rejection;
    m_services.publish(status_key,out.str());
  }

  Services& m_services;
  ExternalNavigationService* const m_sender;
  std::string m_source, m_reason="awaiting_begin", m_rejection="none";
  double m_next_status=0, m_started=0, m_last_measurement=0, m_run_expires=0;
  uint64_t m_run=0,m_last_run=0,m_last_capture_us=0,m_last_sequence=0,m_last_settled=0;
  uint64_t m_queued=0,m_sent=0,m_unsent=0,m_rejected=0;
};
}

extern "C" Plugin* mavmoos_create_plugin(unsigned abi,Services* services)
{
  return abi==plugin_abi && services?new ExternalNavigation(*services):nullptr;
}

extern "C" void mavmoos_destroy_plugin(Plugin* plugin) { delete plugin; }

} // namespace mavmoos::runtime

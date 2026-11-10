#pragma once

#include "ControlBackend.h"
#include <optional>
#include <set>

namespace mavmoos::runtime::plane {

// A private service view for the ordinary helm adapter. Direct and native
// operations continue to use the provider's original host services.
class HelmTasks final : private Services {
public:
  HelmTasks(Services&,const VehicleControlBackend&,std::set<std::string>& history);
  Services& helmServices() {return *this;}
  void configure(const std::string& key,const std::string& value);
  std::vector<Capability> capabilities() const;
  bool onRequest(const Mail&,Plugin& helm,bool other_work);
  bool onReadiness(const Mail&);
  bool onNavControl(const Mail&,Plugin& helm);
  void beforeHelm(Plugin& helm);
  void afterHelm(Plugin& helm);
  void onReset(const std::string& reason);
  bool busy() const {return m_task.has_value();}
  std::optional<double> deadline() const;

private:
  struct Task {
    std::string id,action;
    double selected_at=0,expires=0,ready_at=0;
    Vehicle home;
    bool acquired=false,confirmed=false;
  };
  bool enabled() const;
  bool freshMail(const Mail&) const;
  bool sameHome() const;
  bool readyToAcquire() const;
  bool acquisitionFitsDeadline() const;
  void releaseHelm(Plugin&);
  void finish(const char* status,const char* reason,Plugin* helm);
  void report(const Task&,const char* status,const char* reason);
  double bound(double expires) const;

  double now() const override;
  const Vehicle& vehicle() const override;
  void publish(const std::string&,const std::string&) override;
  void publish(const std::string&,double) override;
  bool sendCommand(const CommandRequest&) override;
  bool acquireNavigation(const std::string&) override;
  void revokeNavigation(const std::string&) override;
  bool navigation(const NavigationTarget&) override;
  bool navigationGranted() const override;
  bool controlBusy() const override;
  bool projectGlobal(double,double,double&,double&) const override;
  bool suspendNavigation(double until) override;
  bool resumeNavigation(const std::string&) override;
  void commandFinished(const std::string&) override;
  void cancelCommand(const std::string&) override;

  Services& m_host;
  const VehicleControlBackend& m_backend;
  std::set<std::string>& m_history;
  std::set<std::string> m_configured;
  std::optional<Task> m_task;
  std::string m_helm_phase="DISABLED";
  std::string m_helm_source="pHelmIvP";
  double m_floor=0,m_last_now=0,m_consent_at=-1;
  bool m_configured_enabled=false,m_helm_enabled=true,m_shadow=false,m_clock_fault=false,m_initialized=false;
};
} // namespace mavmoos::runtime::plane

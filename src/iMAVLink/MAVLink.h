/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: MAVLink.h                                       */
/*    DATE: October 29, 2026                                */
/************************************************************/
#ifndef MAVLink_HEADER
#define MAVLink_HEADER
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "Config.h"
#include "Telemetry.h"
#include "UdpReceiver.h"
#include "Runtime.h"
#include "ModuleSelection.h"
#include <memory>
#include <atomic>

class MAVLink : public AppCastingMOOSApp {
public:
  MAVLink() = default;
  ~MAVLink() override = default;
protected:
  bool OnNewMail(MOOSMSG_LIST& mail) override;
  bool Iterate() override;
  bool OnConnectToServer() override;
  bool OnDisconnectFromServer() override;
  bool OnStartUp() override;
  bool buildReport() override;
  void registerVariables();
  void applyConnectionEvents();
private:
  bool handleConfig(const std::string& param, const std::string& value);
  void postObservations(double now);
  std::string status(double now) const;
  mavmoos::RxConfig m_config;
  mavmoos::UdpReceiver m_receiver;
  std::unique_ptr<mavmoos::Telemetry> m_telemetry;
  std::unique_ptr<mavmoos::runtime::Runtime> m_runtime;
  bool m_simulation=false,m_shadow=false,m_helm_enabled=false;
  bool m_helm_explicit=false,m_expected_type_explicit=false,m_expected_autopilot_explicit=false;
  bool m_firmware_requests=true;
  std::optional<bool> m_control_native_override,m_control_helm_override;
  int m_sitl_pid=0;
  std::string m_plugin_directory;
  std::string m_auto_recovery_directory;
  std::string m_control_replay_directory;
  std::string m_vehicle_profile="arducopter",m_facing_policy="course",m_copter_frame;
  std::vector<std::string> m_plugins;
  std::vector<std::string> m_loaded_plugins;
  std::string m_last_capabilities;
  std::atomic<uint64_t> m_disconnect_epoch{0};
  std::atomic<bool> m_registration_pending{false};
  uint64_t m_applied_disconnect_epoch=0;
  std::vector<std::pair<std::string,std::string>> m_plugin_config;
  std::map<std::string,std::vector<mavmoos::runtime::ModuleSetting>> m_module_config;
  std::string m_session = mavmoos::newSessionId();
  std::string m_state = "CONFIG_ERROR";
  std::string m_reason = "not_started";
  bool m_config_error = false;
  double m_last_publish = -1;
};
#endif

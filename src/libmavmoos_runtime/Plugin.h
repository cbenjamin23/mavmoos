#pragma once
#include <ardupilotmega/mavlink.h>
#include <array>
#include <string>
#include <vector>

namespace mavmoos::runtime {
constexpr unsigned plugin_abi = 10;
class VehicleControlBackend;
struct ObservationStream {
  uint32_t message_id=0;
  float interval_us=0;
};
struct Capability {
  std::string name;
  std::string request_variable;
  std::string request_value;
  std::string access="request";
};
struct PluginDescriptor {
  std::string profile;
  std::vector<Capability> capabilities;
  std::vector<std::string> exclusive_resources;
  std::vector<std::string> request_owners;
  std::vector<ObservationStream> observation_streams;
  bool streamed_parameter_bench_compatible=false;
};
struct Mail {
  std::string key, text, source;
  double value = 0, published_at = 0, received_at = 0;
  bool numeric = false;
};
struct MessageTime {
  double effective_at=0, read_at=0;
  std::string age_basis="READ_ACQUISITION";
};
struct Vehicle {
  bool heartbeat_valid=false, position_valid=false, home_valid=false, landed_valid=false, estimator_valid=false;
  bool estimator_horizontal_valid=false;
  bool armed=false, in_air=false;
  unsigned mode=0, landed_state=0;
  unsigned type=0, autopilot=0;
  double heartbeat_at=0, position_at=0, home_at=0, landed_at=0, estimator_at=0;
  double position_read_at=0;
  std::string position_time_basis="READ_ACQUISITION";
  double latitude=0, longitude=0, msl_height=0, relative_height=0;
  double north_velocity=0, east_velocity=0, down_velocity=0, heading=0;
  double home_latitude=0, home_longitude=0, home_msl=0;
  unsigned estimator_flags=0;
};
enum class NavigationKind { CourseSpeed, GlobalPosition };
enum class NavigationDimensions { Spatial, Planar };
enum class FacingPolicy { Course, Keep, Heading };
struct NavigationTarget {
  double course=0, speed=0, height=0, expires=0;
  NavigationKind kind=NavigationKind::CourseSpeed;
  NavigationDimensions dimensions=NavigationDimensions::Spatial;
  double latitude=0, longitude=0;
  FacingPolicy facing=FacingPolicy::Course;
  double facing_heading=0;
};
struct CommandRequest {
  std::string id;
  uint16_t command=0;
  std::array<float,7> params{};
  double expires=0;
  double operation_expires=0;
  // Mission execution is a separate persistent host lease. These bindings do
  // not grant permission when copied into an ordinary native command.
  std::string execution_id,abort_id;
};
enum class ParameterAction { Read, List, Write };
struct ParameterRequest {
  std::string id, name;
  ParameterAction action=ParameterAction::Read;
  int16_t index=-1;
  float value=0;
  uint8_t type=MAV_PARAM_TYPE_REAL32;
  double expires=0;
};
enum class MissionAction { RequestList, RequestItem, Count, Item, Clear, Ack };
struct MissionItem {
  uint16_t seq=0, command=MAV_CMD_NAV_WAYPOINT;
  uint8_t frame=MAV_FRAME_GLOBAL_RELATIVE_ALT, current=0, autocontinue=1;
  std::array<float,4> params{};
  int32_t x=0,y=0;
  float z=0;
};
struct MissionRequest {
  std::string id,operation_id;
  MissionAction action=MissionAction::RequestList;
  uint8_t mission_type=MAV_MISSION_TYPE_MISSION;
  uint16_t seq=0,count=0;
  uint8_t result=MAV_MISSION_ACCEPTED;
  MissionItem item;
  double expires=0,operation_expires=0;
};
enum class FlightLogAction { List, Data, End };
struct FlightLogRequest {
  std::string id, operation_id;
  FlightLogAction action=FlightLogAction::List;
  uint16_t first=0,last=0xffff,log_id=0;
  uint32_t offset=0,count=0;
  double expires=0,operation_expires=0;
};
struct StoredMissionProof {
  bool valid=false;
  uint64_t session_epoch=0,revision=0;
  std::string verification_id,content_id;
  double verified_at=0;
  std::vector<MissionItem> items;
};
struct ModuleSetting {
  std::string key, value;
};
enum class ModuleConfigStatus { Accepted, UnknownKey, Invalid };
struct ModuleConfigResult {
  ModuleConfigStatus status=ModuleConfigStatus::Accepted;
  std::string key, reason;
};
class Services {
public:
  virtual ~Services()=default;
  virtual double now() const=0;
  virtual const Vehicle& vehicle() const=0;
  virtual void publish(const std::string&, const std::string&)=0;
  virtual void publish(const std::string&, double)=0;
  virtual bool sendCommand(const CommandRequest&)=0;
  virtual bool acquireNavigation(const std::string& request_id)=0;
  virtual void revokeNavigation(const std::string& reason)=0;
  virtual bool navigation(const NavigationTarget&)=0;
  virtual bool parameter(const ParameterRequest&) { return false; }
  virtual bool mission(const MissionRequest&) { return false; }
  virtual void missionFinished(const std::string&) {}
  virtual StoredMissionProof storedMissionProof() const { return {}; }
  virtual bool recordStoredMissionProof(const std::string&,const std::vector<MissionItem>&) { return false; }
  virtual void invalidateStoredMissionProof(const std::string&) {}
  virtual bool beginMissionExecution(const std::string&,uint64_t,const std::string&,double) { return false; }
  virtual bool missionExecutionGranted(const std::string&) const { return false; }
  virtual bool beginMissionExecutionAbort(const std::string&,const std::string&,double) { return false; }
  virtual bool finishMissionExecution(const std::string&,const std::string&) { return false; }
  virtual bool navigationGranted() const { return false; }
  // Pending native command transactions, excluding a protocol client's own lease.
  virtual bool controlBusy() const { return false; }
  virtual bool projectGlobal(double,double,double&,double&) const { return false; }
  virtual bool suspendNavigation(double) { return false; }
  virtual bool resumeNavigation(const std::string&) { return false; }
  virtual void commandFinished(const std::string&) {}
  // Retire only waiting external work; sent-effect uncertainty retains its lease.
  virtual void cancelCommand(const std::string&) {}
  virtual bool flightLog(const FlightLogRequest&) { return false; }
  // Retire data work. Once a packet has reached the socket, only the same
  // operation may send a bounded END until its fixed cleanup deadline.
  virtual void flightLogFinished(const std::string&) {}
};
class Plugin {
public:
  virtual ~Plugin()=default;
  virtual std::string name() const=0;
  virtual PluginDescriptor descriptor() const { return {}; }
  virtual const VehicleControlBackend* controlBackend() const { return nullptr; }
  virtual bool projectGlobal(double,double,double&,double&) const { return false; }
  virtual std::vector<std::string> subscriptions() const { return {}; }
  virtual void configure(const std::string&,const std::string&) {}
  // One complete local startup batch. A module must recognize every supplied key.
  virtual ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) {
    if(settings.empty()) return {};
    return {ModuleConfigStatus::UnknownKey,settings.front().key,"module accepts no local settings"};
  }
  virtual void onMail(const Mail&) {}
  virtual void onMessage(const mavlink_message_t&,double) {}
  // Observations retain the conservative acquisition floor. Protocol handlers
  // keep actual local-read correlation through the legacy callback by default.
  virtual void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time) {
    onMessage(message,time.read_at);
  }
  virtual void onReset(const std::string&) {}
  virtual void onParameterSent(const ParameterRequest&,double,bool) {}
  virtual void onMissionSent(const MissionRequest&,double,bool) {}
  virtual void onCommandSent(const CommandRequest&,double,bool) {}
  // Observe the old cache at current owner time before incoming traffic refreshes it.
  virtual void onInputBoundary() {}
  virtual void onFlightLogSent(const FlightLogRequest&,double,bool) {}
  virtual void iterate()=0;
};
using CreatePlugin=Plugin* (*)(unsigned,Services*);
using DestroyPlugin=void (*)(Plugin*);
} // namespace mavmoos::runtime

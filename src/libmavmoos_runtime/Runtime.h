#pragma once
#include "Plugin.h"
#include "FileTransferService.h"
#include "PayloadService.h"
#include "PluginHost.h"
#include "ControlBackend.h"
#include "Px4Offboard.h"
#include "Px4RoverPolicy.h"
#include "ArduRoverMissionPolicy.h"
#include "ObservationRequests.h"
#include "FirmwareDiscovery.h"
#include "ControlReplayService.h"
#include <deque>
#include <functional>
#include <iosfwd>
#include <memory>
#include <optional>
#include <map>
#include <set>

namespace mavmoos::runtime {
class ControlReplayJournal;
class Runtime final: public Services, public FileTransferService, public PayloadService, public ExternalNavigationService,
  public Px4RoverMissionBinding, public ControlReplayService {
public:
  using Publish=std::function<void(const std::string&,const std::string&,double,bool)>;
  using Send=std::function<bool(const uint8_t*,std::size_t)>;
  using SendEligibility=std::function<bool(double)>;
  using BoundedSend=std::function<bool(const uint8_t*,std::size_t,double,const SendEligibility&)>;
  Runtime(Publish publish,Send send,bool simulation,bool shadow,std::function<double()> clock={},
          unsigned expected_type=MAV_TYPE_QUADROTOR,unsigned expected_autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA,
          bool defer_startup=false);
  ~Runtime() override;
  // Synchronous socket handoff used by the app; the two-argument constructor
  // sender remains a fake-I/O adapter for existing component tests.
  void setBoundedSend(BoundedSend send);
  // Explicit host opt-in for bounded read-only discovery. Passive observations
  // remain available in every mode; this never enables control or general commands.
  void setFirmwareRequestsEnabled(bool enabled);
  bool load(const std::string& path,const std::string& expected_name="");
  void configure(const std::string& key,const std::string& value);
  void configureModules(const std::map<std::string,std::vector<ModuleSetting>>& batches);
  // Host-only, after checked configuration and before finishStartup.
  void configureAutoRecovery(const std::string& directory,const std::string& process_session);
  void configureControlReplay(const std::string& directory,const std::string& route,bool create=false);
  bool controlReplayEnabled() const override { return m_control_replay_enabled; }
  bool controlRequestSeen(const std::string&) const override;
  ReplayResult reserveControlRequest(const std::string&) override;
  ReplayResult reserveControlEvent() override;
  // The app defers input/effects through its final policy and subscription checks.
  void finishStartup();
  // The app calls this after startup checks only for the dedicated file bench.
  using FileTransferPathGuard=std::function<bool(FileTransferAction,const std::string&)>;
  void enableFileTransferBench(FileTransferPathGuard);
  void enablePayloadBench(uint8_t camera,uint8_t gimbal);
  void enableExternalNavigationBench();
  std::string lastConfigurationError() const { return m_configuration_error.empty()?m_host.lastError():m_configuration_error; }
  std::vector<std::string> subscriptions() const;
  void mail(const Mail& mail,double original_publication=0);
  void message(const mavlink_message_t& msg,double acquired);
  void message(const mavlink_message_t& msg,double effective_at,double read_at,
               const std::string& age_basis="QUEUE_EMPTY_FLOOR");
  bool checkpoint(double now);
  void tick(double now);
  void disconnect(double boundary=-1,const std::string& reason="vehicle_connection_lost");
  std::string status() const;
  std::string capabilities() const;
  bool hasCapability(const std::string& name) const { return m_host.hasCapability(name); }
  bool streamedParameterBenchCompatible() const { return m_host.streamedParameterBenchCompatible(); }
  std::string controlProfile() const { return m_backend?m_backend->profile():""; }
  std::vector<std::string> moduleNames() const;
  double now() const override { return m_now; }
  const Vehicle& vehicle() const override { return m_vehicle; }
  void publish(const std::string&,const std::string&) override;
  void publish(const std::string&,double) override;
  bool sendCommand(const CommandRequest&) override;
  bool acquireNavigation(const std::string&) override;
  bool suspendNavigation(double resume_until) override;
  bool resumeNavigation(const std::string&) override;
  void commandFinished(const std::string&) override;
  void cancelCommand(const std::string&) override;
  void revokeNavigation(const std::string&) override;
  bool navigation(const NavigationTarget&) override;
  bool parameter(const ParameterRequest&) override;
  bool mission(const MissionRequest&) override;
  void missionFinished(const std::string&) override;
  bool flightLog(const FlightLogRequest&) override;
  void flightLogFinished(const std::string&) override;
  bool fileTransferReady() const override { return !m_startup_blocked && !m_configuration_callbacks && fileTransferGroundSafe(); }
  bool fileTransfer(const FileTransferRequest&) override;
  void retireFileTransferPacket(const std::string&,const std::string&) override;
  void cancelFileTransfer(const std::string&) override;
  void finishFileTransfer(const std::string&) override;
  bool payloadReady(PayloadAction) const override;
  bool payload(const PayloadRequest&) override;
  void cancelPayload(const std::string&) override;
  void finishPayload(const std::string&) override;
  bool externalNavigationReady() const override;
  bool beginExternalNavigation(uint64_t,double) override;
  bool externalNavigation(const ExternalNavigationSample&) override;
  void stopExternalNavigation(uint64_t) override;
  StoredMissionProof storedMissionProof() const override;
  bool recordStoredMissionProof(const std::string&,const std::vector<MissionItem>&) override;
  bool recordPx4MissionProof(const std::string&,const std::vector<MissionItem>&,uint32_t) override;
  uint32_t verifiedPx4MissionId() const override;
  void invalidateStoredMissionProof(const std::string&) override;
  bool beginMissionExecution(const std::string&,uint64_t,const std::string&,double) override;
  bool missionExecutionGranted(const std::string&) const override;
  bool beginMissionExecutionAbort(const std::string&,const std::string&,double) override;
  bool finishMissionExecution(const std::string&,const std::string&) override;
  bool navigationGranted() const override { return !m_startup_blocked && !m_configuration_callbacks && m_configuration_valid && !autoRecoveryBlocksEffects() && (m_nav_permission || (m_shadow && m_shadow_grant)); }
  bool controlBusy() const override { return autoRecoveryBlocksEffects() || m_execution.has_value() || !m_command_leases.empty(); }
  bool projectGlobal(double,double,double&,double&) const override;
private:
  void invalidateConfiguration();
  bool configurationEffectAttempt();
  bool fresh(double acquired,double limit) const;
  bool sampleClock();
  bool flightReady(bool motion) const;
  bool commandEligible(const CommandRequest&) const;
  bool px4Profile() const;
  bool px4FixedWingProfile() const;
  bool px4VtolProfile() const;
  bool px4ForwardFlight() const;
  bool px4AircraftNativeActions() const;
  bool px4OffboardIntentRequired() const;
  bool px4RoverProfile() const;
  const Px4RoverPolicy* px4RoverPolicy() const;
  bool px4ObservedMode(unsigned mode,double after) const;
  bool roverHoldStopped(double after) const;
  bool px4LocalReady() const;
  bool px4NativeHoldReady() const;
  void observePx4ModeIntent(const mavlink_message_t&,double);
  bool beginPx4Acquisition(const std::string&);
  void observePx4Local(const mavlink_message_t&,double);
  bool acceptPx4OrderedPosition(const mavlink_message_t&,double,uint32_t boot);
  void observePx4Mode(double);
  void refreshPx4();
  bool suspendPx4(double);
  bool resumePx4();
  bool px4ModeSendEligible(const CommandRequest&) const;
  void trackPx4Mode(bool accepted);
  void flushPx4Hold();
  void flushPx4Motion(const NavigationTarget&,uint64_t);
  void appendPx4Status(std::ostream&) const;
  bool subProfile() const;
  bool guidedHandoffRequired() const;
  bool quadPlaneProfile() const;
  unsigned quadPlanePhase() const;
  bool quadPlaneMissionEntry(const std::vector<MissionItem>&) const;
  bool quadPlaneCommandEligible(const CommandRequest&) const;
  bool quadPlaneHomeBound() const;
  bool quadPlaneAtLandingPoint() const;
  bool quadPlaneGroundHandback(double after) const;
  void observeQuadPlaneCursor(const mavlink_message_t&,double);
  void refreshQuadPlaneExecution();
  void trackQuadPlaneHandoff(const CommandRequest&,bool);
  bool planeProfile() const;
  bool quadPlaneHelmProfile() const;
  bool quadPlaneHoverProfile() const;
  bool quadPlaneHoverAdapterReady(bool in_guided) const;
  bool quadPlaneHoverSettling() const;
  unsigned planeHoldMode() const;
  unsigned quadPlaneHelmPhase() const;
  void observeQuadPlaneHoverAdapter(const mavlink_message_t&,double at);
  void flushQuadPlaneHoverPoint(const NavigationTarget&,uint64_t authority);
  bool planeGuidanceProfile() const;
  bool planeProfileReady() const;
  std::size_t planeParameterCount() const;
  bool planeHeartbeatEligible() const;
  bool planeCanReadProfile() const;
  bool planeModeHandoffEligible(const CommandRequest&,bool acquisition) const;
  bool planeArmAllowed(const CommandRequest&) const;
  bool planeSpeedAllowed(const CommandRequest&) const;
  bool planeNativeAutoland() const;
  bool planeMissionEntry(const std::vector<MissionItem>&) const;
  bool planeMissionCommandEligible(const CommandRequest&) const;
  bool planeMissionCompleted() const;
  void refreshPlaneMissionExecution();
  void trackPlaneMissionHandoff(const CommandRequest&,bool accepted);
  bool planeExplicitModeAllowed(const CommandRequest&) const;
  bool planeResumeAllowed() const;
  void observePlaneMessage(const mavlink_message_t&,double effective_at,double read_at);
  void flushPlaneProfileRead();
  void refreshPlaneAuthority();
  void planPlaneHandback(const std::string& reason,bool held);
  void queuePlaneHandback();
  void trackPlaneMode(const CommandRequest&,bool acquisition,bool accepted);
  void flushPlanePoint(const NavigationTarget&,uint64_t authority);
  void appendPlaneStatus(std::ostream&) const;
  bool parameterEligible(const ParameterRequest&) const;
  bool missionEligible(const MissionRequest&) const;
  void expireMissionOperations();
  bool flightLogGroundSafe() const;
  bool flightLogEligible(const FlightLogRequest&) const;
  void refreshFlightLog();
  void resetFlightLog();
  bool fileTransferGroundSafe() const;
  bool fileTransferEligible(const FileTransferRequest&) const;
  void refreshFileTransfer();
  void resetFileTransfer();
  void flushFileTransfer();
  bool externalNavigationEligible(const ExternalNavigationSample&) const;
  void refreshExternalNavigation();
  void resetExternalNavigation();
  bool externalNavigationWaiting() const;
  void flushExternalNavigation();
  bool payloadGroundSafe() const;
  bool payloadEligible(const PayloadRequest&) const;
  bool payloadWaiting() const;
  void observePayload(const mavlink_message_t&,double effective_at);
  void refreshPayload();
  void resetPayload();
  void flushPayload();
  bool executionCommandEligible(const CommandRequest&) const;
  void refreshMissionExecution();
  void refreshPx4RoverExecution();
  void observePx4RoverMissionCurrent(const mavlink_message_t&,double);
  bool recordMissionProof(const std::string&,const std::vector<MissionItem>&,uint32_t remote_id);
  bool px4MissionIdentityCurrent() const;
  bool px4RoverMissionCommandEligible(const CommandRequest&) const;
  void trackPx4RoverMissionHandoff(const CommandRequest&,bool accepted);
  const ArduRoverMissionPolicy* arduRoverMissionPolicy() const;
  bool arduRoverHandbackMode() const;
  bool arduRoverMissionHandback(double after) const;
  bool arduRoverStopAllowed() const;
  void observeArduRoverAutoAuthority();
  void observeArduRoverMissionCurrent(const mavlink_message_t&,double);
  bool arduRoverCursorFresh(double after) const;
  bool arduRoverMissionEntryReady(const std::vector<MissionItem>&) const;
  void refreshArduRoverMissionExecution();
  bool arduRoverMissionCommandEligible(const CommandRequest&) const;
  void trackArduRoverMissionHandoff(const CommandRequest&,bool accepted);
  bool missionEntryReady(const std::vector<MissionItem>&) const;
  unsigned executionMode() const;
  bool observedExecutionMode() const;
  void quarantineMissionExecution(const std::string&);
  void cancelExecutionCommands(bool include_abort);
  void publishMissionExecutionLease();
  void publishStoredMissionProof(const std::string&);
  bool autoRecoveryBlocksEffects() const;
  const char* autoRecoveryState() const;
  void observeAutoRecoveryHeartbeat(const mavlink_message_t&);
  void handleAutoRecoveryRequest(const Mail&);
  void publishAutoRecoveryStatus();
  bool freshNonAutoRecoveryEvidence() const;
  bool armAutoRecoveryMarker(double original_deadline);
  bool clearAutoRecoveryMarker();
  void autoRecoveryStorageFault(const char* reason);
  bool flushParameter();
  bool flushMission();
  void flushFlightLog();
  bool flushObservationRequest();
  void publishStreamStatus();
  bool firmwareEffectsAllowed() const;
  bool flushFirmwareRequest();
  void publishFirmwareStatus(bool force=false);
  bool send(const mavlink_message_t&,double expires=0,std::function<bool()> eligible={});
  bool sendControlFrame(const ControlFrame&,double expires,std::function<bool()> eligible);
  bool handoff(const uint8_t*,std::size_t,double,const std::function<bool()>&);
  void refreshValidity();
  bool queueCommand(const CommandRequest&,bool navigation_acquisition);
  bool acquireNavigationImpl(const std::string&);
  bool navigationModeAllowed() const;
  void retireAcquisition();
  void expireCommandLeases();
  Publish m_publish; Send m_send; BoundedSend m_bounded_send;
  std::function<double()> m_clock;
  bool m_simulation=false,m_shadow=true,m_nav_permission=false,m_had_connection=false;
  bool m_configuration_valid=true;
  bool m_startup_blocked=false;
  bool m_configuration_callbacks=false,m_configuration_effect_attempted=false,m_module_configuration_complete=false,m_operational_started=false;
  std::string m_configuration_error;
  double m_now=0,m_last_heartbeat=-10,m_last_nav=-10,m_acquire_until=0;
  Vehicle m_vehicle;
  PluginHost m_host;
  const VehicleControlBackend* m_backend=nullptr;
  unsigned m_expected_type,m_expected_autopilot;
  unsigned m_acquire_from=0;
  double m_guided_acquire_sent_at=-1;
  struct Px4LocalSample { px4::LocalPoint point;double at;uint32_t boot; };
  std::optional<Px4LocalSample> m_px4_local;
  std::optional<uint32_t> m_px4_global_boot;
  struct Px4ModeSample { unsigned active=0,intended=0;double at=-1; } m_px4_mode;
  uint64_t m_px4_estimator_usec=0,m_px4_hold_accepted=0;
  struct Px4Session {
    std::string id;
    std::optional<px4::LocalPoint> anchor;
    double intent_until=0,mode_sent_at=-1,first_setpoint=-1,last_setpoint=-1;
    unsigned prestream_count=0;
    bool confirmed=false,motion_started=false;
  } m_px4;
  struct QueuedCommand { CommandRequest request; bool navigation_acquisition=false; };
  std::deque<QueuedCommand> m_commands;
  std::optional<QueuedCommand> m_command_handoff;
  std::map<std::string,double> m_command_leases;
  double m_resume_until=0;
  bool m_shadow_grant=false;
  std::deque<ParameterRequest> m_parameters;
  std::deque<MissionRequest> m_missions;
  std::optional<MissionRequest> m_mission_handoff;
  struct MissionOperation { double expires; uint8_t type; };
  std::map<std::string,MissionOperation> m_mission_operations;
  std::map<std::string,double> m_mission_mutations;
  struct FlightLogOperation {
    std::string id;
    double expires;
    bool handed_off=false,end_only=false;
  };
  std::optional<FlightLogOperation> m_flight_log;
  std::optional<FlightLogRequest> m_flight_log_pending,m_flight_log_handoff;
  struct FileTransferLease {
    std::string id;
    double expires;
    uint8_t session;
    bool read=false,open_handed_off=false,cleanup_only=false;
  };
  FileTransferPathGuard m_file_path_guard;
  bool m_files_bench=false;
  std::optional<FileTransferLease> m_file_transfer;
  std::optional<FileTransferRequest> m_file_pending,m_file_handoff;
  struct PayloadLease {
    std::string operation_id;
    double until=0,claim_sent_at=-1;
    bool cancelled=false,started=false,point_attempted=false,release_attempted=false;
  };
  struct PayloadSlot {
    uint8_t component=0;
    std::optional<PayloadLease> lease;
    std::optional<PayloadRequest> pending;
  };
  std::array<PayloadSlot,2> m_payload_slots{};
  std::optional<PayloadRequest> m_payload_handoff;
  bool m_payload_bench=false;
  bool m_external_navigation_bench=false;
  struct ExternalNavigationLease {
    uint64_t run_id;
    double expires,started,last_measurement;
    uint64_t sequence=0,capture_us=0;
  };
  std::optional<ExternalNavigationLease> m_external_lease;
  std::optional<ExternalNavigationSample> m_external_pending,m_external_handoff;
  uint64_t m_external_high_run=0,m_external_accepted=0,m_external_replaced=0;
  double m_external_last_send=-1;
  unsigned m_payload_next=0;
  uint32_t m_payload_camera_sequence=0;
  struct {
    double information_at=0,status_at=0;
    uint32_t boot_ms=0;
    bool capture_supported=false,idle=false,seen_status=false;
  } m_payload_camera;
  struct {
    double at=0;
    uint32_t boot_ms=0,flags=0;
    uint8_t primary_system=0,primary_component=0,secondary_system=0,secondary_component=0;
    bool valid=false,seen=false;
  } m_payload_gimbal;
  std::optional<MissionRequest> m_mission_receipt;
  bool m_mission_receipt_accepted=false;
  StoredMissionProof m_stored_mission;
  uint64_t m_stored_mission_revision=0;
  uint32_t m_px4_mission_id=0;
  unsigned m_quadplane_cursor=0;
  double m_quadplane_cursor_at=-1;
  struct Px4MissionCursor { uint32_t id=0; uint16_t seq=0,total=0; double at=-1; } m_px4_mission_cursor;
  struct ArduRoverMissionCursor { mavlink_mission_current_t value{}; double at=-1; } m_ardu_rover_cursor;
  struct ExecutionLease {
    enum class RoverPhase { Starting, Driving, PauseSent, Paused, ResumeSent };
    enum class ArduRoverAutoAuthority { BeforeStart, Owned, Paused, Lost };
    std::string id,content_id,reason="active",abort_id;
    uint64_t session_epoch=0,proof_revision=0,abort_epoch=0;
    double expires=0,started=0,abort_expires=0,abort_started=0,abort_sent_at=-1,last_execution_handoff=-1;
    Vehicle home;
    bool quarantined=false,start_handoff=false,auto_seen=false;
    double copter_start_accepted_at=-1;
    bool copter_auto_departed=false;
    bool rover_start_sent=false,rover_abort_attempted=false;
    RoverPhase rover_phase=RoverPhase::Starting;
    double rover_transition_at=-1,ardurover_select_at=-1;
    uint16_t ardurover_paused_seq=0;
    ArduRoverAutoAuthority ardurover_auto_authority=ArduRoverAutoAuthority::BeforeStart;
    unsigned ardurover_abort_from=0;
    bool ardurover_abort_mode_changed=false;
    double quadplane_selected_at=-1,quadplane_handback_at=-1;
    bool quadplane_start_sent=false,quadplane_disarm_sent=false,quadplane_abort_attempted=false,quadplane_left_auto=false;
    std::set<std::string> command_ids;
    std::set<std::string> abort_ids;
  };
  std::optional<ExecutionLease> m_execution;
  struct AutoRecovery {
    enum class State { AwaitingHeartbeat, Ready, Unresolved, StorageFault };
    ~AutoRecovery();
    int directory_fd=-1;
    std::string directory,session,reason,last_status;
    State state=State::AwaitingHeartbeat;
    bool marker_present=false;
    double observation_after=0,mail_after=0,observed_at=-1,last_report=-1;
    unsigned observed_mode=0;
  };
  std::unique_ptr<AutoRecovery> m_auto_recovery;
  std::unique_ptr<ControlReplayJournal> m_control_replay;
  std::string m_replay_event;
  bool m_control_replay_enabled=false;
  // Mission, parameter, discovery/streams, flight logs, files. Preserve parameter-first startup.
  unsigned m_next_protocol=1;
  struct PlaneState {
    struct Target {
      int32_t lat=0,lon=0;
      float relative_height=0;
      double sent_at=0,verify_until=0,msl_height=0,verified_at=0;
    };
    struct Transition {
      std::string id;
      unsigned origin=0,destination=0;
      double until=0,sent_at=-1;
      bool internal=false,queued=false,attempted=false;
    };
    std::optional<Target> awaiting,verified;
    std::optional<std::pair<int32_t,int32_t>> hover_hold;
    std::optional<Transition> transition;
    std::string handback="IDLE";
    std::size_t parameter_index=0;
    unsigned parameter_attempts=0;
    bool parameter_mismatch=false,heartbeat_started=false,guided_confirmed=false,version_verified=false;
    bool ground_disarmed_seen=false;
    std::optional<double> launch_heading;
    double parameter_sent_at=-1,parameter_retry_at=0,last_target_at=-10;
    double acquire_sent_at=-1,intent_until=0;
    double hover_offer_at=-1,hover_lease_until=0;
    uint32_t hover_offer=0,hover_attempt=0;
    uint8_t hover_offer_kind=0;
    unsigned mission_seq=0;
    double mission_cursor_at=-1,mission_selected_at=-1,mission_started_at=-1,mission_return_sent_at=-1;
    uint64_t verified_count=0;
  } m_plane;
  std::optional<NavigationTarget> m_navigation;
  std::optional<ControlFrame> m_hold;
  double m_hold_until=0;
  std::string m_reason="not_acquired";
  uint64_t m_attempts=0,m_accepted=0,m_expired=0,m_cancelled=0,m_revision=0,m_nav_accepted=0,m_authority_revision=0;
  ObservationRequests m_stream_requests;
  FirmwareDiscovery m_firmware;
  bool m_firmware_requests_enabled=false,m_firmware_turn=true;
  double m_last_firmware_publish=-1;
  uint64_t m_connection_revision=1;
  mavlink_status_t m_codec{};
};
}

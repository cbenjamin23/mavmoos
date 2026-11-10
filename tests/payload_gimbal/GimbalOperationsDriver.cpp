#include "PayloadService.h"
#include "Plugin.h"

#include <algorithm>
#include <cmath>
#include <dlfcn.h>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

using namespace mavmoos::runtime;

namespace {
constexpr uint8_t component = 154;
constexpr char request_key[] = "MAVMOOS_GIMBAL_REQUEST";
constexpr char result_key[] = "MAVMOOS_GIMBAL_RESULT";
constexpr float pitch = -30, yaw = 40;
constexpr char point_request[] = "schema=1,id=point1,action=point,pitch_deg=-30,yaw_deg=40";
constexpr char cancel_request[] = "schema=1,id=cancel1,action=cancel,operation_id=point1";

void require(bool condition, const char* failure)
{
  if(!condition) throw std::runtime_error(failure);
}

struct Host final : Services, PayloadService {
  double time = 100;
  Vehicle state;
  bool bench_ready = true, accept = true;
  std::vector<std::string> results, cancelled, finished;
  std::vector<PayloadRequest> requests;
  std::optional<PayloadRequest> pending;

  double now() const override { return time; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key, const std::string& text) override
  {
    if(key == result_key) results.push_back(text);
  }
  void publish(const std::string&, double) override { unexpected(); }
  bool sendCommand(const CommandRequest&) override { unexpected(); }
  bool acquireNavigation(const std::string&) override { unexpected(); }
  void revokeNavigation(const std::string&) override { unexpected(); }
  bool navigation(const NavigationTarget&) override { unexpected(); }

  bool payloadReady(PayloadAction) const override { return bench_ready; }
  bool payload(const PayloadRequest& request) override
  {
    require(!pending && request.operation_id == "gimbal/point1" && request.capture_sequence == 0 &&
            time < request.expires && request.expires <= request.operation_expires,
            "request bypassed expected operation scope");
    if(!accept) return false;
    requests.push_back(request);
    pending = request;
    return true;
  }
  void cancelPayload(const std::string& operation) override
  {
    require(operation == "gimbal/point1", "unscoped payload cancellation");
    cancelled.push_back(operation);
    pending.reset();
  }
  void finishPayload(const std::string& operation) override
  {
    require(operation == "gimbal/point1", "unscoped payload completion");
    finished.push_back(operation);
    pending.reset();
  }
  bool has(std::string_view status, std::string_view reason) const
  {
    const std::string target = ",status=" + std::string(status) + ",reason=" + std::string(reason) + ',';
    return std::any_of(results.begin(), results.end(), [&](const auto& value) {
      return value.find(target) != std::string::npos;
    });
  }
  [[noreturn]] static void unexpected() { throw std::runtime_error("operation escaped typed payload service"); }
};

struct Rig {
  Host host;
  Plugin* const plugin;
  DestroyPlugin destroy;
  PayloadListener* const listener;
  mavlink_status_t codec{};
  uint32_t manager_ms = 1000, pose_ms = 1000;

  Rig(CreatePlugin create, DestroyPlugin deleter)
    : plugin(create(plugin_abi, &host)), destroy(deleter), listener(dynamic_cast<PayloadListener*>(plugin))
  {
    require(plugin && listener, "gimbal module missing typed payload listener");
    host.state.type = MAV_TYPE_QUADROTOR;
    host.state.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    plugin->configure("gimbal_component", "154");
    require(plugin->configureModule({{"point_enabled", "true"}}).status == ModuleConfigStatus::Accepted,
            "gimbal point configuration failed");
    const auto descriptor = plugin->descriptor();
    require(descriptor.request_owners == std::vector<std::string>{request_key} &&
            plugin->subscriptions() == descriptor.request_owners && !descriptor.streamed_parameter_bench_compatible,
            "gimbal capability was not scoped to opt-in payload operations");
  }
  ~Rig() { destroy(plugin); }

  void receive(const mavlink_message_t& frame, double read, double effective = -1)
  {
    host.time = read;
    plugin->onMessageWithMetadata(frame, {effective < 0 ? read : effective, read,
      effective < 0 ? "READ_ACQUISITION" : "QUEUE_EMPTY_FLOOR"});
  }
  void heartbeat(double at)
  {
    host.state.heartbeat_valid = true;
    host.state.heartbeat_at = at;
    mavlink_message_t frame{};
    mavlink_msg_heartbeat_pack_status(1, 1, &codec, &frame, MAV_TYPE_QUADROTOR,
      MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, MAV_STATE_STANDBY);
    receive(frame, at);
  }
  void manager(double read, uint8_t system = 0, uint8_t owner = 0, double effective = -1,
               uint8_t source = component, uint8_t device = component, bool repeated = false)
  {
    mavlink_gimbal_manager_status_t status{};
    status.time_boot_ms = repeated ? manager_ms : ++manager_ms;
    status.gimbal_device_id = device;
    status.primary_control_sysid = system;
    status.primary_control_compid = owner;
    mavlink_message_t frame{};
    mavlink_msg_gimbal_manager_status_encode_status(1, source, &codec, &frame, &status);
    receive(frame, read, effective);
  }
  void pose(double read, float pitch_deg = pitch, float yaw_deg = yaw,
            uint16_t flags = GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME, uint8_t source = component,
            double effective = -1)
  {
    mavlink_gimbal_device_attitude_status_t status{};
    status.time_boot_ms = ++pose_ms;
    status.flags = flags;
    const double half_rad = 0.008726646259971648;
    const double cp = std::cos(pitch_deg * half_rad), sp = std::sin(pitch_deg * half_rad);
    const double cy = std::cos(yaw_deg * half_rad), sy = std::sin(yaw_deg * half_rad);
    status.q[0] = cp * cy;
    status.q[1] = -sp * sy;
    status.q[2] = sp * cy;
    status.q[3] = cp * sy;
    mavlink_message_t frame{};
    mavlink_msg_gimbal_device_attitude_status_encode_status(1, source, &codec, &frame, &status);
    receive(frame, read, effective);
  }
  void ack(double read, uint16_t command)
  {
    mavlink_message_t frame{};
    mavlink_msg_command_ack_pack_status(1, component, &codec, &frame, command,
      MAV_RESULT_ACCEPTED, 0, 0, 245, 191);
    receive(frame, read);
  }
  void mail(double at, const char* text = point_request, double published = -1)
  {
    host.time = at;
    plugin->onMail({request_key, text, "operator", 0, published < 0 ? at : published, at, false});
  }
  void ready()
  {
    heartbeat(100.1);
    manager(100.2);
    pose(100.21, 0, 0);
  }
  void sent(double at, PayloadAction action, bool accepted = true)
  {
    require(host.pending && host.pending->action == action, "expected outbound payload request missing");
    const auto request = *std::exchange(host.pending, std::nullopt);
    host.time = at;
    listener->onPayloadSent(request, at, accepted);
  }
  void tick(double at)
  {
    host.time = at;
    plugin->iterate();
  }
};

void complete(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  rig.ready();
  rig.mail(100.3);
  require(rig.host.has("QUEUED", "original_deadline_bound") &&
          rig.host.pending->action == PayloadAction::GimbalClaim, "claim was not queued");
  const double deadline = rig.host.pending->operation_expires;
  rig.sent(100.35, PayloadAction::GimbalClaim);
  rig.ack(100.36, MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE);
  require(rig.host.requests.size() == 1, "command ACK alone advanced claim");
  rig.manager(100.4, 245, 191, 100.34);
  require(rig.host.requests.size() == 1, "claim consumed pre-handoff ownership");
  rig.manager(100.45, 245, 191, -1, 155);
  rig.manager(100.46, 245, 191, -1, component, 155);
  require(rig.host.requests.size() == 1, "claim consumed foreign ownership");
  rig.manager(100.5, 245, 191);
  require(rig.host.pending && rig.host.pending->pitch_deg == pitch && rig.host.pending->yaw_deg == yaw &&
          rig.host.pending->operation_expires == deadline, "point did not preserve aim and original deadline");
  rig.sent(100.55, PayloadAction::GimbalPoint);
  rig.pose(100.56, pitch, yaw, GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME, component, 100.54);
  require(!rig.host.has("POINTED", "matching_vehicle_frame_pose"), "point consumed pre-handoff pose");
  rig.pose(100.6, pitch, yaw, GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME);
  rig.pose(100.61, pitch, yaw, GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME, 155);
  rig.pose(100.62, pitch + 5, yaw);
  require(!rig.host.has("POINTED", "matching_vehicle_frame_pose"), "unqualified pose confirmed aim");
  rig.pose(100.7);
  require(rig.host.has("POINTED", "matching_vehicle_frame_pose") && rig.host.pending &&
          rig.host.pending->operation_expires == deadline, "matching pose did not queue bounded release");
  rig.sent(100.75, PayloadAction::GimbalRelease);
  rig.manager(100.8, 0, 0, 100.74);
  rig.manager(100.81, 0, 0, -1, component, component, true);
  require(!rig.host.has("CONFIRMED", "point_released"), "release consumed old ownership report");
  rig.manager(100.9);
  require(rig.host.has("CONFIRMED", "point_released") && rig.host.finished.size() == 1 &&
          rig.host.requests.size() == 3, "point and release did not complete exactly once");
  rig.mail(100.95);
  require(rig.host.has("REJECTED", "duplicate_or_history_full") && !rig.host.pending,
          "duplicate operator request restarted a completed action");
}

void cancelQueued(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  rig.ready();
  rig.mail(100.25);
  rig.mail(100.3, cancel_request);
  require(rig.host.has("CANCEL_ACCEPTED", "operator_cancel") && rig.host.has("CANCELLED", "claim_retired") &&
          !rig.host.pending && rig.host.requests.size() == 1 && rig.host.finished.size() == 1,
          "queued claim cancellation left a payload effect or issued cleanup");
}

void cancelClaimInFlight(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  rig.ready();
  rig.mail(100.25);
  rig.sent(100.3, PayloadAction::GimbalClaim);
  rig.mail(100.35, cancel_request);
  require(!rig.host.pending && rig.host.requests.size() == 1, "cleanup began before observing ownership");
  rig.manager(100.4, 245, 191);
  rig.sent(100.5, PayloadAction::GimbalRelease);
  rig.manager(100.6);
  require(rig.host.has("CANCELLED", "operator_cancel") && rig.host.requests.size() == 2,
          "cancelled claim did not settle by release without pointing");
}

void takeover(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  rig.ready();
  rig.mail(100.25);
  rig.sent(100.3, PayloadAction::GimbalClaim);
  rig.manager(100.4, 245, 191);
  require(rig.host.pending && rig.host.pending->action == PayloadAction::GimbalPoint,
          "scenario failed to queue the point before takeover");
  rig.manager(100.5, 42, 190);
  require(rig.host.has("FAILED", "manager_taken_over") && !rig.host.pending &&
          rig.host.requests.size() == 2 && rig.host.finished.size() == 1,
          "takeover failed to retire point work or emitted release against another owner");
  rig.manager(100.6, 245, 191);
  rig.pose(100.7);
  require(rig.host.requests.size() == 2, "observations resurrected retired work");
}

void poseLostBeforePoint(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  rig.ready();
  rig.mail(100.25);
  rig.sent(100.3, PayloadAction::GimbalClaim);
  rig.manager(100.4, 245, 191);
  require(rig.host.pending && rig.host.pending->action == PayloadAction::GimbalPoint,
          "scenario failed to queue a point");
  rig.pose(100.5, pitch, yaw, GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME);
  rig.sent(100.55, PayloadAction::GimbalRelease);
  rig.manager(100.6);
  require(rig.host.has("FAILED", "pose_lost_before_point"), "pose loss did not cancel a queued point");
}

void relinquishedBeforeRelease(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  rig.ready();
  rig.mail(100.25);
  rig.sent(100.3, PayloadAction::GimbalClaim);
  rig.manager(100.4, 245, 191);
  rig.sent(100.5, PayloadAction::GimbalPoint);
  rig.pose(100.6);
  require(rig.host.pending && rig.host.pending->action == PayloadAction::GimbalRelease,
          "scenario failed to queue a release");
  rig.manager(100.65);
  require(rig.host.has("FAILED", "manager_relinquished") && !rig.host.pending &&
          rig.host.finished.size() == 1, "free manager did not retire queued release");
}

void rejectAndReset(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  rig.ready();
  rig.mail(100.22, "schema=1,id=invalid,action=point,pitch_deg=nan,yaw_deg=40");
  require(rig.host.has("REJECTED", "invalid_request"), "nonfinite point was admitted");
  rig.mail(100.23, point_request, 99.);
  require(rig.host.has("REJECTED", "stale_future_or_pre_session_request"), "pre-session point was admitted");
  rig.host.bench_ready = false;
  rig.mail(100.24);
  require(rig.host.has("REJECTED", "fresh_free_manager_and_vehicle_pose_required") && !rig.host.pending,
          "point was admitted with host bench disabled");
  rig.host.bench_ready = true;
  rig.mail(100.25);
  rig.sent(100.3, PayloadAction::GimbalClaim);
  rig.host.time = 100.4;
  rig.plugin->onReset("fixture_reset");
  require(rig.host.has("UNCERTAIN", "session_reset") && !rig.host.pending &&
          rig.host.finished.size() == 1, "reset treated possible claim as confirmed or replayable");
}

void releaseTimeout(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  rig.ready();
  rig.mail(100.25);
  rig.sent(100.3, PayloadAction::GimbalClaim);
  rig.manager(100.4, 245, 191);
  rig.sent(100.5, PayloadAction::GimbalPoint);
  rig.pose(100.6);
  rig.sent(100.7, PayloadAction::GimbalRelease);
  rig.heartbeat(101.6);
  rig.manager(101.7, 245, 191);
  rig.heartbeat(102.7);
  rig.tick(102.8);
  require(rig.host.has("UNCERTAIN", "release_unconfirmed") && rig.host.requests.size() == 3,
          "release timeout silently succeeded or retransmitted");
}
} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "expected plugin path");
    void* handle = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    if(!handle) throw std::runtime_error(dlerror());
    auto create = reinterpret_cast<CreatePlugin>(dlsym(handle, "mavmoos_create_plugin"));
    auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(handle, "mavmoos_destroy_plugin"));
    require(create && destroy, "missing plugin factories");
    complete(create, destroy);
    cancelQueued(create, destroy);
    cancelClaimInFlight(create, destroy);
    takeover(create, destroy);
    poseLostBeforePoint(create, destroy);
    relinquishedBeforeRelease(create, destroy);
    rejectAndReset(create, destroy);
    releaseTimeout(create, destroy);
    dlclose(handle);
    std::cout << "Gimbal point, release, cancellation, takeover, reset and uncertainty checks passed\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

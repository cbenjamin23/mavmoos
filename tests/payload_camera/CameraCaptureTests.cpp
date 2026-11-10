#include "CameraFrames.h"
#include "PayloadService.h"
#include "Plugin.h"

#include <cmath>
#include <dlfcn.h>
#include <iostream>
#include <memory>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace fixture = mavmoos::test::camera;

namespace {
constexpr char request_key[] = "MAVMOOS_CAMERA_REQUEST";

void require(bool valid, const char* reason)
{
  if (!valid) throw std::runtime_error(reason);
}

struct Host final : Services, PayloadService {
  double time = 100;
  bool bench = true, admit = true;
  Vehicle state;
  std::vector<PayloadRequest> requests;
  std::vector<std::string> results, cancelled, finished;

  Host()
  {
    state.type = MAV_TYPE_QUADROTOR;
    state.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
  }
  double now() const override { return time; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key, const std::string& value) override
  {
    if (key == "MAVMOOS_CAMERA_RESULT") results.push_back(value);
  }
  void publish(const std::string&, double) override { unexpected(); }
  bool sendCommand(const CommandRequest&) override { unexpected(); }
  bool acquireNavigation(const std::string&) override { unexpected(); }
  void revokeNavigation(const std::string&) override { unexpected(); }
  bool navigation(const NavigationTarget&) override { unexpected(); }

  bool payloadReady(PayloadAction action) const override
  { return bench && action == PayloadAction::CameraCaptureOne; }
  bool payload(const PayloadRequest& request) override
  {
    if (!admit) return false;
    requests.push_back(request);
    return true;
  }
  void cancelPayload(const std::string& id) override { cancelled.push_back(id); }
  void finishPayload(const std::string& id) override { finished.push_back(id); }

  [[noreturn]] static void unexpected() { throw std::runtime_error("camera used an unrelated effect path"); }
};

struct Rig {
  Host host;
  std::unique_ptr<Plugin, DestroyPlugin> plugin;
  PayloadListener* listener;
  mavlink_status_t codec{};

  Rig(CreatePlugin create, DestroyPlugin destroy) : plugin(create(plugin_abi, &host), destroy),
    listener(dynamic_cast<PayloadListener*>(plugin.get()))
  {
    require(plugin && listener, "camera payload listener unavailable from loaded module");
    plugin->configure("camera_component", "100");
    require(plugin->subscriptions().empty() && plugin->descriptor().capabilities.size() == 4,
            "camera operations enabled without opt-in");
    require(plugin->configureModule({{"capture_enabled", "yes"}}).status == ModuleConfigStatus::Invalid,
            "malformed capture opt-in accepted");
    require(plugin->configureModule({{"capture_enabled", "true"}}).status == ModuleConfigStatus::Accepted &&
            plugin->subscriptions() == std::vector<std::string>{request_key},
            "camera operation subscription unavailable");
    require(plugin->descriptor().capabilities.back().name == "camera.capture.single" &&
            !plugin->descriptor().streamed_parameter_bench_compatible,
            "capture facet absent from startup descriptor");
  }

  void receive(const mavlink_message_t& message, double at, double effective = -1)
  {
    host.time = at;
    plugin->onMessageWithMetadata(message, {effective < 0 ? at : effective, at,
      effective < 0 ? "READ_ACQUISITION" : "QUEUE_EMPTY_FLOOR"});
  }

  void heartbeat(double at)
  {
    host.state.heartbeat_valid = true;
    host.state.heartbeat_at = at;
    mavlink_message_t frame{};
    mavlink_msg_heartbeat_pack_status(1, 1, &codec, &frame, host.state.type, host.state.autopilot,
                                      0, 0, MAV_STATE_STANDBY);
    receive(frame, at);
  }

  void status(double at, int32_t count, uint32_t boot, unsigned image_status = 0)
  {
    auto value = fixture::capturePayload();
    value.image_status = static_cast<uint8_t>(image_status);
    value.video_status = 0;
    value.image_count = count;
    value.time_boot_ms = boot;
    receive(fixture::encode(codec, value, mavlink_msg_camera_capture_status_encode_status), at);
  }

  void ready(double at, int32_t count, uint32_t boot = 300)
  {
    heartbeat(at);
    receive(fixture::information(codec), at + .01);
    status(at + .02, count, boot);
  }

  void mail(const std::string& id, double at, const char* action = "capture", double age = 0)
  {
    host.time = at;
    plugin->onMail({request_key, "schema=1,action=" + std::string(action) + ",id=" + id,
                    "fixture", 0, at - age, at, false});
  }

  void sent(double at, bool accepted = true)
  {
    host.time = at;
    require(!host.requests.empty(), "no accepted camera request to report");
    listener->onPayloadSent(host.requests.back(), at, accepted);
  }

  void ack(double at, uint8_t source = 100, uint8_t target = 191, uint8_t result = MAV_RESULT_ACCEPTED,
           double effective = -1, uint8_t system = 1)
  {
    mavlink_command_ack_t value{};
    value.command = MAV_CMD_IMAGE_START_CAPTURE;
    value.result = result;
    value.target_system = 245;
    value.target_component = target;
    receive(fixture::encode(codec, value, mavlink_msg_command_ack_encode_status, system, source), at, effective);
  }

  void image(double at, int32_t index, uint32_t boot, uint8_t source = 100, int8_t result = 1,
             double effective = -1)
  {
    mavlink_camera_image_captured_t value{};
    value.time_boot_ms = boot;
    value.image_index = index;
    value.capture_result = result;
    receive(fixture::encode(codec, value, mavlink_msg_camera_image_captured_encode_status, 1, source),
            at, effective);
  }

  bool reported(const std::string& id, const std::string& status, const char* reason = nullptr) const
  {
    const auto marker = "\"id\":\"" + id + "\",\"status\":\"" + status + '"';
    for (const auto& result : host.results)
      if (result.find(marker) != std::string::npos &&
          (!reason || result.find(reason) != std::string::npos)) return true;
    return false;
  }
};

void confirmation(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  rig.ready(100.1, 8);
  rig.mail("shot-1", 100.2, "capture", .05);
  require(rig.reported("shot-1", "QUEUED") && rig.host.requests.size() == 1,
          "fresh idle camera request did not queue");
  const auto& request = rig.host.requests.front();
  require(request.action == PayloadAction::CameraCaptureOne && request.id == "shot-1" &&
          request.operation_id == "camera/shot-1" && request.capture_sequence == 1 &&
          std::abs(request.expires - 102.15) < .00001 && std::abs(request.operation_expires - 106.15) < .00001,
          "single-shot request did not retain identity or original deadlines");
  rig.ack(100.21);
  rig.image(100.22, 8, 301);
  require(!rig.reported("shot-1", "ACK_OBSERVED"), "pre-send response joined a queued request");
  rig.sent(100.23);
  rig.ack(100.24, 101);
  rig.ack(100.25, 100, 0);
  rig.ack(100.26, 100, 191, MAV_RESULT_ACCEPTED, -1, 2);
  rig.ack(100.27, 100, 191, MAV_RESULT_IN_PROGRESS);
  require(!rig.reported("shot-1", "ACK_OBSERVED"), "foreign or nonterminal ACK admitted");
  // Queue-empty floor can precede the local handoff, while the actual read follows it.
  rig.ack(100.28, 100, 191, MAV_RESULT_ACCEPTED, 100.225);
  require(rig.reported("shot-1", "ACK_OBSERVED") && !rig.reported("shot-1", "CONFIRMED"),
          "ACK reported capture without image evidence");
  rig.image(100.29, 7, 301);
  rig.image(100.30, 8, 300);
  rig.image(100.31, 8, 302, 101);
  rig.image(100.32, 8, 302, 100, 0);
  rig.image(100.33, 9, 302);
  require(!rig.reported("shot-1", "CONFIRMED"), "wrong image evidence completed capture");
  rig.image(100.34, 8, 302);
  require(rig.reported("shot-1", "CONFIRMED") &&
          rig.host.results.back().find("\"observed_image_index\":8") != std::string::npos &&
          rig.host.finished == std::vector<std::string>{"camera/shot-1"}, "new image did not confirm capture");
  rig.image(100.35, 8, 302);
  rig.mail("shot-1", 100.36);
  rig.mail("no-advance", 100.37);
  require(rig.reported("shot-1", "REJECTED", "duplicate") &&
          rig.reported("no-advance", "REJECTED", "new_count") && rig.host.requests.size() == 1,
          "duplicate request or consumed image baseline could be sent");

  rig.status(100.39, 9, 310);
  rig.mail("shot-2", 100.4);
  require(rig.host.requests.size() == 2 && rig.host.requests.back().capture_sequence == 2,
          "new count did not admit a fresh shot");
  rig.sent(100.41);
  rig.image(100.42, 9, 311);
  require(rig.reported("shot-2", "CONFIRMED") && !rig.reported("shot-2", "ACK_OBSERVED"),
          "bound image did not complete without an ACK");
}

void failureAndCancellation(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  rig.ready(100.1, 0);
  rig.mail("trimmed-count", 100.2);
  require(rig.reported("trimmed-count", "REJECTED", "new_count") && rig.host.requests.empty(),
          "omitted count was treated as a known zero");
  rig.status(100.3, 3, 400, 1);
  rig.mail("busy", 100.31);
  rig.status(100.32, 3, 401);
  rig.host.bench = false;
  rig.mail("unowned", 100.33);
  rig.host.bench = true;
  rig.mail("stale", 100.34, "capture", 2.1);
  require(rig.reported("busy", "REJECTED") && rig.reported("unowned", "REJECTED", "owned_disarmed") &&
          rig.reported("stale", "REJECTED", "stale") && rig.host.requests.empty(),
          "busy, stale or unowned capture escaped checks");
  rig.mail("queued", 100.4);
  rig.mail("queued", 100.41, "cancel");
  rig.sent(100.42);
  require(rig.reported("queued", "CANCELLED") && !rig.reported("queued", "SENT") &&
          rig.host.cancelled.back() == "camera/queued", "queued cancellation lost its host identity");
  rig.mail("changed", 100.43);
  rig.status(100.44, 3, 402, 1);
  require(rig.reported("changed", "CANCELLED", "state_changed"), "camera busy state did not retire waiting work");
  rig.status(100.45, 3, 403);
  rig.mail("sent", 100.46);
  rig.sent(100.47);
  rig.mail("sent", 100.48, "cancel");
  rig.image(100.49, 3, 404);
  rig.mail("unsafe-repeat", 100.5);
  require(rig.reported("sent", "CANCELLED") && !rig.reported("sent", "CONFIRMED") &&
          rig.reported("unsafe-repeat", "REJECTED", "new_count"),
          "cancelled sent effect reused the old camera count");

  rig.status(100.51, 4, 405);
  rig.mail("lost-image", 100.52);
  rig.sent(100.53);
  rig.ack(100.54);
  for (int at = 101; at <= 107; ++at) {
    rig.heartbeat(at);
    rig.plugin->iterate();
  }
  require(rig.reported("lost-image", "UNCERTAIN", "capture_evidence_deadline") &&
          !rig.reported("lost-image", "CONFIRMED") && rig.host.requests.size() == 4,
          "missing image did not expire without a retry");

  rig.ready(107.1, 5, 500);
  rig.mail("disconnected", 107.2);
  rig.sent(107.3);
  const auto previous_sequence = rig.host.requests.back().capture_sequence;
  rig.host.time = 107.4;
  rig.plugin->onReset("fixture_disconnected");
  rig.ready(107.5, 5, 501);
  rig.mail("after-reset", 107.6);
  require(rig.reported("disconnected", "UNCERTAIN", "session_lost") &&
          rig.reported("after-reset", "REJECTED", "new_count"), "reset allowed ambiguous capture retry");
  rig.status(107.65, 6, 502);
  rig.mail("new-after-reset", 107.7);
  require(rig.host.requests.back().capture_sequence == previous_sequence + 1,
          "new camera count lost the sequence across connection reset");
  rig.sent(107.8);
  rig.ack(107.9, 100, 191, MAV_RESULT_DENIED);
  require(rig.reported("new-after-reset", "ACK_OBSERVED") && !rig.reported("new-after-reset", "REJECTED"),
          "an unbound camera denial falsely rejected the active capture");
  rig.image(107.91, 6, 503);
  require(rig.reported("new-after-reset", "CONFIRMED"), "unbound denial overruled the new image");
}

void replayAndBenchLoss(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  rig.ready(100.1, 5);
  rig.heartbeat(101.3);
  rig.heartbeat(102.2);
  rig.receive(fixture::information(rig.codec), 102.21);
  rig.status(102.22, 5, 300);
  rig.mail("replayed-idle", 102.23);
  require(rig.reported("replayed-idle", "REJECTED", "new_count") && rig.host.requests.empty(),
          "replayed status renewed the usable camera count");
  rig.status(102.24, 5, 310);
  rig.mail("bench-lost", 102.25);
  rig.sent(102.26);
  rig.host.bench = false;
  rig.ack(102.27);
  rig.host.bench = true;
  rig.image(102.28, 5, 311);
  require(rig.reported("bench-lost", "UNCERTAIN", "payload_bench_lost") &&
          !rig.reported("bench-lost", "CONFIRMED") && rig.host.requests.size() == 1,
          "lost bench eligibility preserved completion rights");
}
} // namespace

int main(int argc, char** argv)
{
  if (argc != 2) return 2;
  void* library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  if (!library) { std::cerr << dlerror() << '\n'; return 2; }
  try {
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(library, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(library, "mavmoos_destroy_plugin"));
    require(create && destroy, "loaded module has no camera factory");
    confirmation(create, destroy);
    failureAndCancellation(create, destroy);
    replayAndBenchLoss(create, destroy);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    dlclose(library);
    return 1;
  }
  dlclose(library);
  std::cout << "Camera capture DSO qualified for bounded requests, paired evidence, cancellation and loss\n";
}

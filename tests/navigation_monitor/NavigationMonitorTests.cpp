#include "NavigationFrames.h"
#include "Plugin.h"
#include <dlfcn.h>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
using namespace mavmoos::runtime;
namespace packets = mavmoos::test::navigation_frames;

namespace {
void require(bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}
struct Host final : Services {
  double time = 100;
  Vehicle state;
  std::map<std::string, std::string> posts;
  double now() const override { return time; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key, const std::string& value) override { posts[key] = value; }
  void publish(const std::string&, double) override { effect(); }
  bool sendCommand(const CommandRequest&) override { effect(); }
  bool acquireNavigation(const std::string&) override { effect(); }
  void revokeNavigation(const std::string&) override { effect(); }
  bool navigation(const NavigationTarget&) override { effect(); }
  bool parameter(const ParameterRequest&) override { effect(); }
  bool mission(const MissionRequest&) override { effect(); }
  [[noreturn]] static void effect() { throw std::runtime_error("observer attempted an effect"); }
};
const std::string ctrl = "MAVMOOS_NAV_CONTROLLER_OUTPUT";
const std::string local = "MAVMOOS_POSITION_TARGET_LOCAL_NED";
const std::string global = "MAVMOOS_POSITION_TARGET_GLOBAL_INT";

struct Rig {
  Host host;
  std::unique_ptr<Plugin,DestroyPlugin> plugin;
  mavlink_status_t codec{};
  Rig(CreatePlugin make, DestroyPlugin destroy) : plugin(make(plugin_abi, &host), destroy)
  {
    require(bool(plugin), "factory");
  }
  void heartbeat(double at, unsigned autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA)
  {
    host.state.heartbeat_valid = true;
    host.state.heartbeat_at = at;
    host.state.type = MAV_TYPE_QUADROTOR;
    host.state.autopilot = autopilot;
    mavlink_message_t m{};
    mavlink_msg_heartbeat_pack_status(1, 1, &codec, &m, host.state.type, autopilot, 0, 0, MAV_STATE_ACTIVE);
    receive(m, at);
  }
  void receive(const mavlink_message_t& message, double read, double floor = 0)
  {
    host.time = read;
    plugin->onMessageWithMetadata(message, {floor ? floor : read, read, floor ? "QUEUE_EMPTY_FLOOR" : "READ_ACQUISITION"});
  }
  void tick(double at) { host.time = at; host.posts.clear(); plugin->iterate(); }
  bool has(const std::string& key, const char* expected) const
  {
    const auto item = host.posts.find(key);
    return item != host.posts.end() && item->second.find(expected) != std::string::npos;
  }
};
}

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "module path");
    void* lib = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    require(lib, "cannot load navigation monitor module");
    const auto make = reinterpret_cast<CreatePlugin>(dlsym(lib,"mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(lib,"mavmoos_destroy_plugin"));
    require(make && destroy && !make(plugin_abi+1,nullptr), "invalid factory");
    unsigned count = 0;
    auto test = [&](const char* label, const std::function<void(Rig&)>& f) {
      Rig r(make,destroy);f(r);++count;std::cout << "PASS " << label << '\n';
    };
    test("no egress, control owner or stream requests", [](Rig& r) {
      const auto desc = r.plugin->descriptor();
      require(r.plugin->name()=="navigation_monitor" && !r.plugin->controlBackend() &&
        r.plugin->subscriptions().empty() && desc.observation_streams.empty() &&
        desc.request_owners.empty() && desc.capabilities.size()==3, "not a passive composition");
    });
    test("distinct packets, masks and coordinates", [](Rig& r) {
      r.heartbeat(100.1);
      const auto g = packets::global(r.codec);
      require(g.len < MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT_LEN, "fixture not trimmed");
      r.receive(packets::controller(r.codec),100.2,100.19);
      r.receive(packets::local(r.codec),100.3);
      r.receive(g,100.4);
      r.tick(100.5);
      require(r.has(ctrl,"\"nav_bearing_reported_deg\":-115") &&
              r.has(ctrl,"\"cross_track_error_reported_m\":-2.75"), "controller output fields");
      require(r.has(local,"\"time_boot_ms_raw\":4294967295") && r.has(local,"\"position_xyz_m\":[1.25,null,-3.5]") &&
              r.has(local,"\"velocity_xyz_m_s\":[5,-1.25,null]") && r.has(local,"\"acceleration_or_force\":\"force_n\"") &&
              r.has(local,"\"yaw_rate_rad_s\":null"), "local mask or frame interpretation");
      require(r.has(global,"\"longitude_e7_raw\":-710875890") && r.has(global,"\"coordinate_frame\":\"global_msl\"") &&
              r.has(global,"\"mask_unassigned_bits\":61440") && r.has(global,"\"velocity_xyz_m_s\":[null,null,null]"),
              "global decode of valid V2 trimming");
      r.receive(packets::global(r.codec,false,222),100.55);
      r.tick(100.65);
      require(r.has(global,"\"coordinate_frame_raw\":222,\"coordinate_frame\":null") &&
              r.has(global,"\"longitude_deg\":null"), "unsupported frame given geographic interpretation");
    });
    test("source rejection, malformed active fields and ordering", [](Rig& r) {
      r.receive(packets::controller(r.codec),100.1);
      r.heartbeat(100.2);
      r.receive(packets::controller(r.codec,2,1),100.3);
      r.receive(packets::controller(r.codec,1,24),100.4);
      r.tick(100.5);
      require(r.has(ctrl,"\"present\":false") && r.has(ctrl,"\"source_rejected\":3"), "wrong producer admitted");
      r.receive(packets::local(r.codec),100.6);
      r.receive(packets::local(r.codec,true),100.7);
      r.receive(packets::global(r.codec,true),100.8);
      r.receive(packets::local(r.codec),100.9,100.59);
      auto invalid = packets::controller(r.codec);
      invalid.len = 0;
      r.receive(invalid,100.91);
      r.tick(101);
      require(r.has(local,"\"observation_id\":1,") && r.has(local,"\"invalid_samples\":1") &&
              r.has(local,"\"order_rejected\":1") && r.has(global,"\"present\":false") &&
              r.has(global,"\"invalid_samples\":1") && r.has(ctrl,"\"invalid_samples\":1"),
              "rejected packet renewed or replaced a sample");
    });
    test("each original timestamp ages and resets, including PX4-shaped source", [](Rig& r) {
      require(r.plugin->configureModule({{"sample_timeout","0.5"},{"publish_interval","0.05"}}).status==
              ModuleConfigStatus::Accepted, "configure");
      r.heartbeat(100.1,MAV_AUTOPILOT_PX4);
      r.receive(packets::controller(r.codec),100.2);
      r.receive(packets::global(r.codec),100.6);
      r.tick(100.8);
      require(r.has(ctrl,"\"present\":true,\"fresh\":false") && r.has(global,"\"present\":true,\"fresh\":true") &&
              r.has(global,"\"autopilot\":12"), "mixed observation age or identity");
      r.host.time = 100.9;
      r.plugin->onReset("owned route reset");
      r.receive(packets::global(r.codec),101);
      r.tick(101.1);
      require(r.has(ctrl,"\"session_epoch\":2") && r.has(global,"\"data\":null"), "old session revived without heartbeat");
      r.heartbeat(101.2,MAV_AUTOPILOT_PX4);
      r.receive(packets::global(r.codec),101.3);
      r.tick(101.4);
      require(r.has(global,"\"present\":true,\"fresh\":true"), "no rebind");
    });
    dlclose(lib);
    std::cout << "PASS " << count << " navigation monitor groups\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

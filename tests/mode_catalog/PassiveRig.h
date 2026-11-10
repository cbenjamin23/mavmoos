#pragma once

#include "Plugin.h"
#include <dlfcn.h>

#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string_view>

namespace mavmoos::test {

using namespace mavmoos::runtime;

inline void require(bool valid, std::string_view what)
{
  if (!valid) throw std::runtime_error(std::string(what));
}

struct PassiveHost final : Services {
  double clock = 100;
  Vehicle state;
  std::map<std::string, std::string> posts;

  double now() const override { return clock; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key, const std::string& value) override
  {
    require(posts.emplace(key, value).second, "duplicate publication");
  }
  void publish(const std::string&, double) override { effect(); }
  bool sendCommand(const CommandRequest&) override { effect(); }
  bool acquireNavigation(const std::string&) override { effect(); }
  void revokeNavigation(const std::string&) override { effect(); }
  bool navigation(const NavigationTarget&) override { effect(); }
  bool parameter(const ParameterRequest&) override { effect(); }
  bool mission(const MissionRequest&) override { effect(); }
  bool flightLog(const FlightLogRequest&) override { effect(); }

  [[noreturn]] static void effect() { throw std::runtime_error("passive observer attempted an effect"); }
};

class PassiveRig {
public:
  explicit PassiveRig(const char* path)
    : m_library(dlopen(path, RTLD_NOW | RTLD_LOCAL), &dlclose)
  {
    require(bool(m_library), "plugin load failed");
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(m_library.get(), "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(m_library.get(), "mavmoos_destroy_plugin"));
    require(create && destroy && !create(plugin_abi - 1, &host) && !create(plugin_abi, nullptr),
            "plugin ABI or services guard failed");
    plugin = std::unique_ptr<Plugin, DestroyPlugin>(create(plugin_abi, &host), destroy);
    require(bool(plugin) && plugin->subscriptions().empty(), "passive plugin requires unexpected MOOS input");
    const auto descriptor = plugin->descriptor();
    require(descriptor.request_owners.empty() && descriptor.observation_streams.empty() &&
            !plugin->controlBackend(), "passive plugin owns an active service");
    for (const auto& capability : descriptor.capabilities)
      require(capability.access == "observation" && capability.request_variable.empty(),
              "passive plugin declares an active capability");
  }

  template<typename Packet, typename Encoder>
  mavlink_message_t encode(const Packet& packet, Encoder pack, uint8_t system = 1, uint8_t component = 1)
  {
    mavlink_message_t message{};
    pack(system, component, &m_codec, &message, &packet);
    return message;
  }

  void receive(const mavlink_message_t& message, double at, double read = -1)
  {
    host.clock = read < 0 ? at : read;
    plugin->onMessageWithMetadata(message, {at, host.clock,
      at == host.clock ? "READ_ACQUISITION" : "QUEUE_EMPTY_FLOOR"});
  }

  void heartbeat(double at, uint32_t mode = 17, uint8_t flags = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
  {
    host.state.heartbeat_valid = true;
    host.state.heartbeat_at = at;
    host.state.type = MAV_TYPE_QUADROTOR;
    host.state.autopilot = MAV_AUTOPILOT_PX4;
    host.state.mode = mode;
    mavlink_heartbeat_t heartbeat{};
    heartbeat.type = host.state.type;
    heartbeat.autopilot = host.state.autopilot;
    heartbeat.base_mode = flags;
    heartbeat.custom_mode = mode;
    receive(encode(heartbeat, mavlink_msg_heartbeat_encode_status), at);
  }

  void capture(const char* phase, double at, unsigned count)
  {
    host.clock = at;
    host.posts.clear();
    plugin->iterate();
    require(host.posts.size() == count, "unexpected number of status publications");
    std::cout << "{\"phase\":\"" << phase << "\",\"posts\":{";
    bool first = true;
    for (const auto& [key, value] : host.posts) {
      if (!first) std::cout << ',';
      first = false;
      std::cout << '"' << key << "\":" << value;
    }
    std::cout << "}}\n";
  }

  PassiveHost host;

private:
  std::unique_ptr<void, int(*)(void*)> m_library;
  mavlink_status_t m_codec{};

public:
  std::unique_ptr<Plugin, DestroyPlugin> plugin{nullptr, nullptr};
};

} // namespace mavmoos::test

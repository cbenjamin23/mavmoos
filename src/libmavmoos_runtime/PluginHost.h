#pragma once
#include "Plugin.h"
#include "FileTransferService.h"
#include "PayloadService.h"
#include "ExternalNavigationService.h"
#include <memory>
#include <map>

namespace mavmoos::runtime {
// Startup-only composition of trusted modules built against this exact C++ ABI.
// The host owns every plugin object; feature code remains resident until process exit.
class PluginHost final {
public:
  PluginHost();
  ~PluginHost();
  PluginHost(const PluginHost&)=delete;
  PluginHost& operator=(const PluginHost&)=delete;
  bool load(const std::string& path, Services& services,const std::string& expected_name="");
  const std::string& lastError() const { return m_error; }
  std::vector<Plugin*> plugins() const;
  bool hasCapability(const std::string&) const;
  bool streamedParameterBenchCompatible() const;
  const VehicleControlBackend* controlBackend() const;
  bool projectGlobal(double,double,double&,double&) const;
  std::vector<ObservationStream> observationStreams() const;
  std::string catalog() const;
  std::vector<std::string> subscriptions() const;
  void configure(const std::string& key,const std::string& value);
  bool configureModules(const std::map<std::string,std::vector<ModuleSetting>>& batches);
  void dispatchMail(const Mail& mail);
  void dispatchMessage(const mavlink_message_t& message,double received_at);
  void dispatchMessage(const mavlink_message_t& message,const MessageTime& time);
  void parameterSent(const ParameterRequest&,double sent_at,bool accepted);
  void missionSent(const MissionRequest&,double sent_at,bool accepted);
  void flightLogSent(const FlightLogRequest&,double sent_at,bool accepted);
  void fileTransferSent(const FileTransferRequest&,double sent_at,bool accepted);
  void payloadSent(const PayloadRequest&,double sent_at,bool accepted);
  void externalNavigationSent(const ExternalNavigationSample&,double,bool);
  void commandSent(const CommandRequest&,double sent_at,bool accepted);
  void inputBoundary();
  void reset(const std::string& reason);
  void iterate();
  void clear();
private:
  struct Entry;
  std::vector<std::unique_ptr<Entry>> m_entries;
  std::string m_error;
};
} // namespace mavmoos::runtime

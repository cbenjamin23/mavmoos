/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: LinkStatusPlugin.cpp                            */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "PassiveObserver.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <map>
#include <string_view>

namespace mavmoos::runtime {
namespace {

constexpr std::size_t modem_limit = 8;

template<typename Number>
void writeKnown(std::ostream& out, Number value, bool known)
{
  if (known)
    out << +value;
  else
    out << "null";
}

void writeRadio(std::ostream& out, const mavlink_radio_status_t& value)
{
  out << "{\"signal_scale\":\"device_dependent\",\"rssi_raw\":" << unsigned(value.rssi)
      << ",\"remrssi_raw\":" << unsigned(value.remrssi) << ",\"noise_raw\":" << unsigned(value.noise)
      << ",\"remnoise_raw\":" << unsigned(value.remnoise) << ",\"rssi_device_units\":";
  writeKnown(out, value.rssi, value.rssi != UINT8_MAX);
  out << ",\"remrssi_device_units\":";
  writeKnown(out, value.remrssi, value.remrssi != UINT8_MAX);
  out << ",\"noise_device_units\":";
  writeKnown(out, value.noise, value.noise != UINT8_MAX);
  out << ",\"remnoise_device_units\":";
  writeKnown(out, value.remnoise, value.remnoise != UINT8_MAX);
  out << ",\"tx_buffer_free_pct_raw\":" << unsigned(value.txbuf) << ",\"tx_buffer_free_pct\":";
  writeKnown(out, value.txbuf, value.txbuf <= 100);
  out << ",\"rx_errors_raw\":" << value.rxerrors << ",\"corrected_packets_raw\":" << value.fixed << '}';
}

bool finiteExtensions(const mavlink_cellular_status_t& value)
{
  return std::isfinite(value.band_frequency) && std::isfinite(value.rx_level) && std::isfinite(value.tx_level) &&
    std::isfinite(value.rx_quality) && std::isfinite(value.sinr);
}

void writeCellTower(std::ostream& out, std::string_view raw, bool transmitted)
{
  out << ",\"cell_tower_id_raw\":";
  if (transmitted) {
    out << '[';
    for (std::size_t i = 0; i < raw.size(); ++i) {
      if (i) out << ',';
      out << unsigned(static_cast<unsigned char>(raw[i]));
    }
    out << ']';
  } else {
    out << "null";
  }
  const auto text = raw.substr(0, raw.find('\0'));
  const bool printable = !text.empty() && std::all_of(text.begin(), text.end(), [](unsigned char value) {
    return value >= 0x20 && value <= 0x7e;
  });
  out << ",\"cell_tower_id\":";
  if (transmitted && printable)
    out << std::quoted(std::string(text));
  else
    out << "null";
}

void writeCellular(std::ostream& out, const mavlink_cellular_status_t& value, unsigned length)
{
  out << "{\"status_raw\":" << unsigned(value.status)
      << ",\"failure_reason_raw\":" << unsigned(value.failure_reason)
      << ",\"failure_reason_applicable\":" << (value.status == CELLULAR_STATUS_FLAG_FAILED)
      << ",\"network_type_raw\":" << unsigned(value.type)
      << ",\"quality_pct_raw\":" << unsigned(value.quality) << ",\"quality_pct\":";
  writeKnown(out, value.quality, value.quality <= 100);
  out << ",\"mcc_raw\":" << value.mcc << ",\"mnc_raw\":" << value.mnc << ",\"lac_raw\":" << value.lac << ",\"mcc\":";
  writeKnown(out, value.mcc, value.mcc != UINT16_MAX);
  out << ",\"mnc\":";
  writeKnown(out, value.mnc, value.mnc != UINT16_MAX);
  out << ",\"lac\":";
  writeKnown(out, value.lac, value.lac != 0);
  // These optional fields use zero as an unavailable value. Decode partial
  // fields with MAVLink 2 zero-padding before applying the sentinel.
  out << ",\"link_tx_rate_kib_s\":";
  writeKnown(out, value.link_tx_rate, value.link_tx_rate != 0);
  out << ",\"link_rx_rate_kib_s\":";
  writeKnown(out, value.link_rx_rate, value.link_rx_rate != 0);
  writeCellTower(out, {value.cell_tower_id, sizeof(value.cell_tower_id)},
                 length > offsetof(mavlink_cellular_status_t, cell_tower_id));
  out << ",\"band_number\":";
  writeKnown(out, value.band_number, value.band_number != 0);
  out << ",\"band_frequency_mhz\":";
  writeKnown(out, value.band_frequency, value.band_frequency != 0);
  out << ",\"channel_number\":";
  writeKnown(out, value.channel_number, value.channel_number != 0);
  out << ",\"rx_level_dbm\":";
  writeKnown(out, value.rx_level, value.rx_level != 0);
  out << ",\"tx_level_dbm\":";
  writeKnown(out, value.tx_level, value.tx_level != 0);
  out << ",\"rx_quality_raw\":";
  writeKnown(out, value.rx_quality, value.rx_quality != 0);
  out << ",\"sinr_db\":";
  writeKnown(out, value.sinr, value.sinr != 0);
  out << '}';
}

class LinkStatus final : public PassiveObserver {
public:
  explicit LinkStatus(Services& services) : PassiveObserver(services, true) {}

  std::string name() const override { return "link_status"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "mavlink_link_status_observations";
    result.capabilities = {{"link_status.radio_status", "", "", "observation"},
                           {"link_status.cellular_status", "", "", "observation"}};
    result.exclusive_resources = {"observation.link_status"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

private:
  template<typename Payload>
  struct Sample {
    Stamp stamp;
    unsigned wire_version = 0;
    unsigned length = 0;
    Payload payload{};
  };

  bool observes(uint32_t id) const override
  {
    return id == MAVLINK_MSG_ID_RADIO_STATUS || id == MAVLINK_MSG_ID_CELLULAR_STATUS;
  }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    if (message.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
      if (!wireShape(message, MAVLINK_MSG_ID_RADIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_RADIO_STATUS_LEN)) {
        rejectWire();
        ++m_radio_invalid;
      } else if (update(m_radio.stamp, time)) {
        m_radio.wire_version = message.magic == MAVLINK_STX ? 2u : 1u;
        m_radio.length = message.len;
        mavlink_msg_radio_status_decode(&message, &m_radio.payload);
      }
      return;
    }

    mavlink_cellular_status_t value{};
    const bool shape = wireShape(message, MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CELLULAR_STATUS_LEN);
    if (shape)
      mavlink_msg_cellular_status_decode(&message, &value);
    if (!shape || !finiteExtensions(value)) {
      rejectWire();
      ++m_cellular_invalid;
      return;
    }

    auto modem = m_modems.find(value.id);
    if (modem == m_modems.end()) {
      if (m_modems.size() >= modem_limit) {
        ++m_capacity_drops;
        return;
      }
      modem = m_modems.try_emplace(value.id).first;
    }
    if (update(modem->second.stamp, time)) {
      modem->second.wire_version = 2;
      modem->second.length = message.len;
      modem->second.payload = value;
    }
  }

  void clearSamples() override
  {
    m_radio = {};
    m_modems.clear();
  }

  void publishSamples() override
  {
    auto radio_meta = metadata(m_radio.stamp);
    radio_meta.pop_back();
    std::ostringstream radio;
    radio.imbue(std::locale::classic());
    radio << radio_meta << ",\"family\":\"RADIO_STATUS\",\"message_id\":109,\"invalid_samples\":" << m_radio_invalid
          << ",\"wire_version\":";
    writeKnown(radio, m_radio.wire_version, m_radio.stamp.id != 0);
    radio << ",\"payload_length\":";
    writeKnown(radio, m_radio.length, m_radio.stamp.id != 0);
    radio << ",\"data\":";
    if (m_radio.stamp.id)
      writeRadio(radio, m_radio.payload);
    else
      radio << "null";
    radio << '}';
    services().publish("MAVMOOS_RADIO_STATUS", radio.str());

    std::ostringstream cellular;
    cellular.imbue(std::locale::classic());
    cellular << std::setprecision(std::numeric_limits<float>::max_digits10) << std::boolalpha
             << "{\"schema\":1,\"family\":\"CELLULAR_STATUS\",\"message_id\":334,\"present\":" << !m_modems.empty()
             << ",\"read_only\":true,\"authoritative_for_control\":false,\"atomic\":false"
             << ",\"modem_limit\":" << modem_limit << ",\"invalid_samples\":" << m_cellular_invalid
             << ",\"capacity_drops\":" << m_capacity_drops << ",\"modems\":[";
    bool first = true;
    for (const auto& [id, sample] : m_modems) {
      if (!first) cellular << ',';
      first = false;
      cellular << "{\"modem_id\":" << id << ",\"wire_version\":" << sample.wire_version
               << ",\"payload_length\":" << sample.length
               << ",\"metadata\":" << metadata(sample.stamp) << ",\"data\":";
      writeCellular(cellular, sample.payload, sample.length);
      cellular << '}';
    }
    cellular << "]}";
    services().publish("MAVMOOS_CELLULAR_STATUS", cellular.str());
  }

  Sample<mavlink_radio_status_t> m_radio;
  std::map<unsigned, Sample<mavlink_cellular_status_t>> m_modems;
  uint64_t m_radio_invalid = 0;
  uint64_t m_cellular_invalid = 0;
  uint64_t m_capacity_drops = 0;
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services)
{
  return abi == mavmoos::runtime::plugin_abi && services ? new mavmoos::runtime::LinkStatus(*services) : nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}

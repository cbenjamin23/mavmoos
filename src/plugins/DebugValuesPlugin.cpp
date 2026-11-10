/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: DebugValuesPlugin.cpp                           */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "PassiveObserver.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>

namespace mavmoos::runtime {
namespace {

constexpr std::size_t sample_limit = 8;

struct Wire {
  unsigned version;
  unsigned length;
};

template<typename Payload>
std::string labelOf(const Payload& value)
{
  const auto end = std::find(std::begin(value.name), std::end(value.name), '\0');
  return {std::begin(value.name), end};
}

template<typename Payload>
std::string keyOf(const Payload& value) { return labelOf(value); }

uint8_t keyOf(const mavlink_debug_t& value) { return value.ind; }
uint16_t keyOf(const mavlink_debug_float_array_t& value) { return value.array_id; }

void writeKey(std::ostream& out, uint8_t index, const mavlink_debug_t&)
{
  out << "\"index\":" << unsigned(index);
}

void writeName(std::ostream& out, const std::string& name)
{
  constexpr char hex[] = "0123456789abcdef";
  out << "\"name_raw_hex\":\"";
  for (const unsigned char byte : name)
    out << hex[byte >> 4] << hex[byte & 15];
  const bool printable = std::all_of(name.begin(), name.end(), [](unsigned char byte) {
    return byte >= 32 && byte <= 126;
  });
  out << "\",\"name_state\":\"" << (name.empty() ? "empty" : printable ? "text" : "non_ascii")
      << "\",\"name\":";
  if (name.empty() || !printable) {
    out << "null";
    return;
  }
  out << '"';
  for (const char byte : name) {
    if (byte == '"' || byte == '\\') out << '\\';
    out << byte;
  }
  out << '"';
}

template<typename Payload>
void writeKey(std::ostream& out, const std::string& name, const Payload&) { writeName(out, name); }

void writeKey(std::ostream& out, uint16_t id, const mavlink_debug_float_array_t& value)
{
  out << "\"array_id\":" << id << ',';
  writeName(out, labelOf(value));
}

const char* valueState(float value)
{
  if (std::isnan(value)) return "wire_nan";
  if (std::isinf(value)) return std::signbit(value) ? "wire_negative_infinity" : "wire_positive_infinity";
  return "reported";
}

void writeValue(std::ostream& out, float value)
{
  if (!std::isfinite(value)) out << "null";
  else if (value == 0 && std::signbit(value)) out << "-0.0";
  else out << value;
}

void writeData(std::ostream& out, const mavlink_debug_vect_t& value, const Wire&)
{
  out << "{\"time_usec_raw\":\"" << value.time_usec
      << "\",\"remote_clock\":\"unmapped_boot_or_unix_us\",\"xyz\":[";
  writeValue(out, value.x);
  out << ',';
  writeValue(out, value.y);
  out << ',';
  writeValue(out, value.z);
  out << "],\"xyz_states\":[\"" << valueState(value.x) << "\",\"" << valueState(value.y)
      << "\",\"" << valueState(value.z) << "\"]}";
}

void writeData(std::ostream& out, const mavlink_debug_float_array_t& value, const Wire& wire)
{
  constexpr unsigned base_length = MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN;
  constexpr unsigned elements = MAVLINK_MSG_DEBUG_FLOAT_ARRAY_FIELD_DATA_LEN;
  const auto extension_bytes = wire.length > base_length ? wire.length - base_length : 0u;
  const auto wireBytes = [extension_bytes](unsigned index) {
    constexpr unsigned element_size = 4;
    const auto offset = index * element_size;
    return offset < extension_bytes ? std::min(element_size, extension_bytes - offset) : 0u;
  };
  out << "{\"time_usec_raw\":\"" << value.time_usec
      << "\",\"remote_clock\":\"unmapped_boot_or_unix_us\",\"extension_payload_bytes\":" << extension_bytes
      << ",\"values\":[";
  for (unsigned index = 0; index < elements; ++index) {
    if (index) out << ',';
    writeValue(out, value.data[index]);
  }
  out << "],\"element_wire_bytes\":[";
  for (unsigned index = 0; index < elements; ++index) {
    if (index) out << ',';
    out << wireBytes(index);
  }
  out << "],\"value_states\":[";
  for (unsigned index = 0; index < elements; ++index) {
    if (index) out << ',';
    out << '"' << (wireBytes(index) ? valueState(value.data[index]) : "zero_padded") << '"';
  }
  out << "]}";
}

void writeScalar(std::ostream& out, float value)
{
  writeValue(out, value);
  out << ",\"value_state\":\"" << valueState(value) << '"';
}

void writeScalar(std::ostream& out, int32_t value)
{
  out << value << ",\"value_state\":\"reported\"";
}

template<typename Payload>
void writeData(std::ostream& out, const Payload& value, const Wire&)
{
  out << "{\"time_boot_ms_raw\":" << value.time_boot_ms
      << ",\"remote_clock\":\"unmapped_boot_ms\",\"value\":";
  writeScalar(out, value.value);
  out << '}';
}

class DebugValues final : public PassiveObserver {
public:
  explicit DebugValues(Services& services) : PassiveObserver(services, true) {}

  std::string name() const override { return "debug_values"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "mavlink_debug_value_observations";
    result.capabilities = {{"debug_values.debug", "", "", "observation"},
                           {"debug_values.debug_vect", "", "", "observation"},
                           {"debug_values.debug_float_array", "", "", "observation"},
                           {"debug_values.named_value_float", "", "", "observation"},
                           {"debug_values.named_value_int", "", "", "observation"}};
    result.exclusive_resources = {"observation.debug_values"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

private:
  template<typename Payload>
  struct Sample {
    Stamp stamp;
    Wire wire{};
    Payload payload{};
  };

  template<typename Key, typename Payload>
  struct Family {
    std::map<Key, Sample<Payload>> samples;
    uint64_t invalid = 0;
    uint64_t capacity_drops = 0;
  };

  bool observes(uint32_t id) const override
  {
    return id == MAVLINK_MSG_ID_DEBUG || id == MAVLINK_MSG_ID_DEBUG_VECT ||
           id == MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY ||
           id == MAVLINK_MSG_ID_NAMED_VALUE_FLOAT || id == MAVLINK_MSG_ID_NAMED_VALUE_INT;
  }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    switch (message.msgid) {
    case MAVLINK_MSG_ID_DEBUG:
      accept(m_debug, message, time, MAVLINK_MSG_ID_DEBUG_LEN, mavlink_msg_debug_decode);
      break;
    case MAVLINK_MSG_ID_DEBUG_VECT:
      accept(m_vectors, message, time, MAVLINK_MSG_ID_DEBUG_VECT_LEN, mavlink_msg_debug_vect_decode);
      break;
    case MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY:
      accept(m_arrays, message, time, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN, mavlink_msg_debug_float_array_decode);
      break;
    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
      accept(m_floats, message, time, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN, mavlink_msg_named_value_float_decode);
      break;
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
      accept(m_ints, message, time, MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN, mavlink_msg_named_value_int_decode);
      break;
    }
  }

  template<typename Key, typename Payload>
  void accept(Family<Key, Payload>& family, const mavlink_message_t& message, const MessageTime& time,
              unsigned length, void (*decode)(const mavlink_message_t*, Payload*))
  {
    if (!wireShape(message, length, length)) {
      ++family.invalid;
      rejectWire();
      return;
    }
    Payload payload{};
    decode(&message, &payload);
    const auto key = keyOf(payload);
    auto sample = family.samples.find(key);
    if (sample == family.samples.end()) {
      if (family.samples.size() >= sample_limit) {
        ++family.capacity_drops;
        return;
      }
      sample = family.samples.try_emplace(key).first;
    }
    if (update(sample->second.stamp, time)) {
      sample->second.wire = {message.magic == MAVLINK_STX ? 2u : 1u, message.len};
      sample->second.payload = payload;
    }
  }

  void clearSamples() override
  {
    m_debug.samples.clear();
    m_vectors.samples.clear();
    m_arrays.samples.clear();
    m_floats.samples.clear();
    m_ints.samples.clear();
  }

  void publishSamples() override
  {
    publish(m_debug, "DEBUG", MAVLINK_MSG_ID_DEBUG);
    publish(m_vectors, "DEBUG_VECT", MAVLINK_MSG_ID_DEBUG_VECT);
    publish(m_arrays, "DEBUG_FLOAT_ARRAY", MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY);
    publish(m_floats, "NAMED_VALUE_FLOAT", MAVLINK_MSG_ID_NAMED_VALUE_FLOAT);
    publish(m_ints, "NAMED_VALUE_INT", MAVLINK_MSG_ID_NAMED_VALUE_INT);
  }

  template<typename Key, typename Payload>
  void publish(const Family<Key, Payload>& family, const char* name, unsigned id)
  {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<float>::max_digits10) << std::boolalpha
        << "{\"schema\":1,\"family\":\"" << name << "\",\"message_id\":" << id
        << ",\"present\":" << !family.samples.empty()
        << ",\"read_only\":true,\"authoritative_for_control\":false,\"atomic\":false"
        << ",\"sample_limit\":" << sample_limit << ",\"invalid_samples\":" << family.invalid
        << ",\"capacity_drops\":" << family.capacity_drops << ",\"samples\":[";
    bool first = true;
    for (const auto& [key, sample] : family.samples) {
      if (!first) out << ',';
      first = false;
      out << '{';
      writeKey(out, key, sample.payload);
      out << ",\"wire_version\":" << sample.wire.version << ",\"payload_length\":" << sample.wire.length
          << ",\"metadata\":" << metadata(sample.stamp) << ",\"data\":";
      writeData(out, sample.payload, sample.wire);
      out << '}';
    }
    out << "]}";
    services().publish(std::string("MAVMOOS_") + name, out.str());
  }

  Family<uint8_t, mavlink_debug_t> m_debug;
  Family<std::string, mavlink_debug_vect_t> m_vectors;
  Family<uint16_t, mavlink_debug_float_array_t> m_arrays;
  Family<std::string, mavlink_named_value_float_t> m_floats;
  Family<std::string, mavlink_named_value_int_t> m_ints;
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services)
{
  return abi == mavmoos::runtime::plugin_abi && services ? new mavmoos::runtime::DebugValues(*services) : nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}

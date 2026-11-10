#pragma once

#include <ardupilotmega/mavlink.h>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <string_view>

namespace mavmoos::test::camera {
constexpr uint8_t component = MAV_COMP_ID_CAMERA;

template<class Packet, class Encoder>
mavlink_message_t encode(mavlink_status_t& codec, const Packet& packet, Encoder encoder,
                         uint8_t system = 1, uint8_t source = component)
{
  mavlink_message_t frame{};
  encoder(system, source, &codec, &frame, &packet);
  return frame;
}

inline mavlink_camera_information_t informationPayload()
{
  mavlink_camera_information_t value{};
  value.time_boot_ms = 100;
  value.firmware_version = 0x04030201;
  value.focal_length = 12.5f;
  value.sensor_size_h = std::numeric_limits<float>::quiet_NaN();
  value.sensor_size_v = 4.25f;
  value.flags = 0x80000003U;
  value.resolution_h = 1280;
  value.cam_definition_version = 7;
  value.gimbal_device_id = 154;
  constexpr std::string_view vendor = "Blue\"Lab\n";
  constexpr std::string_view model = "abcdefghijklmnopqrstuvwxyz012345";
  constexpr std::string_view uri = "https://fixture.invalid/definition.xml";
  std::copy(vendor.begin(), vendor.end(), value.vendor_name);
  std::copy(model.begin(), model.end(), value.model_name);
  std::copy(uri.begin(), uri.end(), value.cam_definition_uri);
  return value;
}

inline mavlink_camera_settings_t settingsPayload()
{
  mavlink_camera_settings_t value{};
  value.time_boot_ms = 200;
  value.mode_id = CAMERA_MODE_VIDEO;
  value.focusLevel = std::numeric_limits<float>::quiet_NaN();
  return value;
}

inline mavlink_camera_capture_status_t capturePayload()
{
  mavlink_camera_capture_status_t value{};
  value.time_boot_ms = 300;
  value.image_interval = 2.5f;
  value.recording_time_ms = 1234;
  value.available_capacity = 128;
  value.image_status = 3;
  value.video_status = 1;
  value.image_count = 42;
  return value;
}

inline mavlink_storage_information_t storagePayload(uint8_t id = 1, uint8_t count = 2)
{
  mavlink_storage_information_t value{};
  value.time_boot_ms = 400 + id;
  value.storage_id = id;
  value.storage_count = count;
  value.status = STORAGE_STATUS_READY;
  value.type = STORAGE_TYPE_MICROSD;
  value.total_capacity = 1024;
  value.used_capacity = 128;
  value.available_capacity = 896;
  value.read_speed = 10.5f;
  value.write_speed = std::numeric_limits<float>::quiet_NaN();
  value.storage_usage = STORAGE_USAGE_FLAG_SET | STORAGE_USAGE_FLAG_PHOTO;
  constexpr std::string_view name = "Card\\\"A";
  std::copy(name.begin(), name.end(), value.name);
  return value;
}

inline mavlink_message_t information(mavlink_status_t& codec, uint8_t system = 1, uint8_t source = component)
{ return encode(codec, informationPayload(), mavlink_msg_camera_information_encode_status, system, source); }

inline mavlink_message_t settings(mavlink_status_t& codec, uint8_t system = 1, uint8_t source = component)
{ return encode(codec, settingsPayload(), mavlink_msg_camera_settings_encode_status, system, source); }

inline mavlink_message_t capture(mavlink_status_t& codec, uint8_t system = 1, uint8_t source = component)
{ return encode(codec, capturePayload(), mavlink_msg_camera_capture_status_encode_status, system, source); }

inline mavlink_message_t storage(mavlink_status_t& codec, uint8_t system = 1, uint8_t source = component)
{ return encode(codec, storagePayload(), mavlink_msg_storage_information_encode_status, system, source); }

inline mavlink_message_t excludedImage(mavlink_status_t& codec)
{
  mavlink_camera_image_captured_t value{};
  value.time_boot_ms = 500;
  value.capture_result = 1;
  constexpr std::string_view uri = "file:///ignored-image.jpg";
  std::copy(uri.begin(), uri.end(), value.file_url);
  return encode(codec, value, mavlink_msg_camera_image_captured_encode_status);
}
} // namespace mavmoos::test::camera

#include "CameraFrames.h"
#include <iomanip>
#include <iostream>

namespace {
void emit(const char* name, const mavlink_message_t& message)
{
  uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
  const auto size = mavlink_msg_to_send_buffer(bytes, &message);
  std::cout << name << ' ';
  for (unsigned i = 0; i < size; ++i)
    std::cout << std::hex << std::setfill('0') << std::setw(2) << unsigned(bytes[i]);
  std::cout << '\n';
}
}

int main()
{
  mavlink_status_t codec{};
  namespace camera = mavmoos::test::camera;
  for (unsigned i = 0; i < 100; ++i) {
    emit("camera_information", camera::information(codec));
    emit("camera_settings", camera::settings(codec));
    emit("camera_capture", camera::capture(codec));
    emit("camera_storage", camera::storage(codec));
    emit("camera_other_component", camera::information(codec, 1, 101));
    emit("camera_other_system", camera::information(codec, 2, 100));
    emit("camera_from_primary", camera::information(codec, 1, 1));
    emit("camera_image_excluded", camera::excludedImage(codec));
    mavlink_message_t heartbeat{};
    mavlink_msg_heartbeat_pack_status(1, 100, &codec, &heartbeat,
      MAV_TYPE_CAMERA, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    emit("camera_heartbeat", heartbeat);
  }
}

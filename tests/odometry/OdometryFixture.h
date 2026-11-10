#pragma once
#include <ardupilotmega/mavlink.h>
#include <algorithm>
#include <iterator>
#include <limits>

namespace mavmoos::test::odometry {

inline mavlink_local_position_ned_t localPayload()
{
  mavlink_local_position_ned_t value{};
  value.time_boot_ms = 0x12345678;
  value.x = 1.25;
  value.y = -2.5;
  value.z = -3.75;
  value.vx = -0.5;
  value.vy = 0.25;
  return value;
}

inline mavlink_odometry_t rawPayload()
{
  mavlink_odometry_t value{};
  value.time_usec = 9007199254740993ull;
  value.frame_id = MAV_FRAME_LOCAL_FRD;
  value.child_frame_id = MAV_FRAME_BODY_FRD;
  value.x = 8;
  value.y = -9;
  value.z = 10;
  value.q[0] = 0.5;
  value.q[1] = 0.5;
  value.q[2] = -0.5;
  value.q[3] = 0.5;
  value.vx = 1.25;
  value.vz = -1;
  value.rollspeed = 0.125;
  value.pitchspeed = -0.25;
  value.yawspeed = 0.375;
  value.pose_covariance[0] = std::numeric_limits<float>::quiet_NaN();
  for (unsigned i = 0; i < 21; ++i) value.velocity_covariance[i] = 0.25f + i * 0.125f;
  value.reset_counter = 7;
  value.estimator_type = MAV_ESTIMATOR_TYPE_VISION;
  value.quality = 87;
  return value;
}

inline mavlink_odometry_t px4RawPayload()
{
  auto value = rawPayload();
  value.frame_id = MAV_FRAME_LOCAL_NED;
  value.estimator_type = MAV_ESTIMATOR_TYPE_AUTOPILOT;
  const float unknown = std::numeric_limits<float>::quiet_NaN();
  // PX4 supplies position, orientation and linear velocity variances. The
  // off-diagonals and angular velocity variances are unknown.
  std::fill(std::begin(value.pose_covariance), std::end(value.pose_covariance), unknown);
  std::fill(std::begin(value.velocity_covariance), std::end(value.velocity_covariance), unknown);
  value.pose_covariance[0] = 0.25;
  value.pose_covariance[6] = 0.5;
  value.pose_covariance[11] = 0.75;
  value.pose_covariance[15] = 0.125;
  value.pose_covariance[18] = 0.25;
  value.pose_covariance[20] = 0.375;
  value.velocity_covariance[0] = 0.125;
  value.velocity_covariance[6] = 0.25;
  value.velocity_covariance[11] = 0.375;
  return value;
}

inline mavlink_gps_global_origin_t originPayload()
{
  mavlink_gps_global_origin_t value{};
  value.latitude = 423584560;
  value.longitude = -710875890;
  value.altitude = 12345;
  value.time_usec = 9007199254740993ull;
  return value;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_local_position_ned_t& payload,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_message_t message{};
  mavlink_msg_local_position_ned_encode_status(system, component, &codec, &message, &payload);
  return message;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_odometry_t& payload,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_message_t message{};
  mavlink_msg_odometry_encode_status(system, component, &codec, &message, &payload);
  return message;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_gps_global_origin_t& payload,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_message_t message{};
  mavlink_msg_gps_global_origin_encode_status(system, component, &codec, &message, &payload);
  return message;
}

} // namespace mavmoos::test::odometry

#include "air_data/AirDataFrames.h"
#include "esc_status/EscFrames.h"
#include "imu/VibrationFrames.h"
#include "odometry/OdometryFixture.h"
#include "status/DiagnosticFrames.h"
#include "traffic/TrafficFrames.h"

#include <iomanip>
#include <iostream>
#include <string>

namespace {
void frame(const mavlink_message_t& message,bool final=true)
{
  uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
  const auto length = mavlink_msg_to_send_buffer(bytes, &message);
  for (unsigned i = 0; i < length; ++i)
    std::cout << std::hex << std::setfill('0') << std::setw(2) << unsigned(bytes[i]);
  if(final) std::cout << '\n';
}

mavlink_message_t landingTarget(mavlink_status_t& codec, uint8_t system,
                               uint8_t component, uint8_t target)
{
  mavlink_landing_target_t value{};
  value.time_usec = 9007199254740995ULL;
  value.target_num = target;
  value.frame = MAV_FRAME_BODY_FRD;
  value.type = LANDING_TARGET_TYPE_VISION_FIDUCIAL;
  value.angle_x = .125f;
  value.angle_y = -.25f;
  value.distance = 3.5f;
  value.position_valid = 1;
  value.x = .75f;
  value.y = -1.25f;
  value.z = 3;
  value.q[0] = 1;
  mavlink_message_t message{};
  mavlink_msg_landing_target_encode_status(system, component, &codec, &message, &value);
  return message;
}
}

int main(int argc, char** argv)
{
  if (argc != 2) return 2;
  const std::string family = argv[1];
  if (family != "ardupilot" && family != "px4") return 2;
  const bool px4 = family == "px4";
  mavlink_status_t codec{};
  using namespace mavmoos::test;
  // Advance the wire sequence even when fixture payload values repeat.
  for (int batch = 0; batch < 96; ++batch) {
    mavlink_message_t heartbeat{};
    mavlink_msg_heartbeat_pack_status(1, 1, &codec, &heartbeat, MAV_TYPE_QUADROTOR,
                                     px4 ? MAV_AUTOPILOT_PX4 : MAV_AUTOPILOT_ARDUPILOTMEGA,
                                     MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_STANDBY);
    frame(heartbeat);
    frame(odometry::encode(codec, odometry::localPayload()));
    frame(odometry::encode(codec, px4 ? odometry::px4RawPayload() : odometry::rawPayload()));
    frame(odometry::encode(codec, odometry::originPayload()));
    if (!px4) {
      frame(air_data::vfrHud(codec));
      frame(air_data::wind(codec));
      frame(air_data::windCov(codec));
      frame(esc::info(codec));
      frame(esc::status(codec));
      frame(esc::telemetry(codec));
      frame(air_data::pressure(codec));
      frame(imu::vibration(codec));
      frame(landingTarget(codec, 1, 158, 42));
      frame(traffic::encode(codec, traffic::complete()));
      frame(traffic::encode(codec, traffic::partial()));
      // Keep the extra status families in one datagram to preserve the
      // observer matrix's bounded per-batch socket work.
      frame(diagnostics::encode(codec,diagnostics::power(),mavlink_msg_power_status_encode_status),false);
      frame(diagnostics::encode(codec,diagnostics::mcu(2),mavlink_msg_mcu_status_encode_status),false);
      frame(diagnostics::encode(codec,diagnostics::mcu(6),mavlink_msg_mcu_status_encode_status),false);
      frame(diagnostics::encode(codec,diagnostics::ekf(),mavlink_msg_ekf_status_report_encode_status),false);
      frame(diagnostics::encode(codec,diagnostics::estimator(),mavlink_msg_estimator_status_encode_status),false);
      mavlink_sys_status_t system{};
      system.load=125;
      frame(diagnostics::encode(codec,system,mavlink_msg_sys_status_encode_status));
      frame(landingTarget(codec, 2, 158, 98));
      frame(landingTarget(codec, 1, 159, 99));
      // This component may send a target observation only. Its heartbeat
      // must never replace or invalidate the flight controller's identity.
      mavlink_message_t sensor_heartbeat{};
      mavlink_msg_heartbeat_pack_status(1, 158, &codec, &sensor_heartbeat,
                                       MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_PX4,
                                       MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_STANDBY);
      frame(sensor_heartbeat);
      frame(traffic::encode(codec, traffic::complete(traffic::foreign_icao), 2));
      frame(traffic::encode(codec, traffic::complete(traffic::foreign_icao), 1, MAV_COMP_ID_ADSB));
      frame(traffic::encode(codec, traffic::complete(traffic::invalid_icao)));
      auto foreign_power=diagnostics::power();
      foreign_power.Vcc=9000;
      frame(diagnostics::encode(codec,foreign_power,mavlink_msg_power_status_encode_status,2,1),false);
      frame(diagnostics::encode(codec,diagnostics::mcu(99),mavlink_msg_mcu_status_encode_status,1,2));
      frame(diagnostics::encode(codec,diagnostics::power(),mavlink_msg_power_status_encode_status));
    }
  }
}

#pragma once

#include <ardupilotmega/mavlink.h>

namespace mavmoos::test::diagnostics {

template<class Payload>
mavlink_message_t encode(mavlink_status_t& codec,const Payload& value,
                         uint16_t (*pack)(uint32_t,uint8_t,mavlink_status_t*,mavlink_message_t*,const Payload*),
                         uint8_t system=1,uint8_t component=1)
{
  mavlink_message_t message{};
  pack(system,component,&codec,&message,&value);
  return message;
}

inline mavlink_power_status_t power()
{
  mavlink_power_status_t report{};
  report.Vcc=5025;
  report.flags=MAV_POWER_STATUS_BRICK_VALID|MAV_POWER_STATUS_SERVO_VALID|
               MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT|0x8000;
  return report;
}

inline mavlink_mcu_status_t mcu(uint8_t id)
{
  mavlink_mcu_status_t report{};
  report.id=id;
  if(id==2) {
    report.MCU_temperature=-125;
    report.MCU_voltage=3300;
    report.MCU_voltage_min=3200;
    report.MCU_voltage_max=3400;
  } else {
    report.MCU_temperature=2500;
    report.MCU_voltage=3200;
    report.MCU_voltage_max=3100;
  }
  return report;
}

inline mavlink_ekf_status_report_t ekf()
{
  mavlink_ekf_status_report_t report{};
  report.flags=EKF_ATTITUDE|EKF_UNINITIALIZED|EKF_GPS_GLITCHING|0x1000;
  report.velocity_variance=.25f;
  report.pos_horiz_variance=.5f;
  report.pos_vert_variance=.75f;
  report.compass_variance=1.25f;
  report.airspeed_variance=1.5f;
  return report;
}

inline mavlink_estimator_status_t estimator()
{
  mavlink_estimator_status_t report{};
  report.time_usec=9007199254740997ULL;
  report.flags=ESTIMATOR_ATTITUDE|ESTIMATOR_GPS_GLITCH|ESTIMATOR_ACCEL_ERROR|0x8000;
  report.vel_ratio=.125f;
  report.pos_horiz_ratio=.25f;
  report.pos_vert_ratio=.5f;
  report.mag_ratio=1.25f;
  report.tas_ratio=.75f;
  report.pos_horiz_accuracy=2.5f;
  report.pos_vert_accuracy=3.75f;
  return report;
}

} // namespace mavmoos::test::diagnostics

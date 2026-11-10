#pragma once
#include <ardupilotmega/mavlink.h>
#include <cstring>
#include <limits>

namespace mavmoos::test::power_fixture {
inline mavlink_battery_info_t battery(uint8_t id=4)
{
  mavlink_battery_info_t p{};
  p.id=id; p.battery_function=MAV_BATTERY_FUNCTION_ALL; p.type=MAV_BATTERY_TYPE_LIPO;
  p.design_capacity=10.25f;p.full_charge_capacity=8.5f;p.nominal_voltage=24.5f;
  p.discharge_minimum_voltage=3;p.charging_minimum_voltage=3.125f;
  p.resting_minimum_voltage=3.25f;p.charging_maximum_voltage=4.25f;
  p.charging_maximum_current=6.25f;p.discharge_maximum_current=20.5f;p.discharge_maximum_burst_current=40;
  p.state_of_health=83;p.cells_in_series=6;p.cycle_count=42;p.weight=1370;
  std::memcpy(p.name,"MAVMOOS_test_pack",17);std::memcpy(p.serial_number,"unit-4",6);
  std::memcpy(p.manufacture_date,"03112026",8);
  return p;
}
inline mavlink_battery_info_t unknown()
{
  mavlink_battery_info_t p{};
  p.id=9;p.cycle_count=UINT16_MAX;p.state_of_health=UINT8_MAX;
  p.full_charge_capacity=std::numeric_limits<float>::quiet_NaN();
  p.name[0]=static_cast<char>(255);
  for(auto& byte:p.serial_number) byte='X';
  return p;
}
inline mavlink_generator_status_t generator()
{
  mavlink_generator_status_t p{};
  p.status=(UINT64_C(1)<<60)|MAV_GENERATOR_STATUS_FLAG_GENERATING|MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED;
  p.battery_current=-3.25f;p.load_current=8.25f;p.power_generated=124.5f;p.bus_voltage=24.5f;
  p.bat_current_setpoint=std::numeric_limits<float>::quiet_NaN();
  p.runtime=UINT32_MAX;p.time_until_maintenance=-90;p.generator_speed=3800;
  p.rectifier_temperature=-12;p.generator_temperature=INT16_MAX;
  return p;
}
inline mavlink_message_t batteryFrame(mavlink_status_t& codec,
                                     const mavlink_battery_info_t& p=battery(),uint8_t sys=1,uint8_t comp=1)
{
  mavlink_message_t msg{};mavlink_msg_battery_info_encode_status(sys,comp,&codec,&msg,&p);return msg;
}
inline mavlink_message_t generatorFrame(mavlink_status_t& codec,
                                       const mavlink_generator_status_t& p=generator(),uint8_t sys=1,uint8_t comp=1)
{
  mavlink_message_t msg{};mavlink_msg_generator_status_encode_status(sys,comp,&codec,&msg,&p);return msg;
}
}

#include "air_data/AirDataFrames.h"
#include "debug_values/DebugFrames.h"
#include "event_timeline/EventFrames.h"
#include "status/PowerFrames.h"
#include "water_depth/WaterDepthFrames.h"
#include "navigation_monitor/NavigationFrames.h"
#include "link_status/LinkStatusFrames.h"
#include "gnss_observations/GnssCases.h"
#include "clock_status/ClockStatusFrames.h"
#include "obstacle_observations/ObstacleFrames.h"
#include "optical_flow/OpticalFlowFrames.h"
#include "terrain/TerrainFrames.h"
#include "wheel_odometry/WheelFrames.h"
#include <iomanip>
#include <iostream>
#include <string>

namespace {
void frame(const mavlink_message_t& message, bool last = true)
{
  uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
  const auto size = mavlink_msg_to_send_buffer(bytes, &message);
  for (unsigned i = 0; i < size; ++i)
    std::cout << std::hex << std::setfill('0') << std::setw(2) << unsigned(bytes[i]);
  if (last) std::cout << '\n';
}
}

int main(int argc, char** argv)
{
  if (argc != 2) return 2;
  const std::string identity = argv[1];
  if (identity != "ardupilot" && identity != "px4" && identity != "plane" && identity != "boat") return 2;
  mavlink_status_t codec{};
  using namespace mavmoos::test;
  for (int batch = 0; batch < 96; ++batch) {
    if (identity == "boat") {
      frame(water_depth::heartbeat(codec));
      const auto depth=water_depth::generatedFrames(codec);
      frame(depth[0],false);frame(depth[1]);
      const auto foreign=water_depth::generatedWrongSources(codec);
      frame(foreign[0],false);frame(foreign[1]);
      continue;
    }
    mavlink_message_t heartbeat{};
    mavlink_msg_heartbeat_pack_status(1, 1, &codec, &heartbeat,
      identity == "plane" ? MAV_TYPE_FIXED_WING : MAV_TYPE_QUADROTOR,
      identity == "px4" ? MAV_AUTOPILOT_PX4 : MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, MAV_STATE_STANDBY);
    frame(heartbeat);
    if (identity == "plane") {
      frame(air_data::airspeed(codec), false);
      frame(air_data::airspeed(codec, 1), false);
      frame(air_data::highLatency(codec), false);
      frame(air_data::vfrHud(codec));
      frame(air_data::airspeed(codec, 0, 2, 1), false);
      frame(air_data::highLatency(codec, 1, 2), false);
      frame(air_data::highLatency(codec, 1, 1, MAV_TYPE_QUADROTOR), false);
      frame(air_data::highLatency(codec, 1, 1, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_PX4), false);
      frame(air_data::airspeed(codec, 1));
      continue;
    }
    frame(navigation_frames::controller(codec), false);
    frame(navigation_frames::local(codec), false);
    frame(navigation_frames::global(codec), false);
    frame(wheels::wheelDistance(codec), false);
    frame(optical_flow_fixture::flow(codec), false);
    frame(optical_flow_fixture::trimmedFlow(codec), false);
    frame(optical_flow_fixture::rad(codec), false);
    frame(link_status_fixture::radio(codec), false);
    frame(link_status_fixture::cellular(codec), false);
    frame(link_status_fixture::cellularBase(codec), false);
    frame(clock_status_fixture::systemTime(codec), false);
    frame(clock_status_fixture::request(codec), false);
    frame(clock_status_fixture::reply(codec), false);
    frame(terrain::report(codec), false);
    if (identity == "ardupilot") {
      frame(power_fixture::batteryFrame(codec),false);
      frame(power_fixture::generatorFrame(codec),false);
      frame(obstacles::obstacle(codec), false);
      frame(gnss::frame(codec, gnss::secondaryFix(), mavlink_msg_gps2_raw_encode_status), false);
      frame(gnss::frame(codec, gnss::baseline<mavlink_gps_rtk_t>(RTK_BASELINE_COORDINATE_SYSTEM_ECEF),
                        mavlink_msg_gps_rtk_encode_status), false);
      auto second = gnss::baseline<mavlink_gps2_rtk_t>(RTK_BASELINE_COORDINATE_SYSTEM_NED);
      second.rtk_receiver_id = 9;
      second.baseline_a_mm = 3100;
      frame(gnss::frame(codec, second, mavlink_msg_gps2_rtk_encode_status), false);
    }
    for(const auto& msg:debug_fixture::frames(codec)) frame(msg,false);
    for(const auto& msg:event_timeline_fixture::frames(codec,batch)) frame(msg,false);
    frame(terrain::request(codec));
    frame(navigation_frames::controller(codec, 2, 1), false);
    frame(optical_flow_fixture::flow(codec, 1, 2), false);
    frame(debug_fixture::encode(codec, debug_fixture::debug(), 2, 1), false);
    frame(debug_fixture::encode(codec, debug_fixture::namedFloat(), 1, 2), false);
    auto foreign_event=event_timeline_fixture::event(700);
    foreign_event.id=0xfefdfcfb;
    frame(event_timeline_fixture::encode(codec,foreign_event,2,1),false);
    frame(event_timeline_fixture::encode(codec,event_timeline_fixture::current(701,1),1,2),
          identity != "ardupilot");
    if(identity == "ardupilot") {
      frame(power_fixture::batteryFrame(codec,power_fixture::battery(34),2,1),false);
      auto foreign_generator=power_fixture::generator();
      foreign_generator.bus_voltage=999;
      frame(power_fixture::generatorFrame(codec,foreign_generator,1,2));
    }
  }
}

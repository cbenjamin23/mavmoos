#pragma once

#include "Plugin.h"

namespace mavmoos::test::gnss {
template<typename Report, typename Encode>
mavlink_message_t frame(mavlink_status_t& codec, const Report& report, Encode encode,
                        unsigned system = 1, unsigned component = 1)
{
  mavlink_message_t message{};
  encode(system, component, &codec, &message, &report);
  return message;
}

inline mavlink_gps2_raw_t secondaryFix()
{
  mavlink_gps2_raw_t report{};
  report.time_usec = 18446744073709551000ULL;
  report.fix_type = GPS_FIX_TYPE_RTK_FIXED;
  report.lat = 423584561;
  report.lon = -710875891;
  report.alt = -1234;
  report.eph = 123;
  report.epv = 234;
  report.vel = 345;
  report.cog = 35999;
  report.satellites_visible = 14;
  report.dgps_numch = 7;
  report.dgps_age = 351;
  report.yaw = 36000;
  report.alt_ellipsoid = 35000;
  report.h_acc = 1200;
  report.v_acc = 2300;
  report.vel_acc = 123;
  report.hdg_acc = 345000;
  return report;
}

template<typename Report>
Report baseline(uint8_t coordinateSystem)
{
  Report report{};
  report.wn = 2365;
  report.tow = 456789123;
  report.nsats = 17;
  report.baseline_coords_type = coordinateSystem;
  report.baseline_a_mm = -1500;
  report.baseline_b_mm = 2750;
  report.baseline_c_mm = -990;
  report.accuracy = UINT32_MAX;
  report.iar_num_hypotheses = -1;
  return report;
}

template<typename Rig>
void run(const char* path)
{
  {
    Rig rig(path);
    rig.capture("gnss_absent");
    auto secondary = secondaryFix();
    const auto gps2 = frame(rig.codec, secondary, mavlink_msg_gps2_raw_encode_status);
    auto primaryRtk = baseline<mavlink_gps_rtk_t>(RTK_BASELINE_COORDINATE_SYSTEM_ECEF);
    const auto rtk = frame(rig.codec, primaryRtk, mavlink_msg_gps_rtk_encode_status);
    auto secondaryRtk = baseline<mavlink_gps2_rtk_t>(RTK_BASELINE_COORDINATE_SYSTEM_NED);
    secondaryRtk.time_last_baseline_ms = 1500;
    secondaryRtk.rtk_receiver_id = 9;
    secondaryRtk.rtk_rate = 3;
    secondaryRtk.rtk_health = 11;
    secondaryRtk.baseline_a_mm = 3100;
    const auto rtk2 = frame(rig.codec, secondaryRtk, mavlink_msg_gps2_rtk_encode_status);
    rig.receive(gps2, 100.07, 100.07);
    rig.bind();
    rig.receive(rtk, 100.09, 100.12);
    rig.capture("gnss_prebind");

    mavlink_gps_raw_int_t primary{};
    primary.time_usec = 42;
    primary.fix_type = GPS_FIX_TYPE_3D_FIX;
    primary.lat = 123;
    primary.lon = -456;
    rig.gps(primary, 100.2, 100.2);
    rig.receive(gps2, 100.3, 100.4);
    rig.receive(rtk, 100.45, 100.5);
    rig.receive(rtk2, 100.6, 100.6, "READ_ACQUISITION");
    rig.capture("gnss_bound");
    rig.host.time = 100.8;
    rig.capture("gnss_republish");

    rig.plugin->configure("status_timeout", ".6");
    rig.heartbeat(100.9, 100.9);
    rig.receive(rtk2, 101.02, 101.02);
    rig.host.time = 101.2;
    rig.capture("gnss_independent_age");
    rig.host.time = 103;
    rig.capture("gnss_lapse");
    rig.receive(rtk2, 103.08, 103.08);
    rig.capture("gnss_after_lapse_data");
    rig.heartbeat(103.2, 103.2);
    rig.receive(gps2, 103.15, 103.3);
    rig.capture("gnss_rebind_old");
    rig.receive(gps2, 103.4, 103.4);
    rig.capture("gnss_rebind_fresh");
    rig.host.time = 103.5;
    rig.plugin->onReset("gnss_explicit_reset");
    rig.capture("gnss_reset");
  }
  {
    Rig rig(path);
    rig.bind();
    auto gps2 = secondaryFix();
    gps2.fix_type = GPS_FIX_TYPE_NO_FIX;
    gps2.eph = gps2.epv = gps2.vel = gps2.cog = UINT16_MAX;
    gps2.satellites_visible = UINT8_MAX;
    gps2.dgps_numch = 0;
    gps2.dgps_age = 0;
    gps2.yaw = UINT16_MAX;
    gps2.h_acc = gps2.vel_acc = gps2.hdg_acc = 0;
    rig.receive(frame(rig.codec, gps2, mavlink_msg_gps2_raw_encode_status), 100.2, 100.2);
    rig.capture("gnss_sentinels");

    rig.codec.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    rig.receive(frame(rig.codec, gps2, mavlink_msg_gps2_raw_encode_status), 100.3, 100.3);
    rig.capture("gnss_mavlink1");
    rig.codec.flags = 0;

    gps2.fix_type = GPS_FIX_TYPE_3D_FIX;
    gps2.lat = gps2.lon = gps2.alt = 0;
    gps2.vel = gps2.cog = 0;
    gps2.yaw = 90;
    gps2.alt_ellipsoid = gps2.v_acc = 0;
    rig.receive(frame(rig.codec, gps2, mavlink_msg_gps2_raw_encode_status), 100.4, 100.4);
    rig.capture("gnss_mavlink2_trimmed");

    gps2.fix_type = UINT8_MAX;
    gps2.lat = INT32_MAX;
    gps2.cog = 40000;
    gps2.yaw = 40000;
    rig.receive(frame(rig.codec, gps2, mavlink_msg_gps2_raw_encode_status), 100.5, 100.5);
    auto rtk = baseline<mavlink_gps_rtk_t>(7);
    rig.receive(frame(rig.codec, rtk, mavlink_msg_gps_rtk_encode_status), 100.51, 100.51);
    rig.host.time = 100.56;
    rig.capture("gnss_invalid_fields");
  }
  {
    Rig rig(path);
    rig.bind();
    const auto gps2 = secondaryFix();
    const auto good = frame(rig.codec, gps2, mavlink_msg_gps2_raw_encode_status);
    rig.receive(good, 100.3, 100.4);
    rig.capture("gnss_before_rejection");
    rig.receive(good, 100.2, 100.5);
    rig.receive(frame(rig.codec, gps2, mavlink_msg_gps2_raw_encode_status, 1, 2), 100.5, 100.55);
    auto bad = good;
    bad.len = MAVLINK_MSG_ID_GPS2_RAW_LEN + 1;
    rig.receive(bad, 100.56, 100.56);
    bad.magic = MAVLINK_STX_MAVLINK1;
    bad.len = MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN - 1;
    rig.receive(bad, 100.57, 100.57);
    rig.capture("gnss_after_rejection");
  }
}
}

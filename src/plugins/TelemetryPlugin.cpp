/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: TelemetryPlugin.cpp                             */
/*    DATE: November 3, 2026                                */
/************************************************************/
#include "Plugin.h"
#include "ArduPilotObservations.h"
#include "Px4Offboard.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
double numeric(const std::string& value,double low,double high)
{
  std::size_t end=0;
  const double result=std::stod(value,&end);
  if(end!=value.size() || !std::isfinite(result) || result<low || result>high)
    throw std::invalid_argument("invalid telemetry configuration value: "+value);
  return result;
}

class TelemetryPlugin final:public Plugin {
public:
  explicit TelemetryPlugin(Services& services):m_services(services) {}
  std::string name() const override { return "telemetry"; }
  PluginDescriptor descriptor() const override {
    PluginDescriptor d; d.profile=m_px4?"px4_navigation":"ardupilot_navigation";
    d.capabilities={{"telemetry.navigation","","","observation"}};
    d.exclusive_resources={"projection.navigation"};
    if(!m_px4)d.observation_streams=arduPilotObservationStreams();
    d.streamed_parameter_bench_compatible=true; return d;
  }
  bool projectGlobal(double latitude,double longitude,double& x,double& y) const override {
    if(!m_origin_ready || !std::isfinite(latitude) || !std::isfinite(longitude) ||
       latitude < -80 || latitude > 84 || std::abs(longitude)>180) return false;
    double north=0,east=0;
    if(!m_geodesy.LatLong2LocalGrid(latitude,longitude,north,east) ||
       !std::isfinite(north) || !std::isfinite(east)) return false;
    x=east; y=north; return true;
  }
  void onReset(const std::string&) override {
    m_last_position=std::numeric_limits<double>::quiet_NaN();
    m_reset_at=m_services.now();
    m_altitude_home.reset();
  }
  void configure(const std::string& key,const std::string& value) override {
    if(key=="vehicle_profile") {m_px4=px4::isProfile(value);m_sub=value=="ardusub";return;}
    if(key=="lat_origin") { m_latitude=numeric(value,-80,84); m_origin_ready=false; }
    else if(key=="long_origin") { m_longitude=numeric(value,-180,180); m_origin_ready=false; }
    else if(key=="nav_position_timeout") m_timeout=numeric(value,0.01,10);
    else return;
    // A new origin must project even an otherwise unchanged sample again.
    m_last_position=std::numeric_limits<double>::quiet_NaN();
    if(m_latitude && m_longitude) {
      m_origin_ready=m_geodesy.Initialise(*m_latitude,*m_longitude);
      if(!m_origin_ready) throw std::invalid_argument("MOOS geodesy rejected navigation origin");
    }
  }
  void iterate() override {
    const auto& v=m_services.vehicle();
    const double now=m_services.now();
    const double age=now-v.position_at;
    const bool valid=v.position_valid && std::isfinite(now) && std::isfinite(v.position_at) &&
      v.position_at>m_reset_at && age>=0 && age<=m_timeout && std::isfinite(v.latitude) && std::isfinite(v.longitude) &&
      std::abs(v.latitude)<=90 && std::abs(v.longitude)<=180 && std::isfinite(v.msl_height) &&
      std::isfinite(v.relative_height) && std::isfinite(v.north_velocity) &&
      std::isfinite(v.east_velocity) && std::isfinite(v.down_velocity);
    const bool heading_valid=valid && std::isfinite(v.heading) && v.heading>=0 && v.heading<360;
    const double speed=valid ? std::hypot(v.north_velocity,v.east_velocity) : 0;
    const bool course_valid=valid && speed>0.001;
    const bool new_position=valid && (v.position_at!=m_last_position || v.position_read_at!=m_last_read);
    const std::array<double,3> home{v.home_latitude,v.home_longitude,v.home_msl};
    const bool home_valid=v.home_valid && std::isfinite(v.home_latitude) && std::isfinite(v.home_longitude) &&
      std::isfinite(v.home_msl) && std::isfinite(v.home_at) && v.home_at>=0;
    // A HOME update alone cannot validate a retained NAV_ALTITUDE. Wait for a
    // position on the new reference. A repeated unchanged HOME retains it.
    if(!valid || !home_valid || (m_altitude_home && *m_altitude_home!=home))
      m_altitude_home.reset();
    if(new_position && home_valid && (m_altitude_home || v.home_at<=v.position_at))
      m_altitude_home=home;
    const bool altitude_valid=m_altitude_home.has_value();
    bool local_valid=false;
    double north=0,east=0;
    if(valid && m_origin_ready)
      local_valid=m_geodesy.LatLong2LocalGrid(v.latitude,v.longitude,north,east) &&
        std::isfinite(north) && std::isfinite(east);

    // Status retains the original receipt time even when diagnostics republish.
    std::ostringstream status;
    status<<std::setprecision(16)<<"schema=2,position_valid="<<(valid?"true":"false")
      <<",local_valid="<<(local_valid?"true":"false")
      <<",heading_valid="<<(heading_valid?"true":"false")
      <<",course_valid="<<(course_valid?"true":"false")
      <<",altitude_valid="<<(altitude_valid?"true":"false")
      <<",received_at="<<v.position_at<<",age_s="<<(v.position_valid?age:-1)
      <<",read_at="<<v.position_read_at<<",age_basis="<<v.position_time_basis
      <<",age_is_upper_bound="<<(v.position_time_basis=="QUEUE_EMPTY_FLOOR"?"true":"false")
      <<",source_age=unknown,xy_frame=MOOS_LOCAL_GRID,altitude_reference=home";
    m_services.publish("MAVMOOS_NAV_STATUS",status.str());
    m_services.publish("NAV_VALID",valid?1.0:0.0);
    m_services.publish("NAV_ALTITUDE_VALID",altitude_valid?1.0:0.0);
    const bool depth_valid=m_sub && altitude_valid && now>=v.home_at && now-v.home_at<=4;
    if(m_sub) m_services.publish("NAV_DEPTH_VALID",depth_valid?1.0:0.0);
    m_services.publish("NAV_HEADING_VALID",heading_valid?1.0:0.0);
    m_services.publish("NAV_COURSE_VALID",course_valid?1.0:0.0);
    m_services.publish("NAV_XY_VALID",local_valid?1.0:0.0);
    if(!new_position) return;
    m_last_position=v.position_at;
    m_last_read=v.position_read_at;
    m_services.publish("NAV_LAT",v.latitude);
    m_services.publish("NAV_LONG",v.longitude);
    m_services.publish("NAV_SPEED",speed);
    m_services.publish("NAV_ALTITUDE_MSL",v.msl_height);
    m_services.publish("NAV_VERTICAL_SPEED",-v.down_velocity);
    m_services.publish("NAV_SAMPLE_RECEIVED_AT",v.position_at);
    if(altitude_valid) m_services.publish("NAV_ALTITUDE",v.relative_height);
    if(depth_valid) m_services.publish("NAV_DEPTH",v.home_msl-v.msl_height);
    if(heading_valid) m_services.publish("NAV_HEADING",v.heading);
    if(course_valid) {
      double course=std::atan2(v.east_velocity,v.north_velocity)*180/std::acos(-1.0);
      if(course<0) course+=360;
      m_services.publish("NAV_COURSE",course);
    }
    if(local_valid) {
      m_services.publish("NAV_X",east);
      m_services.publish("NAV_Y",north);
    }
  }
private:
  Services& m_services;
  mutable CMOOSGeodesy m_geodesy;
  std::optional<double> m_latitude,m_longitude;
  std::optional<std::array<double,3>> m_altitude_home;
  bool m_origin_ready=false,m_px4=false,m_sub=false;
  double m_timeout=0.5;
  double m_reset_at=-std::numeric_limits<double>::infinity();
  double m_last_position=std::numeric_limits<double>::quiet_NaN();
  double m_last_read=-1;
};
}
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{
  if(abi!=mavmoos::runtime::plugin_abi || !services) return nullptr;
  return new mavmoos::runtime::TelemetryPlugin(*services);
}
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin) { delete plugin; }

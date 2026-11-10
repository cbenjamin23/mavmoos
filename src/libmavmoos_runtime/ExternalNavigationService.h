#pragma once

#include <ardupilotmega/mavlink.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace mavmoos::runtime {

inline constexpr char external_navigation_capability[] = "external_navigation.odometry.sitl";
inline constexpr double external_navigation_age_s = .25;
inline constexpr double external_navigation_lease_s = 30;
inline constexpr double external_navigation_silence_s = .5;
inline constexpr double external_navigation_clock_slop_s = .005;

struct ExternalNavigationSample {
  uint64_t run_id=0, sequence=0, capture_us=0;
  double capture_at=0, expires=0;
  std::array<float,3> position{}, velocity{}, angular_rate{};
  std::array<float,4> quaternion{};
  std::array<float,3> pose_variance{}, angle_variance{}, velocity_variance{}, rate_variance{};
  uint8_t quality=0;
};

inline double externalNavigationCaptureTime(uint64_t capture_us)
{
  return double(capture_us) / 1e6;
}

inline bool validExternalNavigationSample(const ExternalNavigationSample& sample,double now)
{
  constexpr uint64_t max_exact_capture_us=uint64_t{1}<<53;
  if(!sample.run_id || !sample.sequence || !sample.capture_us || sample.capture_us>max_exact_capture_us ||
     !std::isfinite(now) || !std::isfinite(sample.capture_at) || !std::isfinite(sample.expires) ||
     sample.capture_at<=0 || sample.capture_at>now+external_navigation_clock_slop_s ||
     sample.expires<=now || sample.quality<50 || sample.quality>100) return false;
  const double measured_at=externalNavigationCaptureTime(sample.capture_us);
  const double original=std::min(measured_at,sample.capture_at);
  if(measured_at>now || measured_at>sample.capture_at+external_navigation_clock_slop_s ||
     now-original>=external_navigation_age_s || sample.expires>original+external_navigation_age_s) return false;

  const auto within=[](const auto& values,float minimum,float maximum) {
    return std::all_of(values.begin(),values.end(),[=](float value) {
      return std::isfinite(value) && value>=minimum && value<=maximum;
    });
  };
  if(!within(sample.position,-30,30) || !within(sample.velocity,-3,3) ||
     !within(sample.angular_rate,-2,2) || !within(sample.pose_variance,.0025f,4) ||
     !within(sample.angle_variance,.0001f,1) || !within(sample.velocity_variance,.0025f,4) ||
     !within(sample.rate_variance,.0001f,4)) return false;

  double norm_squared=0;
  for(float value:sample.quaternion) norm_squared+=double(value)*value;
  return std::isfinite(norm_squared) && std::abs(norm_squared-1)<=.002;
}

// This shape matches the pinned ArduPilot ExternalNav input contract.
inline mavlink_odometry_t makeExternalNavigationOdometry(const ExternalNavigationSample& sample)
{
  mavlink_odometry_t packet{};
  packet.time_usec=sample.capture_us;
  packet.frame_id=MAV_FRAME_LOCAL_FRD;
  packet.child_frame_id=MAV_FRAME_BODY_FRD;
  packet.estimator_type=MAV_ESTIMATOR_TYPE_VISION;
  packet.quality=static_cast<int8_t>(sample.quality);
  packet.x=sample.position[0];packet.y=sample.position[1];packet.z=sample.position[2];
  packet.vx=sample.velocity[0];packet.vy=sample.velocity[1];packet.vz=sample.velocity[2];
  packet.rollspeed=sample.angular_rate[0];packet.pitchspeed=sample.angular_rate[1];packet.yawspeed=sample.angular_rate[2];
  double norm_squared=0;
  for(float value:sample.quaternion) norm_squared+=double(value)*value;
  const double norm=std::sqrt(norm_squared);
  for(size_t i=0;i<sample.quaternion.size();++i) packet.q[i]=static_cast<float>(sample.quaternion[i]/norm);
  constexpr std::array<size_t,6> diagonal{0,6,11,15,18,20};
  for(size_t i=0;i<3;++i) {
    packet.pose_covariance[diagonal[i]]=sample.pose_variance[i];
    packet.pose_covariance[diagonal[i+3]]=sample.angle_variance[i];
    packet.velocity_covariance[diagonal[i]]=sample.velocity_variance[i];
    packet.velocity_covariance[diagonal[i+3]]=sample.rate_variance[i];
  }
  return packet;
}

// The host owns the lease, latest-sample slot and final eligibility check.
class ExternalNavigationService {
public:
  virtual ~ExternalNavigationService()=default;
  virtual bool externalNavigationReady() const=0;
  virtual bool beginExternalNavigation(uint64_t run_id,double expires)=0;
  virtual bool externalNavigation(const ExternalNavigationSample&)=0;
  virtual void stopExternalNavigation(uint64_t run_id)=0;
};

class ExternalNavigationListener {
public:
  virtual ~ExternalNavigationListener()=default;
  virtual void onExternalNavigationSent(const ExternalNavigationSample&,double,bool)=0;
};

} // namespace mavmoos::runtime

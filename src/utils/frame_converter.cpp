#include "utils/frame_converter.hpp"

#include <cmath>
#include <stdexcept>

#include "utils/gnss_local_frame.hpp"

namespace arch_nav::utils::frame_converter {

namespace {

vehicle::Waypoint enu_to_ned(const vehicle::Waypoint& wp) {
  return {wp.y, wp.x, -wp.z};
}

vehicle::Waypoint body_to_ned(const vehicle::Waypoint& wp, double heading) {
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  return {
    wp.x * c - wp.y * s,
    wp.x * s + wp.y * c,
    wp.z
  };
}

vehicle::TrajectoryPoint enu_to_ned(const vehicle::TrajectoryPoint& pt) {
  vehicle::TrajectoryPoint out;
  out.t  = pt.t;
  out.x  =  pt.y;  out.y  =  pt.x;  out.z  = -pt.z;
  out.vx =  pt.vy; out.vy =  pt.vx; out.vz = -pt.vz;
  out.ax =  pt.ay; out.ay =  pt.ax; out.az = -pt.az;
  out.heading = M_PI / 2.0 - pt.heading;
  out.omega   = -pt.omega;
  return out;
}

vehicle::TrajectoryPoint body_to_ned(const vehicle::TrajectoryPoint& pt, double heading) {
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  vehicle::TrajectoryPoint out;
  out.t  = pt.t;
  out.x  = pt.x * c - pt.y * s;
  out.y  = pt.x * s + pt.y * c;
  out.z  = pt.z;
  out.vx = pt.vx * c - pt.vy * s;
  out.vy = pt.vx * s + pt.vy * c;
  out.vz = pt.vz;
  out.ax = pt.ax * c - pt.ay * s;
  out.ay = pt.ax * s + pt.ay * c;
  out.az = pt.az;
  out.heading = pt.heading + heading;
  out.omega   = pt.omega;
  return out;
}

}  // namespace

vehicle::Waypoint to_ned(
    const vehicle::Waypoint& wp,
    constants::ReferenceFrame frame,
    double heading) {
  switch (frame) {
    case constants::ReferenceFrame::LOCAL_NED:
      return wp;
    case constants::ReferenceFrame::LOCAL_ENU:
      return enu_to_ned(wp);
    case constants::ReferenceFrame::BODY_FCS:
      return body_to_ned(wp, heading);
    default:
      throw std::runtime_error("Use global_to_ned for GLOBAL_WGS84 waypoints");
  }
}

std::vector<vehicle::Waypoint> to_ned(
    const std::vector<vehicle::Waypoint>& waypoints,
    constants::ReferenceFrame frame,
    double heading) {
  std::vector<vehicle::Waypoint> result;
  result.reserve(waypoints.size());
  for (const auto& wp : waypoints) {
    result.push_back(to_ned(wp, frame, heading));
  }
  return result;
}

vehicle::TrajectoryPoint to_ned(
    const vehicle::TrajectoryPoint& pt,
    constants::ReferenceFrame frame,
    double heading) {
  switch (frame) {
    case constants::ReferenceFrame::LOCAL_NED:
      return pt;
    case constants::ReferenceFrame::LOCAL_ENU:
      return enu_to_ned(pt);
    case constants::ReferenceFrame::BODY_FCS:
      return body_to_ned(pt, heading);
    default:
      throw std::runtime_error("Cannot convert GLOBAL_WGS84 to TrajectoryPoint");
  }
}

std::vector<vehicle::TrajectoryPoint> to_ned(
    const std::vector<vehicle::TrajectoryPoint>& trajectory,
    constants::ReferenceFrame frame,
    double heading) {
  std::vector<vehicle::TrajectoryPoint> result;
  result.reserve(trajectory.size());
  for (const auto& pt : trajectory) {
    result.push_back(to_ned(pt, frame, heading));
  }
  return result;
}

vehicle::Waypoint global_to_ned(
    const vehicle::Waypoint& wp,
    const vehicle::GlobalPosition& ref) {
  return {
    gnss_local_frame::lat_to_ned_x(wp.lat, ref.lat),
    gnss_local_frame::lon_to_ned_y(wp.lon, ref.lon, ref.lat),
    gnss_local_frame::alt_to_ned_z(wp.alt, ref.alt)
  };
}

vehicle::Waypoint ned_to_global(
    const vehicle::Waypoint& wp,
    const vehicle::GlobalPosition& ref) {
  return {
    gnss_local_frame::ned_x_to_lat(wp.x, ref.lat),
    gnss_local_frame::ned_y_to_lon(wp.y, ref.lon, ref.lat),
    gnss_local_frame::ned_z_to_alt(wp.z, ref.alt)
  };
}

}  // namespace arch_nav::utils::frame_converter

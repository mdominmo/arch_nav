#include "utils/gnss_local_frame.hpp"

#include <cmath>

namespace arch_nav::utils::gnss_local_frame {

static constexpr double kEarthRadius  = 6371000.0;
static constexpr double kDegToRad     = M_PI / 180.0;

double lat_to_ned_x(double lat, double ref_lat) {
  return (lat - ref_lat) * kDegToRad * kEarthRadius;
}

double lon_to_ned_y(double lon, double ref_lon, double ref_lat) {
  return (lon - ref_lon) * kDegToRad * kEarthRadius * std::cos(ref_lat * kDegToRad);
}

double alt_to_ned_z(double alt, double ref_alt) {
  return -(alt - ref_alt);
}

static constexpr double kRadToDeg = 180.0 / M_PI;

double ned_x_to_lat(double x, double ref_lat) {
  return ref_lat + (x / kEarthRadius) * kRadToDeg;
}

double ned_y_to_lon(double y, double ref_lon, double ref_lat) {
  return ref_lon + (y / (kEarthRadius * std::cos(ref_lat * kDegToRad))) * kRadToDeg;
}

double ned_z_to_alt(double z, double ref_alt) {
  return ref_alt - z;
}

}  // namespace arch_nav::utils::gnss_local_frame

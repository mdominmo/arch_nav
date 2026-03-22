#ifndef ARCH_NAV__CORE__MODEL__VEHICLE__GEO_WAYPOINT_HPP_
#define ARCH_NAV__CORE__MODEL__VEHICLE__GEO_WAYPOINT_HPP_

namespace arch_nav::vehicle {

struct GeoWaypoint {
  double lat;
  double lon;
  double alt;

  double qx;
  double qy;
  double qz;
  double qw;

  GeoWaypoint(
      double lat_value = 0.0,
      double lon_value = 0.0,
      double alt_value = 0.0,
      double qx_value = 0.0,
      double qy_value = 0.0,
      double qz_value = 0.0,
      double qw_value = 1.0)
      : lat(lat_value),
        lon(lon_value),
        alt(alt_value),
        qx(qx_value),
        qy(qy_value),
        qz(qz_value),
        qw(qw_value) {}
};

}  // namespace arch_nav::vehicle

#endif  // ARCH_NAV__CORE__MODEL__VEHICLE__GEO_WAYPOINT_HPP_

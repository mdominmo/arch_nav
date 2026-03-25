#ifndef ARCH_NAV_MODEL_VEHICLE_WAYPOINT_HPP_
#define ARCH_NAV_MODEL_VEHICLE_WAYPOINT_HPP_

namespace arch_nav::vehicle {

struct Waypoint {
  union { double lat; double x; };
  union { double lon; double y; };
  union { double alt; double z; };

  Waypoint(double a = 0.0, double b = 0.0, double c = 0.0)
      : x(a), y(b), z(c) {}
};

}  // namespace arch_nav::vehicle

#endif  // ARCH_NAV_MODEL_VEHICLE_WAYPOINT_HPP_

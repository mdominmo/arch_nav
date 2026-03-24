#ifndef ARCH_NAV__CORE__MODEL__VEHICLE__TRAJECTORY_POINT_HPP_
#define ARCH_NAV__CORE__MODEL__VEHICLE__TRAJECTORY_POINT_HPP_

namespace arch_nav::vehicle {

struct TrajectoryPoint {
  double t;

  double x;
  double y;
  double z;

  double vx;
  double vy;
  double vz;

  double ax;
  double ay;
  double az;

  double heading;
  double omega;
};

}  // namespace arch_nav::vehicle

#endif  // ARCH_NAV__CORE__MODEL__VEHICLE__TRAJECTORY_POINT_HPP_

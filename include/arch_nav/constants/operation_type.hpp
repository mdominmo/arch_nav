#ifndef ARCH_NAV_CONSTANTS_OPERATION_TYPE_HPP_
#define ARCH_NAV_CONSTANTS_OPERATION_TYPE_HPP_

namespace arch_nav::constants {

enum class OperationType {
  NONE,
  TAKEOFF,
  LAND,
  WAYPOINT_FOLLOWING,
  TRAJECTORY_EXECUTION,
  CHANGE_YAW,
  FOLLOW_TARGET
};

}  // namespace arch_nav::constants

#endif  // ARCH_NAV_CONSTANTS_OPERATION_TYPE_HPP_

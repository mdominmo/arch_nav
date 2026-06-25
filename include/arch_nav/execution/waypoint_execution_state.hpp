#ifndef ARCH_NAV_EXECUTION_WAYPOINT_EXECUTION_STATE_HPP_
#define ARCH_NAV_EXECUTION_WAYPOINT_EXECUTION_STATE_HPP_

#include <atomic>

namespace arch_nav::execution {

struct WaypointExecutionState {
  std::atomic<int> current_waypoint{0};
  std::atomic<int> total_waypoints{0};
};

}  // namespace arch_nav::execution

#endif  // ARCH_NAV_EXECUTION_WAYPOINT_EXECUTION_STATE_HPP_

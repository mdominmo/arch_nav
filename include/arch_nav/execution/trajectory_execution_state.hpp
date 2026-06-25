#ifndef ARCH_NAV_EXECUTION_TRAJECTORY_EXECUTION_STATE_HPP_
#define ARCH_NAV_EXECUTION_TRAJECTORY_EXECUTION_STATE_HPP_

#include <atomic>

namespace arch_nav::execution {

struct TrajectoryExecutionState {
  std::atomic<int> current_point_index{0};
  std::atomic<int> total_points{0};
};

}  // namespace arch_nav::execution

#endif  // ARCH_NAV_EXECUTION_TRAJECTORY_EXECUTION_STATE_HPP_

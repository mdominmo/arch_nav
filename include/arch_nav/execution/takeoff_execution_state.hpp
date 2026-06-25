#ifndef ARCH_NAV_EXECUTION_TAKEOFF_EXECUTION_STATE_HPP_
#define ARCH_NAV_EXECUTION_TAKEOFF_EXECUTION_STATE_HPP_

#include <atomic>

namespace arch_nav::execution {

struct TakeoffExecutionState {
  std::atomic<double> current_altitude{0.0};
  std::atomic<double> target_altitude{0.0};
};

}  // namespace arch_nav::execution

#endif  // ARCH_NAV_EXECUTION_TAKEOFF_EXECUTION_STATE_HPP_

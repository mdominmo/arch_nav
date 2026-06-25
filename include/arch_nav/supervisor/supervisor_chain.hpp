#ifndef ARCH_NAV_SUPERVISOR_SUPERVISOR_CHAIN_HPP_
#define ARCH_NAV_SUPERVISOR_SUPERVISOR_CHAIN_HPP_

#include <mutex>
#include <vector>

#include "arch_nav/supervisor/i_supervisor.hpp"
#include "arch_nav/controller/preemption_info.hpp"
#include "arch_nav/controller/preemption_type.hpp"

namespace arch_nav::supervisor {

class SupervisorChain {
 public:
  explicit SupervisorChain(controller::IOperationalController& controller);

  void register_supervisor(ISupervisor& supervisor, int priority,
                           controller::PreemptionType type);

  void request_control(ISupervisor& requester,
                       const controller::PreemptionInfo& info);
  void release_control(ISupervisor& requester);

 private:
  struct RegisteredSupervisor {
    ISupervisor* supervisor;
    int priority;
    controller::PreemptionType type;
  };

  controller::IOperationalController& controller_;

  mutable std::mutex mutex_;
  std::vector<RegisteredSupervisor> supervisors_;
  ISupervisor* active_supervisor_{nullptr};
  int active_priority_{std::numeric_limits<int>::max()};
  controller::PreemptionType active_type_{controller::PreemptionType::TERMINAL};
};

}  // namespace arch_nav::supervisor

#endif  // ARCH_NAV_SUPERVISOR_SUPERVISOR_CHAIN_HPP_

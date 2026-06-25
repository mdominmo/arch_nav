#include "arch_nav/supervisor/supervisor_chain.hpp"

namespace arch_nav::supervisor {

SupervisorChain::SupervisorChain(controller::IOperationalController& controller)
    : controller_(controller) {}

void SupervisorChain::register_supervisor(ISupervisor& supervisor, int priority,
                                          controller::PreemptionType type) {
  std::lock_guard<std::mutex> lock(mutex_);
  supervisors_.push_back({&supervisor, priority, type});
}

void SupervisorChain::request_control(ISupervisor& requester,
                                      const controller::PreemptionInfo& info) {
  std::lock_guard<std::mutex> lock(mutex_);

  const RegisteredSupervisor* entry = nullptr;
  for (const auto& s : supervisors_) {
    if (s.supervisor == &requester) {
      entry = &s;
      break;
    }
  }
  if (!entry) return;

  if (active_supervisor_ && entry->priority >= active_priority_) {
    return;
  }

  controller_.preempt(entry->type, info);

  active_supervisor_ = &requester;
  active_priority_ = entry->priority;
  active_type_ = entry->type;

  requester.execute(controller_);
}

void SupervisorChain::release_control(ISupervisor& requester) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (active_supervisor_ != &requester) return;

  active_supervisor_ = nullptr;
  active_priority_ = std::numeric_limits<int>::max();
}

}  // namespace arch_nav::supervisor

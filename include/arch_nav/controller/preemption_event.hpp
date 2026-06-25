#ifndef ARCH_NAV_CONTROLLER_PREEMPTION_EVENT_HPP_
#define ARCH_NAV_CONTROLLER_PREEMPTION_EVENT_HPP_

#include <string>

#include "arch_nav/controller/preemption_event_type.hpp"
#include "arch_nav/controller/preemption_type.hpp"

namespace arch_nav::controller {

struct PreemptionEvent {
  PreemptionEventType event_type;
  std::string name;
  std::string reason;
  PreemptionType preemption_type;
  std::string details;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV_CONTROLLER_PREEMPTION_EVENT_HPP_

#ifndef ARCH_NAV_CONTROLLER_PREEMPTION_INFO_HPP_
#define ARCH_NAV_CONTROLLER_PREEMPTION_INFO_HPP_

#include <string>

namespace arch_nav::controller {

struct PreemptionInfo {
  std::string name;
  std::string reason;
  std::string details;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV_CONTROLLER_PREEMPTION_INFO_HPP_

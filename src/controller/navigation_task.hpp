#ifndef ARCH_NAV_CONTROLLER_NAVIGATION_TASK_HPP_
#define ARCH_NAV_CONTROLLER_NAVIGATION_TASK_HPP_

#include <functional>
#include <memory>

#include "arch_nav/constants/command_response.hpp"
#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/driver/i_command_dispatcher.hpp"

namespace arch_nav::controller {

class NavigationTask {
 public:
  virtual constants::CommandResponse start(
      platform::ICommandDispatcher& dispatcher,
      std::function<void()> on_complete) = 0;

  virtual void abort() = 0;

  virtual std::shared_ptr<report::OperationReport> make_report() const = 0;

  virtual ~NavigationTask() = default;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV_CONTROLLER_NAVIGATION_TASK_HPP_

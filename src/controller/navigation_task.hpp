#ifndef ARCH_NAV_CONTROLLER_NAVIGATION_TASK_HPP_
#define ARCH_NAV_CONTROLLER_NAVIGATION_TASK_HPP_

#include <functional>
#include <memory>

#include "arch_nav/constants/command_response.hpp"
#include "arch_nav/context/vehicle_context.hpp"
#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/driver/i_command_dispatcher.hpp"

namespace arch_nav::controller {

class NavigationTask {
 public:
  virtual constants::CommandResponse start(
      context::VehicleContext& context,
      dispatchers::ICommandDispatcher& dispatcher,
      std::function<void()> on_complete) = 0;

  virtual void abort() = 0;

  virtual std::shared_ptr<report::OperationReport> make_report() = 0;

  virtual ~NavigationTask() = default;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV_CONTROLLER_NAVIGATION_TASK_HPP_

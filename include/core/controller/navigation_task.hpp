#ifndef ARCH_NAV__CORE__CONTROLLER__NAVIGATION_TASK_HPP_
#define ARCH_NAV__CORE__CONTROLLER__NAVIGATION_TASK_HPP_

#include <functional>
#include <memory>

#include "core/constants/command_response.hpp"
#include "core/context/vehicle_context.hpp"
#include "core/model/report/operation_report.hpp"
#include "dispatchers/i_command_dispatcher.hpp"

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

#endif  // ARCH_NAV__CORE__CONTROLLER__NAVIGATION_TASK_HPP_

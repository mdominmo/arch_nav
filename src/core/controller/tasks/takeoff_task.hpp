#ifndef ARCH_NAV__CORE__CONTROLLER__TAKEOFF_TASK_HPP_
#define ARCH_NAV__CORE__CONTROLLER__TAKEOFF_TASK_HPP_

#include "core/controller/navigation_task.hpp"
#include "core/constants/reference_frame.hpp"

namespace arch_nav::controller {

class TakeoffTask : public NavigationTask {
 public:
  TakeoffTask(double height, constants::ReferenceFrame frame);

  constants::CommandResponse start(
      context::VehicleContext& context,
      dispatchers::ICommandDispatcher& dispatcher,
      std::function<void()> on_complete) override;

  void abort() override;

  std::shared_ptr<report::OperationReport> make_report() override;

 private:
  double height_;
  constants::ReferenceFrame frame_;
  dispatchers::ICommandDispatcher* dispatcher_{nullptr};
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__TAKEOFF_TASK_HPP_

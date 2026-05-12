#ifndef ARCH_NAV__CORE__CONTROLLER__CHANGE_YAW_TASK_HPP_
#define ARCH_NAV__CORE__CONTROLLER__CHANGE_YAW_TASK_HPP_

#include "controller/navigation_task.hpp"

namespace arch_nav::controller {

class ChangeYawTask : public NavigationTask {
 public:
  ChangeYawTask(double new_yaw, constants::ReferenceFrame frame);

  constants::CommandResponse start(
    context::VehicleContext& context,
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) override;

  void abort() override;

  std::shared_ptr<report::OperationReport> make_report() override;

 private:
  double new_yaw_;
  constants::ReferenceFrame frame_;
  platform::ICommandDispatcher* dispatcher_{nullptr};
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__CHANGE_YAW_TASK_HPP_
#ifndef ARCH_NAV__CORE__CONTROLLER__TAKEOFF_TASK_HPP_
#define ARCH_NAV__CORE__CONTROLLER__TAKEOFF_TASK_HPP_

#include "controller/navigation_task.hpp"
#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/model/report/takeoff_report.hpp"

namespace arch_nav::controller {

class TakeoffTask : public NavigationTask {
 public:
  TakeoffTask(double height, constants::ReferenceFrame frame);

  constants::CommandResponse start(
      context::VehicleContext& context,
      platform::ICommandDispatcher& dispatcher,
      std::function<void()> on_complete) override;

  void abort() override;

  std::shared_ptr<report::OperationReport> make_report() override;

 private:
  double height_;
  constants::ReferenceFrame frame_;
  platform::ICommandDispatcher* dispatcher_{nullptr};
  std::shared_ptr<report::TakeoffReport> report_;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__TAKEOFF_TASK_HPP_

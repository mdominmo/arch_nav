#ifndef ARCH_NAV__CORE__CONTROLLER__TAKEOFF_TASK_HPP_
#define ARCH_NAV__CORE__CONTROLLER__TAKEOFF_TASK_HPP_

#include <functional>
#include <memory>

#include "controller/navigation_task.hpp"
#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/execution/takeoff_execution_state.hpp"
#include "arch_nav/model/report/takeoff_report.hpp"

namespace arch_nav::controller {

class TakeoffTask : public NavigationTask {
 public:
  TakeoffTask(double height, constants::ReferenceFrame frame);

  constants::CommandResponse start(
      platform::ICommandDispatcher& dispatcher,
      std::function<void()> on_complete) override;

  void abort() override;

  std::shared_ptr<report::OperationReport> make_report() override;
  std::unique_ptr<NavigationTaskMemento> make_memento() const override;

 private:
  double height_;
  constants::ReferenceFrame frame_;
  std::shared_ptr<execution::TakeoffExecutionState>  state_;
  platform::ICommandDispatcher* dispatcher_{nullptr};
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__TAKEOFF_TASK_HPP_

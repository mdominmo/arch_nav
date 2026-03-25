#ifndef ARCH_NAV__CORE__CONTROLLER__TRAJECTORY_EXECUTION_TASK_HPP_
#define ARCH_NAV__CORE__CONTROLLER__TRAJECTORY_EXECUTION_TASK_HPP_

#include <functional>
#include <memory>
#include <vector>

#include "controller/navigation_task.hpp"
#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/model/vehicle/trajectory_point.hpp"

namespace arch_nav::controller {

class TrajectoryExecutionTask : public NavigationTask {
 public:
  TrajectoryExecutionTask(std::vector<vehicle::TrajectoryPoint> trajectory,
                          constants::ReferenceFrame frame);

  constants::CommandResponse start(
      context::VehicleContext& context,
      dispatchers::ICommandDispatcher& dispatcher,
      std::function<void()> on_complete) override;

  void abort() override;

  std::shared_ptr<report::OperationReport> make_report() override;

 private:
  std::vector<vehicle::TrajectoryPoint>     trajectory_;
  constants::ReferenceFrame                 frame_;
  std::shared_ptr<report::OperationReport>  report_;
  dispatchers::ICommandDispatcher*          dispatcher_{nullptr};
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__TRAJECTORY_EXECUTION_TASK_HPP_

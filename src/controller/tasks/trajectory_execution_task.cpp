#include "trajectory_execution_task.hpp"

namespace arch_nav::controller {

TrajectoryExecutionTask::TrajectoryExecutionTask(
    std::vector<vehicle::TrajectoryPoint> trajectory,
    constants::ReferenceFrame frame)
    : trajectory_(std::move(trajectory)),
      frame_(frame),
      report_(std::make_shared<report::OperationReport>()) {}

constants::CommandResponse TrajectoryExecutionTask::start(
    context::VehicleContext&,
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_trajectory(
      std::move(trajectory_), frame_, std::move(on_complete));
}

void TrajectoryExecutionTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> TrajectoryExecutionTask::make_report() {
  return report_;
}

}  // namespace arch_nav::controller

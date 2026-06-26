#include "trajectory_execution_task.hpp"

namespace arch_nav::controller {

TrajectoryExecutionTask::TrajectoryExecutionTask(
    descriptor::TrajectoryOperationDescriptor& descriptor)
    : descriptor_(descriptor) {}

constants::CommandResponse TrajectoryExecutionTask::start(
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_trajectory(
      descriptor_.trajectory(), descriptor_.frame(),
      std::move(on_complete), descriptor_.progress());
}

void TrajectoryExecutionTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport>
TrajectoryExecutionTask::make_report() const {
  return descriptor_.make_report();
}

}  // namespace arch_nav::controller

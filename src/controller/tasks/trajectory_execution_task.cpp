#include "trajectory_execution_task.hpp"

namespace arch_nav::controller {

TrajectoryExecutionTask::TrajectoryExecutionTask(
    std::vector<vehicle::TrajectoryPoint> trajectory,
    constants::ReferenceFrame frame)
    : trajectory_(std::move(trajectory)),
      frame_(frame),
      state_(std::make_shared<execution::TrajectoryExecutionState>()) {}

constants::CommandResponse TrajectoryExecutionTask::start(
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_trajectory(
      trajectory_, frame_, std::move(on_complete), *state_);
}

void TrajectoryExecutionTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> TrajectoryExecutionTask::make_report() {
  state_->total_points.store(static_cast<int>(trajectory_.size()));
  return std::make_shared<report::TrajectoryReport>(state_);
}

std::unique_ptr<NavigationTaskMemento> TrajectoryExecutionTask::make_memento() const {
  int current = state_->current_point_index.load();
  if (current < 0 || static_cast<std::size_t>(current) >= trajectory_.size()) {
    return nullptr;
  }

  std::vector<vehicle::TrajectoryPoint> remaining(
      trajectory_.begin() + current, trajectory_.end());

  class TrajectoryExecutionTaskMemento : public NavigationTaskMemento {
    std::vector<vehicle::TrajectoryPoint> trajectory_;
    constants::ReferenceFrame frame_;
   public:
    TrajectoryExecutionTaskMemento(std::vector<vehicle::TrajectoryPoint> traj,
                                   constants::ReferenceFrame frame)
        : trajectory_(std::move(traj)), frame_(frame) {}
    std::unique_ptr<NavigationTask> reconstruct() const override {
      return std::make_unique<TrajectoryExecutionTask>(trajectory_, frame_);
    }
  };

  return std::make_unique<TrajectoryExecutionTaskMemento>(
      std::move(remaining), frame_);
}

}  // namespace arch_nav::controller

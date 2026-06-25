#include "waypoint_task.hpp"

namespace arch_nav::controller {

WaypointTask::WaypointTask(
    std::vector<vehicle::Waypoint> waypoints,
    constants::ReferenceFrame frame)
    : waypoints_(std::move(waypoints)),
      frame_(frame),
      state_(std::make_shared<execution::WaypointExecutionState>()) {}

constants::CommandResponse WaypointTask::start(
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_waypoint_following(
      waypoints_, frame_, std::move(on_complete), *state_);
}

void WaypointTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> WaypointTask::make_report() {
  state_->total_waypoints.store(static_cast<int>(waypoints_.size()));
  return std::make_shared<report::WaypointReport>(state_);
}

std::unique_ptr<NavigationTaskMemento> WaypointTask::make_memento() const {
  int current = state_->current_waypoint.load();
  if (current < 0 || static_cast<std::size_t>(current) >= waypoints_.size()) {
    return nullptr;
  }

  std::vector<vehicle::Waypoint> remaining(
      waypoints_.begin() + current, waypoints_.end());

  class WaypointTaskMemento : public NavigationTaskMemento {
    std::vector<vehicle::Waypoint> waypoints_;
    constants::ReferenceFrame frame_;
   public:
    WaypointTaskMemento(std::vector<vehicle::Waypoint> wps,
                        constants::ReferenceFrame frame)
        : waypoints_(std::move(wps)), frame_(frame) {}
    std::unique_ptr<NavigationTask> reconstruct() const override {
      return std::make_unique<WaypointTask>(waypoints_, frame_);
    }
  };

  return std::make_unique<WaypointTaskMemento>(std::move(remaining), frame_);
}

}  // namespace arch_nav::controller

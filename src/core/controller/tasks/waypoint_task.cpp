#include "waypoint_task.hpp"

namespace arch_nav::controller {

WaypointTask::WaypointTask(
    std::vector<vehicle::Waypoint> waypoints,
    constants::ReferenceFrame frame)
    : waypoints_(std::move(waypoints)),
      frame_(frame),
      report_(std::make_shared<report::WaypointReport>(waypoints_.size())) {}

constants::CommandResponse WaypointTask::start(
    context::VehicleContext&,
    dispatchers::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_waypoint_following(
      std::move(waypoints_), frame_, std::move(on_complete));
}

void WaypointTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> WaypointTask::make_report() {
  return report_;
}

}  // namespace arch_nav::controller

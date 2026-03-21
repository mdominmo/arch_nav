#include "waypoint_task.hpp"

namespace arch_nav::controller {

WaypointTask::WaypointTask(std::vector<geographic_msgs::msg::GeoPose> waypoints)
    : waypoints_(std::move(waypoints)),
      report_(std::make_shared<report::WaypointReport>(waypoints_.size())) {}

void WaypointTask::start(
    context::VehicleContext&,
    planner::ILocalPlanner&,
    dispatchers::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  dispatcher.execute_waypoint_following(std::move(waypoints_), std::move(on_complete));
}

void WaypointTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> WaypointTask::make_report() {
  return report_;
}

}  // namespace arch_nav::controller

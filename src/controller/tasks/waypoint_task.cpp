#include "waypoint_task.hpp"

namespace arch_nav::controller {

WaypointTask::WaypointTask(
    std::vector<vehicle::Waypoint> waypoints,
    constants::ReferenceFrame frame)
    : waypoints_(std::move(waypoints)), frame_(frame) {}

constants::CommandResponse WaypointTask::start(
    context::VehicleContext&,
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_waypoint_following(
      std::move(waypoints_), frame_, std::move(on_complete),
      report_->driver_data());
}

void WaypointTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> WaypointTask::make_report() {
  report_ = std::make_shared<report::WaypointReport>();
  report_->driver_data().total_waypoints.store(
      static_cast<int>(waypoints_.size()));
  return report_;
}

}  // namespace arch_nav::controller

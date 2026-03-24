#include "arch_nav_api.hpp"

namespace arch_nav {

ArchNavApi::ArchNavApi(controller::OperationalController& controller)
    : controller_(controller) {}

constants::CommandResponse ArchNavApi::takeoff(
    double height, constants::ReferenceFrame frame) {
  return controller_.takeoff(height, frame);
}

constants::CommandResponse ArchNavApi::land() {
  return controller_.land();
}

constants::CommandResponse ArchNavApi::waypoint_following(
    std::vector<vehicle::Waypoint> waypoints,
    constants::ReferenceFrame frame) {
  return controller_.waypoint_following(std::move(waypoints), frame);
}

constants::CommandResponse ArchNavApi::trajectory_execution(
    std::vector<vehicle::TrajectoryPoint> trajectory,
    constants::ReferenceFrame frame) {
  return controller_.trajectory_execution(std::move(trajectory), frame);
}

void ArchNavApi::cancel_operation() {
  controller_.stop();
}

constants::CommandResponse ArchNavApi::arm() {
  return controller_.arm();
}

constants::CommandResponse ArchNavApi::disarm() {
  return controller_.disarm();
}

constants::OperationStatus ArchNavApi::operation_status() const {
  return controller_.operation_status();
}

const report::OperationReport* ArchNavApi::last_operation_report() const {
  return controller_.last_operation_report();
}

void ArchNavApi::on_operation_complete(
    std::function<void(const report::OperationReport&)> callback) {
  on_complete_callback_ = std::move(callback);
}

void ArchNavApi::on_operation_progress(
    std::function<void(const report::OperationReport&)> callback) {
  on_progress_callback_ = std::move(callback);
}

}  // namespace arch_nav

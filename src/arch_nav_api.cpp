#include "arch_nav_api.hpp"

namespace arch_nav {

ArchNavApi::ArchNavApi(controller::OperationalController& controller)
    : controller_(controller) {}

void ArchNavApi::takeoff(double height) {
  controller_.takeoff(height);
}

void ArchNavApi::land() {
  controller_.land();
}

void ArchNavApi::waypoint_following(
    std::vector<vehicle::GeoWaypoint> waypoints) {
  controller_.waypoint_following(std::move(waypoints));
}

void ArchNavApi::cancel_operation() {
  controller_.stop();
}

constants::OperationStatus ArchNavApi::operation_status() const {
  return controller_.operation_status();
}

const report::OperationReport* ArchNavApi::last_operation_report() const {
  return controller_.last_operation_report();
}

void ArchNavApi::arm() {
  controller_.arm();
}

void ArchNavApi::disarm() {
  controller_.disarm();
}

}  // namespace arch_nav

#include "arch_nav/arch_nav_api.hpp"

#include "controller/operational_controller.hpp"
#include "arch_nav/context/vehicle_context.hpp"

namespace arch_nav {

struct ArchNavApi::Impl {
  controller::OperationalController& controller;
  context::VehicleContext& vehicle_context;
  std::function<void(const report::OperationReport&)> on_complete_callback;
  std::function<void(const report::OperationReport&)> on_progress_callback;

  Impl(controller::OperationalController& ctrl, context::VehicleContext& ctx)
      : controller(ctrl), vehicle_context(ctx) {}
};

ArchNavApi::ArchNavApi(
    controller::OperationalController& controller,
    context::VehicleContext& vehicle_context)
    : impl_(std::make_unique<Impl>(controller, vehicle_context)) {}

ArchNavApi::~ArchNavApi() = default;

constants::CommandResponse ArchNavApi::takeoff(
    double height, constants::ReferenceFrame frame) {
  return impl_->controller.takeoff(height, frame);
}

constants::CommandResponse ArchNavApi::land() {
  return impl_->controller.land();
}

constants::CommandResponse ArchNavApi::change_yaw(
    double new_yaw, constants::ReferenceFrame frame) {
  return impl_->controller.change_yaw(new_yaw, frame);
}

constants::CommandResponse ArchNavApi::waypoint_following(
    std::vector<vehicle::Waypoint> waypoints,
    constants::ReferenceFrame frame) {
  return impl_->controller.waypoint_following(std::move(waypoints), frame);
}

constants::CommandResponse ArchNavApi::trajectory_execution(
    std::vector<vehicle::TrajectoryPoint> trajectory,
    constants::ReferenceFrame frame) {
  return impl_->controller.trajectory_execution(std::move(trajectory), frame);
}

void ArchNavApi::cancel_operation() {
  impl_->controller.stop();
}

constants::CommandResponse ArchNavApi::arm() {
  return impl_->controller.arm();
}

constants::CommandResponse ArchNavApi::disarm() {
  return impl_->controller.disarm();
}

constants::OperationStatus ArchNavApi::operation_status() const {
  return impl_->controller.operation_status();
}

const report::OperationReport* ArchNavApi::last_operation_report() const {
  return impl_->controller.last_operation_report();
}

vehicle::GlobalPosition ArchNavApi::global_position() const {
  return impl_->vehicle_context.get_global_position();
}

vehicle::Kinematics ArchNavApi::kinematics() const {
  return impl_->vehicle_context.get_kinematic();
}

vehicle::VehicleStatus ArchNavApi::vehicle_status() const {
  return impl_->vehicle_context.get_vehicle_status();
}

void ArchNavApi::on_operation_complete(
    std::function<void(const report::OperationReport&)> callback) {
  impl_->on_complete_callback = callback;
  impl_->controller.set_on_complete_listener(std::move(callback));
}

void ArchNavApi::on_operation_progress(
    std::function<void(const report::OperationReport&)> callback) {
  impl_->on_progress_callback = callback;
  impl_->controller.set_on_progress_listener(std::move(callback));
}

}  // namespace arch_nav

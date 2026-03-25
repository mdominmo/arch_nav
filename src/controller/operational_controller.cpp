#include "controller/operational_controller.hpp"

#include <memory>

#include "commands/arm_command.hpp"
#include "commands/disarm_command.hpp"
#include "states/disarmed_state.hpp"
#include "states/handover_state.hpp"
#include "states/idle_state.hpp"
#include "states/running_state.hpp"
#include "tasks/land_task.hpp"
#include "tasks/takeoff_task.hpp"
#include "tasks/waypoint_task.hpp"
#include "tasks/trajectory_execution_task.hpp"

namespace arch_nav::controller {

OperationalController::OperationalController(
    context::VehicleContext& vehicle_context,
    dispatchers::ICommandDispatcher& dispatcher)
    : vehicle_context_(vehicle_context),
      dispatcher_(dispatcher),
      current_state_(nullptr),
      current_status_(constants::OperationStatus::HANDOVER) {
  change_state(
      std::make_unique<HandoverState>(),
      constants::OperationStatus::HANDOVER);
  vehicle_context_.subscribe_vehicle_status(
      [this](const vehicle::VehicleStatus& status) {
        on_vehicle_status_update(status);
      });
}

OperationalController::~OperationalController() = default;

constants::CommandResponse OperationalController::takeoff(
    double height, constants::ReferenceFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_->try_execute(
      *this, std::make_unique<TakeoffTask>(height, frame));
}

constants::CommandResponse OperationalController::land() {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_->try_execute(
      *this, std::make_unique<LandTask>());
}

constants::CommandResponse OperationalController::waypoint_following(
    std::vector<vehicle::Waypoint> waypoints,
    constants::ReferenceFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_->try_execute(
      *this, std::make_unique<WaypointTask>(std::move(waypoints), frame));
}

constants::CommandResponse OperationalController::trajectory_execution(
    std::vector<vehicle::TrajectoryPoint> trajectory,
    constants::ReferenceFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_->try_execute(
      *this, std::make_unique<TrajectoryExecutionTask>(std::move(trajectory), frame));
}

void OperationalController::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  current_state_->try_stop(*this);
}

constants::CommandResponse OperationalController::arm() {
  std::lock_guard<std::mutex> lock(mutex_);
  current_state_->try_command(*this, std::make_unique<ArmCommand>());
  return constants::CommandResponse::ACCEPTED;
}

constants::CommandResponse OperationalController::disarm() {
  std::lock_guard<std::mutex> lock(mutex_);
  current_state_->try_command(*this, std::make_unique<DisarmCommand>());
  return constants::CommandResponse::ACCEPTED;
}

constants::OperationStatus OperationalController::operation_status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_status_;
}

const report::OperationReport* OperationalController::last_operation_report() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_report_.get();
}

void OperationalController::on_vehicle_status_update(
    const vehicle::VehicleStatus& status) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_state_->on_vehicle_status_update(*this, status);
}

void OperationalController::on_operation_complete() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_report_) last_report_->complete();
  change_state(
      std::make_unique<IdleState>(),
      constants::OperationStatus::IDLE);
}

void OperationalController::change_state(
    std::unique_ptr<State> new_state,
    constants::OperationStatus status) {
  current_status_ = status;
  current_state_ = std::move(new_state);
  current_state_->on_enter(*this);
}

}  // namespace arch_nav::controller

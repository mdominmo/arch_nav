#include "running_state.hpp"

#include <memory>

#include "disarmed_state.hpp"
#include "handover_state.hpp"
#include "idle_state.hpp"

#include "core/constants/operation_status.hpp"
#include "core/constants/vehicle_status_states.hpp"

namespace arch_nav::controller {

OperationalController::RunningState::RunningState(
    std::unique_ptr<NavigationTask> task)
    : task_(std::move(task)) {}

void OperationalController::RunningState::try_stop(OperationalController& ctx) {
  task_->abort();
  ctx.last_report_->abort();
  ctx.change_state(
      std::make_unique<IdleState>(),
      constants::OperationStatus::IDLE);
}

void OperationalController::RunningState::on_vehicle_status_update(
    OperationalController& ctx, const vehicle::VehicleStatus& status) {
  if (status.is_valid() &&
      status.control_state == constants::ControlState::KERNEL_CONTROLLED &&
      status.arm_state == constants::ArmState::ARMED) {
    return;
  }

  task_->abort();
  ctx.last_report_->abort();

  if (!status.is_valid() ||
      status.control_state != constants::ControlState::KERNEL_CONTROLLED) {
    ctx.change_state(
        std::make_unique<HandoverState>(),
        constants::OperationStatus::HANDOVER);
    return;
  }

  ctx.change_state(
      std::make_unique<DisarmedState>(),
      constants::OperationStatus::DISARMED);
}

}  // namespace arch_nav::controller

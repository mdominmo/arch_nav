#include "idle_state.hpp"

#include <memory>

#include "disarmed_state.hpp"
#include "handover_state.hpp"
#include "running_state.hpp"

#include "arch_nav/constants/command_response.hpp"
#include "arch_nav/constants/operation_status.hpp"
#include "arch_nav/constants/vehicle_status_states.hpp"

namespace arch_nav::controller {

void OperationalController::IdleState::on_vehicle_status_update(
    OperationalController& ctx, const vehicle::VehicleStatus& status) {
  if (!status.is_valid() ||
      status.control_state != constants::ControlState::KERNEL_CONTROLLED) {
    ctx.change_state(
        std::make_unique<HandoverState>(),
        constants::OperationStatus::HANDOVER);
    return;
  }

  if (status.arm_state != constants::ArmState::ARMED) {
    ctx.change_state(
        std::make_unique<DisarmedState>(),
        constants::OperationStatus::DISARMED);
  }
}

constants::CommandResponse OperationalController::IdleState::try_execute(
    OperationalController& ctx, std::unique_ptr<NavigationTask> task) {
  auto report = task->make_report();
  auto response = task->start(
      ctx.vehicle_context_,
      ctx.dispatcher_,
      [&ctx]() { ctx.on_operation_complete(); });

  if (response != constants::CommandResponse::ACCEPTED) {
    report->fail();
    ctx.last_report_ = report;
    return response;
  }

  ctx.last_report_ = report;
  ctx.current_state_ = std::make_unique<RunningState>(std::move(task));
  ctx.current_status_ = constants::OperationStatus::RUNNING;
  return constants::CommandResponse::ACCEPTED;
}

void OperationalController::IdleState::try_command(
    OperationalController& ctx, std::unique_ptr<VehicleCommand> cmd) {
  cmd->execute(ctx.dispatcher_);
}

}  // namespace arch_nav::controller

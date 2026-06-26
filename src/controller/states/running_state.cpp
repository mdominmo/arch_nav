#include "running_state.hpp"

#include <memory>

#include "disarmed_state.hpp"
#include "handover_state.hpp"
#include "idle_state.hpp"

#include "arch_nav/constants/operation_status.hpp"
#include "arch_nav/constants/vehicle_status_states.hpp"

namespace arch_nav::controller {

OperationalController::RunningState::RunningState(
    std::unique_ptr<NavigationTask> task)
    : task_(std::move(task)) {}

void OperationalController::RunningState::try_stop(OperationalController& ctx) {
  ctx.stop_progress_thread();
  task_->abort();
  ctx.last_report_->abort();

  auto desc = ctx.operation_context_.current_descriptor();
  if (desc) desc->set_lifecycle_status(report::ReportStatus::ABORTED);
  ctx.operation_context_.clear_current_descriptor();

  auto report   = ctx.last_report_;
  auto listener = ctx.on_complete_listener_;

  ctx.change_state(
      std::make_unique<IdleState>(),
      constants::OperationStatus::IDLE);

  if (listener && report) listener(*report);
}

void OperationalController::RunningState::on_vehicle_status_update(
    OperationalController& ctx, const vehicle::VehicleStatus& status) {
  if (status.is_valid() &&
      status.control_state == constants::ControlState::KERNEL_CONTROLLED &&
      status.arm_state == constants::ArmState::ARMED) {
    return;
  }

  ctx.stop_progress_thread();
  task_->abort();
  ctx.last_report_->abort();

  auto desc = ctx.operation_context_.current_descriptor();
  if (desc) desc->set_lifecycle_status(report::ReportStatus::ABORTED);
  ctx.operation_context_.clear_current_descriptor();

  auto report   = ctx.last_report_;
  auto listener = ctx.on_complete_listener_;

  if (!status.is_valid() ||
      status.control_state != constants::ControlState::KERNEL_CONTROLLED) {
    ctx.change_state(
        std::make_unique<HandoverState>(),
        constants::OperationStatus::HANDOVER);
  } else {
    ctx.change_state(
        std::make_unique<DisarmedState>(),
        constants::OperationStatus::DISARMED);
  }

  if (listener && report) listener(*report);
}

OperationalController::State::PreemptionResult
OperationalController::RunningState::try_preempt(OperationalController& ctx) {
  ctx.stop_progress_thread();

  auto user_descriptor = ctx.operation_context_.current_descriptor();
  if (user_descriptor)
    user_descriptor->set_lifecycle_status(report::ReportStatus::SUPERVISED);

  task_->abort();

  return {std::move(user_descriptor), ctx.last_report_, true};
}

}  // namespace arch_nav::controller

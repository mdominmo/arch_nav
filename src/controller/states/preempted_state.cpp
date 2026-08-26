#include "preempted_state.hpp"

#include <memory>

#include "idle_state.hpp"
#include "running_state.hpp"
#include "disarmed_state.hpp"
#include "handover_state.hpp"

#include "arch_nav/constants/operation_status.hpp"
#include "arch_nav/constants/vehicle_status_states.hpp"
#include "arch_nav/controller/preemption_event.hpp"
#include "controller/navigation_task_factory.hpp"

namespace arch_nav::controller {

OperationalController::PreemptedState::PreemptedState(
    std::shared_ptr<descriptor::OperationDescriptor> user_descriptor,
    std::shared_ptr<report::OperationReport> user_report,
    PreemptionType preemption_type,
    PreemptionInfo preemption_info)
    : user_descriptor_(std::move(user_descriptor)),
      user_report_(std::move(user_report)),
      preemption_type_(preemption_type),
      preemption_info_(std::move(preemption_info)) {}

constants::CommandResponse
OperationalController::PreemptedState::try_execute(
    OperationalController& ctx,
    std::unique_ptr<NavigationTask> task) {
  supervisor_task_ = std::move(task);

  auto report = supervisor_task_->make_report();
  auto response = supervisor_task_->start(
      ctx.dispatcher_,
      [&ctx] { ctx.on_supervisor_task_complete(); });

  if (response != constants::CommandResponse::ACCEPTED) {
    supervisor_task_.reset();
    resolve_locked(ctx);
  }

  return response;
}

void OperationalController::PreemptedState::try_stop(
    OperationalController& ctx) {
  if (supervisor_task_) {
    supervisor_task_->abort();
  }
  resolve_locked(ctx);
}

OperationalController::State::PreemptionResult
OperationalController::PreemptedState::try_preempt(OperationalController& ctx) {
  if (supervisor_task_) {
    ctx.stop_progress_thread();
    supervisor_task_->abort();
    supervisor_task_.reset();
  }

  return {std::move(user_descriptor_), std::move(user_report_), true,
          PreemptionEvent{
              PreemptionEventType::RESOLVED,
              preemption_info_.name, preemption_info_.reason,
              preemption_type_, preemption_info_.details}};
}

void OperationalController::PreemptedState::on_supervisor_task_complete(
    OperationalController& ctx) {
  std::lock_guard<std::mutex> lock(ctx.mutex_);
  resolve_locked(ctx);
}

void OperationalController::PreemptedState::resolve_locked(
    OperationalController& ctx) {
  ctx.stop_progress_thread();
  supervisor_task_.reset();

  PreemptionEvent resolved_event{
      PreemptionEventType::RESOLVED,
      preemption_info_.name, preemption_info_.reason,
      preemption_type_, preemption_info_.details};

  if (preemption_type_ == PreemptionType::TRANSIENT && user_descriptor_) {
    user_descriptor_->set_lifecycle_status(report::ReportStatus::IN_PROGRESS);
    ctx.operation_context_.set_current_descriptor(user_descriptor_);

    auto restored_task =
        NavigationTaskFactory::create_from_descriptor(*user_descriptor_);
    if (restored_task) {
      ctx.last_report_ = restored_task->make_report();

      auto response = restored_task->start(
          ctx.dispatcher_,
          [&ctx] { ctx.on_operation_complete(); });

      if (response == constants::CommandResponse::ACCEPTED) {
        ctx.change_state(
            std::make_unique<RunningState>(std::move(restored_task)),
            constants::OperationStatus::RUNNING);
        ctx.start_progress_thread();

        if (ctx.on_preemption_event_listener_)
          ctx.on_preemption_event_listener_(resolved_event);
        return;
      }
    }
  }

  auto report = user_report_;
  auto listener = ctx.on_complete_listener_;

  if (preemption_type_ == PreemptionType::TERMINAL && report) {
    report->abort();
  }

  if (user_descriptor_)
    user_descriptor_->set_lifecycle_status(report::ReportStatus::ABORTED);
  ctx.operation_context_.clear_current_descriptor();

  ctx.change_state(
      std::make_unique<IdleState>(),
      constants::OperationStatus::IDLE);

  if (ctx.on_preemption_event_listener_)
    ctx.on_preemption_event_listener_(resolved_event);

  if (listener && report) listener(*report);
}

void OperationalController::PreemptedState::on_vehicle_status_update(
    OperationalController& ctx, const vehicle::VehicleStatus& status) {
  if (status.is_valid() &&
      status.control_state == constants::ControlState::KERNEL_CONTROLLED &&
      status.arm_state == constants::ArmState::ARMED) {
    return;
  }

  if (supervisor_task_) {
    ctx.stop_progress_thread();
    supervisor_task_->abort();
    supervisor_task_.reset();
  }

  auto report = user_report_;
  auto listener = ctx.on_complete_listener_;
  if (report) report->abort();

  if (user_descriptor_)
    user_descriptor_->set_lifecycle_status(report::ReportStatus::ABORTED);
  ctx.operation_context_.clear_current_descriptor();

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

  if (ctx.on_preemption_event_listener_) {
    ctx.on_preemption_event_listener_({
        PreemptionEventType::RESOLVED,
        preemption_info_.name, preemption_info_.reason,
        preemption_type_, preemption_info_.details});
  }

  if (listener && report) listener(*report);
}

}  // namespace arch_nav::controller

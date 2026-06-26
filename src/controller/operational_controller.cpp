#include "controller/operational_controller.hpp"

#include <chrono>
#include <memory>
#include <optional>

#include "commands/arm_command.hpp"
#include "commands/clear_roi_command.hpp"
#include "commands/disarm_command.hpp"
#include "commands/set_roi_command.hpp"
#include "states/disarmed_state.hpp"
#include "states/handover_state.hpp"
#include "states/idle_state.hpp"
#include "states/running_state.hpp"
#include "states/preempted_state.hpp"
#include "tasks/land_task.hpp"
#include "tasks/change_yaw_task.hpp"
#include "tasks/takeoff_task.hpp"
#include "tasks/waypoint_task.hpp"
#include "tasks/trajectory_execution_task.hpp"

#include "arch_nav/descriptor/waypoint_operation_descriptor.hpp"
#include "arch_nav/descriptor/takeoff_operation_descriptor.hpp"
#include "arch_nav/descriptor/trajectory_operation_descriptor.hpp"
#include "arch_nav/descriptor/land_operation_descriptor.hpp"
#include "arch_nav/descriptor/change_yaw_operation_descriptor.hpp"
#include "arch_nav/descriptor/follow_target_operation_descriptor.hpp"
#include "tasks/follow_target_task.hpp"

namespace arch_nav::controller {

OperationalController::OperationalController(
    context::OperationContext& operation_context,
    platform::ICommandDispatcher& dispatcher)
    : operation_context_(operation_context),
      dispatcher_(dispatcher),
      current_state_(nullptr),
      current_status_(constants::OperationStatus::HANDOVER) {
  change_state(
      std::make_unique<HandoverState>(),
      constants::OperationStatus::HANDOVER);
}

OperationalController::~OperationalController() {
  stop_progress_thread();
}

constants::CommandResponse OperationalController::takeoff(
    double height, constants::ReferenceFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto desc = std::make_shared<descriptor::TakeoffOperationDescriptor>(
      height, frame);
  operation_context_.set_current_descriptor(desc);
  auto response = current_state_->try_execute(
      *this, std::make_unique<TakeoffTask>(*desc));
  if (response != constants::CommandResponse::ACCEPTED)
    operation_context_.clear_current_descriptor();
  return response;
}

constants::CommandResponse OperationalController::land() {
  std::lock_guard<std::mutex> lock(mutex_);
  auto desc = std::make_shared<descriptor::LandOperationDescriptor>();
  operation_context_.set_current_descriptor(desc);
  auto response = current_state_->try_execute(
      *this, std::make_unique<LandTask>(*desc));
  if (response != constants::CommandResponse::ACCEPTED)
    operation_context_.clear_current_descriptor();
  return response;
}

constants::CommandResponse OperationalController::change_yaw(
  double new_yaw, constants::ReferenceFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto desc = std::make_shared<descriptor::ChangeYawOperationDescriptor>(
      new_yaw, frame);
  operation_context_.set_current_descriptor(desc);
  auto response = current_state_->try_execute(
      *this, std::make_unique<ChangeYawTask>(*desc));
  if (response != constants::CommandResponse::ACCEPTED)
    operation_context_.clear_current_descriptor();
  return response;
}

constants::CommandResponse OperationalController::waypoint_following(
    std::vector<vehicle::Waypoint> waypoints,
    constants::ReferenceFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto desc = std::make_shared<descriptor::WaypointOperationDescriptor>(
      std::move(waypoints), frame);
  operation_context_.set_current_descriptor(desc);
  auto response = current_state_->try_execute(
      *this, std::make_unique<WaypointTask>(*desc));
  if (response != constants::CommandResponse::ACCEPTED)
    operation_context_.clear_current_descriptor();
  return response;
}

constants::CommandResponse OperationalController::trajectory_execution(
    std::vector<vehicle::TrajectoryPoint> trajectory,
    constants::ReferenceFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto desc = std::make_shared<descriptor::TrajectoryOperationDescriptor>(
      std::move(trajectory), frame);
  operation_context_.set_current_descriptor(desc);
  auto response = current_state_->try_execute(
      *this, std::make_unique<TrajectoryExecutionTask>(*desc));
  if (response != constants::CommandResponse::ACCEPTED)
    operation_context_.clear_current_descriptor();
  return response;
}

constants::CommandResponse OperationalController::follow_target(
    constants::ReferenceFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto desc = std::make_shared<descriptor::FollowTargetOperationDescriptor>(
      frame);
  operation_context_.set_current_descriptor(desc);
  auto response = current_state_->try_execute(
      *this, std::make_unique<FollowTargetTask>(*desc));
  if (response != constants::CommandResponse::ACCEPTED)
    operation_context_.clear_current_descriptor();
  return response;
}

void OperationalController::update_follow_target_position(
    double x, double y, double z) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto desc = operation_context_.current_descriptor();
  if (desc && desc->operation_type() ==
      constants::OperationType::FOLLOW_TARGET) {
    auto& ft = static_cast<descriptor::FollowTargetOperationDescriptor&>(*desc);
    ft.update_target_position(x, y, z);
  }
}

void OperationalController::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  current_state_->try_stop(*this);
}

constants::CommandResponse OperationalController::arm() {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_->try_command(*this, std::make_unique<ArmCommand>());
}

constants::CommandResponse OperationalController::disarm() {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_->try_command(*this, std::make_unique<DisarmCommand>());
}

constants::CommandResponse OperationalController::set_roi(
    vehicle::GlobalPosition position, constants::ReferenceFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_->try_command(
      *this, std::make_unique<SetRoiCommand>(std::move(position), frame));
}

constants::CommandResponse OperationalController::clear_roi() {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_->try_command(
      *this, std::make_unique<ClearRoiCommand>());
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

void OperationalController::set_on_complete_listener(
    std::function<void(const report::OperationReport&)> cb) {
  std::lock_guard<std::mutex> lock(mutex_);
  on_complete_listener_ = std::move(cb);
}

void OperationalController::set_on_progress_listener(
    std::function<void(const report::OperationReport&)> cb) {
  std::lock_guard<std::mutex> lock(mutex_);
  on_progress_listener_ = std::move(cb);
}

void OperationalController::set_on_preemption_event_listener(
    std::function<void(const PreemptionEvent&)> cb) {
  std::lock_guard<std::mutex> lock(mutex_);
  on_preemption_event_listener_ = std::move(cb);
}

void OperationalController::start_progress_thread() {
  auto listener = on_progress_listener_;
  auto report   = last_report_;

  progress_stop_ = false;
  progress_thread_ = std::thread([this, listener, report] {
    while (!progress_stop_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (progress_stop_) break;
      if (listener && report) listener(*report);
    }
  });
}

void OperationalController::stop_progress_thread() {
  progress_stop_ = true;
  if (progress_thread_.joinable()) progress_thread_.join();
}

void OperationalController::on_operation_complete() {
  stop_progress_thread();

  std::shared_ptr<report::OperationReport> report;
  std::function<void(const report::OperationReport&)> listener;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_report_) last_report_->complete();
    auto desc = operation_context_.current_descriptor();
    if (desc) desc->set_lifecycle_status(report::ReportStatus::COMPLETED);
    operation_context_.clear_current_descriptor();
    report   = last_report_;
    listener = on_complete_listener_;
    change_state(
        std::make_unique<IdleState>(),
        constants::OperationStatus::IDLE);
  }

  if (listener && report) listener(*report);
}

void OperationalController::preempt(PreemptionType type,
                                     const PreemptionInfo& info) {
  std::function<void(const PreemptionEvent&)> listener;
  std::optional<PreemptionEvent> displaced_event;
  PreemptionEvent activated_event;
  bool accepted = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    auto result = current_state_->try_preempt(*this);
    if (result.accepted) {
      displaced_event = std::move(result.displaced_preemption_event);

      change_state(
          std::make_unique<PreemptedState>(
              std::move(result.user_descriptor),
              std::move(result.user_report),
              type, info),
          constants::OperationStatus::SUPERVISED);
      listener = on_preemption_event_listener_;
      activated_event = {
          PreemptionEventType::ACTIVATED,
          info.name, info.reason, type, info.details};
      accepted = true;
    }
  }
  if (accepted && listener) {
    if (displaced_event) listener(*displaced_event);
    listener(activated_event);
  }
}

void OperationalController::on_supervisor_task_complete() {
  auto* preempted = dynamic_cast<PreemptedState*>(current_state_.get());
  if (preempted) {
    preempted->on_supervisor_task_complete(*this);
  }
}

void OperationalController::change_state(
    std::unique_ptr<State> new_state,
    constants::OperationStatus status) {
  current_status_ = status;
  current_state_ = std::move(new_state);
  current_state_->on_enter(*this);
}

}  // namespace arch_nav::controller

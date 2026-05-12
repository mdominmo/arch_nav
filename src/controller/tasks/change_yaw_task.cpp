#include "change_yaw_task.hpp"

namespace arch_nav::controller {

ChangeYawTask::ChangeYawTask(double new_yaw, constants::ReferenceFrame frame)
    : new_yaw_(new_yaw), frame_(frame) {}

constants::CommandResponse ChangeYawTask::start(
    context::VehicleContext&,
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_change_yaw(
    new_yaw_, frame_, std::move(on_complete));
}

void ChangeYawTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> ChangeYawTask::make_report() {
  return std::make_shared<report::OperationReport>();
}

}  // namespace arch_nav::controller

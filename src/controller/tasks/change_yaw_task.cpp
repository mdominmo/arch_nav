#include "change_yaw_task.hpp"

namespace arch_nav::controller {

ChangeYawTask::ChangeYawTask(
    descriptor::ChangeYawOperationDescriptor& descriptor)
    : descriptor_(descriptor) {}

constants::CommandResponse ChangeYawTask::start(
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_change_yaw(
      descriptor_.target_yaw(), descriptor_.frame(),
      std::move(on_complete));
}

void ChangeYawTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> ChangeYawTask::make_report() const {
  return descriptor_.make_report();
}

}  // namespace arch_nav::controller

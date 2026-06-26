#include "follow_target_task.hpp"

namespace arch_nav::controller {

FollowTargetTask::FollowTargetTask(
    descriptor::FollowTargetOperationDescriptor& descriptor)
    : descriptor_(descriptor) {}

constants::CommandResponse FollowTargetTask::start(
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_follow_target(
      descriptor_.frame(), std::move(on_complete), descriptor_);
}

void FollowTargetTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport>
FollowTargetTask::make_report() const {
  return descriptor_.make_report();
}

}  // namespace arch_nav::controller

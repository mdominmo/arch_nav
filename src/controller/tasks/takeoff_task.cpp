#include "takeoff_task.hpp"

namespace arch_nav::controller {

TakeoffTask::TakeoffTask(
    descriptor::TakeoffOperationDescriptor& descriptor)
    : descriptor_(descriptor) {}

constants::CommandResponse TakeoffTask::start(
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_takeoff(
      descriptor_.height(), descriptor_.frame(),
      std::move(on_complete), descriptor_.progress());
}

void TakeoffTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> TakeoffTask::make_report() const {
  return descriptor_.make_report();
}

}  // namespace arch_nav::controller

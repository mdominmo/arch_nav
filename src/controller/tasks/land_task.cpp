#include "land_task.hpp"

namespace arch_nav::controller {

LandTask::LandTask(descriptor::LandOperationDescriptor& descriptor)
    : descriptor_(descriptor) {}

constants::CommandResponse LandTask::start(
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_land(std::move(on_complete));
}

void LandTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> LandTask::make_report() const {
  return descriptor_.make_report();
}

}  // namespace arch_nav::controller

#include "takeoff_task.hpp"

namespace arch_nav::controller {

TakeoffTask::TakeoffTask(double height, constants::ReferenceFrame frame)
    : height_(height), frame_(frame) {}

constants::CommandResponse TakeoffTask::start(
    context::VehicleContext&,
    dispatchers::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_takeoff(height_, frame_, std::move(on_complete));
}

void TakeoffTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> TakeoffTask::make_report() {
  return std::make_shared<report::OperationReport>();
}

}  // namespace arch_nav::controller

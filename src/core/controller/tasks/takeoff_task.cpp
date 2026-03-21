#include "takeoff_task.hpp"

namespace arch_nav::controller {

TakeoffTask::TakeoffTask(double height) : height_(height) {}

void TakeoffTask::start(
    context::VehicleContext&,
    planner::ILocalPlanner&,
    dispatchers::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  dispatcher.execute_takeoff(height_, std::move(on_complete));
}

void TakeoffTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> TakeoffTask::make_report() {
  return std::make_shared<report::OperationReport>();
}

}  // namespace arch_nav::controller

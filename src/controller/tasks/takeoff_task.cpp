#include "takeoff_task.hpp"

namespace arch_nav::controller {

TakeoffTask::TakeoffTask(double height, constants::ReferenceFrame frame)
    : height_(height), frame_(frame) {}

constants::CommandResponse TakeoffTask::start(
    context::VehicleContext&,
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_takeoff(
      height_, frame_, std::move(on_complete), report_->driver_data());
}

void TakeoffTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> TakeoffTask::make_report() {
  report_ = std::make_shared<report::TakeoffReport>();
  report_->driver_data().target_altitude.store(height_);
  return report_;
}

}  // namespace arch_nav::controller

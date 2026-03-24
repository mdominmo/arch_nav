#include "land_task.hpp"

namespace arch_nav::controller {

constants::CommandResponse LandTask::start(
    context::VehicleContext&,
    dispatchers::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_land(std::move(on_complete));
}

void LandTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> LandTask::make_report() {
  return std::make_shared<report::OperationReport>();
}

}  // namespace arch_nav::controller

#include "land_task.hpp"

namespace arch_nav::controller {

constants::CommandResponse LandTask::start(
    platform::ICommandDispatcher& dispatcher,
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

std::unique_ptr<NavigationTaskMemento> LandTask::make_memento() const {
  class LandTaskMemento : public NavigationTaskMemento {
   public:
    std::unique_ptr<NavigationTask> reconstruct() const override {
      return std::make_unique<LandTask>();
    }
  };
  return std::make_unique<LandTaskMemento>();
}

}  // namespace arch_nav::controller

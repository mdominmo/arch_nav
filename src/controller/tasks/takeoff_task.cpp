#include "takeoff_task.hpp"

namespace arch_nav::controller {

TakeoffTask::TakeoffTask(double height, constants::ReferenceFrame frame)
    : height_(height),
      frame_(frame),
      state_(std::make_shared<execution::TakeoffExecutionState>()) {}

constants::CommandResponse TakeoffTask::start(
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_takeoff(
      height_, frame_, std::move(on_complete), *state_);
}

void TakeoffTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> TakeoffTask::make_report() {
  state_->target_altitude.store(height_);
  return std::make_shared<report::TakeoffReport>(state_);
}

std::unique_ptr<NavigationTaskMemento> TakeoffTask::make_memento() const {
  class TakeoffTaskMemento : public NavigationTaskMemento {
    double height_;
    constants::ReferenceFrame frame_;
   public:
    TakeoffTaskMemento(double h, constants::ReferenceFrame f)
        : height_(h), frame_(f) {}
    std::unique_ptr<NavigationTask> reconstruct() const override {
      return std::make_unique<TakeoffTask>(height_, frame_);
    }
  };
  return std::make_unique<TakeoffTaskMemento>(height_, frame_);
}

}  // namespace arch_nav::controller

#include "change_yaw_task.hpp"

namespace arch_nav::controller {

ChangeYawTask::ChangeYawTask(double new_yaw, constants::ReferenceFrame frame)
    : new_yaw_(new_yaw), frame_(frame) {}

constants::CommandResponse ChangeYawTask::start(
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_change_yaw(
    new_yaw_, frame_, std::move(on_complete));
}

void ChangeYawTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> ChangeYawTask::make_report() {
  return std::make_shared<report::OperationReport>();
}

std::unique_ptr<NavigationTaskMemento> ChangeYawTask::make_memento() const {
  class ChangeYawTaskMemento : public NavigationTaskMemento {
    double yaw_;
    constants::ReferenceFrame frame_;
   public:
    ChangeYawTaskMemento(double y, constants::ReferenceFrame f)
        : yaw_(y), frame_(f) {}
    std::unique_ptr<NavigationTask> reconstruct() const override {
      return std::make_unique<ChangeYawTask>(yaw_, frame_);
    }
  };
  return std::make_unique<ChangeYawTaskMemento>(new_yaw_, frame_);
}

}  // namespace arch_nav::controller

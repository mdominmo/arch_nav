#ifndef ARCH_NAV__CORE__CONTROLLER__FOLLOW_TARGET_TASK_HPP_
#define ARCH_NAV__CORE__CONTROLLER__FOLLOW_TARGET_TASK_HPP_

#include "controller/navigation_task.hpp"
#include "arch_nav/descriptor/follow_target_operation_descriptor.hpp"

namespace arch_nav::controller {

class FollowTargetTask : public NavigationTask {
 public:
  explicit FollowTargetTask(
      descriptor::FollowTargetOperationDescriptor& descriptor);

  constants::CommandResponse start(
      platform::ICommandDispatcher& dispatcher,
      std::function<void()> on_complete) override;

  void abort() override;
  std::shared_ptr<report::OperationReport> make_report() const override;

 private:
  descriptor::FollowTargetOperationDescriptor& descriptor_;
  platform::ICommandDispatcher* dispatcher_{nullptr};
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__FOLLOW_TARGET_TASK_HPP_

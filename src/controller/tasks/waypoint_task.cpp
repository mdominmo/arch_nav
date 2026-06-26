#include "waypoint_task.hpp"

namespace arch_nav::controller {

WaypointTask::WaypointTask(
    descriptor::WaypointOperationDescriptor& descriptor)
    : descriptor_(descriptor) {}

constants::CommandResponse WaypointTask::start(
    platform::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;
  return dispatcher.execute_waypoint_following(
      descriptor_.waypoints(), descriptor_.frame(),
      std::move(on_complete), descriptor_.progress());
}

void WaypointTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> WaypointTask::make_report() const {
  return descriptor_.make_report();
}

}  // namespace arch_nav::controller

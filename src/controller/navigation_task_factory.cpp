#include "navigation_task_factory.hpp"

#include "arch_nav/descriptor/waypoint_operation_descriptor.hpp"
#include "arch_nav/descriptor/takeoff_operation_descriptor.hpp"
#include "arch_nav/descriptor/trajectory_operation_descriptor.hpp"
#include "arch_nav/descriptor/land_operation_descriptor.hpp"
#include "arch_nav/descriptor/change_yaw_operation_descriptor.hpp"
#include "arch_nav/descriptor/follow_target_operation_descriptor.hpp"
#include "tasks/waypoint_task.hpp"
#include "tasks/takeoff_task.hpp"
#include "tasks/trajectory_execution_task.hpp"
#include "tasks/land_task.hpp"
#include "tasks/change_yaw_task.hpp"
#include "tasks/follow_target_task.hpp"

namespace arch_nav::controller {

std::unique_ptr<NavigationTask>
NavigationTaskFactory::create_from_descriptor(
    descriptor::OperationDescriptor& desc) {
  switch (desc.operation_type()) {
    case constants::OperationType::WAYPOINT_FOLLOWING:
      return std::make_unique<WaypointTask>(
          static_cast<descriptor::WaypointOperationDescriptor&>(desc));
    case constants::OperationType::TAKEOFF:
      return std::make_unique<TakeoffTask>(
          static_cast<descriptor::TakeoffOperationDescriptor&>(desc));
    case constants::OperationType::TRAJECTORY_EXECUTION:
      return std::make_unique<TrajectoryExecutionTask>(
          static_cast<descriptor::TrajectoryOperationDescriptor&>(desc));
    case constants::OperationType::LAND:
      return std::make_unique<LandTask>(
          static_cast<descriptor::LandOperationDescriptor&>(desc));
    case constants::OperationType::CHANGE_YAW:
      return std::make_unique<ChangeYawTask>(
          static_cast<descriptor::ChangeYawOperationDescriptor&>(desc));
    case constants::OperationType::FOLLOW_TARGET:
      return std::make_unique<FollowTargetTask>(
          static_cast<descriptor::FollowTargetOperationDescriptor&>(desc));
    default:
      return nullptr;
  }
}

}  // namespace arch_nav::controller
